/*
 * Copyright (c) 2026 Daniel Kampert <DanielKampert@kampis-elektroecke.de>
 *
 * Texas Instruments ADS7028, 8 channel 12 bit SAR ADC with SPI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_ads7028

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(adc_ads7028, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/** @name SPI opcodes
 * @{
 */
#define ADS7028_OP_NOP     0x00 /**< No operation, only clocks out the conversion result */
#define ADS7028_OP_RD_REG  0x10 /**< Read register, data is returned in the next frame */
#define ADS7028_OP_WR_REG  0x08 /**< Write register */
#define ADS7028_OP_SET_BIT 0x18 /**< Set the bits of a register given by the mask */
#define ADS7028_OP_CLR_BIT 0x20 /**< Clear the bits of a register given by the mask */
/** @} */

/** @name Register addresses
 * @{
 */
#define ADS7028_REG_SYSTEM_STATUS   0x00 /**< System status flags */
#define ADS7028_REG_GENERAL_CFG     0x01 /**< General configuration (reset, calibration, CRC) */
#define ADS7028_REG_DATA_CFG        0x02 /**< Data format, status append */
#define ADS7028_REG_OSR_CFG         0x03 /**< Oversampling ratio */
#define ADS7028_REG_SEQUENCE_CFG    0x10 /**< Sequencer mode and start */
#define ADS7028_REG_CHANNEL_SEL     0x11 /**< Channel selection in manual mode */
#define ADS7028_REG_AUTO_SEQ_CH_SEL 0x12 /**< Channel mask for the auto sequencer */
/** @} */

/** @name SYSTEM_STATUS bits
 * @{
 */
#define ADS7028_SYSTEM_STATUS_RSVD       BIT(7) /**< Reserved, reads 1 on a responding device */
#define ADS7028_SYSTEM_STATUS_CRC_ERR_IN BIT(1) /**< CRC error on SPI input, write 1 to clear */
#define ADS7028_SYSTEM_STATUS_BOR        BIT(0) /**< Brown out / reset flag, write 1 to clear */
/** @} */

/** @name GENERAL_CFG bits
 * @{
 */
#define ADS7028_GENERAL_CFG_CRC_EN BIT(6) /**< Enable the CRC on the SPI interface */
#define ADS7028_GENERAL_CFG_CAL    BIT(1) /**< Start offset calibration, cleared when done */
#define ADS7028_GENERAL_CFG_RST    BIT(0) /**< Software reset */
/** @} */

/** Bit position of the APPEND_STATUS field in DATA_CFG */
#define ADS7028_DATA_CFG_APPEND_STATUS_POS 4

/** @name SEQUENCE_CFG bits
 * @{
 */
#define ADS7028_SEQUENCE_CFG_SEQ_START     BIT(4) /**< Start the sequence */
#define ADS7028_SEQUENCE_CFG_SEQ_MODE_AUTO 0x01   /**< SEQ_MODE = auto sequence */
/** @} */

/**
 * @name Values of DATA_CFG.APPEND_STATUS
 * Same order as the ti,append-status enum.
 * @{
 */
#define ADS7028_APPEND_NONE         0 /**< Nothing appended */
#define ADS7028_APPEND_CHANNEL_ID   1 /**< Channel id appended */
#define ADS7028_APPEND_STATUS_FLAGS 2 /**< Status flags appended */
/** @} */

/**
 * @name Appended status flags
 * Layout is {1, 0, CRCERR_IN, ALERT}.
 * @{
 */
#define ADS7028_STATUS_FLAGS_FIXED_MASK GENMASK(3, 2) /**< Bits with a fixed value */
#define ADS7028_STATUS_FLAGS_FIXED      BIT(3)        /**< Expected value of the fixed bits */
#define ADS7028_STATUS_FLAGS_CRC_ERR_IN BIT(1)        /**< CRC error on the SPI input */
/** @} */

#define ADS7028_NUM_CHANNELS     8                              /**< Number of analog inputs */
#define ADS7028_CHANNEL_MASK     BIT_MASK(ADS7028_NUM_CHANNELS) /**< Mask of all channels */
#define ADS7028_RESOLUTION       12                             /**< Native resolution in bits */
#define ADS7028_RESOLUTION_AVG   16 /**< Resolution in bits with oversampling */
#define ADS7028_MAX_OVERSAMPLING 7  /**< Highest OSR_CFG value */

#define ADS7028_FRAME_LEN     3 /**< Frame length in bytes without CRC */
#define ADS7028_FRAME_LEN_CRC 4 /**< Frame length in bytes with CRC */

/** CRC-8-CCITT (x^8 + x^2 + x + 1) seed, preset with all ones like the ADS7066 */
#define ADS7028_CRC_SEED 0xFF

/** Conversion time in microseconds, tCONV is 600 ns max */
#define ADS7028_T_CONV_US 1

/** Cycle time in microseconds of the internally started conversions while averaging,
 *  high speed oscillator
 */
#define ADS7028_T_CYCLE_US 1

/** Delays from this value on (in microseconds) sleep instead of busy waiting. A sleep is
 *  rounded up to whole ticks, so short sleeps would take much longer than requested.
 */
#define ADS7028_SLEEP_THRESHOLD_US MAX(100U, 2U * k_ticks_to_us_ceil32(1))
/** Marks a cached register value as unknown, the next acquisition rewrites the register */
#define ADS7028_CACHE_INVALID      0xFF
/** Time in milliseconds the device needs after a reset */
#define ADS7028_RESET_TIME_MS      1
/** Poll interval in microseconds while waiting for the calibration */
#define ADS7028_CAL_POLL_US        100
/** Maximum calibration time in microseconds */
#define ADS7028_CAL_TIMEOUT_US     10000

/** Any instance uses CRC, otherwise the CRC code is left out */
#define ADS7028_ANY_CRC      DT_ANY_INST_HAS_BOOL_STATUS_OKAY(ti_crc)
/** Any instance uses the auto sequencer, otherwise the auto mode code is left out */
#define ADS7028_ANY_AUTO_SEQ DT_ANY_INST_HAS_BOOL_STATUS_OKAY(ti_auto_sequence)

#define ADS7028_CRC_ENABLED(cfg) (ADS7028_ANY_CRC && (cfg)->crc)
/** CRC is currently active on the SPI interface */
#define ADS7028_CRC_ACTIVE(data) (ADS7028_ANY_CRC && (data)->crc_active)
/** The auto sequencer is enabled in the devicetree configuration @p cfg */
#define ADS7028_AUTO_SEQ(cfg)    (ADS7028_ANY_AUTO_SEQ && (cfg)->auto_sequence)

/** @brief Constant (devicetree) configuration of an ADS7028 instance. */
struct ads7028_config {
	/** SPI bus the device is connected to. */
	struct spi_dt_spec bus;
	/** Value of DATA_CFG.APPEND_STATUS (one of ADS7028_APPEND_*). */
	uint8_t append_status;
	/** Use the internal channel sequencer instead of manual channel selection. */
	bool auto_sequence;
	/** Enable the CRC on the SPI interface. */
	bool crc;
	/** Run an offset calibration during initialization. */
	bool calibrate;
};

/** @brief Runtime data of an ADS7028 instance. */
struct ads7028_data {
	/** Generic ADC context (locking, sequence and timer handling). */
	struct adc_context ctx;
	/** Back reference to the device, used by the acquisition work. */
	const struct device *dev;
	/** Current write position in the user buffer. */
	uint16_t *buffer;
	/** Start of the current sampling round, used to repeat a sampling. */
	uint16_t *repeat_buffer;
	/** Bit mask of the channels of the active sequence. */
	uint8_t channels;
	/** Resolution of the active sequence in bits. */
	uint8_t resolution;
	/** Oversampling setting (OSR_CFG value) of the active sequence. */
	uint8_t oversampling;
	/** Offset calibration requested by the active sequence. */
	bool calibrate;

	/** OSR_CFG value currently programmed in the device, or ADS7028_CACHE_INVALID. */
	uint8_t osr;
	/** AUTO_SEQ_CH_SEL value currently programmed in the device, 0 if unknown. */
	uint8_t auto_seq_mask;
	/** Channel the multiplexer is set to in manual mode, or ADS7028_CACHE_INVALID. */
	uint8_t chan_sel;
	/** CRC is currently enabled in the device. */
	bool crc_active;
	/** Time a conversion takes with the current oversampling setting. */
	uint32_t settle_us;

	/** Cycle counter at the end of the last frame. */
	uint32_t frame_end;
	/** Conversion time of the last frame, the next frame waits until it has passed. */
	uint32_t frame_settle_us;

	/** Runs one sampling round in the shared work queue. */
	struct k_work work;
};

/** Work queue shared by all instances, runs the SPI transfers of the reads. */
static struct k_work_q ads7028_workq;

static K_KERNEL_STACK_DEFINE(ads7028_workq_stack, CONFIG_ADC_ADS7028_ACQUISITION_THREAD_STACK_SIZE);

/**
 * @brief Wait for a given time, sleeping for long and busy waiting for short delays.
 *
 * @param us Time to wait in microseconds.
 */
static void ads7028_wait(uint32_t us)
{
	if (us >= ADS7028_SLEEP_THRESHOLD_US) {
		k_usleep(us);
	} else {
		k_busy_wait(us);
	}
}

/**
 * @brief Wait until the conversion started by the last frame has finished.
 *
 * Only the time not already spent since the end of the last frame is waited. If the
 * cycle counter wrapped around in between, this waits at most one conversion time too long.
 *
 * @param data Driver data.
 */
static void ads7028_wait_ready(struct ads7028_data *data)
{
	uint32_t elapsed_us = k_cyc_to_us_floor32(k_cycle_get_32() - data->frame_end);

	if (elapsed_us < data->frame_settle_us) {
		ads7028_wait(data->frame_settle_us - elapsed_us);
	}
}

/**
 * @brief Transfer one SPI frame with CS asserted.
 *
 * Waits for the conversion of the previous frame first. The conversion started by the
 * CS rising edge of this frame is not waited for, the next frame does that.
 *
 * @param dev    ADS7028 device.
 * @param opcode Command opcode (ADS7028_OP_*).
 * @param reg    Register address.
 * @param value  Register value or bit mask.
 * @param rx     Receive buffer with room for ADS7028_FRAME_LEN_CRC bytes, or NULL to
 *               discard the received data.
 *
 * @retval 0 on success.
 * @retval negative errno from the SPI driver.
 */
static int ads7028_frame(const struct device *dev, uint8_t opcode, uint8_t reg, uint8_t value,
			 uint8_t *rx)
{
	const struct ads7028_config *cfg = dev->config;
	struct ads7028_data *data = dev->data;
	uint8_t tx[ADS7028_FRAME_LEN_CRC] = {opcode, reg, value, 0};
	size_t len = ADS7028_FRAME_LEN;
	int ret;

	if (ADS7028_CRC_ACTIVE(data)) {
		tx[ADS7028_FRAME_LEN] = crc8_ccitt(ADS7028_CRC_SEED, tx, ADS7028_FRAME_LEN);
		len = ADS7028_FRAME_LEN_CRC;
	}

	const struct spi_buf tx_buf = {.buf = tx, .len = len};
	const struct spi_buf rx_buf = {.buf = rx, .len = len};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	ads7028_wait_ready(data);

	ret = spi_transceive_dt(&cfg->bus, &tx_set, (rx != NULL) ? &rx_set : NULL);

	data->frame_end = k_cycle_get_32();
	data->frame_settle_us = data->settle_us;

	return ret;
}

/**
 * @brief Read a device register.
 *
 * @param dev   ADS7028 device.
 * @param reg   Register address.
 * @param value Read value.
 *
 * @retval 0 on success.
 * @retval -EIO on CRC error.
 * @retval negative errno on SPI error.
 */
static int ads7028_reg_read(const struct device *dev, uint8_t reg, uint8_t *value)
{
	struct ads7028_data *data = dev->data;
	uint8_t rx[ADS7028_FRAME_LEN_CRC];
	int ret;

	ret = ads7028_frame(dev, ADS7028_OP_RD_REG, reg, 0, NULL);
	if (ret < 0) {
		return ret;
	}

	ret = ads7028_frame(dev, ADS7028_OP_NOP, 0, 0, rx);
	if (ret < 0) {
		return ret;
	}

	if (ADS7028_CRC_ACTIVE(data) && (crc8_ccitt(ADS7028_CRC_SEED, rx, 1) != rx[1])) {
		LOG_ERR("CRC error reading register 0x%02x", reg);

		return -EIO;
	}

	*value = rx[0];

	return 0;
}

/**
 * @brief Conversion time for an oversampling setting.
 *
 * @param osr OSR_CFG value (0 = no averaging).
 *
 * @return Conversion time in microseconds.
 */
static uint32_t ads7028_settle_us(uint8_t osr)
{
	if (osr == 0) {
		return ADS7028_T_CONV_US;
	}

	return ADS7028_T_CONV_US + (BIT(osr) * ADS7028_T_CYCLE_US);
}

/**
 * @brief Remember the oversampling setting and update the conversion settle time.
 *
 * Does not access the device.
 *
 * @param data Driver data.
 * @param osr  OSR_CFG value (0 = no averaging).
 */
static void ads7028_set_osr(struct ads7028_data *data, uint8_t osr)
{
	data->osr = osr;
	data->settle_us = ads7028_settle_us(osr);
}

/**
 * @brief Mark all cached register values as unknown.
 *
 * Used when the device state is uncertain, e.g. after a failed transfer. The next
 * acquisition rewrites the registers, until then the longest conversion time is used.
 *
 * @param data Driver data.
 */
static void ads7028_invalidate_cache(struct ads7028_data *data)
{
	data->osr = ADS7028_CACHE_INVALID;
	data->settle_us = ads7028_settle_us(ADS7028_MAX_OVERSAMPLING);
	data->auto_seq_mask = 0;
	data->chan_sel = ADS7028_CACHE_INVALID;
}

/**
 * @brief Start an offset calibration and wait until it has finished.
 *
 * @param dev ADS7028 device.
 *
 * @retval 0 on success.
 * @retval -ETIMEDOUT if the calibration did not finish in time.
 * @retval negative errno on SPI error.
 */
static int ads7028_calibrate(const struct device *dev)
{
	struct ads7028_data *data = dev->data;
	uint8_t general_cfg;
	int ret;

	/* Do not rely on the multiplexer setting across a calibration */
	data->chan_sel = ADS7028_CACHE_INVALID;

	ret = ads7028_frame(dev, ADS7028_OP_SET_BIT, ADS7028_REG_GENERAL_CFG,
			    ADS7028_GENERAL_CFG_CAL, NULL);
	if (ret < 0) {
		return ret;
	}

	for (uint32_t t = 0; t < ADS7028_CAL_TIMEOUT_US; t += ADS7028_CAL_POLL_US) {
		ret = ads7028_reg_read(dev, ADS7028_REG_GENERAL_CFG, &general_cfg);
		if (ret < 0) {
			return ret;
		}

		if ((general_cfg & ADS7028_GENERAL_CFG_CAL) == 0) {
			return 0;
		}

		/* Sleep even if the tick rounding makes it longer, the calibration takes a while */
		k_usleep(ADS7028_CAL_POLL_US);
	}

	LOG_ERR("Offset calibration timed out");

	return -ETIMEDOUT;
}

/**
 * @brief Decode the conversion result of a frame and check CRC and the appended status.
 *
 * @param dev     ADS7028 device.
 * @param rx      Received frame.
 * @param channel Channel the data is expected from.
 * @param sample  Decoded sample, scaled to the requested resolution.
 *
 * @retval 0 on success.
 * @retval -EIO on CRC error, channel mismatch or invalid status flags.
 */
static int ads7028_decode(const struct device *dev, const uint8_t *rx, uint8_t channel,
			  uint16_t *sample)
{
	const struct ads7028_config *cfg = dev->config;
	struct ads7028_data *data = dev->data;
	size_t len = (data->osr > 0) ? 3 : 2;
	uint16_t value;
	uint8_t append;

	if (ADS7028_CRC_ACTIVE(data) && (crc8_ccitt(ADS7028_CRC_SEED, rx, len) != rx[len])) {
		LOG_ERR("CRC error in data of channel %u", channel);

		return -EIO;
	}

	if (data->osr > 0) {
		uint32_t raw = sys_get_be24(rx);

		value = raw >> 8;
		append = (raw >> 4) & 0x0F;
	} else {
		uint16_t raw = sys_get_be16(rx);

		value = raw >> 4;
		append = raw & 0x0F;
	}

	switch (cfg->append_status) {
	case ADS7028_APPEND_CHANNEL_ID: {
		if (append != channel) {
			LOG_ERR("Expected data of channel %u, got channel %u", channel, append);

			return -EIO;
		}
		break;
	}
	case ADS7028_APPEND_STATUS_FLAGS: {
		if ((append & ADS7028_STATUS_FLAGS_FIXED_MASK) != ADS7028_STATUS_FLAGS_FIXED) {
			LOG_ERR("Invalid status flags 0x%x for channel %u", append, channel);

			return -EIO;
		}

		if ((append & ADS7028_STATUS_FLAGS_CRC_ERR_IN) != 0) {
			LOG_ERR("Device reports a CRC error on its input");

			return -EIO;
		}
		break;
	}
	default: {
		break;
	}
	}

	if ((data->osr > 0) && (data->resolution == ADS7028_RESOLUTION)) {
		value >>= (ADS7028_RESOLUTION_AVG - ADS7028_RESOLUTION);
	}

	*sample = value;

	return 0;
}

/**
 * @brief Read the channels in manual mode, selecting each channel with a register write.
 *
 * The channel selection is pipelined: the write that selects a channel is sent in the
 * frame that also returns the result of the previous one. A channel write takes effect
 * for the conversion of the following frame.
 *
 * If the multiplexer is still set to the first channel from the previous round, the
 * selection frame for it is skipped.
 *
 * @param dev   ADS7028 device.
 * @param list  Channels to read, in ascending order.
 * @param count Number of entries in @p list.
 *
 * @retval 0 on success.
 * @retval negative errno on SPI or data error.
 */
static int ads7028_acquire_manual(const struct device *dev, const uint8_t *list, size_t count)
{
	struct ads7028_data *data = dev->data;
	uint8_t rx[ADS7028_FRAME_LEN_CRC];
	bool selected = (data->chan_sel == list[0]);
	int ret;

	/* Unknown until the round has finished without error */
	data->chan_sel = ADS7028_CACHE_INVALID;

	if (selected == false) {
		ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_CHANNEL_SEL, list[0], NULL);
		if (ret < 0) {
			return ret;
		}
	}

	/* Samples the first channel, the data out are stale */
	if (count > 1) {
		ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_CHANNEL_SEL, list[1], NULL);
	} else {
		ret = ads7028_frame(dev, ADS7028_OP_NOP, 0, 0, NULL);
	}

	if (ret < 0) {
		return ret;
	}

	for (size_t i = 0; i < count; i++) {
		if ((i + 2) < count) {
			ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_CHANNEL_SEL,
					    list[i + 2], rx);
		} else {
			ret = ads7028_frame(dev, ADS7028_OP_NOP, 0, 0, rx);
		}
		if (ret < 0) {
			return ret;
		}

		ret = ads7028_decode(dev, rx, list[i], data->buffer++);
		if (ret < 0) {
			return ret;
		}
	}

	/* The last channel written stays selected */
	data->chan_sel = list[count - 1];

	return 0;
}

/**
 * @brief Read the channels with the internal channel sequencer (auto mode).
 *
 * @param dev   ADS7028 device.
 * @param list  Channels to read, in ascending order.
 * @param count Number of entries in @p list.
 *
 * @retval 0 on success.
 * @retval negative errno on SPI or data error.
 */
static int ads7028_acquire_auto(const struct device *dev, const uint8_t *list, size_t count)
{
	struct ads7028_data *data = dev->data;
	uint8_t rx[ADS7028_FRAME_LEN_CRC];
	int ret;

	if (data->auto_seq_mask != data->channels) {
		ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_AUTO_SEQ_CH_SEL,
				    data->channels, NULL);
		if (ret < 0) {
			return ret;
		}

		data->auto_seq_mask = data->channels;
	}

	ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_SEQUENCE_CFG,
			    ADS7028_SEQUENCE_CFG_SEQ_MODE_AUTO | ADS7028_SEQUENCE_CFG_SEQ_START,
			    NULL);
	if (ret < 0) {
		return ret;
	}

	/* Samples the first channel, the data out are stale */
	ret = ads7028_frame(dev, ADS7028_OP_NOP, 0, 0, NULL);
	if (ret < 0) {
		return ret;
	}

	for (size_t i = 0; i < count; i++) {
		if ((i + 1) == count) {
			ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_SEQUENCE_CFG,
					    ADS7028_SEQUENCE_CFG_SEQ_MODE_AUTO, rx);
		} else {
			ret = ads7028_frame(dev, ADS7028_OP_NOP, 0, 0, rx);
		}
		if (ret < 0) {
			return ret;
		}

		ret = ads7028_decode(dev, rx, list[i], data->buffer++);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

/**
 * @brief Run one sampling round of the active sequence into the user buffer.
 *
 * Applies a pending calibration and oversampling change first.
 *
 * @param dev ADS7028 device.
 *
 * @retval 0 on success.
 * @retval negative errno on SPI or data error.
 */
static int ads7028_acquire(const struct device *dev)
{
	const struct ads7028_config *cfg = dev->config;
	struct ads7028_data *data = dev->data;
	uint8_t list[ADS7028_NUM_CHANNELS];
	size_t count = 0;
	int ret;

	if (data->calibrate == true) {
		data->calibrate = false;

		ret = ads7028_calibrate(dev);
		if (ret < 0) {
			return ret;
		}
	}

	if (data->osr != data->oversampling) {
		/* The conversion of this frame may already use the new setting */
		data->settle_us = MAX(data->settle_us, ads7028_settle_us(data->oversampling));

		ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_OSR_CFG, data->oversampling,
				    NULL);
		if (ret < 0) {
			return ret;
		}

		/* Only update the cache when the device has the new value */
		ads7028_set_osr(data, data->oversampling);
	}

	for (uint8_t ch = 0; ch < ADS7028_NUM_CHANNELS; ch++) {
		if ((data->channels & BIT(ch)) != 0) {
			list[count++] = ch;
		}
	}

	if (ADS7028_AUTO_SEQ(cfg)) {
		return ads7028_acquire_auto(dev, list, count);
	}

	return ads7028_acquire_manual(dev, list, count);
}

/**
 * @brief Bring the device back into a known state after a failed read.
 *
 * Clears the CRC error flag, stops a running sequence and forgets the cached register
 * values, so the next acquisition writes them again.
 *
 * @param dev ADS7028 device.
 */
static void ads7028_recover(const struct device *dev)
{
	const struct ads7028_config *cfg = dev->config;
	struct ads7028_data *data = dev->data;

	ads7028_invalidate_cache(data);

	if (ADS7028_CRC_ACTIVE(data)) {
		(void)ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_SYSTEM_STATUS,
				    ADS7028_SYSTEM_STATUS_CRC_ERR_IN, NULL);
	}

	if (ADS7028_AUTO_SEQ(cfg)) {
		(void)ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_SEQUENCE_CFG,
				    ADS7028_SEQUENCE_CFG_SEQ_MODE_AUTO, NULL);
	}
}

/**
 * @brief Acquisition work, runs one sampling round each time it is submitted.
 *
 * @param work Work item of the instance (ads7028_data.work).
 */
static void ads7028_acquisition_work(struct k_work *work)
{
	struct ads7028_data *data = CONTAINER_OF(work, struct ads7028_data, work);
	const struct device *dev = data->dev;
	int ret;

	ret = ads7028_acquire(dev);
	if (ret < 0) {
		LOG_ERR("Acquisition failed: %d", ret);

		ads7028_recover(dev);

		if ((data->ctx.sequence.options != NULL) &&
		    (data->ctx.sequence.options->interval_us != 0U)) {
			adc_context_disable_timer(&data->ctx);
		}

		adc_context_complete(&data->ctx, ret);

		return;
	}

	/* Resubmits the work for the next sampling if there is one without interval */
	adc_context_on_sampling_done(&data->ctx, dev);
}

/**
 * @brief ADC context callback, start a sampling round.
 *
 * @param ctx ADC context.
 */
static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads7028_data *data = CONTAINER_OF(ctx, struct ads7028_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	(void)k_work_submit_to_queue(&ads7028_workq, &data->work);
}

/**
 * @brief ADC context callback, rewind the buffer pointer when a sampling is repeated.
 *
 * @param ctx             ADC context.
 * @param repeat_sampling True to repeat the last sampling into the same buffer position.
 */
static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct ads7028_data *data = CONTAINER_OF(ctx, struct ads7028_data, ctx);

	if (repeat_sampling == true) {
		data->buffer = data->repeat_buffer;
	}
}

/**
 * @brief ADC API channel_setup, validate the channel configuration.
 *
 * @param dev         ADS7028 device.
 * @param channel_cfg Channel configuration.
 *
 * @retval 0 on success.
 * @retval -EINVAL on invalid channel id.
 * @retval -ENOTSUP on unsupported gain, reference, acquisition time or differential mode.
 */
static int ads7028_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	ARG_UNUSED(dev);

	if (channel_cfg->channel_id >= ADS7028_NUM_CHANNELS) {
		LOG_ERR("Unsupported channel %u", channel_cfg->channel_id);

		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported gain %d", channel_cfg->gain);

		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_VDD_1) {
		LOG_ERR("Unsupported reference %d, AVDD is the reference", channel_cfg->reference);

		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported acquisition time %u", channel_cfg->acquisition_time);

		return -ENOTSUP;
	}

	if (channel_cfg->differential == true) {
		LOG_ERR("Differential mode is not supported");

		return -ENOTSUP;
	}

	/* All channels are analog inputs after reset, nothing to configure */
	return 0;
}

/**
 * @brief Validate a sequence, start it and wait for completion.
 *
 * @param dev      ADS7028 device.
 * @param sequence Sequence to run.
 *
 * @retval 0 on success.
 * @retval -ENOTSUP on unsupported channels, oversampling or resolution.
 * @retval -ENOMEM if the buffer is too small.
 * @retval negative errno from the acquisition.
 */
static int ads7028_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ads7028_data *data = dev->data;
	size_t needed;

	if ((sequence->channels == 0) || ((sequence->channels & ~ADS7028_CHANNEL_MASK) != 0)) {
		LOG_ERR("Unsupported channel selection 0x%08x", sequence->channels);

		return -ENOTSUP;
	}

	if (sequence->oversampling > ADS7028_MAX_OVERSAMPLING) {
		LOG_ERR("Unsupported oversampling %u", sequence->oversampling);

		return -ENOTSUP;
	}

	if ((sequence->resolution != ADS7028_RESOLUTION) &&
	    ((sequence->resolution != ADS7028_RESOLUTION_AVG) || (sequence->oversampling == 0))) {
		LOG_ERR("Unsupported resolution %u with oversampling %u", sequence->resolution,
			sequence->oversampling);

		return -ENOTSUP;
	}

	needed = POPCOUNT(sequence->channels) * sizeof(uint16_t);
	if (sequence->options != NULL) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		LOG_ERR("Buffer too small, %zu bytes needed", needed);

		return -ENOMEM;
	}

	data->resolution = sequence->resolution;
	data->oversampling = sequence->oversampling;
	data->calibrate = sequence->calibrate;
	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

/**
 * @brief ADC API read_async.
 *
 * @param dev      ADS7028 device.
 * @param sequence Sequence to run.
 * @param async    Signal raised on completion, or NULL for a synchronous read.
 *
 * @return See ads7028_start_read().
 */
static int ads7028_read_async(const struct device *dev, const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct ads7028_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, async != NULL, async);
	ret = ads7028_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}

/**
 * @brief ADC API read, synchronous.
 *
 * @param dev      ADS7028 device.
 * @param sequence Sequence to run.
 *
 * @return See ads7028_start_read().
 */
static int ads7028_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return ads7028_read_async(dev, sequence, NULL);
}

/**
 * @brief Reset the device and verify that it responds.
 *
 * @param dev ADS7028 device.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the device does not respond.
 * @retval negative errno on SPI or CRC error.
 */
static int ads7028_reset(const struct device *dev)
{
	const struct ads7028_config *cfg = dev->config;
	struct ads7028_data *data = dev->data;
	uint8_t status;
	int ret;

	/* Register values after reset */
	ads7028_set_osr(data, 0);
	data->auto_seq_mask = 0;
	data->chan_sel = 0;

	/* A previous run may have left the CRC enabled, so send the reset with and without CRC */
	data->crc_active = false;
	ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_GENERAL_CFG,
			    ADS7028_GENERAL_CFG_RST, NULL);
	if (ret < 0) {
		return ret;
	}

	if (ADS7028_CRC_ENABLED(cfg)) {
		data->crc_active = true;
		ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_GENERAL_CFG,
				    ADS7028_GENERAL_CFG_RST, NULL);
		data->crc_active = false;
		if (ret < 0) {
			return ret;
		}
	}

	k_msleep(ADS7028_RESET_TIME_MS);

	ret = ads7028_reg_read(dev, ADS7028_REG_SYSTEM_STATUS, &status);
	if (ret < 0) {
		return ret;
	}

	if ((status & ADS7028_SYSTEM_STATUS_RSVD) == 0) {
		LOG_ERR("No response, SYSTEM_STATUS = 0x%02x", status);

		return -ENODEV;
	}

	/* Clear the brown out flag of the reset */
	return ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_SYSTEM_STATUS,
			     ADS7028_SYSTEM_STATUS_BOR, NULL);
}

/**
 * @brief Apply the devicetree configuration (status append, sequencer, calibration, CRC).
 *
 * @param dev ADS7028 device.
 *
 * @retval 0 on success.
 * @retval -EIO if the device reports a CRC error after enabling CRC.
 * @retval negative errno on SPI error or calibration failure.
 */
static int ads7028_configure(const struct device *dev)
{
	const struct ads7028_config *cfg = dev->config;
	struct ads7028_data *data = dev->data;
	uint8_t status;
	int ret;

	if (cfg->append_status != ADS7028_APPEND_NONE) {
		ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_DATA_CFG,
				    cfg->append_status << ADS7028_DATA_CFG_APPEND_STATUS_POS, NULL);
		if (ret < 0) {
			return ret;
		}
	}

	if (ADS7028_AUTO_SEQ(cfg)) {
		ret = ads7028_frame(dev, ADS7028_OP_WR_REG, ADS7028_REG_SEQUENCE_CFG,
				    ADS7028_SEQUENCE_CFG_SEQ_MODE_AUTO, NULL);
		if (ret < 0) {
			return ret;
		}
	}

	if (cfg->calibrate == true) {
		ret = ads7028_calibrate(dev);
		if (ret < 0) {
			return ret;
		}
	}

	if (ADS7028_CRC_ENABLED(cfg)) {
		ret = ads7028_frame(dev, ADS7028_OP_SET_BIT, ADS7028_REG_GENERAL_CFG,
				    ADS7028_GENERAL_CFG_CRC_EN, NULL);
		if (ret < 0) {
			return ret;
		}

		data->crc_active = true;

		/* Verify that the device talks CRC now */
		ret = ads7028_reg_read(dev, ADS7028_REG_SYSTEM_STATUS, &status);
		if (ret < 0) {
			return ret;
		}

		if ((status & ADS7028_SYSTEM_STATUS_CRC_ERR_IN) != 0) {
			LOG_ERR("Device reports a CRC error after enabling CRC");

			return -EIO;
		}
	}

	return 0;
}

/**
 * @brief Start the work queue shared by all instances, on the first call only.
 *
 * Device init functions run one after the other, so no locking is needed.
 */
static void ads7028_workq_start(void)
{
	static bool started;
	const struct k_work_queue_config workq_cfg = {.name = "ads7028"};

	if (started == true) {
		return;
	}

	k_work_queue_init(&ads7028_workq);
	k_work_queue_start(&ads7028_workq, ads7028_workq_stack,
			   K_KERNEL_STACK_SIZEOF(ads7028_workq_stack),
			   CONFIG_ADC_ADS7028_ACQUISITION_THREAD_PRIO, &workq_cfg);

	started = true;
}

/**
 * @brief Initialize the driver: reset and configure the device, start the work queue.
 *
 * @param dev ADS7028 device.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the SPI bus is not ready or the device does not respond.
 * @retval negative errno on other errors.
 */
static int ads7028_init(const struct device *dev)
{
	const struct ads7028_config *cfg = dev->config;
	struct ads7028_data *data = dev->data;
	int ret;

	data->dev = dev;

	adc_context_init(&data->ctx);
	k_work_init(&data->work, ads7028_acquisition_work);

	if (spi_is_ready_dt(&cfg->bus) == false) {
		LOG_ERR("SPI bus %s not ready", cfg->bus.bus->name);

		return -ENODEV;
	}

	ret = ads7028_reset(dev);
	if (ret < 0) {
		LOG_ERR("Reset failed: %d", ret);

		return ret;
	}

	ret = ads7028_configure(dev);
	if (ret < 0) {
		LOG_ERR("Configuration failed: %d", ret);

		return ret;
	}

	ads7028_workq_start();

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/** ADC driver API of the ADS7028 */
static DEVICE_API(adc, ads7028_api) = {
	.channel_setup = ads7028_channel_setup,
	.read = ads7028_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads7028_read_async,
#endif
	.ref_internal = 0,
};

/** SPI operation flags: controller mode, 8 bit words, MSB first */
#define ADS7028_SPI_OPERATION (SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

/**
 * @brief Define config, data and device of one ADS7028 devicetree instance.
 *
 * @param n Devicetree instance number.
 */
#define ADS7028_INIT(n)                                                                            \
	static const struct ads7028_config ads7028_config_##n = {                                  \
		.bus = SPI_DT_SPEC_INST_GET(n, ADS7028_SPI_OPERATION),                             \
		.append_status = DT_INST_ENUM_IDX(n, ti_append_status),                            \
		.auto_sequence = DT_INST_PROP(n, ti_auto_sequence),                                \
		.crc = DT_INST_PROP(n, ti_crc),                                                    \
		.calibrate = DT_INST_PROP(n, ti_calibrate_on_init),                                \
	};                                                                                         \
                                                                                                   \
	static struct ads7028_data ads7028_data_##n;                                               \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ads7028_init, NULL, &ads7028_data_##n, &ads7028_config_##n,       \
			      POST_KERNEL, CONFIG_ADC_ADS7028_INIT_PRIORITY, &ads7028_api);

DT_INST_FOREACH_STATUS_OKAY(ADS7028_INIT)
