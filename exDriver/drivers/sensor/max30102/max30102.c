/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max30102

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "max30102.h"

#define MAX30102_REG_INT_STS1 (0x00U)
#define MAX30102_REG_INT_STS2 (0x01U)
#define MAX30102_REG_INT_EN1 (0x02U)
#define MAX30102_REG_INT_EN2 (0x03U)
#define MAX30102_REG_FIFO_WR (0x04U)
#define MAX30102_REG_FIFO_OVF (0x05U)
#define MAX30102_REG_FIFO_RD (0x06U)
#define MAX30102_REG_FIFO_DATA (0x07U)
#define MAX30102_REG_FIFO_CFG (0x08U)
#define MAX30102_REG_MODE_CFG (0x09U)
#define MAX30102_REG_SPO2_CFG (0x0AU)
#define MAX30102_REG_LED1_PA (0x0CU)
#define MAX30102_REG_LED2_PA (0x0DU)
#define MAX30102_REG_MULTI_LED (0x11U)
#define MAX30102_REG_TINT (0x1FU)
#define MAX30102_REG_TFRAC (0x20U)
#define MAX30102_REG_TEMP_CFG (0x21U)
#define MAX30102_REG_REV_ID (0xFEU)
#define MAX30102_REG_PART_ID (0xFFU)

#define MAX30102_FIFO_CFG_SMP_AVE_SHIFT (5U)
#define MAX30102_FIFO_CFG_FIFO_FULL_SHIFT (0U)
#define MAX30102_FIFO_CFG_ROLLOVER_EN_MASK (1U << 4)

#define MAX30102_MODE_CFG_SHDN_MASK (1U << 7)
#define MAX30102_MODE_CFG_RESET_MASK (1U << 6)

#define MAX30102_SPO2_ADC_RGE_SHIFT (5U)
#define MAX30102_SPO2_SR_SHIFT (2U)
#define MAX30102_SPO2_PW_SHIFT (0U)

#define MAX30102_INT_PPG_MASK (1U << 6)
#define MAX30102_BYTES_PER_CHANNEL (3U)
#define MAX30102_MAX_NUM_CHANNELS (2U)
#define MAX30102_MAX_BYTES_PER_SAMPLE (MAX30102_MAX_NUM_CHANNELS * \
                                       MAX30102_BYTES_PER_CHANNEL)
#define MAX30102_SLOT_LED_MASK (0x03U)
#define MAX30102_FIFO_DATA_BITS (18U)
#define MAX30102_FIFO_DATA_MASK ((1U << MAX30102_FIFO_DATA_BITS) - 1U)

#define MAX30102_PART_ID (0x15U)

LOG_MODULE_REGISTER(max30102, CONFIG_SENSOR_LOG_LEVEL);

static int max30102_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val)
{
   
}

static int max30102_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan)
{

}

static const struct sensor_driver_api max30102_driver_api = {
    .sample_fetch = max30102_sample_fetch,
    .channel_get = max30102_channel_get,
};

static struct max30102_config max30102_config = {
    // .i2c = I2C_DT_SPEC_INST_GET(0),
};

static struct max30102_data max30102_data;

static int max30102_int(const struct device *dev)
{

}

#define MAX30102_INIT(inst)                                                            \
   static struct max30102_data max30102_data_##inst;                                   \
                                                                                       \
   static const struct max30102_config max30102_config_##inst;                         \
                                                                                       \
   SENSOR_DEVICE_DT_INST_DEFINE(inst, max30102_int, NULL,                              \
                                &max30102_data_##inst,                                 \
                                &max30102_config_##inst, POST_KERNEL,                  \
                                CONFIG_SENSOR_INIT_PRIORITY, &max30102_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX30102_INIT)