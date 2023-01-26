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
   struct max30102_data *data = dev->data;
   struct ring_buf *ringBuf = NULL;
   uint32_t *buffer = NULL;
   uint32_t dataRet = 0U;
   uint32_t rbData = 0u;
   uint16_t rbType = 0u;
   uint8_t  rbValue = 0u;
   uint8_t  rbSize = 0u;
   int ret = 0;

   /** Determine the data type need to return based on chan variable. */
   switch (chan)
   {
   case SENSOR_CHAN_IR:
      ringBuf = &data->rawRedRb;
      buffer = data->rawRed;
      break;
   case SENSOR_CHAN_RED:
      ringBuf = &data->rawIRRb;
      buffer = data->rawIR;
      break;
   default:
      LOG_ERR ("Unsupported sensor channel.");
      return -ENOTSUP;
   }

   /** Get the data from ring buffer. */
   ring_buf_item_get (ringBuf, rbType, rbValue, &dataRet, rbSize);
   LOG_DBG ("Got item of type %u value &u of size %u dwords\n", rbType, rbValue, rbSize);

   val->val1 = dataRet;
   val->val2 = 0;

   /** Return status. */
   return ret;
}

static int max30102_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan)
{
   struct max30102_data *data = dev->data;
   const struct max30102_config *config = dev->config;
}

static const struct sensor_driver_api max30102_driver_api = {
    .sample_fetch = max30102_sample_fetch,
    .channel_get = max30102_channel_get,
};

static int max30102_int(const struct device *dev)
{
   struct max30102_config *config = dev->config;
   struct max30102_data *data = dev->data;
   uint8_t part_id = 0U;
   max30102_mode_t modeCfg = 0U;
   uint32_t ledChnl;
   int fifoChnl;

   /** Initialize the buffer for Red and IR value. */
   ring_buf_item_init (&data->rawRedRb, MAX30102_NO_OF_ITEM, data->rawRed);
   ring_buf_item_init (&data->rawIRRb, MAX30102_NO_OF_ITEM, data->rawIR);

   /** Config mode and slot. */
#ifdef CONFIG_MAX30102_HEART_RATE_MODE
   config->mode.B.mode = MAX30102_MODE_HEART_RATE;
#elif CONFIG_MAX30102_SPO2_MODE
   config->mode.B.mode = MAX30102_MODE_SPO2;
#elif CONFIG_MAX30102_MULTI_LED_MODE
   config->mode.B.mode = MAX30102_MODE_MULTI_LED;
#endif

   /** Determine whether i2c bus ready. */
   if (!device_is_ready (config->i2c.bus)) 
   {
      LOG_ERR ("Bus device is not ready!");
      return -ENODEV;
   }

   /* Check the part id to make sure this is MAX30102 */
   if (i2c_reg_read_byte_dt (&config->i2c, MAX30102_REG_PART_ID,
                             &part_id))
   {
      LOG_ERR("Could not get Part ID");
      return -EIO;
   }

   if (part_id != MAX30102_PART_ID)
   {
      LOG_ERR("Got Part ID 0x%02x, expected 0x%02x",
              part_id, MAX30102_PART_ID);
      return -EIO;
   }

   /** Reset the sensor. */
   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_MODE_CFG, MAX30102_MODE_CFG_RESET_MASK))
   {
      return -EIO;
   }

   /** Wait for reset to be cleared. */
   do
   {
      if (i2c_reg_read_byte_dt (&config->i2c, MAX30102_REG_MODE_CFG,
                                &modeCfg))
      {
         LOG_ERR("Could read mode cfg after reset");
         return -EIO;
      }
   } while (modeCfg & MAX30102_MODE_CFG_RESET_MASK);

   /** Write the FIFO configuration register */
   if (i2c_reg_write_byte_dt  (&config->i2c, MAX30102_REG_FIFO_CFG,
                               config->fifo.R))
   {
      return -EIO;
   }

   /** Write the mode configuration register */
   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_MODE_CFG,
                              config->mode.R))
   {
      return -EIO;
   }

   /* Write the SpO2 configuration register */
   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_SPO2_CFG,
                              config->spo2.R))
   {
      return -EIO;
   }

   /* Write the LED pulse amplitude registers */
   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_LED1_PA,
                              config->ledPa[0]))
   {
      return -EIO;
   }
   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_LED2_PA,
                              config->ledPa[1]))
   {
      return -EIO;
   }

#ifdef CONFIG_MAX30102_MULTI_LED_MODE
   uint8_t multiLed[2] = {0U};

   /* Write the multi-LED mode control registers */
   multiLed[0] = (config->slot[1] << 4) | (config->slot[0]);
   multiLed[1] = (config->slot[3] << 4) | (config->slot[2]);

   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_MULTI_LED,
                              multiLed[0]))
   {
      return -EIO;
   }
   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_MULTI_LED + 1,
                              multiLed[1]))
   {
      return -EIO;
   }
#endif
}

static int max30102_deinit (const struct device *dev)
{
   const struct max30102_config *config = dev->config;
   uint8_t res = 0U;
   uint8_t prev = 0U;
   uint8_t ret = 0U;

   /** Determine whether i2c bus ready. */
   if (!device_is_ready (config->i2c.bus)) 
   {
      LOG_ERR ("Bus device is not ready!");
      return -ENODEV;
   }

   if (0 != i2c_reg_read_byte_dt (&config->i2c, MAX30102_REG_MODE_CFG, &prev))
   {
      LOG_ERR("Read mode config failed.");
      ret = -EIO; 
      return ret;
   }

   prev &=~ (1u << 7u);
   prev |= (1u << 7u);
   ret = i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_MODE_CFG, &prev);
   if (0u != ret) 
   {
      LOG_ERR("Write mode config failed.");
      return ret;
   }

   /** FIXME: Deinit i2c bus used for this sensor. */

   return ret;
}

static struct max30102_data max30102_data;              
static const struct max30102_config max30102_config = { 
   .i2c = I2C_DT_SPEC_INST_GET(0),                          
   .fifo.B.smpAve = CONFIG_MAX30102_SMP_AVE,                   
   .fifo.B.enRollOver = CONFIG_MAX30102_FIFO_ROLLOVER_EN,      
   .fifo.B.fifoAlmostFull = CONFIG_MAX30102_FIFO_A_FULL,       
   .spo2.B.adcRange = CONFIG_MAX30102_ADC_RGE,                 
   .spo2.B.sampleRate = CONFIG_MAX30102_SR,                    
   .spo2.B.ledPw = MAX30102_PW_18BITS,                         
#ifdef CONFIG_MAX30102_HEART_RATE_MODE
   .mode.B.mode = MAX30102_MODE_HEART_RATE,
   .slot[0] = MAX30102_LED_RED,
   .slot[1] = MAX30102_LED_NONE,
   .slot[2] = MAX30102_LED_NONE,
   .slot[3] = MAX30102_LED_NONE,
#elif CONFIG_MAX30102_SPO2_MODE
   .mode.B.mode = MAX30102_MODE_SPO2,
   .slot[0] = MAX30102_LED_RED,
   .slot[1] = MAX30102_LED_IR,
   .slot[2] = MAX30102_LED_NONE,
   .slot[3] = MAX30102_LED_NONE,
#elif CONFIG_MAX30102_MULTI_LED_MODE
   .mode.B.mode = MAX30102_MODE_MULTI_LED,
   .slot[0] = CONFIG_MAX30102_SLOT1,
   .slot[1] = CONFIG_MAX30102_SLOT2,
   .slot[2] = CONFIG_MAX30102_SLOT3,
   .slot[3] = CONFIG_MAX30102_SLOT4,
#endif
   .mode.B.RESET = false,
   .mode.B.SHDN = false,
   .ledPa[0] = CONFIG_MAX30102_LED1_PA,
   .ledPa[1] = CONFIG_MAX30102_LED2_PA,
};

SENSOR_DEVICE_DT_INST_DEFINE(0, max30102_int, NULL,                              
                             &max30102_data,                                 
                             &max30102_config, POST_KERNEL,                  
                             CONFIG_SENSOR_INIT_PRIORITY, &max30102_driver_api);