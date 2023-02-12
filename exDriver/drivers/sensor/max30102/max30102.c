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

LOG_MODULE_REGISTER(max30102, CONFIG_SENSOR_LOG_LEVEL);

/** 
 * @brief The mutex used in this sensor.
 */
static SYS_MUTEX_DEFINE (mutex);

static int max30102_channel_get (const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{
   struct max30102_data *data = (struct max30102_data *)dev->data;
   struct ring_buf *ringBuf = NULL;
   uint32_t *buffer = NULL;
   uint32_t dataRet = 0U;
   float temp = 0;
   uint8_t raw[2] = {0};
   int ret = 0;

   /** Determine the data type need to return based on chan variable. */
   switch (chan)
   {
   case SENSOR_CHAN_IR:
      ringBuf = &data->rawIRRb;
      buffer = data->rawIR;
      break;
   case SENSOR_CHAN_RED:
      ringBuf = &data->rawRedRb;
      buffer = data->rawRed;
      break;
   case SENSOR_CHAN_DIE_TEMP:
      max30102_read_temperature (dev, raw, &temp);
      val->val1 = raw[0];
      val->val2 = raw[1];
      break;
   default:
      LOG_ERR ("Unsupported sensor channel.");
      return -ENOTSUP;
   }

   if (SENSOR_CHAN_DIE_TEMP != chan)
   {
      /** Get the data from ring buffer. */
      sys_mutex_lock(&mutex, K_FOREVER);
      if (false == ring_buf_is_empty(ringBuf))
      {
         ring_buf_get(ringBuf, (uint8_t *)&dataRet, sizeof(dataRet));
      }
      else
      {
         ret = -ENODATA;
      }
      sys_mutex_unlock(&mutex);

      val->val1 = dataRet;
      val->val2 = 0;
   }

   /** Return status. */
   return ret;
}

static int max30102_sample_fetch (const struct device *dev,
				                      enum sensor_channel chan)
{

}

static const struct sensor_driver_api max30102_driver_api = {
   .sample_fetch = max30102_sample_fetch,
   .channel_get = max30102_channel_get,
};

static int max30102_int (const struct device *dev)
{
   struct max30102_config *config = dev->config;
   struct max30102_data *data = dev->data;
   uint8_t part_id = 0U;
   max30102_mode_t modeCfg = 0U;
   uint32_t ledChnl;
   int fifoChnl;
   uint8_t byteRead = 0u;

   /** Initialize the buffer for Red and IR value. */
   ring_buf_init (&data->rawRedRb, MAX30102_NO_OF_ITEM * sizeof(data->rawRed[0]), data->rawRed);
   ring_buf_init (&data->rawIRRb, MAX30102_NO_OF_ITEM * sizeof(data->rawIR[0]), data->rawIR);

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

   /* Read mode config */
   if (i2c_reg_read_byte_dt (&config->i2c, MAX30102_REG_MODE_CFG, &byteRead))
   {
      LOG_ERR ("Max30102: Read mode config failed.");
      return -EIO;
   }

   /* Clear config */
   byteRead &= ~(1U << 6U);
   byteRead |= (1u << 6u);
   
   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_MODE_CFG, &byteRead))
   {
      LOG_ERR ("Max30102: Write mode config failed.");
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

   /* Setup interrupt. */
   max30102_setup_interrupt(dev);

   /** Write the FIFO configuration register */
   if (i2c_reg_write_byte_dt  (&config->i2c, MAX30102_REG_FIFO_CFG,
                               config->fifo.R))
   {
      return -EIO;
   }

   /** Read fifo read point */
   if (0 != i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_FIFO_RD, 0x00))
   {
      LOG_ERR ("Write fifo read point failed.");
   }

   /** Read fifo write point */
   if (0 != i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_FIFO_RD, 0x00))
   {
      LOG_ERR ("Write fifo write point failed.");
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

static int max30102_read_temperature (const struct device *dev, uint8_t * raw, float *temp)
{
   struct max30102_config *config = dev->config;
   uint8_t retI2c = 0u;
   int8_t tempInt = 0;
   uint8_t tempFrac = 0u;
   uint8_t counter = 0u;
   int ret = 0;

   /** Determine whether i2c bus ready. */
   if (!device_is_ready(config->i2c.bus))
   {
      LOG_ERR ("Bus device is not ready!");
      return -ENODEV;
   }

   /** Config die temperature register to take 1 temperature sample. */
   if (i2c_reg_write_byte_dt (&config->i2c, MAX30102_REG_TEMP_CFG, 0x01u))
   {
      LOG_ERR ("Max30102: Error enable configuration die temperature!");
      return -EIO;
   }

   /** Wait for reset to be cleared. */
   do
   {
      if (i2c_reg_read_byte_dt (&config->i2c, MAX30102_REG_INT_STS2,
                                &retI2c))
      {
         LOG_ERR ("Max30102: Could read mode cfg after reset");
         return -EIO;
      }

      if (false != (retI2c & (1u << MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY))) break;
      
      k_busy_wait (1U);
      
      counter += 1;
   } while (counter < 100u);

   /** Read die temperature register. */
   i2c_reg_read_byte_dt (&config->i2c, MAX30102_REG_TINT, &tempInt);
   i2c_reg_read_byte_dt (&config->i2c, MAX30102_REG_TFRAC, &tempFrac);

   *temp = (float)tempInt + ((float)tempFrac * 0.0625f);

   raw[0] = tempInt;
   raw[1] = tempFrac;

   ret = 0;
}

struct max30102_data max30102_data = {0};

const struct max30102_config max30102_config = { 
   .i2c = I2C_DT_SPEC_INST_GET(0),                          
   .irq1.B.a_full_en = true,
   .irq1.B.alc_ovf_en = false,
   .irq2.B.die_temp_rdy_en = false,
   .irq1.B.ppg_rdy_en = false,
   .fifo.B.smpAve = CONFIG_MAX30102_SMP_AVE,                   
   .fifo.B.enRollOver = false,
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
   .int_gpio = GPIO_DT_SPEC_INST_GET_OR (0, int_gpios, {0}),
};

SENSOR_DEVICE_DT_INST_DEFINE(0, max30102_int, NULL,                              
                             &max30102_data,                                 
                             &max30102_config, 
                             POST_KERNEL,                  
                             CONFIG_SENSOR_INIT_PRIORITY, 
                             &max30102_driver_api);