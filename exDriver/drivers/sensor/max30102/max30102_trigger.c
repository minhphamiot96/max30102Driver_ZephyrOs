#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "max30102.h"
#include "algorithmRF.h"
#include "algorithm.h"

LOG_MODULE_DECLARE (max30102, CONFIG_SENSOR_LOG_LEVEL);

static K_KERNEL_STACK_DEFINE(max30102_thread_stack, MAX30102_THREAD_STACK_SIZE);
static struct k_thread max30102_thread;
static uint8_t rawDataBuff[MAX30102_MAX_BYTES_FIFO] = {0U};

static void max30102_thread_main (void * ptr);

/** Interrupt function callback. */
static void max30102_irqHandler (const struct device * dev, struct gpio_callback *cb, uint32_t pins)
{
   struct max30102_data * data = &max30102_data;

   LOG_DBG ("Start function interrupt of max30102 sensor!");

	// gpio_pin_interrupt_configure_dt (&max30102_config.int_gpio, GPIO_INT_EDGE_TO_ACTIVE);

   k_sem_give (&max30102_data.sem);
}

/** The function initializes the interrupt for sensor.*/
int max30102_setup_interrupt (const struct device * dev)
{
   struct max30102_data const * data = dev->data;
   const struct max30102_config * cfg = dev->config;

   /** Set up interrupt when FIFO on sensor full. */
   if (i2c_reg_write_byte_dt (&cfg->i2c, MAX30102_REG_INT_EN1, cfg->irq1.R))
   {
      LOG_ERR ("Can not configurate the interrupt!");
      return -EIO;
   }
   if (i2c_reg_write_byte_dt (&cfg->i2c, MAX30102_REG_INT_EN2, cfg->irq2.R))
   {
      LOG_ERR ("Can not configurate the interrupt!");
      return -EIO;
   }

   /** Init semaphore used for this sensor. */
   k_sem_init (&data->sem, 0, K_SEM_MAX_LIMIT);

   /** Create the thread for this sensor. */
   k_thread_create ( &max30102_thread, 
                     max30102_thread_stack, 
                     MAX30102_THREAD_STACK_SIZE, 
                     (k_thread_entry_t) max30102_thread_main, 
                     (void *)dev, NULL, NULL,
                     K_PRIO_COOP(MAX30102_THREAD_PRIORITY), 0, K_NO_WAIT);
   
   if (!device_is_ready (cfg->int_gpio.port))
   {
      LOG_ERR ("Interrupt gpio device is not ready!");
      return -ENODEV;
   }

   /** Initialize the gpio interrupt and set the callback function. */
   gpio_pin_configure_dt (&cfg->int_gpio, GPIO_INPUT);
   gpio_init_callback (&data->intIrq, max30102_irqHandler, BIT(cfg->int_gpio.pin));
   gpio_add_callback (cfg->int_gpio.port, &data->intIrq);

   /** Setting irq event at the edge of interrupt pin change to low. */
	gpio_pin_interrupt_configure_dt (&cfg->int_gpio, GPIO_INT_EDGE_FALLING);
}

/** The thread used to handle data from max30102 sensor. */
static void max30102_thread_main (void * ptr)
{
   struct device * dev = (struct device *)ptr;

   struct max30102_data * data = dev->data;
   const struct max30102_config * cfg = dev->config;
   uint8_t retI2c = 0U;
   uint8_t readPtr = 0u;
   uint8_t writePtr = 0u;
   uint8_t noItems = 0u;
   uint8_t k = 0u;
   uint8_t bits = 0u;
   uint8_t count = 0u;
   uint32_t rawRed = 0u;
   uint32_t rawIR = 0u;
   bool isFifoFull = false;
   int ret = 0;
   float spo2Value = 0.0f;
   int8_t spo2Valid = 0;
   int32_t heartRate = 0;
   int8_t hearRateValid = 0;
   float ratio = 0;
   float correl = 0;
   bool isBufferSampleFull = false;

   /** Determine whether i2c bus ready. */
   if (!device_is_ready (cfg->i2c.bus))
   {
      LOG_ERR ("Bus device is not ready!");
      return -ENODEV;
   }

   /** Sitting in while loop */
   while (true) 
   {
      /* Waiting interrupt event. */
      k_sem_take (&data->sem, K_FOREVER);

      /** Determine the interrupt1 event. */
      ret = i2c_reg_read_byte_dt (&cfg->i2c, MAX30102_REG_INT_STS1, &retI2c);
      if (0U != (retI2c & (1u << MAX30102_INTERRUPT_STATUS_FIFO_FULL)))
      {
         /** FIFO full. */
         isFifoFull = true;
      }
      
      /** Read data from fifo in sensor. */
      if (isFifoFull)
      {
         /** Read overflow counter */
         if (i2c_reg_read_byte_dt(&cfg->i2c, MAX30102_REG_FIFO_OVF, &retI2c))
         {
            LOG_ERR("Read overflow counter failed!");
            return -EIO;
         }
         else
         {
            LOG_DBG("Max30102: Fifo overrun: %d", retI2c);
         }

         /** Read fifo read addr. */
         if (i2c_reg_read_byte_dt(&cfg->i2c, MAX30102_REG_FIFO_RD, &readPtr))
         {
            LOG_ERR("Max30102: Read fifo read point failed!");
            return -EIO;
         }

         /** Read fifo write point. */
         if (i2c_reg_read_byte_dt(&cfg->i2c, MAX30102_REG_FIFO_WR, &writePtr))
         {
            LOG_ERR("Max30102: Read fifo write point failed!");
            return -EIO;
         }

         /** Check the size of buffer in sensor. */
         if (writePtr > readPtr)
         {
            noItems = writePtr - readPtr;
         }
         else
         {
            noItems = 32u + writePtr - readPtr;
         }

         /** Determine the size of each sample in internal FIFO of sensor. */
         switch (cfg->mode.B.mode)
         {
         case MAX30102_MODE_HEART_RATE:
            k = 3;
            break;
         case MAX30102_MODE_SPO2:
            k = 6;
            break;
         case MAX30102_MODE_MULTI_LED:
            k = 6;
            break;
         default:
            LOG_ERR("Max30102: Mode is invalid!");
            break;
         }

         /** Read fifo. */
         if (i2c_burst_read_dt(&cfg->i2c, MAX30102_REG_FIFO_DATA, &rawDataBuff[0U], noItems * k))
         {
            LOG_ERR("Max30102: Read fifo data register failed!");
            ret = -EIO;
         }
         else
         {
            /** Shift bit. */
            if (MAX30102_PW_15BITS == cfg->spo2.B.ledPw)
            {
               bits = 3u;
            }
            else if (MAX30102_PW_16BITS == cfg->spo2.B.ledPw)
            {
               bits = 2u;
            }
            else if (MAX30102_PW_17BITS == cfg->spo2.B.ledPw)
            {
               bits = 1u;
            }
            else
            {
               bits = 0u;
            }

            /** TODO: Comment */
            for (count = 0u; count < noItems; count++)
            {
               if (MAX30102_MODE_HEART_RATE == cfg->mode.B.mode)
               {
                  rawRed = (rawDataBuff[count * 3u + 0u] << 16u) |
                           (rawDataBuff[count * 3u + 1u] << 8u) |
                           (rawDataBuff[count * 3u + 2u] << 0u);
                  rawRed = rawRed >> bits;

                  /** Update the Red value to Ring buffer of Red led. */
                  ring_buf_put(&data->rawRedRb, &rawRed, sizeof(rawRed));
               }
               else
               {
                  rawRed = (rawDataBuff[count * 6u + 0u] << 16u) |
                           (rawDataBuff[count * 6u + 1u] << 8u) |
                           (rawDataBuff[count * 6u + 2u] << 0u);
                  rawRed = rawRed >> bits;

                  rawIR = (rawDataBuff[count * 6u + 3u] << 16u) |
                          (rawDataBuff[count * 6u + 4u] << 8u) |
                          (rawDataBuff[count * 6u + 5u] << 0u);
                  rawIR = rawIR >> bits;

                  /** Update the Red value to Ring buffer of Red led and IR led. */
                  ring_buf_put (&data->rawRedRb, (uint8_t *)&rawRed, sizeof(rawRed));
                  ring_buf_put (&data->rawIRRb, (uint8_t *)&rawIR, sizeof(rawIR));

                  /** Check the size of ring buffer of Red and Ir led. */
                  if ((0u == ring_buf_space_get (&data->rawIRRb)) || (0u == ring_buf_space_get (&data->rawRedRb)))
                  {
                     /** Set flag identfy the buffer is full. */
                     isBufferSampleFull = true;

                     /** Break the loop. */
                     break;
                  }
               }
            }
         }

         /** Clear flag which identify the fifo full. */
         isFifoFull = false;   
      }

      /** Check if the buffer sample full, calculate the SPO2 and HeartBeat value. */
      if (true == isBufferSampleFull)
      {
         /** Clear flag. */
         isBufferSampleFull = false;

         /** Calculate the SPO2 and HeartBeat. */
         maxim_heart_rate_and_oxygen_saturation (&data->rawRedRb,
                                                 &data->rawIRRb,
                                                 &spo2Value,
                                                 &spo2Valid,
                                                 &heartRate,
                                                 &hearRateValid);
         // rf_heart_rate_and_oxygen_saturation(&data->rawRedRb,
         //                                     &data->rawIRRb,
         //                                     &spo2Value,
         //                                     &spo2Valid,
         //                                     &heartRate,
         //                                     &hearRateValid,
         //                                     &ratio,
         //                                     &correl);

         /** Reset ring buffer */
         ring_buf_reset(&data->rawIRRb);
         ring_buf_reset(&data->rawRedRb);

         if (true == spo2Valid)
         {
            printk ("SpO2: %f\n", spo2Value);
            spo2Valid = false;
         }

         if (true == hearRateValid)
         {
            printk ("Pulse: %d\n", heartRate);
            hearRateValid = false;
         }
      }
   }
}