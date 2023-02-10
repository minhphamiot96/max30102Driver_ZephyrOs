#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "max30102.h"

LOG_MODULE_DECLARE (max30102, CONFIG_SENSOR_LOG_LEVEL);

static K_KERNEL_STACK_DEFINE(max30102_thread_stack, MAX30102_THREAD_STACK_SIZE);
static struct k_thread max30102_thread;

static void max30102_thread_main (struct max30102_data * data);


/** Interrupt function callback. */
static void max30102_irqHandler (const struct device * dev, struct gpio_callback *cb, uint32_t pins)
{
   struct max30102_data * data = dev->data;
   struct max30102_config * cfg = dev->config;

   k_sem_give (&data->sem);
}

/** The function initializes the interrupt for sensor.*/
int max30102_setup_interrupt (const struct device * dev)
{
   struct max30102_data const * data = dev->data;
   const struct max30102_config * cfg = dev->config;

   /** Set up interrupt when FIFO on sensor full. */
   if (i2c_burst_write_dt (&cfg->i2c, MAX30102_REG_INT_EN1, (uint8_t *)&cfg->irq1.R, sizeof(cfg->irq1.R)))
   {
      LOG_ERR ("Can not configurate the interrupt!");
      return -EIO;
   }
   if (i2c_burst_write_dt (&cfg->i2c, MAX30102_REG_INT_EN2, (uint8_t *)&cfg->irq2.R, sizeof(cfg->irq2.R)))
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
                     data, NULL, NULL,
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
   unsigned int flags = GPIO_INT_EDGE_TO_ACTIVE;
	gpio_pin_interrupt_configure_dt (&cfg->int_gpio, flags);
}

/** The thread used to handle data from max30102 sensor. */
static void max30102_thread_main (struct max30102_data * data)
{
   /** Intialize the param used for this thread. */
   uint32_t counter = 0u;

   /** Sitting in while loop */
   while (true) 
   {
      k_sem_take (&data->sem, K_FOREVER);

      /** FIXME: Processing handle the data. */
      LOG_DBG ("Thread run counter: %d", counter);
   }
}