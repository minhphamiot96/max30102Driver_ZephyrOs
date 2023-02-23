/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>

const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(max30102_sensor));
const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

void main(void)
{
   struct sensor_value irValue;
   struct sensor_value redValue;
   struct sensor_value temp;
   float temp_real = 0.0f;

   if (dev == NULL)
   {
      printf("Could not get max30102 device\n");
      return;
   }

   if (!device_is_ready(dev))
   {
      printf("max30102 device %s is not ready\n", dev->name);
      return;
   }

   while (1)
   {
      // sensor_channel_get (dev, SENSOR_CHAN_DIE_TEMP, &temp);
      // temp_real = (float)((uint32_t)temp.val1) + ((float)((uint32_t)temp.val2) * 0.0625f);
      // printk ("Temporature = %f\n", temp_real);

      k_sleep(K_SECONDS(1U));
   }
}
