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

void main(void)
{
   int ret = 0;
   struct sensor_value irValue;
   struct sensor_value redValue;

   const struct device *const dev = DEVICE_DT_GET_ANY(maxim_max30102);
	const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

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

   // ret = usb_enable(NULL);
	// if (ret != 0) {
	// 	return;
	// }

   while (1)
   {
      sensor_sample_fetch(dev);
      sensor_channel_get(dev, SENSOR_CHAN_IR, &irValue);
      sensor_channel_get(dev, SENSOR_CHAN_RED, &redValue);

      /* Print green LED data*/
      printf("IR value =%d\n", irValue.val1);
      printf("Red value =%d\n", redValue.val1);

      k_sleep(K_MSEC(20));
   }
}
