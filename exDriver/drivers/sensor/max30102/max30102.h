/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MAXIM_MAX30102_H_
#define ZEPHYR_DRIVERS_SENSOR_MAXIM_MAX30102_H_

#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/**
 * @brief max30102 bool enumeration definition
 */
typedef enum 
{
   MAX30102_BOOL_FALSE = 0x00,      /**< False  */
   MAX30102_BOOL_TRUE = 0x01,       /**< True   */
} max30102_bool_t;

/**
 * @brief max30102 sample averaging enumeration definition
 */
typedef enum
{
   MAX30102_SAMPLE_NO_AVERAGING = 0x00,               /**< No Sample averaging       */
   MAX30102_SAMPLE_AVERAGING_2  = 0x01,               /**< Sample Averaging is 2     */
   MAX30102_SAMPLE_AVERAGING_4  = 0x02,               /**< Sample Averaging is 4     */
   MAX30102_SAMPLE_AVERAGING_8  = 0x03,               /**< Sample Averaging is 8     */
   MAX30102_SAMPLE_AVERAGING_16 = 0x04,               /**< Sample Averaging is 16    */
   MAX30102_SAMPLE_AVERAGING_32 = 0x05,               /**< Sample Averaging is 32    */
} max30102_sample_averaging_t;

/**
 * @brief max30102 interrupt status enumeration definition
 */
typedef enum
{
   MAX30102_INTERRUPT_STATUS_FIFO_FULL    = 7,        /**< Fifo almost full flag                */
   MAX30102_INTERRUPT_STATUS_PPG_RDY      = 6,        /**< New fifo data ready                  */ 
   MAX30102_INTERRUPT_STATUS_ALC_OVF      = 5,        /**< Ambient light cancellation overflow  */
   MAX30102_INTERRUPT_STATUS_PWR_RDY      = 0,        /**< Power ready flag                     */
   MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY = 1,        /**< Internal temperature ready flag      */
} max30102_interrupt_status_t;

/**
 * @brief max30102 interrupt enumeration definition
 */
typedef enum
{
   MAX30102_INTERRUPT_FIFO_FULL_EN    = 7,            /**< Fifo almost full enable                    */
   MAX30102_INTERRUPT_PPG_RDY_EN      = 6,            /**< New fifo data ready enable                 */
   MAX30102_INTERRUPT_ALC_OVF_EN      = 5,            /**< Ambient light cancellation overflow enable */
   MAX30102_INTERRUPT_DIE_TEMP_RDY_EN = 1,            /**< Internal temperature enable                */
} max30102_interrupt_t;

/**
 * @brief max30102 spo2 adc range enumeration definition
 */
typedef enum
{
   MAX30102_SPO2_ADC_RANGE_2048  = 0,                 /**< Range 2048   */
   MAX30102_SPO2_ADC_RANGE_4096  = 1,                 /**< Range 4096   */
   MAX30102_SPO2_ADC_RANGE_8192  = 2,                 /**< Range 8192   */
   MAX30102_SPO2_ADC_RANGE_16384 = 3,                 /**< Range 16384  */
} max30102_spo2_adc_range_t;

/**
 * @brief max30102 spo2 sample rate enumeration definition
 */
typedef enum
{
   MAX30102_SPO2_SAMPLE_RATE_50_HZ   = 0,             /**< 50Hz   */
   MAX30102_SPO2_SAMPLE_RATE_100_HZ  = 1,             /**< 100Hz  */
   MAX30102_SPO2_SAMPLE_RATE_200_HZ  = 2,             /**< 200Hz  */
   MAX30102_SPO2_SAMPLE_RATE_400_HZ  = 3,             /**< 400Hz  */
   MAX30102_SPO2_SAMPLE_RATE_800_HZ  = 4,             /**< 800Hz  */
   MAX30102_SPO2_SAMPLE_RATE_1000_HZ = 5,             /**< 1000Hz */
   MAX30102_SPO2_SAMPLE_RATE_1600_HZ = 6,             /**< 1600Hz */
   MAX30102_SPO2_SAMPLE_RATE_3200_HZ = 7,             /**< 3200Hz */
} max30102_spo2_sample_rate_t;

/**
 * @brief max30102 adc resolution enumeration definition
 */
typedef enum
{
   MAX30102_ADC_RESOLUTION_15_BIT = 0,                /**< 15 bits */
   MAX30102_ADC_RESOLUTION_16_BIT = 1,                /**< 16 bits */
   MAX30102_ADC_RESOLUTION_17_BIT = 2,                /**< 17 bits */
   MAX30102_ADC_RESOLUTION_18_BIT = 3,                /**< 18 bits */
} max30102_adc_resolution_t;

/** 
 * @brief max30102 spo2 configuration
 */
typedef union 
{
   uint8_t R; 
   struct {
      uint8_t reverse : 1;                            /**< Bit reverse.                                  */
      max30102_spo2_adc_range_t adcRange : 2;         /**< SpO2 ADC Range Control. (18 bit resolution)   */
      max30102_spo2_sample_rate_t sampleRate : 3;     /**< SpO2 Sample Rate Control.                     */
      max30102_adc_resolution_t  ledPw : 2;           /**< LED Pulse Width Control.                      */
   } B;
} max30102_spo2_config_t;
/**
 * @brief max30102 led enumeration definition
 */
typedef enum
{
   MAX30102_LED_NONE = 0,           /**< Time slot is disabled */
   MAX30102_LED_RED  = 1,           /**< Enable red            */
   MAX30102_LED_IR   = 2,           /**< Enable ir             */
} max30102_led_t;

/**
 * @brief max30102 slot enumeration definition
 */
typedef enum
{
   MAX30102_SLOT_1 = 0,             /**< Slot 1 */
   MAX30102_SLOT_2 = 1,             /**< Slot 2 */
   MAX30102_SLOT_3 = 2,             /**< Slot 3 */
   MAX30102_SLOT_4 = 3,             /**< Slot 4 */
} max30102_slot_t;

/**
 * @brief max30102 mode enumeration definition
 */
typedef enum
{
   MAX30102_MODE_HEART_RATE = 0x02, /**< heart rate mode */
   MAX30102_MODE_SPO2       = 0x03, /**< spo2 mode */
   MAX30102_MODE_MULTI_LED  = 0x07, /**< multi-led mode */
} max30102_mode_t;

/** 
 * @brief max30102 mode configuration
 */
typedef union
{
   uint8_t R;
   struct {
      bool SHDN : 1;                /**< Shutdown control. */
      bool RESET : 1;               /**< Reset control. */
      uint8_t reverse : 3;          /**< Bits reverse. */
      max30102_mode_t mode : 3;     /**< Mode control. */
   } B;
} max30102_mode_config_t;

/**
 * @brief max30102 fifo enumeration definition
 */
typedef union
{
   uint8_t R;
   struct {
      max30102_sample_averaging_t smpAve : 3;         /**< Fifo configure: sample                  */
      bool enRollOver : 1;                            /**< Fifo configure: enable fifo rollover.   */
      uint8_t fifoAlmostFull : 4;                     /**< Fifo configure: threshold of fifo.      */
   } B;
} max30102_fifo_config_t;

/**
 * @brief max30102 information structure definition
 */
typedef struct
{
   char chip_name[32];              /**< Chip name                      */
   char manufacturer_name[32];      /**< Manufacturer name              */
   char interface[8];               /**< Chip interface name            */
   float supply_voltage_min_v;      /**< Chip min supply voltage        */
   float supply_voltage_max_v;      /**< Chip max supply voltage        */
   float max_current_ma;            /**< Chip max current               */
   float temperature_min;           /**< Chip min operating temperature */
   float temperature_max;           /**< Chip max operating temperature */
   uint32_t driver_version;         /**< Driver version                 */
} max30102_info_t;

/** 
 * @brief         Configuration data structures.
 */
struct max30102_config {
   const struct i2c_dt_spec i2c;
   max30102_fifo_config_t fifo;
   max30102_mode_config_t mode;
   max30102_slot_t slot[4];
   max30102_spo2_config_t spo2;
   uint8_t ledPa[2];
};

/** 
 * @brief         Data structures of this sensor. 
 */
struct max30102_data {
   uint32_t raw[32];
   uint8_t map[32];
   max30102_led_t numChannel;
};

/**
 * @brief         Get chip's information
 * @param[out]    *info points to a max30102 info structure
 * @return        status code
 *                - 0 success
 *                - 2 handle is NULL
 * @note          none
 */
static uint8_t max30102_info (const struct device *dev, max30102_info_t *info);

/**
 * @brief         Initialize the chip
 * @param[in]     *handle points to a max30102 handle structure
 * @return        status code
 *                - 0 success
 *                - 1 iic initialization failed
 *                - 2 handle is NULL
 *                - 3 linked functions is NULL
 *                - 4 id is invalid
 *                - 5 reset failed
 *                - 6 reset fifo failed
 * @note          none
 */
static int max30102_int (const struct device *dev);

/**
 * @brief         Close the chip
 * @param[in]     *handle points to a max30102 handle structure
 * @return        status code
 *                - 0 success
 *                - 1 iic deinit failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 *                - 4 power down failed
 * @note          none
 */
static uint8_t max30102_deinit (const struct device *dev);

/**
 * @brief         read the data
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *raw_red points to a red raw data buffer
 * @param[out]    *raw_ir points to an ir raw data buffer
 * @param[in,out] *len points to a length buffer
 * @return        status code
 *                - 0 success
 *                - 1 read failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 *                - 4 fifo overrun
 *                - 5 mode is invalid
 * @note          none
 */
static uint8_t max30102_read (const struct device *dev, uint32_t *raw_red, uint32_t *raw_ir, uint8_t *len);

/**
 * @brief         Read the temperature
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *raw points to a raw data buffer
 * @param[out]    *temp points to a converted temperature buffer
 * @return        Status code:
 *                - 0 success
 *                - 1 read temperature failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_read_temperature (const struct device *dev, uint16_t *raw, float *temp);

/**
 * @brief         irq handler
 * @param[in]     *handle points to a max30102 handle structure
 * @return        status code
 *                - 0 success
 *                - 1 run failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_irq_handler (const struct device *dev);

/**
 * @brief         Get the interrupt status
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     status is the interrupt status
 * @param[out]    *enable points to a bool value buffer
 * @return        status code
 *                - 0 success
 *                - 1 get interrupt status failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_interrupt_status (  const struct device *dev, 
                                                max30102_interrupt_status_t status, 
                                                max30102_bool_t *enable);

/**
 * @brief         Set the interrupt bool
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     type is the interrupt type
 * @param[in]     enable is a bool value
 * @return        status code
 *                - 0 success
 *                - 1 set interrupt failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_interrupt (const struct device *dev, max30102_interrupt_t type, max30102_bool_t enable);

/**
 * @brief         Get the interrupt bool
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     type is the interrupt type
 * @param[out]    *enable points to a bool value buffer
 * @return        status code
 *                - 0 success
 *                - 1 get interrupt failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_interrupt (const struct device *dev, max30102_interrupt_t type, max30102_bool_t *enable);

/**
 * @brief         Set the fifo write pointer
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     pointer is the written pointer
 * @return        status code
 *                - 0 success
 *                - 1 set fifo write pointer failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 *                - 4 pointer can't be over 0x1F
 * @note          pointer <= 0x1F
 */
static uint8_t max30102_set_fifo_write_pointer (const struct device *dev, uint8_t pointer);

/**
 * @brief         Get the fifo write pointer
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *pointer points to a pointer buffer
 * @return        status code
 *                - 0 success
 *                - 1 get fifo write pointer failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          nonw
 */
static uint8_t max30102_get_fifo_write_pointer (const struct device *dev, uint8_t *pointer);

/**
 * @brief         Set the fifo overflow counter
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     counter is the overflow counter
 * @return        status code
 *                - 0 success
 *                - 1 set fifo overflow counter failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 *                - 4 counter can't be over 0x1F
 * @note          counter <= 0x1F
 */
static uint8_t max30102_set_fifo_overflow_counter (const struct device *dev, uint8_t counter);

/**
 * @brief         Get the fifo overflow counter
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *counter points to a counter buffer
 * @return        status code
 *                - 0 success
 *                - 1 get fifo overflow counter failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          nonw
 */
static uint8_t max30102_get_fifo_overflow_counter (const struct device *dev, uint8_t *counter);

/**
 * @brief         Set the fifo read pointer
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     pointer is the read pointer
 * @return        status code
 *                - 0 success
 *                - 1 set fifo read pointer failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 *                - 4 pointer can't be over 0x1F
 * @note          pointer <= 0x1F
 */
static uint8_t max30102_set_fifo_read_pointer (const struct device *dev, uint8_t pointer);

/**
 * @brief         Get the fifo read pointer
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *pointer points to a pointer buffer
 * @return        status code
 *                - 0 success
 *                - 1 get fifo read pointer failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_fifo_read_pointer (const struct device *dev, uint8_t *pointer);

/**
 * @brief         Set the fifo data
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     data is the fifo data
 * @return        status code
 *                - 0 success
 *                - 1 set fifo data failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_fifo_data (const struct device *dev, uint8_t data);

/**
 * @brief         Get the fifo data
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *data points to a fifo data buffer
 * @return        status code
 *                - 0 success
 *                - 1 get fifo data failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_fifo_data (const struct device *dev, uint8_t *data);

/**
 * @brief         set the fifo sample averaging
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     sample is the fifo sample averaging
 * @return        status code
 *                - 0 success
 *                - 1 set fifo sample averaging failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_fifo_sample_averaging (const struct device *dev, max30102_sample_averaging_t sample);

/**
 * @brief         Get the fifo sample averaging
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *sample points to a fifo sample averaging buffer
 * @return        status code
 *                - 0 success
 *                - 1 get fifo sample averaging failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_fifo_sample_averaging (const struct device *dev, max30102_sample_averaging_t *sample);

/**
 * @brief         Enable or disable the fifo roll 
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     enable is a bool value
 * @return        status code
 *                - 0 success
 *                - 1 set fifo roll failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_fifo_roll (const struct device *dev, max30102_bool_t enable);

/**
 * @brief         Get the fifo roll status
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *enable points to a bool value buffer
 * @return        status code
 *                - 0 success
 *                - 1 get fifo roll failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_fifo_roll (const struct device *dev, max30102_bool_t *enable);

/**
 * @brief         Set the fifo almost full value
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     value is the fifo almost full value
 * @return        status code
 *                - 0 success
 *                - 1 set fifo almost full failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 *                - 4 value can't be over 0xF
 * @note          none
 */
static uint8_t max30102_set_fifo_almost_full (const struct device *dev, uint8_t value);

/**
 * @brief         Get the fifo almost full value
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *value points to a fifo almost full value buffer
 * @return        status code
 *                - 0 success
 *                - 1 get fifo almost full failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_fifo_almost_full (const struct device *dev, uint8_t *value);

/**
 * @brief         Set the shutdown
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     enable is a bool value
 * @return        status code
 *                - 0 success
 *                - 1 set shutdown failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_shutdown (const struct device *dev, max30102_bool_t enable);

/**
 * @brief         Get the shutdown
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *enable points to a bool value buffer
 * @return        status code
 *                - 0 success
 *                - 1 get shutdown failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_shutdown (const struct device *dev, max30102_bool_t *enable);

/**
 * @brief         Reset the chip
 * @param[in]     *handle points to a max30102 handle structure
 * @return        status code
 *                - 0 success
 *                - 1 reset failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_reset (const struct device *dev);

/**
 * @brief         Set the mode
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     mode is the chip mode
 * @return        status code
 *                - 0 success
 *                - 1 set mode failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_mode (const struct device *dev, max30102_mode_t mode);

/**
 * @brief         Get the mode
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *mode points to a chip mode buffer
 * @return        status code
 *                - 0 success
 *                - 1 get mode failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_mode (const struct device *dev, max30102_mode_t *mode);

/**
 * @brief         Set the spo2 adc range
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     range is the spo2 adc range
 * @return        status code
 *                - 0 success
 *                - 1 set spo2 adc range failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_spo2_adc_range (const struct device *dev, max30102_spo2_adc_range_t range);

/**
 * @brief         Get the spo2 adc range
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *range points to an spo2 adc range buffer
 * @return        status code
 *                - 0 success
 *                - 1 get spo2 adc range failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_spo2_adc_range (const struct device *dev, max30102_spo2_adc_range_t *range);

/**
 * @brief         Set the spo2 sample rate
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     rate is the spo2 sample rate
 * @return        status code
 *                - 0 success
 *                - 1 set spo2 sample rate failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_spo2_sample_rate (const struct device *dev, max30102_spo2_sample_rate_t rate);

/**
 * @brief         Get the spo2 sample rate
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *rate points to an spo2 sample rate buffer
 * @return        status code
 *                - 0 success
 *                - 1 get spo2 sample rate failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_spo2_sample_rate (const struct device *dev, max30102_spo2_sample_rate_t *rate);

/**
 * @brief         Set the adc resolution
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     resolution is the adc resolution
 * @return        status code
 *                - 0 success
 *                - 1 set adc resolution failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_adc_resolution (const struct device *dev, max30102_adc_resolution_t resolution);

/**
 * @brief         Get the adc resolution
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *resolution points to an adc resolution buffer
 * @return        status code
 *                - 0 success
 *                - 1 get adc resolution failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_adc_resolution (const struct device *dev, max30102_adc_resolution_t *resolution);

/**
 * @brief         Set the red led pulse amplitude
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     amp is the red led pulse amplitude
 * @return        status code
 *                - 0 success
 *                - 1 set led red pulse amplitude failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_led_red_pulse_amplitude (const struct device *dev, uint8_t amp);

/**
 * @brief         Get the red led pulse amplitude
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *amp points to a red led pulse amplitude buffer
 * @return        status code
 *                - 0 success
 *                - 1 get led red pulse amplitude failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_led_red_pulse_amplitude (const struct device *dev, uint8_t *amp);

/**
 * @brief         Set the ir led pulse amplitude
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     amp is the ir led pulse amplitude
 * @return        status code
 *                - 0 success
 *                - 1 set led ir pulse amplitude failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_led_ir_pulse_amplitude (const struct device *dev, uint8_t amp);

/**
 * @brief         Get the ir led pulse amplitude
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *amp points to an ir led pulse amplitude buffer
 * @return        status code
 *                - 0 success
 *                - 1 get led ir pulse amplitude failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_led_ir_pulse_amplitude (const struct device *dev, uint8_t *amp);

/**
 * @brief         Set the led slot
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     slot is the slot number
 * @param[in]     led is the led mode
 * @return        status code
 *                - 0 success
 *                - 1 set slot failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_slot (const struct device *dev, max30102_slot_t slot, max30102_led_t led);

/**
 * @brief         Get the led slot
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     slot is the slot number
 * @param[out]    *led points to a led mode buffer
 * @return        status code
 *                - 0 success
 *                - 1 get slot failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_slot (const struct device *dev, max30102_slot_t slot, max30102_led_t *led);

/**
 * @brief         Enable or disable die temperature
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     enable is a bool value
 * @return        status code
 *                - 0 success
 *                - 1 set die temperature failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_die_temperature (const struct device *dev, max30102_bool_t enable);

/**
 * @brief         Get the die temperature status
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *enable points to a bool value buffer
 * @return        status code
 *                - 0 success
 *                - 1 get die temperature failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_die_temperature (const struct device *dev, max30102_bool_t *enable);

/**
 * @brief         Get the chip id
 * @param[in]     *handle points to a max30102 handle structure
 * @param[out]    *revision_id points to a revision id buffer
 * @param[out]    *part_id points to a part id buffer
 * @return        status code
 *                - 0 success
 *                - 1 get id failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_id (const struct device *dev, uint8_t *revision_id, uint8_t *part_id);

/**
 * @brief         Set the chip register
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     reg is the iic register address
 * @param[in]     *buf points to a data buffer
 * @param[in]     len is the data buffer length
 * @return        status code
 *                - 0 success
 *                - 1 write failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_set_reg (const struct device *dev, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief         Get the chip register
 * @param[in]     *handle points to a max30102 handle structure
 * @param[in]     reg is the iic register address
 * @param[out]    *buf points to a data buffer
 * @param[in]     len is the data buffer length
 * @return        status code
 *                - 0 success
 *                - 1 read failed
 *                - 2 handle is NULL
 *                - 3 handle is not initialized
 * @note          none
 */
static uint8_t max30102_get_reg (const struct device *dev, uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* ZEPHYR_DRIVERS_SENSOR_MAXIM_MAX30102_H_ */