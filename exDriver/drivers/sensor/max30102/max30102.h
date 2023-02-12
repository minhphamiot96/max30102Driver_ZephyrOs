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
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/mutex.h>

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
#define MAX30102_MAX_BYTES_FIFO (MAX30102_MAX_BYTES_PER_SAMPLE * \
                                 MAX30102_NO_OF_ITEM)
#define MAX30102_SLOT_LED_MASK (0x03U)
#define MAX30102_FIFO_DATA_BITS (18U)
#define MAX30102_FIFO_DATA_MASK ((1U << MAX30102_FIFO_DATA_BITS) - 1U)

#define MAX30102_PART_ID (0x15U)

#define MAX30102_THREAD_PRIORITY (1U)
#define MAX30102_THREAD_STACK_SIZE (2048U)

/** 
 * @brief The number of item in buffer used in this device driver.
 */
#define MAX30102_NO_OF_ITEM (32U)

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

// /**
//  * @brief max30102 interrupt enumeration definition
//  */
// typedef enum
// {
//    MAX30102_INTERRUPT_FIFO_FULL_EN    = 7,            /**< Fifo almost full enable                    */
//    MAX30102_INTERRUPT_PPG_RDY_EN      = 6,            /**< New fifo data ready enable                 */
//    MAX30102_INTERRUPT_ALC_OVF_EN      = 5,            /**< Ambient light cancellation overflow enable */
//    MAX30102_INTERRUPT_DIE_TEMP_RDY_EN = 1,            /**< Internal temperature enable                */
// } max30102_interrupt_t;

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
 * @brief max30102 LED pulse width control.
 */
typedef enum
{
   MAX30102_PW_15BITS = 0,
   MAX30102_PW_16BITS,
   MAX30102_PW_17BITS,
   MAX30102_PW_18BITS,
} max30102_pw_control_t;

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
      max30102_pw_control_t  ledPw : 2;               /**< LED Pulse Width Control.                      */
      max30102_spo2_sample_rate_t sampleRate : 3;     /**< SpO2 Sample Rate Control.                     */
      max30102_spo2_adc_range_t adcRange : 2;         /**< SpO2 ADC Range Control. (18 bit resolution)   */
      uint8_t reverse : 1;                            /**< Bit reverse.                                  */
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
      max30102_mode_t mode : 3;     /**< Mode control. */
      uint8_t reverse : 3;          /**< Bits reverse. */
      bool RESET : 1;               /**< Reset control. */
      bool SHDN : 1;                /**< Shutdown control. */
   } B;
} max30102_mode_config_t;

/**
 * @brief max30102 fifo enumeration definition
 */
typedef union
{
   uint8_t R;
   struct {
      uint8_t fifoAlmostFull : 4;                     /**< Fifo configure: threshold of fifo.      */
      bool enRollOver : 1;                            /**< Fifo configure: enable fifo rollover.   */
      max30102_sample_averaging_t smpAve : 3;         /**< Fifo configure: sample                  */
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
 * @brief max30102 interrupt1 configration. 
 */
typedef union 
{
   uint8_t R;
   struct {
      uint8_t reserve      : 5;
      bool alc_ovf_en      : 1;
      bool ppg_rdy_en      : 1;
      bool a_full_en       : 1;
   } B;
} max30102_interrupt1_t;

/**
 * @brief max30102 interrupt2 configration. 
 */
typedef union 
{
   uint8_t R;
   struct {
      uint8_t reserve1     : 1;
      bool die_temp_rdy_en : 1;
      uint8_t reserve2     : 6;
   } B;
} max30102_interrupt2_t;

/** 
 * @brief         Configuration data structures.
 */
struct max30102_config {
   struct i2c_dt_spec i2c;
   max30102_interrupt1_t irq1;
   max30102_interrupt2_t irq2;
   max30102_fifo_config_t fifo;
   max30102_mode_config_t mode;
   max30102_slot_t slot[4];
   max30102_spo2_config_t spo2;
   uint8_t ledPa[2];
   struct gpio_dt_spec int_gpio;
};

/** 
 * @brief         Data structures of this sensor. 
 */
struct max30102_data {
   struct k_sem sem;
   struct gpio_callback intIrq;
   float temporature;
   struct ring_buf rawRedRb;
   uint32_t rawRed[MAX30102_NO_OF_ITEM];
   struct ring_buf rawIRRb;
   uint32_t rawIR[MAX30102_NO_OF_ITEM];
};

/** Max30102: Data of this sensor. */
extern struct max30102_data max30102_data;

/** Max30102: Configuration of this sensor. */
extern const struct max30102_config max30102_config;

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
int max30102_setup_interrupt (const struct device * dev);

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
static int max30102_deinit (const struct device *dev);

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
static int max30102_read_temperature (const struct device *dev, uint8_t * raw, float *temp);

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

#endif /* ZEPHYR_DRIVERS_SENSOR_MAXIM_MAX30102_H_ */