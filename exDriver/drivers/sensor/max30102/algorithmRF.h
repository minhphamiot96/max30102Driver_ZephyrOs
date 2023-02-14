#ifndef ALGORITHM_BY_RF_H_
#define ALGORITHM_BY_RF_H_

#include <stdio.h>
#include <stdint.h>
#include <zephyr/sys/ring_buffer.h>

void rf_heart_rate_and_oxygen_saturation (struct ring_buf * const ringBufRed, 
                                          struct ring_buf * const ringBufIr, 
                                          float *pn_spo2, 
                                          int8_t *pch_spo2_valid, 
                                          int32_t *pn_heart_rate, 
                                          int8_t *pch_hr_valid, 
                                          float *ratio, 
                                          float *correl);
float rf_linear_regression_beta(float *pn_x, float xmean, float sum_x2);
float rf_autocorrelation(float *pn_x, int32_t n_size, int32_t n_lag);
float rf_rms(float *pn_x, int32_t n_size, float *sumsq);
float rf_Pcorrelation(float *pn_x, float *pn_y, int32_t n_size);
void rf_initialize_periodicity_search(float *pn_x, int32_t n_size, int32_t *p_last_periodicity, int32_t n_max_distance, float min_aut_ratio, float aut_lag0);
void rf_signal_periodicity(float *pn_x, int32_t n_size, int32_t *p_last_periodicity, int32_t n_min_distance, int32_t n_max_distance, float min_aut_ratio, float aut_lag0, float *ratio);

#endif /* ALGORITHM_BY_RF_H_ */
