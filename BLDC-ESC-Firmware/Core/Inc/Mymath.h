/*
 * Mymath.h
 *
 *  Created on: Jun 29, 2024
 *      Author: Leon
 *
 */

#ifndef INC_MYMATH_H_
#define INC_MYMATH_H_

typedef struct{
  float max /*! Max manipulated value */;
  float min /*! Miniumum manipulated value */;
  float e /*! Error value */;
  float i /*! Integrator value */;
  float kp /*! Proportional constant */;
  float ki /*! Integrator constant */;
  float kd /*! Differential constant */;
} pid_f_t;

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
float adc_volt(uint16_t val);
float adc_cur(uint16_t val);
float adc_temp(uint16_t val);
float rpm_tokmh(float rpm);
float voltageToSOC(float voltage);
void pid_init_f(pid_f_t * ptr ,float min ,float max );
uint16_t apply_pid(pid_f_t* ptr, int val, float ref);



#endif /* INC_MYMATH_H_ */
