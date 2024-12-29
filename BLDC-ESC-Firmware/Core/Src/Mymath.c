/*
 * Mymath.c
 *
 *  Created on: Jun 29, 2024
 *      Author: Leon
 */
#include "main.h"
#include "Mymath.h"
#include "math.h"

/**
  * @brief Re-maps a number from one range to another
  * @param x 		= The number to map.
  * @param in_min	= The lower bound of the value’s current range.
  * @param in_max	= The upper bound of the value’s current range.
  * @param out_min	= The lower bound of the value’s target range.
  * @param out_max	= The upper bound of the value’s target range.
  * @retval			= The mapped value
  */
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
  * @brief Converts a raw ADC value to the corresponding input voltage
  * @param val    = Raw 12-bit ADC value (0-4095)
  * @retval float = Calculated input voltage
  */
float adc_volt(uint16_t val){
	//((val/(39000+2200))*2200) /12bitADC
    float Vcc = 3.3;
    float R1 = 37000.0;
    float R2 = 2200.0;
    // Convert ADC value to voltage across R2
    float Vout = (val / 4095.0) * Vcc;

    // Calculate the total voltage across R1 and R2
    float Vin = Vout * (R1 + R2) / R2;

    return Vin;
}

/**
  * @brief Converts a raw ADC value to the corresponding current based on shunt resistor and amplification factor
  * @param val    = Raw 12-bit ADC value (0-4095)
  * @retval float = Calculated current in amperes
  */
float adc_cur(uint16_t val){
    float Vcc = 3.3;
    float amplification_factor = 20.0;
    float R_shunt = 0.004;
    float offset_voltage = Vcc / 2.0;

    // Convert ADC value to the amplified voltage
    float Vadc = (val / 4095.0) * Vcc;

    // Adjust for the offset voltage
    float Vadc_adjusted = Vadc - offset_voltage;

    // Determine the actual voltage drop across the shunt
    float Vshunt = Vadc_adjusted / amplification_factor;

    // Calculate the current through the shunt
    float current = Vshunt / R_shunt;

    return current;
}

/**
  * @brief Converts a raw ADC value to the corresponding temperature in Celsius using an NTC thermistor
  * @param val    = Raw 12-bit ADC value (0-4095)
  * @retval float = Calculated temperature in Celsius
  */
float adc_temp(uint16_t val){
    float Vcc = 3.3;
    float R2 = 10000.0;
    float T0 = 298.15; // 25°C in Kelvin
    float R0 = 10000.0; // Resistance at 25°C
    float B = 2904.0; // Beta parameter

    // Convert ADC value to voltage
    float Vadc = (val / 4095.0) * Vcc;

    // Calculate the resistance of the NTC thermistor
    float R1 = R2 * (Vcc / Vadc - 1.0);

    // Calculate temperature in Kelvin using B-parameter equation
    float T = 1.0 / ((1.0 / T0) + (1.0 / B) * log(R1 / R0));

    // Convert Kelvin to Celsius
    float T_Celsius = T - 273.15;

    return T_Celsius;
}

/**
  * @brief Converts motor RPM to speed in kilometers per hour (km/h) based on wheel circumference
  * @param rpm    = Motor RPM (revolutions per minute)
  * @retval float = Calculated speed in kilometers per hour (km/h)
  */
float rpm_tokmh(float rpm){
	//36inch wheel
	//91,44 cm wheel
	//U = 2·π·r
	//U = 287,267cm
	//U = 2,87267m
	float circumference = 2.87267;
    // distance traveled per minute in meters
    float distance_per_minute = rpm * circumference;
    // Convert distance to kilometers per hour
    float speed_kmh = distance_per_minute * 60 / 1000;
    return speed_kmh;
}

/*
 * @brief This function initializes the data in a PID structure.
 * @param ptr A pointer to the PID data structure
 * @param min The manipulated variable's minimum value
 * @param max The manipulated variable's maximum value
 */
void pid_init_f(pid_f_t * ptr ,float min ,float max ){
  ptr->min = min;
  ptr->max = max;
  ptr->e   = 0;
  ptr->i   = 1;
  ptr->kp  = 5;
  ptr->ki  = 3;
  ptr->kd  = 2;
}

/**
  * @brief Computes the PID control output based on setpoint, process variable, and PID parameters
  * @param ptr    = Pointer to a pid_f_t structure containing PID parameters and state variables
  * @param sp     = Setpoint value for the PID controller
  * @param pv     = Process variable (current system value)
  * @retval uint16_t = Computed PID control output, bounded by specified limits
  */
uint16_t apply_pid(pid_f_t* ptr, int sp, float pv){
	  //float temp;
	  //float p;
	  float e;
	  float manp;
	  float tmpi;

	  e = ptr->e;
	  ptr->e = sp - pv;
	  tmpi = ptr->i + ptr->e;
	  //bound the integral
	  manp = (ptr->kp * ptr->e) + (ptr->ki * tmpi) + (ptr->kd * (e - ptr->e));
	  if ( (manp < ptr->max)&&(manp > ptr->min) ){
	    ptr->i = tmpi;
	  } else if ( manp > ptr->max ){
	    manp = ptr->max;
	    ptr->i = 1;
	  } else if ( manp < ptr->min ){
	    manp = ptr->min;
	  }
	  return manp;
}

