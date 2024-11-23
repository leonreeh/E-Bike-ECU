/*
 * BLDC.h
 *
 *  Created on: Jun 29, 2024
 *      Author: Leon
 *      Functions related to moving and stoping the BLDC motor
 */

#ifndef INC_BLDC_H_
#define INC_BLDC_H_


uint16_t hallState(uint16_t hall[]);


void commutator(int commutatorStep, int duty, int dir);

void initBLDC();

void BLDCbreak();
#endif /* INC_BLDC_H_ */
