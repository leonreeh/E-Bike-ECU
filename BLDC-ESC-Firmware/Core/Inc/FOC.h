/*
 * FOC.h
 *
 *  Created on: Oct 4, 2024
 *      Author: Leon
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_


#define POLE_PAIRS 24
#define SECTOR_ANGLE (360 / 6)  // Each Hall sensor state represents 60° mechanical
#define ELECTRICAL_CYCLES (POLE_PAIRS * SECTOR_ANGLE)


void moving();

#endif /* INC_FOC_H_ */
