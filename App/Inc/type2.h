/**
  ******************************************************************************
  * @file           : type2.h
  * @brief          : Type 2 connector handling code
  * THIS CODE *SHOULD* REMAIN HARDWARE INDEPENDENT
  ******************************************************************************
  * @author AGH Eko-Energia
  *	@author Kacper Lasota
  ******************************************************************************
  */

#ifndef INC_TYPE2_H_
#define INC_TYPE2_H_

#include "adc.h"

/*
 * GENERAL
 */
/**
 * @brief The maximum current on one charger on 87V
 * max max max = 37A (just for reference)
 */
#define MAX_CHARGER_CURRENT (37)
#define MAX_CHARGER_VOLTAGE (87)
#define MAX_TYPE2_CURRENT (16)
#define MAX_TYPE2_VOLTAGE (400)
#define PP_MAX_VOLTAGE (3.3f)
#define PP_VOLTAGE_DISCONNECTED (2.2f)
#define CURVE_C (300)
/*
 * TYPE 2
 */
typedef enum {
	Type2_DISCONNECTED = 0,
	Type2_IDLE = 1, // connected, but not charging
	Type2_CHARGING = 2,
} Type2_StateTypeDef;

/*
 * Utils
 */

float Type2_MaxChargerCurrent(float PP_voltage, float CP_duty, float BMS_temp);

/*
 * CP
 */

#endif /* INC_TYPE2_H_ */
