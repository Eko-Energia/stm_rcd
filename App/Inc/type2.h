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
#define TYPE2_MAX_VOLTAGE 	87 //(V)
#define TYPE2_MAX_CURRENT 	16 //(A)

/*
 * TYPE 2
 */
typedef enum {
	Type2_DISCONNECTED = 0,
	Type2_IDLE = 1, // connected, but not charging
	Type2_CHARGING = 2,
	Type2_FAULT = 3
} Type2_StateTypeDef;

extern Type2_StateTypeDef Type2_state;

void Type2_Process(float PP_voltage, float CP_duty);


// to jest do wyrzucenia jak będzie gotowy adc_driver
/*
 * @brief  raw value from adc
 * @params hadc ADC instance
 * @params ADC_Channel pin identification
 * @retval raw value of ADC_Channel
 */
uint32_t Read_ADC(ADC_HandleTypeDef* hadc, uint32_t ADC_Channel);

/*
 * PP
 */
typedef struct {
	float minVoltage;
	float maxVoltage;
	int maxCableCurrent;
} PP_CurrentRange;

uint8_t PP_GetMaxCurrent(float voltage);

/*
 * CP
 */
float CP_DutyToCurrent(float PWM_duty);

#endif /* INC_TYPE2_H_ */
