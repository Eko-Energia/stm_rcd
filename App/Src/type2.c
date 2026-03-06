/**
  ******************************************************************************
  * @file           : type2.c
  * @brief          : Type 2 connector handling code
  * THIS CODE *SHOULD* REMAIN HARDWARE INDEPENDENT
  ******************************************************************************
  * @author AGH Eko-Energia
  *	@author Kacper Lasota
  ******************************************************************************
  */
#include "type2.h"

/*
 * Private variables
 */

/*
 * Private functions prototypes
 */
static float CP_GetMaxCurrent(float PWM_Duty);
static uint8_t PP_GetMaxCurrent(float voltage);

// hardcoded so it doesn't need to be calculated each time
#define SQRT_3 1.73205080757

float Type2_MaxChargerCurrent(float PP_voltage, float CP_duty)
{
	const float CP_maxCurrent = CP_GetMaxCurrent(CP_duty);
	const float PP_maxCurrent = PP_GetMaxCurrent(PP_voltage);

	const float maxCurrent = CP_maxCurrent > PP_maxCurrent ?
			PP_maxCurrent : CP_maxCurrent;

	// calculate maxCurrent @ 87V
	// hardcoded sqrt(3) bo na chuj ma się ciągle liczyć
	float maxChargerCurrent = (float) MAX_TYPE2_CURRENT * maxCurrent / (float) MAX_TYPE2_VOLTAGE * SQRT_3;

	// divided for 3 chargers
	maxChargerCurrent = maxChargerCurrent / 3.0;
	maxChargerCurrent = maxChargerCurrent > MAX_CHARGER_CURRENT ?
			MAX_CHARGER_CURRENT : maxChargerCurrent;

	return maxChargerCurrent;
}

/*
 * PP connector
 */
struct PP_CurrentRange{
	const float minVoltage;
	const float maxVoltage;
	const uint16_t maxCableCurrent;
};

/*
 * Contains voltage ranges that represent different cable width
 * https://www.smart-emotion.de/wiki/entry/325-pp/?synonym=443
 */
static const struct PP_CurrentRange currentRange[] = {
	{0.157f, 0.3f,   63},
	{0.3f,   0.595f, 32},
	{0.595f, 1.32f,  20},
	{1.32f,  2.0f,   13}
};

/*
 * @brief  Return cable current capabilty of the connected charger
 * @detail Voltage measured between PP and PE gives information about max vable current capability
 * @params voltage voltage on PP pin of the type 2 connector
 * @retval Max current in AMPS, -1 if not in range
 */
static uint8_t PP_GetMaxCurrent(float voltage)
{
	if (voltage < currentRange[0].minVoltage)
	{
		return 0;
	}

	int maxIndex = (int)(sizeof(currentRange)/sizeof(currentRange[0]));

	// return current that corresponds to the voltage
	for(int i = 0; i < maxIndex; i++)
	{
		if (voltage <= currentRange[i].maxVoltage)
		{
			return currentRange[i].maxCableCurrent;
		}
	}

	// charger connected, voltage not in range
	return -1;
}


/*
 * CP connector
 */

/*
 * assigned from
 * https://www.smart-emotion.de/wiki/entry/321-typ-2-charging-protocol-using-the-cp-contact/?synonym=420#module30www
 */
static float CP_GetMaxCurrent(float PWM_Duty)
{
	if(PWM_Duty < 8)
	{
		return 0;
	}
	else if (PWM_Duty >= 8 && PWM_Duty < 10)
	{
		return 6;
	}
	else if (PWM_Duty >= 10 && PWM_Duty < 85)
	{
		return PWM_Duty * 0.6;
	}
	else if (PWM_Duty >= 85 && PWM_Duty < 96)
	{
		return (PWM_Duty - 64) * 2.5;
	}
	else if (PWM_Duty >= 96 && PWM_Duty < 97)
	{
		return 80;
	}
	else
	{
		return 0;
	}
}

/*
 * Utils
 */


