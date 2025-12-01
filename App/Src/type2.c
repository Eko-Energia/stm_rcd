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
 * GENERAL
 */
Type2_StateTypeDef Type2_state = Type2_DISCONNECTED;

/*
 * Private variables
 */
static float Type2_maxCurrent = 0;

void Type2_Process(float PP_voltage, float CP_duty)
{
	if(Type2_state != Type2_DISCONNECTED && CP_duty < 3)
	{
		Type2_state = Type2_DISCONNECTED;
		return;
	}

	switch(Type2_state)
	{
	case Type2_DISCONNECTED:
		// tutaj można sprawdzać, czy jest 9v
		if (CP_duty >= 3)
		{
			Type2_state = Type2_IDLE;
		}
		break;
	// connected, ready to charge
	case Type2_IDLE:
		UpdateMaxCurrent(PP_voltage, CP_duty);

		if(Type2_maxCurrent > 0)
		{
			Type2_state = Type2_CHARGING;
		}
		break;
	case Type2_CHARGING:
		UpdateMaxCurrent(PP_voltage, CP_duty);

		if(Type2_maxCurrent == 0)
		{
			Type2_IDLE;
		}
		break;
	default:
		// ErrorHandler("Type2 state unknown");
		return;
		break;
	}
}

static void UpdateMaxCurrent(float PP_voltage, float CP_duty)
{
	CP_maxCurrent = CP_DutyToCurrent(CP_duty);
	PP_maxCurrent = PP_GetMaxCurrent(PP_voltage);

	Type2_maxCurrent = CP_maxCurrent > PP_maxCurrent ?
			PP_maxCurrent : CP_maxCurrent;
}

/*
 * @brief  raw value from adc
 * @params hadc ADC instance
 * @params ADC_Channel pin identification
 * @retval raw value of ADC_Channel
 */
uint32_t Read_ADC(ADC_HandleTypeDef* hadc, uint32_t ADC_Channel)
{
//	ADC_ChannelConfTypeDef sConfig = {0};
//	sConfig.Channel = ADC_Channel;
//	sConfig.Rank = 1;
//	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
//
//	HAL_ADC_ConfigChannel(hadc, &sConfig);
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

	uint32_t value = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);

	return value;
}

/*
 * PP connector
 */

/*
 * Contains voltage ranges that represent different cable width
 * https://www.smart-emotion.de/wiki/entry/325-pp/?synonym=443
 */
static PP_CurrentRange currentRange[] = {
	{0.157f, 0.3f,   63},
	{0.3f,   0.595f, 32},
	{0.595f, 1.32f,  20},
	{1.32f,  2.0f,   13},
};



/*
 * @brief  Return cable current capabilty of the connected charger
 * @detail Voltage measured between PP and PE gives information about max vable current capability
 * @params voltage voltage on PP pin of the type 2 connector
 * @retval Max current in AMPS, -1 if not in range
 */
uint8_t PP_GetMaxCurrent(float voltage){
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

	// charger connected, but not
	return -1;
}


/*
 * CP connector
 */

/*
 * assigned from
 * https://www.smart-emotion.de/wiki/entry/321-typ-2-charging-protocol-using-the-cp-contact/?synonym=420#module30www
 */
float CP_DutyToCurrent(float PWM_Duty)
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
