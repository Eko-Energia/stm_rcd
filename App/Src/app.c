#include "app.h"
#include "type2.h"
#include "can_driver.h"
#include "adc_driver.h"
#include "pwm_driver.h"
#include "main.h"

/*
 * Private defines
 */
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

/*
 * Private variables
 */
static float maxChargerCurrent = 0;
static Type2_StateTypeDef Type2_state = Type2_DISCONNECTED;

/*
 * CAN
 */
static struct CAN_scheduledMsgList CAN_buffer;

/*
* ADC
*/
static ADC_BufferTypeDef ADC_buffer;
static ADC_ChannelsTypeDef ADC_channels;

/*
* PWM
*/
static struct PWM_signal PWM_sig;

// used to
typedef enum
{
	PHASE_UNPOWERED = 0x0,
	PHASE1_SET = 0x1,
	PHASE2_SET = 0x2,
	PHASE3_SET = 0x4,
	PHASE1_RESET = ~PHASE1_SET,
	PHASE2_RESET = ~PHASE2_SET,
	PHASE3_RESET = ~PHASE3_SET
} phaseStatus_e;

static phaseStatus_e phaseStatus = PHASE_UNPOWERED;

/*
 * Private function prototypes
 */
static void updateTransoptorVoltage();
static void startCharging();
static void stopCharging();
static void chargerGetData(uint8_t *data, void *context);

void app_main()
{
	CAN_init(&hcan);
	ADC_Init(&hadc1, &ADC_buffer, &ADC_channels);
	PWM_initialize(&PWM_sig, 1000, 1, &htim1);

	uint32_t x;
	x = ADC_buffer.idma.BufferADC[0];

	uint32_t startedCharging = 0;
	float PP_voltage = 0;

	while(1)
	{	
		updateTransoptorVoltage();
		//TODO add safety checks for each state
		// Type2_state informs what should be happening
		switch(Type2_state)
		{
		case Type2_DISCONNECTED:
			//TODO olac jezeli jedzie
			ADC_GetValue(&hadc1, &ADC_channels, &ADC_buffer, MAX_PP_VOLTAGE, PP_ADC_CHANNEL, &PP_voltage);
			if(PP_voltage > 0)
			{
				Type2_state = Type2_IDLE;
			}
			break;
		case Type2_IDLE:
			ADC_GetValue(&hadc1, &ADC_channels, &ADC_buffer, MAX_PP_VOLTAGE, PP_ADC_CHANNEL, &PP_voltage);
			PWM_update(&htim1, &PWM_sig, 1);
			maxChargerCurrent = Type2_MaxChargerCurrent(PP_voltage, PWM_sig.PWM_width);

			if(maxChargerCurrent > 0)
			{
				startCharging();
				startedCharging = HAL_GetTick();
			}
			else if (PP_voltage > 2.0f)
			{
				Type2_state = Type2_DISCONNECTED;
			}
			//TODO startcharging
			break;
		case Type2_CHARGING:
			//TODO stop charging
			if(maxChargerCurrent <= 0 || HAL_GetTick() - startedCharging > 1000)
			{
				stopCharging();
			}
			break;
		}

		CAN_handleScheduled(&hcan, &CAN_buffer);
	}
}

/*
 * Private function definitions
 */
static void startCharging()
{
	HAL_GPIO_WritePin(START_CHARGING_GPIO_Port, START_CHARGING_Pin, GPIO_PIN_SET);

	struct CAN_scheduledMsg chargerComms;
	chargerComms.header.DLC = 3;
	chargerComms.header.IDE = CAN_ID_EXT;
	chargerComms.header.RTR = CAN_RTR_DATA;
	chargerComms.lastTick = 0;
	chargerComms.periodMs = 1000;
	chargerComms.getData = chargerGetData;

	chargerComms.header.ExtId = CANID_RCD_STATIC_CHARGER1COMMS;
	CAN_addScheduledMessage(chargerComms, &CAN_buffer);

	chargerComms.header.ExtId = CANID_RCD_STATIC_CHARGER2COMMS;
	CAN_addScheduledMessage(chargerComms, &CAN_buffer);

	chargerComms.header.ExtId = CANID_RCD_STATIC_CHARGER3COMMS;
	CAN_addScheduledMessage(chargerComms, &CAN_buffer);

	Type2_state = Type2_CHARGING;
}

static void stopCharging()
{
	HAL_GPIO_WritePin(START_CHARGING_GPIO_Port, START_CHARGING_Pin, GPIO_PIN_RESET);

	CAN_removeScheduledMessage(CANID_RCD_STATIC_CHARGER1COMMS, &CAN_buffer);
	Type2_state = Type2_IDLE;
}

/*
 * @brief Detects voltage on phases
 */
static void updateTransoptorVoltage()
{
	if(HAL_GPIO_ReadPin(L1_SENSOR_GPIO_Port, L1_SENSOR_Pin) == GPIO_PIN_SET)
	{
		phaseStatus |= PHASE1_SET;
	}
	else
	{
		phaseStatus &= PHASE1_RESET;
	}

	if(HAL_GPIO_ReadPin(L2_SENSOR_GPIO_Port, L2_SENSOR_Pin) == GPIO_PIN_SET)
	{
		phaseStatus |= PHASE2_SET;
	}
	else
	{
		phaseStatus &= PHASE2_RESET;
	}

	if(HAL_GPIO_ReadPin(L3_SENSOR_GPIO_Port, L3_SENSOR_Pin) == GPIO_PIN_SET)
	{
		phaseStatus |= PHASE3_SET;
	}
	else
	{
		phaseStatus &= PHASE3_RESET;
	}
}

static void chargerGetData(uint8_t *data, void *context)
{
	// preserve one decimal place
	float maxCurrent = maxChargerCurrent * 10;
	float maxVoltage = MAX_CHARGER_VOLTAGE * 10;

	uint16_t maxChargerCurrentInt = (uint16_t) maxCurrent;
	uint16_t maxVoltageInt = (uint16_t) maxVoltage;
	uint8_t control = 1;

	// TODO test
	// syntax error is probably an IDE bug
	// charger frame requires big endian
	maxVoltageInt = SWAP_ENDIANNESS(maxVoltageInt);
	maxChargerCurrentInt = SWAP_ENDIANNESS(maxChargerCurrentInt);
	data[0] = GET_BYTE(maxVoltageInt, 0);
	data[1] = GET_BYTE(maxVoltageInt, 1);
	data[2] = GET_BYTE(maxChargerCurrentInt, 0);
	data[3] = GET_BYTE(maxChargerCurrentInt, 1);
	// charging is requested whenever this function is called;
	data[4] = SWAP_ENDIANNESS(control);
}


