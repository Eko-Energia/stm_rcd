#include "app.h"
#include "type2.h"
#include "can_driver.h"
#include "adc_driver.h"
#include "pwm_driver.h"
#include "led_driver.h"
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
// incoming CAN message FIFO (capacity 5)
#define CAN_INCOMING_CAPACITY 5
uint8_t CAN_receiveFlag = 0;

typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
} CAN_IncomingMsg_t;

static CAN_IncomingMsg_t CAN_incomingBuffer[CAN_INCOMING_CAPACITY];
static uint8_t CAN_incoming_head = 0; // next write index
static uint8_t CAN_incoming_tail = 0; // next read index
static uint8_t CAN_incoming_count = 0;

static struct CAN_scheduledMsgList CAN_buffer;

HAL_StatusTypeDef CAN_addIncomingMessage(CAN_RxHeaderTypeDef *header, uint8_t *data) {
	if (CAN_incoming_count >= CAN_INCOMING_CAPACITY) {
		// TODO error handler
		return HAL_ERROR;
	}

	// copy header and data
	CAN_incomingBuffer[CAN_incoming_head].header = *header;
	for (int i = 0; i < 8; i++) {
		CAN_incomingBuffer[CAN_incoming_head].data[i] = data[i];
	}

	CAN_incoming_head = (CAN_incoming_head + 1) % CAN_INCOMING_CAPACITY;
	CAN_incoming_count++;
	return HAL_OK;
}

HAL_StatusTypeDef CAN_getLatestMessage(CAN_IncomingMsg_t *msg) {
	if (CAN_incoming_count == 0) {
		return HAL_ERROR;
	}

	// Get message from tail (oldest message in FIFO)
	*msg = CAN_incomingBuffer[CAN_incoming_tail];
	CAN_incoming_tail = (CAN_incoming_tail + 1) % CAN_INCOMING_CAPACITY;
	CAN_incoming_count--;
	return HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t data[8];
	// read the message from FIFO0
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) == HAL_OK) {
		// try to enqueue, set flag if successful
		if (CAN_addIncomingMessage(&rxHeader, data) == HAL_OK) {
			CAN_receiveFlag = 1;
		}
	}
}

void CAN_passIncoming()
{
	CAN_IncomingMsg_t incMsg;

	if(CAN_getLatestMessage(&incMsg) != HAL_OK)
	{
		return;
	}

	CAN_TxHeaderTypeDef txMsg;
	txMsg.StdId = incMsg.header.StdId;
	txMsg.ExtId = incMsg.header.ExtId;
	txMsg.DLC = incMsg.header.DLC;
	txMsg.IDE = incMsg.header.IDE;
	txMsg.RTR = incMsg.header.RTR;

	HAL_CAN_AddTxMessage(&hcan, &txMsg, incMsg.data, &CAN_buffer.txMailbox);
}

/*
 * PWM
 */
static struct PWM_IC_signal PWM_sig;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == CP_PWM_CHANNEL) {
		PWM_IC_update(&PWM_sig, &htim1);
	}
}

// used to
typedef enum {
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

void app_main() {
	CAN_init(&hcan);
	//ADC_Init(&hadc1, &ADC_buffer, &ADC_channels);
	HAL_ADC_Start(&hadc1);
	PWM_IC_Init(&PWM_sig, &htim1, 1000, 1);

	struct LED GREEN_LED = { LED_OFF, LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0 };
	struct LED RED_LED = { LED_OFF, LED_RED_GPIO_Port, LED_RED_Pin, 0 };
	struct LED Type2_GREEN_LED = { LED_OFF, TYPE2_LED_GREEN_GPIO_Port,
			TYPE2_LED_GREEN_Pin, 0 };
	struct LED Type2_RED_LED = { LED_OFF, TYPE2_LED_RED_GPIO_Port,
			TYPE2_LED_RED_Pin, 0 };

	LED_ChangeState(&GREEN_LED, LED_BLINK);

	// switch relays off
	// TODO off for testing purposes RCD_FAULT is set to input (no clicking)
	//HAL_GPIO_WritePin(RCD_FAULT_GPIO_Port, RCD_FAULT_Pin, GPIO_PIN_RESET);

	uint32_t raw_adc_value = 0;
	float PP_voltage = 0.0f;
	const float VREF = 3.28f;

	while (1)
	{
		updateTransoptorVoltage();

	// --- 5-Sample Trimmed Mean ADC Reading ---
			uint32_t adc_samples[5];
			uint32_t sum = 0;
			uint32_t min_val = 0xFFFFFFFF; // Max possible uint32 value
			uint32_t max_val = 0;

			for (int i = 0; i < 5; i++) {
				// Wait for the conversion to finish (10ms timeout)
				if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
				{
					adc_samples[i] = HAL_ADC_GetValue(&hadc1);
				}
				else
				{
					adc_samples[i] = raw_adc_value; // Fallback to last known good value if timeout occurs
				}

				// Add to total sum
				sum += adc_samples[i];

				// Find highest and lowest values
				if (adc_samples[i] < min_val) min_val = adc_samples[i];
				if (adc_samples[i] > max_val) max_val = adc_samples[i];
			}

			// Eliminate the lowest and highest values, average the remaining 3
			raw_adc_value = (sum - min_val - max_val) / 3;

			// Calculate the final stabilized voltage
			PP_voltage = ((float)raw_adc_value * VREF) / 4095.0f;
			// -----------------------------------------

		//TODO add safety checks for each state
		// Type2_state informs what should be happening
		switch (Type2_state)
		{
			case Type2_DISCONNECTED:

				//TODO olac jezeli jedzie
				if (PP_voltage < PP_VOLTAGE_DISCONNECTED)
					{
						Type2_state = Type2_IDLE;
						LED_ChangeState(&Type2_RED_LED, LED_ON);
						HAL_GPIO_WritePin(RCD_FAULT_GPIO_Port, RCD_FAULT_Pin, GPIO_PIN_SET);
					}
				break;
			case Type2_IDLE:
				maxChargerCurrent = Type2_MaxChargerCurrent(PP_voltage,PWM_sig.duty,0,0);

				if (maxChargerCurrent > 0)
				{
					startCharging();
					Type2_state = Type2_CHARGING;
					LED_ChangeState(&Type2_GREEN_LED, LED_BLINK);
					LED_ChangeState(&Type2_RED_LED, LED_ON);
				}
				else if (PP_voltage > PP_VOLTAGE_DISCONNECTED)
				{
					Type2_state = Type2_DISCONNECTED;
					LED_ChangeState(&Type2_RED_LED, LED_OFF);
					HAL_GPIO_WritePin(RCD_FAULT_GPIO_Port, RCD_FAULT_Pin,
							GPIO_PIN_RESET);
				}
				break;
			case Type2_CHARGING:
				maxChargerCurrent = Type2_MaxChargerCurrent(PP_voltage, PWM_sig.duty,0,0);
				//TODO stop charging
				if (maxChargerCurrent <= 0)
				{
					stopCharging();
					Type2_state = Type2_IDLE;
					LED_ChangeState(&Type2_RED_LED, LED_ON);
					LED_ChangeState(&Type2_GREEN_LED, LED_OFF);
				}
				break;
		}


		PWM_IC_Monitor(&PWM_sig, CP_GPIO_Port, CP_Pin);
		LED_Handle(&RED_LED);
		LED_Handle(&GREEN_LED);
		LED_Handle(&Type2_GREEN_LED);
		LED_Handle(&Type2_RED_LED);
		CAN_handleScheduled(&hcan, &CAN_buffer);
	}
}

/*
 * Private function definitions
 */
static void startCharging() {
	HAL_GPIO_WritePin(START_CHARGING_GPIO_Port, START_CHARGING_Pin,
			GPIO_PIN_SET);

	struct CAN_scheduledMsg chargerComms;
	chargerComms.header.DLC = 5;
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
}

static void stopCharging() {
	HAL_GPIO_WritePin(START_CHARGING_GPIO_Port, START_CHARGING_Pin,
			GPIO_PIN_RESET);

	CAN_removeScheduledMessage(CANID_RCD_STATIC_CHARGER1COMMS, &CAN_buffer);
	CAN_removeScheduledMessage(CANID_RCD_STATIC_CHARGER2COMMS, &CAN_buffer);
	CAN_removeScheduledMessage(CANID_RCD_STATIC_CHARGER3COMMS, &CAN_buffer);
}

/*
 * @brief Detects voltage on phases
 */
static void updateTransoptorVoltage() {
	if (HAL_GPIO_ReadPin(L1_SENSOR_GPIO_Port, L1_SENSOR_Pin) == GPIO_PIN_SET) {
		phaseStatus |= PHASE1_SET;
	} else {
		phaseStatus &= PHASE1_RESET;
	}

	if (HAL_GPIO_ReadPin(L2_SENSOR_GPIO_Port, L2_SENSOR_Pin) == GPIO_PIN_SET) {
		phaseStatus |= PHASE2_SET;
	} else {
		phaseStatus &= PHASE2_RESET;
	}

	if (HAL_GPIO_ReadPin(L3_SENSOR_GPIO_Port, L3_SENSOR_Pin) == GPIO_PIN_SET) {
		phaseStatus |= PHASE3_SET;
	} else {
		phaseStatus &= PHASE3_RESET;
	}
}

static void chargerGetData(uint8_t *data, void *context) {
	// preserve one decimal place
	float maxCurrent = maxChargerCurrent * 10;
	float maxVoltage = MAX_CHARGER_VOLTAGE * 10;

	uint16_t maxChargerCurrentInt = (uint16_t) maxCurrent;
	uint16_t maxVoltageInt = (uint16_t) maxVoltage;

	// syntax error is probably an IDE bug
	// charger frame requires big endian
	maxVoltageInt = SWAP_ENDIANNESS(maxVoltageInt);
	maxChargerCurrentInt = SWAP_ENDIANNESS(maxChargerCurrentInt);
	data[0] = GET_BYTE(maxVoltageInt, 0);
	data[1] = GET_BYTE(maxVoltageInt, 1);
	data[2] = GET_BYTE(maxChargerCurrentInt, 0);
	data[3] = GET_BYTE(maxChargerCurrentInt, 1);

	// charging is requested whenever this function is called;
	uint8_t control = 0;
	data[4] = SWAP_ENDIANNESS(control);
}

