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
static float PP_voltage = PP_MAX_VOLTAGE;

/*
* ADC
*/
#define ADC_CHANNELS 1
#define ADC_SAMPLES 10
#define PP_ADC_CHANNEL 0
const float ADC_vRef = 3.3f;
const float ADC_resolution = 4096.0f;
uint16_t ADC_buffer[ADC_CHANNELS];

/*
 * CAN
 */
static struct CAN_IncomingMsgList CAN_RxBuffer = {0};
static struct CAN_scheduledMsgList CAN_buffer = {0};

/*
 * Unpacked CAN RX data
 */
typedef struct {
	float outputVoltage;
	float outputCurrent;
	uint8_t hardwareFailure;
	uint8_t overTemp;
	uint8_t inputVoltageErr;
	uint8_t startingState;
	uint8_t commTimeout;
} ChargerOutput_t;

typedef struct {
	float batteryVoltage;
	float batteryCurrent;
	float batteryTemp;
} BMSMasterData_t;

typedef struct {
	float absoluteEncoder;
	uint8_t prnd;
} DashboardControl_t;

static ChargerOutput_t charger1Output = {0};
static ChargerOutput_t charger2Output = {0};
static ChargerOutput_t charger3Output = {0};
static BMSMasterData_t bmsMasterData = {0};
static DashboardControl_t dashboardControl = {0};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
	{
		Error_Handler();
	}

	if(CAN_AddIncomingMsg(&CAN_RxBuffer, &header, data) != HAL_OK)
	{
		Error_Handler();
	}
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
static void unpackChargerOutput(const uint8_t *data, ChargerOutput_t *out);
static void handleRxCanMessages(void);
static float uint8_t ProcessADC1Data(void);

void app_main() {
	CAN_Init(&hcan);
	HAL_ADC_Start(&hadc1);
	PWM_IC_Init(&PWM_sig, &htim1, 1000, 1);

	struct LED GREEN_LED = { LED_OFF, LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0 };
	struct LED RED_LED = { LED_OFF, LED_RED_GPIO_Port, LED_RED_Pin, 0 };
	struct LED Type2_GREEN_LED = { LED_OFF, TYPE2_LED_GREEN_GPIO_Port, TYPE2_LED_GREEN_Pin, 0};
	struct LED Type2_RED_LED = { LED_OFF, TYPE2_LED_RED_GPIO_Port, TYPE2_LED_RED_Pin, 0};

	LED_ChangeState(&GREEN_LED, LED_BLINK);

	while (1)
	{
		updateTransoptorVoltage();
		PP_voltage = ADC_buffer[PP_ADC_CHANNEL];

		//TODO block charging when not in park dashboard_control

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
				maxChargerCurrent = Type2_MaxChargerCurrent(PP_voltage, PWM_sig.duty, bmsMasterData.batteryTemp);

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
				maxChargerCurrent = Type2_MaxChargerCurrent(PP_voltage, PWM_sig.duty, bmsMasterData.batteryTemp);
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

		handleRxCanMessages();
		PWM_IC_Monitor(&PWM_sig, CP_GPIO_Port, CP_Pin);
		LED_Handle(&RED_LED);
		LED_Handle(&GREEN_LED);
		LED_Handle(&Type2_GREEN_LED);
		LED_Handle(&Type2_RED_LED);
		CAN_HandleScheduled(&hcan, &CAN_buffer);
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
	CAN_AddScheduledMsg(&chargerComms, &CAN_buffer);

	chargerComms.header.ExtId = CANID_RCD_STATIC_CHARGER2COMMS;
	CAN_AddScheduledMsg(&chargerComms, &CAN_buffer);

	chargerComms.header.ExtId = CANID_RCD_STATIC_CHARGER3COMMS;
	CAN_AddScheduledMsg(&chargerComms, &CAN_buffer);
}

static void stopCharging() {
	HAL_GPIO_WritePin(START_CHARGING_GPIO_Port, START_CHARGING_Pin,
			GPIO_PIN_RESET);

	CAN_RemoveScheduledMsg(CANID_RCD_STATIC_CHARGER1COMMS, &CAN_buffer);
	CAN_RemoveScheduledMsg(CANID_RCD_STATIC_CHARGER2COMMS, &CAN_buffer);
	CAN_RemoveScheduledMsg(CANID_RCD_STATIC_CHARGER3COMMS, &CAN_buffer);
}

/*
 * @brief Detects voltage on phases
 */
static void updateTransoptorVoltage()
{
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

/*
 * @brief Updates PP_voltage
 */
 static uint8_t ProcessADC1Data(void)
 {
	 const float ADC_vRef = 3.3f; // Reference voltage
	 const float ADC_resolution = 4096.0f; // 12-bit ADC resolution
	 static uint8_t sampleIndex = 0;
	 static uint8_t samplesCollected = 0;
	 static uint16_t ADC_Samples[ADC_CHANNELS][ADC_SAMPLES] = {0};
	 uint16_t ADC_snapshot[ADC_CHANNELS] = {0};
 
	 __disable_irq(); // Disable interrupts to prevent race conditions while
	 // create a snapshot of the ADC values (mask to keep only 12 bits)
	 for(uint8_t channel = 0; channel < ADC_CHANNELS; channel++)
	 {
		 ADC_snapshot[channel] = ADC_buffer[channel] & 0x0FFFu;
	 }
	 __enable_irq(); // Re-enable interrupts after snapshot is taken
 
	 for (uint8_t channel = 0; channel < ADC_CHANNELS; channel++)
	 {
		 ADC_Samples[channel][sampleIndex] = ADC_snapshot[channel];
 
		 // Calculate the average of the samples for each channel (remove max and min for better accuracy)
		 uint32_t sum = 0;
		 uint16_t min = 0xFFFFu;
		 uint16_t max = 0;
		 
		 for (uint8_t i = 0; i < ADC_SAMPLES; i++)
		 {
			 uint16_t sample = ADC_Samples[channel][i];
			 sum += sample;
			 if (sample < min) min = sample;
			 if (sample > max) max = sample;
		 }
 
		 sum -= (min + max);
		 float average = (float) sum / (ADC_SAMPLES - 2); 
 
		 ADC_Voltage[channel] = average * (ADC_vRef/ ADC_resolution);
	 }
 
	 sampleIndex++;
	 if (sampleIndex >= ADC_SAMPLES)
	 {
		 sampleIndex = 0;
	 }
 
	 // usage not allowed without ADC_SAMPLES samples
	 if(samplesCollected < ADC_SAMPLES)
	 {
		 samplesCollected++;
		 return 0; // voltages cannot be updated yet, try again later
	 }
	 
	 return 1; // voltages can be updated now
 }

/*
 * CAN
 */
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

static void unpackChargerOutput(const uint8_t *data, ChargerOutput_t *out)
{
	uint16_t rawVoltage = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
	uint16_t rawCurrent = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
	out->outputVoltage = rawVoltage * 0.1f;
	out->outputCurrent = rawCurrent * 0.1f;
	out->hardwareFailure    = (data[4] >> 0) & 0x01;
	out->overTemp           = (data[4] >> 1) & 0x01;
	out->inputVoltageErr    = (data[4] >> 2) & 0x01;
	out->startingState      = (data[4] >> 3) & 0x01;
	out->commTimeout        = (data[4] >> 4) & 0x01;
}

static void handleRxCanMessages(void)
{
	if (!CAN_RxBuffer.receiveFlag) return;

	struct CAN_IncomingMsg msg;
	while (CAN_GetLatestMessage(&CAN_RxBuffer, &msg) == HAL_OK)
	{
		uint8_t *data = msg.data;

		if (msg.header.IDE == CAN_ID_STD)
		{
			switch (msg.header.StdId)
			{
				case CANID_BMSMASTER_MASTERVOLTCURRTEMP:
				{
					uint16_t rawVolt = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
					uint16_t rawCurr = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
					uint16_t rawTemp = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
					bmsMasterData.batteryVoltage = (rawVolt - 12.0f) * 0.0059f;
					bmsMasterData.batteryCurrent = (rawCurr - 300.0f) * 0.1465f;
					bmsMasterData.batteryTemp    = (rawTemp - 50.0f) * 0.024f;
					break;
				}
				case CANID_DASHBOARD_CONTROL:
				{
					uint16_t rawEncoder = ((uint16_t)data[0] | ((uint16_t)data[1] << 8)) & 0x3FFF;
					dashboardControl.absoluteEncoder = (rawEncoder - 540.) * 0.06591796875f;
					dashboardControl.prnd = ((data[1] >> 6) & 0x03) | ((data[2] & 0x3F) << 2);
					break;
				}
				default:
					break;
			}
		}
		else if (msg.header.IDE == CAN_ID_EXT)
		{
			switch (msg.header.ExtId)
			{
				case CANID_CHARGER1_STATIC_OUT:
					unpackChargerOutput(data, &charger1Output);
					break;
				case CANID_CHARGER2_STATIC_OUT:
					unpackChargerOutput(data, &charger2Output);
					break;
				case CANID_CHARGER3_STATIC_OUT:
					unpackChargerOutput(data, &charger3Output);
					break;
				default:
					break;
			}
		}
	}
}

