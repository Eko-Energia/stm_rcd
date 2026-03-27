/*
 *
 *
 */

#ifndef __APP_H__
#define __APP_H__

#include "main.h"

/*
 * GENERAL
 */
void app_main(void);

/*
 * CAN
 */


/*
 * PP
 */
#define PP_ADC_CHANNEL ADC_CHANNEL_11


/*
 * CP
 */
#define CP_PWM_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1


/*
 * RCD
 */
#define RCD_PWM_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1

#endif /* __APP_H_ */
