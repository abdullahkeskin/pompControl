/*
 * processTask.c
 *
 *  Created on: 22 Tem 2023
 *      Author: Abdullah KESKİN
*/

#include "main.h"


extern IWDG_HandleTypeDef hiwdg;

extern RTC_HandleTypeDef hrtc;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

__IO uint8_t Req_steper_Flag = 0;
__IO uint8_t Req_Hartbeat_Flag = 0;
__IO uint8_t Req_Info_Flag = 0;
__IO uint8_t Req_Firmware_Flag = 0;
__IO uint8_t Req_Measurement_Flag = 0;
__IO uint8_t Req_AdcCalibrate_Flag = 0;
__IO uint8_t Req_ChainInit_Flag = 0;
__IO uint8_t Req_ChainStatus_Flag = 0;
__IO uint8_t Req_ChainItem_Flag = 0;
__IO uint8_t Req_ChainItemSetState_Flag = 0;
__IO uint8_t Req_Debug_Flag = 0;
__IO uint8_t Req_Bootloader_Flag = 0;
__IO uint8_t ChainInitOptions_Flag = 0;
__IO uint8_t Req_DebugParam_Flag = 0;
__IO uint8_t Req_DebugHeaderMeta_Flag = 0;
__IO uint8_t Req_DebugHeaderData_Flag = 0;
__IO uint8_t Req_HeadVoltage_Flag = 0;


/* @param
 * @retval
 * @brief  All interrupt signals are evaluated here. requests are returned.
 * 			for use, must be added interrupt signal flap.
 */
void process_requests(){


HAL_IWDG_Refresh(&hiwdg);

}
