/*
 * _clockGenerate.c
 *
 *  Created on: Jul 22, 2023
 *      Author: Supervisor
 */

#include "main.h"


extern TIM_HandleTypeDef htim2;

uint8_t clock_1ms = 0;
uint8_t clock_50ms = 0;
uint8_t clock_100ms = 0;

static count = 0;
uint8_t step_cnt = 0;

HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *tim){

	if(tim->Instance == TIM2){

		//step time for stepper
		if (step_cnt > 0)step_cnt--;
		if (step_cnt == 0){
			step_cnt = 5;
		}

	}

}


