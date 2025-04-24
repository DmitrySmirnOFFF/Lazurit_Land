/*
 * Hardware_Driver.c
 *
 *  Created on: Jan 24, 2024
 *      Author: Ivan
 */
#include "Hardware_Driver.h"


// Дискретные входы
enum_state TEST_IN_State;

enum_state Fault_H1_State;
enum_state Fault_H2_State;
enum_state Fault_H3_State;
enum_state Fault_H4_State;
enum_state Fault_H4_1_State;
enum_state Fault_H5_State;

enum_state Fault_L1_State;
enum_state Fault_L2_State;
enum_state Fault_L3_State;
enum_state Fault_L4_State;
enum_state Fault_L4_2_State;
enum_state Fault_L5_State;

enum_state PWR_Fault1_State;
enum_state PWR_Fault2_State;
enum_state PWR_Fault3_State;
enum_state PWR_Fault4_State;
enum_state PWR_Fault5_State;

// Функц кнопки
enum_state But_1_State;
enum_state But_2_State;
enum_state EN_OS_State;

// Дискретные выходы
enum_state DO_1_State;
enum_state DO_2_State;
enum_state EN_PWM_State;

enum_state LED_1_State;
uint32_t LED_1_Cnt;

//Аналогавые входы
uint16_t U_Value;
uint16_t I_Value;
uint16_t Trt_Value;
uint16_t T_RT1_Value;
uint16_t T_RT2_Value;
uint16_t T_RT3_Value;
uint16_t T_RT4_Value;
uint16_t T_RT5_Value;
uint16_t AI_8_Value;

// перемнные для ацп в стм, мгновенные значения
uint16_t Device_ADC_Buf[ADC_CHANNELS];

float U_Instant;
float U_ADC_ref = 0;
float U_ADC_koef = U_SENS_K;

uint32_t U_mean;
uint32_t U_mean_sum;
uint16_t U_mean_buf[MEAN_BUF_SIZE];

float I_Instant;
float I_ADC_ref = 1990;//2036
float I_ADC_koef = I_SENS_K / TURN_NUM;

float I_abs;

uint32_t I_mean;
uint32_t I_mean_sum;
uint16_t I_mean_buf[MEAN_BUF_SIZE];

HAL_StatusTypeDef Init_Error;

uint8_t Periph_Init() {

	// Запуск АЦП
	__HAL_ADC_ENABLE(&hadc2);
	__HAL_ADC_ENABLE(&hadc3);
	Init_Error = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) Device_ADC_Buf, ADC_CHANNELS);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TE);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_FE);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_DME);
	//HAL_Delay(ADC_DELAY);
	Init_Error = HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);

	//Настройка и запуск основного таймера
	Init_Error = HAL_TIM_Base_Start_IT(&htim1);

	// ?нициализация сдвинутых таймеров
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
	TIM1->CCR2 = TIM1->ARR;
	TIM1->CCR3 = TIM1->ARR;
	TIM1->CCR4 = TIM1->ARR;

	//Запуск вспомогательных таймеров
	Init_Error = HAL_TIM_Base_Start(&htim2);
	Init_Error = HAL_TIM_Base_Start(&htim3);
	Init_Error = HAL_TIM_Base_Start(&htim9);


	//Запуск таймера для жесткой коммутации
	//в нем же работаем с обсчетом регуляторов
	Init_Error = HAL_TIM_Base_Start_IT(&htim8);

	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC2);
	TIM8->CCR2 = TIM8->ARR;

	Init_Error = HAL_TIM_Base_Start(&htim12);

	if(Init_Error != HAL_OK){
		return -1;
	} else {
		return 0;
	}

}

void Periph_Deinit() {
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop(&htim4, TIM_CHANNEL_4);
}

void HW_Driver_DI_AI_Read() {

	// Считывание дискретных сигналов

	Fault_H1_State = !HAL_GPIO_ReadPin(Fault_H1_GPIO_Port, Fault_H1_Pin);
	Fault_H2_State = !HAL_GPIO_ReadPin(Fault_L2_GPIO_Port, Fault_L2_Pin);
	Fault_H3_State = !HAL_GPIO_ReadPin(Fault_H3_GPIO_Port, Fault_H3_Pin);
	Fault_H4_State = !HAL_GPIO_ReadPin(Fault_H4_GPIO_Port, Fault_H4_Pin);
	Fault_H4_1_State = !HAL_GPIO_ReadPin(Fault_H4_1_GPIO_Port, Fault_H4_1_Pin);
	Fault_H5_State = !HAL_GPIO_ReadPin(Fault_H5_GPIO_Port, Fault_H5_Pin);

	Fault_L1_State = !HAL_GPIO_ReadPin(Fault_L1_GPIO_Port, Fault_L1_Pin);
	Fault_L2_State = !HAL_GPIO_ReadPin(Fault_L2_GPIO_Port, Fault_L2_Pin);
	Fault_L3_State = !HAL_GPIO_ReadPin(Fault_L3_GPIO_Port, Fault_L3_Pin);
	Fault_L4_State = !HAL_GPIO_ReadPin(Fault_L4_GPIO_Port, Fault_L4_Pin);
	Fault_L4_2_State = !HAL_GPIO_ReadPin(Fault_L4_2_GPIO_Port, Fault_L4_2_Pin);
	Fault_L5_State = !HAL_GPIO_ReadPin(Fault_L5_GPIO_Port, Fault_L5_Pin);

	PWR_Fault1_State = HAL_GPIO_ReadPin(PWR_Fault1_GPIO_Port, PWR_Fault1_Pin);
	PWR_Fault2_State = HAL_GPIO_ReadPin(PWR_Fault2_GPIO_Port, PWR_Fault2_Pin);
	PWR_Fault3_State = HAL_GPIO_ReadPin(PWR_Fault3_GPIO_Port, PWR_Fault3_Pin);
	PWR_Fault4_State = HAL_GPIO_ReadPin(PWR_Fault4_GPIO_Port, PWR_Fault4_Pin);
	//PWR_Fault5_State = HAL_GPIO_ReadPin(PWR_Fault5_GPIO_Port, PWR_Fault5_Pin);

	// Функц кнопки
	But_1_State = !HAL_GPIO_ReadPin(GLOBAL_SB_GPIO_Port, GLOBAL_SB_Pin);
	//But_2_State = !HAL_GPIO_ReadPin(But_2_GPIO_Port, But_2_Pin);
	//EN_OS_State = HAL_GPIO_ReadPin(EN_OS_GPIO_Port, EN_OS_Pin);


	// Считывание аналоговых сигналов
	//T_RT4_Value = Device_ADC_Buf[0];
	//T_RT1_Value = Device_ADC_Buf[1];
	//T_RT2_Value = Device_ADC_Buf[2];
	//T_RT3_Value = Device_ADC_Buf[3];
	//AI_8_Value = Device_ADC_Buf[4];
	U_Value = Device_ADC_Buf[0];
	I_Value = Device_ADC_Buf[1];
	Trt_Value = Device_ADC_Buf[2];
	//T_RT5_Value = Device_ADC_Buf[8];
	I_Instant = ((float)I_mean - I_ADC_ref) * I_ADC_koef;

	if(I_Instant < 0){
		I_abs = -1*I_Instant;
	} else {
		I_abs = I_Instant;
	}

	U_Instant = ((float)U_mean - U_ADC_ref) * U_ADC_koef;
	calc_os_u = U_Instant - R_KABEL*I_abs;

}

void HW_Driver_DO_PWM_Out() {

	HAL_GPIO_WritePin(DO_1_GPIO_Port, DO_1_Pin, DO_1_State);
	HAL_GPIO_WritePin(DO_2_GPIO_Port, DO_2_Pin, DO_2_State);
	HAL_GPIO_WritePin(EN_PWM_GPIO_Port, EN_PWM_Pin, EN_PWM_State);

	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, LED_1_State);


}
/**
uint32_t BIT_READ(uint32_t value, uint32_t pos){

	return (value >> pos) & 0x01;

}

uint32_t BIT_SET(uint32_t value, uint32_t pos){

	return value |= (0x01 << pos);

}

uint32_t BIT_RESET(uint32_t value, uint32_t pos){

	return value &= ~(0x01 << pos);

}

uint32_t BIT_TOGGLE(uint32_t value, uint32_t pos){

	return value ^= ~(0x01 << pos);

}
*/
void BIT_WRITE(uint8_t* value, uint8_t pos, uint8_t state){

	if(state){
		*value |= (0x01 << pos);
	} else {
		*value &= ~(0x01 << pos);
	}

}

float map_value(float in, float in_min, float in_max, float out_min, float out_max){
	return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Out - среднее знанчение
// S - входное число
// Ss - вся сумма
// S_buf - буффер
// N -  размер буффера
void MEAN_Signal(uint32_t *Out, uint16_t *S, uint32_t *Ss, uint16_t *S_buf, uint16_t N) {

	*Ss -= S_buf[0];
	for (int i = 0; i < N - 1; i++) {
		S_buf[i] = S_buf[i + 1];
	}
	S_buf[N - 1] = *S;

	*Ss += S_buf[N - 1];
	*Out = *Ss / N;
}
