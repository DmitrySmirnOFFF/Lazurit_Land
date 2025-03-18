/*
 * Hardware_Driver.h
 *
 *  Created on: Jan 24, 2024
 *      Author: Ivan
 */
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "tim.h"

#ifndef INC_HW_DRIVER_H_
#define INC_HW_DRIVER_H_

#define BIT_READ(value, pos) (((value) >> (pos)) & 0x01)
#define BIT_SET(value, pos) ((value) |= (1UL << (pos)))
#define BIT_RESET(value, pos) ((value) &= ~(1UL << (pos)))
#define BIT_TOGGLE(value, pos) ((value) ^= ~(1UL << (pos)))

//Настройки АЦП
#define ADC_CHANNELS 3 // Количество каналов АЦП
#define ADC_DELAY 1000 // Задержка перед запуском основного таймера

//Настройки датчика тока
#define I_SENSOR_MIN_ADC 500
#define I_SENSOR_MAX_ADC 3500

#define MEAN_BUF_SIZE 50

typedef enum{
	 OFF	 = 0,
	 ON  	 = 1
} enum_state;

// Дискретные входы
extern enum_state TEST_IN_State;

extern enum_state Fault_H1_State;
extern enum_state Fault_H2_State;
extern enum_state Fault_H3_State;
extern enum_state Fault_H4_State;
extern enum_state Fault_H4_1_State;
extern enum_state Fault_H5_State;

extern enum_state Fault_L1_State;
extern enum_state Fault_L2_State;
extern enum_state Fault_L3_State;
extern enum_state Fault_L4_State;
extern enum_state Fault_L4_2_State;
extern enum_state Fault_L5_State;

extern enum_state PWR_Fault1_State;
extern enum_state PWR_Fault2_State;
extern enum_state PWR_Fault3_State;
extern enum_state PWR_Fault4_State;
extern enum_state PWR_Fault5_State;

// Функц кнопки
extern enum_state But_1_State;
extern enum_state But_2_State;
extern enum_state EN_OS_State;

// Дискретные выходы
extern enum_state DO_1_State;
extern enum_state DO_2_State;
extern enum_state EN_PWM_State;

extern enum_state LED_1_State;
extern uint32_t LED_1_Cnt;

//Аналогавые входы
extern uint16_t U_Value;
extern uint16_t I_Value;
extern uint16_t Trt_Value;
extern uint16_t T_RT1_Value;
extern uint16_t T_RT2_Value;
extern uint16_t T_RT3_Value;
extern uint16_t T_RT4_Value;
extern uint16_t T_RT5_Value;
extern uint16_t AI_8_Value;

extern float U_Instant;
extern float U_ADC_ref;
extern float U_ADC_koef;

extern uint32_t U_mean;
extern uint32_t U_mean_sum;
extern uint16_t U_mean_buf[MEAN_BUF_SIZE];

extern float I_Instant;
extern float I_ADC_ref;
extern float I_ADC_koef;

extern float I_abs;

extern uint32_t I_mean;
extern uint32_t I_mean_sum;
extern uint16_t I_mean_buf[MEAN_BUF_SIZE];

// перемнные для ацп в стм, мгновенные значения
extern uint16_t Device_ADC_Buf[ADC_CHANNELS];

//-----TIMEOUT-------
#define TIMEOUT_CHECK_MS 100

void Periph_Deinit();
uint8_t Periph_Init();


void HW_Driver_DI_AI_Read();
void HW_Driver_DO_PWM_Out();
/*
uint32_t BIT_READ(uint32_t value, uint32_t pos);
uint32_t BIT_SET(uint32_t value, uint32_t pos);
uint32_t BIT_RESET(uint32_t value, uint32_t pos);
uint32_t BIT_TOGGLE(uint32_t value, uint32_t pos);
*/
void BIT_WRITE(uint8_t* value, uint8_t pos, uint8_t state);
float map_value(float in, float in_min, float in_max, float out_min, float out_max);

void MEAN_Signal(uint32_t *Out, uint16_t *S, uint32_t *Ss, uint16_t *S_buf, uint16_t N);

#endif /* INC_HW_DRIVER_H_ */
