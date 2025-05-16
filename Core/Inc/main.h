/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "Hardware_Driver.h"
#include "ModbusRTU.h"
#include <other_func.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

//Здесь выбор какой преобразователь
//LAND_PART - наземный
//UNDERWATER_PART - подводный
#define LAND_PART
//#define UNDERWATER_PART

typedef struct{
	uint8_t* State;
	uint8_t Old_State;
	uint32_t cnt;
	uint32_t delay;
}Button_TypeDef;

typedef enum{
	LED_OFF = 0,
	LED_ON = 1,
	LED_BLINK = 2,
} LED_CMD_TypeDef;

typedef enum{
	Init = 0,
	Wait = 1,
	PreCharge = 2,
	Delay = 3,
	Work = 4,
	Stop = 5,
	Error = 6,
	TmpDS = 2147483649,
} Device_State_TypeDef;

extern Device_State_TypeDef State;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//ALL PROGRAM CONSTANTS HERE

//Вся логика работы платы содержится в main.c
//см. main_timer_function
//Если кратко то все управление завязано на кнопку PC13
//Нажимаешь ее и плата идет в предзаряд, потом идет небольшая пауза
//После этого выходит Ш�?М, при повторном нажатии кнопки все выключается
//и плата уходит в ожидание
//Можно имитировать кнопку программно, для этого есть PC_Button
#ifdef LAND_PART
//Разрешение импульсов для стоек с мягкой коммутацией
#define SOFT_SW_EN 1
//Разрешение импульсов для стоек с жесткой коммутацией (для регулирования напряжения)
#define HARD_SW_EN 1

//Выбор стейтов, если 0, то он пропускается
#define ST_WAIT_EN 1
#define ST_PRECHARGE_EN 1
#define ST_DELAY_EN 1
#define ST_WORK_EN 1
#define ST_STOP_EN 1
#define ST_ERROR_EN 1

//Проверки ошибок, если выставить все нули, то плата ничего не будет проверять
#define CHECK_ERROR_EN 0
//Проверка наличия датчика тока
#define CHECK_I_SENS_EN 0
//Провека стойки 1
#define CHECK_DRW1_EN 0
//Провека стойки 2
#define CHECK_DRW2_EN 0
//Провека стойки 3
#define CHECK_DRW3_EN 0
//Провека стойки 4
#define CHECK_DRW4_EN 0
//Провека стойки 5
#define CHECK_DRW5_EN 0

//проверка наличия питания на драйверах
#define CHECK_DRW_PWR 0
//проверка хардверной ошибки от драйвера
#define CHECK_DRW_HW 1

//Сколько нужно держать кнопку, прежде чем засчитается нажатие
#define SB1_T_DELAY 200

//Выдержки времени
//На предзаряд
#define T_PRECHARGE 10000 //in ms
//в стейте delay
#define T_DELAY 1000 //in ms
//в стейте work
#define T_DELAY_WORK 1000 //in ms
//в стейте stop
#define T_STOP 1000 //in ms

//% от периода, длительность в сек можно пересчитать через частоту
//(рассчитано и проверено экс. путем)
// если сделать меньше, то может быть жёсткая коммутация
// но все зависит от транзисторного модуля
#define SOFTSW_DUTY 0.35 //old - 0.45

//Уставки частоты для мягкой коммутации
//В какой то момент надо было изменять частоту в реальном времени
//поэтому появились переменные max и min
//реальная частота задается SOFTSW_FREQ
//она ограничится min и max программно
#define SOFTSW_MAX_FREQ 18000
#define SOFTSW_FREQ 16000
#define SOFTSW_MIN_FREQ 11000

//сдвиги фаз для импульсов транзисторов по стойкам
#define SOFTSW_CH2_PHASE 180
#define SOFTSW_CH3_PHASE 90
#define SOFTSW_CH4_PHASE 270

//Длительность для жесткой коммутации
//Задается регулятором, поэтому 0
#define HARDSW_DUTY 0.0
//Уставки частоты для жесткой коммутации
#define HARDSW_MAX_FREQ 18000
#define HARDSW_FREQ 16000
#define HARDSW_MIN_FREQ 14000
//сдвиги фаз для импульсов транзисторов по стойкам
#define HARDSW_CH2_PHASE 180

#endif
//Для подводной части все идентично надводной
#ifdef UNDERWATER_PART
#define SOFT_SW_EN 1
#define HARD_SW_EN 0

#define ST_WAIT_EN 0
#define ST_PRECHARGE_EN 0
#define ST_DELAY_EN 1
#define ST_WORK_EN 1
#define ST_STOP_EN 1
#define ST_ERROR_EN 1

#define CHECK_ERROR_EN 1
#define CHECK_I_SENS_EN 0
#define CHECK_DRW1_EN 1
#define CHECK_DRW2_EN 1
#define CHECK_DRW3_EN 1
#define CHECK_DRW4_EN 1
#define CHECK_DRW5_EN 0

#define CHECK_DRW_PWR 1
#define CHECK_DRW_HW 1

#define SB1_T_DELAY 200

//Выдержки времени
//На предзаряд
#define T_PRECHARGE 3000 //in ms
//в стейте delay
#define T_DELAY 200 //in ms
//в стейте work
#define T_DELAY_WORK 10 //in ms
//в стейте stop
#define T_STOP 1000 //in ms

#define SOFTSW_DUTY 0.45

#define SOFTSW_MAX_FREQ 18000
#define SOFTSW_FREQ 16000
#define SOFTSW_MIN_FREQ 14000

#define SOFTSW_CH2_PHASE 180
#define SOFTSW_CH3_PHASE 90
#define SOFTSW_CH4_PHASE 270

#define HARDSW_DUTY 0.45

#define HARDSW_MAX_FREQ 18000
#define HARDSW_FREQ 16000
#define HARDSW_MIN_FREQ 14000

#define HARDSW_CH2_PHASE 180
#endif

//коэффициенты датчиков
//датчик напряжения
#define U_SENS_K 4.68634 //5.3
//датчик тока
#define I_SENS_K 0.087137
//это если нужно учесть что ты пропустил провод несколько раз через датчик
#define TURN_NUM 1

//здесь цифры для расчета напряжения внизу
//нужно для корректной работы обратной связи по напряжению
//сопротивление кабеля
#define R_KABEL 3
//коэффициент трансформации в подводной части
#define DOWN_TV_KOEF 3.25 //2,48 новые 3,21 гаммамет
//требуемое напряжение на выходе подводной части
#define DOWN_TARGET_VOLT 2000 //320 старое
//максимальное напряжение на выходе надводной части
//!!!! НЕ РАБОТАЕТ // РАБОТАЕТ!!!!!!
#define HV_MAX_VOLT 2100

extern uint32_t SoftSw_CCR;
extern uint32_t HardSw_CCR;

extern float HardSw_Duty;
extern float zad_u;
extern float targ_u;
extern float zad_u_PC;
extern float down_targ_volt;
extern uint8_t autocomp_enable;
extern uint8_t PC_Start_flag;
extern float calc_os_u;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef enum{
	DEVICE_OFF = 0,
	DIRECT_CONTROL = 1,
	REG_CONTROL = 2,
	TEST = 3,
} Common_enum;

//-----NON BLOKING TIMER--------------------------------------------------------------------------
//CURRENTLY USING SysTick AS A TIMEBASE!
//THIS TIMER PROVIDES >= TIME DELAY, NOT PRECISE DELAY
//TRY NOT USE HAL_Delay!
//To use this you need:
//1. create custom_timer as NB_Timer_TypeDef
//2. init it using init_nb_timer() BEFORE while(1)
//3. start your timer with start_nb_timer()
//4. call check_nb_timer() in while(1)

//Timer states definition
#define STOPPED 0
#define SINGLE_DELAY 1
#define MULTIPLE_DELAY 2

typedef struct{

	uint8_t state;
	uint32_t counter;
	uint32_t delay;
	void (*function)(void);

} TIM_NB_TypeDef;

void TIM_NB_Init(TIM_NB_TypeDef* timer,uint32_t delay, void (*funptr));
void TIM_NB_Start(TIM_NB_TypeDef* timer, uint8_t mode);
void TIM_NB_Stop(TIM_NB_TypeDef* timer);
void TIM_NB_Check(TIM_NB_TypeDef* timer);
uint32_t millis();
//-----NON BLOKING TIMER--------------------------------------------------------------------------
void main_timer_function();
void Button_Control(Button_TypeDef *Button, uint8_t Target_state);
void LED_Control(LED_CMD_TypeDef Cmd, uint32_t Blink_Period, enum_state *LED_Name, uint32_t *LED_cnt);

void SoftSw_PWM_Channels_OFF(void);
void SoftSw_PWM_Channels_ON(void);
void SoftSw_PWM_Channels_UpdateDuty(float New_Duty);
void SoftSw_PWM_Channels_UpdatePhase(uint8_t Tim_Ch_num ,uint32_t New_Phase);
void SoftSw_PWM_Channels_UpdateFreq(uint32_t New_Freq);

void HardSw_PWM_Channels_ON(void);
void HardSw_PWM_Channels_OFF(void);
void HardSw_PWM_Channels_UpdateDuty(float New_Duty);
void HardSw_PWM_Channels_UpdatePhase(uint8_t Tim_Ch_num ,uint32_t New_Phase);
void HardSw_PWM_Channels_UpdateFreq(uint32_t New_Freq);

void Device_Check_Error();
void Device_Indication_Process();

void APP_SoftShutdown();

void ModbusRTU_update_reg();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Trt_Pin GPIO_PIN_0
#define Trt_GPIO_Port GPIOC
#define U_Pin GPIO_PIN_1
#define U_GPIO_Port GPIOC
#define I_Pin GPIO_PIN_2
#define I_GPIO_Port GPIOC
#define PWR_Fault3_Pin GPIO_PIN_7
#define PWR_Fault3_GPIO_Port GPIOA
#define Fault_L3_Pin GPIO_PIN_4
#define Fault_L3_GPIO_Port GPIOC
#define Fault_H3_Pin GPIO_PIN_5
#define Fault_H3_GPIO_Port GPIOC
#define PWR_Fault4_Pin GPIO_PIN_0
#define PWR_Fault4_GPIO_Port GPIOB
#define Fault_L4_Pin GPIO_PIN_1
#define Fault_L4_GPIO_Port GPIOB
#define Fault_H4_Pin GPIO_PIN_2
#define Fault_H4_GPIO_Port GPIOB
#define EN_PWM_Pin GPIO_PIN_10
#define EN_PWM_GPIO_Port GPIOB
#define Fault_L5_Pin GPIO_PIN_12
#define Fault_L5_GPIO_Port GPIOB
#define Fault_H5_Pin GPIO_PIN_13
#define Fault_H5_GPIO_Port GPIOB
#define Fault_L4_2_Pin GPIO_PIN_7
#define Fault_L4_2_GPIO_Port GPIOC
#define Fault_H4_1_Pin GPIO_PIN_8
#define Fault_H4_1_GPIO_Port GPIOC
#define PWR_Fault1_Pin GPIO_PIN_10
#define PWR_Fault1_GPIO_Port GPIOA
#define Fault_L1_Pin GPIO_PIN_11
#define Fault_L1_GPIO_Port GPIOA
#define Fault_H1_Pin GPIO_PIN_12
#define Fault_H1_GPIO_Port GPIOA
#define PWR_Fault2_Pin GPIO_PIN_10
#define PWR_Fault2_GPIO_Port GPIOC
#define Fault_L2_Pin GPIO_PIN_11
#define Fault_L2_GPIO_Port GPIOC
#define UART5_DIR_Pin GPIO_PIN_3
#define UART5_DIR_GPIO_Port GPIOB
#define GLOBAL_SB_Pin GPIO_PIN_4
#define GLOBAL_SB_GPIO_Port GPIOB
#define Fault_H2_Pin GPIO_PIN_5
#define Fault_H2_GPIO_Port GPIOB
#define DO_1_Pin GPIO_PIN_6
#define DO_1_GPIO_Port GPIOB
#define DO_2_Pin GPIO_PIN_7
#define DO_2_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_8
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_9
#define LED_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
