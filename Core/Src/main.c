/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t dout_state_mdb = 0;


uint32_t LED_1_cnt;

uint16_t U_DC_buf[MEAN_BUF_SIZE];
uint32_t U_DC_sum;
uint32_t U_DC_mean;

uint16_t I_DC_buf[MEAN_BUF_SIZE];
uint32_t I_DC_sum;
uint32_t I_DC_mean;

int32_t EEPROM_Start_Cnt;
//Definition of non-bloking timers
TIM_NB_TypeDef main_timer;

float zad_freq = PFM_MAX_FREQ;
float zad_u = 0;
float targ_u = 0;
float zad_u_PC = 0;
float down_targ_volt = DOWN_TARGET_VOLT;
float zad_phase = 90;
float zad_phase_old = 0;

uint8_t autocomp_enable = 0;
uint8_t PC_Start_flag = 0;
float calc_os_u;

float zad_amp = 0;

float voltage_avg;

Device_State_TypeDef State;
Device_State_TypeDef Old_State;
uint32_t time_tmp;

uint8_t init_err;
uint16_t Device_Error;

uint32_t tmp_ch2_phase = SOFTSW_CH2_PHASE;
uint32_t tmp_ch3_phase = SOFTSW_CH3_PHASE;
uint32_t tmp_ch4_phase = SOFTSW_CH4_PHASE;

float HardSw_Duty = HARDSW_DUTY;

Button_TypeDef Button_1;
Button_TypeDef PC_Button;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_ADC3_Init();
  //MX_UART5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	//Неблокирующий таймер выполнения основного кода
	TIM_NB_Init(&main_timer, 1, main_timer_function);
	TIM_NB_Start(&main_timer, MULTIPLE_DELAY);

	//Задежка инициализации
	HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		TIM_NB_Check(&main_timer);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim8) {

		//Регуляторы и рампы работают в прерывании таймера
		//потому что SysTick не обеспечивает малые постоянные времени
		if(State == Work){
			//Ramp_Process(&Ramp_Freq);
			Ramp_Process(&Ramp_Amp);
			Regulator_Process(&Reg_U);

		}

	}

}

void SoftSw_PWM_Channels_OFF(void) {
	//Функция для снятия Ш ?Ма на резонансные преобразователи

	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM3, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM9, TIM_CHANNEL_1, TIM_CCx_DISABLE);

	TIM1->BDTR &= ~(TIM_BDTR_MOE);
	TIM1->BDTR &= ~(TIM_BDTR_OSSI);
	TIM2->BDTR &= ~(TIM_BDTR_MOE);
	TIM2->BDTR &= ~(TIM_BDTR_OSSI);
	TIM3->BDTR &= ~(TIM_BDTR_MOE);
	TIM3->BDTR &= ~(TIM_BDTR_OSSI);
	TIM9->BDTR &= ~(TIM_BDTR_MOE);
	TIM9->BDTR &= ~(TIM_BDTR_OSSI);

}

void SoftSw_PWM_Channels_ON(void) {
	//Функция для выдачи ШиМа на резонансные преобразователи

	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	TIM_CCxChannelCmd(TIM3, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	TIM_CCxChannelCmd(TIM9, TIM_CHANNEL_1, TIM_CCx_ENABLE);

	TIM1->BDTR |= (TIM_BDTR_MOE);
	TIM2->BDTR |= (TIM_BDTR_MOE);
	TIM3->BDTR |= (TIM_BDTR_MOE);
	TIM9->BDTR |= (TIM_BDTR_MOE);

}

void SoftSw_PWM_Channels_UpdateDuty(float New_Duty){

	uint32_t duty_tmp;

	if(New_Duty > 1){
		New_Duty = 1;
	}
	if(New_Duty < 0){
		New_Duty = 0;
	}

	duty_tmp = TIM1->ARR * New_Duty;

	if (TIM1->CCR1 != duty_tmp) {
		//common
		TIM1->CCR1 = duty_tmp;
		TIM2->CCR1 = duty_tmp;
		TIM3->CCR1 = duty_tmp;
		TIM9->CCR1 = duty_tmp;
	}

}

void SoftSw_PWM_Channels_UpdatePhase(uint8_t Tim_Ch_num ,uint32_t New_Phase){

	uint32_t phase_tmp;

	switch(Tim_Ch_num){
	//case TIM_CHANNEL_1:
	//	phase_tmp = TIM1->ARR - (uint32_t) ((((float) TIM1->ARR / 360)) * New_Phase) - ZERO_PHASE;
	//	if(TIM1->CCR1 != phase_tmp){
	//		TIM1->CCR1 = phase_tmp;
	//	}
	//	break;
	case TIM_CHANNEL_2:
		phase_tmp = TIM1->ARR - (uint32_t) ((((float) TIM1->ARR / 360)) * New_Phase) - ZERO_PHASE;
		if(TIM1->CCR2 != phase_tmp){
			TIM1->CCR2 = phase_tmp;
		}
		break;
	case TIM_CHANNEL_3:
		phase_tmp = TIM1->ARR - (uint32_t) ((((float) TIM1->ARR / 360)) * New_Phase) - ZERO_PHASE;
		if(TIM1->CCR3 != phase_tmp){
			TIM1->CCR3 = phase_tmp;
		}
		break;
	case TIM_CHANNEL_4:
		phase_tmp = TIM1->ARR - (uint32_t) ((((float) TIM1->ARR / 360)) * New_Phase) - ZERO_PHASE;
		if(TIM1->CCR4 != phase_tmp){
			TIM1->CCR4 = phase_tmp;
		}
		break;
	default:

		break;
	}

}

void SoftSw_PWM_Channels_UpdateFreq(uint32_t New_Freq){

	uint32_t freq_tmp;

	//Ограничение макс частоты
	if (New_Freq > SOFTSW_MAX_FREQ) {
		New_Freq = SOFTSW_MAX_FREQ;
	}
	//Ограничение мин частоты
	if (New_Freq < SOFTSW_MIN_FREQ) {
		New_Freq = SOFTSW_MIN_FREQ;
	}

	freq_tmp = TIMER_CLK_FREQ / New_Freq;

	if(TIM1->ARR != freq_tmp){
		TIM1->ARR = freq_tmp;
		TIM2->ARR = freq_tmp;
		TIM3->ARR = freq_tmp;
		TIM9->ARR = freq_tmp;
		//АЦП!
		TIM4->ARR = freq_tmp;
	}

}

void HardSw_PWM_Channels_ON(void) {
	//Эта функция нужна для усправления выдачей импульсов 8 таймера

	TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	//TIM_CCxChannelCmd(TIM12, TIM_CHANNEL_1, TIM_CCx_ENABLE);

	TIM8->BDTR |= (TIM_BDTR_MOE);
	//TIM12->BDTR |= (TIM_BDTR_MOE);

}
void HardSw_PWM_Channels_OFF(void) {
	//Эта функция нужна для усправления выдачей импульсов 8 таймера

	TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM12, TIM_CHANNEL_1, TIM_CCx_DISABLE);

	TIM8->BDTR &= ~(TIM_BDTR_MOE);
	TIM8->BDTR &= ~(TIM_BDTR_OSSI);
	TIM12->BDTR &= ~(TIM_BDTR_MOE);
	TIM12->BDTR &= ~(TIM_BDTR_OSSI);
}

void HardSw_PWM_Channels_UpdateDuty(float New_Duty){

	uint32_t duty_tmp;

	if(New_Duty > 1){
		New_Duty = 1;
	}
	if(New_Duty < 0){
		New_Duty = 0;
	}

	duty_tmp = TIM8->ARR * New_Duty;

	if (TIM8->CCR1 != duty_tmp) {
		//common
		TIM8->CCR1 = duty_tmp;
		TIM12->CCR1 = duty_tmp;
	}

}

void HardSw_PWM_Channels_UpdatePhase(uint8_t Tim_Ch_num ,uint32_t New_Phase){

	uint32_t phase_tmp;

	switch(Tim_Ch_num){
	//case TIM_CHANNEL_1:
	//	phase_tmp = TIM8->ARR - (uint32_t) ((((float) TIM8->ARR / 360)) * New_Phase) - ZERO_PHASE;
	//	if(TIM8->CCR1 != phase_tmp){
	//		TIM8->CCR1 = phase_tmp;
	//	}
	//	break;
	case TIM_CHANNEL_2:
		phase_tmp = TIM8->ARR - (uint32_t) ((((float) TIM8->ARR / 360)) * New_Phase) - ZERO_PHASE;
		if(TIM8->CCR2 != phase_tmp){
			TIM8->CCR2 = phase_tmp;
		}
		break;
	case TIM_CHANNEL_3:
		phase_tmp = TIM8->ARR - (uint32_t) ((((float) TIM8->ARR / 360)) * New_Phase) - ZERO_PHASE;
		if(TIM8->CCR3 != phase_tmp){
			TIM8->CCR3 = phase_tmp;
		}
		break;
	case TIM_CHANNEL_4:
		phase_tmp = TIM8->ARR - (uint32_t) ((((float) TIM8->ARR / 360)) * New_Phase) - ZERO_PHASE;
		if(TIM8->CCR4 != phase_tmp){
			TIM8->CCR4 = phase_tmp;
		}
		break;
	default:

		break;
	}

}

void HardSw_PWM_Channels_UpdateFreq(uint32_t New_Freq){

	uint32_t freq_tmp;

	//Ограничение макс частоты
	if (New_Freq > HARDSW_MAX_FREQ) {
		New_Freq = HARDSW_MAX_FREQ;
	}
	//Ограничение мин частоты
	if (New_Freq < HARDSW_MIN_FREQ) {
		New_Freq = HARDSW_MIN_FREQ;
	}

	freq_tmp = TIMER_CLK_FREQ / New_Freq;

	if(TIM8->ARR != freq_tmp){
		TIM8->ARR = freq_tmp;
		TIM12->ARR = freq_tmp;
	}

}

void main_timer_function() {

	//Основная функция, где крутится вся логика

	//расчет среднего значения датчиков
	//для фильтрации
	MEAN_Signal(&U_mean, &U_Value, &U_mean_sum, U_mean_buf, MEAN_BUF_SIZE);
	MEAN_Signal(&I_mean, &I_Value, &I_mean_sum, I_mean_buf, MEAN_BUF_SIZE);

	//функции считывания входов и выдачи сигналов на выходы
	HW_Driver_DI_AI_Read();
	HW_Driver_DO_PWM_Out();

	//Светодиодная индикация состояния
	Device_Indication_Process();

	//Проверка ошибок
	if(CHECK_ERROR_EN){
		Device_Check_Error();
	}

	if(Device_Error > 0){
		State = Stop;
	}

	//По этому флагу разрешается работа доп стойки для жесткой коммутациии
	if(EN_OS_State){
		autocomp_enable = 1;
	} else {
		autocomp_enable = 0;
	}

	// обновление регистров MDB
	ModbusRTU_update_reg();

	//Основная стейт машина
	switch (State) {

	case Init:

		//не имеет смысла
		if(State != Old_State){
			time_tmp = millis();
			Old_State = State;
		}

		Button_1.State = &But_1_State;
		Button_1.delay = SB1_T_DELAY;
		Button_1.Old_State = OFF;
		Button_1.cnt = 0;

		PC_Button.State = &PC_Start_flag;
		PC_Button.delay = 100;
		PC_Button.Old_State = OFF;
		PC_Button.cnt = 0;

		init_err = Periph_Init();

		// инициализация рамп и регуляторов
		//Работают сейчас
		Ramp_Init(&Ramp_Amp, &zad_u, 1200, 0, 10, 1);
		//Regulator_Init(&Reg_U, &Ramp_Amp.Out, &U_Instant, 0.0005, 0.005, 0.8, 0, 0.8, 0);
		Regulator_Init(&Reg_U, &Ramp_Amp.Out, &calc_os_u, 0.0005, 0.005, 0.8, 0, 0.8, 0);
		//Не работают
		Ramp_Init(&Ramp_Freq, &zad_freq, PFM_MAX_FREQ, PFM_MIN_FREQ, 40, 1);
		Ramp_Init(&Ramp_Phase, &zad_phase, PFM_MAX_PHASE, PFM_MIN_PHASE, 1, 100);


		//Modbus заготовочка
		ModbusRTU_Init();

		//Конфигурируем регистры таймеров на работу с нужной частотой
		SoftSw_PWM_Channels_UpdateFreq(SOFTSW_FREQ);
		SoftSw_PWM_Channels_UpdatePhase(TIM_CHANNEL_2, SOFTSW_CH2_PHASE);
		SoftSw_PWM_Channels_UpdatePhase(TIM_CHANNEL_3, SOFTSW_CH3_PHASE);
		SoftSw_PWM_Channels_UpdatePhase(TIM_CHANNEL_4, SOFTSW_CH4_PHASE);
		SoftSw_PWM_Channels_UpdateDuty(0);
		SoftSw_PWM_Channels_OFF();

		//Мне удобно так его проконфигурировать и потом выключить шим
		HardSw_PWM_Channels_UpdateFreq(HARDSW_FREQ);
		HardSw_PWM_Channels_UpdatePhase(TIM_CHANNEL_2, HARDSW_CH2_PHASE);
		HardSw_PWM_Channels_UpdateDuty(0);
		HardSw_PWM_Channels_OFF();

		if (init_err == 0) {
			State = Wait;
		} else {
			State = Error;
		}



		break;

	case Wait:

		if(ST_WAIT_EN){
			if(State != Old_State){
				time_tmp = millis();
				Old_State = State;
			}

			Button_Control(&Button_1, PreCharge);
			Button_Control(&PC_Button, PreCharge);

		} else {
			State = PreCharge;
		}

		break;

	case PreCharge:

		if(ST_PRECHARGE_EN){
			if(State != Old_State){
				time_tmp = millis();
				Old_State = State;
				DO_1_State = ON;
				DO_2_State = OFF;
			}

			Button_Control(&Button_1, Stop);
			Button_Control(&PC_Button, Stop);

			if(millis() - time_tmp >= T_PRECHARGE){
				DO_2_State = ON;
				time_tmp = millis();
			}

			if((millis() - time_tmp >= 500) && (DO_2_State == ON)){
				//DO_1_State = OFF; ВЫКЛЮЧАТЬ НЕ НУЖНО, Т.К. ПЕРЕДЕЛАЛИ СХЕМУ ДЛЯ ЗАРЯДА КОНДЕНСАТОРОВ
				State = Delay;
			}

		} else {
			State = Delay;
		}

		break;

	case Delay:

		if(ST_DELAY_EN){
			if(State != Old_State){
				time_tmp = millis();
				Old_State = State;
			}

			Button_Control(&Button_1, Stop);
			Button_Control(&PC_Button, Stop);

			if(millis() - time_tmp >= T_DELAY){
				State = Work;
			}
		} else {
			State = Work;
		}

		break;

	case Work:

		if(ST_WORK_EN){
			if(State != Old_State){
				time_tmp = millis();

				EN_PWM_State = ON;

				//Включение шима
				if(SOFT_SW_EN){
					SoftSw_PWM_Channels_ON();
					SoftSw_PWM_Channels_UpdateDuty(SOFTSW_DUTY);
				}

				if(HARD_SW_EN){
					HardSw_PWM_Channels_ON();
					HardSw_PWM_Channels_UpdateDuty(HARDSW_DUTY);
				}

				Old_State = State;
			}

			//SoftSw_PWM_Channels_UpdatePhase(TIM_CHANNEL_2, tmp_ch2_phase);
			//SoftSw_PWM_Channels_UpdatePhase(TIM_CHANNEL_3, tmp_ch3_phase);
			//SoftSw_PWM_Channels_UpdatePhase(TIM_CHANNEL_4, tmp_ch4_phase);

			if(HARD_SW_EN){

				//Разомкнутая система
				//HardSw_PWM_Channels_UpdateDuty(HardSw_Duty);

				//Замкнутая система
				HardSw_PWM_Channels_UpdateDuty(Reg_U.Out);

				if(millis() - time_tmp >= T_DELAY_WORK){

					if(autocomp_enable){
						if(targ_u != zad_u){
							zad_u = targ_u;
						}
					} else {
						if(zad_u_PC != zad_u){
							zad_u = zad_u_PC;
						}
					}

					if(zad_u >= HV_MAX_VOLT){
						zad_u = HV_MAX_VOLT;
					}

					//targ_u = DOWN_TARGET_VOLT*DOWN_TV_KOEF;

					targ_u = down_targ_volt*DOWN_TV_KOEF;

				}

			}

			Button_Control(&Button_1, Stop);
			Button_Control(&PC_Button, Stop);

		} else {
			State = Stop;
		}


		break;

	case Stop:

		if(ST_STOP_EN){

			if(State != Old_State){
				time_tmp = millis();

				EN_PWM_State = OFF;
				//Отключение шима
				SoftSw_PWM_Channels_OFF();
				SoftSw_PWM_Channels_UpdateDuty(0);

				HardSw_PWM_Channels_OFF();
				HardSw_PWM_Channels_UpdateDuty(0);

				Ramp_Clear(&Ramp_Amp);
				//zad_amp = 0;

				zad_u = 0;
				Regulator_Clear(&Reg_U);


				DO_1_State = OFF;
				DO_2_State = OFF;

				Old_State = State;
			}

			if(Device_Error > 0){
				State = Error;
			} else {
				if(millis() - time_tmp >= T_STOP){
					State = Wait;
				}
			}

		} else {
			State = Wait;
		}

		break;

	case Error:

		if(ST_ERROR_EN){

			if(DO_1_State){
				DO_1_State = OFF;
			}

			if(DO_2_State){
				DO_2_State = OFF;
			}

			if(EN_PWM_State){
				EN_PWM_State = OFF;
			}

			//На всякий случай ещё раз выключаем шим
			SoftSw_PWM_Channels_OFF();
			SoftSw_PWM_Channels_UpdateDuty(0);

			HardSw_PWM_Channels_OFF();
			HardSw_PWM_Channels_UpdateDuty(0);

			if(Device_Error == 0){
				State = Wait;
			}

		} else {
			State = Wait;
		}

		break;

	default:
		break;

	}

}

//-----NON BLOKING TIMER--------------------------------------------------------------------------
void TIM_NB_Init(TIM_NB_TypeDef *timer, uint32_t delay, void (*funptr)) {

	timer->delay = delay;
	timer->function = funptr;
}

void TIM_NB_Start(TIM_NB_TypeDef *timer, uint8_t mode) {

	if (timer->delay != 0) {
		timer->counter = millis();
		timer->state = mode;
	}
}

void TIM_NB_Stop(TIM_NB_TypeDef *timer) {

	if (timer->state != STOPPED) {
		timer->state = STOPPED;
	}
}

void TIM_NB_Check(TIM_NB_TypeDef *timer) {

	if (timer->state != STOPPED) {
		if (HAL_GetTick() - timer->counter >= timer->delay) {
			switch (timer->state) {
			case SINGLE_DELAY:
				timer->state = STOPPED;
				timer->counter = 0;
				timer->function();
				break;
			case MULTIPLE_DELAY:
				timer->counter = HAL_GetTick();
				timer->function();
				break;
			}
		}
	}
}

uint32_t millis() {
	return HAL_GetTick();
}
//-----NON BLOKING TIMER--------------------------------------------------------------------------

void LED_Control(LED_CMD_TypeDef Cmd, uint32_t Blink_Period, enum_state *LED_Name, uint32_t *LED_cnt) {

	if (Cmd == LED_OFF) {
		*LED_Name = OFF;
	} else if (Cmd == LED_ON) {
		*LED_Name = ON;
	} else if (Cmd == LED_BLINK) {
		if (HAL_GetTick() - *LED_cnt >= Blink_Period) {
			*LED_cnt = HAL_GetTick();
			if (*LED_Name == ON) {
				*LED_Name = OFF;
			} else {
				*LED_Name = ON;
			}
		}
	}

}

void Button_Control(Button_TypeDef *Button, uint8_t Target_state){

	if((*Button->State == ON) && (Button->Old_State == OFF)){
		if(millis() - Button->cnt >= Button->delay){
			Button->Old_State = ON;
			State = Target_state;
		}
	} else {
		if(*Button->State == OFF){
			Button->Old_State = OFF;
		}
		Button->cnt = millis();
	}

}

void Device_Indication_Process(){

	if (State == Work)
		LED_Control(LED_ON, 0, &LED_1_State, &LED_1_Cnt);
	else if ((State == Delay) || (State == PreCharge))
		LED_Control(LED_BLINK, 100, &LED_1_State, &LED_1_Cnt);
	else if (State == Wait)
		LED_Control(LED_BLINK, 500, &LED_1_State, &LED_1_Cnt);
	else if (State == Error)
		LED_Control(LED_OFF, 0, &LED_1_State, &LED_1_Cnt);
	else
		LED_Control(LED_OFF, 0, &LED_1_State, &LED_1_Cnt);

}

void Device_Check_Error(void){

	//Ошибка датчика тока
	if(CHECK_I_SENS_EN){
		if(I_Value < I_SENSOR_MIN_ADC){
			BIT_SET(Device_Error, 0);
		} else {
			//BIT_RESET(Device_Error, 0);
		}
		if(I_Value > I_SENSOR_MAX_ADC){
			BIT_SET(Device_Error, 0);
		} else {
			//BIT_RESET(Device_Error, 0);
		}
	}

	//Ошибки первого драйвера
	if(CHECK_DRW1_EN){
		if(CHECK_DRW_PWR){
			if(PWR_Fault1_State){
				BIT_SET(Device_Error, 1);
			}else {
				//BIT_RESET(Device_Error, 1);
			}
		}
		if(CHECK_DRW_HW){
			if(Fault_H1_State){
				BIT_SET(Device_Error, 6);
			}else {
				//BIT_RESET(Device_Error, 6);
			}
			if(Fault_L1_State){
				BIT_SET(Device_Error, 7);
			}else {
				//BIT_RESET(Device_Error, 7);
			}
		}
	}

	//Ошибки второго драйвера
	if(CHECK_DRW2_EN){
		if(CHECK_DRW_PWR){
			if(PWR_Fault2_State){
				BIT_SET(Device_Error, 2);
			}else {
				//BIT_RESET(Device_Error, 2);
			}
		}
		if(CHECK_DRW_HW){
			if(Fault_H2_State){
				BIT_SET(Device_Error, 8);
			}else {
				//BIT_RESET(Device_Error, 8);
			}
			if(Fault_L2_State){
				BIT_SET(Device_Error, 9);
			}else {
				//BIT_RESET(Device_Error, 9);
			}
		}
	}

	//Ошибки третьего драйвера
	if(CHECK_DRW3_EN){
		if(CHECK_DRW_PWR){
			if(PWR_Fault3_State){
				BIT_SET(Device_Error, 3);
			}else {
				//BIT_RESET(Device_Error, 3);
			}
		}
		if(CHECK_DRW_HW){
			if(Fault_H3_State){
				BIT_SET(Device_Error, 10);
			}else {
				//BIT_RESET(Device_Error, 10);
			}
			if(Fault_L3_State){
				BIT_SET(Device_Error, 11);
			}else {
				//BIT_RESET(Device_Error, 11);
			}
		}
	}

	//Ошибки четвертого драйвера
	if(CHECK_DRW4_EN){
		if(CHECK_DRW_PWR){
			if(PWR_Fault4_State){
				BIT_SET(Device_Error, 4);
			}else {
				//BIT_RESET(Device_Error, 4);
			}
		}
		if(CHECK_DRW_HW){
			if(Fault_H4_State){
				BIT_SET(Device_Error, 12);
			}else {
				//BIT_RESET(Device_Error, 12);
			}
			if(Fault_H4_1_State){
				BIT_SET(Device_Error, 12);
			}else {
				//BIT_RESET(Device_Error, 12);
			}
			if(Fault_L4_State){
				BIT_SET(Device_Error, 13);
			}else {
				//BIT_RESET(Device_Error, 13);
			}
			if(Fault_L4_2_State){
				BIT_SET(Device_Error, 13);
			}else {
				//BIT_RESET(Device_Error, 13);
			}
		}
	}

	//Ошибки пятого драйвера
	if(CHECK_DRW5_EN){
		if(CHECK_DRW_PWR){
			if(PWR_Fault5_State){
				BIT_SET(Device_Error, 5);
			}else {
				//BIT_RESET(Device_Error, 5);
			}
		}
		if(CHECK_DRW_HW){
			if(Fault_H5_State){
				BIT_SET(Device_Error, 14);
			}else {
				//BIT_RESET(Device_Error, 14);
			}
			if(Fault_L5_State){
				BIT_SET(Device_Error, 15);
			}else {
				//BIT_RESET(Device_Error, 15);
			}
		}
	}

}

void ModbusRTU_Init_AO ()
{
	// Определение переменных AO
	for(uint8_t i = 0; i < ADR_MAX_AO; i++)
	{
		DATA_AO[i] = &DATA_AO_buf[i];
	}
}

void ModbusRTU_Init_AI ()
{
	// Определение переменных AI
	for(uint8_t i = 0; i < ADR_MAX_AI; i++)
	{
		DATA_AI[i] = &DATA_AI_buf[i];
	}
}

void ModbusRTU_update_reg()
{
	int16_t  tmp_int16 = 0;
	uint16_t tmp_uint16 = 0;

	// Read reg mdb
	// State machine for modbus
	switch (State)
	{
	case Init:
		tmp_uint16 = 0b0000000000000001;
		break;
	case Wait:
		tmp_uint16 = 0b0000000000000010;
		break;
	case PreCharge:
		tmp_uint16 = 0b0000000000000100;
		break;
	case Delay:
		tmp_uint16 = 0b0000000000001000;
		break;
	case Work:
		tmp_uint16 = 0b0000000000010000;
		break;
	case Stop:
		tmp_uint16 = 0b0000000000100000;
		break;
	case Error:
		tmp_uint16 = 0b0000000001000000;
		break;
	default:
		break;
	}
	DATA_AI_buf[0] = tmp_uint16;

	// State dout for modbus
	if (DO_1_State)   { tmp_uint16 |=  0b0000000000000001; }
	else              { tmp_uint16 &= ~0b0000000000000001; }

	if (DO_2_State)   { tmp_uint16 |=  0b0000000000000010; }
	else              { tmp_uint16 &= ~0b0000000000000010; }

	if (EN_PWM_State) { tmp_uint16 |=  0b0000000000000100; }
	else              { tmp_uint16 &= ~0b0000000000000100; }
	DATA_AI_buf[1] = tmp_uint16;

	// Erorr
	DATA_AI_buf[2] = Device_Error;

	// U
	tmp_int16 = U_Instant;
	DATA_AI_buf[3] = (uint16_t)tmp_int16;

	// I
	tmp_int16 = I_Instant;
	DATA_AI_buf[4] = (uint16_t)tmp_int16;

	// Reg_U.Out
	tmp_int16 = Reg_U.Out;
	DATA_AI_buf[5] = (uint16_t)tmp_int16;
	PC_Start_flag = DATA_AO_buf[0];


	// Write reg mdb
	if (DATA_AO_buf[0] == 1)
	{
		PC_Start_flag = 1;
	}
	else
	{
		PC_Start_flag = 0;
	}

	return;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
