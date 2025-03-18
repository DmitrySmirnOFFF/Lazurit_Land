/*
 * my_func.h
 *
 *  Created on: Mar 20, 2023
 *      Author: freem
 */

#ifndef INC_MY_FUNC_H_
#define INC_MY_FUNC_H_

#endif /* INC_MY_FUNC_H_ */

#include "stm32f4xx.h"
#include "arm_math.h"

#define APB2_BASE_FREQ 84000000
#define TIMER_CLK_FREQ 21000000 //Hz (APB timer clock/timer_prescaler) (84000000 / 4)
#define TIMER_SAMPLE_TIME 0.00000000476190476//4.76190476e-8 //sec (1/21000000)

#define SQRT_3_DIV_2 0.866f
#define _2_DIV_SQRT_3 1.1547f
#define _DIV_SQRT_3 0.5774f

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

//OPTIONS FOR PFM
//Окно в котором может меняться частота ШИМ
#define PFM_MAX_FREQ 16000
#define PFM_MIN_FREQ 1000

//Частота работы таймера, в котором вызывается функция регулятора
#define TIMER_FREQ 16000
#define	TIMER_PERIOD 0.0000625  // 1/TIMER_FREQ

//#define PFM_DUTY 0.26 //резонанс обычный
#define PFM_DUTY 0.45 //резонанс обычный
//0.23 это 14 мкс на 16 кгц с нашим драйвером //0.28 это настройка для 32кгц
//#define PFM_DUTY 0.485 //мягкая коммутация
//#define PFM_DUTY 0.33 // супер-мягкая коммутация //old - 0.46

#define PFM_DUTY_CH1 0.26
#define PFM_DUTY_CH2 0.26
#define PFM_DUTY_CH3 0.26
#define PFM_DUTY_CH4 0.26

//Все идет от первого канала, поэтому у него 0
#define PFM_PHASE_CH2 180//90
#define PFM_PHASE_CH3 180
#define PFM_PHASE_CH4 180//270


//Ограниечения на сдвиги по фазе импульсов
#define PFM_MAX_PHASE 168
#define PFM_MIN_PHASE 0
#define PFM_DEFAULT_PHASE 90
#define ZERO_PHASE 20
//Разрешение двигания импульсов по фазе
#define PHASE_REGULATION 1

typedef struct{
	uint16_t Counter;
	uint16_t Pass;
	float Step;
	float Out;
	float *In;
	float UpLimit;
	float DownLimit;
}Ramp_Struct;

typedef struct {
float  	I;
float  	P;
float  	k_I;
float  	k_P;
float  	Out;
float  	*In;
float  	*Fb;
float  	d;
float  	UpLimit;
float  	DownLimit;
float  	I_UpLimit;
float  	I_DownLimit;
}PI_Reg_Struct;

extern Ramp_Struct Ramp_Freq;
extern PI_Reg_Struct Reg_U;
extern Ramp_Struct Ramp_Amp;
extern Ramp_Struct Ramp_Phase;

void Clarke_transform(float *In_ABC, float *Out_alpha_beta);
void Inverse_Clarke_transform(float *In_alpha_beta, float *Out_ABC);
void Direct_Park_transform(float *In_alpha_beta,float *In_cos_sin, float *Out_D_Q);
void Inverse_Park_transform(float *In_D_Q,float *In_cos_sin, float *Out_alpha_beta);

void Ramp_Init(Ramp_Struct *Ramp, float *in,float uplimit, float downlimit, float step, uint16_t pass);
void Ramp_Process(Ramp_Struct *Ramp);
void Ramp_Clear(Ramp_Struct *Ramp);

void Regulator_Init(PI_Reg_Struct *Reg, float *in, float *fb, float k_p, float k_i, float I_uplimit, float I_downlimit, float uplimit, float downlimit);
void Regulator_Clear(PI_Reg_Struct *Reg);
void Regulator_Process(PI_Reg_Struct *Reg);

void PFM_Generator(TIM_TypeDef *Tim, float Freq, float Duty, uint8_t Channel_number);
