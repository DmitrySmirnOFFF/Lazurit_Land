/*
 * my_func.c
 *
 *  Created on: Mar 20, 2023
 *      Author: freem
 */
#include <other_func.h>

Ramp_Struct Ramp_Amp;
Ramp_Struct Ramp_Freq;
Ramp_Struct Ramp_Phase;
PI_Reg_Struct Reg_U;

// Это поправочное число чтобы подстроить сдвиги по фазе
// найдено в процессе отладки, лучше не трогать
//uint32_t zero_phase = 20;
float phase_tmp;
float ss_phase_ch2 = 10;
float ss_phase_ch3 = 180;
float ss_phase_ch4 = 10;

/*
 * Функция инициализации рампы. Рамп может быть много, все независимо друг от друга
 * *Ramp - Указатель на структуру рампы
 * *in - Указатель на переменную, на которую воздействует рампа
 * uplimit - верхний предел
 * downlimit - нижний предел
 * step - число, которое прибавляется/вычитается из in на каждом проходе
 * pass - сколько циклов таймера рампа пропускает
 * */
void Ramp_Init(Ramp_Struct *Ramp, float *in, float uplimit, float downlimit, float step, uint16_t pass) {

	Ramp->In = in;
	Ramp->Counter = 0;
	Ramp->Pass = pass;
	Ramp->DownLimit = downlimit;
	Ramp->Out = 0;
	Ramp->Step = step;
	Ramp->UpLimit = uplimit;

}
/*
 * Функция очистки рампы
 * *Ramp - Указатель на структуру рампы подлежащей очистке
 * */
void Ramp_Clear(Ramp_Struct *Ramp) {

	Ramp->Counter = 0;
	Ramp->Out = 0;

}

/*
 * Функция работы рампы
 * *Ramp - Указатель на структуру рампы которая работает
 * */
void Ramp_Process(Ramp_Struct *Ramp) {

	if (Ramp->Counter < Ramp->Pass) {
		Ramp->Counter++;
	} else {

		/*
		if (*Ramp->In > Ramp->UpLimit) {
			*Ramp->In = Ramp->UpLimit;
		}
		if (*Ramp->In <= Ramp->DownLimit) {
			*Ramp->In = Ramp->DownLimit;
		}

		if (Ramp->Out < *Ramp->In) {
			Ramp->Out += Ramp->Step;
		} else if (Ramp->Out > *Ramp->In) {
			Ramp->Out -= Ramp->Step;
		}

		if (Ramp->Out >= *Ramp->In) {
			Ramp->Out = *Ramp->In;
		} else if (Ramp->Out <= *Ramp->In) {
			Ramp->Out = *Ramp->In;
		}
		*/

		 if ((Ramp->Out < *Ramp->In) && (Ramp->Out >= 0)) {
			 if((Ramp->Out + Ramp->Step) < *Ramp->In){
				 Ramp->Out += Ramp->Step;
			 } else {
				 Ramp->Out = *Ramp->In;
			 }
		 } else if ((Ramp->Out > *Ramp->In) && (Ramp->Out > 0)) {
		 Ramp->Out -= Ramp->Step;
		 } else if ((Ramp->Out > *Ramp->In) && (Ramp->Out <= 0)) {
			 if((Ramp->Out + Ramp->Step) > *Ramp->In){
			 				 Ramp->Out -= Ramp->Step;
			 			 } else {
			 				 Ramp->Out = *Ramp->In;
			 			 }
		 } else if ((Ramp->Out < *Ramp->In) && (Ramp->Out < 0)) {
		 Ramp->Out += Ramp->Step;
		 }



		 if (Ramp->Out >= Ramp->UpLimit){
		 Ramp->Out = Ramp->UpLimit;
		 } else if (Ramp->Out <= Ramp->DownLimit){
		 Ramp->Out = Ramp->DownLimit;
		 }

		Ramp->Counter = 0;
	}

}

void Regulator_Init(PI_Reg_Struct *Reg, float *in, float *fb, float k_p, float k_i, float I_uplimit, float I_downlimit, float uplimit, float downlimit) {

	Reg->Fb = fb;
	Reg->DownLimit = downlimit;
	Reg->I = 0;
	Reg->I_DownLimit = I_downlimit;
	Reg->I_UpLimit = I_uplimit;
	Reg->In = in;
	Reg->Out = 0;
	Reg->P = 0;
	Reg->UpLimit = uplimit;
	Reg->d = 0;
	Reg->k_I = k_i;
	Reg->k_P = k_p;

}

void Regulator_Clear(PI_Reg_Struct *Reg) {

	Reg->I = 0;
	Reg->Out = 0;
	Reg->P = 0;
	Reg->d = 0;

}

void Regulator_Process(PI_Reg_Struct *Reg) {

	Reg->d = *Reg->In - *Reg->Fb;

	Reg->I += TIMER_PERIOD * Reg->k_I * Reg->d;
	if (Reg->I < Reg->I_DownLimit)
		Reg->I = Reg->I_DownLimit;
	else if (Reg->I > Reg->I_UpLimit)
		Reg->I = Reg->I_UpLimit;

	Reg->P = Reg->k_P * Reg->d;

	Reg->Out = Reg->P + Reg->I;
	if (Reg->Out < Reg->DownLimit)
		Reg->Out = Reg->DownLimit;
	else if (Reg->Out > Reg->UpLimit)
		Reg->Out = Reg->UpLimit;

}

void Clarke_transform(float *In_ABC, float *Out_alpha_beta) {

	Out_alpha_beta[0] = In_ABC[0];
	Out_alpha_beta[1] = (In_ABC[1] - In_ABC[2]) / _DIV_SQRT_3;

}

void Inverse_Clarke_transform(float *In_alpha_beta, float *Out_ABC) {

	Out_ABC[0] = In_alpha_beta[1];
	Out_ABC[1] = -0.5 * In_alpha_beta[1] + SQRT_3_DIV_2 * In_alpha_beta[0];
	Out_ABC[2] = -0.5 * In_alpha_beta[1] - SQRT_3_DIV_2 * In_alpha_beta[0];

}

void Direct_Park_transform(float *In_alpha_beta, float *In_cos_sin, float *Out_D_Q) {

	Out_D_Q[0] = In_alpha_beta[0] * In_cos_sin[0] + In_alpha_beta[1] * In_cos_sin[1];
	Out_D_Q[1] = -In_alpha_beta[0] * In_cos_sin[1] + In_alpha_beta[1] * In_cos_sin[0];

}

void Inverse_Park_transform(float *In_D_Q, float *In_cos_sin, float *Out_alpha_beta) {

	Out_alpha_beta[0] = In_D_Q[0] * In_cos_sin[0] - In_D_Q[1] * In_cos_sin[1];
	Out_alpha_beta[1] = In_D_Q[0] * In_cos_sin[1] + In_D_Q[1] * In_cos_sin[0];

}

void PFM_Generator(TIM_TypeDef *Tim, float Freq, float Duty, uint8_t Channel_number) {
	//Эта функция пересчитывает частоту выдаваемых импульсов и пересчитывает
	//значения соответствующих регистров
	//!!!Если задать частоту меньше минимальной импульсы снимутся

	//Disable timer
	//Tim->CR1 &= ~(TIM_CR1_CEN);
	//Ограничение макс частоты
	if (Freq > PFM_MAX_FREQ) {
		Freq = PFM_MAX_FREQ;
	}

	//Если частота меньше минимума выключаем шим
	if (Freq < PFM_MIN_FREQ) {

		Tim->ARR = TIMER_CLK_FREQ / PFM_MIN_FREQ;

		switch (Channel_number) {
		case 0:
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_1, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			Tim->CCR1 = Tim->ARR * Duty;
			break;
		case 4:
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_2, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			Tim->CCR2 = Tim->ARR * Duty;
			break;
		case 8:
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_3, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			Tim->CCR3 = Tim->ARR * Duty;
			break;
		default:
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_1, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_2, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_3, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			break;
		}

		Tim->BDTR &= ~(TIM_BDTR_MOE);
		Tim->BDTR &= ~(TIM_BDTR_OSSI);

	} else {

		Tim->ARR = TIMER_CLK_FREQ / Freq;

		switch (Channel_number) {
		case 0:
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_1, TIM_CCx_ENABLE | TIM_CCxN_ENABLE);
			Tim->CCR1 = Tim->ARR * Duty;
			break;
		case 4:
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_2, TIM_CCx_ENABLE | TIM_CCxN_ENABLE);
			Tim->CCR2 = Tim->ARR * Duty;
			break;
		case 8:
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_3, TIM_CCx_ENABLE | TIM_CCxN_ENABLE);
			Tim->CCR3 = Tim->ARR * Duty;
			break;
		default:
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_1, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_2, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			TIM_CCxChannelCmd(Tim, TIM_CHANNEL_3, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
			break;
		}

		Tim->BDTR |= (TIM_BDTR_MOE);

	}

	if (PHASE_REGULATION && (Tim == TIM1)) {
		phase_tmp = (((float) TIM1->ARR / 360));
		//common

		TIM1->CCR2 = TIM1->ARR - (uint32_t) (phase_tmp * PFM_PHASE_CH2) - ZERO_PHASE; 	//сдвиг на 90
		TIM1->CCR3 = TIM1->ARR - (uint32_t) (phase_tmp * PFM_PHASE_CH3) - ZERO_PHASE;	//сдвиг на 180
		TIM1->CCR4 = TIM1->ARR - (uint32_t) (phase_tmp * PFM_PHASE_CH4) - ZERO_PHASE;	//сдвиг на 270

		//super soft
		/*
		TIM1->CCR2 = 1060; 	// old - 1240
		TIM1->CCR3 = 640;	//сдвиг на 180
		TIM1->CCR4 = 400;	// old - 575
		*/
	} else if (PHASE_REGULATION && (Tim == TIM8)){
		phase_tmp = (((float) TIM8->ARR / 360));
		TIM8->CCR2 = TIM8->ARR - (uint32_t) (phase_tmp * 180) - ZERO_PHASE;	//сдвиг на 180
	}

	//Enable timer
	//Tim->CR1|=(TIM_CR1_CEN);

}

