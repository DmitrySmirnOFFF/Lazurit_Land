/*
 * ModbusRTU.h
 *
 *  Created on: Oct 1, 2020
 *      Author: Иван
 */

#include "stm32f4xx_hal.h"
#include "usart.h"

#ifndef INC_MODBUSRTU_H_
#define INC_MODBUSRTU_H_

// Modbus init
#define ID_DEV 1 //по умолчанию
#define ADR_MAX_AO 10
#define ADR_MAX_AI 10

#define UART_MODBUS huart5
#define UART_DIR_PORT UART5_DIR_GPIO_Port
#define UART_DIR_PIN UART5_DIR_Pin


#define Num_Data_buf 100

// Для работы библиотеки модбас
extern unsigned short DATA_AO_buf[ADR_MAX_AO];
extern unsigned short DATA_AI_buf[ADR_MAX_AI];

extern unsigned short *DATA_AO[ADR_MAX_AO];
extern unsigned short *DATA_AI[ADR_MAX_AI];


extern unsigned short MODBUS_FAULT;
extern uint16_t ID_Device;

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void ModbusRTU_Init();
void ModbusRTU_Deinit();
void ModbusRTU_Handler();
void ModbusRTU_Transmit();
void ModbusRTU_Receive();

void ModbusRTU_UART_Init();
void ModbusRTU_Init_AO();
void ModbusRTU_Init_AI();
void ModbusRTU_Data_In();
void ModbusRTU_Data_Out();
unsigned short ModbusRTU_Conv_Out_us(float in_x, float in_Min, float in_Max, float out_Min, float out_Max);
float ModbusRTU_Conv_Out_f(unsigned short in_x, float in_Min, float in_Max, float out_Min, float out_Max);

unsigned short CRC16(unsigned char * pcBlock, unsigned short len);
#endif /* INC_MODBUSRTU_H_ */



/* поместить и заполнить в main.h

// Modbus init
#define ID_Device 2
#define ADR_MAX_AI 5
#define ADR_MAX_AO 5
#define UART_MODBUS huart6
#include "ModbusRTU.h"

// Для работы библиотеки модбас
extern  unsigned short *DATA_AO[ADR_MAX_AO];
extern  unsigned short *DATA_AI[ADR_MAX_AI];
extern  unsigned short MODBUS_FAULT;

*/


/* поместить и заполнить stm32f4xx_it.c
ModbusRTU_Handler();


void USART6_IRQHandler(void)
{
   USER CODE BEGIN USART6_IRQn 0
	ModbusRTU_Handler();!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   USER CODE END USART6_IRQn 0
  HAL_UART_IRQHandler(&huart6);
   USER CODE BEGIN USART6_IRQn 1

   USER CODE END USART6_IRQn 1
}


*/


/* поместить и заполнить в main.c "user code 4"


void ModbusRTU_UART_Init() {
	MX_USART6_UART_Init(); // поставить нужный
}

void ModbusRTU_Init_AO() {
	// Определение переменных
	DATA_AO[0] = &RELE_ON_State;

}
void ModbusRTU_Init_AI() {
	// Определение переменных
	DATA_AI[0] = &U_zpt_State;

}

// использовать в нужном месте
void ModbusRTU_Data_Out(){
//согласование принимаемых переменных
float var = ModbusRTU_Conv_Out_f(in_var, float in_Min, float in_Max, float out_Min, float out_Max)
}

// использовать в нужном месте
void ModbusRTU_Data_In(){
//согласование отправляемых переменных
unsigned short var = ModbusRTU_Conv_Out_us(in_var, float in_Min, float in_Max, float out_Min, float out_Max)
}


*/


unsigned short ModbusRTU_Conv_Out_us(float in_x, float in_Min, float in_Max, float out_Min, float out_Max);
float ModbusRTU_Conv_Out_f(unsigned short in_x, float in_Min, float in_Max, float out_Min, float out_Max);


/*
 UART_ERR = 1   =>   HAL_UART_ERROR_PE — ошибка контроля чётности.
 UART_ERR = 2   =>   HAL_UART_ERROR_NE — шум в линии.
 UART_ERR = 4   =>   HAL_UART_ERROR_FE — ошибка кадра. У передающего устройства «плавает» частота.
 UART_ERR = 8   =>   HAL_UART_ERROR_ORE — ошибка переполнения.
 Возникает когда в USART приходит новый байт, а предыдущий ещё не был считан из
 приёмного регистра — Data Register (DR). Это происходит из-за того,
 что прерывание обрабатывается слишком медленно и программа
 не успевает очистить приёмный регистр.
 */
