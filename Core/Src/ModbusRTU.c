/*
 * ModbusRTU.c
 *
 *  Created on: Oct 1, 2020
 *      Author: Иван
 */

#include "ModbusRTU.h"


uint16_t ID_Device = 1;// ID_DEV;

unsigned short *DATA_AO[ADR_MAX_AO];
unsigned short *DATA_AI[ADR_MAX_AI];

unsigned short DATA_AO_buf[ADR_MAX_AO] = {0};
unsigned short DATA_AI_buf[ADR_MAX_AI] = {0};

unsigned char buff_rx[Num_Data_buf];
unsigned char buff_tx[Num_Data_buf];

unsigned short Num_Data_RX;
unsigned short Num_Data_TX;
unsigned short UART_ERR;
unsigned short RX_en;
unsigned short MODBUS_FAULT;
unsigned char ID;
unsigned char FK;
unsigned short ADR;
unsigned short NUM_REG;
unsigned char END_FRAME;
unsigned short CRC_modbus;

const unsigned short Crc16Table[256];

void ModbusRTU_UART_Init() {
	MX_UART5_Init(); // поставить нужный
}

void ModbusRTU_Init() {
	ModbusRTU_Init_AO();
	ModbusRTU_Init_AI();
	ModbusRTU_UART_Init();
	MODBUS_FAULT = 0;
	UART_ERR = 0;
	HAL_GPIO_WritePin(UART_DIR_PORT, UART_DIR_PIN, 0);
	__HAL_UART_ENABLE_IT(&UART_MODBUS, UART_IT_IDLE);
	HAL_UART_Receive_IT(&UART_MODBUS, (uint8_t*) buff_rx, Num_Data_buf);
}

void ModbusRTU_Deinit() {
	MODBUS_FAULT = 0;
	UART_ERR = 0;
	HAL_GPIO_WritePin(UART_DIR_PORT, UART_DIR_PIN, 0);
	__HAL_UART_DISABLE_IT(&UART_MODBUS, UART_IT_IDLE);
	HAL_UART_AbortReceive_IT(&UART_MODBUS);

}

void ModbusRTU_Handler() {
	if (RESET != __HAL_UART_GET_FLAG(&UART_MODBUS, UART_FLAG_IDLE)) {
		__HAL_UART_DISABLE_IT(&UART_MODBUS, UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(&UART_MODBUS); //Clear idle interrupt sign (otherwise it will continue to enter interrupt)
		ModbusRTU_Receive();
		ModbusRTU_Transmit();
		__HAL_UART_ENABLE_IT(&UART_MODBUS, UART_IT_IDLE);

	}
}

void ModbusRTU_Receive(){

	Num_Data_RX = Num_Data_buf - UART_MODBUS.RxXferCount;
	CRC_modbus = CRC16(buff_rx, Num_Data_RX);
	HAL_UART_AbortReceive_IT(&UART_MODBUS);

	if (CRC_modbus == 0) {

		// Считываем ИД и функциональную команду
		ID = buff_rx[0];
		FK = buff_rx[1];

		if (ID == ID_Device) // Проверяем ИД, если не совпал чистим буфер
		{

			ADR = buff_rx[2]; // Выделяем адрес первой переменной
			ADR = ADR << 8;
			ADR = ADR | buff_rx[3];

			NUM_REG = buff_rx[4]; // Выделяем количество адресов, так же данные для ФК 5 и 6
			NUM_REG = NUM_REG << 8;
			NUM_REG = NUM_REG | buff_rx[5];

			switch (FK) {
			case 1:
				buff_tx[0] = ID_Device;
				buff_tx[1] = FK | 0x80;
				buff_tx[2] = 2;
				CRC_modbus = CRC16(buff_tx, 3);
				buff_tx[3] = CRC_modbus & 0x00ff;
				buff_tx[4] = CRC_modbus >> 8;
				Num_Data_TX = 5;
				break;
			case 2:
				buff_tx[0] = ID_Device;
				buff_tx[1] = FK | 0x80;
				buff_tx[2] = 2;
				CRC_modbus = CRC16(buff_tx, 3);
				buff_tx[3] = CRC_modbus & 0x00ff;
				buff_tx[4] = CRC_modbus >> 8;
				Num_Data_TX = 5;
				break;
			case 3:
				if ((ADR <= ADR_MAX_AO) && (ADR + NUM_REG <= ADR_MAX_AO)
						&& (NUM_REG > 0)) {
					buff_tx[0] = ID_Device;
					buff_tx[1] = FK;
					buff_tx[2] = 2 * NUM_REG;
					for (int i = 0; i < NUM_REG; i++) {
						buff_tx[3 + 2 * i] = *DATA_AO[ADR + i] >> 8;
						buff_tx[4 + 2 * i] = *DATA_AO[ADR + i] & 0x00ff;
					}
					CRC_modbus = CRC16(buff_tx, 3 + buff_tx[2]);
					buff_tx[3 + buff_tx[2]] = CRC_modbus & 0x00ff;
					buff_tx[4 + buff_tx[2]] = CRC_modbus >> 8;
					Num_Data_TX = 5 + buff_tx[2];
				} else {
					buff_tx[0] = ID_Device;
					buff_tx[1] = FK | 0x80;
					buff_tx[2] = 2;
					CRC_modbus = CRC16(buff_tx, 3);
					buff_tx[3] = CRC_modbus & 0x00ff;
					buff_tx[4] = CRC_modbus >> 8;
					Num_Data_TX = 5;
				}

				break;
			case 4:
				if ((ADR <= ADR_MAX_AI) && (ADR + NUM_REG <= ADR_MAX_AI)
						&& (NUM_REG > 0)) {
					buff_tx[0] = ID_Device;
					buff_tx[1] = FK;
					buff_tx[2] = 2 * NUM_REG;
					for (int i = 0; i < NUM_REG; i++) {
						buff_tx[3 + 2 * i] = *DATA_AI[ADR + i] >> 8;
						buff_tx[4 + 2 * i] = *DATA_AI[ADR + i] & 0x00ff;
					}
					CRC_modbus = CRC16(buff_tx, 3 + buff_tx[2]);
					buff_tx[3 + buff_tx[2]] = CRC_modbus & 0x00ff;
					buff_tx[4 + buff_tx[2]] = CRC_modbus >> 8;
					Num_Data_TX = 5 + buff_tx[2];
				} else {
					buff_tx[0] = ID_Device;
					buff_tx[1] = FK | 0x80;
					buff_tx[2] = 2;
					CRC_modbus = CRC16(buff_tx, 3);
					buff_tx[3] = CRC_modbus & 0x00ff;
					buff_tx[4] = CRC_modbus >> 8;
					Num_Data_TX = 5;
				}
				break;

			case 5:
				buff_tx[0] = ID_Device;
				buff_tx[1] = FK | 0x80;
				buff_tx[2] = 2;
				CRC_modbus = CRC16(buff_tx, 3);
				buff_tx[3] = CRC_modbus & 0x00ff;
				buff_tx[4] = CRC_modbus >> 8;
				Num_Data_TX = 5;
				break;

			case 6:
				if (ADR <= ADR_MAX_AO) {
					*DATA_AO[ADR] = NUM_REG;
					buff_tx[0] = buff_rx[0];
					buff_tx[1] = buff_rx[1];
					buff_tx[2] = buff_rx[2];
					buff_tx[3] = buff_rx[3];
					buff_tx[4] = buff_rx[4];
					buff_tx[5] = buff_rx[5];
					buff_tx[6] = buff_rx[6];
					buff_tx[7] = buff_rx[7];
					Num_Data_TX = 8;
				} else {
					buff_tx[0] = ID_Device;
					buff_tx[1] = FK | 0x80;
					buff_tx[2] = 2;
					CRC_modbus = CRC16(buff_tx, 3);
					buff_tx[3] = CRC_modbus & 0x00ff;
					buff_tx[4] = CRC_modbus >> 8;
					Num_Data_TX = 5;
				}
				break;

			case 15:
				buff_tx[0] = ID_Device;
				buff_tx[1] = FK | 0x80;
				buff_tx[2] = 2;
				CRC_modbus = CRC16(buff_tx, 3);
				buff_tx[3] = CRC_modbus & 0x00ff;
				buff_tx[4] = CRC_modbus >> 8;
				Num_Data_TX = 5;
				break;
			case 16:

				if ((ADR <= ADR_MAX_AO) && (ADR + NUM_REG <= ADR_MAX_AO)
						&& (NUM_REG > 0)) {
					for (int i = 0; i < NUM_REG; i++) {
						*DATA_AO[ADR + i] = buff_rx[7 + 2 * i] << 8;
						*DATA_AO[ADR + i] |= buff_rx[8 + 2 * i];
					}
					buff_tx[0] = ID_Device;
					buff_tx[1] = FK;
					buff_tx[2] = buff_rx[2];
					buff_tx[3] = buff_rx[3];
					buff_tx[4] = buff_rx[4];
					buff_tx[5] = buff_rx[5];
					CRC_modbus = CRC16(buff_tx, 6);
					buff_tx[6] = CRC_modbus & 0x00ff;
					buff_tx[7] = CRC_modbus >> 8;
					Num_Data_TX = 8;
				} else {
					buff_tx[0] = ID_Device;
					buff_tx[1] = FK | 0x80;
					buff_tx[2] = 2;
					CRC_modbus = CRC16(buff_tx, 3);
					buff_tx[3] = CRC_modbus & 0x00ff;
					buff_tx[4] = CRC_modbus >> 8;
					Num_Data_TX = 5;
				}
				break;

			default:

				break;
			}
		} else {
			HAL_UART_Receive_IT(&UART_MODBUS, (uint8_t*) buff_rx, Num_Data_buf);
			//HAL_UART_Receive_IT(&UART_MODBUS, buff_rx, Num_Data_buf);
		}
	} else {
		HAL_UART_Receive_IT(&UART_MODBUS, (uint8_t*) buff_rx, Num_Data_buf);
		//HAL_UART_Receive_IT(&UART_MODBUS, buff_rx, Num_Data_buf);
	}
}

void ModbusRTU_Transmit() {
	if (Num_Data_TX > 0) {
		HAL_GPIO_WritePin(UART_DIR_PORT, UART_DIR_PIN, 1);
		//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		HAL_UART_Transmit_IT(&UART_MODBUS, (uint8_t*) buff_tx, Num_Data_TX);
		Num_Data_TX=0;
		//HAL_UART_Transmit_IT(&UART_MODBUS, buff_tx, Num_Data_TX);

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &UART_MODBUS) {
		ModbusRTU_Receive(); //что-то делаем
		ModbusRTU_Transmit();

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &UART_MODBUS) {
		HAL_GPIO_WritePin(UART_DIR_PORT, UART_DIR_PIN, 0);
		//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		Num_Data_TX = 0;


		HAL_UART_Receive_IT(&UART_MODBUS, (uint8_t*) buff_rx, Num_Data_buf);

		//HAL_UART_Receive_IT(&UART_MODBUS, buff_rx, Num_Data_buf);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == &UART_MODBUS) {
		MODBUS_FAULT = 2;
		UART_ERR = HAL_UART_GetError(&UART_MODBUS);

		/*   switch(er)
		 {
		 case HAL_UART_ERROR_PE:

		 __HAL_UART_CLEAR_PEFLAG(&UART_MODBUS);
		 huart->ErrorCode = HAL_UART_ERROR_NONE;
		 break;

		 case HAL_UART_ERROR_NE:

		 __HAL_UART_CLEAR_NEFLAG(&UART_MODBUS);
		 huart->ErrorCode = HAL_UART_ERROR_NONE;
		 break;

		 case HAL_UART_ERROR_FE:

		 __HAL_UART_CLEAR_FEFLAG(&UART_MODBUS);
		 huart->ErrorCode = HAL_UART_ERROR_NONE;
		 break;

		 case HAL_UART_ERROR_ORE:

		 __HAL_UART_CLEAR_OREFLAG(&UART_MODBUS);
		 huart->ErrorCode = HAL_UART_ERROR_NONE;
		 break;

		 case HAL_UART_ERROR_DMA:

		 huart->ErrorCode = HAL_UART_ERROR_NONE;
		 break;

		 default:
		 break;
		 }*/
	}
}

float ModbusRTU_Conv_Out_f(unsigned short in_x, float in_Min, float in_Max, float out_Min, float out_Max){
	float x;
	if(in_Min>=0)
	x = (float)in_x;
	else
		x = (float) ((short) in_x);
	if(x>in_Max)
		x=in_Max;
	else if(x<in_Min)
		x=in_Min;
	return (x-in_Min)*(out_Max-out_Min) / (in_Max-in_Min)+out_Min;
}

unsigned short ModbusRTU_Conv_Out_us(float in_x, float in_Min, float in_Max, float out_Min, float out_Max){
	unsigned short x;

	if(in_x>in_Max)
		in_x=in_Max;
	else if(in_x<in_Min)
		in_x=in_Min;

	in_x =(in_x-in_Min)*(out_Max-out_Min) / (in_Max-in_Min)+out_Min;

	if(in_Min>=0)
		x = (unsigned short)in_x;
		else
			x = (unsigned short) ((short) in_x);

	return x;
}

unsigned short CRC16(unsigned char * pcBlock, unsigned short len)
{
    unsigned short crc = 0xFFFF;

    while (len--)
        crc = (crc >> 8) ^ Crc16Table[(crc & 0xFF) ^ *pcBlock++];

    return crc;
}

const unsigned short Crc16Table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
