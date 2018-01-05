/*
 * CONFIG.H
 *
 *  Created on: 2015-10-21
 *      Author: admin
 */
#ifndef CONFIG_H
#define CONFIG_H

#include "stdint.h"         // Define standard integers.

#pragma REGPARMS					// Enable register parameter passing.

//User data struct
#define RX_BUF_SIZE 30

typedef struct
{
	uint8_t		framehead;
	uint8_t 	address;
	uint8_t 	command;
	uint8_t 	length;
	uint8_t 	serialData[RX_BUF_SIZE];
	uint8_t 	checksum;
	uint8_t 	frameend;
}  RECEIVEFRAME;

#define CAT24WCXX_ADDR	0xa0		//EEPROM ������ַ
#define PARA_NUM	21
//*********************������EEPROM�еĵ�ַ**********************
#define	EEPROM_NEW_FLAG				0									//��EEPROM�жϱ�־

#define	EEPROM_ENERGY_START			10
#define	EEPROM_TDK6545H_PARA_ADDR0	(EEPROM_ENERGY_START )				//У������								//27 byte
#define	EEPROM_TDK6545H_PARA_ADDR1	(EEPROM_TDK6545H_PARA_ADDR0 + 40)	//40 byte

#define EEPROM_ENERGY_WH_PT			(EEPROM_TDK6545H_PARA_ADDR1 + 40 )	// ����
#define EEPROM_ENERGY_WH_NT			(EEPROM_ENERGY_WH_PT + 16)			// 16 byte
#define EEPROM_ENERGY_VARH_Q1		(EEPROM_ENERGY_WH_NT + 16)			// 16 byte
#define EEPROM_ENERGY_VARH_Q2		(EEPROM_ENERGY_VARH_Q1 + 16)		// 16 byte
#define EEPROM_ENERGY_VARH_Q3		(EEPROM_ENERGY_VARH_Q2 + 16)		// 16 byte
#define EEPROM_ENERGY_VARH_Q4		(EEPROM_ENERGY_VARH_Q3 + 16)		// 16 byte
#define EEPROM_CAL_FINISHED			(EEPROM_ENERGY_VARH_Q4 + 16)		// 1 byte
#endif