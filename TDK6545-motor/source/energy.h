/*
 * energy.h
 *
 *  Created on: 2015-11-1
 *      Author: admin
 */
#ifndef ENERGY_H
#define ENERGY_H

#include "options.h"            // Define the IC, CE code and options.
#include <string.h>
#include <math.h>
#include "CONFIG.H"

#define	VMAX	366
#define IMAX	69
#define NOT_TELL	DIO_9 = 1	//��MCU����
#define	TELL_MCU	DIO_9 = 0
#define IADCN_MAX_SAMPLES		50	//��IADCԭʼֵ����������

#define BIT0	0x01
#define BIT1	0x02
#define BIT2	0x04
#define BIT3	0x08
#define BIT4	0x10
#define BIT5	0x20
#define BIT6	0x40
#define BIT7	0x80



extern	uint32_t g_I_Temp[5];//����
extern	uint16_t g_V_Temp[3];//���ѹ
extern	uint16_t g_V_Line_Temp[3];	//�ߵ�ѹ
extern	uint32_t g_PW_T[4];
extern	uint32_t g_PVar_T[4];
extern	uint32_t g_Wh_PT[5],g_Wh_NT[5];//[0][1][2]ΪABC�����[3]Ϊ��[4]Ϊ[3]�ı���
extern	uint32_t g_Varh_Q1[5],g_Varh_Q2[5],g_Varh_Q3[5],g_Varh_Q4[5];//�й�������,ֻ����������
extern 	bool 	g_EnergyClear;
extern  uint8_t Bakbuf[PARA_NUM];
extern  uint8_t code TDK6545PARAADDR[10];
extern  uint8_t net_status;				//����״̬��
extern	uint8_t net_status_lock;
extern  uint32_t n_Current;			//���ߵ������
extern  uint8_t cal_finished;
extern	int32_t  iadcN[IADCN_MAX_SAMPLES];	//N ADC��ԭʼֵbuf
extern	uint8_t	 iadcN_p;
extern  uint8_t EarthDelayStar; 	//���߼�⿪ʼ��ʶ
extern  uint8_t EarthDelayCnt;		//���߼�ⳬʱʱ��
extern  uint8_t	EarthDelayOut;			//���߼�ⳬʱ��ʶ
extern 	void Init_Xfer(void);
extern  void earth_dec();
extern  void Convert_Ncurrent(void);
void earth_dec_init(void);

#endif
