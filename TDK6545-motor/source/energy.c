#include "options.h"            // Define the IC, CE code and options.
#include <string.h>
#include <math.h>
#include "CONFIG.H"
#include "eeprom.h"
#include "energy.h"

#define EARTH_DEC	DIO_10

/***********************CE����******************************/
#define PULSE_SCARE		1.0149
#define N_ACC 2184
#define Fs (32768/15)
#define V_RMS_CONST		(1.0856e-12*VMAX*VMAX)
#define	I_RMS_CONST		(1.0856e-12*IMAX*IMAX)
#define E_CONST			(1.0856e-12*VMAX*IMAX)
#define ENERY_PER_UNIT 	(long)(10/(1.0856e-12*VMAX*IMAX))//xxxxxx.xx��ȷ��0.01kwh
#define IADC01_PRE_AMPLIFY 	8	//IADC01��ǰ�÷Ŵ�������PRE_E���ơ�

/*************************����***********************************/
#define MAX_V_3P4W  50000   // 500V ���ѹ�Ϸ����ж����ֵ
#define MAX_CURRENT	IMAX*100000	//��ȷ��0.001A���Գ���1000
#define MIN_CURRENT_POINT_5 500	   	// 0.5A
#define MIN_CURRENT_POINT_2 100		// 0.1A
#define	MIN_VOL_50		 5000		//50v
#define WATT_CREEP 5200       //Ǳ��ֵ
/*************************���ߵ���ÿ�ܲ�����****************************************/
//315416  ��CTʱֵ��10~12֮�䲨��
//318603  δ��CTʱֵ��6~7
//398254  Ϊ50������ֵ�����ƽ��ֵ
#define DC_BIAS		97624
#define N_SCARE		3665	//ԭʼֵ��С����

//**********************��������**********************************
uint32_t g_I_Temp[5];//����,��©�������ABCN,N(ADC)
uint16_t g_V_Temp[3];//���ѹ0.01v
uint16_t g_V_Line_Temp[3];	//�ߵ�ѹ
uint32_t g_PW_T[4];
uint32_t g_PVar_T[4];
int16_t g_Cos_T[4];			//ABC�ܹ�������
uint16_t g_PhUI_T[3];     	//0.01 ��
uint16_t g_PhUU_T[2];    	// 0.01 ��
uint32_t tem_Wh_P,tem_Wh_N;			//�й������� �ܵ�������
uint32_t tem_Varh_P,tem_Varh_N;		//�޹������� �ܵ�������
uint32_t temA_Wh_P,temA_Wh_N,temB_Wh_P,temB_Wh_N,temC_Wh_P,temC_Wh_N;//�й������� ABC��������
uint32_t temA_Varh_P,temA_Varh_N,temB_Varh_P,temB_Varh_N,temC_Varh_P,temC_Varh_N;//�޹������� ABC��������
uint32_t g_Wh_PT[5],g_Wh_NT[5];//[0][1][2]ΪABC�����[3]Ϊ��[4]Ϊ[3]�ı���,[3]ʵʱ����,[4]��ʵʱ����
uint32_t g_Varh_Q1[5],g_Varh_Q2[5],g_Varh_Q3[5],g_Varh_Q4[5];//�й�������,ֻ����������
bool 	 g_EnergyClear;				//�������

int32_t  iadcN[IADCN_MAX_SAMPLES];	//N ADC��ԭʼֵbuf
uint8_t	 iadcN_p;					//����ֵ����IADCN_MAX_SAMPLES��ʱ����һ�λ��֡�
int32_t  n_current_adc;
int32_t  dc_bias;	//ֱ��ƫ��



//***********************У��*************************
#define FAC						    DIO_5		//Ӳ��У�����
#define D_ERR_MODE                	0x00        //������ʾģʽ
#define D_NORMAL_MODE		      	0x10	    //��������ģʽ
#define D_CAL_START_MODE		    0x21	    //У��ģʽ������
#define D_CAL_END_MODE		        0x23	    //У��ģʽ�����
#define FAC_EN						0			//Ӳ��У������
#define FAC_DIS						0x80		//Ӳ��У������
#define	FAC_STABLE_TIME				10			//FAC����10����Ч�ſ�ʼУ��
#define CAL_POWER_MIN				15400		//2200*0.7W У��ʱ����������
#define	CAL_POWER_MAX				28600		//2200*1.3W	У��ʱ����������
#define CAL_POWER_DEVIATION			50			//������� 5W
#define	AV_POWER_N					3			//���ʻ�������С��У����
#define	STAND_V						22000		//��׼��ѹ220.00V У����
#define	STAND_I						10000		//��׼���� 10.000AУ����
#define	STAND_P						22000		//��׼���� 2200.0WУ����

uint8_t Bakbuf[PARA_NUM];
uint8_t code TDK6545PARAADDR[10] = {0x10,0x11,0x13,0x14,0x16,0x17,0x12,0x15,0x18,0x21};

uint8_t    	U8_CURR_WorkMode;
uint16_t 	U16_FAC_STABLE_Times;				//�ж�FAC�ȶ��Ĵ�����10��ȥ��
uint32_t 	g_PW_T_Buf[3][AV_POWER_N];			//�������N�ι���ֵ (ABC����)

/************************************************************/
uint8_t net_status;				//����״̬�� bit0 ���߼��
uint8_t net_status_lock;		//����״̬�����棬ͨѶ������ܸ��¡�
uint32_t n_Current;			//���ߵ������
uint8_t cal_finished;		//У����ɱ�־ 1����У  0��δУ
uint8_t EarthDelayStar; 	//���߼�⿪ʼ��ʶ
uint8_t EarthDelayCnt;		//���߼�ⳬʱʱ��
uint8_t	EarthDelayOut;			//���߼�ⳬʱ��ʶ

//***********************����***************************
void Cal_Energy(void);

void Convert_Ncurrent(void)
{
	uint8_t i;
	int32_t sum = 0;
	int32_t sum_with_bias = 0;
	float float_data;
	static int32_t n_current_old = 0;

	if(iadcN_p == (IADCN_MAX_SAMPLES))//���һ���ڲ���
	{
		IEN1 &= (~EX3_);//����CEBUSY�ж�
		for(i=0 ; i < IADCN_MAX_SAMPLES ;i++)
		{
			iadcN[i] /= IADC01_PRE_AMPLIFY;//PRE_E =1 ����ǰ�÷Ŵ���
			sum_with_bias += iadcN[i];
		}
		dc_bias = sum_with_bias/IADCN_MAX_SAMPLES;
		for(i=0 ; i < IADCN_MAX_SAMPLES ;i++)
			iadcN[i] =(iadcN[i]-dc_bias);//��ֹ���.
		for(i=0 ; i < IADCN_MAX_SAMPLES ;i++)
			iadcN[i] /= N_SCARE;
		for(i=0 ; i < IADCN_MAX_SAMPLES ;i++)
			sum += (iadcN[i]*iadcN[i]/IADCN_MAX_SAMPLES);
		float_data = sqrt(sum);
		n_current_adc = (float_data);//����������ת������
		if(n_current_adc >= MAX_CURRENT)
			n_current_adc = 0;
		g_I_Temp[4] = n_current_adc;
		if(n_current_adc >= n_Current && n_Current != 0 && n_current_old >=n_Current)//�ж��Ƿ�©��
			{
				net_status |= 0x02; //bit1 ��1
				if(net_status != 0)
					TELL_MCU;
				net_status_lock |= 0x02;
			}
		else
			{
				net_status &= 0xfd; //bit1 ��0
				if(net_status == 0)
					NOT_TELL;
			}
		n_current_old =n_current_adc;
		iadcN_p = 0;//ָ���0����֡������ж��ж����״̬�ſ��������ж�
	}

}

int16_t Cal_Cos(int32_t w, int32_t var, uint8_t phase)
{
	int16_t Cos_T;
	float floatdata;
	if(phase != 3)
	{
		if(g_I_Temp[phase] > MIN_CURRENT_POINT_5)
			floatdata = 1000 * (w>>13) / (sqrt( (w>>13)*(w>>13) + (var>>13) * (var>>13)));
		else if(g_I_Temp[phase] <= MIN_CURRENT_POINT_5 && g_I_Temp[phase] >= MIN_CURRENT_POINT_2)
			floatdata = 1000 * (w>>9) / (sqrt( (w>>9)*(w>>9) + (var>>9) * (var>>9) ));
		else
			floatdata = 1000;//������Сʱ��Ĭ�Ϲ�������Ϊ1
	}
	else
	{
		if(g_I_Temp[0] < MIN_CURRENT_POINT_2 && g_I_Temp[1] < MIN_CURRENT_POINT_2 && g_I_Temp[2] < MIN_CURRENT_POINT_2)
			floatdata = 1000;//�����������Сʱ��Ĭ�Ϲ�������Ϊ1
		else
		{
			if(g_I_Temp[0] > MIN_CURRENT_POINT_5 || g_I_Temp[1] > MIN_CURRENT_POINT_5 || g_I_Temp[2] > MIN_CURRENT_POINT_5)
				floatdata = 1000 * (w>>13) / (sqrt( (w>>13)*(w>>13) + (var>>13) * (var>>13)));
			else
				floatdata = 1000 * (w>>9) / (sqrt( (w>>9)*(w>>9) + (var>>9) * (var>>9) ));
		}
	}
    Cos_T = floatdata;//����ת��
	return Cos_T;
}

int16_t Cal_phUI(int32_t var, int16_t cos_t)
{
	uint32_t temp;
	int16_t phui_t;

    temp = acos(cos_t / 1000.00000) * 18000.00 / 3.1416;
    if(var < 0)
		temp = 36000 - temp;
    phui_t = temp;

    return phui_t;
}

void Xfer_Process(void)
{
	uint8_t	i,j;
	uint32_t temp;
	int32_t temp_sum[4];//��ʱ�������� ������
	int32_t pulse_sum;
	//float f_sum;
	RESET_WD();
	// ����Ǳ��Test use
    if (labs(w0sum) < WATT_CREEP)
    {
        w0sum = 0;
        var0sum = 0;
    }
    if (labs(w1sum) < WATT_CREEP)
    {
        w1sum = 0;
        var1sum = 0;
    }
    if (labs(w2sum) < WATT_CREEP)
    {
        w2sum = 0;
        var2sum = 0;
    }

    // ����Դ�Ĵ���Feed the CE's pulse generator.
    pulse_sum = w0sum + w1sum + w2sum;
    pulse_sum *= PULSE_SCARE;
    apulsew = pulse_sum;
    pulse_sum = var0sum + var1sum + var2sum;
    pulse_sum *= PULSE_SCARE;
    apulser = pulse_sum;

	//==========�й�����  ����Ϊ0.1W===============
    temp_sum[0] = w0sum;
    temp_sum[1] = w1sum;
    temp_sum[2] = w2sum;
    temp_sum[3] = wsum;

	g_PW_T[3]=0;
    for(i=0; i<3; i++)//�й��ܹ���   ����Ϊ0.1W �� 0.0001kW
    {
		g_PW_T[i] = (temp_sum[i] * (E_CONST*10) * 3600 * Fs / N_ACC);
		///g_PW_T[i]-=g_PW_T[i]%10;
		g_PW_T[3]+=g_PW_T[i];		//lz modify 20100612
	}
    //�������N��ABC���ʣ�����ķ�[i][0]
    for(i=0 ;i<3; i++)//���κ���
    {
    	for(j=AV_POWER_N-1; j>=1; j--)
    		g_PW_T_Buf[i][j] = g_PW_T_Buf[i][j-1];
    }
    for(i=0 ;i<3; i++)
    	g_PW_T_Buf[i][0] = g_PW_T[i];

	//=========�޹����� 0.01Var==============
    temp_sum[0] = var0sum;
    temp_sum[1] = var1sum;
    temp_sum[2] = var2sum;
    temp_sum[3] = varsum;

    g_PVar_T[3]=0;
    for(i=0; i<3; i++)
	{
		temp= temp_sum[i] * (E_CONST * 10)* 3600 * Fs / N_ACC ;//�޹�����
		g_PVar_T[i] = (int32_t)temp;
		//g_PVar_T[i]-=g_PVar_T[i]%10;
		g_PVar_T[3]+=g_PVar_T[i];	//lz modify 20100612
	}

	//========��ѹ ����Ϊ0.01V==============
	temp_sum[0] = v0sqsum;
	temp_sum[1] = v1sqsum;
	temp_sum[2] = v2sqsum;
    for(i=0; i<3; i++)
	{
    	temp = sqrt( temp_sum[i] * (V_RMS_CONST *10000)* 3600 * Fs / N_ACC); //����Ϊ0.01V "*10000"��Ϊ�˾�ȷ��0.01V
		if(temp<MAX_V_3P4W)
			g_V_Temp[i] = (uint16_t)temp;
	}
    //========��������Ϊ0.001A==========
    temp_sum[0] = i0sqsum;
    temp_sum[1] = i1sqsum;
    temp_sum[2] = i2sqsum;
    temp_sum[3] = idsqsum;
    for(i=0; i<4; i++)
	{
		temp = sqrt( temp_sum[i] * (I_RMS_CONST * 1000000) * 3600 * Fs / N_ACC); //����Ϊ0.001A
		if(temp < MAX_CURRENT)
		{
			if(i == 3)
				g_I_Temp[i] = (temp/IADC01_PRE_AMPLIFY);
			else
				g_I_Temp[i] = temp;
		}
	}

    //=====�������� ���====================
    g_Cos_T[0] = Cal_Cos(w0sum, var0sum, 0);
    g_PhUI_T[0] = Cal_phUI(var0sum, g_Cos_T[0]);
	g_Cos_T[1] = Cal_Cos(w1sum, var1sum, 1);
	g_PhUI_T[1] = Cal_phUI(var1sum, g_Cos_T[1]);
	g_Cos_T[2] = Cal_Cos(w2sum, var2sum, 2);
	g_PhUI_T[2] = Cal_phUI(var2sum, g_Cos_T[2]);//�����ѹ������н�
	g_Cos_T[3] = Cal_Cos(wsum, varsum, 3);

	//=====��ѹ��� A-B,A-C==================
	//����50V���жϵ�ѹ���
	if( g_V_Temp[0] >= MIN_VOL_50 && g_V_Temp[1] >= MIN_VOL_50 )
	{
		temp =(uint32_t) (ph_atob*360/N_ACC+(2.4*15.0/13.0))*100;
		g_PhUU_T[0] = (uint16_t)temp;
	}
	else
		g_PhUU_T[0] = 0;

	if( g_V_Temp[0] >= MIN_VOL_50 && g_V_Temp[2] >= MIN_VOL_50 )
	{
		temp =(uint32_t) (ph_atoc*360/N_ACC+(4.8*15.0/13.0))*100;
		g_PhUU_T[1] = (uint16_t)temp;
	}
	else
		g_PhUU_T[1] = 0;
	//=====©���ж�====================
/*	if(g_I_Temp[3] > n_Current && n_Current != 0)
	{
		//�ų�ABCͬ��λ������ӷ�
		if( (g_PhUU_T[0] >= 20 && g_PhUU_T[0] <= 340) || (g_PhUU_T[1] >= 20 && g_PhUU_T[1] <= 340) )
		{
			net_status &= 0xfd; //bit1 ��0
			if(net_status == 0)
				NOT_TELL;

		}
		else
		{
			net_status |= 0x02; //bit1 ��1
			if(net_status != 0)
				TELL_MCU;
		}
		//�����϶γ�������Ϊ��ʹʹ��ͬ��ӷ�������ʱ©�������Ϊ0 2016-11-15
		net_status |= 0x02; //bit1 ��1
		if(net_status != 0)
			TELL_MCU;
	}
	else
	{
		net_status &= 0xfd; //bit1 ��0
		if(net_status == 0)
			NOT_TELL;

	}*/
	//�ۼǵ���
    Cal_Energy();
    //�Ƿ���Ҫ�����
	if(g_EnergyClear == TRUE)
		{
			RESET_WD();
			g_EnergyClear = FALSE;
			OSEEPROMWrite(EEPROM_ENERGY_WH_PT, CAT24WCXX_ADDR,(uint8_t*)&g_Wh_PT[0],16);
			OSEEPROMWrite(EEPROM_ENERGY_WH_NT, CAT24WCXX_ADDR,(uint8_t*)&g_Wh_NT[0],16);
			OSEEPROMWrite(EEPROM_ENERGY_VARH_Q1, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q1[0],16);
			OSEEPROMWrite(EEPROM_ENERGY_VARH_Q2, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q2[0],16);
			OSEEPROMWrite(EEPROM_ENERGY_VARH_Q3, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q3[0],16);
			OSEEPROMWrite(EEPROM_ENERGY_VARH_Q4, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q4[0],16);
		}
}

void Cal_Energy(void)
{
	//�й�
	//Accumulate Wh energy
	if(wsum<0)		 		// Total
		tem_Wh_N-=wsum;//�й�����������
	else
		tem_Wh_P+=wsum;//�й�����������

	if(w0sum <0)		 	//A
		temA_Wh_N-=w0sum;//�й�����A������
	else
		temA_Wh_P+=w0sum;//�й�����A������

	if(w1sum < 0)		 	//B
		temB_Wh_N-=w1sum;//�й�����B������
	else
		temB_Wh_P+=w1sum;//�й�����B������

	if(w2sum< 0)		 //C
		temC_Wh_N -= w2sum;//�й�����C������
	else
		temC_Wh_P += w2sum;//�й�����C������

	//�޹�
	//Accumulate Varh energy
	if(varsum<0)		 	// Total
		tem_Varh_N-=varsum;
	else
		tem_Varh_P+=varsum;

	if(var0sum < 0)		 	//A
		temA_Varh_N-=var0sum;
	else
		temA_Varh_P+=var0sum;

	if(var1sum < 0)		 	//B
		temB_Varh_N-=var1sum;
	else
		temB_Varh_P+=var1sum;

	if(var2sum < 0)		 //C
		temC_Varh_N-=var2sum;
	else
		temC_Varh_P+=var2sum;

	//���潫β������0.01kwh�Ĳ����ۼӵ�������.

    while(tem_Wh_P > ENERY_PER_UNIT)
	{
		tem_Wh_P -= ENERY_PER_UNIT;
		//�����й���
        g_Wh_PT[3] ++;
   	}

	while(temA_Wh_P > ENERY_PER_UNIT)
	{
		temA_Wh_P -= ENERY_PER_UNIT;
		//�����й�A
	    g_Wh_PT[0] ++;
    }

	 while(temB_Wh_P > ENERY_PER_UNIT)
	{
		temB_Wh_P -=  ENERY_PER_UNIT;
		//�����й�B
		g_Wh_PT[1] ++;
	}

	while(temC_Wh_P > ENERY_PER_UNIT)
	{
		temC_Wh_P -= ENERY_PER_UNIT;
		//�����й�C
		g_Wh_PT[2] ++;
	}

	//====����=================
	while(tem_Wh_N > ENERY_PER_UNIT)
	{
		tem_Wh_N -= ENERY_PER_UNIT;
		//�����й���
       	g_Wh_NT[3] ++;
	}
	while(temA_Wh_N > ENERY_PER_UNIT)
	{
		temA_Wh_N -= ENERY_PER_UNIT;
		//�����й�A
       	g_Wh_NT[0] ++;
	}
	while(temB_Wh_N > ENERY_PER_UNIT)
	{
		temB_Wh_N -= ENERY_PER_UNIT;
		//�����й�B
		g_Wh_NT[1] ++;
	}

	while(temC_Wh_N > ENERY_PER_UNIT)
	{
		temC_Wh_N -= ENERY_PER_UNIT;
		//�����й�C
       	g_Wh_NT[2] ++;
	}
	//==========�����޹�===============
	while(tem_Varh_P > ENERY_PER_UNIT)
	{
		tem_Varh_P -= ENERY_PER_UNIT;
		if(wsum>0)
			g_Varh_Q1[3]++;
		else
			g_Varh_Q2[3]++;
    }

	while(temA_Varh_P > ENERY_PER_UNIT)
	{
		temA_Varh_P -= ENERY_PER_UNIT;
		if(w0sum > 0)
			g_Varh_Q1[0]++;//�й��� �޹��� ��һ����
		else
			g_Varh_Q2[0]++;//�й��� �޹��� �ڶ�����
    }

	while(temB_Varh_P > ENERY_PER_UNIT)
	{
		temB_Varh_P -= ENERY_PER_UNIT;
		if(w1sum > 0)
			g_Varh_Q1[1]++;//ͬA��
		else
			g_Varh_Q2[1]++;
	}

	while(temC_Varh_P > ENERY_PER_UNIT)
	{
		temC_Varh_P -= ENERY_PER_UNIT;
		if(w2sum > 0)
			g_Varh_Q1[2]++;
		else
			g_Varh_Q2[2]++;
    }
	//============�����޹�========================
	while(tem_Varh_N > ENERY_PER_UNIT)
	{
		tem_Varh_N -= ENERY_PER_UNIT;
		if(wsum < 0)
			g_Varh_Q3[3]++;
		else
			g_Varh_Q4[3]++;
	}

	while(temA_Varh_N > ENERY_PER_UNIT)
	{
		temA_Varh_N -= ENERY_PER_UNIT;
		if(w0sum < 0)
			g_Varh_Q3[0]++;	//�й��� �޹��� λ�ڵ�3����
		else
			g_Varh_Q4[0]++; //�й��� �޹��� λ�ڵ�3����
	}

	while(temB_Varh_N > ENERY_PER_UNIT)
	{
		temB_Varh_N -= ENERY_PER_UNIT;
		if(w1sum < 0)
			g_Varh_Q3[1]++;
		else
			g_Varh_Q4[1]++;
	}

	while(temC_Varh_N > ENERY_PER_UNIT)
	{
		temC_Varh_N -= ENERY_PER_UNIT;
		if(w2sum < 0)
			g_Varh_Q3[2]++;
		else
			g_Varh_Q4[2]++;
	}
}

void SaveEnergy(uint32_t addon)
{
	RESET_WD();

	if(g_Wh_PT[3] > (g_Wh_PT[4] + addon))
	{
		g_Wh_PT[4] = g_Wh_PT[3];
		OSEEPROMWrite(EEPROM_ENERGY_WH_PT, CAT24WCXX_ADDR,(uint8_t*)&g_Wh_PT[0],16);
	}

	if(g_Wh_NT[3] > (g_Wh_NT[4] + addon))
	{
		g_Wh_NT[4] = g_Wh_NT[3];
		OSEEPROMWrite(EEPROM_ENERGY_WH_NT, CAT24WCXX_ADDR,(uint8_t*)&g_Wh_NT[0],16);
	}

	if(g_Varh_Q1[3] > (g_Varh_Q1[4] + addon))
	{
		g_Varh_Q1[4] = g_Varh_Q1[3];
		OSEEPROMWrite(EEPROM_ENERGY_VARH_Q1, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q1[0],16);
	}

	if(g_Varh_Q2[3] > (g_Varh_Q2[4] + addon))
	{
		g_Varh_Q2[4] = g_Varh_Q2[3];
		OSEEPROMWrite(EEPROM_ENERGY_VARH_Q2, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q2[0],16);
	}

	if(g_Varh_Q3[3] > (g_Varh_Q3[4] + addon))
	{
		g_Varh_Q3[4] = g_Varh_Q3[3];
		OSEEPROMWrite(EEPROM_ENERGY_VARH_Q3, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q3[0],16);
	}

	if(g_Varh_Q4[3] > (g_Varh_Q4[4] + addon))
	{
		g_Varh_Q4[4] = g_Varh_Q4[3];
		OSEEPROMWrite(EEPROM_ENERGY_VARH_Q4, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q4[0],16);
	}

}

void Init_Xfer(void)
{
	bool    	sucess = TRUE;
	uint8_t     i,j,eeprom_new_flag = 0;

	net_status = net_status_lock = 0;	//����״̬��
	n_Current  = 0;			//���ߵ������
	tem_Wh_P = tem_Wh_N = tem_Varh_P = tem_Varh_N = 0;
	temA_Wh_P = temB_Wh_P = temC_Wh_P = 0;
	temA_Wh_N = temB_Wh_N = temC_Wh_N = 0;
	temA_Varh_P = temB_Varh_P = temC_Varh_P = 0;
	temA_Varh_N = temB_Varh_N = temC_Varh_N = 0;

	g_EnergyClear 	= FALSE;
	U8_CURR_WorkMode = D_NORMAL_MODE;//��������ģʽ
	OSEEPROMRead(EEPROM_NEW_FLAG,CAT24WCXX_ADDR,&eeprom_new_flag,1);
	if( eeprom_new_flag == 0x55)//��ʼ�������ڿ�
	{
		for(i=0;i<3;i++)//����3�� ��ȡ����
		{
			sucess &= OSEEPROMRead(EEPROM_ENERGY_WH_PT, CAT24WCXX_ADDR, (uint8_t*)&g_Wh_PT[0], 16);
			sucess &= OSEEPROMRead(EEPROM_ENERGY_WH_NT, CAT24WCXX_ADDR, (uint8_t*)&g_Wh_NT[0], 16);
			sucess &= OSEEPROMRead(EEPROM_ENERGY_VARH_Q1, CAT24WCXX_ADDR, (uint8_t*)&g_Varh_Q1[0], 16);
			sucess &= OSEEPROMRead(EEPROM_ENERGY_VARH_Q2, CAT24WCXX_ADDR, (uint8_t*)&g_Varh_Q2[0], 16);
			sucess &= OSEEPROMRead(EEPROM_ENERGY_VARH_Q3, CAT24WCXX_ADDR, (uint8_t*)&g_Varh_Q3[0], 16);
			sucess &= OSEEPROMRead(EEPROM_ENERGY_VARH_Q4, CAT24WCXX_ADDR, (uint8_t*)&g_Varh_Q4[0], 16);

			if(sucess == TRUE) break;//�ɹ�,�Ͳ����ٳ�����
		}

		g_Wh_PT[4] = g_Wh_PT[3];
		g_Wh_NT[4] = g_Wh_NT[3];

		g_Varh_Q1[4] = g_Varh_Q1[3];
		g_Varh_Q2[4] = g_Varh_Q2[3];
		g_Varh_Q3[4] = g_Varh_Q3[3];
		g_Varh_Q4[4] = g_Varh_Q4[3];
	}
	else//���ڿ�
	{
		for(i=0; i< 5; i++)
		{
			g_Wh_PT[i] = 0;
			g_Wh_NT[i] = 0;
			g_Varh_Q1[i] = 0;
			g_Varh_Q2[i] = 0;
			g_Varh_Q3[i] = 0;
			g_Varh_Q4[i] = 0;
		}
		RESET_WD();
		OSEEPROMWrite(EEPROM_ENERGY_WH_PT, CAT24WCXX_ADDR,(uint8_t*)&g_Wh_PT[0],16);
		OSEEPROMWrite(EEPROM_ENERGY_WH_NT, CAT24WCXX_ADDR,(uint8_t*)&g_Wh_NT[0],16);
		OSEEPROMWrite(EEPROM_ENERGY_VARH_Q1, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q1[0],16);
		OSEEPROMWrite(EEPROM_ENERGY_VARH_Q2, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q2[0],16);
		OSEEPROMWrite(EEPROM_ENERGY_VARH_Q3, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q3[0],16);
		OSEEPROMWrite(EEPROM_ENERGY_VARH_Q4, CAT24WCXX_ADDR,(uint8_t*)&g_Varh_Q4[0],16);
	}

	for(i=0;i<3;i++)
	{
		g_V_Temp[i]    		= 0;
		g_V_Line_Temp[i]   	= 0;
		g_I_Temp[i]    		= 0;
		g_Cos_T[i]     		= 1000;//ABC ����������ʼ��Ϊ1
		g_PhUI_T[i]			= 0;
	}
	g_I_Temp[3]    = 0;
	g_Cos_T[3]	   = 1000;//�� ����������ʼ��Ϊ1
	g_PhUU_T[0]	   =12000;	//��ʼ��Ϊ120,240��
	g_PhUU_T[1]	   =24000;

	for(i = 0; i < 4; i++)
	{
		g_PW_T[i]   = 0;
		g_PVar_T[i] = 0;
	}

	for(i = 0; i < 3; i++)
	{
		for (j = 0; j < AV_POWER_N ; j++)
			g_PW_T_Buf[i][j] = 0;
	}
	Load_Calib_Data(EEPROM_TDK6545H_PARA_ADDR0);//����У׼����
	iadcN_p = 0;
	n_current_adc = 0;
	return;
}

void Cal_module()
{
	uint32_t  av_power[3],power_temp;//���3��ƽ������
	uint32_t  p_buf[3][AV_POWER_N];//����һ�ݻ��湦�ʣ���������
	uint8_t   i,j,k;
	int32_t	  Ev,Axv;
	int32_t	  E0,Axi;
	int32_t   CAL_V[3],CAL_I[3];
	int16_t	  PHADJ[3];
	int32x_t  * dadr;
	int32x_t  sdata;
	static	bool	  Whether_cal = FALSE;//�Ƿ�У����
	//static  bool	  Rst_cal = FALSE;//У����ʼʱ����ҪУ�����࣬У������Ҫ�ָ�Ĭ��ֵ
	for (i=0; i< 3; i++)//��ʼ��
	{
		av_power[i] = 0;
		CAL_V[i] = 0;
		CAL_I[i] = 0;
		PHADJ[i] = 0;
	}
	//ƽ������,ABC��
	for (i=0;i<3;i++)
	{
		for(j=0;j<AV_POWER_N;j++)
			av_power[i] += g_PW_T_Buf[i][j];
		av_power[i] /= 3;
	}
	//����һ�ݹ��ʻ�����������
	for(i = 0; i < 3; i++)
	{
		for(j =0 ;j< AV_POWER_N; j++)
			p_buf[i][j] = g_PW_T_Buf[i][j];
	}
	for(k=0; k < 3; k++)//ABC����
	{
		for(i = 0; i < AV_POWER_N; i++)//ð������
		    {
		        for(j = 0; i < AV_POWER_N - j- 1; j++)
		        {
		            if(p_buf[k][j] > p_buf[k][j + 1])
		            {
		            	power_temp = p_buf[k][j];
		            	p_buf[k][j] = p_buf[k][j + 1];
		            	p_buf[k][j + 1] = power_temp;
		            }
		        }
		    }
	}
	if(U8_CURR_WorkMode == D_NORMAL_MODE)//����ģʽ�£��ж��Ƿ���Ҫת��״̬
	{
		if(FAC == FAC_EN)
		{
			if(U16_FAC_STABLE_Times >= 10)
			{
				U16_FAC_STABLE_Times = 0;
				U8_CURR_WorkMode = D_CAL_START_MODE;
			}
			else
				U16_FAC_STABLE_Times ++;

		}
		else
			U16_FAC_STABLE_Times = 0;
	}
	if(U8_CURR_WorkMode == D_CAL_START_MODE)//У��״̬
		{
			if (Whether_cal == FALSE)//ÿ�ν���У��״̬��ֻУ��1�Ρ�
			{
				for(i = 0; i < 3; i++)
				{
					if(av_power[i] >= CAL_POWER_MIN && av_power[i] <= CAL_POWER_MAX
						&& p_buf[i][AV_POWER_N-1] - p_buf[i][0] <= CAL_POWER_DEVIATION)
						{
							//Ev=(����ֵ-��׼ֵ)/��׼ֵ  , Axv = 1+Ev �Ŵ�10000��
							Ev = ( (int32_t)g_V_Temp[i] - STAND_V ) * 10000 / STAND_V;
							Axv = 10000 + Ev;
							CAL_V[i] = 163840000 / Axv;
							//E0=(����ֵ-��׼ֵ)/��׼ֵ, Axi = (1+E0)/Axv ��������1.0
							E0 = ( (int32_t)g_PW_T[i] - STAND_P ) *10000 / STAND_P;
							Axi =  (10000 + E0) * 10000  / Axv;
							CAL_I[i] = 163840000 / Axi;
							//У��ֵ���浽RAM �͵�ַ��ŵ��ֽ� �ߵ�ַ��Ÿ��ֽ�
							Bakbuf[4*i]   = CAL_I[i] % 256;
							Bakbuf[4*i+1] = CAL_I[i] / 256;
							Bakbuf[4*i+2] = CAL_V[i] % 256;
							Bakbuf[4*i+3] = CAL_V[i] / 256;
							Whether_cal = TRUE;
						}
				}
				if( Whether_cal == TRUE )
					{
						//У��ֵд��CE RAM,ȫ��ˢ��һ��
						EA = 0;
						for (j=0;j<10;j++)
						{
							//Bakbuf�е�ÿ2�ֽڣ��ϳ�16λ��
							sdata = Bakbuf[j*2+1]*256 + Bakbuf[j*2];
							dadr = (TDK6545PARAADDR[j]*4);//ȡCE��Ӧ�ô�ŵĵ�ַ
							memcpy_cex_1( dadr, &sdata);
						}
						EA = 1;
						//У��ֵ����
						OSEEPROMWrite(EEPROM_TDK6545H_PARA_ADDR0, CAT24WCXX_ADDR,&Bakbuf[0],PARA_NUM);
					}
			}
			if(FAC == FAC_DIS)
			{
				if(U16_FAC_STABLE_Times >= 10)
				{
					U16_FAC_STABLE_Times = 0;
					U8_CURR_WorkMode = D_NORMAL_MODE;
					Whether_cal = FALSE;
				}
				else
					U16_FAC_STABLE_Times ++;
			}
			else
				U16_FAC_STABLE_Times = 0;
		}
		//У׼����
}
void earth_dec_init(void)
{
	EarthDelayStar = 0; 	//���߼�⿪ʼ��ʶ
	EarthDelayCnt = 0;		//���߼�ⳬʱʱ��
	EarthDelayOut = 0;			//���߼�ⳬʱ��ʶ
}
void earth_dec()
{
	if(EARTH_DEC == 0 && EarthDelayStar == 0)//��һ�μ�⵽���ߴ���
	{
		EarthDelayStar = 1;
		EarthDelayCnt = 2;//20ms
	}
	if( EARTH_DEC == 0 && EarthDelayOut ==1)//��ʱ����Ȼ���ߴ���
	{
		earth_dec_init();
		net_status |= 0x01;//set bit 0 ���߼��
		if( net_status != 0)
			TELL_MCU;
		net_status_lock |= 0x01;//set bit 0 ���߼��
	}
	if(EARTH_DEC == 1 )
	{
		earth_dec_init();
		net_status &= 0xfe;//clr bit 0 ���߼��
		if( net_status == 0)
			NOT_TELL;
	}

}

