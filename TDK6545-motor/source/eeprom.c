#include "options.h"
#include "intrins.h"
#include <string.h>
#include <math.h>
#include "CONFIG.H"
#include "energy.h"

#define	SDA_OUTPUT 		DIR_3 = 1;
#define	SDA_INPUT		DIR_3 = 0;
#define	SCL_OUTPUT		DIR_2 = 1;
#define	I2C_SDA			DIO_3
#define	I2C_SCL			DIO_2
#define	HIGH			1
#define	LOW				0

extern uint8_t cal_finished;

void delay(unsigned char  t)
{
	unsigned char  i;
	while(t)
	{
		for(i=0;i<20;i++)
		{
			_nop_();
			_nop_();
			_nop_();
			_nop_();

			_nop_();
			_nop_();
			_nop_();
			_nop_();
		}

	--t;
	}
}


void I2C_Start(void)
{
	SDA_OUTPUT;  				/* set P04 to output */
	SCL_OUTPUT;					/* set P05 to output */
	I2C_SDA = HIGH;     		/* SCL high, SDA high */
	delay(2);
	I2C_SCL = HIGH;
	delay(2);
	I2C_SDA = LOW;    			/* SDA low */
	delay(2);
	I2C_SCL = LOW;
	delay(2);
}

void I2C_Stop(void)
{
	SDA_OUTPUT;  			// set P04 to output
	I2C_SDA = LOW;    		// SDA low
	delay(2);
	I2C_SCL = HIGH;
	delay(2);
  	I2C_SDA = HIGH;
  	delay(2);
	I2C_SCL = LOW;
	delay(2);
}

void EnableWriteI2c()
{
	DIR_4 = 1;
	DIO_4 = 0;
	delay(2);
}

void DisableWriteI2c()
{
	DIR_4 = 1;
	DIO_4 = 1;
	delay(2);
}


bool I2C_Write_Byte(uint8_t Data)
{
	uint8_t i;

//	EnableWriteI2c();
	RESET_WD();
	for (i=0; i<8; i++)
	{
		if (Data & 0x80)
			I2C_SDA = HIGH;   	// change data
		else
			I2C_SDA = LOW;    	// change data
		delay(2);
		I2C_SCL = HIGH;     	// set SCL high
		delay(2);
		Data <<= 1;     		// shift to next bit
		if(i < 7)
		{
			I2C_SCL = LOW;      	// set SCL low
			delay(2);
		}
		else
		{
			I2C_SCL = LOW;      	// set SCL low
		}

	}
	SDA_INPUT;  				//?��2��D?��?,��?��?����?��SDA?����?��?��-�̨�,?a��?EEPROM��?ACK,��??������?T1?.
	delay(2);
	I2C_SCL = HIGH;   			//SCL high to sample acknowledge of I2C device
	delay(2);
	if (I2C_SDA == 1)
	{
		I2C_SCL = LOW;
		I2C_SDA = HIGH;
		SDA_OUTPUT;  			//set P04 to output
		delay(2);
//		DisableWriteI2c();
		return FALSE;
	}
	else
	{
		//?ACK?,SDA??,?????????????SCL????,?????SDA
		//SDA?????,?????SDA????,SDA??????????
		//??SDA?????,???,??SCL??????????????
		I2C_SDA = LOW;			// TEST 160215
		SDA_OUTPUT;  			// set P04 to output
		delay(1);
		I2C_SCL = LOW;   		// set SCL LOW for next transfer
		//I2C_SDA = HIGH;
//		DisableWriteI2c();
		return TRUE;
	}
}


uint8_t I2C_Read_Byte(uint8_t ACK)
{
	uint8_t read_byte = 0;
	uint8_t i;
	RESET_WD();
	SDA_INPUT;						// set P04 to input
	for (i=0; i<8; i++)
   	{
    	read_byte = read_byte << 1;
		I2C_SCL = HIGH;   				// set SCL high
		delay(2);
	   	read_byte |= I2C_SDA;  			// shift in next bit
    	I2C_SCL = LOW;        			// set SCL low
    	delay(2);
   	}
	SDA_OUTPUT;						// set P04 to output
	if (ACK==0)
		I2C_SDA = 0; 					// acknowledge
	else
		I2C_SDA = 1; 					// no acknowledge
	delay(2);
	I2C_SCL = HIGH;    					// set SCL high
	delay(2);
	I2C_SCL = LOW; 						// Acknowledge clock
	delay(1);
	return (read_byte);
}

bool I2C_Sequential_Write (uint16_t MemAddr,uint8_t DevSlaveAddr,uint8_t _buff_I2C[],uint8_t N)
{
	uint8_t i,addr_byte;
	uint16_t addr;
	uint8_t * data_point;

	addr=MemAddr;
	data_point = _buff_I2C;
	i=N;

	EnableWriteI2c();

	while(i>0)		//for some reason data not transfer right
	{
  		I2C_Start();               // start I2C transfer

		if (I2C_Write_Byte(DevSlaveAddr & 0xFE)==FALSE)        /* transfer slave address, and define Write cycle (Bit0 = 0) */
   		{
    			I2C_Stop();        						//Error
    			return FALSE;
   		}
		if (DevSlaveAddr == CAT24WCXX_ADDR)					//for at24WCxx address with is 8bit
		{
			addr_byte = (uint8_t)(addr >> 8);
			if (I2C_Write_Byte(addr_byte) == FALSE)      	// transfer memory address
   			{
    			I2C_Stop();        						//Error
    			return FALSE;
			}
			addr_byte = (uint8_t)(addr & 0xFF);
			if (I2C_Write_Byte(addr_byte) == FALSE)      	// transfer memory address
   			{
    			I2C_Stop();        						//Error
    			return FALSE;
			}
		}
  		while(i>0)
		{
			I2C_Write_Byte(* data_point);
  			data_point ++;
			addr=addr+1;
			i=i-1;
			if (addr==0)
			{
				I2C_Stop();
				delay(10);
				break;
			}
		}
		if(addr==0)
			break;
	}
	I2C_Stop();
	delay(10);		//very important here!
	delay(10);

	DisableWriteI2c();
	return TRUE;
}

bool I2C_Sequential_Read (uint16_t MemAddr,uint8_t  DevSlaveAddr,uint8_t  _data_point[],uint8_t N)
{
	uint8_t i,addr_byte;
	uint16_t addr;
	uint8_t * data_point;
	uint8_t ACK =0;

	addr=MemAddr;
	data_point = _data_point;
	i=N;
	while(i>0)
	{
  		I2C_Start();               // start I2C transfer

		if (I2C_Write_Byte(DevSlaveAddr & 0xFE)==FALSE)        /* transfer slave address, and define Write cycle (Bit0 = 0) */
   		{
    			I2C_Stop();        						//Error
    			return FALSE;
   		}
		if (DevSlaveAddr==CAT24WCXX_ADDR)					//for at24WCxx address with is 8bit
		{
			addr_byte = (uint8_t)(addr >> 8);
			if (I2C_Write_Byte(addr_byte) == FALSE)      	// transfer memory address
   			{
    			I2C_Stop();        						//Error
    			return FALSE;
			}
			addr_byte = (uint8_t)(addr & 0xFF);
			if (I2C_Write_Byte(addr_byte) == FALSE)      	// transfer memory address
   			{
    			I2C_Stop();        						//Error
    			return FALSE;
			}
		}

  		I2C_Start();          //start I2C transfer

		if (I2C_Write_Byte(((DevSlaveAddr & 0xFE)|0x01))==FALSE)        // transfer slave address, and define Read cycle (Bit0 = 1)
   		{
    			I2C_Stop();        						//Error
    			return FALSE;
		}
		else
		{
			while(i>0)
			{
				if(i == 1) ACK = 1;
				* data_point = I2C_Read_Byte(ACK); // read the byte
  				data_point ++;
				addr=addr+1;
				i=i-1;
				if (addr==0)
				{
					I2C_Stop();
					break;
				}
			delay(1);
			}
		}
		if(addr==0)
			break;
	}
	I2C_Stop();
	return TRUE;
}


bool OSEEPROMRead (uint16_t MemAddr,uint8_t  DevSlaveAddr,uint8_t  _data_point[],uint8_t N)
{
	uint8_t i;
	uint16_t addr;
	uint8_t * data_point;
//	U08 count,loop;
	bool successfail=0;

	addr=MemAddr;
	data_point = _data_point;
	i=N;
	successfail = I2C_Sequential_Read ( addr , DevSlaveAddr, data_point ,i );
	return successfail;

}
//����1��ȷ  ����0����
bool OSEEPROMWrite (uint16_t MemAddr,uint8_t  DevSlaveAddr,uint8_t  _data_point[],uint8_t N)
{
	uint8_t i,a;
	uint16_t addr;
	uint8_t * data_point;
	uint8_t count,loop;
	bool successfail=0;

	addr=MemAddr;
	data_point = _data_point;
	i=N;

	a = 8- addr % 8;
	if(i <= a)
	{
		successfail = I2C_Sequential_Write(addr, DevSlaveAddr, data_point,i);
		delay(100);
		//д��һ�����ֽں�һ��Ҫ����ʱ����ڿ������ݴӻ���д�뵽EEPROM,������һ���������ڿ������ACK
		if(!successfail)
			return successfail;
	}
	else
	{
		successfail = I2C_Sequential_Write(addr, DevSlaveAddr, data_point,a);
		delay(100);
		//�ڿ������ݴ�cacheŲ��EEPROM��Ҫʱ��
		if(!successfail)
			return successfail;
		i -= a;
		addr += a;
		data_point += a;

		count = i / 8;
		for(loop = 0; loop < count; loop ++)
		{
			successfail = I2C_Sequential_Write ((addr+ loop * 8), DevSlaveAddr, (data_point+ loop * 8), 8);
			delay(100);
			//�ڿ������ݴ�cacheŲ��EEPROM��Ҫʱ��
			if(!successfail)
				return successfail;
		}
		successfail = I2C_Sequential_Write ((addr + count * 8), DevSlaveAddr, (data_point + count * 8), (i - count * 8));
		delay(100);
		//�ڿ������ݴ�cacheŲ��EEPROM��Ҫʱ��
	}


	return successfail;

}

void memcpy_cex_1 (int32_t *pDst, int32_t *pSrc)
{

// Never any overlapped data.

//	EX_CE_BUSY = FALSE;
	//EX_XFER_RTC = FALSE;

    *((uint8_t *) pDst)  = *((uint8_t *) pSrc);  ((uint8_t *) pDst)++;   ((uint8_t *) pSrc)++;
    *((uint8_t *) pDst)  = *((uint8_t *) pSrc);  ((uint8_t *) pDst)++;   ((uint8_t *) pSrc)++;
    *((uint8_t *) pDst)  = *((uint8_t *) pSrc);  ((uint8_t *) pDst)++;   ((uint8_t *) pSrc)++;

    //CKCON = STRETCH_6;
    *((uint8_t *) pDst)  = *((uint8_t *) pSrc);  ((uint8_t *) pDst)++;   ((uint8_t *) pSrc)++;
    //CKCON = STRETCH_1;

	//EX_XFER_RTC = TRUE;
//  	EX_CE_BUSY = TRUE;

}

//���� ������У�����
//�汾��2 ʱ�� 2016 06 17
//�ð汾���ص�Ĭ�ϲ�������0x4000 �� 0x00 ����u:0x4105 i:0x4216 phadj:0xfe2b
//��У׼��������г�ʼ��
bool Load_Calib_Data(uint16_t paraaddr)// rerurn 1 is ok,return 0 is fail
{
	int32x_t *dadr;
	int32x_t sdata;
	uint8_t i,loop;
	uint8_t eeprom_new_flag;
	bool successfail = FALSE;

//read the backup parameter
	for(loop = 0; loop < 3; loop ++)
	{
		successfail = OSEEPROMRead(paraaddr, CAT24WCXX_ADDR, &Bakbuf[0], PARA_NUM);
		if( successfail == FALSE)
			continue;
		else
			break;
	}
	OSEEPROMRead(EEPROM_CAL_FINISHED,CAT24WCXX_ADDR,&cal_finished,1);//У����ɱ�־
	if(cal_finished != 0 && cal_finished != 1)//���ڿ����Ƿ�ֵ,����ΪδУ����
		cal_finished = 0;
	OSEEPROMRead(EEPROM_NEW_FLAG,CAT24WCXX_ADDR,&eeprom_new_flag,1);//�ж��Ƿ���EEPROM
	if( successfail == FALSE || eeprom_new_flag != 0x55)//ʧ�ܻ����ڿ� дĬ��ֵ
	{
//���ξɳ���
//		for(i = 0;i <6; i++)
//		{
//			Bakbuf[2*i] = 0x00;
//			Bakbuf[2*i+1] = 0x40;
//		}
//		for(i = 6; i < 9 ; i ++)
//		{
//			Bakbuf[2*i] = 0x00;
//			Bakbuf[2*i+1] = 0x00;
//		}
//�³���
		Bakbuf[0] = 0x16;//IA
		Bakbuf[1] = 0x42;
		Bakbuf[2] = 0x05;//VA
		Bakbuf[3] = 0x41;
		Bakbuf[4] = 0x16;//IB
		Bakbuf[5] = 0x42;
		Bakbuf[6] = 0x05;//VB
		Bakbuf[7] = 0x41;
		Bakbuf[8] = 0x16;//IC
		Bakbuf[9] = 0x42;
		Bakbuf[10] = 0x05;//VC
		Bakbuf[11] = 0x41;
		Bakbuf[12] = 0x2b;//A PHADJ
		Bakbuf[13] = 0xfe;
		Bakbuf[14] = 0x2b;//B PHADJ
		Bakbuf[15] = 0xfe;
		Bakbuf[16] = 0x2b;//C PHADJ
		Bakbuf[17] = 0xfe;

		Bakbuf[18] = 0x1d; //wrate = 29
		Bakbuf[19] = 0x00;
		if(eeprom_new_flag != 0x55)//���ڿ�����дУ�����
		{
			OSEEPROMWrite(EEPROM_TDK6545H_PARA_ADDR0, CAT24WCXX_ADDR,&Bakbuf[0],PARA_NUM);
			eeprom_new_flag = 0x55;
			OSEEPROMWrite(EEPROM_NEW_FLAG,CAT24WCXX_ADDR,&eeprom_new_flag,1);
			OSEEPROMWrite(EEPROM_CAL_FINISHED, CAT24WCXX_ADDR,&cal_finished,1);
		}

	}
	for (i=0;i<10;i++)
	{
		//bakbuf�ǵ�λ��ǰ˳��,2�ֽںϳɳ�һ����,��ʵ���˸�λ��ǰ����Ƭ��Ĭ�ϴ洢��ʽ��
		sdata = Bakbuf[i*2+1]*256 + Bakbuf[i*2];
		dadr = (TDK6545PARAADDR[i]*4);//ȡCE��Ӧ�ô�ŵĵ�ַ
		memcpy_cex_1( dadr, &sdata);
	}
//���峣��test use
//	ce_collate_locate = 0x1000 + 0x2d*4;
//	*sadr = (Bakbuf[18] +  Bakbuf[19] * 256);	//Kh = VMAX*IMAX*66.1782 / (In_8*WRATE*NACC*X)  // some thing wrong here
//	memcpy_cex_1( (S32x *) ce_collate_locate, (S32x *)sadr);
	return TRUE;
}
