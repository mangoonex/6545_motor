/***************************************************************************
 * This code and information is provided "as is" without warranty of any   *
 * kind, either expressed or implied, including but not limited to the     *
 * implied warranties of merchantability and/or fitness for a particular   *
 * purpose.                                                                *
 * AUTHOR: RGV, SD                                                         *
 * DESCRIPTION: 71M654x POWER METER - 6543 CE example                      *
 * Copyright (C) 2009 Maxim Integrated Products Inc. All Rights Reserved.  *
 ***************************************************************************/

#include "options.h"            // Define the IC, CE code and options.
#include <string.h>
#include <math.h>
#include "CONFIG.H"
#include "energy.h"
#include "eeprom.h"

// Never use 'using 0' for ISRs: it won't preserve the registers used
// in the non-interrupt code.
#define   PULSE_BANK  1         // Register bank of the pulse-bank interrupt.

#define SAG_RECOVERY_TIME 2     // It takes 2 seconds for CE data to be reliable. 


//************************����*********************************

bit xfer_update;                // CE data is ready to use.
uint8_t data power_fail_timer;  // Wait for CE's filter to stabilize.
volatile int32_t zc_cnt;        // Count zero crossings.
volatile int32_t sag_cnt;       // Count sags that affected all the phases.


//**************************ͨѶ*************************

uint8_t 	rx_count=0;	//rx counter
uint8_t 	tx_count=0;	//tx counter
uint8_t 	*p_rx_buf;
uint8_t 	*p_tx_buf;
uint8_t 	NetAddress; //��ַ
bool 		Is_Rx_Ok;
bool 		Is_Txing;	//TRUE:������ FALSE:���Ϳ���
RECEIVEFRAME xdata SerialBuffer;//֡��ʽ
uint8_t ReceFrameStar; //��ʼ����֡��ʶ
uint8_t ReceDelayCnt;	//���ճ�ʱʱ��  30*10ms=300ms
uint8_t	ReceDelayOut;	//���ճ�ʱ��ʶ
#define	REC_DELAY_MAX					6	//��ʱʱ�� 6*10ms=60ms
#define CMD_READ_VOLTAGE				0x30	//��ȡ��ѹ
#define	CMD_READ_CURRENT				0x31
#define	CMD_READ_POWER					0x32
#define	CMD_READ_NCURRENT				0x33
#define	CMD_SET_CALIB					0x38
#define	CMD_RSET_CALIB					0x39
#define CMD_READ_CALIB					0x3a
#define	CMD_CLEAR_P_ENERGY				0x5a
#define	CMD_READ_POS_P_ENERGY			0x54
#define	CMD_READ_NEG_P_ENERGY			0x55
#define	CMD_READ_VERSION				0x5b
#define CMD_READ_STATUS					0x5c
#define	CMD_READ_PARA					0x5d
#define	CMD_WRITE_PARA					0x5e
#define CMD_RELAY_CTR					0x10
//����汾�ţ����ӽӵر��������ߵ���������У����ɱ�־���Լ���ز����Ķ�д���
//20160819
#define	VERSION							0x03

#define	CAL_FINISHED					0x00	//1byte
#define	N_CURR							0x01	//4byte
#define SAG_THR_RMS 					50

//*****************��Ħ�������ض���*****************
#define IO_OUT_1						0x03	//0b 0000 0011 ����(bit1)�����  ��ƽ(bit0)����
#define IO_OUT_0						0x02	//0b 0000 0010 ����(bit1)�����  ��ƽ(bit0)����
#define R1_OFF							DIO19
#define R1_ON							DIO20
#define R2_OFF							DIO21
#define R2_ON							DIO22
#define R3_OFF							DIO23
#define R3_ON							DIO24
#define R1_CTR_ON						{R1_ON = IO_OUT_1; R1_OFF = IO_OUT_0;}
#define R1_CTR_OFF						{R1_ON = IO_OUT_0; R1_OFF = IO_OUT_1;}
#define R1_CTR_KEEP						{R1_ON = IO_OUT_0; R1_OFF = IO_OUT_0;}
#define R2_CTR_ON						{R2_ON = IO_OUT_1; R2_OFF = IO_OUT_0;}
#define R2_CTR_OFF						{R2_ON = IO_OUT_0; R2_OFF = IO_OUT_1;}
#define R2_CTR_KEEP						{R2_ON = IO_OUT_0; R2_OFF = IO_OUT_0;}
#define R3_CTR_ON						{R3_ON = IO_OUT_1; R3_OFF = IO_OUT_0;}
#define R3_CTR_OFF						{R3_ON = IO_OUT_0; R3_OFF = IO_OUT_1;}
#define R3_CTR_KEEP						{R3_ON = IO_OUT_0; R3_OFF = IO_OUT_0;}
#define CLOSE							0x1B
#define OPEN							0x1A


//***********************����*************************
bool Serial_Buf_Process(void);

/* Detect battery power*/
bool batmode_is_brownout (void) small reentrant;

/* CE data ready ISR */
void xfer_rtc_isr (void) small reentrant; 

/* Power failure detection ISR */
void pulse_isr (void) small reentrant;

/* Null interrupt ISR to reset interrupts*/
static void null_isr (void);

/* Copy xdata table */
void xdata_table(uint8r_t *c_ptr);
void Serial_Process(void);
void Init_Serial(void);
void ACfailureSave(void);
void Relay_Ctr(uint8_t num ,uint8_t cmd);
extern void Xfer_Process(void);
extern bool OSEEPROMWrite (uint16_t MemAddr,uint8_t  DevSlaveAddr,uint8_t  _data_point[],uint8_t N);
extern	bool OSEEPROMRead (uint16_t MemAddr,uint8_t  DevSlaveAddr,uint8_t  _data_point[],uint8_t N);
extern bool I2C_Sequential_Write (uint16_t MemAddr,uint8_t DevSlaveAddr,uint8_t _buff_I2C[],uint8_t N);
extern bool I2C_Sequential_Read (uint16_t MemAddr,uint8_t  DevSlaveAddr,uint8_t  _data_point[],uint8_t N);
extern void SaveEnergy(uint32_t addon);
extern void Cal_module();
/* Internal IO RAM table; This is a demo that assumes
 * hard reset in order to show only bits the CE needs.
 * Production code should set every bit in the IC.
 * The comments describe the fields that follow. */
const uint8r_t io_ram_table[] =
{
   0x21, 0x00, 0x00, 0x13, // address and length
   // This is a 3-phase meter with neutral current.  
   // MUX_DIV is off at first! Changing the selectors with the multiplexor
   //   running can cause contention. 
   // Order of multiplexer is:
   //   IA, VA, IB, VB, IC, VC, IN, unused, unused, unused, unused
   //   The unused ADC slots are set to an invalid number (1) as a defensive
   //   programming practice.  1 is invalid because IA is differential,
   //   so current input 1 is IAN, thus not selectable.
   //   Remote current sensors need a different sequence:
   //   The voltages are clustered at the end of the cycle.
   //   The ADC slots at the start of the cycle should be unused, invalid
   //   ADC slot numbers (e.g. "1" if IA is differential.)
   // The equation is "5" so the CE code makes Wh = VA*IA + VB*IB + VC*IC
   // The Vref chop is automatic (the best.)
   // RTM_E is off, because we don't need ADC data out of the IC. (RTM is
   //   a bit-serial data stream from TMUX1's 0x1F selection. It sends
   //   DSP data.)
   // CE_E (the CE) is off because its input system is not yet configured.
   // SUMSAMPS is set to 0x0888, 2184 decimal, the number of mux
   //   frames per summation interval. It makes the  summation interval
   //   ~1 S. Why? ADC's FIR length = 288, 2 crystal times * 7 samples 
   //   per mux frame, and one  crystal time of silence.
   //   (2*7)+1 = (15/32768S)/sample.  So, the Fs (sample frequency) = 
   //   32768/15 = 2184.53Hz
   // The CE's code is at 0x03*0x400, 0xC00 because the Keil linker
   //   is commanded to put it there in the build options.
   // The pulse width is 0.01S, very good for most calibrators.
   // The current inputs are all differential.
   // RTM is set to the first 4 ADC locations, so if it is enabled, good
   //   data comes out.
   0x01, 0x11, 0x10, 0xA6, 0x94, 0x82, 0xA0, 0x08, // 2100..2107
   0x88, 0x03, 0x42, 0x5D, 0xF2, 0x00, 0x00, 0x01, // 2008..200F
   0x02, 0x03, 0x04,                               // 2110..2112

   0x22, 0x00, 0x00, 0x01, // address and length
   0x10,                   // PLL_FAST=1 MPU_DIV[2:0]=0
   	   	   	   	   	   	   // Clock tree: MCK=19.6608MHz, ADC,MPU=4.9152MHz

   0x21, 0x00, 0x00, 0x01, // address and length ; set mux_div after ADC clock
   0x71,                   // Read 7 voltages per mux frame.

   0x24, 0x50, 0x00, 0x09, // address and length
   // Don't attach any interrupts or timer gates to pins.
   //��ʹ������I2C
   // Set the optical pin to send a Wh pulse.
   // The optical pin is not inverted. (i.e. negative-going pulse, for an LED)
   // The pulse outputs are disabled to prevent bad pulses
   //     from the CE start-up.
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,  // External interrupts
   0x00, 

   0x27, 0x00, 0x00, 0x05, // address and length
   // CE interrupts are enabled, others disabled.
   // The flash controls are zeroed (not needed by the CE).
   // Vref is placed on a pin so it can be measured easily. (VREF_CAL = 1)
   //   Very helpful for debugging the temperature compensation of Vref.  
   //   Not recommended for production meters.
   // The preamp is disabled (PRE_E = 0)
   // The ADC is enabled (ADC_E = 1)
   // The battery-measurement current source is off. (not needed by the CE)
   //PRE_E =1 20170518 enable IADC0-IADC1 PRE AMPLIFY 8x leak current detect
   0x61, 0x00, 0x00, 0x00, 0xb0,                    // 2700, No preamp, Ch. 1

   
   0x27, 0x09, 0x00, 0x04, // address and length
   // Remote current sensors are disabled.
   // DIO outputs are enabled (PORT_E = 1)
   // Clock is sent to both the SPI and EEPROM (SPI_E = 1; for realism)
   // Slave SPI is prevented from corrupt RAM, especially CE RAM (SPI_SAFE = 1)
   0x85, 0x55, 0x00, 0x38,                    // 2709; Remotes disabled

   0x00, 0x00  // end of table
};

/* Initialize MPU */ 
void mpu_init(void)
{
    RESET_WD();                     //  Clear WDT.

    // Set up the MPU.
    CKCON       = 0x00;  // The static RAM is fast; even shared.
    ADRMSB      = 0x02;  // Set up the PDATA upper address byte.
    DPL         = 0x00;
    DPH         = 0x00;
    DPL1        = 0x00;
    DPH1        = 0x00;
    DPS         = 0x00; // Select data pointer 0.

    // Clear all existing interrupts
    IEN0        = 0x00;
    IEN1        = 0x00;
    IEN2        = 0x00;
    
    // Clear all waiting interrupts.
    IRCON       = 0x00;

    // Reset the interrupt priorities.
    IP0         = 0x0C;
    IP1         = 0x1B;
    // Interrupt priorities: frequent interrupts are higher.
    // Group,               Level
    // 0=int0, ser1         01 (serial, timer, pulse: 100Hz-~1Kz)
    // 1=timer0,ce_pulses   01
    // 2=int1, ce_busyz     10 (ce busyZ, ~2KHz)
    // 3=timer1, vstat      11 (VSTAT= no power, so no time)
    // 4=ser0, eeprom/SPI   01 (serial... 1KHz)
    // 5=xfer_busy,rtc1s    00 (1Hz..)

    // Run "RTI" to clear each of the four priority levels.
    ((void (code *) (void)) null_isr) (); // Clear 1st priority level.
    ((void (code *) (void)) null_isr) (); // Clear 2nd priority level. 
    ((void (code *) (void)) null_isr) (); // Clear 3rd priority level. 
    ((void (code *) (void)) null_isr) (); // Clear 4th priority level. 

    // Set up the digital I/O ports.  
    // Zero grounds the pins to help prevent oscillation and EMI.
    // DIO 0 & 1 (PORT0 bit 0 & 1) are Wh and VARh pulses, outputs.  (high off)
    // DIO 2 & 3 are the EEPROM pins, outputs. High so EEPROM is not selected.
    // DIO 4 is for the bit-banged RS232 input.
    // DIO 5 reads the button on the debug PCB. It should be an input.
    // DIO 5 is also TX2; On a serial output, idle is high.
    // DIO 6 is disabled; XPULSE is selected by the I/O RAM, output. (high off)
    // DIO 7 is disabled; YPULSE is selected by the I/O RAM, output. (high off)
    // DIO 9 is for the bit-banged RS-232 output.  Idle is high.
    // DIO 52 and 53 drive D2 and D3 on the debug PCB.
    // All others should default to inputs, so they don't drive the LCDs.
    PORT0       = 0xF3;
    PORT1       = 0xCC;
    PORT2       = 0x20;
    PORT3       = 0x00;

    T2CON       = 0x60;     // Interrupt when CE_BUSY & PULSE go high.
    PSW         = 0x00;
    A           = 0x00;
    B           = 0x00;

    //************************init uart0*************************

	WDCON|=0x80; //Select internal baud rate generator
	PCON=0; // SMOD=0
	// 9600BPS ����cpuͨѶ
	// BAUD=2^SMOD*fCKMPU/(64*(2^10-S0REL))
	S0RELH=0xFF;
	S0RELL=0xF8;

	S0CON=0xde;  // UART mode 3. 8bit mode,parity,stop bit,receive enable
	//S0CON=0x5E;
	ES0=1; // enable serial 0 interrupt enable

    //**********************init t0***********************
    TMOD = 0x11;                  // 8-bit auto reload timer/counter 0 & 1
    //ET0 = 1;                      // Enable Timer/Counter 0 Interrupts
    TR0 = 1;                      // Start Timer/Counter 0 Running
    TL0 = 0x84;                   // 3460 counts 10ms
    TH0 = 0x0d;

    // Clear all the interrupt flags
    FLAG0 = 0;
    FLAG1 = 0;

    //********************��ʼ��DIO**********************
    DIR_4 = 1;//SDA
    DIO_4 = 1;
    DIR_9 = 1;//©��������,�����ߵ�ƽ�������͵�ƽ
    DIO_9 = 1;
    //init DIO DOI_5 У������
    DIR_5 = 0;
    DIR_10 = 0;//���߼��
    //*****************��ʼ����Ħ����IO********************
    R1_CTR_KEEP
    R2_CTR_KEEP
    R3_CTR_KEEP
    RESET_WD();
}

void main ( void )
{
    // This is a loop to restart the CE after brownout mode.
    while (TRUE)
    {
        /* Disable interrupts */
        EA = 0;

        if (COLDSTART())
        {
            // Set TMUXs
            TMUX = 0x0E;    // Set low frequency signal to save power
            TMUX2 = 0x0E;
        }

        /* Initialize MPU */ 
        mpu_init();



        /* Copy IO ram table */
        xdata_table(&io_ram_table[0]);

        // In brownout don't set up the CE; the CE can't
        // run anyway because the ADCs are off.  Copying the data
        // takes time, and at a slow clock rate, that's waste.
        RESET_WD();                     //  Clear WDT.
        Init_Serial();
        if (!batmode_is_brownout ())
        {
            RESET_WD();                     //  Clear WDT.
            /* Clear xfer_update flag */
            xfer_update = FALSE;

            /* Ignore data until the phase locked loop in the CE
             * has locked: 1S + 1 accumulation interval. */
            power_fail_timer = SAG_RECOVERY_TIME;

            RESET_WD();
            /* Set CE RAM to default */
            memcpy((uint8x_t*)CE_DATA_BASE, CeData, (4 * NumCeData));
            Init_Xfer();
            earth_dec_init();

            /* Set up power-failure detection not set-up elsewhere
             * Enable, 0.1S 
             */
             ceconfig = ( ceconfig & (~(long)CE_SAG_CNT))
                     | CE_SAG_INT | CE_EXT_PULSE
                     | ((((int32_t)FS_INT)/10L) << 8) | CE_EDGE_INT;
             wrate = 29; //TEST USE 10000imp/kwh
             sag_thr = SAG_THR_RMS*1.414/(VMAX*7.8798e-9);//AC�����ֵⷧ
             /* demonstration variables */
             zc_cnt = 0;
             sag_cnt = 0;

            // Set-up:
            // CE0..CE5, CONFIG0..4
            // CE's power fail pulse appears on PULSE_Y 
            // PULSE_Y = DIO_9 := INT0 @ 0x200D
            // PULSE_Y = 1 @ 200F
            // DIO_9 = 0 @ P1, DIO_9 = out @ DIR1
            // INT0 @ IEN0, =0, falling @ TCON

            RESET_WD();
            IEN1 = 0x22;        // Enable xfer_busy, and pulse interrupt,CE_BUSY�ݲ��򿪣�������ٴ�
            //IEN1 = 0x00;	//testee
            IEN0 = 0x92;        // Enable interrupts.����0,��ʱ��0
            //IEN0 = 0x00;		//testee
            CE6 |= CE_E;        // Enable the CE, now that it's set up.
            while(!batmode_is_brownout())   // While in mission mode, run meter.
            {
                RESET_WD();                     //  Clear WDT.
                earth_dec();
                Convert_Ncurrent();
                if (xfer_update)
                {
                	RESET_WD();
                    xfer_update = FALSE;
                    Xfer_Process();
                    SaveEnergy(500);
                }
                ACfailureSave();
                Serial_Process();
                //Cal_module();
            } 
        }

        // brownout mode never sleeps or slows the clock
        // in this design, so it wastes power.
        while (batmode_is_brownout())
        {
            RESET_WD();
        }        
    } // Restart after brownout stops the CE.
}

// Moves data from a table into xdata RAM.
void xdata_table(uint8r_t *c_ptr)
{
    int16_t cnt;           // This is a count and temporary variable.
    uint8_t xdata * x_ptr; // This is a pointer to xdata RAM.
    do
    {
        // Get the pointer from the table.
        // Use "cnt" as a temporary variable.
        cnt = (int16_t)*c_ptr++;
        cnt = (cnt << 8) | (int16_t)*c_ptr++;
        if (0 == cnt) break;  // Zero marks the end of the table.
        x_ptr = (uint8_t xdata *)cnt;

        // Get the size of the data from the table.
        cnt = (int16_t)*c_ptr++;
        cnt = (cnt << 8) | (int16_t)*c_ptr++;

        // Copy some data from the table into the I/O-RAM.
        for (; 0 < cnt; --cnt)
        {
            *x_ptr++ = *c_ptr++;
        }
    }
    while (TRUE);
}

/* Find battery mode */
#pragma save
#pragma noaregs
bool batmode_is_brownout (void) small reentrant
{
    return ((STAT & VSTAT) > 0) ? 1 : 0;
}
#pragma restore

#pragma save
#pragma NOAREGS
// Interrupt 6.
void xfer_rtc_isr (void) small reentrant interrupt XFER_RTC_IV
{
    // The RTC interrupt is first to reduce timing uncertainties
    // for RTC calibration and compensation.
    if (IE_RTC1S)
    {
        CLR_IE_RTC1S();                    // Just clear IE_RTC bit.
    }

    // Alarm-clock logic.
    // To use it, set the hour and minute, and enable the alarm.
    if (IE_RTCT)
    {
        CLR_IE_RTCT();
    }

    if (IE_XFER)    // Data from the CE?
    {
        CLR_IE_XFER();                  // clear IE_XFER bit

        // For the first second, the CE's calculations are incorrect,..
        // ..so the results are not worth recording.
        if (power_fail_timer == 0)      // Data is valid?
        {
            xfer_update = TRUE;         // Tell the main loop, new data is here.
        }
        else if (0 == --power_fail_timer)
        {
            DIO1 |= DIO_PW | DIO_PV;    // Enable "WPULSE" and "VARPULSE".
            DIO2 |= DIO_PY | DIO_PX;    // Enable interrupts from the CE.
        }
    }
}
#pragma restore


#define ALL_CE_SAG_BITS (CE_SAG_A|CE_SAG_B|CE_SAG_C)
// Interrupts on pulses from the CE.  The two most important
// pulses are AC mains failure detection interrupt, on Y_PULSE,
// and the AC zero-crossing interrupt, on X_PULSE.
// These can be brought out to pins,
// but they do not need to be.
// The watt-hour pulse and var-hour pulse also have interrupts,
// but the CE has built-in pulse counters for these, so no
// interrupt is usually needed.
#pragma save
#pragma REGISTERBANK (PULSE_BANK)
// Vector 9, interrupts on pulses from the CE.
void pulse_isr (void) small reentrant interrupt CE_PULSE_IV using PULSE_BANK
{
    if (IE_YPULSE)  // AC Mains failure detected by CE?
    {
        uint32_t ce_status_tmp = cestatus;  // Read the CE's status word
        int8_t fs = -1;         // The new frequency select index is undecoded.

        CLR_IE_YPULSE();        // Just clear IE_YPULSE bit.

        // Will the power to the meter fail? (Are all phases in sag?)
        // If the power supply is single phase, the test should
        // test just the bit of the phase that supplies the meter's power.
        if (ALL_CE_SAG_BITS == (ce_status_tmp & ALL_CE_SAG_BITS))
        {
            // Yes. The meter's power supply will certainly fail.
            // If the grid switches many times in a second, only
            // save the billing registers the first time.
            // Reclosing fault breakers typically switch five times,
            // with 100ms or less between on-times.
            if (0 == power_fail_timer)
            {
                // save the billing registers here.
                ++sag_cnt;
            }
            power_fail_timer = SAG_RECOVERY_TIME;

            apulsew = 0;    // Stop pulse outputs while CE data is invalid.
            apulser = 0;

            // Switch to battery RTC.  A convenient way is to
            // disable the zero-crossing RTC's interrupt
            // and then use this bit as a global flag.
            INT1_E &= ~EX_XPULSE;   // Disable mains Hz interrupt.
        } else {
            // No, at least one phase has voltage.
            INT1_E |= EX_XPULSE;   // Enable mains Hz interrupt.
            // Switch frequency measurement, zero crossing and VARh
            // phase detection to a working voltage input.
            if (0 == (CE_SAG_A & ce_status_tmp)) // Power on phase A?
            {
                fs = 0;
            }
            else if (0 == (CE_SAG_B & ce_status_tmp)) // Power on phase B?
            {
                fs = 1;
            }
            else // voltage has to be on phase C.
            {
                fs = 2;
            }
            ceconfig = (ceconfig & ~CE_FREQSEL) | ( (int32_t)(fs << 6) );
        }
    }

    if (IE_XPULSE)  // 60Hz zero-crossing interrupt from the CE?
    {
        CLR_IE_XPULSE();        // Just clear IE_XPULSE bit.

        // Why is it checking the enable bit?
        // Because the interrupt vector is shared, every
        // interrupt on it will invoke this code if the
        // x-pulse's interrupt bit is set.  The interrupt
        // enable does not disable bit, just whether the
        // bit causes an interrupt.
        if (0 != (INT1_E & EX_XPULSE)) // if the interrupt is enabled.
        {
            // Here, count zero crossings of AC mains to make a clock.
            ++zc_cnt;
            if(iadcN_p == 0)
            	IEN1 |= EX3_;	//����CEBUSY�ж�
        }
    } // end if mains zero-crossing interrupt.

    if (IE_WPULSE)  // Watt-hour pulse interrupt?
    {
        // This is served whenever a pulse interrupt occurs,
        // and the bit is set because a pulse occurred.
        CLR_IE_WPULSE();        // Just clear IE_WPULSE bit.
        INT2_E &= ~EX_WPULSE;   // Disable the interrupt.
    }

    if (IE_VPULSE)  // VAR-hour pulse interrupt?
    {
        // This is served whenever a pulse interrupt occurs,
        // and the bit is set because a pulse occurred.
        CLR_IE_VPULSE();        // Just clear IE_VPULSE bit.
        INT2_E &= ~EX_VPULSE;   // Disable the interrupt.
    }
}
#pragma restore

void read_adc_isr (void) small reentrant interrupt CE_BUSYZ_IV
		{
			if(iadcN_p >= IADCN_MAX_SAMPLES)
			{
				//do nothing �ж�̫�죬��ѭ��ʱ��̫�����������ֺ�ͣ�¡�
			}
			else
			{
				iadcN[iadcN_p] = iadc67;//�������ֵ
				iadcN_p++;
			}
		}
/* An unused interrupt. This has an RTI instruction.
 * Calling it will clear interrupts waiting on the current
 * priority level.
 */
static void null_isr (void) interrupt 31
{                             // '31' is a dummy interrupt,..
}



void Init_Serial(void)
{
	//added for test
	TI=0;
	RI=0;
	S0CON |= REN_; //������� 2016/01/17

	Is_Rx_Ok =FALSE;
	Is_Txing = FALSE;//���Ϳ���
	p_rx_buf = &SerialBuffer.framehead;
	rx_count = 0;
	tx_count = 0;
	ReceFrameStar = 0;
	ReceDelayCnt = REC_DELAY_MAX;
	ReceDelayOut = 0;
	SerialBuffer.address = 0x01;
}
//************************���ڴ������**********************

void Serial_Process(void)
{
	if(ReceDelayOut == 1)
		Init_Serial();
	if (Is_Rx_Ok)
	{
		if(Is_Txing == FALSE)//����״̬�£������ظ�����
		{
			if(Serial_Buf_Process())
			{
				RESET_WD();
				tx_count=SerialBuffer.length + 5;
				p_tx_buf= &SerialBuffer.address;
				S0BUF &= (~REN_); //׼������֮ǰ�����ý��� 2016/01/17
				S0BUF = 0x68;
				S0CON |= 0x08;
			}
			/*���ݽ������Ҳ����������������³�ʼ������
			 ����Is_Rx_Ok��һֱΪ1�����½����жϻᶪ���������ݡ�
			 ���ڽ�����ȷ�������ͷ���������ɺ󣬳�����ʼ������*/
			else
			{
				Init_Serial();
			}
		}
		//Is_Rx_Ok=FALSE;  ����δ���֮ǰ������������µ����� modifyed 2016/01/17
		//2016/01/17������һ������˼�ǣ��ڽ��յ�����ѽ��������ǻ�û�з����귵������ǰ
		//���ν��գ��жϻ��ж������־��ΪTRUEʱ���ᶪ���������ݣ�,��ô����ԭ���ǣ���������
		//��������ݹ���һ�����ݽṹ���ڷ������֮ǰ���������ݻ���Ҵ��������ݡ�
	}
	else
	{
//		if((ticks > 120) && (Rstate == OFF))
		{
//			WDT_Awaken();
//			SaveBeforeRST(0);
//			Soft_Reset();
		}
	}

}
bool Serial_Buf_Process(void)
{
	uint8_t checksum;
	uint8_t loop;

	bool comtaskadd ;
//	uint16_t  crccheck;
//	U16x  crccheck;
	uint8_t *p;

	uint32_t temp1;
	uint8_t	i;
	uint8_t	para_num;
	int32x_t  * dadr;
	int32x_t  sdata;
	comtaskadd = 0;
	RESET_WD();
	if (SerialBuffer.address == NetAddress || SerialBuffer.address == 0xfe || SerialBuffer.address == 0xff)
	{
		p = &SerialBuffer.serialData[0];
		switch (SerialBuffer.command)//what control code
		{
			case CMD_READ_VOLTAGE:
				{
				//voltage of a b c
				for(loop = 0; loop < 3; loop++)
				{
					*p++ = g_V_Temp[loop];
					*p++ = g_V_Temp[loop]>>8;
				}

				//line frequence
					//*p++ = g_Line_freq;

				//temperature
					//*p++ = g_Chip_Temp;

					comtaskadd = 1;

				break;

				}
			case CMD_READ_CURRENT:
				{

					for(loop = 0; loop < 4; loop++)
					{
						*p ++ = g_I_Temp[loop];
						*p ++ = g_I_Temp[loop]>>8;
						*p ++ = g_I_Temp[loop]>>16;
						*p ++ = g_I_Temp[loop]>>24;
					}
						comtaskadd = 1;
						break;
				}
			case CMD_READ_NCURRENT:
				{
					*p ++ = g_I_Temp[4];
					*p ++ = g_I_Temp[4]>>8;
					*p ++ = g_I_Temp[4]>>16;
					*p ++ = g_I_Temp[4]>>24;
					comtaskadd = 1;
					break;
				}

			case CMD_READ_POWER:											// Read 3 phases Power
				{
					//�й�����
					for(loop = 0; loop < 4; loop++)
					{
						/*
						*p ++ =  PW_T[loop];
						*p ++ =  PW_T[loop]>>8;
						*p ++ =  PW_T[loop]>>16;
						*p ++ =  PW_T[loop]>>24;
						*/
						temp1 = g_PW_T[loop];
						//temp1 = 3+loop;
						*p ++ =  temp1;
						*p ++ =  temp1>>8;
						*p ++ =  temp1>>16;
						*p ++ =  temp1>>24;


					}
					//�޹�����
					/*for(loop = 0; loop < 3; loop++)
					{
						*p ++ =  g_PVar_T[loop];
						*p ++ =  g_PVar_T[loop]>>8;
						*p ++ =  g_PVar_T[loop]>>16;
						*p ++ =  g_PVar_T[loop]>>24;
					}*/

						comtaskadd = 1;
						break;
				}
		  	case CMD_READ_POS_P_ENERGY:										// READ POSITIVE  active ENERGY
				{
					for(loop=0; loop<4; loop++)
					{
			       		*p ++  = g_Wh_PT[loop];
						*p ++  = g_Wh_PT[loop]>>8;
						*p ++  = g_Wh_PT[loop]>>16;
						*p ++  = g_Wh_PT[loop]>>24;
					}

					comtaskadd = 1;
					break;
		  		}

		     case CMD_READ_NEG_P_ENERGY:										//  READ NEGATIVE ACTIVE ENERGY
				{
					for(loop=0; loop<4; loop++)
					{
			       		*p ++  = g_Wh_NT[loop];
						*p ++  = g_Wh_NT[loop]>>8;
						*p ++  = g_Wh_NT[loop]>>16;
						*p ++  = g_Wh_NT[loop]>>24;
					}

					comtaskadd = 1;
					break;
		  		}
			case CMD_SET_CALIB:
	    			{
						int32x_t * dadr;
						int32x_t sdata;
						//Bakbuf �͵�ַ��ŵ��ֽ� �ߵ�ַ��Ÿ��ֽ�
						Bakbuf[SerialBuffer.serialData[0]*2] = SerialBuffer.serialData[1];	//serialData[0]��Ϊ�������
						Bakbuf[SerialBuffer.serialData[0]*2+1] = SerialBuffer.serialData[2];

						sdata = SerialBuffer.serialData[2]*256 + SerialBuffer.serialData[1];//2���ֽ�У��ֵ����ϳ�һ��16λ����
						dadr = (TDK6545PARAADDR[SerialBuffer.serialData[0]]*4);	//ȡУ��ֵ��CE��������ַ

						EA = 0;
						memcpy_cex_1( dadr, &sdata);
						EA = 1;
						if(OSEEPROMWrite(EEPROM_TDK6545H_PARA_ADDR0, CAT24WCXX_ADDR,&Bakbuf[0],PARA_NUM) == FALSE)
							*p ++ = 0;//���ɹ�
						else
							*p ++ = 0x01;//����ֵ1��ʾ�ɹ�
						comtaskadd = 1;
						break;
                    }
			 case CMD_RSET_CALIB:
			 	 {
					for(i = 0;i <6; i++)
					{
						Bakbuf[2*i] = 0x00;
						Bakbuf[2*i+1] = 0x40;
					}
					for(i = 6; i < 9 ; i ++)
					{
						Bakbuf[2*i] = 0x00;
						Bakbuf[2*i+1] = 0x00;
					}
					Bakbuf[18] = 0x1d; //wrate = 29
					Bakbuf[19] = 0x00;
					EA = 0;
					for (i=0;i<10;i++)
					{
						//Bakbuf�е�ÿ2�ֽڣ��ϳ�16λ��
						sdata = Bakbuf[i*2+1]*256 + Bakbuf[i*2];
						dadr = (TDK6545PARAADDR[i]*4);//ȡCE��Ӧ�ô�ŵĵ�ַ
						memcpy_cex_1( dadr, &sdata);
					}
					EA = 1;
					*p ++  = 0x01;

					comtaskadd = 1;
					break;
			 	 }
			 case	CMD_READ_CALIB://��ȡУ�����
			 {
				 for(i = 0; i<20 ; i++)
				 {
					 *p++ = Bakbuf[i];
				 }
				 comtaskadd = 1;
				 break;
			 }
			 case CMD_CLEAR_P_ENERGY:										//  CLEAR ACTIVE ENERGY
				{
					for(loop = 0; loop < 5; loop++)
					{
						g_Wh_PT[loop] = 0;
						g_Wh_NT[loop] = 0;
						g_Varh_Q1[loop] = 0;
						g_Varh_Q2[loop] = 0;
						g_Varh_Q3[loop] = 0;
						g_Varh_Q4[loop] = 0;
					}

					g_EnergyClear = TRUE;

					*p ++  = 0x01;

					comtaskadd = 1;
					break;
				}
			 case CMD_READ_VERSION://������汾��
				 {
					 *p++ = VERSION;
					 comtaskadd = 1;
				 }
				 break;
			 case CMD_READ_STATUS://��״̬��
				 {
					 *p++ = net_status_lock;
					 comtaskadd = 1;
					 net_status_lock = 0;
				 }
				 break;
			 case CMD_READ_PARA:	//������
			 	 {
			 		 para_num = SerialBuffer.serialData[0];
				 		switch(para_num)
				 		{
				 			case CAL_FINISHED:
				 			{
				 				*p ++ = cal_finished;
				 				comtaskadd = 1;
				 			}
				 			break;
				 			case N_CURR:
				 			{
								*p ++ = n_Current;
								*p ++ = n_Current>>8;
								*p ++ = n_Current>>16;
								*p ++ = n_Current>>24;
								comtaskadd = 1;
				 			}
				 			break;
				 			default://û�������������ظ�
				 				comtaskadd = 0;
				 		}
			 	 }
				 break;
			 case CMD_WRITE_PARA:	//д����
			 	 {
			 		para_num = SerialBuffer.serialData[0];
			 		switch(para_num)
			 		{
			 			case CAL_FINISHED:
			 			{
			 				temp1 =  SerialBuffer.serialData[1];
			 				if(temp1 == 0 || temp1 ==1)//��־ֻ��Ϊ0 �� 1
			 				{
			 					cal_finished = temp1;
			 					if(OSEEPROMWrite(EEPROM_CAL_FINISHED, CAT24WCXX_ADDR,&cal_finished,1) == FALSE)
			 						*p ++ = 0;//���ɹ�
			 					else
			 						*p ++ = 0x01;//����ֵ1��ʾ�ɹ�
			 					comtaskadd = 1;
			 				}
			 				else
			 				{
			 					*p ++ = 0;
			 					comtaskadd = 1;
			 				}
			 			}
			 			break;
			 			case N_CURR:
			 			{
			 				temp1 = SerialBuffer.serialData[1] + SerialBuffer.serialData[2]*256;//ȡ2�ֽ��Ѿ�����
			 				if(temp1 > 0 && temp1 <=5000)//0~5AΪ�Ϸ�ֵ
			 				{
			 					n_Current = temp1;
			 					*p ++  = 0x01;
			 					comtaskadd = 1;
			 				}
			 				else
			 				{
			 					n_Current = 0;
			 					*p ++  = 0x00;
			 					comtaskadd = 1;
			 				}
			 			}
			 			break;
			 			default://û�����������޷���
			 					comtaskadd = 0;
			 		}
			 	 }
			 	 break;
			 case CMD_RELAY_CTR:
			 {
				 uint8_t relay_num;
				 uint8_t ctr_cmd;
				 relay_num = SerialBuffer.serialData[0];//�ƶ��ĵڼ�·�̵���
				 ctr_cmd = SerialBuffer.serialData[1];//��������
				 if(relay_num <=3 && relay_num >=1 && (ctr_cmd == OPEN || ctr_cmd == CLOSE))//�Ϸ�����
				 {
					 Relay_Ctr(relay_num,ctr_cmd);
					 *p ++ = 0x01;//����ֵ1��ʾ�ɹ�
					 comtaskadd = 1;
				 }
				 else
				 {
					 *p ++ = 0x00;//����ֵ1��ʾʧ��
					 comtaskadd = 1;
				 }
			 }
			 break;
	    }
	}
	if(comtaskadd)
		{
			//if(READPOWER == SerialBuffer.command)
			//	memset_x(SerialBuffer.serialData,  0, 10);

			SerialBuffer.command = (SerialBuffer.command | 0x80);
			SerialBuffer.length = p - SerialBuffer.serialData;
			checksum = 0x00;
			checksum += SerialBuffer.framehead;
			checksum += SerialBuffer.address;
			checksum += SerialBuffer.command;
			checksum += SerialBuffer.length;

			if(SerialBuffer.length != 0)
			{
				for(loop = 0; loop < SerialBuffer.length; loop ++)
					checksum += SerialBuffer.serialData[loop];
			}

			SerialBuffer.checksum = checksum;
			SerialBuffer.frameend = 0x16;
			Is_Txing = TRUE;
		}

		return comtaskadd;
}

//*****************���籣�����*************************
void ACfailureSave(void)
{
	uint32_t ce_status_tmp = cestatus;  // Read the CE's status word

	if (1 == (CE_SAG_A & ce_status_tmp)) // Power on phase A?
	{
		net_status |= BIT2; //set bit 2
		net_status_lock |= BIT2;
		SaveEnergy(1);
	}
	else
	{
		net_status &= (~BIT2); //clr bit 2
	}
	if( net_status != 0)	//�������
		TELL_MCU;
	else
		NOT_TELL;
}
//*****************�����жϷ������**********************

#pragma NOAREGS
void uart0_int (void) small interrupt ES0_IV using 1
{
	uint8i_t  temp,i;

	if(RI)
	{
		RI=0;

		if(Is_Rx_Ok) return;//�Ѿ�������1������û����

		if(rx_count > (RX_BUF_SIZE + 5))//���������
		{
			Init_Serial();
			return;
		}

		p_rx_buf[rx_count] = S0BUF;


		switch(rx_count)
		{
			case 0:	 //��68��ͷ
				if(p_rx_buf[0] == 0x68)
				{
					rx_count=1;
					ReceFrameStar = 1;//��ʼ���㳬ʱ
					ReceDelayCnt = REC_DELAY_MAX;
				}
				break;
			case 1:  //�жϵ�ַ ��ʵ��ַ,�㲥��ַ,����ͨѶר�õ�ַ
				if((p_rx_buf[1]==NetAddress) || (p_rx_buf[1] == 0xff) || (p_rx_buf[1] == 0xfe))
					rx_count=2;
				break;
			case 2: //������
				rx_count=3;
				break;
			case 3://�����ֽ�
				if(p_rx_buf[3] == 0)//����Ϊ0��׼������У���16
				{
					rx_count = 4 + RX_BUF_SIZE;
				}
				else
					rx_count = 4;
				break;

			case (RX_BUF_SIZE+4):
				rx_count=RX_BUF_SIZE+5;
				break;

			case (RX_BUF_SIZE+5):
				if(0x16 == p_rx_buf[rx_count])
				{
					temp=0;
					for(i=0; i < (p_rx_buf[3] + 4); i++)
						temp += p_rx_buf[i];	//checksum

					if (p_rx_buf[(RX_BUF_SIZE + 4)] == temp)
					{
						Is_Rx_Ok=TRUE;
						ReceFrameStar = 0;
						ReceDelayCnt = REC_DELAY_MAX;
						ReceDelayOut = 0;
					}
					else
						Init_Serial();//����У���룬�������
				}
				else
					Init_Serial();//������������������
				break;

			default:
				if((rx_count>=4) && (rx_count < (p_rx_buf[3] + 3)))
				{
					rx_count++;
				}
				else if(rx_count == (p_rx_buf[3] + 3))
				{
					rx_count = 4 + RX_BUF_SIZE;//׼������У��
				}

				break;
		}
	}
	if(TI)
	{
		TI=0;

		if (tx_count > 0)//send next byte if necessary
		{
			tx_count = tx_count - 1;
			ACC = *p_tx_buf;
			if(PSW & 0x01)
				S0CON |= 0x08;
			else
				S0CON &= 0xf7;

			S0BUF = *p_tx_buf;
			if(tx_count <= 2)
			{
				if(tx_count == 2)//����У����
					p_tx_buf = &SerialBuffer.checksum ;
				else//����֡β
					p_tx_buf = &SerialBuffer.frameend;
			}
			else
				p_tx_buf = p_tx_buf + 1;
		}
		else
		{
			Init_Serial();//�������ˣ����ý�����һ��׼��
		}
	}
}
//****************��ʱ��0�жϷ������**********************
//ÿ10ms�ж�һ��
void timer0_isr (void) interrupt TMR0_IV
{
	if( ReceFrameStar == 1)
	{
		if( ReceDelayCnt > 0)
		{
			if( --ReceDelayCnt == 0 )
				ReceDelayOut = 1;
		}

	}

	if(EarthDelayStar == 1)
	{
		if( EarthDelayCnt > 0)
		{
			if(--EarthDelayCnt == 0)
				EarthDelayOut = 1;
		}
	}
}

void Relay_Ctr(uint8_t num ,uint8_t cmd)
{
	switch(num)
	{
		case 1:
			if(cmd == OPEN)
				R1_CTR_OFF
			if(cmd == CLOSE)
				R1_CTR_ON
			break;
		case 2:
			if(cmd == OPEN)
				R2_CTR_OFF
			if(cmd == CLOSE)
				R2_CTR_ON
			break;
		case 3:
			if(cmd == OPEN)
				R2_CTR_OFF
			if(cmd == CLOSE)
				R2_CTR_ON
			break;
	}
}
/***************************************************************************
 * History:
 * $Log: ce43_ex.c,v $
 * Revision 1.2  2010/06/30 23:36:03  Ray.Vandewalker
 * Ported to B01
 *
 * Revision 1.1  2009/06/15 18:25:33  dsang
 * *** empty log message ***
 *
 *
 * Copyright (C) 2009 Teridian Semiconductor Corp. All Rights Reserved.
 * this program is fully protected by the United States copyright
 * laws and is the property of Teridian Semiconductor Corporation.
 ***************************************************************************/

