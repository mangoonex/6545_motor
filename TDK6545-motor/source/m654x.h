/***************************************************************************
 * This code and information is provided "as is" without warranty of any 
 * kind, either expressed or implied, including but not limited to
 * implied warranties of merchantability and/or fitness for a particular
 * purpose.
 *
 * Copyright (C) 2009,2010 Maxim Integrated Products Inc. All Rights Reserved.
 * AUTHOR:  MTF, RGV
 * HISTORY: See the end of file.
 * DESCRIPTION: 71M654x POWER METER SOC Family - Registers.  Follows
 * specification B1.3
 ***************************************************************************/
#ifndef _HDW_H
#define _HDW_H 1

/*========================================================================*/
// Digital input and output.
sfr PORT0 = 0x80;      // Direction and data register for PORT0[3:0].
sbit DIO_0 = PORT0^0;  // DIO_0/SEG0/WPULSE 
sbit DIO_1 = PORT0^1;  // DIO_1/SEG1/VPULSE
// DIOs 2..11 can be interrupts or timer gates.
sbit DIO_2 = PORT0^2;  // DIO_2/SEG2/SDCK
sbit DIO_3 = PORT0^3;  // DIO_3/SEG3/SDATA/SDO
sbit DIR_0 = PORT0^4;  // DIR_0 
sbit DIR_1 = PORT0^5;  // DIR_1
sbit DIR_2 = PORT0^6;  // DIR_2
sbit DIR_3 = PORT0^7;  // DIR_3

/*========================================================================*/
// DIOs 2..11 can be interrupts or timer gates.
sfr PORT1   = 0x90;    // Port 1. Bit addressable.
sbit DIO_4 = PORT1^0;  // DIO_4/SEG4 
sbit DIO_5 = PORT1^1;  // DIO_5/SEG5/TX2
sbit DIO_6 = PORT1^2;  // DIO_6/SEG6/XPULSE
sbit DIO_7 = PORT1^3;  // DIO_7/SEG7/YPULSE
sbit DIR_4 = PORT1^4;  // DIR_4 
sbit DIR_5 = PORT1^5;  // DIR_5
sbit DIR_6 = PORT1^6;  // DIR_6
sbit DIR_7 = PORT1^7;  // DIR_7

/*========================================================================*/
// DIOs 2..11 can be interrupts or timer gates.
sfr PORT2   = 0xA0;    // Port 2. Bit addressable.
sbit DIO_8 =  PORT2^0; // DIO_8/SEG8/DI
sbit DIO_9 =  PORT2^1; // DIO_9/SEG9
sbit DIO_10 = PORT2^2; // DIO_10/SEG10
sbit DIO_11 = PORT2^3; // DIO_11/SEG11
sbit DIR_8 =  PORT2^4; // DIR_8 
sbit DIR_9 =  PORT2^5; // DIR_9
sbit DIR_10 = PORT2^6; // DIR_10
sbit DIR_11 = PORT2^7; // DIR_11

/*========================================================================*/
sfr PORT3   = 0xB0;    // Port 2. Bit addressable.
sbit DIO_12 = PORT3^0; // DIO_12 
sbit DIO_13 = PORT3^1; // DIO_13
sbit DIO_14 = PORT3^2; // DIO_14
sbit DIO_15 = PORT3^3; // DIO_15
sbit DIR_12 = PORT3^4; // DIR_12 
sbit DIR_13 = PORT3^5; // DIR_13
sbit DIR_14 = PORT3^6; // DIR_14
#ifdef M6543
sbit DIR_15 = PORT3^7; // DIR_15
#endif

/*========================================================================*/
// Timer control, bit addressable.
sfr TCON  = 0x88;       // Timer/Counter Control.
sbit TF1   = TCON^7;    // Timer 1 overflow.
sbit TR1   = TCON^6;    // Timer 1 run.
sbit TF0   = TCON^5;    // Timer 0 overflow. 
sbit TR0   = TCON^4;    // Timer 0 run. 
sbit IE1   = TCON^3;    // Interrupt 1 occurred (Auto-cleared).
sbit IT1   = TCON^2;    // Interrupt 1 on falling edge /low-level.
sbit IE0   = TCON^1;    // Interrupt 0 occurred (Auto-cleared).
sbit IT0   = TCON^0;    // Interrupt 0 on falling edge /low-level.

#define   TF1_  0x80    // Timer 1 overflow.
#define   TR1_  0x40    // Timer 1 run.
#define   TF0_  0x20    // Timer 0 overflow. 
#define   TR0_  0x10    // Timer 0 run. 
#define   IE1_  0x08    // Interrupt 1 occurred (Auto-cleared).
#define   IT1_  0x04    // Interrupt 1 on falling edge /low-level.
#define   IE0_  0x02    // Interrupt 0 occurred (Auto-cleared).
#define   IT0_  0x01    // Interrupt 0 on falling edge /low-level.

/*========================================================================*/
sfr TMOD  = 0x89;       // Timer Mode Control.
#define T1_GATE_        0x80    // Timer 1 is gated via INT1.
#define T1_CNTR_        0x40    // Timer 1 is a counter.
#define T1_TMR_         0x00    // Timer 1 is a timer.
#define T1_13_          0x00    // Timer 1 is a 13-bit counter/timer.
#define T1_08_          0x20    // Timer 1 is a 8-bit auto-reload counter/timer.
#define T1_16_          0x10    // Timer 1 is a 16-bit counter/timer.
#define T1_STOP_        0x30    // Timer 1 is stopped.

#define T0_GATE_        0x08    // Timer 0 is gated via INT0.
#define T0_CNTR_        0x04    // Timer 0 is a counter.
#define T0_TMR_         0x00    // Timer 0 is a timer.
#define T0_13_          0x00    // Timer 0 is a 13-bit counter/timer.
#define T0_08_          0x02    // Timer 0 is a 8-bit auto-reload counter/timer.
#define T0_16_          0x01    // Timer 0 is a 16-bit counter/timer.
#define T0_08_2_        0x03    // Timer 0 is a dual 8-bit counters/timers.
/*========================================================================*/
sfr TL0   = 0x8A;       // Timer 0, low byte.
sfr TL1   = 0x8B;       // Timer 1, low byte.
sfr TH0   = 0x8C;       // Timer 0, high byte.
sfr TH1   = 0x8D;       // Timer 1, high byte.

//========================================================================//
// Serial ports
sfr SCON  = 0x98;       // Serial Port, Control Register. Bit addressable.
sfr S0CON = 0x98;       // Serial Port 0, Control Register.
sbit SM0   = S0CON^7;   // Mode bits.
sbit SM1   = S0CON^6;   //
sbit SM2   = S0CON^5;   //
sbit REN   = S0CON^4;   // Enable receiver.
sbit TB8   = S0CON^3;   // BIT8 xmt'd.
sbit RB8   = S0CON^2;   // BIT8 rcv'd.
sbit TI    = S0CON^1;   // TXD interrupt'd.
sbit RI    = S0CON^0;   // RXD interrupt'd.
sfr S1CON  = 0x9B;      // Serial Port 1, Control Register.

#define   SM0_   0x80   // Defines for S0CON = 0x98 & S1CON = 0x9B except SM_;
#define   SM1_   0x40
#define   SM2_   0x20
#define   REN_   0x10   // Enable receiver. 
#define   TB8_   0x08   // BIT8 xmt'd.
#define   RB8_   0x04   // BIT8 rcv'd.
#define   TI_    0x02   // TXD interrupt.
#define   RI_    0x01   // RXD interrupt.
                        // Defines for S0CON = 0x98 exclusively;
#define _8BIT_SERIAL0_    (0|SM1_)
#define _9BIT_SERIAL0_ (SM0_|SM1_)

#define SM_ 0x80        // Defines for S1CON = 0x9B exclusively;
#define _8BIT_SERIAL1_ SM_
#define _9BIT_SERIAL1_  0

//========================================================================//
sfr SBUF   = 0x99;      // Serial Port, Data Buffer.
sfr S0BUF  = 0x99;      // Serial Port 0, Data Buffer.
sfr S0RELH = 0xBA;      // Serial Port 0, Reload Register, high byte.
sfr S0RELL = 0xAA;      // Serial Port 0, Reload Register, low byte.
sfr WDCON = 0xD8;       // Other bits do nothing. 
sbit ES0_SEL = WDCON^7;  // 1 => Serial 0 clocked by S0REL.
                         // 0 => Serial 0 clocked by TH1 (Timer 1).
#define ES0_SEL_ 0x80

sfr S1BUF  = 0x9C;      // Serial Port 1, Data Buffer.
sfr S1RELH = 0xBB;      // Serial Port 1, Reload Register, high byte.
sfr S1RELL = 0x9D;      // Serial Port 1, Reload Register, low byte.

//========================================================================//
sfr PCON  = 0x87;       // Used only for serial BAUD rate.
#define SMOD_           0x80    // Double baud rate for Serial 0.
#define GF1_            0x08    // General use flag.
#define GF0_            0x04    // General use flag.
#define PD_             0x02    // STOP, power down. (deprecated)
#define IDL_            0x01    // Idle MPU. (deprecated)
#define PD              1       // Powerdown MPU. (deprecated)

//========================================================================//
// EEPROM controls
sfr EEDATA = 0x9E;      // Serial EEPROM interface data.
sfr EECTRL = 0x9F;      // Serial EEPROM interface control.
//--------------------------------------// 2-wire interface.
// EEPROM status bits
#define EE_ERROR      0x80              // RO: Illegal command is rcv'd.
#define EE_BUSY       0x40              // RO: Serial data bus is busy.
#define EE_RX_ACK     0x20              // RO: EEPROM sent an ACK.
#define EE_TX_ACK     0x10              // RO: ACK sent to EEPROM.
#define EE_CMD        0x0F              // RW: Command for EEPROM interface.
// EEPROM commands
#define  EE_NOOP      0x00              // No-op.
#define  EE_RCV_ACK   0x02              // Receive byte, send ACK.
#define  EE_TX        0x03              // Transmit byte.
#define  EE_STOP      0x05              // Issue 'STOP' sequence.
#define  EE_RCV       0x06              // Receive last byte, do not send ACK.
#define  EE_START     0x09              // Issue 'START' sequence. 
// All other commands are a no-op, that also asserts the error bit.

//--------------------------------------// 3-wire interface.   
#define EE_WFR     0x80                 // RW: Wait for READY.
//      EE_BUSY    0x40                 // RO: Serial data bus is busy.
#define EE_HiZ     0x20                 // RW: SD is to be floated immediately after last SCK rising edge.
#define EE_RD      0x10                 // RW: EEDATA is to be filled with data from EEPROM.
#define EE_CNT     0x0F                 // RW: Number of clocks to be issued; Allowed values: 0 thru 8.

//===========================================================================//
// Interrupt registers
// Together w/ IPH allows four priority levels for six groups of interrupts.
sfr IP     = 0xA9;      // Interrupt Priority Register.
sfr IP0    = 0xA9;      // Interrupt Priority Register 0.
sfr IPL	   = 0xA9;      // Low-order Interrupt Priority Register.
#define OWDS_           0x80    //
#define WDTS_           0x40    // Core watchdog timer status.
#define PX6_L_          0x20    // EX6      priority LSb.
#define PX5S0_L_        0x10    // EX5/ES0  priority LSb.
#define PX4T1_L_        0x08    // EX4/TMR1 priority LSb.
#define PX3X1_L_        0x04    // EX3/EX1  priority LSb.
#define PX2T0_L_        0x02    // EX2/TMR0 priority LSb.
#define PX0S1_L_        0x01    // EX0/ES1  priority LSb.
//===========================================================================//
sfr IP1    = 0xB9;      // Interrupt Priority Register 1.
sfr IPH	   = 0xB9;	    // High-order Interrupt Priority Register.
#define PX6_H_          0x20    // EX6      priority MSb.
#define PX5S0_H_        0x10    // EX5/ES0  priority MSb.
#define PX4T1_H_        0x08    // EX4/TMR1 priority MSb.
#define PX3X1_H_        0x04    // EX3/EX1  priority MSb.
#define PX2T0_H_        0x02    // EX2/TMR0 priority MSb.
#define PX0S1_H_        0x01    // EX0/ES1  priority MSb.
#define IP_MSK          0x3F    // Interrupt Priority Mask.

// Interrupt Vector Address = IV * 8 + 3.
#define EX0_IV 		 0  // 80515 external interrupt 0 vector.
#define IO_INT0_IV   0  // TSC IO interrupt 0 vector (set up with DIO_DIR0).

#define TMR0_IV		 1  // Timer 0 interrupt vector.

#define EX1_IV 		 2  // 80515 external interrupt 1 vector.
#define IO_INT1_IV   2  // TSC IO interrupt 1 vector (set up with DIO_DIR0).

#define TMR1_IV		 3  // Timer 1 interrupt vector.

#define ES0_IV 		 4  // Serial UART 0 interrupt vector.

#define EX2_IV 		 9  // 80515 external interrupt 2 vector.
#define CE_PULSE_IV  9  // TSC interrupt-on-CE-pulse vector.

#define EX3_IV 		10  // 80515 external interrupt 3 vector.
#define CE_BUSYZ_IV 10  // TSC CE not busy interrupt vector.

#define EX4_IV 		11  // 80515 external interrupt 4 vector.
#define VSTAT_IV    11  // TSC voltage status change interrupt vector.

#define EX5_IV 		12  // 80515 external interrupt 5 vector.
#define EEPROM_IV   12  // TSC EEPROM data move done interrupt vector.

#define EX6_IV 		13  // 80515 external interrupt 6 vector.
#define XFER_RTC_IV 13  // TSC data transfer ready and RTC vector.

#define ES1_IV 		16  // Serial UART 1 interrupt vector.

#define NULL_IV		17 // Not used by any hardware.

//===========================================================================//
sfr IE    = 0xA8;       // Interrupt Enable Register. Bit addressable.
sfr IEN0  = 0xA8;       // Interrupt Enable Register 0.
sbit EA         = IEN0^7;   // Enable/Disable ALL interrupts.
sbit WDT        = IEN0^6;   // Watchdog timer refresh.
sbit ES0        = IEN0^4;   // Enable Serial 0 interrupts.
sbit ET1        = IEN0^3;   // Enable Timer 1 interrupts.
sbit EX1        = IEN0^2;   // Enable External 1 interrupts.
sbit EX_IO_INT1 = IEN0^2;   // Enable TSC IO interrupt 1.
sbit ET0        = IEN0^1;   // Enable Timer 0 interrupts.
sbit EX0        = IEN0^0;   // Enable External 0 interrupts.
sbit EX_IO_INT0 = IEN0^0;   // Enable TSC IO interrupt 0.

#define EA_     0x80    // Enable/Disable ALL interrupts.
#define WDT_    0x40    // Watchdog timer refresh.
#define ES0_    0x10    // Enable Serial 0 interrupts.
#define ET1_    0x08    // Enable Timer 1 interrupts.
#define EX1_    0x04    // Enable External 1 interrupts.
#define ET0_    0x02    // Enable Timer 0 interrupts.
#define EX0_    0x01    // Enable External 0 interrupts.

//========================================================================//
sfr IEN1  = 0xB8;       // Interrupt Enable Register 1. Bit addressable.
sbit SWDT        = IEN1^6;    // 80515 Watchdog timer start/refresh. (deprecated)
sbit EX6         = IEN1^5;    // Enable 80515 External 6 interrupts.
sbit EX_XFER_RTC = IEN1^5;    // Enable TSC xfer ready/RTC 1 sec interrupt.
sbit EX5         = IEN1^4;    // Enable 80515 External 5 interrupts.
sbit EX_EEPROM   = IEN1^4;    // Enable TSC eeprom ready interrupt.
sbit EX4         = IEN1^3;    // Enable 80515 External 4 interrupts.
sbit EX_VSTAT    = IEN1^3;    // Enable TSC voltage changed interrupt.
sbit EX3         = IEN1^2;    // Enable 80515 External 3 interrupts.
sbit EX_CE_BUSYZ = IEN1^2;    // Enable TSC CE not busy interrupt.
sbit EX2         = IEN1^1;    // Enable 80515 External 2 interrupts.
sbit EX_CE_PULSE = IEN1^1;    // Enable TSC pulse interrupts.

#define SWDT_   0x40    // Watchdog timer start/refresh.
#define EX6_    0x20    // Enable External 6 interrupts.
#define EX5_    0x10    // Enable External 5 interrupts.
#define EX4_    0x08    // Enable External 4 interrupts.
#define EX3_    0x04    // Enable External 3 interrupts.
#define EX2_    0x02    // Enable External 2 interrupts.

sfr IEN2   = 0x9A;      // Interrupt Enable Register 2.
#define ES1_            0x01    // Enable/Disable SERIAL1 interrupts.

//========================================================================//
sfr IRCON = 0xC0;           // Interrupt Request Control Register. Bit addressable.
sbit IE6 = IRCON^5;         // External interrupt 6 (Auto-cleared).
sbit IE5 = IRCON^4;         // External interrupt 5 (Auto-cleared).
sbit IE4 = IRCON^3;         // External interrupt 4 (Auto-cleared).
sbit IE_VSTAT = IRCON^3;    // Power changed interrupt request.
sbit IE3 = IRCON^2;         // External interrupt 3 (Auto-cleared).
sbit IE_CE_BUSYZ = IRCON^2; // CE not busy interrupt request.
sbit IE2 = IRCON^1;         // External interrupt 2 (Auto-cleared).

#define IE6_    0x20    // External interrupt 6 (Auto-cleared).
#define IE5_    0x10    // External interrupt 5 (Auto-cleared).
#define IE4_    0x08    // External interrupt 4 (Auto-cleared).
#define IE3_    0x04    // External interrupt 3 (Auto-cleared).
#define IE2_    0x02    // External interrupt 2 (Auto-cleared).

//========================================================================//
sfr T2CON = 0xC8;       // 0 bit = falling edge, 1 bit = rising edge.
sbit I3FR   = T2CON^6;  // External 3 interrupt edge.
sbit I2FR   = T2CON^5;  // External 2 interrupt edge.

#define I3FR_   0x40    // External 3 interrupt edge.
#define I2FR_   0x20    // External 2 interrupt edge.

//========================================================================//
sfr FLAG0 = 0xE8;           // Identify multiplexed interrupts.
// Use these definitions only to read, never to clear the bit.
// A clear with these actually performs a read-modify-write,
// and other bits in the register can change after the read, but
// before the write.
sbit IE_EEX     = FLAG0^7;  // EEPROM operation done
sbit IE_XPULSE  = FLAG0^6;  // Y Pulse occurred
sbit IE_YPULSE  = FLAG0^5;  // Y Pulse occurred
sbit IE_RTCT    = FLAG0^4;  // RTC alarm
sbit IE_RTC1M   = FLAG0^2;  // RTC ticked, 1 minute
sbit IE_RTC1S   = FLAG0^1;  // RTC ticked, 1 second
sbit IE_XFER    = FLAG0^0;  // XFER data available

// Write the 1s-complement (bitwise invert) of these definitions to clear.
#define IE_EEX_         0x80    // EEPROM done bit.
#define IE_XPULSE_      0x40    // XPULSE occurred (usually on edge-crossing)
#define IE_YPULSE_      0x20    // YPULSE occurred (usually AC mains power fail)
#define IE_RTCT_        0x10    // RTC Alarm
#define IE_RTC1M_       0x04    // RTC minute ticked
#define IE_RTC1S_       0x02    // RTC second ticked
#define IE_XFER_        0x01    // XFER data available

// These macros safely clear the interrupt bits!
#define CLR_IE_EEX()        FLAG0 = ~IE_EEX_
#define CLR_IE_XPULSE()     FLAG0 = ~IE_XPULSE_
#define CLR_IE_YPULSE()     FLAG0 = ~IE_YPULSE_   
#define CLR_IE_RTCT()       FLAG0 = ~IE_RTCT_
#define CLR_IE_RTC1M()      FLAG0 = ~IE_RTC1M_
#define CLR_IE_RTC1S()      FLAG0 = ~IE_RTC1S_
#define CLR_IE_XFER()       FLAG0 = ~IE_XFER_


// Use these definitions only to read, never to clear the bit.
// A clear with these actually performs a read-modify-write,
// and other bits in the register can change after the read, but
// before the write.
sfr FLAG1 = 0xF8;           // Enable of multiplexed interrupts.
sbit IE_SPI     = FLAG1^7;  // SPI interrupt
sbit IE_WPULSE  = FLAG1^6;  // Wh Pulse occurred
sbit IE_VPULSE  = FLAG1^5;  // VARh Pulse occurred
sbit PB_STATE   = FLAG1^0;  // Pushbutton's state, read-only.
sbit DIO_PB     = FLAG1^0;  // Pushbutton's state, read-only.

// Write the 1s-complement (bitwise invert) of these definitions to clear.
#define IE_SPI_         0x80    // SPI interrupt occurred
#define IE_WPULSE_      0x40    // A Wh pulse occurred
#define IE_VPULSE_      0x20    // A VARh occurred
#define PB_STATE_       0x01    // Push button's state bit, read-only

// These macros safely clear the interrupt bits!
#define CLR_IE_SPI()        FLAG1 = ~IE_SPI_
#define CLR_IE_WPULSE()     FLAG1 = ~IE_WPULSE_
#define CLR_IE_VPULSE()     FLAG1 = ~IE_VPULSE_   
#define STRETCH_1_      0x01    // Default value.

/*========================================================================*/
// Memory management registers.
sfr CKCON = 0x8E;       // Clock Control (Default: Wait = 0, Stretch = 1).
#define WAIT_           0x70    // Wait states between instruction fetches.
#define WAIT_0_         0x00    // Default value.

//========================================================================//
// Flash SFR Register Definitions.
sfr FLSHCTL = 0xB2;    // Flash Control.
#define PREBOOT_   0x80 // In 'preboot' state, ICE locked out.
#define SECURE_    0x40 // In 'secure' mode, reading of FLASH prohibited.
#define WRPROT_BT_ 0x20 // write-protect boot code
#define WRPROT_CE_ 0x10 // write-protect CE code
#define FLSH_PEND_ 0x08 // A posted write is pending.
#define FLSH_PSTWR_ 0x04 // Enable posted writes.
#define FLSH_MEEN_ 0x02 // Mass Erase enable, auto-clears.
#define FLSH_PWE_  0x01 // Program Write Enable, MOVX @DPTR,A writes to code space (Flash).

//========================================================================//
sfr PGADR = 0xB7;           // Flash Page Erase Address.
#define PGADR_ 0xFE         // 6-bit Flash page address, auto-clears.

//========================================================================//
sfr ERASE = 0x94;           // Flash Erase.
#define PAGE_ERASE_ 0x55    // Initiate Flash Page Erase cycle..
                            // ..must be proceeded by a write to PGADR.
#define MASS_ERASE_ 0xAA    // Initiate Flash Mass Erase cycle..
// ..must be proceeded by a write to MEEN of FCTRL and..
// ..the debug (CC) port must be enabled. 

//========================================================================//
sfr FL_BANK = 0xB6;						// Flash bank. (6541G only)

#define FPAGE_SIZE 0x0400               // Size of Flash page in bytes.
#define FPAGE_MASK (FPAGE_SIZE-1)
#define NEXT_PAGE  1
#define BANK_SIZE 0x8000

//========================================================================//
sfr ADRMSB = 0xBF;  // MSB of PDATA addresses, for MOVX @Ri instructions.

/*========================================================================*/
sfr SP    = 0x81;       // Stack Pointer.
sfr DPL   = 0x82;       // Data Pointer Low.
sfr DPH   = 0x83;       // Data Pointer High.
sfr DPL0  = 0x82;       // Data Pointer Low  0.
sfr DPH0  = 0x83;       // Data Pointer High 0.
sfr DPL1  = 0x84;       // Data Pointer Low  1.
sfr DPH1  = 0x85;       // Data Pointer High 1.

#define SP_DEFAULT 7

//========================================================================//
sfr DPS   = 0x92;       // DPTR select.
#define DPS1_           0x80    // Select DPTR1.

//========================================================================//
sfr PSW   = 0xD0;       // Program Status Word. Bit addressable.
sbit CY    = PSW^7;     // Carry flag.
sbit AC    = PSW^6;     // Auxilary carry flag for BCD operations.
sbit F0    = PSW^5;     // General purpose flag.
sbit RS1   = PSW^4;     // RS1:0 Register bank select.
sbit RS0   = PSW^3;     // 
sbit OV    = PSW^2;     // Overflow flag.
sbit F1    = PSW^1;     // General purpose flag.   
sbit P     = PSW^0;     // Parity flag.   (i.e. EVEN parity).

#define CY_     0x80    // Carry flag.
#define AC_     0x40    // Auxilary carry flag for BCD operations.
#define F0_     0x20    // General purpose flag.
#define RS1_    0x10    // RS1:0 Register bank select.
#define RS0_    0x08    // 
#define OV_     0x04    // Overflow flag.
#define F1_     0x02    // General purpose flag.   
#define P_      0x01    // Parity flag (i.e. EVEN parity).


//===========================================================================//
sfr ACC   = 0xE0;       // Accumulator.
// Accumulator bits.
sbit   ACC_7 = ACC^7;
sbit   ACC_6 = ACC^6;
sbit   ACC_5 = ACC^5;
sbit   ACC_4 = ACC^4;
sbit   ACC_3 = ACC^3;
sbit   ACC_2 = ACC^2;
sbit   ACC_1 = ACC^1;
sbit   ACC_0 = ACC^0;

#define A   ACC
#define A_0 ACC_0
#define A_1 ACC_1
#define A_2 ACC_2
#define A_3 ACC_3
#define A_4 ACC_4
#define A_5 ACC_5
#define A_6 ACC_6
#define A_7 ACC_7
sfr B     = 0xF0;       // B-register.

//===========================================================================//
// Voltage status register.
sfr STAT    = 0x0F9;       // Status register.
#define PLL_OK 0x10       // PLL is OK.
#define VSTAT 0x07        // The bits that define the status.
enum eVSTAT {
    // When meter has full power, metering is accurate.
    // CE can take up to a second to resynchronize.
    _FULL_POWER = 0x00,             // Meter has all necessary power. 
    // At analog failure, meter is not accurate.
    _ANALOG_FAIL = 0x01,
    // Switched to battery power. MPU should sleep or go to LCD mode. 
    _BATTERY = 0x02,        // Digital hardware still works OK. 
    _FLASH_WRITE_FAIL = 0x03,   // Flash writes not possible.
    _MPU_FAIL = 0x05        // Battery low.  MPU will fail soon.
};

//===========================================================================//
sfr REMOTE0 = 0x0FC;       	// Remote status register.
// Parity errors should be cleared before sending, and read after sending.
#define PERR_RD 0x40        // Bad parity on data from remote IC.
#define PERR_WR 0x20		// Bad parity on data sent to remote IC.
#define RD_CODE 0x1C        // The bits that define the remote command code.
#define INTFC_NO 0x03       // The bits that select the remote.


//===========================================================================//
sfr SPICMD = 0xFD;         // SPI command received.

/*** I/O RAM **************************************************************/
// This section defines bit masks for items less than one byte.  The byte-wide
// and larger items are variables in the following section.

#define IO_BASE 0x2000        // Base of IO RAM.


//=========================100=MUX5 bit definitions===========================//
#define MUX_DIV          0xF0  // States per MUX cycle.
enum eMUX_DIV { _11 = 0xB0, _10 = 0xA0, _9 = 0x90,  _8 = 0x80, 
                _7 = 0x70,  _6 = 0x60,  _4 = 0x40,  _3 = 0x30, 
                _2 = 0x20, _1 = 0x10,  _OFF = 0x00 };  

//=========================100-105 MUX bit definitions==== ==================//
// MUX[6]

//=========================106=CE6 bit definitions===========================//
#define EQU              0xE0   // Specifies power equation.  See options.h
enum eEQU { 
// Meter equations                   // Element 0      | El. 1   | El. 2
    _1element_2wire =   0x00,        // VAxIA          | VAxIB   | N/A
    _1element_3wire =   0x20,        // VAx(IA - IB)/2 | VAxIB   | N/A
    _2element_3wire_d = 0x40,        // VAxIA          | VBxIB   | N/A
    _2element_4wire_d = 0x60,        // VAx(IA - IB)/2 | VBxIB   | VCxIC
    _2element_3wire_y = 0x80,        // VAx(IA - IB)/2 | VBx(IC-IB)/2 | N/A
    _3element_4wire_y = 0xA0         // VAxIA          | VBxIB   | VCxIC
};  

#define  CHOP_E         0x0C  // Chop enable.
enum eCHOP_E { _ENABLED = 0x00, _POSITIVE = 0x04, 
    _REVERSED = 0x08,  _ALWAYS = 0x0C};

#define  RTM_E          0x02  // Real Time Monitor enable (When '0', the RTM output is low).

#define  CE_E           0x01   // Compute Engine Enable.

#define CE_ENABLE()      CE6 |= CE_E; \
    FLAG0 = ~(IE_XFER_)
#define CE_DISABLE()     CE6 &= ~CE_E
#define CE_BUSY          IE_CE_BUSYZ
#define CE_ACTIVE        (CE6 & CE_E)

//=========================107=CE5 bit definitions===========================//
//=========================108=CE4 bit definitions===========================//
// SUMSAMPS

//=========================109=CE3 bit definitions===========================//
// CE_LCTN

//=========================10A=CE2 bit definitions===========================//
// PLS_MAXWIDTH

//=========================10B=CE1 bit definitions===========================//
// PLS_INTERVAL

//=========================10C=CE0 bit definitions===========================//
#define DIFF3_E 0x80    // ADC0 & 1 Differential
#define DIFF2_E 0x40    // ADC2 & 3 Differential
#define DIFF1_E 0x20    // ADC4 & 5 Differential
#define DIFF0_E 0x10    // ADC6 & 7 Differential
#define RFLY_DIS 0x08   // 1 = push and pull, 0 = push only
#define FIR_LEN 0x06    // PLL_FAST:00=141, 01 = 288, 10 = 384 cycles
                        // PLL_SLOW:00=135, 01 = 276, 10 = Invalid
#define PLS_INV 0x01    // 1 = inverted (active high)

//=========================10D-111 RTM ======================================//

//=========================200 CKGN =========================================//
#define ADV_DIV  0x20   // ADC divider, 0=4.92MHz, 1=2.46MHz
#define PLL_FAST 0x10   // Master clock, from PLL, 0=4.92MHz, 1=19.66MHz
#define RESET    0x08   // 1=reset the IC
#define MPU_DIV  0x07   // MPU clock = (master clock)/(2^(2+MPU_DIV))

//=========================300 TRIMS ========================================//
enum eTRIMSEL { 
    _CHIPID = 0x80, _TRIMVDD = 0x82, _TRIMC, _TRIMI, _TRIMM, _P_VDD, 
    _TRIMT = 0x89, _TEMP22, _TEMP_TRIM = 0x8D, _P_NV, _TRIMuREF, 
    _TEMP85, _TRIMBGB = 0x92, _TRIMBGD = 0x94
};

//=========================400 LCD0 =========================================//
#define  LCD_E           0x80  // Enables the LCD display.
#define  LCD_MODE        0x70  // The LCD bias mode.  
enum eLCD_MODE 
{ 
    BIAS_3rd_4_STATES  = 0x00, BIAS_3rd_3_STATES  = 0x10,
    BIAS_half_2_STATES = 0x20, BIAS_half_3_STATES = 0x30, 
    BIAS_STATIC = 0x40,
    BIAS_3rd_5_STATES  = 0x50, BIAS_3rd_6_STATES  = 0x60
};
#define  LCD_ALLCOM      0x08  // 1 = SEG/COM pins = COM
#define  LCD_Y           0x04  // Blink frequency 1 => 1.0 Hz; 0 => 0.5 Hz.
#define  LCD_CLK         0x03  // Sets the LCD clock frequency.
enum eLCD_CLK { _64HZ, _128HZ, _300HZ, _600HZ };

//=========================401 LCD1 =========================================//
#define LCD_VMODE 0x0C0     
enum eLCD_VMODE { _NoBoostNoDac = 0x00, _Dac = 0x40, 
    _BoostDac = 0x50, _ExternalVLcd = 0x60 };
#define LCD_BLNKMAP23 0x3F

//=========================402 LCD2 =========================================//
#define LCD_BAT       0x80  // 1=Connect LCD power supply to VBAT.
#define LCD_TEST      0x40  // 1=LCD test.  All pins make a test pattern.
#define LCD_BLNKMAP22 0x3F

//=========================405..40B LCD_MAP[] ===============================//
//=========================40C LCD4 =========================================//
#define LCD_RST       0x04  // 1=Connect LCD power supply to VBAT.
#define LCD_BLANK     0x02  // Clear all LCD segments. 
#define LCD_ON        0x01  // 1=Invert effect of LCD_BLANK (all segs set)

//=========================456 DIO0 bit definitions==========================//
#define DIO_EEX          0xC0  // Configure EEPROM interface.
//           DIO 4 & 5;         2-wire EEPROM;     3-wire EEPROM.
enum eDIO { DIO_EEX_DSB = 0x00, DIO_EEX_2W = 0x40, DIO_EEX_3W = 0x80,
DIO_EEX_SPI = 0x0C0 };

#define OPT_TXE          0x0C  // Configure OPT_TX as output pin.
enum eOPT_TXE{ _DIO2 = 0x00, _OPT_TX = 0x04, _WPULSE = 0x08, _VARPULSE = 0x0C };

#define OPT_TXMOD        0x02  // 1 => Enable OPT_TX modulation.
#define OPT_TXINV        0x01  // 1 => Invert output from OPT_TX.

//=========================457 DIO1 bit definitions==========================//
#define DIO_PW           0x80  // WPULSE   tied to SEG/DIO 0
#define DIO_PV           0x40  // VARPULSE tied to SEG/DIO 1
#define OPT_FDC          0x30  // OPT_TX modulation duty cycle low.
enum eOPT_FDC { _50percent, _25percent=0x10, 
    _12_5percent=0x20, _6_25percent=0x30 };
#define OPT_RXDIS        0x04  // 0 => OPT_RX as analog input to optical UART.
#define OPT_RXINV        0x02  // 1 => Invert input from OPT_RX.
#define OPT_RXBB         0x01  // 1 => Input of optical port is DIO_5.

//=========================458 DIO2 bit definitions==========================//
#define DIO_PX           0x80  // PULSE X  tied to SEG/DIO 6
#define DIO_PY           0x40  // PULSE Y  tied to SEG/DIO 7

//=========================500 SPARE_NV bit definitions======================//
#define TEMP_TEST        0x03  // temperature test selection.
enum eTEMP_TEST { _normal, _1uAdiode, _2uAdiode, _BatteryMonitor };

//=========================600 6000 interface ===============================//

//=========================700 INT1_E Interrupt enables =====================//
#define EX_EEX           0x80  // Enable EEPROM interrupt
#define EX_XPULSE        0x40  // Enable X-PULSE interrupt (AC power fail)
#define EX_YPULSE        0x20  // Enable Y-PULSE interrupt (each edge-crossing)
#define EX_RTCT          0x10  // Enable alarm clock interrupt

#define EX_RTC1M         0x04  // Enable real-time-clock interrupt-per-minute
#define EX_RTC1S         0x02  // Enable real-time-clock interrupt-per-second
#define EX_XFER          0x01  // Enable end of sum cycle interrupt (XFER_BUSYZ)

//=========================701 INT2_E Interrupt enables =====================//
#define EX_SPI           0x80  // Enable serial peripheral interface interrupt
#define EX_WPULSE        0x40  // Enable Wh-pulse interrupt
#define EX_VPULSE        0x20  // Enable VARh-pulse interrupt

//=========================702 SECURE =======================================//
#define FLSH_UNLOCK      0xF0  // If unlock value set, flash can erase.
// bit 0x08 is reserved, and should be zero at all times.
#define FLSH_RDE         0x04  // Indicates that flash can be read by SPI.
#define FLSH_WRE         0x02  // Indicates that flash may be writted by SPI.
// bit 0x01 is reserved, and should be zero at all times.

//=========================704 ANALOG0 ======================================//
#define VREF_CAL         0x80  // 1 = Bring Vref out to (or in from) the pin.
#define VREF_DIS         0x40  // 1 = Disable the internal voltage reference.
#define PRE_E            0x20  // 1 = Enable an 8x preamp on the ADC input.
#define ADC_E            0x10  // 1 = Enable ADC and VRef; 0 saves bias current.
#define BCURR            0x08  // Connect a 100ua load to battery under test.
// Bits 0x07 are reserved.

//=========================708 SPI0 =========================================//
//=========================709 RCE0 =========================================//
// Remote control data.
#define CHOPR           0xC0    // Remote chop
#define RMTC_E          0x20    // Enable remote C
#define RMTB_E          0x10    // Enable remote B
#define RMTA_E          0x08    // Enable remote A
#define TMUXRC          0x07    // Select test multiplexor output for remote C
#define TMUXRC_PTR      0x2709  // Address multiplexor output for remote C
#define TMUXRC_BIT      0       // Shift bits for multiplexor output C
//=========================70A RTMUX ========================================//
#define TMUXRB          0x70    // Select test multiplexor output for remote B
#define TMUXRB_PTR      0x270A  // Address test multiplexor output for remote B
#define TMUXRB_BIT      0x04    // Shift bits for multiplexor output B
#define TMUXRA          0x07    // Select test multiplexor output for remote A
#define TMUXRA_PTR      0x270A  // Address test multiplexor output for remote A
#define TMUXRA_BIT      0       // Shifts bits for multiplexor outputs A.
//=========================70B INFOPG =======================================//
//=========================70C DIO3 =========================================//
#define PORT_E          0x20    // Enable port outputs.
#define SPI_E           0x10    // Enable SPI
#define SPI_SAFE        0x08    // SPI Safe; Limits SPI writes to 16-byte area.

//=========================887 LKPADDR ======================================//
#define LKUPAUTOI       0x80    // Autoincrement look-up address.

//=========================889 LKPCTRL ======================================//
#define LKP_RD          0x02    // Read slow NVRAM area.
#define LKP_WR          0x01    // Write slow NVRAM area.

//=========================890 RTC0 =========================================//
#define RTC_WR          0x80    // 1 = Freeze RTC for writes. 0 = start write
#define RTC_RD          0x40    // 1 = Freeze RTC output for reading.
#define RTC_FAIL        0x10    // 1 = RTC can't be trusted. A count was wrong.
enum eDAY { SUN = 1, MON, TUE, WED, THU, FRI, SAT };
enum eMONTH { JAN = 1, FEB, MAR, APR, MAY, JUN, JUL, AUG, SEP, OCT, NOV, DEC };

//=========================8A0 RTC0 =========================================//
// Each temperature measurement uses current.  See the spec.
#define TEMP_BSEL       0x80    // 1 = Measure VBat, 0 = Measure VBat-RTC
#define TEMP_PWR        0x40    // 1 = Temp sensor powered by V3P3D, 0 = RTC bat
#define OSC_COMP        0x20    // 1 = Freeze RTC for writes. 0 = start write
#define TEMP_BAT        0x10    // 1 = Measure VBat when temp. measured.
#define TBYTE_BUSY      0x08    // 1 = Previous write to TEMP not done.
// Less frequent will make the battery last longer.
#define TEMP_PER        0x07    // Period between temp. measurements.
enum eTEMP_PER { 
    _TP_NEVER, // Never measure.
    _TP_16S, _TP_32S, _TP_64S, _TP_128S, _TP_256S, _TP_512S, // Seconds between measurements.
    _TP_ALWAYS }; // As often as possible.
//=========================8B0 WF1 ==========================================//
#define WF_CSTART       0x80    // 1 = Wake from cold start; RTC is invalid.
#define WF_RST          0x40    // 1 = Wake from reset pin.
#define WF_RSTBIT       0x20    // 1 = Wake from software set of reset bit.
#define WF_OVF          0x10    // 1 = Wake from watchdog timer overflow.
#define WF_ERST         0x08    // 1 = Wake from emulator reset.
#define WF_BADVDD       0x04    // 1 = Wake after low battery voltage.
#define COLDSTART() (0 != (WF1 & WF_CSTART))
//=========================8B1 WF2 ==========================================//
#define WF_TMR          0x20    // 1 = Wake from wake timer.
#define WF_RX           0x10    // 1 = Wake from serial receiver.
#define WF_PB           0x08    // 1 = Wake from push button.
#define WF_W4           0x04    // 1 = Wake from DIO_4 on.
#define WF_W52          0x02    // 1 = Wake from DIO52 on.
#define WF_W55          0x01    // 1 = Wake from DIO55 on.
//=========================8B2 MISC =========================================//
#define SLEEP           0x80    // 1 = Request to enter sleep mode.
#define LCD_ONLY        0x40    // 1 = Request to enter LCD-only mode.
#define WAKE_ARM        0x20    // 1 = Arms and loads the wake timer.
//=========================8B3 WAKE_E =======================================//
#define EW_RX           0x10    // 1 = Enable wake from RX.
#define EW_PB           0x08    // 1 = Enable wake from pushbutton.
#define EW_W4           0x04    // 1 = Enable wake from DIO 4.
#define EW_W52          0x02    // 1 = Enable wake from DIO 52.
#define EW_W55          0x01    // 1 = Enable wake from DIO 55.
//=========================8B4 WDRST ========================================//
#define WD_RST          0x80    // 1 = Restart watchdog.
#define TEMP_START      0x40    // 1 = Start temeprature measurement.
#define RESET_WD()  WDRST = WD_RST

extern volatile uint8x_t MUX[6];
extern volatile uint8x_t CE6;
extern volatile uint8x_t CE5;
extern volatile uint8x_t CE4;
extern volatile uint8x_t CE3;
extern volatile uint8x_t CE2;
extern volatile uint8x_t CE1;
extern volatile uint8x_t CE0;
extern volatile uint16x_t RTM0;
extern volatile uint8x_t RTM[3];
extern volatile uint8x_t CKGN;
extern volatile uint16x_t CHIPID;
extern volatile int8x_t TRIMT;
extern volatile uint8x_t LCD0;
extern volatile uint8x_t LCD1;
extern volatile uint8x_t LCD2;
extern volatile uint8x_t LCD_MAP[7];
extern volatile uint8x_t LCD4;
extern volatile uint8x_t LCD_DAC;
extern volatile uint8x_t SEGDIO[55];
extern volatile uint8x_t DIO_R[6];
extern volatile uint8x_t DIO0;
extern volatile uint8x_t DIO1;
extern volatile uint8x_t DIO2;
extern volatile uint8x_t SPARENV;   
extern volatile uint8x_t FOVRD;
extern volatile uint8x_t TMUX;
extern volatile uint8x_t TMUX2;
extern volatile uint8x_t RTCA_ADJ;
extern volatile uint16x_t REMOTE;   
extern volatile uint8x_t INT1_E;   
extern volatile uint8x_t INT2_E;   
extern volatile uint8x_t SECURE;
extern volatile uint8x_t ANALOG0;
extern volatile uint8x_t VERSION;
extern volatile uint8x_t INTBITS;
extern volatile uint8x_t SPI0;
extern volatile uint8x_t RCE0;
extern volatile uint8x_t RTMUX;
extern volatile uint8x_t INFO_PG;
extern volatile uint8x_t DIO3;
#define NVRAM[_x_] (*(volatile uint8x_t *)(0x2800 + ( _x_ )))
extern volatile uint8x_t WAKE;
extern volatile int16x_t STEMP;
extern volatile uint8x_t BSENSE;
extern volatile uint8x_t LKPADDR;
extern volatile uint8x_t LKPDATA;
extern volatile uint8x_t LKPCTRL;
extern volatile uint8x_t RTC0;
extern volatile uint8x_t RTC2;
extern volatile uint8x_t RTC[7];   
extern volatile uint8x_t RTC_PNQREG[3];
extern volatile uint8x_t RTC_TMIN;
extern volatile uint8x_t RTC_THR;
extern volatile uint8x_t TEMP;
extern volatile uint8x_t WF1;
extern volatile uint8x_t WF2;
extern volatile uint8x_t MISC;
extern volatile uint8x_t WAKE_E;
extern volatile uint8x_t WDRST;

//YANGKE in order to access dio16~dio31 20180103
extern volatile uint8x_t DIO19;
extern volatile uint8x_t DIO20;
extern volatile uint8x_t DIO21;
extern volatile uint8x_t DIO22;
extern volatile uint8x_t DIO23;
extern volatile uint8x_t DIO24;
extern volatile uint8x_t DIO25;
extern volatile uint8x_t DIO28;
extern volatile uint8x_t DIO29;

#define SUBSEC      RTC2
#define RTC_SEC     RTC[0]
#define RTC_MIN     RTC[1]
#define RTC_HR      RTC[2]
#define RTC_DAY     RTC[3]
#define RTC_DATE    RTC[4]
#define RTC_MO      RTC[5]
#define RTC_YEAR    RTC[6]

#define TEMP_CAL0 (220) // Factory calibration temperature in 0.1C.
#define TEMP_CAL1 (55214L) // 6000: 55214 ~=(((2^14)*0.337*10)
#define TEMP_CAL2 (25165L) // 6000: 25165 ~=((2^24*0.00015*10)

// Values for RTC subsec counter
#define CLK_TCK 128L /* tck/S */
#define CLK_ROUND (8L) /* max milliseconds in 1 tick */
#define SUBSEC_MASK (0x7F) // 7 bits are significant.

// Values for RTC_PNQ
#define RTC_PNQ_MIN 261885L // minimum value, nominal + 988ppm
#define RTC_PNQ_NOM 262144L // nominal, center value
#define RTC_PNQ_MAX 262403L // maximum value, nominal - 988ppm

// CE definitions.
#define CE_DATA_BASE 0x0000  // Base of CE Data RAM (4-byte wide elements).
#define CE_DATA_SIZE 0x0140  // Length of CE Data RAM in 32-bit words.

#define CE_PARM_BASE 0x0040  // Start of CE's parameters in XDATA
#define CE_PARM_SIZE 0x00C0  // Length of parameters in 8 bit bytes

// Definitions for 'ceconfig'.
#define CE_6K_TEMP      0x800000    // When set using 6000 devices on current
#define CE_EXT_TEMP     0x400000    // when set, gain_adj set by MPU
#define CE_EDGE_INT     0x200000    // 1=pulse 2 interrupts MPU at mains freq. 
#define CE_SAG_INT      0x100000    // 1=pulse 3 interrupts MPU at sag_ctr=0.
#define CE_SAG_CNT      0x0FFF00    // count of samples of sag
#define CE_FREQSEL      0x00C0      // Select VA..VD for sag.
#define CE_EXT_PULSE    0x0020      // 1=pulse using external inputs
#define CE_I2_SHUNT     0x0010      // 1=8x gain on I2
#define CE_I1_SHUNT     0x0008      // 1=8x gain on I1
#define CE_I0_SHUNT     0x0004      // 1=8x gain on I0
#define CE_PULSE_FAST   0x0002      // 1=pulse faster
#define CE_PULSE_SLOW   0x0001      // 1=pulse slower 

// CE variables

// CE configuration variables
extern volatile int32x_t iadc67;
extern volatile int32x_t cal_i0;
extern volatile int32x_t cal_v0;
extern volatile int32x_t phadj_0;
extern volatile int32x_t cal_i1;
extern volatile int32x_t cal_v1;
extern volatile int32x_t phadj_1;
extern volatile int32x_t cal_i2;
extern volatile int32x_t cal_v2;
extern volatile int32x_t phadj_2;
extern volatile int32x_t cal_i3;
extern volatile int32x_t degscale;
extern volatile int32x_t ce_ppmc1;
extern volatile int32x_t ce_ppmc2;

extern volatile int32x_t ceconfig;
extern volatile int32x_t wrate;
extern volatile int32x_t kvar;
extern volatile int32x_t sumpre;
extern volatile int32x_t sag_thr;

extern volatile int32x_t quant_va;
extern volatile int32x_t quant_ia;
extern volatile int32x_t quant_a;
extern volatile int32x_t quant_vara;

extern volatile int32x_t quant_vb;
extern volatile int32x_t quant_ib;
extern volatile int32x_t quant_b;
extern volatile int32x_t quant_varb;

extern volatile int32x_t quant_vc;
extern volatile int32x_t quant_ic;
extern volatile int32x_t quant_c;
extern volatile int32x_t quant_varc;

extern volatile int32x_t quant_id;
extern volatile int32x_t iadc67;

// (the CE's zero-terminated file name starts at 0x38)

// (the CE word at 0x3F can be overwritten by a checksum)

extern volatile int32x_t gain_adj[5];

extern volatile int32x_t apulsew;
extern volatile int32x_t wpulse_ctr;
extern volatile int32x_t wpulse_frac;
extern volatile int32x_t wsum_accum;
extern volatile int32x_t apulser;
extern volatile int32x_t rpulse_ctr;
extern volatile int32x_t rpulse_frac;
extern volatile int32x_t vsum_accum;


// Outputs from CE.
extern volatile int32_t pdata cestatus;   
extern volatile int32_t pdata temp_ce;   
extern volatile int32_t pdata freq;   
extern volatile int32_t pdata mainedge;   

extern volatile int32_t pdata wsum;   
extern volatile int32_t pdata w0sum;   
extern volatile int32_t pdata w1sum;   
extern volatile int32_t pdata w2sum;   

extern volatile int32_t pdata varsum;   
extern volatile int32_t pdata var0sum;   
extern volatile int32_t pdata var1sum;
extern volatile int32_t pdata var2sum;

extern volatile int32_t pdata i0sqsum;
extern volatile int32_t pdata i1sqsum;   
extern volatile int32_t pdata i2sqsum;   
extern volatile int32_t pdata idsqsum;   

extern volatile int32_t pdata v0sqsum;   
extern volatile int32_t pdata v1sqsum;   
extern volatile int32_t pdata v2sqsum;   
extern volatile int32_t pdata vdsqsum;   

extern volatile int32_t pdata ph_atob; 
extern volatile int32_t pdata ph_atoc;

extern volatile int32_t pdata i0sqres;   
extern volatile int32_t pdata i1sqres;   
extern volatile int32_t pdata i2sqres;   
extern volatile int32_t pdata idsqres;

#ifndef CeData
extern const unsigned char code CeData[];
#endif
#define ce_name ((int8r_t *)&CeData[ 0x38 * 4 ])

// Definitions for 'cestatus'.
#define CE_SAG_A   0x001
#define CE_SAG_B   0x002
#define CE_SAG_C   0x004
#define CE_F0      0x008

/***************************************************************************
 * History:
 * $Log: m654x.h,v $
 * Revision 1.4  2010/06/30 23:36:04  Ray.Vandewalker
 * Ported to B01
 *
 * Revision 1.30  2010/06/26 03:56:44  Ray.Vandewalker
 * Added commit" to logging.  Added shorter nap for me.
 *
 * Revision 1.29  2010/06/16 03:47:14  Ray.Vandewalker
 * *** empty log message ***
 *
 * Revision 1.28  2010/05/24 22:26:19  Ray.Vandewalker
 * 5.2g
 * First cut at 2-pole shunt filtering.
 * Changed name to Maxim.
 * version broken out to a separate file.
 *
 * Revision 1.27  2010/05/20 01:18:34  tvander
 * Has RTC alarm clock interrupt, also temperature compensation constants.
 *
 * Revision 1.26  2010/04/28 18:39:20  tvander
 * Pulse width is unused by both CE and MPU code.
 *
 * Revision 1.25  2010/04/16 23:11:09  tvander
 * Made CE_ENABLE macro active.
 *
 * Revision 1.24  2010/03/26 21:08:59  tvander
 * First cut that works with B01
 *
 * Revision 1.23  2010/02/09 00:42:48  tvander
 * Updated temperature slope for 6000
 *
 * Revision 1.22  2010/01/15 18:41:04  tvander
 * fixed OPT_TXE's enumeration
 *
 * Revision 1.21  2010/01/11 21:26:51  tvander
 * Updated and enabled temperature scale.
 *
 * Revision 1.20  2010/01/04 22:31:15  tvander
 * Clean build
 *
 * Revision 1.19  2009/12/19 00:32:36  tvander
 * Added the fixed-point version of the remote sensor's temperature sensor
 * slope.
 *
 * Revision 1.18  2009/11/12 17:06:39  tvander
 * Corrected linear temperature constant for remote code.
 *
 * Revision 1.17  2009/11/03 21:52:10  tvander
 * Changes for temperature measurement using a 6000.
 * Fixed the read code for a 6000.
 *
 * Revision 1.16  2009/06/16 05:23:24  tvander
 * *** empty log message ***
 *
 * Revision 1.15  2009/05/29 18:18:15  tvander
 * Integrated 6000
 *
 * Revision 1.14  2009/05/07 20:24:33  tvander
 * Added fractional pulses.
 *
 * Revision 1.13  2009/04/22 22:32:55  tvander
 * Changed name of pulse enable.
 *
 * Revision 1.12  2009/04/21 00:39:39  tvander
 * Modified to permit actual use of nonvolatile memory.
 *
 * Revision 1.11  2009/04/16 18:41:31  tvander
 * *** empty log message ***
 *
 * Revision 1.10  2009/04/14 17:36:42  dsang
 * Fix incorrect FLSH_PEND_ bit
 *
 * Revision 1.9  2009/04/10 20:55:10  tvander
 * Corrected address of RTCA_ADJ
 *
 * Revision 1.8  2009/04/09 22:14:21  tvander
 * Added RTC_ADJ
 *
 * Revision 1.7  2009/04/07 21:00:49  tvander
 * CLK_TCK was off by a factor of two.
 *
 * Revision 1.6  2009/04/06 20:49:15  tvander
 * Clock resolution and direction for RTC_SBSC is included, now.
 *
 * Revision 1.5  2009/03/30 18:48:17  tvander
 * TEMP_PWR and BSEL are at bits 6 and 7 respectively.
 *
 * Revision 1.4  2009/03/14 02:19:14  tvander
 * *** empty log message ***
 *
 * Revision 1.3  2009/03/07 03:09:17  tvander
 * *** empty log message ***
 *
 * Revision 1.2  2009/03/04 19:03:10  tvander
 * Fixed starting address of CE data.
 *
 * Revision 1.1  2009/03/02 10:13:06  tvander
 * *** empty log message ***
 *
 *
 * Constructed from 80515.h, reg653x.h and io653x.h, 8/27/08
 *
 * Copyright (C) 2005 Maxim Integrated Products Inc. All Rights Reserved.
 * this program is fully protected by the United States copyright
 * laws and is the property of Maxim Integrated Products Inc.
 ***************************************************************************/
#endif  /* _HDW_H */
