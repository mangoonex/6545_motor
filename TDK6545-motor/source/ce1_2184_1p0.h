/***************************************************************************
 * This code and information is provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a particular
 * purpose.
 *
 * Copyright (C) 2008 Maxim Integrated Products Inc. All Rights Reserved.
 * DESCRIPTION: 71M65xx POWER METER - CE code constants.
 *
 * HISTORY: See end of file
 **************************************************************************/
// MUX_DIV is 7
// FIR length is 1 clock cycle, 288 ADC clocks
#define FS_INT 2184     // Sample frequency is 2184.53 Hz.
#define FS_FLOAT (32768.0/15.0)
// PRESAMPS = 00, 42
// SUMCYCLES = 34, 52
#define SUMPRE (FS_INT)  // Fixed SUMPRE for reference meter.
// CE's PLL is stable after 1 second, so first valid data is after that.
#define CE_PLL_OK 2 // Accumulation intervals before CE's PLL is stable.

#define WH_PER_CE   (1.0851e-14)    // (1/100)*1.0851x10^-12 VMAX * IMAX / In8
// The Wh scale applied to VARh, as well.
#define W_PER_CE    (3.906360e-11)  // (1/100)*1.0851x10^-12*3600 
// The W scale applies to VAR, as well.
// Multiplied by 10^2 because a_max is in 0.1A units.
// Multiplied by FS_INT because it's divided by sumpre to compensate for a changed
// accumulation interval.
#define A_PER_CE_SQ (0.085314)      // ((1000/10)^2 * 1.0851x10^-12 * 3600 * FS_INT)
#define SMALL_IRMS_THRESHOLD 59 // corresponds to 0.1A, LSB same as i0sqsum
#define V_PER_CE_SQ (A_PER_CE_SQ)
#define TENTH_HZ    (0x001E0000l / 10)   // 15 * 2^17 / 10 = (2^32 / (2^15 / 15)) / 10.
#define TWENTIETH_HZ (0x001E0000l / 20)  // 15 * 2^17 / 10 = (2^32 / (2^15 / 15)) / 10.
#define SEVENTY_HZ  (TENTH_HZ * 700)
#define hz_freq(d) ((d + TWENTIETH_HZ) / TENTH_HZ)
#define DEG_SCALE (-22195L)         // (temp_nom - temp-raw)/DEG_SCALE = temp in 0.1C units
#define TEMP_NOM (25396442L)        // must be calibrated for each unit.
#define LOGIC_BAD_MV (2000)         // logic may fail at 2V (2000 mV)
// Filtered VBAT
//#define VBAT_CE_PER_MV (-47819)
//#define VBAT_BAD_CE (-942635551)    // 2V, must be calibrated for each unit.
// Unfiltered VBAT
#define VBAT_CE_PER_MV (-23909)
#define VBAT_BAD_CE (-471317700)    // 2V, must be calibrated for each unit.

/***************************************************************************
 * History:
 * $Log: ce1_2184_1p0.h,v $
 * Revision 1.2  2010/06/30 23:36:03  Ray.Vandewalker
 * Ported to B01
 *
 * Revision 1.10  2010/05/25 00:42:11  Ray.Vandewalker
 * *** empty log message ***
 *
 * Revision 1.9  2009/06/24 00:34:57  tvander
 * *** empty log message ***
 *
 * Revision 1.8  2009/02/25 22:16:51  tvander
 * *** empty log message ***
 *
 * Revision 1.7  2009/01/05 18:39:23  tvander
 * *** empty log message ***
 *
 * Revision 1.6  2008/11/21 02:57:45  tvander
 * Fixed typo
 *
 * Revision 1.5  2008/11/05 01:08:17  tvander
 * Fixed constants.
 *
 * Revision 1.4  2008/10/30 02:14:25  tvander
 * CE constants.
 *
 * Revision 1.3  2008/10/24 18:56:47  tvander
 * *** empty log message ***
 *
 * Revision 1.2  2008/10/24 18:42:33  tvander
 * *** empty log message ***
 *
 * Revision 1.1  2008/10/22 22:42:40  tvander
 * *** empty log message ***
 *
 * 2008 Oct 22; First Version. 
 * Copyright (C) 2005 Maxim Integrated Products Inc. All Rights Reserved.  *
 * this program is fully protected by the United States copyright          *
 * laws and is the property of Maxim Integrated Products Inc.              *
 ***************************************************************************/

