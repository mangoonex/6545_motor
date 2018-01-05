/***************************************************************************
 * This code and information is provided "as is" without warranty of any   *
 * kind, either expressed or implied, including but not limited to the     *
 * implied warranties of merchantability and/or fitness for a particular   *
 * purpose.                                                                *
 *                                                                         *
 * Copyright (C) 2005 Maxim Integrated Products Inc. All Rights Reserved.    *
 ***************************************************************************/
//**************************************************************************
//    
// DESCRIPTION: 71M652x POWER METER- standard integer definitions
// This include file is close to the POSIX and C99 standard.
// It doesn't have 64-bit definitions, because KEIL C doesn't.
// It doesn't have signal definitions.
// 
//  AUTHOR:  RGV
//
//  HISTORY: See end of file
//
//**************************************************************************
//               
// File: stdint.h
//               
//**************************************************************************

#ifndef STDINT_H
#define STDINT_H
#define HAVE_STDINT_H

/* See your compiler manual for what this actually means.  Many
 * 8051 compilers, notably Keil, do not put local variables on
 * the stack! Instead, the linker manages a set of nesting overlays
 * in the type of memory selected by the memory model. So,
 * many compilers generate nonreentrant code by default. (I.e. unlike
 * other C compilers, functions can't just be called from interrupt code). */
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned long      uint32_t;
typedef signed char        int8_t;
typedef signed short       int16_t;
typedef signed long        int32_t;

/* These are defined to make it easy to port to other 8051 compilers. 
 * Compared to other CPUs, the 8051 has a zoo of memory types. */

/* internal data, lower 128 bytes, addressed directly
 * Fastest available memory (except registers), not battery-backed-up,
 * competes with stack, registers, bools, and idata.
 * For portability, see uint_fast8_t and its sisters which are POSIX standard.
 * Obviously there's not much.  Don't waste it.
 * */
typedef unsigned char data      uint8d_t;
typedef unsigned short data     uint16d_t;
typedef unsigned long data      uint32d_t;
typedef signed char data        int8d_t;
typedef signed short data       int16d_t;
typedef signed long data        int32d_t;

/* internal data, 16 bytes (20h-2Fh), addressed directly, and bit addressable!
 * Fastest available memory, not battery-backed-up on meter chips,  
 * competes with stack, registers, bools, data, and idata. 
 * The space is valuable for bool globals. Don't waste it. */
typedef unsigned char bdata      uint8b_t;
typedef unsigned short bdata     uint16b_t;
typedef unsigned long bdata      uint32b_t;
typedef signed char bdata        int8b_t;
typedef signed short bdata       int16b_t;
typedef signed long bdata        int32b_t;

/* booleans are not a normal part of stdint.h, but pretty portable.
 * In Keil these go in 20h-2Fh.  They're fast, not battery-backed-up on meters,
 * and compete with bdata. Keil functions return bools in the carry bit, 
 * which makes code that's amusingly fast and small. */
#undef bool
typedef bit bool;

#define TRUE ((bool)1)
#define FALSE ((bool)0)
#define ON   ((bool)1)
#define OFF  ((bool)0)

/* internal data, 256 bytes, upper 128 addressed indirectly
 * pretty fast, not battery-backed-up on meter chips, 
 * ugh slower than data. Competes with data for space. */
typedef unsigned char idata      uint8i_t;
typedef unsigned short idata     uint16i_t;
typedef unsigned long idata      uint32i_t;
typedef signed char idata        int8i_t;
typedef signed short idata       int16i_t;
typedef signed long idata        int32i_t;

/* external data, 256 bytes of 2K of CMOS RAM,
 * The upper byte of the XDATA address is supplied by the sfr 0xBf, ADRMSB,
 * on meter chips.  On other 8051s, P2 filled this purpose.
 * Accessed indirectly. kinda fast, battery backed-up on 651X meter chips.
 * This is a great place for nonvolatile globals like power registers 
 * and configuration data. */
typedef unsigned char pdata      uint8p_t;
typedef unsigned short pdata     uint16p_t;
typedef unsigned long pdata      uint32p_t;
typedef signed char pdata        int8p_t;
typedef signed short pdata       int16p_t;
typedef signed long pdata        int32p_t;

/* external data,  2k of CMOS RAM, accessed indirectly via a 16-bit register. 
 * slowest but largest memory, battery backed-up on meter chips
 * Use it for everything possible.  On Keil's large model, this 
 * is the default! */
typedef unsigned char xdata      uint8x_t;
typedef unsigned short xdata     uint16x_t;
typedef unsigned long xdata      uint32x_t;
typedef signed char xdata        int8x_t;
typedef signed short xdata       int16x_t;
typedef signed long xdata        int32x_t;

/* external read-only data, in code space, 
 * accessed indirectly via a 16-bit register. 
 * slowest but largest place, nonvolatile programmable flash on meter chips
 * Use it for constants and tables. */
typedef unsigned char code      uint8r_t;
typedef unsigned short code     uint16r_t;
typedef unsigned long code      uint32r_t;
typedef signed char code        int8r_t;
typedef signed short code       int16r_t;
typedef signed long code        int32r_t;

/* macros to define constants */
#define INT8_C(_v_) _v_
#define INT16_C(_v_) _v_
#define INT32_C(_v_) _v_L
#define INTMAX_C(_v_) _v_L
#define UINT8_C(_v_) _v_
#define UINT16_C(_v_) _v_
#define UINT32_C(_v_) _v_UL
#define UINTMAX_C(_v_) _v_UL

#define UINT8_MAX  255
#define UINT16_MAX 65535
#define UINT32_MAX 4294967295L

#define INT8_MIN  -128
#define INT16_MIN -32768
#define INT32_MIN -2147483648L

#define INT8_MAX  127
#define INT16_MAX 32767
#define INT32_MAX 2147483647L

/* Keil implements void pointers as 24-bit values, so
 * a 32-bit value is the smallest integer that can hold
 * a void pointer.  Typed pointers are always better.  */
typedef unsigned long      uintptr_t;
typedef signed long        intptr_t;

#define UINTPTR_MAX 65535
#define INTPTR_MIN -32768
#define INTPTR_MAX 32767
#define INTPTRDIFF_MIN -32768
#define INTPTRDIFF_MAX 32767
#define SIZE_MAX 65535

#define WCHAR_MIN -128
#define WCHAR_MAX 127
#define WINT_MIN -32768
#define WINT_MAX 32767


typedef unsigned long      uintmax_t;
typedef signed long        intmax_t;

#define UINTMAX_MAX 4294967295L
#define INTMAX_MIN -2147483648L
#define INTMAX_MAX 2147483647L


/* define a type that is at least the desired bit width.  See
 * your compiler manual for what this really means. */
typedef unsigned char      uint_least8_t;
typedef unsigned short     uint_least16_t;
typedef unsigned long      uint_least32_t;

typedef signed char        int_least8_t;
typedef signed short       int_least16_t;
typedef signed long        int_least32_t;

#define INT_LEAST8_MIN -128
#define INT_LEAST16_MIN -32768
#define INT_LEAST32_MIN -2147483648L

#define INT_LEAST8_MAX 127
#define INT_LEAST16_MAX 32767
#define INT_LEAST32_MAX 2147483647L

#define UINT_LEAST8_MAX 255
#define UINT_LEAST16_MAX 65535
#define UINT_LEAST32_MAX 4294967295L

/* define the fastest memory. */
/* See uint8d_t and its brothers, above for a description of the memory. */
typedef unsigned char data      uint_fast8_t;
typedef unsigned short data     uint_fast16_t;
typedef unsigned long  data     uint_fast32_t;

typedef signed char data        int_fast8_t;
typedef signed short data       int_fast16_t;
typedef signed long data        int_fast32_t;

#define INT_FAST8_MIN -128
#define INT_FAST16_MIN -32768
#define INT_FAST32_MIN -2147483648L

#define INT_FAST8_MAX 127
#define INT_FAST16_MAX 32767
#define INT_FAST32_MAX 2147483647L

#define UINT_FAST8_MAX 255
#define UINT_FAST16_MAX 65535
#define UINT_FAST32_MAX 4294967295L

/* These are not part of the standard, but they're very useful */
typedef union Uint8_16_t  { uint8_t c[2]; uint16_t i; } uint8_16_t;
typedef union Uint16_32_t  { uint16_t i[2]; uint32_t l; } uint16_32_t;
typedef union Uint8_16_32_t \
   { uint8_t c[4]; uint16_t i[2]; uint32_t l; } uint8_16_32_t;

typedef union Sint8_16_t  { int8_t c[2]; int16_t i; } int8_16_t;
typedef union Sint16_32_t  { int16_t i[2]; int32_t l; } int16_32_t;
typedef union Sint8_16_32_t \
   { int8_t c[4]; int16_t i[2]; int32_t l; } int8_16_32_t;

typedef struct Uint8_16_8_t { uint8_t f; uint16_t i; uint8_t c; } uint8_16_8_t;
typedef union  Uint8_16_8_32_t { uint8_16_8_t c; uint32_t l; } uint8_16_8_32_t;

#if 1 
// BIG_ENDIAN
#define HI 0
#define LO 1

#define HI_HI 0
#define HI_LO 1
#define LO_HI 2
#define LO_LO 3

#else
// LITTLE_ENDIAN
#define HI 1
#define LO 0

#define HI_HI 3
#define HI_LO 2
#define LO_HI 1
#define LO_LO 0
#endif

#define NO_BITS 0x00
#define BIT0    0x01
#define BIT1    0x02
#define BIT2    0x04
#define BIT3    0x08
#define BIT4    0x10
#define BIT5    0x20
#define BIT6    0x40
#define BIT7    0x80
#define BIT8    0x0100
#define BIT9    0x0200
#define BIT10   0x0400
#define BIT11   0x0800
#define BIT12   0x1000
#define BIT13   0x2000
#define BIT14   0x4000
#define BIT15   0x8000
#define BIT16   0x010000
#define BIT17   0x020000
#define BIT18   0x040000
#define BIT19   0x080000
#define BIT20   0x100000
#define BIT21   0x200000
#define BIT22   0x400000
#define BIT23   0x800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000

/***************************************************************************
 * History
 * $Log: stdint.h,v $
 * Revision 1.2  2010/06/30 23:36:04  Ray.Vandewalker
 * Ported to B01
 *
 * Revision 1.24  2010/06/16 03:48:39  Ray.Vandewalker
 * linted
 *
 * Revision 1.23  2010/05/27 17:00:01  Ray.Vandewalker
 * 5.2h
 * Made equation 1 utilize A03CE code.
 * Fixed build to avoid C51 8.17's defect
 *
 * Revision 1.22  2010/05/24 22:27:15  Ray.Vandewalker
 * 5.2g
 * First cut at 2-pole shunt filtering.
 * Changed name to Maxim.
 * version broken out to a separate file.
 *
 * Revision 1.21  2010/05/23 03:16:16  Ray.Vandewalker
 * Clean build of 6541B01 5.2g, with 2-pole shunt prediction
 *
 * Revision 1.20  2010/04/30 23:40:53  tvander
 * Rolled rev to 5.2e
 *
 * Revision 1.19  2010/03/26 21:10:59  tvander
 * 5.2D
 *
 * Revision 1.18  2010/03/02 17:09:24  tvander
 * 5.2c
 *
 * Revision 1.17  2010/03/02 00:30:44  tvander
 * 5.2b
 *
 * Revision 1.16  2010/02/19 21:25:34  tvander
 * Rolled revision to 5.2a
 *
 * Revision 1.15  2010/02/09 00:38:58  tvander
 * Rolled version to 5.1g
 *
 * Revision 1.14  2010/01/18 01:05:44  tvander
 * Rolled release to 5.1f
 *
 * Revision 1.13  2009/12/15 00:42:18  tvander
 * 5.1e changed creep:Creep for energy uses volts.
 *
 * Revision 1.12  2009/11/21 01:17:45  tvander
 * 5.1d  6543:505.1 Imax; 25uOhm shunts with 14x remotes
 *
 * Revision 1.11  2009/11/12 17:16:11  tvander
 * Rolled revision. 5.1c
 *
 * Revision 1.10  2009/11/05 19:16:33  tvander
 * 5.1b
 *
 * Revision 1.9  2009/09/17 21:21:21  tvander
 * Added power conservation logic.
 * Fixed bad Wh issue in Battery mode demo.
 *
 * Revision 1.8  2009/08/06 17:37:13  dsang
 * *** empty log message ***
 *
 * Revision 1.7  2009/07/23 00:11:22  tvander
 * Rolled version to e
 *
 * Revision 1.6  2009/07/17 00:29:57  dsang
 * 6541 5.0d, compact size code to fit 32k
 *
 * Revision 1.5  2009/07/02 17:50:23  dsang
 * 5.0c with 6541 LCD configuration
 *
 * Revision 1.4  2009/06/22 21:19:30  tvander
 * 5.0b
 *
 * Revision 1.3  2008/09/29 16:58:20  tvander
 * Software version ID.
 *
 * Revision 1.2  2008/09/12 18:32:15  dsang
 * Remove ptrdiff
 *
 * Revision 1.1  2008/09/02 22:15:10  tvander
 * First commit.
 *
 * Revision 1.22  2006/09/09 01:16:00  gmikef
 * *** empty log message ***
 *
 * Revision 1.20  2006/01/10 04:13:39  gmikef
 * Added PDATA support for CE Outputs.
 *
 * Revision 1.19  2006/01/04 04:47:57  gmikef
 * Switched RMS and VA calculations to use floating point. (and Calibration).
 *
 * Revision 1.17  2005/11/09 02:21:17  tvander
 * Added code to display watt hours from brownout mode.
 * Added code to clear EEPROM (lapie command "EEE")
 *
 * Revision 1.16  2005/09/22 23:45:30  tvander
 * Clean build all models and unit tests, updated copyright to be fore Teridian
 *
 * Revision 1.15  2005/09/08 00:31:43  tvander
 * Fixed: Lost data due to overwrite of Ithrshld, xfer busy never runs,
 * unavailable second phase, cosmetic issues with status.  Updated
 * date.
 *
 * Revision 1.14  2005/08/23 02:12:56  gmikef
 * *** empty log message ***
 *
 * Revision 1.13  2005/08/20 01:32:49  gmikef
 * *** empty log message ***
 *
 * Revision 1.12  2005/04/30 02:20:54  gmikef
 * *** empty log message ***
 *
 * Revision 1.11  2005/04/28 19:12:28  tvander
 * Comments only!  Restored history comments.
 *
 * Revision 1.10  2005/04/27 23:49:26  gmikef
 * Some MATH rountines now use 'idata'.
 * Added MATH_FAST flag to 'options.h".
 * Changed "6521B.Uv2" to max optimization.
 *
 * Revision 1.9  2005/04/21 02:09:27  gmikef
 * *** empty log message ***
 *
 * Revision 1.8  2005/04/09 02:28:59  gmikef
 * *** empty log message ***
 *
 * Revision 1.7  2005/03/24 21:44:12  gmikef
 * *** empty log message ***
 *
 * Revision 1.6  2005/03/24 01:39:00  tvander
 * First successful compile of serial unit test
 *
 * Revision 1.5  2005/03/23 19:18:12  tvander
 * *** empty log message ***
 *
 * Revision 1.4  2005/03/12 00:28:08  tvander
 * Added code space as int8r_t (for example)
 *
 * Revision 1.3  2005/03/12 00:06:42  tvander
 * Added full 8051 types
 *
 * Revision 1.2  2005/03/11 22:57:03  tvander
 * Added bool, and 8/16/32 data structures
 *
 * Revision 1.1  2005/03/11 22:17:23  tvander
 * Structure
 *
 * Copyright (C) 2005 Maxim Integrated Products Inc. All Rights Reserved.
 * this program is fully protected by the United States copyright 
 * laws and is the property of Maxim Integrated Products Inc.
 ***************************************************************************/
#endif /* STDINT_H */
