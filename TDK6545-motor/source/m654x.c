/***************************************************************************
 * This code and information is provided "as is" without warranty of any 
 * kind, either expressed or implied, including but not limited to
 * implied warranties of merchantability and/or fitness for a particular
 * purpose.
 *
 * Copyright (C) 2008 Maxim Integrated Products Inc. All Rights Reserved.
 * AUTHOR:  MTF, RGV
 * HISTORY: See end of file
 * DESCRIPTION: 71M654x POWER METER - Registers.
 ***************************************************************************/

#include "options.h"

volatile uint8x_t MUX[6]        _at_ (IO_BASE + 0x100);
volatile uint8x_t CE6           _at_ (IO_BASE + 0x106);
volatile uint8x_t CE5           _at_ (IO_BASE + 0x107);
volatile uint8x_t CE4           _at_ (IO_BASE + 0x108);
volatile uint8x_t CE3           _at_ (IO_BASE + 0x109);
volatile uint8x_t CE2           _at_ (IO_BASE + 0x10A);
volatile uint8x_t CE1           _at_ (IO_BASE + 0x10B);
volatile uint8x_t CE0           _at_ (IO_BASE + 0x10C);
volatile uint16x_t RTM0         _at_ (IO_BASE + 0x10D);
volatile uint8x_t RTM[3]        _at_ (IO_BASE + 0x10F);

volatile uint8x_t CKGN          _at_ (IO_BASE + 0x200);

volatile uint16x_t CHIPID       _at_ (IO_BASE + 0x300);
volatile int8x_t TRIMT          _at_ (IO_BASE + 0x309);

volatile uint8x_t LCD0          _at_ (IO_BASE + 0x400);
volatile uint8x_t LCD1          _at_ (IO_BASE + 0x401);
volatile uint8x_t LCD2          _at_ (IO_BASE + 0x402);
volatile uint8x_t LCD_MAP[7]    _at_ (IO_BASE + 0x405);
volatile uint8x_t LCD4          _at_ (IO_BASE + 0x40C);
volatile uint8x_t LCD_DAC       _at_ (IO_BASE + 0x40D);
//volatile uint8x_t SEGDIO[55]    _at_ (IO_BASE + 0x410); 6545 IO 口并没有这么多，另外在下面定义
volatile uint8x_t DIO_R[6]      _at_ (IO_BASE + 0x450);
volatile uint8x_t DIO0          _at_ (IO_BASE + 0x456);
volatile uint8x_t DIO1          _at_ (IO_BASE + 0x457);
volatile uint8x_t DIO2          _at_ (IO_BASE + 0x458);

volatile uint8x_t SPARENV       _at_ (IO_BASE + 0x500);   
volatile uint8x_t FOVRD         _at_ (IO_BASE + 0x501);
volatile uint8x_t TMUX          _at_ (IO_BASE + 0x502);
volatile uint8x_t TMUX2         _at_ (IO_BASE + 0x503);
volatile uint8x_t RTCA_ADJ      _at_ (IO_BASE + 0x504);

volatile uint16x_t REMOTE       _at_ (IO_BASE + 0x602);   

volatile uint8x_t INT1_E        _at_ (IO_BASE + 0x700);   
volatile uint8x_t INT2_E        _at_ (IO_BASE + 0x701);   
volatile uint8x_t SECURE        _at_ (IO_BASE + 0x702);
volatile uint8x_t ANALOG0       _at_ (IO_BASE + 0x704);
volatile uint8x_t VERSION       _at_ (IO_BASE + 0x706);
volatile uint8x_t INTBITS       _at_ (IO_BASE + 0x707);
volatile uint8x_t SPI0          _at_ (IO_BASE + 0x708);
volatile uint8x_t RCE0          _at_ (IO_BASE + 0x709);
volatile uint8x_t RTMUX         _at_ (IO_BASE + 0x70A);
volatile uint8x_t INFO_PG       _at_ (IO_BASE + 0x70B);
volatile uint8x_t DIO3          _at_ (IO_BASE + 0x70C);

volatile uint8x_t WAKE          _at_ (IO_BASE + 0x880);
volatile int16x_t STEMP         _at_ (IO_BASE + 0x881);
volatile uint8x_t BSENSE        _at_ (IO_BASE + 0x885);
volatile uint8x_t LKPADDR       _at_ (IO_BASE + 0x887);
volatile uint8x_t LKPDATA       _at_ (IO_BASE + 0x888);
volatile uint8x_t LKPCTRL       _at_ (IO_BASE + 0x889);
volatile uint8x_t RTC0          _at_ (IO_BASE + 0x890);
volatile uint8x_t RTC2          _at_ (IO_BASE + 0x892); // SUBSEC
volatile uint8x_t RTC[7]        _at_ (IO_BASE + 0x893);   
volatile uint8x_t RTC_PNQREG[3] _at_ (IO_BASE + 0x89B);
volatile uint8x_t RTC_TMIN      _at_ (IO_BASE + 0x89E);
volatile uint8x_t RTC_THR       _at_ (IO_BASE + 0x89F);
volatile uint8x_t TEMP          _at_ (IO_BASE + 0x8A0);
volatile uint8x_t WF1           _at_ (IO_BASE + 0x8B0);
volatile uint8x_t WF2           _at_ (IO_BASE + 0x8B1);
volatile uint8x_t MISC          _at_ (IO_BASE + 0x8B2);
volatile uint8x_t WAKE_E        _at_ (IO_BASE + 0x8B3);
volatile uint8x_t WDRST         _at_ (IO_BASE + 0x8B4);

//YANGKE in order to access dio16~dio31 20180103
volatile uint8x_t DIO19         _at_ (IO_BASE + 0x423);
volatile uint8x_t DIO20         _at_ (IO_BASE + 0x424);
volatile uint8x_t DIO21         _at_ (IO_BASE + 0x425);
volatile uint8x_t DIO22         _at_ (IO_BASE + 0x426);
volatile uint8x_t DIO23         _at_ (IO_BASE + 0x427);
volatile uint8x_t DIO24         _at_ (IO_BASE + 0x428);
volatile uint8x_t DIO25         _at_ (IO_BASE + 0x429);
volatile uint8x_t DIO28         _at_ (IO_BASE + 0x42C);
volatile uint8x_t DIO29         _at_ (IO_BASE + 0x42D);

// CE calibration registers.
volatile int32x_t iadc67      _at_ (CE_DATA_BASE + (0x00 << 2));
volatile int32x_t cal_i0     _at_ (CE_DATA_BASE + (0x10 << 2));
volatile int32x_t cal_v0     _at_ (CE_DATA_BASE + (0x11 << 2));
volatile int32x_t phadj_0    _at_ (CE_DATA_BASE + (0x12 << 2));

volatile int32x_t cal_i1     _at_ (CE_DATA_BASE + (0x13 << 2));
volatile int32x_t cal_v1     _at_ (CE_DATA_BASE + (0x14 << 2));
volatile int32x_t phadj_1    _at_ (CE_DATA_BASE + (0x15 << 2));

volatile int32x_t cal_i2     _at_ (CE_DATA_BASE + (0x16 << 2));
volatile int32x_t cal_v2     _at_ (CE_DATA_BASE + (0x17 << 2));
volatile int32x_t phadj_2    _at_ (CE_DATA_BASE + (0x18 << 2));

volatile int32x_t cal_i3     _at_ (CE_DATA_BASE + (0x19 << 2));
volatile int32x_t degscale   _at_ (CE_DATA_BASE + (0x1A << 2));
volatile int32x_t ce_ppmc1   _at_ (CE_DATA_BASE + (0x1B << 2));
volatile int32x_t ce_ppmc2   _at_ (CE_DATA_BASE + (0x1C << 2));

volatile int32x_t ceconfig   _at_ (CE_DATA_BASE + (0x20 << 2));
volatile int32x_t wrate      _at_ (CE_DATA_BASE + (0x21 << 2));
volatile int32x_t kvar       _at_ (CE_DATA_BASE + (0x22 << 2));
volatile int32x_t sumpre     _at_ (CE_DATA_BASE + (0x23 << 2));
volatile int32x_t sag_thr    _at_ (CE_DATA_BASE + (0x24 << 2));

volatile int32x_t quant_va   _at_ (CE_DATA_BASE + (0x25 << 2));
volatile int32x_t quant_ia   _at_ (CE_DATA_BASE + (0x26 << 2));
volatile int32x_t quant_a    _at_ (CE_DATA_BASE + (0x27 << 2));
volatile int32x_t quant_vara _at_ (CE_DATA_BASE + (0x28 << 2));

volatile int32x_t quant_vb   _at_ (CE_DATA_BASE + (0x29 << 2));
volatile int32x_t quant_ib   _at_ (CE_DATA_BASE + (0x2A << 2));
volatile int32x_t quant_b    _at_ (CE_DATA_BASE + (0x2B << 2));
volatile int32x_t quant_varb _at_ (CE_DATA_BASE + (0x2C << 2));

volatile int32x_t quant_vc   _at_ (CE_DATA_BASE + (0x2D << 2));
volatile int32x_t quant_ic   _at_ (CE_DATA_BASE + (0x2E << 2));
volatile int32x_t quant_c    _at_ (CE_DATA_BASE + (0x2F << 2));
volatile int32x_t quant_varc _at_ (CE_DATA_BASE + (0x30 << 2));

volatile int32x_t quant_id   _at_ (CE_DATA_BASE + (0x31 << 2));

// (the CE's file name is at 0x38 in the CE data array)

volatile int32x_t gain_adj[5] _at_ (CE_DATA_BASE + (0x40 << 2));

volatile int32x_t apulsew      _at_ (CE_DATA_BASE + (0x45 << 2));
volatile int32x_t wpulse_ctr   _at_ (CE_DATA_BASE + (0x46 << 2));
volatile int32x_t wpulse_frac  _at_ (CE_DATA_BASE + (0x47 << 2));
volatile int32x_t wsum_accum   _at_ (CE_DATA_BASE + (0x48 << 2));

volatile int32x_t apulser      _at_ (CE_DATA_BASE + (0x49 << 2));
volatile int32x_t rpulse_ctr   _at_ (CE_DATA_BASE + (0x4A << 2));
volatile int32x_t rpulse_frac  _at_ (CE_DATA_BASE + (0x4B << 2));
volatile int32x_t vsum_accum   _at_ (CE_DATA_BASE + (0x4C << 2));

// (The CE word at 4F is unused, and can be used for a checksum.)

// Outputs from CE.
volatile int32_t pdata cestatus      _at_ (CE_DATA_BASE + (0x80 << 2));   
volatile int32_t pdata temp_ce       _at_ (CE_DATA_BASE + (0x81 << 2));   
volatile int32_t pdata freq          _at_ (CE_DATA_BASE + (0x82 << 2));   
volatile int32_t pdata mainedge      _at_ (CE_DATA_BASE + (0x83 << 2));   

volatile int32_t pdata wsum          _at_ (CE_DATA_BASE + (0x84 << 2));   
volatile int32_t pdata w0sum         _at_ (CE_DATA_BASE + (0x85 << 2));   
volatile int32_t pdata w1sum         _at_ (CE_DATA_BASE + (0x86 << 2));   
volatile int32_t pdata w2sum         _at_ (CE_DATA_BASE + (0x87 << 2));   

volatile int32_t pdata varsum        _at_ (CE_DATA_BASE + (0x88 << 2));   
volatile int32_t pdata var0sum       _at_ (CE_DATA_BASE + (0x89 << 2));   
volatile int32_t pdata var1sum       _at_ (CE_DATA_BASE + (0x8A << 2));
volatile int32_t pdata var2sum       _at_ (CE_DATA_BASE + (0x8B << 2));

volatile int32_t pdata i0sqsum       _at_ (CE_DATA_BASE + (0x8C << 2));
volatile int32_t pdata i1sqsum       _at_ (CE_DATA_BASE + (0x8D << 2));   
volatile int32_t pdata i2sqsum       _at_ (CE_DATA_BASE + (0x8E << 2));   
volatile int32_t pdata idsqsum       _at_ (CE_DATA_BASE + (0x8F << 2));   

volatile int32_t pdata v0sqsum       _at_ (CE_DATA_BASE + (0x90 << 2));   
volatile int32_t pdata v1sqsum       _at_ (CE_DATA_BASE + (0x91 << 2));   
volatile int32_t pdata v2sqsum       _at_ (CE_DATA_BASE + (0x92 << 2));   
volatile int32_t pdata vdsqsum       _at_ (CE_DATA_BASE + (0x93 << 2));   

volatile int32_t pdata ph_atob       _at_ (CE_DATA_BASE + (0x94 << 2)); 
volatile int32_t pdata ph_atoc       _at_ (CE_DATA_BASE + (0x95 << 2));

volatile int32_t pdata i0sqres       _at_ (CE_DATA_BASE + (0x96 << 2));   
volatile int32_t pdata i1sqres       _at_ (CE_DATA_BASE + (0x97 << 2));   
volatile int32_t pdata i2sqres       _at_ (CE_DATA_BASE + (0x98 << 2));   
volatile int32_t pdata idsqres       _at_ (CE_DATA_BASE + (0x99 << 2));   

/***************************************************************************
 * History:
 * $Log: m654x.c,v $
 * Revision 1.3  2010/06/30 23:36:04  Ray.Vandewalker
 * Ported to B01
 *
 * Revision 1.13  2010/06/26 03:56:44  Ray.Vandewalker
 * Added commit" to logging.  Added shorter nap for me.
 *
 * Revision 1.12  2010/05/24 22:26:18  Ray.Vandewalker
 * 5.2g
 * First cut at 2-pole shunt filtering.
 * Changed name to Maxim.
 * version broken out to a separate file.
 *
 * Revision 1.11  2010/04/28 18:38:10  tvander
 * Pulse width is unused by both CE and MPU code.
 *
 * Revision 1.10  2010/03/26 21:08:58  tvander
 * First cut that works with B01
 *
 * Revision 1.9  2009/06/16 05:23:24  tvander
 * *** empty log message ***
 *
 * Revision 1.8  2009/05/29 18:18:15  tvander
 * Integrated 6000
 *
 * Revision 1.7  2009/05/07 20:24:33  tvander
 * Added fractional pulses.
 *
 * Revision 1.6  2009/04/21 00:39:39  tvander
 * Modified to permit actual use of nonvolatile memory.
 *
 * Revision 1.5  2009/04/10 20:55:10  tvander
 * Corrected address of RTCA_ADJ
 *
 * Revision 1.4  2009/04/09 22:14:21  tvander
 * Added RTC_ADJ
 *
 * Revision 1.3  2009/03/12 01:32:02  tvander
 * Fixed time and date.
 *
 * Revision 1.2  2009/03/07 03:09:17  tvander
 * *** empty log message ***
 *
 * Revision 1.1  2009/03/02 10:13:06  tvander
 * *** empty log message ***
 *
 * Revision 1.9  2009/01/08 01:19:23  tvander
 * Corrected to revised CE code.
 *
 * Revision 1.8  2008/11/20 00:37:13  tvander
 * Changed the name of vah_ex to vah_im, which is more accurate.
 *
 * Revision 1.7  2008/11/14 20:00:56  tvander
 * Latest version of CE interface.
 *
 * Revision 1.6  2008/11/05 01:26:58  tvander
 * Modified to support CE
 *
 * Revision 1.5  2008/10/20 22:27:34  tvander
 * Added CE registers.
 *
 * Revision 1.4  2008/10/07 22:40:49  tvander
 * *** empty log message ***
 *
 * Revision 1.3  2008/09/29 16:45:42  tvander
 * Fixed SUBSEC
 *
 * Revision 1.2  2008/09/13 00:44:32  tvander
 * Fixed I/O RAM variables.
 *
 * Revision 1.1  2008/09/02 22:15:08  tvander
 * First commit.
 *
 * Revision 1.9  2006/09/09 01:15:51  gmikef
 * *** empty log message ***
 *
 * Revision 1.7  2005/09/22 23:45:28  tvander
 * Clean build all models and unit tests, updated copyright to be fore Teridian
 *
 * Revision 1.6  2005/04/30 02:20:25  gmikef
 * *** empty log message ***
 *
 * Revision 1.5  2005/04/28 19:12:28  tvander
 * Comments only!  Restored history comments.
 *
 * Revision 1.4  2005/04/27 23:48:52  gmikef
 * Some MATH rountines now use 'idata'.
 * Added MATH_FAST flag to 'options.h".
 * Changed "6521B.Uv2" to max optimization.
 *
 * Revision 1.3  2005/04/21 02:08:55  gmikef
 * *** empty log message ***
 *
 * Revision 1.2  2005/04/06 18:10:12  gmikef
 * *** empty log message ***
 *
 * Revision 1.1  2005/03/24 21:45:47  gmikef
 * *** empty log message ***
 *
 * Copyright (C) 2005 Maxim Integrated Products Inc. All Rights Reserved.
 * this program is fully protected by the United States copyright
 * laws and is the property of Maxim Integrated Products Inc.
 ***************************************************************************/

