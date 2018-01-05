/***************************************************************************
 * This code and information is provided "as is" without warranty of any   *
 * kind, either expressed or implied, including but not limited to         *
 * implied warranties of merchantability and/or fitness for a particular   *
 * purpose.                                                                *
 *                                                                         *
 * Copyright (C) 2010 Maxim Integrated Products. Inc, All Rights Reserved. *
 * AUTHOR:  Ray.Vandewalker                                                *
 * HISTORY: See the end of file.                                           *
 * DESCRIPTION: Options for a 6543 3-phase meter with neutral current.     *
 ***************************************************************************/
#ifndef _OPTIONS_H
#define _OPTIONS_H

#include <stddef.h>
#include "stdint.h"         // Define standard integers.
#include "m6543.h"          // Define hardware and CE registers.
#include "irq.h"            // Enable/disable interrupts
#include "ce1_2184_1p0.h"   // CE Constants for: FIR1, FS=2184Hz, ~1 second

#define max(_v0_,_v1_) ((_v0_ > _v1_) ? _v0_ : _v1_)
#define min(_v0_,_v1_) ((_v0_ < _v1_) ? _v0_ : _v1_)

// Define CE code and data.
extern const short code NumCeCode;	// The count of words in "CeCode[]".
extern const unsigned char code CeCode[];
extern const short code NumCeData;	// The count of words in "CeData[]".
extern const unsigned char code CeData[];

/***************************************************************************
 * History:
 * $Log: options.h,v $
 * Revision 1.2  2010/06/30 23:36:04  Ray.Vandewalker
 * Ported to B01
 *
 * Revision 1.1  2009/06/15 18:25:35  dsang
 * *** empty log message ***
 *
 *
 * Copyright (C) 2009 Maxim Integrated Products Corp. All Rights Reserved.
 * this program is fully protected by the United States copyright
 * laws and is the property of Maxim Integrated Products.
 ***************************************************************************/
#endif // _OPTIONS_H
