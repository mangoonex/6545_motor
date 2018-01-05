/***************************************************************************
 * This code and information is provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a particular
 * purpose.
 *
 * Copyright (C) 2005, 2008 Maxim Integrated Products, Inc. All Rights Reserved.    *
 *  DESCRIPTION: 71M65xx POWER METER - implement a dual-priority monitor.
 *  This implements two priorities: preemptible, and non-preemptible.
 *  Since there's only two, priority inversion is impossible.
 *  Since the code runs to completion, hangs are impossible
 *  Since there's only one locking flag, and thus one order
 *  to get it, deadlocks are impossible.
 *  Therefore, it's the guts of a very reliable exokernel.
 *  Critical regions have to be kept brief, though! 100 microseconds
 *  is a recommended maximum.
 *  
 *  We use this scheme instead of the more typical scheme that
 *  just disables a single device's interrupt, because the single
 *  device scheme can cause priority inversion.  E.g. a serial
 *  interrupt may be disabled in the main loop.  Another device's (a timer...)
 *  interrupt preempts the CPU because its interrupt is not locked.
 *  The preempting interrupt may be lower priority... and cause
 *  the serial interrupt to be delayed so much that data is lost.
 * 
 *  AUTHOR:  RGV
 *
 *  HISTORY: See end of file.
 ***************************************************************************/

// Place this macro at the start of a routine.
#define IRQ_DEFINES  uint8_t data ea = EA

// Place this macro before a critical region.
#define IRQ_DISABLE() EA = FALSE

// Place this macro after a critical region.
#define IRQ_ENABLE() EA = ea

/***************************************************************************
 * $Log: irq.h,v $
 * Revision 1.2  2010/06/30 23:36:03  Ray.Vandewalker
 * Ported to B01
 *
 * Revision 1.3  2010/05/24 23:24:00  Ray.Vandewalker
 * *** empty log message ***
 *
 * Revision 1.2  2008/11/25 03:22:52  tvander
 * Speeded-up, hopefully, the time interrupts are disabled.
 *
 * Revision 1.1  2008/09/19 23:39:07  tvander
 * Added interrupt control macro.
 *
 *
 * Copyright (C) 2005, 2008 Maxim Integrated Products, Inc. All Rights Reserved.
 * this program is fully protected by the United States copyright
 * laws and is the property of Maxim Integrated Products, Inc.
 ***************************************************************************/
