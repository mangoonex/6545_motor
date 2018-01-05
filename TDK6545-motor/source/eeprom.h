#ifndef EEPROM_H
#define EEPROM_H
#include "options.h"            // Define the IC, CE code and options.
#include <string.h>
#include <math.h>
#include "CONFIG.H"
#include "energy.h"


extern bool OSEEPROMWrite (uint16_t MemAddr,uint8_t  DevSlaveAddr,uint8_t  _data_point[],uint8_t N);
extern	bool OSEEPROMRead (uint16_t MemAddr,uint8_t  DevSlaveAddr,uint8_t  _data_point[],uint8_t N);
extern void memcpy_cex_1 (int32_t *pDst, int32_t *pSrc);
extern bool Load_Calib_Data(uint16_t paraaddr);

#endif
