#if defined(PLATFORM_RD5R)
#ifndef _OPENGD77_RDA5802_H_
#define _OPENGD77_RDA5802_H_

#include <stdint.h>
#include "interfaces/i2c.h"

#define RDA5802_ADDRESS	0x11
#define RDA5802_CHIP_ID_REG 0x00


bool initialize_rda5802();
status_t RDA5802ReadReg2byte(uint8_t reg, uint8_t *val1, uint8_t *val2);
status_t RDA5802WriteReg2byte(uint8_t reg, uint8_t val1, uint8_t val2);

#endif
#endif
