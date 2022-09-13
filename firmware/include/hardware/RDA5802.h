#if defined(PLATFORM_RD5R)
#ifndef _OPENGD77_RDA5802_H_
#define _OPENGD77_RDA5802_H_

#include <stdint.h>
#include "interfaces/i2c.h"

#define RDA5802_ADDRESS	0x11
#define RDA5802_CHIP_ID_REG 0x00


bool initialize_rda5802();
void enable_rda5802();
void disable_rda5802();
void set_freq_rda5802(uint16_t freq);
void set_volume_rda5802(uint8_t volume);
status_t RDA5802ReadReg2byte(uint8_t reg, uint16_t *val);
status_t RDA5802WriteReg2byte(uint8_t reg, uint16_t val);
status_t RDA5802BatchWrite(uint16_t reg02, uint16_t reg03);

#endif
#endif
