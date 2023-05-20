/*
 * RDA5802.c
 *
 *  Created on: Sep 3, 2022
 *      Author: ew1adg
 */
#if defined(PLATFORM_RD5R)

#include <stdio.h>
#include <stdbool.h>

#include "interfaces/gpio.h"
#include "functions/sound.h"
#include "hardware/RDA5802.h"

// Storage for register values
static uint8_t rda5802_regs[8] = {0};


bool initialize_rda5802()
{
	uint16_t value;

	// Verify chip ID
	RDA5802ReadReg2byte(RDA5802_CHIP_ID_REG, &value);
	if (value >> 8 != 0x58) {
		return false;
	}
	disable_rda5802();

	return true;
}

void enable_rda5802()
{
	GPIO_PinWrite(GPIO_FM_preamp, Pin_FM_preamp, 1);
	enableAudioAmp(AUDIO_AMP_MODE_FM);

	// Set default values
	rda5802_regs[0] = RDA5802_DHIZ | RDA5802_DMUTE | RDA5802_MONO;
	rda5802_regs[1] = RDA5802_CLK_MODE_32768 | RDA5802_NEW_METHOD | RDA5802_ENABLE;
	rda5802_regs[2] = 0;
	rda5802_regs[3] = RDA5802_BAND_US_EUROPE | RDA5802_STEP_50;
	rda5802_regs[4] = 0;
	rda5802_regs[5] = 0;
	rda5802_regs[6] = 0b1000 & RDA5802_SEEKTH;
	rda5802_regs[7] = RDA5802_LNA_PORT_SEL_LNAP | RDA5802_VOLUME;

	RDA5802BatchWrite(rda5802_regs);
}

void disable_rda5802()
{
	GPIO_PinWrite(GPIO_FM_preamp, Pin_FM_preamp, 0);
	disableAudioAmp(AUDIO_AMP_MODE_FM);

	// Set default values
	rda5802_regs[0] = 0;
	rda5802_regs[1] = 0;
	rda5802_regs[2] = 0;
	rda5802_regs[3] = 0;
	rda5802_regs[4] = 0;
	rda5802_regs[5] = 0;
	rda5802_regs[6] = 0;
	rda5802_regs[7] = 0;

	RDA5802BatchWrite(rda5802_regs);
}

void set_freq_rda5802(uint16_t freq)
{
	uint16_t chan;

    chan = (freq - 8700) / 5; // 50 Khz

    rda5802_regs[2] = chan >> 2;
    rda5802_regs[3] = ((chan & 0x03) << 6) | RDA5802_TUNE | RDA5802_BAND_US_EUROPE | RDA5802_STEP_50;

    RDA5802BatchWrite(rda5802_regs);
}

/* Get actual frequency value. Multiply by 10 to get value in KHz */
uint16_t get_freq_rda5802()
{
	uint16_t val;

	RDA5802ReadReg2byte(0x0A, &val);
	return 8700 + (val & 0x3ff) * 5; // 50 KHz step
}

/* Read RSSI value */
uint16_t get_rssi_rda5802()
{
	uint16_t val;

	RDA5802ReadReg2byte(0x0B, &val);
	return val >> 10;
}

/* The seek/tune complete flag is set when the seek or tune operation completes. */
bool get_stc_flag()
{
	uint16_t val;

	RDA5802ReadReg2byte(0x0A, &val);
	return (val >> 14) % 2;
}

/* The seek fail flag is set when the seek operation fails */
bool get_sf_flag()
{
	uint16_t val;

	RDA5802ReadReg2byte(0x0A, &val);
	return (val >> 13) % 2;
}

status_t RDA5802ReadReg2byte(uint8_t reg, uint16_t *val)
{
    i2c_master_transfer_t masterXfer;
    status_t status;
    uint8_t buff[4];// Transfers are always 3 bytes but pad to 4 byte boundary

    if (isI2cInUse)
    {
#if defined(USING_EXTERNAL_DEBUGGER) && defined(DEBUG_I2C)
    	SEGGER_RTT_printf(0, "Clash in read_I2C_reg_2byte (4) with %d\n",isI2cInUse);
#endif
    	return kStatus_Success;
    }
    isI2cInUse = 4;

	buff[0] = reg;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = RDA5802_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = buff;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(I2C0, &masterXfer);
    if (status != kStatus_Success)
    {
    	isI2cInUse = 0;
    	return status;
    }

    masterXfer.slaveAddress = RDA5802_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = buff;
    masterXfer.dataSize = 2;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(I2C0, &masterXfer);
    if (status != kStatus_Success)
    {
    	isI2cInUse = 0;
    	return status;
    }

    *val = buff[0] << 8;
    *val |= buff[1];

    isI2cInUse = 0;
	return status;
}

status_t RDA5802BatchWrite(uint8_t *registers)
{
    i2c_master_transfer_t masterXfer;
    status_t status;

    if (isI2cInUse)
    {
#if defined(USING_EXTERNAL_DEBUGGER) && defined(DEBUG_I2C)
        SEGGER_RTT_printf(0, "Clash in write_I2C_reg_2byte (3) with %d\n",isI2cInUse);
#endif
        return kStatus_Success;
    }
    isI2cInUse = 3;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = RDA5802_BATCH_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = registers;
    masterXfer.dataSize = 8;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(I2C0, &masterXfer);

    isI2cInUse = 0;
	return status;
}

status_t RDA5802BatchRead(uint8_t *registers)
{
	i2c_master_transfer_t masterXfer;
	status_t status;

	masterXfer.slaveAddress = RDA5802_BATCH_ADDRESS;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = registers;
	masterXfer.dataSize = 4;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	status = I2C_MasterTransferBlocking(I2C0, &masterXfer);
	if (status != kStatus_Success)
	{
		isI2cInUse = 0;
	    return status;
	}

    isI2cInUse = 0;
	return status;
}

#endif
