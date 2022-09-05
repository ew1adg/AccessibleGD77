/*
 * RDA5802.c
 *
 *  Created on: Sep 3, 2022
 *      Author: ew1adg
 */
#if defined(PLATFORM_RD5R)

#include <stdio.h>
#include <stdbool.h>

#include "hardware/RDA5802.h"


bool initialize_rda5802()
{
	uint16_t value;

	// Verify chip ID
	RDA5802ReadReg2byte(RDA5802_CHIP_ID_REG, &value);
	if (value != 0x5804) {
		return false;
	}

	RDA5802WriteReg2byte(0x02, 0xD200);
	RDA5802WriteReg2byte(0x03, 0x3DD8);

	return true;
}

void enable_rda5802()
{

	// 0x10: 0x90 0x3  0x00 0x18
	RDA5802WriteReg2byte(0x02, 0x903);
	RDA5802WriteReg2byte(0x03, 0x0018);

	// 0x10: 0xD0 0x01 0x2A 0x18
	RDA5802WriteReg2byte(0x02, 0xD001);
	RDA5802WriteReg2byte(0x03, 0x2A18);
}

void set_freq_rda5802(uint16_t freq)
{
	uint16_t channel;
	uint16_t reg;

	channel = (freq - 870);
	reg = 0x0010;
	reg |= channel << 6;

	RDA5802WriteReg2byte(0x03, reg);
}

void set_volume_rda5802(uint8_t volume)
{
	uint16_t reg05;
	RDA5802ReadReg2byte(0x05, &reg05);

	reg05 = reg05 & 0xfff0;
	reg05 = reg05 | (volume & 0xf);

	RDA5802WriteReg2byte(0x05, reg05);
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

status_t RDA5802WriteReg2byte(uint8_t reg, uint16_t val)
{
    i2c_master_transfer_t masterXfer;
    status_t status;
    uint8_t buff[4];// Transfers are always 3 bytes but pad to 4 byte boundary

    if (isI2cInUse)
    {
#if defined(USING_EXTERNAL_DEBUGGER) && defined(DEBUG_I2C)
        SEGGER_RTT_printf(0, "Clash in write_I2C_reg_2byte (3) with %d\n",isI2cInUse);
#endif
        return kStatus_Success;
    }
    isI2cInUse = 3;

	buff[0] = reg;
	buff[1] = val >> 8;
	buff[2] = val & 0xff;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = RDA5802_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = buff;
    masterXfer.dataSize = 3;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(I2C0, &masterXfer);

    isI2cInUse = 0;
	return status;
}

#endif
