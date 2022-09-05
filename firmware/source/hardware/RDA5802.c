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
	uint8_t hi_byte, lo_byte;

	// Verify chip ID
	RDA5802ReadReg2byte(RDA5802_CHIP_ID_REG, &hi_byte, &lo_byte);
	if (hi_byte != 0x58 || lo_byte != 0x04) {
		return false;
	}

	return true;
}

status_t RDA5802ReadReg2byte(uint8_t reg, uint8_t *val1, uint8_t *val2)
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

    *val1 = buff[0];
    *val2 = buff[1];

    isI2cInUse = 0;
	return status;
}

#endif
