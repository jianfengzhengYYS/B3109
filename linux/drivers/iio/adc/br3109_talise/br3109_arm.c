/**
 * \file br3109_arm.c
 * \brief Contains functions to support interfacing with the BR3109 internal
 *          ARM processor
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#include "br3109_arm.h"
#include "br3109_radioctrl.h"
#include "br3109_reg_addr_macros.h"
#include "br3109_arm_macros.h"
#include "br3109_hal.h"
#include "br3109_user.h"
#include "br3109_error.h"
#include "br3109_cals.h"
#include "br3109.h"

uint32_t BR3109_initArm(br3109Device_t *device, br3109Init_t *init)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
    uint32_t regdat = 0;
    uint32_t armClkRate_kHz = 0;
    uint32_t hsDigClkDiv4or5_Hz = 0;
	uint8_t refarm_div = 0; /** 0:div1  1:div2  2:div4  3:div6  4;div8  5:div10 6,7:div12*/
	uint8_t armClk_div =  0;/** 0:div4  1:div6  2:div8  3:div10  4;div12  5:div14 6；div16 7:div18*/
#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_initArm()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	if (init->clocks.deviceClock_kHz > 10000 && init->clocks.deviceClock_kHz <= 200000) {
        refarm_div = 0; 
		regdat = init->clocks.deviceClock_kHz*1000;
		retVal = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_REFCLK_FREQ,  &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	} else {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INITARM_INV_ARMCLK_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	hsDigClkDiv4or5_Hz = device->devStateInfo.clocks.hsDigClkDiv4or5_Hz;
	/* dividing from Hz to kHz */
	armClkRate_kHz =  hsDigClkDiv4or5_Hz / 1000;
	/* the SPI read register and write register clocks must be equal or less than 250MHz */
	/* the ARM clock should not exceed 250 Mhz */
	if (armClkRate_kHz <= 1000000) {
		armClk_div = 0;
	} else if (armClkRate_kHz <= 1500000) {
		armClk_div = 0x01;
	} else if (armClkRate_kHz <= 2000000) {
		armClk_div = 0x02;
	} else if (armClkRate_kHz <= 3000000) {
		armClk_div = 0x04;
	} else if (armClkRate_kHz <= 3500000) {
		armClk_div = 0x05;
	} else if (armClkRate_kHz <= 4000000) {
		armClk_div = 0x06;
	} else if (armClkRate_kHz <= 4500000) {
		armClk_div = 0x07;
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INITARM_INV_ARMCLK_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
    regdat = refarm_div | (armClk_div << 4) | (0 << 8);//8 bit选择refclk 分频器输出
    retVal = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_ARMCLK_DIV,  &regdat, 1);
    IF_ERR_RETURN_U32(retVal);
    regdat = 0;
    retVal = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_ARMCLK_DIV_ACTION,  &regdat, 1);
    IF_ERR_RETURN_U32(retVal);
    regdat = 1 | (0 << 1) | (0 << 2);//1 bit选择refclk 分频器生效
    retVal = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_ARMCLK_DIV_ACTION,  &regdat, 1);
    IF_ERR_RETURN_U32(retVal);
    regdat = 0 | (0 << 1) | (0 << 2);//1 bit选择refclk 分频器生效
    retVal = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_ARMCLK_DIV_ACTION,  &regdat, 1);
    IF_ERR_RETURN_U32(retVal);	
	return (uint32_t)retVal;
}

uint32_t BR3109_writeArmProfile(br3109Device_t *device, br3109Init_t *init)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;

    return (uint32_t)retVal;
}

uint32_t BR3109_loadArmFromBinary(br3109Device_t *device, uint32_t *binary, uint32_t count)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
    static const uint32_t MAX_BIN_CNT = 0x35000;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_loadArmFromBinary()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (binary == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                TAL_ERR_LOADBIN_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    if (count > MAX_BIN_CNT)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                TAL_ERR_LOADBIN_INVALID_BYTECOUNT, retVal, TALACT_ERR_CHECK_PARAM);
    }
    else
    {
        /* writing binary data to ARM memory */
        retVal = (talRecoveryActions_t)BR3109_writeArmMem(device, BR3109_ADDR_ARM_START_PROG_ADDR | 0x80000000, &binary[0], count);
        IF_ERR_RETURN_U32(retVal);
    }

    /* verifying ARM checksum */
//    retVal = (talRecoveryActions_t)BR3109_verifyArmChecksum(device);
//    IF_ERR_RETURN_U32(retVal);
    device->devStateInfo.devState = (br3109States_t)(device->devStateInfo.devState | TAL_STATE_ARMLOADED);
    return (uint32_t)retVal;
}

uint32_t BR3109_readArmMem(br3109Device_t *device, uint32_t address, uint32_t *returnDataWord, uint32_t WordsToRead, uint8_t autoIncrement)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_readArmMem()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* check that start and stop address are in valid range */
    if (!((address & 0x7FFFFFFF) >= BR3109_ADDR_ARM_START_PROG_ADDR && (address & 0x7FFFFFFF) <= BR3109_ADDR_ARM_END_PROG_ADDR ) &&
        !((address & 0x7FFFFFFF) >= BR3109_ADDR_ARM_START_DATA_ADDR && (address & 0x7FFFFFFF) <= BR3109_ADDR_ARM_END_DATA_ADDR) &&
        !((address & 0x7FFFFFFF) >= BR_SPI_SYSCTRL_ADDR_BASE))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_READARMMEM_INV_ADDR_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    if (!(((address & 0x7FFFFFFF) + WordsToRead*4 - 1) >= BR3109_ADDR_ARM_START_PROG_ADDR && ((address & 0x7FFFFFFF) + WordsToRead*4 - 1) <= BR3109_ADDR_ARM_END_PROG_ADDR ) &&
        !(((address & 0x7FFFFFFF) + WordsToRead*4 - 1) >= BR3109_ADDR_ARM_START_DATA_ADDR && ((address & 0x7FFFFFFF) + WordsToRead*4 - 1) <= BR3109_ADDR_ARM_END_DATA_ADDR)&&
        !((address & 0x7FFFFFFF) >= BR_SPI_SYSCTRL_ADDR_BASE))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_READARMMEM_INV_ADDR_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    halError = brSpiReadWordsBlock(device->devHalInfo, address, returnDataWord, WordsToRead);
    retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}

uint32_t BR3109_writeArmMem(br3109Device_t *device, uint32_t address, uint32_t *dataWord, uint32_t WordCount)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
#if BR3109_VERBOSE
    brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_writeArmMem()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (!((address & 0x7FFFFFFF) >= BR3109_ADDR_ARM_START_PROG_ADDR && (address & 0x7FFFFFFF) <= BR3109_ADDR_ARM_END_PROG_ADDR) &&
        !((address & 0x7FFFFFFF) >= BR3109_ADDR_ARM_START_DATA_ADDR && (address & 0x7FFFFFFF) <= BR3109_ADDR_ARM_END_DATA_ADDR))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_WRITEARMMEM_INV_ADDR_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    if (!(((address & 0x7FFFFFFF) + WordCount*4 - 1) >= BR3109_ADDR_ARM_START_PROG_ADDR && ((address & 0x7FFFFFFF) + WordCount*4-1) <= BR3109_ADDR_ARM_END_PROG_ADDR ) &&
        !(((address & 0x7FFFFFFF) + WordCount*4-1) >= BR3109_ADDR_ARM_START_DATA_ADDR && ((address & 0x7FFFFFFF) + WordCount*4-1) <= BR3109_ADDR_ARM_END_DATA_ADDR))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_WRITEARMMEM_INV_ADDR_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* starting write at zero address offset */
	halError = brSpiWriteWordsBlock(device->devHalInfo, address, dataWord, WordCount);
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);
    return (uint32_t)retVal;
}

uint32_t BR3109_writeArmConfig(br3109Device_t *device, uint8_t objectId, uint16_t offset, uint8_t *data, uint8_t byteCount)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    return (uint32_t)retVal;
}

uint32_t BR3109_readArmConfig(br3109Device_t *device, uint8_t objectId, uint16_t offset, uint8_t *data, uint8_t byteCount)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;

    return (uint32_t)retVal;
}

uint32_t BR3109_readArmCmdStatus(br3109Device_t *device, uint32_t *errorWord, uint32_t *statusWord)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
    brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_readArmCmdStatus()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* performing null pointer check */
    if ((errorWord == NULL) ||
        (statusWord == NULL))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_READARMCMDSTATUS_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* ensuring the errorWord and statusWord are cleared */
    *errorWord = 0;
    *statusWord = 0;

    /* read in the entire 64-bit status register into a byte array and parse one byte at a time */
    halError = brSpiReadWord(device->devHalInfo, BR3109_ADDR_CMD_IT_BASE, statusWord);
    retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}

uint32_t BR3109_readArmCmdStatusWord(br3109Device_t *device, uint32_t opCode, uint32_t *cmdStatword)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
    uint32_t cmdword = 0;

#if BR3109_VERBOSE
    brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_readArmCmdStatusWord()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* check for even-numbered opCodes including opcode 0, but must not be greater than 30 */
    if (0)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ARMCMDSTATUS_INV_OPCODE_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    else
    {
        /* reading the command status register for given opcode */
        halError = brSpiReadWord(device->devHalInfo, BR3109_ADDR_WAKE_MCU, &cmdword);
        retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
        if((cmdword & (1 << BR3109_ADDR_WAKE_MCU_BIT)) == 0){
			halError = brSpiReadWord(device->devHalInfo, BR3109_ADDR_CMD_IT_BASE, &cmdword);
			retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
			IF_ERR_RETURN_U32(retVal);
        }

        /* identifying nibble location in command register for given opcode */
        *cmdStatword = cmdword;

        return (uint32_t)retVal;
    }
}

uint32_t BR3109_waitArmCmdStatus(br3109Device_t *device, uint32_t opCode, uint32_t *cmdStatword, uint32_t timeout_us, uint32_t waitInterval_us)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
    uint32_t eventCheck = 0;
    uint32_t numEventChecks = 0;


#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_waitArmCmdStatus()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif


    waitInterval_us = (waitInterval_us > timeout_us) ? timeout_us : waitInterval_us;
    numEventChecks = (waitInterval_us == 0) ? 1 : (timeout_us / waitInterval_us);

    /* timeout event check loop */
    for (eventCheck = 0; eventCheck <= numEventChecks; eventCheck++)
    {
        /* read status of opcode */
        retVal = (talRecoveryActions_t)BR3109_readArmCmdStatusWord(device, opCode, cmdStatword);
        IF_ERR_RETURN_U32(retVal);


        /* if pending bit is set for opcode of interest and the number of events have not expired, perform wait */
        if ((*cmdStatword  != TALAPI_ARMSPI_READY) &&
              (eventCheck < numEventChecks))
        {
            halError = BRHAL_wait_us(device->devHalInfo, waitInterval_us);
            retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_WAIT, halError, retVal, TALACT_ERR_CHECK_TIMER);
        }
        else
        {
            break;
        }
    }

    /* if ARM Command did not complete within the timeout period */
    if (*cmdStatword  != TALAPI_ARMSPI_READY) 
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_FAIL, TAL_ERR_WAITARMCMDSTATUS_TIMEOUT, retVal, TALACT_ERR_RESET_ARM);
    }

    return (uint32_t)retVal;
}

uint32_t BR3109_sendArmCommand(br3109Device_t *device, uint32_t opCode, const uint32_t *extendedData, uint32_t extendedDataNumWords)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
    uint32_t armCommandBusy = 0;
    uint8_t i = 0;
    uint16_t extCmdByteStartAddr = BR3109_ADDR_ARM_EXT_CMD_WORD_1;

    uint32_t timeout_us = SENDARMCMD_TIMEOUT_US;
    uint32_t waitInterval_us = SENDARMCMD_INTERVAL_US;
	uint8_t cmd_size = (opCode>>TALAPI_ARMSPI_OPCODE_CMDSIZE_POS)&0xFF;
#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_sendArmCommand()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* the number of valid extended data bytes is from 0-4 */
    if (extendedDataNumWords > 7 || opCode == 0)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ARMCMD_INV_NUMBYTES_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* setting a 2 second timeout for mailbox busy bit to be clear (can't send an arm mailbox command until mailbox is ready) */

    waitInterval_us = (waitInterval_us > timeout_us) ? timeout_us : waitInterval_us;

	retVal = BR3109_waitArmCmdStatus(device, opCode, &armCommandBusy, timeout_us, waitInterval_us);
	IF_ERR_RETURN_U32(retVal);
    /* if busy bit remains set after timeout event loop function is exited, otherwise command is sent after extended bytes */
    if (armCommandBusy != TALAPI_ARMSPI_READY)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_FAIL, TAL_ERR_TIMEDOUT_ARMMAILBOXBUSY, retVal, TALACT_ERR_RESET_ARM);
    }
    else
    {
		if(cmd_size > 0)
		{
			for(i = 0; i < cmd_size; i++)
			{
	        	halError = brSpiWriteWord(device->devHalInfo, BR3109_ADDR_CMD_IT_BASE+0x08+i*4, extendedData[i]);	//psrc
		        retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
		        IF_ERR_RETURN_U32(retVal);
			}
		}
        if (extendedDataNumWords > cmd_size)
        {
            for (i = cmd_size; i < extendedDataNumWords; i++)
            {
                halError = brSpiWriteWord(device->devHalInfo, (extCmdByteStartAddr + (i-cmd_size)*4), extendedData[i]);
                retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
                IF_ERR_RETURN_U32(retVal);
            }
        }

        halError = brSpiWriteWord(device->devHalInfo, BR3109_ADDR_ARM_COMMAND, opCode&0xffff);
        retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
        halError = brSpiWriteWord(device->devHalInfo, BR3109_ADDR_CMD_IT_BASE, TALAPI_ARMSPI_BUSY);
        retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
        halError = brSpiWriteWord(device->devHalInfo, BR3109_ADDR_WAKE_MCU, 1);
        retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
    }

    return (uint32_t)retVal;
}

uint32_t BR3109_getArmVersion_v2(br3109Device_t *device, br3109ArmVersionInfo_t *talArmVersionInfo)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    uint32_t ver[5] = {0};
    uint32_t fullVersion = 0;

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getArmVersion()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (talArmVersionInfo == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETARMVER_V2_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    // if ((device->devStateInfo.devState & TAL_STATE_ARMLOADED) != TAL_STATE_ARMLOADED)
    // {
    //     return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETARMVER_V2_INVALID_ARM_NOT_LOADED, retVal, TALACT_ERR_CHECK_PARAM);
    // }

    retVal = (talRecoveryActions_t)BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_ARM_GLOBLE_CONFIG_DATA, &fullVersion, 1);
    IF_ERR_RETURN_U32(retVal);
    // fullVersion = (((uint32_t)ver[0]) | (((uint32_t)ver[1]) << 8) | (((uint32_t)ver[2]) << 16) | (((uint32_t)ver[3]) << 24));
    talArmVersionInfo->rcVer = (uint8_t)(fullVersion & 0xff);
    talArmVersionInfo->minorVer = (uint8_t)((fullVersion >> 8) & 0xff);
    talArmVersionInfo->majorVer = (uint8_t)((fullVersion >> 16) & 0xff);

    if(talArmVersionInfo->rcVer & 0x01)
    {
        talArmVersionInfo->buildType = TAL_ARM_BUILD_DEBUG;
    }
    else if(talArmVersionInfo->rcVer & 0x04)
    {
        talArmVersionInfo->buildType = TAL_ARM_BUILD_TEST_OBJECT;
    }
    else
    {
        talArmVersionInfo->buildType = TAL_ARM_BUILD_RELEASE;
    }

    return (uint32_t)retVal;
}

uint32_t BR3109_getArmVersion(br3109Device_t *device, uint8_t *majorVer, uint8_t *minorVer, uint8_t *rcVer)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    uint32_t ver[4] = {0};
    uint32_t fullVersion = 0;

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getArmVersion()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if ((majorVer == NULL) ||
        (minorVer == NULL) ||
        (rcVer == NULL))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETARMVER_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    retVal = (talRecoveryActions_t)BR3109_readArmMem(device, BR3109_ADDR_ARM_VERSION, &ver[0], sizeof(ver)/4, 0);
    IF_ERR_RETURN_U32(retVal);

    fullVersion = (((uint32_t)ver[0]) | (((uint32_t)ver[1]) << 8) | (((uint32_t)ver[2]) << 16) | (((uint32_t)ver[3]) << 24));
    *rcVer = (uint8_t)(fullVersion % 100);
    *minorVer = (uint8_t)((fullVersion / 100) % 100);
    *majorVer = (uint8_t)(fullVersion / 10000);

    return (uint32_t)retVal;
}

uint32_t BR3109_ArmWriteField(br3109Device_t *device, uint32_t addr, uint32_t fieldVal, uint32_t mask, uint32_t startBit)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t readVal=0;
	if(addr&TAL_ARM_SPI_FLAG)//地址最高位表示spi寄存器控制
	{
		retVal = BR3109_armSpiCmd_SPI_blk_read(device, (addr&TAL_ARM_SPI_ID_MASK)>>TAL_ARM_SPI_ID_POS, addr&TAL_ARM_SPI_ADDR_MASK, &readVal, 1);
    	IF_ERR_RETURN_U32(retVal);
    	readVal = (readVal & ~mask) | ((fieldVal << startBit) & mask);
		retVal = BR3109_armSpiCmd_SPI_blk_write(device, (addr&TAL_ARM_SPI_ID_MASK)>>TAL_ARM_SPI_ID_POS, addr&TAL_ARM_SPI_ADDR_MASK, &readVal, 1);
    	IF_ERR_RETURN_U32(retVal);		
	}
	else
	{
		retVal = BR3109_armMemoryCmd_blk_read(device, addr, &readVal, 1);
    	IF_ERR_RETURN_U32(retVal);
    	readVal = (readVal & ~mask) | ((fieldVal << startBit) & mask);
		retVal = BR3109_armMemoryCmd_blk_write(device, addr, &readVal, 1);
    	IF_ERR_RETURN_U32(retVal);
	}	
    return (uint32_t)retVal;
}

// uint32_t BR3109_verifyArmChecksum_2(br3109Device_t *device, uint32_t *data, uint32_t len)
// {
//     brHalErr_t halError = BRHAL_OK;
//     talRecoveryActions_t retVal = TALACT_NO_ACTION;
//     int i = 0,j = 0;
// #define RD_DATA_MAX_LEN 128
//     uint32_t rdata[RD_DATA_MAX_LEN];
//     uint32_t rlen = RD_DATA_MAX_LEN;
//     uint32_t remain_len = len;
//     uint32_t raddr = 0;
// #if BR3109_VERBOSE
//     halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_verifyArmChecksum()\n");
//     retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
// #endif
//     for(i = 0; i< 1.0*len/RD_DATA_MAX_LEN; i++){
//         /* reading four (4) bytes at ARM checksum memory location */
//         rlen = remain_len < RD_DATA_MAX_LEN?remain_len : RD_DATA_MAX_LEN;
//         raddr = TAL_ARM_SPI_FLAG + 0 +i*RD_DATA_MAX_LEN*4;
//         retVal = (talRecoveryActions_t)BR3109_readArmMem(device, raddr, rdata, rlen, 0);
//         IF_ERR_RETURN_U32(retVal);
//         for(j = 0; j < rlen; j++){
//           if(rdata[j] != data[i*RD_DATA_MAX_LEN+j]){
//             raddr = raddr+2;
//             return FAILURE;
//           }
//         }
//         remain_len -= rlen;
//         if(remain_len == 0){
//             break;
//         }
//     }
//     return (uint32_t)retVal;
// }

uint32_t BR3109_verifyArmChecksum(br3109Device_t *device, uint32_t checksum)
{
    brHalErr_t halError = BRHAL_OK;
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    uint32_t rchecksum = 0;
    uint8_t errorFlag = 0;
#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_verifyArmChecksum()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
    retVal = BR3109_waitInitCals(device, 50000, &errorFlag);
	IF_ERR_RETURN_U32(retVal);
	/* reading four (4) bytes at ARM checksum memory location */
	retVal = (talRecoveryActions_t)BR3109_readArmMem(device, TAL_ARM_SPI_FLAG + 0x3F30, &rchecksum, 1, 0);
	IF_ERR_RETURN_U32(retVal);
	if(rchecksum != checksum){
		retVal = TAL_ERR_VERIFYBIN_CHECKSUM_ERR;
	}
    return (uint32_t)retVal;
}

const char* talGetArmErrorMessage(uint32_t errSrc, uint32_t errCode)
{
    

    return "Wrong error handler - not a Br3109 ARM error\n";
}

talRecoveryActions_t talArmCmdErrorHandler(br3109Device_t *device, br3109ErrHdls_t errHdl,
        uint32_t detErr, talRecoveryActions_t retVal, talRecoveryActions_t recAction)
{
    
    return recAction;
}
