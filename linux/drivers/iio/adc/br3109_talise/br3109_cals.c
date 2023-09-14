/**
 * \file br3109_cals.c
 * \brief Contains functions to support Br3109 init and tracking calibrations
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#include "br3109_cals.h"
#include "br3109_reg_addr_macros.h"
#include "br3109_arm_macros.h"
#include "br3109_hal.h"
#include "br3109_user.h"
#include "br3109_error.h"
#include "br3109_radioctrl.h"
#include "br3109_arm.h"
#include "br3109.h"

uint32_t  BR3109_runInitCals (br3109Device_t *device, uint32_t calMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
    uint8_t bw = device->devStateInfo.txBandwidth_Hz <= device->devStateInfo.txBandwidth_SW_Hz? 0 : 1;
    uint32_t regdat = 0x5;
    // uint8_t payload[4] = {0};

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_runInitCals()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    retValWarn = retVal;

    // payload[0] = (uint8_t)calMask;
    // payload[1] = (uint8_t)(calMask >> 8);
    // payload[2] = (uint8_t)(calMask >> 16);
    // payload[3] = (uint8_t)(calMask >> 24);
//	retVal = BR3109_armSpiCmd_playcap_set(device, 0, 1, 0, 0x01, 0, 0x2, 0x1);
//    IF_ERR_RETURN_U32(retVal);
//	retVal = BR3109_armMemoryCmd_blk_write(device, (APB_TX_PATH_TOP_PLAY_MODE ), &regdat, 1);
//	IF_ERR_RETURN_U32(retVal);
//    retVal = BR3109_armSpiCmd_playcap_manual_start(device, 0x1);
//	IF_ERR_RETURN_U32(retVal);
    retVal = (talRecoveryActions_t)BR3109_armSpiCmd_Initical_cali(device, 0x3, 0x3, 0x3, 0, 0, bw, 0x0,calMask);//
    IF_ERR_RETURN_U32(retVal);


    device->devStateInfo.devState = (br3109States_t)(device->devStateInfo.devState | TAL_STATE_CALS_RUN);

    /* If higher priority retVal has no error, allow possible lower priority warning to be returned */
    if (retVal == TALACT_NO_ACTION)
    {
        retVal = retValWarn;
    }

    return (uint32_t)retVal;
}


uint32_t  BR3109_waitInitCals (br3109Device_t *device, uint32_t timeoutMs, uint8_t *errorFlag) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
    uint32_t cmdstatword = 0;
    uint8_t _errFlag = 0;

    static const uint16_t TIMEOUT_MS_FACTOR = 1000;
    static const uint32_t CODECHECK_PARAM_WAITINITCALS_ERR = 2;

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_waitInitCals()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (errorFlag == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                TAL_ERR_WAIT_INITCALS_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    retValWarn = retVal;
	retVal = BR3109_waitArmCmdStatus(device, TALAPI_ARMSPI_CMD_INIT_CAL, &cmdstatword,  (timeoutMs * TIMEOUT_MS_FACTOR), WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);	
	cmdstatword &= ~TALAPI_ARMSPI_READY;//此处进行ready对比
    _errFlag = (cmdstatword >> 1);

    /* SW Test */
    if (device->devStateInfo.swTest == CODECHECK_PARAM_WAITINITCALS_ERR)
    {
        retVal = TALACT_ERR_RESET_SPI;
        cmdstatword = 2;
        _errFlag = (cmdstatword >> 1);
    }

    /* Don't update errorFlag if SPI error because errorFlag could be a random */
    /* value but update error flag for other recovery action types */
    if (retVal == TALACT_ERR_RESET_SPI)
    {
        *errorFlag = 0;
    }
    else
    {
        *errorFlag = _errFlag;
    }

    /* ARM error handler to provide valid recovery action based on ARM error
     * code */
    if (_errFlag > 0)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_ARM_CMD_ERR,
                ARMCMD_ERRCODE(TALAPI_ARMSPI_CMD_INIT_CAL, 0, cmdstatword), retVal, TALACT_ERR_RESET_ARM);
    }
    else
    {
        IF_ERR_RETURN_U32(retVal);
    }

    /* if no error from higher priority calls, return possible log warning */
    if (retVal == TALACT_NO_ACTION)
    {
        retVal = retValWarn;
    }

    return (uint32_t)retVal;
}

uint32_t  BR3109_setDigDcOffsetMShift (br3109Device_t *device, br3109DcOffsetChannels_t channel, uint8_t mShift) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    // talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
    uint32_t dcOffset = 0;

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setDigDcOffsetMShift()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    switch (channel) {
    case TAL_DC_OFFSET_RX_CHN:
        dcOffset |= mShift <<16 |mShift;
        retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_DC_CORR_MAN_RX_CH1_IQ, &dcOffset, 1);
        IF_ERR_RETURN_U32(retVal);
        retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_DC_CORR_MAN_RX_CH2_IQ, &dcOffset, 1);
        IF_ERR_RETURN_U32(retVal);
        break;

    case TAL_DC_OFFSET_ORX_CHN:
        dcOffset |= mShift <<16 |mShift;
        retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_DC_CORR_MAN_ORX_CH1_IQ, &dcOffset, 1);
        IF_ERR_RETURN_U32(retVal);
        retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_DC_CORR_MAN_ORX_CH2_IQ, &dcOffset, 1);
        IF_ERR_RETURN_U32(retVal);
        break;
    
    default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_SETDCOFFSET_INV_CHN, retVal, TALACT_ERR_CHECK_PARAM);
        break;
    }
    
    return (uint32_t)retVal;

}


uint32_t  BR3109_getDigDcOffsetMShift (br3109Device_t *device, br3109DcOffsetChannels_t channel, uint8_t *mShift) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
    uint32_t dcOffset = 0;

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getDigDcOffsetMShift()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (mShift == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETDCOFFSET_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    

    switch (channel) {
    case TAL_DC_OFFSET_RX_CHN:
        retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_DC_CORR_MAN_RX_CH1_IQ, &dcOffset, 1);
        IF_ERR_RETURN_U32(retVal);
        break;

    case TAL_DC_OFFSET_ORX_CHN:
        retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_DC_CORR_MAN_ORX_CH1_IQ, &dcOffset, 1);
        IF_ERR_RETURN_U32(retVal);
        break;
    
    default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETDCOFFSET_INV_CHN, retVal, TALACT_ERR_CHECK_PARAM);
        break;
    }

    *mShift = (uint8_t)(dcOffset & 0xF);
    
    return (uint32_t)retVal;
}

/**
 *     Channel              |  Value  |  Channel description
 * -------------------------|---------|--------------------------
 *  TAL_DC_OFFSET_ALL_OFF   |   0x00  | Disable all the channels
 *  TAL_DC_OFFSET_RX1       |   0x01  | Enables Rx1
 *  TAL_DC_OFFSET_RX2       |   0x02  | Enables Rx1
 *  TAL_DC_OFFSET_ORX1      |   0x04  | Enables ORx1
 *  TAL_DC_OFFSET_ORX2      |   0x08  | Enables ORx2
 *  TAL_DC_OFFSET_ALL_ON    |   0x0F  | Enables all the channels
*/

uint32_t  BR3109_setDigDcOffsetEn (br3109Device_t *device, uint8_t enableMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
    uint32_t enableVal = 0;
    uint32_t wrMask = 0;

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setDigDcOffsetEn()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    wrMask |= BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH1_EN_MASK;
    wrMask |= BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH2_EN_MASK;
    wrMask |= BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH1_EN_MASK;
    wrMask |= BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH2_EN_MASK;

    if (enableMask & TAL_DC_OFFSET_RX1) {
        enableVal |= BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH1_EN_MASK;
    }
    if (enableMask & TAL_DC_OFFSET_RX2) {
        enableVal |= BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH2_EN_MASK;
    }
    if (enableMask & TAL_DC_OFFSET_ORX1) {
        enableVal |= BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH1_EN_MASK;
    }
    if (enableMask & TAL_DC_OFFSET_ORX2) {
        enableVal |= BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH2_EN_MASK;
    }

    retVal = BR3109_ArmWriteField(device,BR3109_ADDR_DC_CORR_MAN_MODE_SETTING,enableVal,wrMask,0);
    IF_ERR_RETURN_U32(retVal);
    return (uint32_t) retVal;
}

/**
 *     Channel              |  Value  |  Channel description
 * -------------------------|---------|--------------------------
 *  TAL_DC_OFFSET_ALL_OFF   |   0x00  | Disable all the channels
 *  TAL_DC_OFFSET_RX1       |   0x01  | Enables Rx1
 *  TAL_DC_OFFSET_RX2       |   0x02  | Enables Rx1
 *  TAL_DC_OFFSET_ORX1      |   0x04  | Enables ORx1
 *  TAL_DC_OFFSET_ORX2      |   0x08  | Enables ORx2
 *  TAL_DC_OFFSET_ALL_ON    |   0x0F  | Enables all the channels
*/

uint32_t  BR3109_getDigDcOffsetEn (br3109Device_t *device, uint8_t *enableMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
    uint32_t enableVal = 0;
    uint32_t rdMask = 0;

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setDigDcOffsetEn()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (enableMask == NULL) {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETDCOFFSETEN_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    retVal = BR3109_armMemoryCmd_blk_read(device,BR3109_ADDR_DC_CORR_MAN_MODE_SETTING,&enableVal,1);
    IF_ERR_RETURN_U32(retVal);
    rdMask |= BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH1_EN_MASK;
    rdMask |= BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH2_EN_MASK;
    rdMask |= BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH1_EN_MASK;
    rdMask |= BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH2_EN_MASK;

    *enableMask = (uint8_t)(enableVal & rdMask);

    return (uint32_t) retVal;
}

#if 0
uint32_t  BR3109_checkInitCalComplete (br3109Device_t *device, uint8_t *areCalsRunning, uint8_t *errorFlag) 
{
	return 123;
}


uint32_t  BR3109_abortInitCals (br3109Device_t *device, uint32_t *calsCompleted) 
{
	return 123;
}

uint32_t  BR3109_getInitCalStatus (br3109Device_t *device, uint32_t *calsSincePowerUp, uint32_t *calsLastRun, uint32_t *calsMinimum, uint8_t *initErrCal, uint8_t *initErrCode) 
{
	return 123;
}
#endif
uint32_t  BR3109_enableTrackingCals (br3109Device_t *device, uint32_t enableMask) 
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = enableMask;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TRACK_CALI, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    device->devStateInfo.tracking_en = enableMask;
	return retVal;
}

#if 0
uint32_t  BR3109_getEnabledTrackingCals (br3109Device_t *device, uint32_t *enableMask) 
{
	return 123;
}


uint32_t  BR3109_getPendingTrackingCals (br3109Device_t *device, uint32_t *pendingCalMask) 
{
	return 123;
}

uint32_t  BR3109_rescheduleTrackingCal (br3109Device_t *device, br3109TrackingCalibrations_t trackingCal) 
{
	return 123;
}
#endif
uint32_t  BR3109_setAllTrackCalState (br3109Device_t *device, uint32_t calSubsetMask, uint32_t resumeCalMask) 
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
    uint32_t enableMask = 0;  
    enableMask = (device->devStateInfo.tracking_en & (~calSubsetMask)) | (device->devStateInfo.tracking_en & (calSubsetMask&resumeCalMask));
	retVal = BR3109_enableTrackingCals(device, enableMask);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

#if 0
uint32_t  BR3109_getAllTrackCalState (br3109Device_t *device, uint32_t *resumeCalMask) 
{
	return 123;
}

uint32_t  BR3109_getTxLolStatus (br3109Device_t *device, br3109TxChannels_t channelSel, br3109TxLolStatus_t *txLolStatus) 
{
return 123;
} 
uint32_t  BR3109_getTxQecStatus (br3109Device_t *device, br3109TxChannels_t channelSel, br3109TxQecStatus_t *txQecStatus) 
{
	return 123;
}


uint32_t  BR3109_getRxQecStatus (br3109Device_t *device, br3109RxChannels_t channelSel, br3109RxQecStatus_t *rxQecStatus) 
{
	return 123;
}


uint32_t  BR3109_getOrxQecStatus (br3109Device_t *device, br3109ObsRxChannels_t channelSel, br3109OrxQecStatus_t *orxQecStatus) 
{
	return 123;
}


uint32_t  BR3109_getRxHd2Status (br3109Device_t *device, br3109RxChannels_t channelSel, br3109RxHd2Status_t *rxHd2Status) 
{
	return 123;
}

uint32_t  BR3109_waitForEvent (br3109Device_t *device, br3109WaitEvent_t waitEvent, uint32_t timeout_us, uint32_t waitInterval_us) 
{
	return 123;
}


uint32_t  BR3109_readEventStatus (br3109Device_t *device, br3109WaitEvent_t waitEvent, uint8_t *eventDone) 
{
return 10;
}


uint32_t  BR3109_resetExtTxLolChannel (br3109Device_t *device, br3109TxChannels_t channelSel) 
{
return 10;
}

uint32_t  BR3109_setRxHd2Config (br3109Device_t *device, br3109RxHd2Config_t *hd2CalConfig) 
{
	return 10;
}


uint32_t  BR3109_getRxHd2Config (br3109Device_t *device, br3109RxHd2Config_t *hd2CalConfig) 
{
	return 10;
}



uint32_t  BR3109_getTrackingCalsBatchSize (br3109Device_t *device, br3109TrackingCalBatchSize_t *batchsize_us) 
{
	return 10;
}

uint32_t  BR3109_setTrackingCalsBatchSize (br3109Device_t *device, br3109TrackingCalBatchSize_t batchsize_us) 
{
	return 10;
}

const char *  talGetCalErrorMessage (uint32_t errSrc, uint32_t errCode) 
{
	return 10;
}
#endif
