/**
 * \file br3109_agc.c
 * \brief Contains Br3109 API AGC function calls
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#include "br3109_agc.h"
#include "br3109_agc.h"
#include "br3109_reg_addr_macros.h"
#include "br3109_user.h"
#include "br3109_hal.h"
#include "br3109_error.h"
#include "br3109_rx_types.h"
#include "br3109_gpio.h"
#include "br3109_arm.h"
uint32_t BR3109_setupRxAgc(br3109Device_t *device, br3109AgcCfg_t *rxAgcCtrl)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;
	uint32_t agcClock_Hz = 0;
	uint32_t agcGainUpdateCounter = 0;
	uint32_t agcUnderRangeLowInterval = 0;
	uint32_t adcClock_Hz = 0;
	uint8_t minAgcSlowLoopSettlingDelay = 0;

	static const uint8_t agcPeakWaitTimeBitMask = 0x1F;
	static const uint32_t agcGainUpdateCounterBitMask = 0x003FFFFF;
	static const uint8_t powerEnableMeasurementBitMask = 0x01;
//	static const uint8_t powerUseRfirOutBitMask = 0x02;
//	static const uint8_t powerUseBBDC2BitMask = 0x08;
	static const uint32_t underRangeHighPowerThreshBitMask = 0x0FFFFFFF;
	static const uint32_t underRangeLowPowerThreshBitMask = 0x0FFFFFFF;
	static const uint8_t underRangeHighPowerGainStepRecoveryBitMask = 0x3F;
	static const uint8_t underRangeLowPowerGainStepRecoveryBitMask = 0x3F;
	static const uint32_t powerMeasurementDurationBitMask = 0x1FFFF;
	static const uint8_t agcSlowLoopSettlingDelayBitMask = 0xFF;
	static const uint8_t apdHighThreshMin = 0x07;
	static const uint8_t apdHighThreshMax = 0x31;
	static const uint8_t apdLowThreshMin = 0x07;
	static const uint8_t apdLowThreshMax = 0x31;
	static const uint8_t apdGainStepAttackBitMask = 0x1F;
	static const uint8_t apdGainStepRecoveryBitMask = 0x1F;
	static const uint8_t enableHb2OverloadBitMask = 0x01;
	static const uint8_t hb2OverloadDurationCntBitMask = 0x7F;
	static const uint8_t hb2OverloadThreshCntBitMask = 0x0F;
	static const uint8_t hb2GainStepHighRecoveryBitMask = 0x1F;
	static const uint8_t hb2GainStepLowRecoveryBitMask = 0x1F;
	static const uint8_t hb2GainStepAttackBitMask = 0x1F;
	static const uint8_t hb2GainStepMidRecoveryBitMask = 0x1F;
	static const uint8_t hb2OverloadPowerModeBitMask = 0x01;
	static const uint8_t hb2OvrgSelBitMask = 0x01;
	static const uint8_t hb2ThreshConfigBitMask = 0x03;
	static const uint32_t upper0PowerThreshBitMask = 0x0FFFFFFF;
	static const uint32_t upper1PowerThreshBitMask = 0x0FFFFFFF;
	static const uint8_t powerLogShiftBitMask = 0x0F;
	static const uint8_t overRangeLowPowerGainStepAttackBitMask = 0x1F;
	static const uint8_t overRangeHighPowerGainStepAttackBitMask = 0x1F;

	static const uint32_t agcLowThreshPreventGainBitMask = (0x1 << 8);
	static const uint32_t agcChangeGainIfThreshHighBitMask = (0x3 << 12);
	static const uint32_t agcPeakThreshGainControlModeBitMask = 0x04;
	static const uint32_t agcResetOnRxonBitMask = (0x1 << 17);
	static const uint32_t agcEnableSyncPulseForGainCounterBitMask = (0x01 << 18);
	static const uint32_t agcEnableIp3OptimizationThreshBitMask = 0x80;
	static const uint32_t agcEnableFastRecoveryLoopBitMask = (0x1 << 19);
	static const uint8_t agcRx1AttackDelayBitMask = 0xFF;
	static const uint8_t agcRx2AttackDelayBitMask = 0xFF;
	static const uint32_t agcUnderRangeLowIntervalMask = 0x0000FFFF;
	static const uint8_t agcUnderRangeMidIntervalMask = 0x3F;
	static const uint8_t agcUnderRangeHighIntervalMask = 0x3F;

//	static const uint8_t MIN_SUPPORTED_SIREV = 0xC0;
//	static const uint8_t ENABLE_IP3_OPTIMIZATION_MASK = 0x80;

#if BR3109_VERBOSE
	halError =brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,"BR3109_setupRxAgc()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,TALACT_WARN_RESET_LOG);
#endif

	/* checking for valid br3109AgcCfg_t device->rx->rxAgcCtrl pointer to determine if it has been initialized */
	if (rxAgcCtrl == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,TAL_ERR_INV_AGC_RX_STRUCT_INIT, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* performing AGC peak wait time value check */
	if (rxAgcCtrl->agcPeakWaitTime & ~agcPeakWaitTimeBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,TAL_ERR_INV_AGC_RX_PEAK_WAIT_TIME_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		/* write AGC peak wait time */
		regdat = rxAgcCtrl->agcPeakWaitTime;
		retVal = BR3109_ArmWriteField( device, BR3109_ADDR_AGC_CONFIG1, regdat, agcPeakWaitTimeBitMask, 0);
		IF_ERR_RETURN_U32(retVal);
	}

	/* performing range check for Rx1 and Rx2 max gain exceedence */
	if ((rxAgcCtrl->agcRx1MaxGainIndex > device->devStateInfo.rxGainCtrl.rx1MaxGainIndex) || (rxAgcCtrl->agcRx2MaxGainIndex > device->devStateInfo.rxGainCtrl.rx2MaxGainIndex) ||
	    (rxAgcCtrl->agcRx1MinGainIndex < device->devStateInfo.rxGainCtrl.rx1MinGainIndex) || (rxAgcCtrl->agcRx2MinGainIndex < device->devStateInfo.rxGainCtrl.rx2MinGainIndex)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_MIN_MAX_GAIN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		/* writing Rx1 and Rx2 max and min gain indices */
		regdat = rxAgcCtrl->agcRx1MaxGainIndex | (rxAgcCtrl->agcRx1MinGainIndex << 8);
		retVal = BR3109_ArmWriteField(device, BR3109_ADDR_MAX_MIN_GAIN_INDEX_RX1, regdat, 0xFFFF, 0);
		IF_ERR_RETURN_U32(retVal);

		regdat = rxAgcCtrl->agcRx2MaxGainIndex | (rxAgcCtrl->agcRx2MinGainIndex << 8);
		retVal = BR3109_ArmWriteField(device, BR3109_ADDR_MAX_MIN_GAIN_INDEX_RX2, regdat, 0xFFFF, 0);
		IF_ERR_RETURN_U32(retVal);
	}

	/*Calculation of Gain Update Time*/
	//todo
	/*Reading back AGC Clock Division Ratio*/
//	halError = brSpiReadField(device->devHalInfo, BR3109_ADDR_CLOCK_CONTROL_5,
//				   &agcGainUpdateCtr[0], agcClock_HzBitMask, 0);
//	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
//				  TALACT_ERR_RESET_SPI);
//	IF_ERR_RETURN_U32(retVal);
//
	adcClock_Hz = (device->devStateInfo.clocks.hsDigClkDiv2_Hz << 1);
//
//	/*Calculation of AGC clock in Hz*/
//	switch (agcGainUpdateCtr[0]) {
//	case 0:
//		agcClock_Hz = device->devStateInfo.clocks.hsDigClkDiv4or5_Hz;
//		break;
//
//	case 1:
//		agcClock_Hz = (device->devStateInfo.clocks.hsDigClkDiv4or5_Hz / 2);
//		break;
//
//	case 2:
//		agcClock_Hz = (device->devStateInfo.clocks.hsDigClkDiv4or5_Hz / 4);
//		break;
//
//	case 3:
//		agcClock_Hz = (device->devStateInfo.clocks.hsDigClkDiv4or5_Hz / 8);
//		break;
//
//	default:
//		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
//						  TAL_ERR_INV_AGC_CLK_DIV_RATIO_PARM, retVal, TALACT_ERR_CHECK_PARAM);
//	}
	agcClock_Hz = device->devStateInfo.clocks.hsDigClkDiv4or5_Hz;//br3109 固定491.52Mhz

	if (agcClock_Hz == 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_INV_AGC_CLK_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((adcClock_Hz % agcClock_Hz) != 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_INV_AGC_CLK_RATIO, retVal, TALACT_ERR_CHECK_PARAM);
	}

	minAgcSlowLoopSettlingDelay = (uint8_t)(65 / (adcClock_Hz / agcClock_Hz));

	agcGainUpdateCounter = (uint32_t)DIV_U64(((uint64_t)rxAgcCtrl->agcGainUpdateCounter_us
					   * (uint64_t)agcClock_Hz) , 1000000);

	/* performing range check for gain update time */
	if (agcGainUpdateCounter & ~agcGainUpdateCounterBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_INV_AGC_RX_GAIN_UPDATE_TIME_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		/* divide gain update counter value into three (3) registers to perform write */
		/* write two low order bytes directly due to byte width of register value */
		retVal = BR3109_ArmWriteField(device, BR3109_ADDR_AGC_CONFIG1, agcGainUpdateCounter, (agcGainUpdateCounterBitMask << 8), 8);
	}

	if (rxAgcCtrl->agcRx1AttackDelay & ~agcRx1AttackDelayBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX1ATTACKDELAY_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		regdat = rxAgcCtrl->agcRx1AttackDelay;
		retVal = BR3109_ArmWriteField(device, BR3109_ADDR_MANUAL_GAIN_INDEX_DELAY_RX1, regdat, agcRx1AttackDelayBitMask, 0);
		IF_ERR_RETURN_U32(retVal);
	}

	if (rxAgcCtrl->agcRx2AttackDelay & ~agcRx2AttackDelayBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX2ATTACKDELAY_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		regdat = rxAgcCtrl->agcRx2AttackDelay;
		retVal = BR3109_ArmWriteField(device, BR3109_ADDR_MANUAL_GAIN_INDEX_DELAY_RX2, regdat, agcRx2AttackDelayBitMask, 0);
		IF_ERR_RETURN_U32(retVal);
	}

	if (rxAgcCtrl->agcSlowLoopSettlingDelay < minAgcSlowLoopSettlingDelay) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_MINAGCSLOWLOOPDELAY_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* AGC Control register checks and write */
	if ((rxAgcCtrl->agcLowThreshPreventGain << 8) & ~agcLowThreshPreventGainBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_LOWTHRSHPREVENTGAIN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcChangeGainIfThreshHigh << 12) &  ~agcChangeGainIfThreshHighBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_CHANGEGAINTHRSHHIGH_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcPeakThreshGainControlMode << 16) & ~agcPeakThreshGainControlModeBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_LOWTHRSHPREVENTGAIN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcResetOnRxon << 17) & ~agcResetOnRxonBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_RESETONRXON_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcEnableSyncPulseForGainCounter << 18)  & ~agcEnableSyncPulseForGainCounterBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_ENSYNCHPULSECAINCNTR_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcEnableIp3OptimizationThresh << 7) & ~agcEnableIp3OptimizationThreshBitMask) {	//TODO no use delay^^^^^
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_ENIP3OPTTHRSH_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcEnableFastRecoveryLoop << 19) & ~agcEnableFastRecoveryLoopBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_ENFASTRECOVERYLOOP_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if (rxAgcCtrl->agcSlowLoopSettlingDelay & ~agcSlowLoopSettlingDelayBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_SLOWLOOPDELAY_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {				   
		regdat = rxAgcCtrl->agcSlowLoopSettlingDelay
				| (rxAgcCtrl->agcLowThreshPreventGain << 8)
				| (rxAgcCtrl->agcChangeGainIfThreshHigh << 12) 
				| (rxAgcCtrl->agcPeakThreshGainControlMode << 16)
				| (rxAgcCtrl->agcResetOnRxon << 17)
				| (rxAgcCtrl->agcEnableSyncPulseForGainCounter << 18)
				| (rxAgcCtrl->agcEnableFastRecoveryLoop << 19);

//		reg_mask = agcSlowLoopSettlingDelayBitMask
//				| agcLowThreshPreventGainBitMask 
//				| agcChangeGainIfThreshHighBitMask
//				| agcPeakThreshGainControlModeBitMask
//				| agcResetOnRxonBitMask
//				| agcEnableSyncPulseForGainCounterBitMask
//				| agcEnableFastRecoveryLoopBitMask;
		retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_SLOWLOOP_CONFIG, &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	}

	/* If peak detect data structure is not included into project */
	if (&rxAgcCtrl->agcPeak == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_STRUCT_INIT, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		agcUnderRangeLowInterval = (uint32_t)DIV_U64(((uint64_t) rxAgcCtrl->agcPeak.agcUnderRangeLowInterval_ns * (uint64_t)agcClock_Hz) , 1000000000);
		/* performing range check for gain update time */
		if (agcUnderRangeLowInterval & ~agcUnderRangeLowIntervalMask) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,  TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_UPDATE_TIME_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}
		if (rxAgcCtrl->agcPeak.agcUnderRangeMidInterval & ~agcUnderRangeMidIntervalMask ) {
				return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_MID_INTERVAL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		} 				
		if (rxAgcCtrl->agcPeak.agcUnderRangeHighInterval & ~agcUnderRangeHighIntervalMask ) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_HIGH_INTERVAL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		} 
		regdat = agcUnderRangeLowInterval | (rxAgcCtrl->agcPeak.agcUnderRangeMidInterval << 16) | (rxAgcCtrl->agcPeak.agcUnderRangeHighInterval <<24);
		retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_AGC_UNDERRANGE0, &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	}

	if ((rxAgcCtrl->agcPeak.apdHighThresh < apdHighThreshMin) || (rxAgcCtrl->agcPeak.apdHighThresh > apdHighThreshMax)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HIGHTHRSH_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcPeak.apdHighThresh < rxAgcCtrl->agcPeak.apdLowThresh)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_APD_HIGH_LOW_THRESH, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcPeak.apdLowThresh < apdLowThreshMin) || (rxAgcCtrl->agcPeak.apdLowThresh > apdLowThreshMax)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_LOWGAINHIGHTHRSH_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		regdat = rxAgcCtrl->agcPeak.apdHighThresh  | (rxAgcCtrl->agcPeak.apdLowThresh << 8) 
				| (rxAgcCtrl->agcPeak.apdUpperThreshPeakExceededCnt << 16) | (rxAgcCtrl->agcPeak.apdLowerThreshPeakExceededCnt << 24);
		//retVal = BR3109_ArmWriteField(device, BR3109_ADDR_UPPER_LOWER_LEVEL_BLOCKER_THRESHOLD, regdat, 0x3F3F, 0);
		retVal = BR3109_armMemoryCmd_blk_write( device, BR3109_ADDR_UPPER_LOWER_LEVEL_BLOCKER_THRESHOLD, &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	}

//	if ((rxAgcCtrl->agcPeak.apdLowGainModeHighThresh < apdLowGainModeHighThreshMin)
//	    ||
//	    (rxAgcCtrl->agcPeak.apdLowGainModeHighThresh > apdLowGainModeHighThreshMax)) {
//		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
//						  TAL_ERR_INV_AGC_PKK_LOWGAINMODEHIGHTHRSH_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
//	} else {
//		halError = talSpiWriteByte(device->devHalInfo,
//					   BR3109_ADDR_UPPER_LEVEL_BLOCKER_THRESHOLD2,
//					   rxAgcCtrl->agcPeak.apdLowGainModeHighThresh);
//		retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
//					  TALACT_ERR_RESET_SPI);
//		IF_ERR_RETURN_U32(retVal);
//	}
//	if ((rxAgcCtrl->agcPeak.apdLowGainModeLowThresh < apdLowGainModeLowThreshMin) ||
//	    (rxAgcCtrl->agcPeak.apdLowGainModeLowThresh > apdLowGainModeLowThreshMax)) {
//		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
//						  TAL_ERR_INV_AGC_PKK_LOWGAINTHRSH_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
//	} else {
//		halError = talSpiWriteByte(device->devHalInfo,
//					   BR3109_ADDR_LOWER_LEVEL_BLOCKER_THRESHOLD2,
//					   rxAgcCtrl->agcPeak.apdLowGainModeLowThresh);
//		retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
//					  TALACT_ERR_RESET_SPI);
//		IF_ERR_RETURN_U32(retVal);
//	}

	if (rxAgcCtrl->agcPeak.apdGainStepAttack & ~apdGainStepAttackBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_GAINSTEPATTACK_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} 
	if (rxAgcCtrl->agcPeak.apdGainStepRecovery & ~apdGainStepRecoveryBitMask) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_GAINSTEPRECOVERY_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	/* HB2 Configuration */
	if ((rxAgcCtrl->agcPeak.enableHb2Overload) & ~enableHb2OverloadBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2OVRLD_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if ((rxAgcCtrl->agcPeak.hb2OverloadDurationCnt) & ~ (hb2OverloadDurationCntBitMask)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2OVRLDDURATION_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else if (rxAgcCtrl->agcPeak.hb2OverloadThreshCnt & ~hb2OverloadThreshCntBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2OVRLDTHRSHCNT_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	regdat = rxAgcCtrl->agcPeak.apdGainStepAttack
			| (rxAgcCtrl->agcPeak.apdGainStepRecovery << 8)
			| (rxAgcCtrl->agcPeak.enableHb2Overload << 15)
			// | ((uint32_t)pow(2,rxAgcCtrl->agcPeak.hb2OverloadDurationCnt) << 16)
			| ((uint32_t)(2<<rxAgcCtrl->agcPeak.hb2OverloadDurationCnt) << 16)
			| (rxAgcCtrl->agcPeak.hb2OverloadThreshCnt << 24);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_AGC_GAIN_STEP_DEC_OVERLOAD_CONFIG, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	regdat = rxAgcCtrl->agcPeak.hb2HighThresh | (rxAgcCtrl->agcPeak.hb2UnderRangeLowThresh << 16);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_DEC_OVERLOAD_UPPER_UNDER_RANGE_LOW_THRESHOLD, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	regdat = rxAgcCtrl->agcPeak.hb2UnderRangeMidThresh | (rxAgcCtrl->agcPeak.hb2UnderRangeHighThresh << 16);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_DEC_UNDERRANGE_THRESHOLD, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	regdat = rxAgcCtrl->agcPeak.hb2UpperThreshPeakExceededCnt | (rxAgcCtrl->agcPeak.hb2LowerThreshPeakExceededCnt << 8);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_AGC_HB2_THRESH_EXCEEDED_CNT, regdat, 0xFFFF, 0);
	IF_ERR_RETURN_U32(retVal);

	if (rxAgcCtrl->agcPeak.hb2GainStepHighRecovery & ~hb2GainStepHighRecoveryBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2GAINSTEPRECOVERY_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (rxAgcCtrl->agcPeak.hb2GainStepLowRecovery & ~hb2GainStepLowRecoveryBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2GAINSTEP0RECOVERY_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (rxAgcCtrl->agcPeak.hb2GainStepMidRecovery & ~hb2GainStepMidRecoveryBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2GAINSTEP1RECOVERY_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (rxAgcCtrl->agcPeak.hb2GainStepAttack & ~hb2GainStepAttackBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2GAINSTEPATTACK_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} 
	regdat = rxAgcCtrl->agcPeak.hb2GainStepHighRecovery | (rxAgcCtrl->agcPeak.hb2GainStepLowRecovery << 8)
			| (rxAgcCtrl->agcPeak.hb2GainStepMidRecovery << 16) | (rxAgcCtrl->agcPeak.hb2GainStepAttack << 24);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_AGC_HB2_GAIN_STEP, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	if ((rxAgcCtrl->agcPeak.hb2OverloadPowerMode) & ~hb2OverloadPowerModeBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2OVRLDPWRMODE_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} 

	if ((rxAgcCtrl->agcPeak.hb2OvrgSel) & ~hb2OvrgSelBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2OVRLDSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} 
	if (rxAgcCtrl->agcPeak.hb2ThreshConfig & ~hb2ThreshConfigBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_HB2THRSHCFG_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	/* Because this field was added after production, preserve the past behavior if 0 is passed by setting to default */
	if (rxAgcCtrl->agcPeak.hb2UnderRangeLowThreshExceededCnt == 0) {
		rxAgcCtrl->agcPeak.hb2UnderRangeLowThreshExceededCnt = 3;
	}

	/* Because this field was added after production, preserve the past behavior if 0 is passed by setting to default */
	if (rxAgcCtrl->agcPeak.hb2UnderRangeMidThreshExceededCnt == 0) {
		rxAgcCtrl->agcPeak.hb2UnderRangeMidThreshExceededCnt = 3;
	}
	
	if (&rxAgcCtrl->agcPower == NULL) { /* Check for null power data structure pointer */
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_STRUCT_INIT, retVal, TALACT_ERR_CHECK_PARAM);
	}else if (rxAgcCtrl->agcPower.powerEnableMeasurement & ~powerEnableMeasurementBitMask) { /* Check for null power data structure pointer */
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_STRUCT_INIT, retVal, TALACT_ERR_CHECK_PARAM);
	}
	regdat = rxAgcCtrl->agcPeak.hb2OverloadPowerMode
			| (rxAgcCtrl->agcPeak.hb2OvrgSel << 1)
			| (rxAgcCtrl->agcPeak.hb2ThreshConfig << 2)
			| (rxAgcCtrl->agcPeak.hb2UnderRangeMidThreshExceededCnt << 8)
			| (rxAgcCtrl->agcPeak.hb2UnderRangeLowThreshExceededCnt << 16)
			| ((rxAgcCtrl->agcPower.powerEnableMeasurement > 0 ? 1 : 0) << 24)
			| ((rxAgcCtrl->agcPower.powerUseRfirOut > 0 ? 0:(rxAgcCtrl->agcPower.powerUseBBDC2 > 0? 0x3:0x3))<<28);//默认dc2 ifconv

	halError = BR3109_ArmWriteField(device, BR3109_ADDR_AGC_HB2_OVERLOAD_POWER, regdat, 0x1FFFF0F, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);
	
	// regdat = (uint32_t)pow(10, rxAgcCtrl->agcPower.underRangeHighPowerThresh/20);
	regdat = (uint32_t)(myintpowbase10( rxAgcCtrl->agcPower.underRangeHighPowerThresh/20, rxAgcCtrl->agcPower.underRangeHighPowerThresh%20)/10000);
	if (regdat & ~underRangeHighPowerThreshBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_LWR0THRSH_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_AGC_LOCK_LEVEL_THRESHOLDS, &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	}

	// regdat = (uint32_t)pow(10, rxAgcCtrl->agcPower.underRangeLowPowerThresh/20);
	regdat = (uint32_t)(myintpowbase10( rxAgcCtrl->agcPower.underRangeLowPowerThresh/20, rxAgcCtrl->agcPower.underRangeLowPowerThresh%20)/10000);
	if (regdat & ~underRangeLowPowerThreshBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_INV_AGC_PWR_LWR1THRSH_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_POWER_THRESHOLDS, &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	}

	if (rxAgcCtrl->agcPower.underRangeHighPowerGainStepRecovery & ~underRangeHighPowerGainStepRecoveryBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_LWR0PWRGAINSTEP_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}else if (rxAgcCtrl->agcPower.underRangeLowPowerGainStepRecovery & ~underRangeLowPowerGainStepRecoveryBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_LWR1PWRGAINSTEP_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	// regdat = (uint32_t)pow(10, rxAgcCtrl->agcPower.powerMeasurementDuration/20);
	regdat = (uint32_t)(myintpowbase10( rxAgcCtrl->agcPower.powerMeasurementDuration/20, rxAgcCtrl->agcPower.powerMeasurementDuration%20)/10000);
	if (regdat & ~powerMeasurementDurationBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_MSR_DURATION_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} 
	regdat = regdat << 10;	//powerMeasurementDurationBitMask<<10
	regdat |= rxAgcCtrl->agcPower.underRangeHighPowerGainStepRecovery | (rxAgcCtrl->agcPower.underRangeLowPowerGainStepRecovery << 5);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_DEC_POWER, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	if(rxAgcCtrl->agcPower.rx1TddPowerMeasDuration & ~0xFFFF){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_MSR_DURATION_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}else if(rxAgcCtrl->agcPower.rx1TddPowerMeasDelay & ~0xFFFF){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_MSR_DURATION_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	regdat = rxAgcCtrl->agcPower.rx1TddPowerMeasDuration | (rxAgcCtrl->agcPower.rx1TddPowerMeasDelay << 16);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_RX1_UL_SIG_POWER_MEAS_DURATION_DELAY, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	
	if(rxAgcCtrl->agcPower.rx2TddPowerMeasDuration & ~0xFFFF){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_MSR_DURATION_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}else if(rxAgcCtrl->agcPower.rx2TddPowerMeasDelay & ~0xFFFF){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_MSR_DURATION_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	regdat = rxAgcCtrl->agcPower.rx2TddPowerMeasDuration | (rxAgcCtrl->agcPower.rx2TddPowerMeasDelay << 16);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_RX2_UL_SIG_POWER_MEAS_DURATION_DELAY, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);	

	// regdat = (uint32_t)pow(10, rxAgcCtrl->agcPower.upper0PowerThresh/20);
	regdat = (uint32_t)(myintpowbase10( rxAgcCtrl->agcPower.upper0PowerThresh/20, rxAgcCtrl->agcPower.upper0PowerThresh%20)/10000);
	if (regdat & ~upper0PowerThreshBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_UPPWR0THRSH_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_AGC_POWER_OVER_RANGE_HIGH_THRESHOLD, &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	}

	// regdat = (uint32_t)pow(10,rxAgcCtrl->agcPower.upper1PowerThresh/20);
	regdat = (uint32_t)(myintpowbase10( rxAgcCtrl->agcPower.upper1PowerThresh/20, rxAgcCtrl->agcPower.upper1PowerThresh%20)/10000);
	if (regdat & ~upper1PowerThreshBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_UPPWR1THRSH_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_AGC_POWER_OVER_RANGE_LOW_THRESHOLD, &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	}

	// regdat = ((uint32_t)pow(10,rxAgcCtrl->agcPower.powerLogShift/20));
	regdat = (uint32_t)(myintpowbase10( rxAgcCtrl->agcPower.powerLogShift/20, rxAgcCtrl->agcPower.powerLogShift%20)/10000);
	if (regdat & ~powerLogShiftBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_LOGSHIFT_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} 
	/* Because this field was added after production, preserve the past behavior if 0 is passed by setting to default */
	if (rxAgcCtrl->agcPower.overRangeLowPowerGainStepAttack == 0) {
		rxAgcCtrl->agcPower.overRangeLowPowerGainStepAttack = 4;
	}
	if ((rxAgcCtrl->agcPower.overRangeLowPowerGainStepAttack) & ~overRangeLowPowerGainStepAttackBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_UPPWR0GAINSTEP_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	} 
	/* Because this field was added after production, preserve the past behavior if 0 is passed by setting to default */
	if (rxAgcCtrl->agcPower.overRangeHighPowerGainStepAttack == 0) {
		rxAgcCtrl->agcPower.overRangeHighPowerGainStepAttack = 4;
	}
	if ((rxAgcCtrl->agcPower.overRangeHighPowerGainStepAttack) & ~overRangeHighPowerGainStepAttackBitMask) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_UPPWR1GAINSTEP_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}		
	regdat |= (rxAgcCtrl->agcPower.overRangeHighPowerGainStepAttack << 8)
			| (rxAgcCtrl->agcPower.overRangeLowPowerGainStepAttack << 16);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_AGC_POWER_UPPER_LOW_SHIFT_CONFIG, regdat, 0x1F1F0F, 0);
	return (uint32_t)retVal;
}

uint32_t BR3109_getAgcCtrlRegisters(br3109Device_t *device, br3109AgcCfg_t *agcCtrl)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;
	uint32_t agcClock_Hz = 0;
	uint32_t agcGainUpdateCounter = 0;


	static const uint8_t agcPeakWaitTimeMask = 0x3F;
	// static const uint8_t agcGainUpdateCounterMask = 0x3F;
	static const uint8_t agcLowThreshPreventGainMask = 0x01;
	static const uint8_t agcChangeGainIfThreshHighMask = 0x3;
	static const uint8_t agcPeakThreshGainControlModeMask = 0x01;
	static const uint8_t agcResetOnRxonMask = 0x01;
	static const uint8_t agcEnableSyncPulseForGainCounterMask = 0x01;
	static const uint8_t agcEnableFastRecoveryLoopBitMask = 0x01;
	// static const uint8_t ip3OverRangeThreshMask = 0x3F;
	// static const uint8_t agcClock_HzBitMask = 0x03;
	static const uint8_t agcRx1AttackDelayBitMask = 0x3F;
	static const uint8_t agcRx2AttackDelayBitMask = 0x3F;
	static const uint8_t agcSlowLoopSettlingDelayBitMask = 0xFF;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getAgcCtrlRegisters()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* checking for valid br3109AgcCfg_t device->rx->rxAgcCtrl pointer to determine if it has been initialized */
	if (agcCtrl == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_STRUCT_INIT, retVal, TALACT_ERR_CHECK_PARAM);
	}

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_CONFIG1, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcCtrl->agcPeakWaitTime = regdat & agcPeakWaitTimeMask;
	
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_MAX_MIN_GAIN_INDEX_RX1, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcCtrl->agcRx1MaxGainIndex = regdat & 0xFF;
	agcCtrl->agcRx1MinGainIndex = (regdat >> 8) & 0xFF;
	
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_MAX_MIN_GAIN_INDEX_RX2, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcCtrl->agcRx2MaxGainIndex = regdat & 0xFF;
	agcCtrl->agcRx2MinGainIndex = (regdat >> 8) & 0xFF;
	
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_GAIN_UPDATE_COUNTER, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcGainUpdateCounter = (regdat >> 8) & 0x3FFFFF;   

	/*Reading back AGC Clock Division Ratio*/

	/*Calculation of AGC clock in Hz*/
	agcClock_Hz = device->devStateInfo.clocks.hsDigClkDiv4or5_Hz;
	if(agcClock_Hz == 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_CLK_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*Calculation of Gain Update Time in uS*/
	agcCtrl->agcGainUpdateCounter_us = (uint32_t)DIV_U64(((uint64_t)agcGainUpdateCounter * 1000000) , agcClock_Hz);


	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_MANUAL_GAIN_INDEX_DELAY_RX1, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcCtrl->agcRx1AttackDelay = regdat & agcRx1AttackDelayBitMask;
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_MANUAL_GAIN_INDEX_DELAY_RX2, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcCtrl->agcRx2AttackDelay = regdat & agcRx2AttackDelayBitMask;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_SLOWLOOP_CONFIG, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcCtrl->agcSlowLoopSettlingDelay = regdat & agcSlowLoopSettlingDelayBitMask;

	agcCtrl->agcLowThreshPreventGain = ((regdat >> 8) & agcLowThreshPreventGainMask);
	agcCtrl->agcChangeGainIfThreshHigh =((regdat >> 12) & agcChangeGainIfThreshHighMask);
	agcCtrl->agcPeakThreshGainControlMode = ((regdat >> 16) & agcPeakThreshGainControlModeMask);
	agcCtrl->agcResetOnRxon = ((regdat >> 17) & agcResetOnRxonMask);
	agcCtrl->agcEnableSyncPulseForGainCounter = ((regdat >> 18) & agcEnableSyncPulseForGainCounterMask);
	agcCtrl->agcEnableFastRecoveryLoop = ((regdat >> 19) & agcEnableFastRecoveryLoopBitMask);
	
	return (uint32_t)retVal;
}

uint32_t BR3109_getAgcPeakRegisters(br3109Device_t *device, br3109AgcPeak_t *agcPeak)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;
	// uint8_t agcRegister[2] = {0};
	uint32_t agcClock_Hz = 0;
	uint16_t agcUnderRangeLowInterval = 0;

	static const uint8_t apdHighThreshMask = 0x3F;
	// static const uint8_t apdLowGainModeHighThreshMask = 0x3F;
	static const uint8_t apdLowThreshMask = 0x3F;
	// static const uint8_t apdLowGainModeLowThreshMask = 0x3F;
	static const uint8_t apdGainStepAttackMask = 0x1F;
	static const uint8_t apdGainStepRecoveryMask = 0x1F;
	static const uint8_t enableHb2OverloadMask = 0x01;
	static const uint8_t hb2OverloadDurationCntMask = 0x07;
	static const uint8_t hb2OverloadThreshCntMask = 0x0F;
	static const uint8_t hb2GainStepHighRecoveryMask = 0x1F;
	static const uint8_t hb2GainStepLowRecoveryMask = 0x1F;
	static const uint8_t hb2GainStepMidRecoveryMask = 0x1F;
	static const uint8_t hb2GainStepAttackMask = 0x1F;
	static const uint8_t hb2OverloadPowerModeMask = 0x01;
	static const uint8_t hb2OvrgSelMask = 0x01;
	static const uint8_t hb2ThreshConfigMask = 0x03;
	// static const uint8_t agcClock_HzBitMask = 0x03;
	static const uint8_t agcUnderRangeMidIntervalMask = 0x3F;
	static const uint8_t agcUnderRangeHighIntervalMask = 0x3F;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getAgcPeakRegisters()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* checking for valid br3109AgcCfg_t device->rx->rxAgcCtrl pointer to determine if it has been initialized */
	if (agcPeak == NULL){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PKK_STRUCT_INIT, retVal, TALACT_ERR_CHECK_PARAM);
	}
	
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_UNDERRANGE0, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcUnderRangeLowInterval = regdat & 0xFFFF;
	agcPeak->agcUnderRangeMidInterval = (regdat >> 16) & agcUnderRangeMidIntervalMask;
	agcPeak->agcUnderRangeHighInterval = (regdat >> 24) & agcUnderRangeHighIntervalMask;


	/*Calculation of AGC clock in Hz*/
		agcClock_Hz = device->devStateInfo.clocks.hsDigClkDiv4or5_Hz;

	if (agcClock_Hz == 0){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_CLK_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*Calculation of Gain Update Time in ns*/
	agcPeak->agcUnderRangeLowInterval_ns = (uint32_t)DIV_U64(((uint64_t)agcUnderRangeLowInterval * 1000000000) , agcClock_Hz);

	
	retVal = BR3109_armMemoryCmd_blk_read( device, BR3109_ADDR_UPPER_LOWER_LEVEL_BLOCKER_THRESHOLD, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPeak->apdHighThresh = (regdat) & apdHighThreshMask;
	agcPeak->apdLowThresh = (regdat >> 8) & apdLowThreshMask;
	agcPeak->apdUpperThreshPeakExceededCnt = (regdat >> 16) & 0xFF;
	agcPeak->apdLowerThreshPeakExceededCnt = (regdat >> 24) & 0xFF;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_GAIN_STEP_DEC_OVERLOAD_CONFIG, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPeak->apdGainStepAttack = regdat & apdGainStepAttackMask;
	agcPeak->apdGainStepRecovery = (regdat >> 8) & apdGainStepRecoveryMask;
	agcPeak->enableHb2Overload = ((regdat >> 15) & enableHb2OverloadMask);
	// agcPeak->hb2OverloadDurationCnt = (((uint32_t)(log10((regdat >> 16) & 0x7F) / log10(2))) & hb2OverloadDurationCntMask); //求解2 n次根转换为log10的除法
	agcPeak->hb2OverloadDurationCnt = (uint32_t)((regdat >> 16) & hb2OverloadDurationCntMask); //求解2 n次根转换为log10的除法
	agcPeak->hb2OverloadThreshCnt = (regdat >> 24) & hb2OverloadThreshCntMask;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_DEC_OVERLOAD_UPPER_UNDER_RANGE_LOW_THRESHOLD, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPeak->hb2HighThresh = regdat & 0x3FFF;
	agcPeak->hb2UnderRangeLowThresh = (regdat >> 16) & 0x3FFF;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_DEC_UNDERRANGE_THRESHOLD, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPeak->hb2UnderRangeMidThresh = regdat & 0x3FFF;
	agcPeak->hb2UnderRangeHighThresh = (regdat >> 16) & 0x3FFF;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_HB2_THRESH_EXCEEDED_CNT, &regdat,1);
	IF_ERR_RETURN_U32(retVal);
	agcPeak->hb2UpperThreshPeakExceededCnt = regdat & 0xFF;
	agcPeak->hb2LowerThreshPeakExceededCnt = (regdat >> 8) & 0xFF;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_HB2_GAIN_STEP, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPeak->hb2GainStepHighRecovery = regdat & hb2GainStepHighRecoveryMask;
	agcPeak->hb2GainStepLowRecovery = (regdat >> 8) & hb2GainStepLowRecoveryMask;
	agcPeak->hb2GainStepMidRecovery = (regdat >> 16) & hb2GainStepMidRecoveryMask;
	agcPeak->hb2GainStepAttack =  (regdat >> 24) & hb2GainStepAttackMask;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_HB2_OVERLOAD_POWER, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPeak->hb2OverloadPowerMode = (regdat & hb2OverloadPowerModeMask);
	agcPeak->hb2OvrgSel = (regdat >> 1) & hb2OvrgSelMask;
	agcPeak->hb2ThreshConfig = (regdat >> 2) & hb2ThreshConfigMask;
	agcPeak->hb2UnderRangeMidThreshExceededCnt = (regdat >> 8) & 0xFF;
	agcPeak->hb2UnderRangeLowThreshExceededCnt = (regdat >> 16) & 0xFF;

	return (uint32_t)retVal;
}

uint32_t BR3109_getAgcPowerRegisters(br3109Device_t *device, br3109AgcPower_t *agcPower)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;
	uint8_t powerInputSel = 0;

	static const uint8_t powerEnableMeasurementMask = 0x01;
	// static const uint8_t powerUseRfirOutMask = 0x01;
	// static const uint8_t powerUseBBDC2Mask = 0x01;
	static const uint8_t underRangeHighPowerThreshMask = 0x7F;
	static const uint8_t underRangeLowPowerThreshMask = 0x1F;
	static const uint8_t underRangeHighPowerGainStepRecoveryMask = 0x1F;
	static const uint8_t underRangeLowPowerGainStepRecoveryMask = 0x1F;
	static const uint8_t powerMeasurementDurationMask = 0x1F;
	static const uint8_t upper0PowerThreshMask = 0x7F;
	static const uint8_t upper1PowerThreshMask = 0x0F;
	static const uint8_t powerLogShiftMask = 0x01;
	static const uint8_t overRangeLowPowerGainStepAttackBitMask = 0x1F;
	static const uint8_t overRangeHighPowerGainStepAttackBitMask = 0x1F;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getAgcPowerRegisters()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* checking for valid br3109AgcCfg_t device->rx->rxAgcCtrl pointer to determine if it has been initialized */
	if (agcPower == NULL){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_PWR_STRUCT_INIT, retVal, TALACT_ERR_CHECK_PARAM);
	}
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_HB2_OVERLOAD_POWER, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPower->powerEnableMeasurement = (regdat >> 24) & powerEnableMeasurementMask;	

	powerInputSel = (regdat >> 28) & 0x3;
	agcPower->powerUseRfirOut = (powerInputSel == 0) ? 1 : 0;
    agcPower->powerUseBBDC2 = (powerInputSel == 3) ? 1 : 0;
	
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_POWER_UPPER_LOW_SHIFT_CONFIG, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);	
	// agcPower->powerLogShift = ((uint32_t)(20 * log10(regdat & 0xF))) & powerLogShiftMask;
	agcPower->powerLogShift = ((uint32_t)(mymdb20log10(regdat & 0xF))) & powerLogShiftMask;
	agcPower->overRangeHighPowerGainStepAttack = (regdat >> 8) & overRangeHighPowerGainStepAttackBitMask;
	agcPower->overRangeLowPowerGainStepAttack = (regdat >> 16) & overRangeLowPowerGainStepAttackBitMask;
		
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_LOCK_LEVEL_THRESHOLDS, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	// agcPower->underRangeHighPowerThresh = ((uint32_t)(20 * log10(regdat & 0x0FFFFFFF))) & underRangeHighPowerThreshMask;
	agcPower->underRangeHighPowerThresh = ((uint32_t)(mymdb20log10(regdat & 0x0FFFFFFF))) & underRangeHighPowerThreshMask;
	
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_POWER_THRESHOLDS, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	// agcPower->underRangeLowPowerThresh = ((uint32_t)(20 * log10(regdat & 0x0FFFFFFF))) & underRangeLowPowerThreshMask;
	agcPower->underRangeLowPowerThresh = ((uint32_t)(mymdb20log10(regdat & 0x0FFFFFFF))) & underRangeLowPowerThreshMask;
		
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_DEC_POWER, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPower->underRangeHighPowerGainStepRecovery = regdat & underRangeHighPowerGainStepRecoveryMask;
	agcPower->underRangeLowPowerGainStepRecovery = (regdat >> 5) & underRangeLowPowerGainStepRecoveryMask;
	// agcPower->powerMeasurementDuration = ((uint32_t)(20 * log10((regdat >> 10) & 0x1FFFFF))) & powerMeasurementDurationMask;
	agcPower->powerMeasurementDuration = ((uint32_t)(mymdb20log10((regdat >> 10) & 0x1FFFFF))) & powerMeasurementDurationMask;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_RX1_UL_SIG_POWER_MEAS_DURATION_DELAY, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPower->rx1TddPowerMeasDuration = (regdat & 0xFFFF);
	agcPower->rx1TddPowerMeasDelay = (regdat >> 16) & 0xFFFF;
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_RX2_UL_SIG_POWER_MEAS_DURATION_DELAY, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	agcPower->rx2TddPowerMeasDuration = regdat & 0xFFFF;
	agcPower->rx2TddPowerMeasDelay = (regdat >> 16) & 0xFFFF;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_POWER_OVER_RANGE_HIGH_THRESHOLD, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	// agcPower->upper0PowerThresh = ((uint32_t)(20 * log10(regdat & 0x0FFFFFFF))) & upper0PowerThreshMask;
	agcPower->upper0PowerThresh = ((uint32_t)(mymdb20log10(regdat & 0x0FFFFFFF))) & upper0PowerThreshMask;

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_AGC_POWER_OVER_RANGE_LOW_THRESHOLD, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	// agcPower->upper1PowerThresh = ((uint32_t)(20 * log10(regdat & 0x0FFFFFFF))) & upper1PowerThreshMask;
	agcPower->upper1PowerThresh = ((uint32_t)(mymdb20log10(regdat & 0x0FFFFFFF))) & upper1PowerThreshMask;

	return (uint32_t)retVal;
}

#if 0
static talRecoveryActions_t talSetupDualBandRx1Agc(br3109Device_t *device,
		br3109AgcDualBandCfg_t *rxAgcCtrlDualBand)
{
	uint8_t registerValue = 0;
	uint16_t currGpio3p3Oe = 0;
	uint16_t usedGpio3p3pins = 0;
	uint16_t gpio3p3FreeMask = 0;
	uint16_t gpio3p3UsedMask = 0;
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

	const uint8_t RX1_DUALBAND_EXT_LNA_SOURCE_CONTROL = 0x05;

	/* range check the gain against the max and min expected values */
	if ((rxAgcCtrlDualBand->agcRxDualbandExtTableUpperIndex >
	     device->devStateInfo.gainIndexes.rx1MaxGainIndex)
	    || (rxAgcCtrlDualBand->agcRxDualbandExtTableLowerIndex <
		device->devStateInfo.gainIndexes.rx1MinGainIndex)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_SETUPDUALBANDRXAGC_GAIN_OUT_OF_RANGE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check current configuration of GPIO 3.3s */
	halError = talSpiReadByte(device->devHalInfo,
				  BR3109_ADDR_GPIO_3P3V_LOWER_BYTE_SOURCE_CONTROL, &registerValue);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	retVal = (talRecoveryActions_t)BR3109_getGpio3v3Oe(device, &currGpio3p3Oe);
	IF_ERR_RETURN(retVal);

	/* Free GPIO3.3[3:0] if used */
	if (((registerValue & 0x0F) == RX1_DUALBAND_EXT_LNA_SOURCE_CONTROL) &&
	    ((currGpio3p3Oe & 0x0F) == 0x0F)) {
		gpio3p3FreeMask = 0x0F;
		usedGpio3p3pins = device->devStateInfo.usedGpio3p3pins & ~gpio3p3FreeMask;
	}

	/* Update GPIO3p3 source control and used masks if enabled*/
	if (rxAgcCtrlDualBand->dualBandGpioEnable > 0) {
		/* check if GPIO's are available for the new configuration*/
		gpio3p3UsedMask = 0x0F;
		if ((usedGpio3p3pins & gpio3p3UsedMask) != 0) {
			return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						TAL_ERR_SETUPDUALBANDRX1AGC_GPIO3P3_IN_USE, retVal, TALACT_ERR_CHECK_PARAM);
		}

		usedGpio3p3pins |= gpio3p3UsedMask;
	}

	/* Set the source control */
	registerValue = (registerValue & ~0x0F ) | ((
				rxAgcCtrlDualBand->dualBandGpioEnable > 0) ?
			RX1_DUALBAND_EXT_LNA_SOURCE_CONTROL : 0x00);
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_GPIO_3P3V_LOWER_BYTE_SOURCE_CONTROL, registerValue);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Set GPIO input enables */
	/* - Set OE for all freed GPIO3p3's to 0 (input) */
	/* - Set OE for all used GPIO3p3's to 1 (output) */
	retVal = (talRecoveryActions_t)BR3109_setGpio3v3Oe(device, gpio3p3UsedMask,
			(gpio3p3UsedMask | gpio3p3FreeMask));
	IF_ERR_RETURN(retVal);

	/* Update used and freed GPIO's */
	device->devStateInfo.usedGpio3p3pins = usedGpio3p3pins;

	/* Set upper LNA index */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_RX1_AGC_DUALBAND_INDEX_X,
				   rxAgcCtrlDualBand->agcRxDualbandExtTableUpperIndex );
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Set lower LNA index */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_RX1_AGC_DUALBAND_INDEX_Y,
				   rxAgcCtrlDualBand->agcRxDualbandExtTableLowerIndex );
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Enable Dec power for the channel. Also clear bitfield to ensure that Dec power calculation is done before compensation */
	registerValue = (rxAgcCtrlDualBand->dualBandGpioEnable > 0) ? 0x03 : 0x00;
	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_DDC_DEC_POWER_CONFIG, registerValue, 0x33, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);

	return retVal;
}

static talRecoveryActions_t talSetupDualBandRx2Agc(br3109Device_t *device,
		br3109AgcDualBandCfg_t *rxAgcCtrlDualBand)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t registerValue = 0;
	uint16_t currGpio3p3Oe = 0;
	uint16_t usedGpio3p3pins = 0;
	uint16_t gpio3p3FreeMask = 0;
	uint16_t gpio3p3UsedMask = 0;

	const uint8_t RX2_DUALBAND_EXT_LNA_SOURCE_CONTROL = 0x05;

	/* range check the gain against the max and min expected values */
	if ((rxAgcCtrlDualBand->agcRxDualbandExtTableUpperIndex >
	     device->devStateInfo.gainIndexes.rx2MaxGainIndex)
	    || (rxAgcCtrlDualBand->agcRxDualbandExtTableLowerIndex <
		device->devStateInfo.gainIndexes.rx2MinGainIndex)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_SETUPDUALBANDRXAGC_GAIN_OUT_OF_RANGE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check current configuration of GPIO 3.3s */
	halError = talSpiReadByte(device->devHalInfo,
				  BR3109_ADDR_GPIO_3P3V_LOWER_BYTE_SOURCE_CONTROL, &registerValue);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	retVal = (talRecoveryActions_t)BR3109_getGpio3v3Oe(device, &currGpio3p3Oe);
	IF_ERR_RETURN(retVal);

	/* Free GPIO3.3[7:4] if used */
	if ((((registerValue & 0xF0) >> 4) == RX2_DUALBAND_EXT_LNA_SOURCE_CONTROL) &&
	    ((currGpio3p3Oe & 0xF0) == 0xF0)) {
		gpio3p3FreeMask = 0xF0;
		usedGpio3p3pins = device->devStateInfo.usedGpio3p3pins & ~gpio3p3FreeMask;
	}

	/* Update GPIO3p3 source control and used masks if enabled*/
	if (rxAgcCtrlDualBand->dualBandGpioEnable > 0) {
		/* check if GPIO's are available for the new configuration*/
		gpio3p3UsedMask = 0xF0;
		if ((usedGpio3p3pins & gpio3p3UsedMask) != 0) {
			return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						TAL_ERR_SETUPDUALBANDRX2AGC_GPIO3P3_IN_USE, retVal, TALACT_ERR_CHECK_PARAM);
		}

		usedGpio3p3pins |= gpio3p3UsedMask;
	}

	/* Set the source control */
	registerValue = (registerValue & ~0xF0 ) | (((
				rxAgcCtrlDualBand->dualBandGpioEnable > 0) ?
			RX2_DUALBAND_EXT_LNA_SOURCE_CONTROL : 0x00) << 4);
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_GPIO_3P3V_LOWER_BYTE_SOURCE_CONTROL, registerValue);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Set GPIO input enables */
	/* - Set OE for all freed GPIO3p3's to 0 (input) */
	/* - Set OE for all used GPIO3p3's to 1 (output) */
	retVal = (talRecoveryActions_t)BR3109_setGpio3v3Oe(device, gpio3p3UsedMask,
			(gpio3p3UsedMask | gpio3p3FreeMask));
	IF_ERR_RETURN(retVal);

	/* Update used and freed GPIO's */
	device->devStateInfo.usedGpio3p3pins = usedGpio3p3pins;

	/* Set upper LNA index */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_RX2_AGC_DUALBAND_INDEX_X,
				   rxAgcCtrlDualBand->agcRxDualbandExtTableUpperIndex );
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Set lower LNA index */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_RX2_AGC_DUALBAND_INDEX_Y,
				   rxAgcCtrlDualBand->agcRxDualbandExtTableLowerIndex );
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Enable Dec power for the channel. Also clear bitfield to ensure that Dec power calculation is done before compensation */
	registerValue = (rxAgcCtrlDualBand->dualBandGpioEnable > 0) ? 0x0C : 0x00;
	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_DDC_DEC_POWER_CONFIG, registerValue, 0xCC, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);

	return retVal;
}

uint32_t BR3109_setupDualBandRxAgc( br3109Device_t *device,
				    br3109RxChannels_t rxChannel, br3109AgcDualBandCfg_t *rxAgcCtrlDualBand)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setupDualBandRxAgc()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Check: Channel */
	if (rxChannel == TAL_RXOFF) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETUPDUALBANDRXAGC_INV_CHANNEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check: Null pointer */
	if ( rxAgcCtrlDualBand == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETUPDUALBANDRXAGC_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check: Power margin */
	if (rxAgcCtrlDualBand->agcDualbandPwrMargin > 0x1F) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETUPDUALBANDRXAGC_INV_PWRMARGIN, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check: Dec power measurement duration */
	if (rxAgcCtrlDualBand->decPowerDdcMeasurementDuration > 0x1F) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETUPDUALBANDRXAGC_INV_DECPWR, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* range check the gain table indices upper > lower */
	if ((rxAgcCtrlDualBand->agcRxDualbandExtTableUpperIndex <=
	     rxAgcCtrlDualBand->agcRxDualbandExtTableLowerIndex)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETUPDUALBANDRXAGC_GAIN_RANGE_MISMATCH, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Write Rx1 specific data */
	if ((rxChannel == TAL_RX1) ||
	    (rxChannel == TAL_RX1RX2)) {
		retVal = talSetupDualBandRx1Agc(device, rxAgcCtrlDualBand);
	}

	/* Write Rx2 specific data */
	if ((rxChannel == TAL_RX2) ||
	    (rxChannel == TAL_RX1RX2)) {
		retVal = talSetupDualBandRx2Agc(device, rxAgcCtrlDualBand);
	}

	/* Enable dualband mode */
	halError = talSpiWriteByte(device->devHalInfo, BR3109_ADDR_SLOWLOOP_CONFIG2,
				   (uint8_t)((rxAgcCtrlDualBand->agcDualBandEnable > 0) ? 1 : 0));
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Power margin */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_AGC_DUALBAND_TOTAL_PWR_MARGIN,
				   rxAgcCtrlDualBand->agcDualbandPwrMargin);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Band power margin */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_AGC_DUALBAND_BAND_PWR_MARGIN,
				   rxAgcCtrlDualBand->agcDualbandLnaStep);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* High LNA threshold */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_AGC_DUALBAND_HIGH_LNA_THRESHOLD,
				   rxAgcCtrlDualBand->agcDualbandHighLnaThreshold);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Low LNA threshold */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_AGC_DUALBAND_LOW_LNA_THRESHOLD,
				   rxAgcCtrlDualBand->agcDualbandLowLnaThreshold);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Dec power measurement duration */
	/* Also enable measurement control through AGC, and improved dynamic range of measured power */
	halError = talSpiWriteByte(device->devHalInfo, BR3109_ADDR_DDC_DEC_POWER_MEAS,
				   ((rxAgcCtrlDualBand->decPowerDdcMeasurementDuration & 0x1F) | 0x60));
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);

	return (uint32_t)retVal;
}

uint32_t BR3109_getDualBandLnaControls(br3109Device_t *device,
				       br3109RxChannels_t rxChannel,
				       br3109DualBandLnaControls_t *rxDualBandLnaControls)
{
	uint16_t lowerBandAddr = 0;
	uint16_t upperBandAddr = 0;
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getDualBandLnaControls()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Read current value of Gain inc/dec and enable */
	switch (rxChannel) {
	case TAL_RX1:
		lowerBandAddr = BR3109_ADDR_GAIN_TABLE_DATA_OUTPUT_DUALBAND_CH1_BAND_A;
		upperBandAddr = BR3109_ADDR_GAIN_TABLE_DATA_OUTPUT_DUALBAND_CH1_BAND_B;
		break;

	case TAL_RX2:
		lowerBandAddr = BR3109_ADDR_GAIN_TABLE_DATA_OUTPUT_DUALBAND_CH2_BAND_A;
		upperBandAddr = BR3109_ADDR_GAIN_TABLE_DATA_OUTPUT_DUALBAND_CH2_BAND_B;
		break;

	default: {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETDUALBANDLNA_INV_CHANNEL, retVal, TALACT_ERR_CHECK_PARAM);
	}
	}

	/*Check passed pointers for NULL */
	if (rxDualBandLnaControls == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETDUALBANDLNA_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Disable gain table read - Otherwise, the readback will read a forced index */
	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_GAIN_TABLE_CONFIGURATION, 0x0, 0x04, 2);
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Read control word for the current LNA index */
	halError = brSpiReadField(device->devHalInfo, lowerBandAddr,
				   &rxDualBandLnaControls->rxLowerBandLnaControl, 0x03, 0);
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	halError = brSpiReadField(device->devHalInfo, upperBandAddr,
				   &rxDualBandLnaControls->rxUpperBandLnaControl, 0x03, 0);
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	return (uint32_t)retVal;
}
#endif
uint32_t BR3109_setRxAgcMinMaxGainIndex(br3109Device_t *device, br3109RxChannels_t rxChannel, uint8_t maxGainIndex, uint8_t minGainIndex)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setRxAgcMinMaxGainIndex()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* Ensure that Max Gain Index is always greater than Min Gain Index */
	if(minGainIndex >= maxGainIndex){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_MIN_GAIN_GRT_THAN_MAX_GAIN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if((rxChannel == TAL_RX1) || (rxChannel == TAL_RX1RX2)){
		/*Ensure that requested min and max gain indexes are within the range supported by the current chip config*/
		if ((maxGainIndex > device->devStateInfo.rxGainCtrl.rx1MaxGainIndex) ||
		    (minGainIndex < device->devStateInfo.rxGainCtrl.rx1MinGainIndex)){
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_MIN_MAX_GAIN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}
		/* Writing Rx1 max and min gain indices to the SPI registers*/
		regdat = maxGainIndex | (minGainIndex << 8);
		halError = BR3109_ArmWriteField(device, BR3109_ADDR_MAX_MIN_GAIN_INDEX_RX1, regdat, 0xFFFF, 0);
		IF_ERR_RETURN_U32(retVal);
	}

	if((rxChannel == TAL_RX2) || (rxChannel == TAL_RX1RX2)){
		/*Ensure that requested min and max gain indexes are within the range supported by the current chip config*/
		if ((maxGainIndex > device->devStateInfo.rxGainCtrl.rx2MaxGainIndex) ||
		    (minGainIndex < device->devStateInfo.rxGainCtrl.rx2MinGainIndex)){
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_AGC_RX_MIN_MAX_GAIN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		/* Writing Rx2 max and min gain indices to the SPI registers*/
		regdat = maxGainIndex | (minGainIndex << 8);
		halError = BR3109_ArmWriteField(device, BR3109_ADDR_MAX_MIN_GAIN_INDEX_RX2, regdat, 0xFFFF, 0);
		IF_ERR_RETURN_U32(retVal);
	}

	return (uint32_t)retVal;
}

uint32_t BR3109_resetRxAgc(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	static const uint8_t AGC_RESET = 1;
	static const uint8_t AGC_RESET_CLEAR = 0;
	static const uint32_t AGC_RESET_MASK = 0x20000;
	static const uint8_t AGC_START_BIT = 17;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_resetRxAgc()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Soft Reset AGC */
	halError = BR3109_ArmWriteField(device, BR3109_ADDR_SLOWLOOP_CONFIG,
				    AGC_RESET, AGC_RESET_MASK, AGC_START_BIT);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/*Release AGC Reset*/
	halError = brSpiWriteField(device->devHalInfo, BR3109_ADDR_SLOWLOOP_CONFIG,
				    AGC_RESET_CLEAR, AGC_RESET_MASK, AGC_START_BIT);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}
