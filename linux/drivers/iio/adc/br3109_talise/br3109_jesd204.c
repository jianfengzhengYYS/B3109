/**
 * \file br3109_jesd204.c
 * \brief Contains functions to support Br3109 JESD204b data interface
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#include "br3109_jesd204.h"
#include "br3109_reg_addr_macros.h"
#include "br3109_arm_macros.h"
#include "br3109_hal.h"
#include "br3109_user.h"
#include "br3109_error.h"
#include "br3109_arm.h"
#include "br3109.h"

uint32_t BR3109_setupSerializers(br3109Device_t *device, br3109Init_t *init)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	// brHalErr_t halError = BRHAL_OK;
	uint8_t laneInvert = 0;
	uint8_t activeFramersMask =
		0; /* bit per framer to for quick access of valid framers enabled. */
	uint8_t activeLanesEnabled = 0;
	// uint8_t activeLanesPowerDown = 0xF;
	uint8_t enPreEmphasisMask = 0x00; /* bit per lane */
	uint8_t preEmp = 0x00; /* same value for all lanes */
	uint8_t txser_div = 0;
	uint8_t i = 0;
	uint8_t obsRxL = 0;
	uint8_t rxL = 0;
	uint32_t rxLaneRate_kHz = 0;
	uint32_t obsRxLaneRate_kHz = 0;
	uint32_t fasterLaneRate_kHz = 0;
	uint32_t slowerLaneRate_kHz = 0;
	uint8_t txser_div_reg = 0;
	br3109Jesd204bFramerConfig_t *currentFramer = NULL;

	static const uint8_t DUALBANDENABLED = 1;

	/* Determine which framer structures are valid */
	if (device->devStateInfo.profilesValid & RX_PROFILE_VALID) {
		switch(init->rx.framerSel) {
		case TAL_FRAMER_A:
			activeFramersMask |= 0x01;
			activeLanesEnabled |= init->jesd204Settings.framerA.serializerLanesEnabled;
			laneInvert |= ((init->jesd204Settings.framerA.serializerLanesEnabled) &
				       (init->jesd204Settings.serInvertLanePolarity));
			currentFramer = &init->jesd204Settings.framerA;
			break;
		case TAL_FRAMER_B:
			activeFramersMask |= 0x02;
			activeLanesEnabled |= init->jesd204Settings.framerB.serializerLanesEnabled;
			laneInvert |= ((init->jesd204Settings.framerB.serializerLanesEnabled) &
				       (init->jesd204Settings.serInvertLanePolarity));
			currentFramer = &init->jesd204Settings.framerB;
			break;
		case TAL_FRAMER_A_AND_B:
			activeFramersMask |= 0x03;
			activeLanesEnabled |= init->jesd204Settings.framerA.serializerLanesEnabled;
			activeLanesEnabled |= init->jesd204Settings.framerB.serializerLanesEnabled;
			laneInvert |= ((init->jesd204Settings.framerA.serializerLanesEnabled) &
				       (init->jesd204Settings.serInvertLanePolarity));
			laneInvert |= ((init->jesd204Settings.framerB.serializerLanesEnabled) &
				       (init->jesd204Settings.serInvertLanePolarity));
			currentFramer = &init->jesd204Settings.framerA;
			break;
		default:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_RXFRAMER_SEL, retVal, TALACT_ERR_CHECK_PARAM);
		}

		//serAmplitude = init->jesd204Settings.serAmplitude;
		preEmp = init->jesd204Settings.serPreEmphasis;
		enPreEmphasisMask |= (init->jesd204Settings.serPreEmphasis > 0) ?
				     activeLanesEnabled : 0;

		/* TODO: Need error check to make sure that if FRAMER A and B, then make sure each framer has same number of lanes, and M matches */
		for (i = 0; i < 4; i++) {
			rxL += ((currentFramer->serializerLanesEnabled >> i) & 0x01);
		}

		if (rxL > 0) {
			rxLaneRate_kHz = currentFramer->Np * 10 * init->rx.rxProfile.rxOutputRate_kHz *
					 currentFramer->M / (8 * rxL);
			if ((rxLaneRate_kHz != 0) && ((rxLaneRate_kHz < 768000)
						      || (rxLaneRate_kHz > 12288000))) {
				return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								  TAL_ERR_SER_INV_LANERATE_PARM, retVal, TALACT_ERR_CHECK_PARAM);
			}
		}

		if ((rxL != 0) &&
		    (rxL != 1) &&
		    (rxL != 2) &&
		    (rxL != 4)) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_L_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if ((currentFramer->Np != 12) &&
		    (currentFramer->Np != 16) &&
		    (currentFramer->Np != 24)) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_NP_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if ((currentFramer->M == 1)
		    && !(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_INT2_REALIF)
		    && !(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_DEC2_REALIF)
		    && !(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_FILTERONLY_REALIF)
		    && !(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_BYPASS_REALIF)) {
			return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						TAL_ERR_SER_INV_ZIF_TO_RIF_DATA_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if ((currentFramer->M == 8) &&
		    !(device->devStateInfo.rxDualBandEnabled != DUALBANDENABLED)) {
			return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						TAL_ERR_SER_INV_DUALBAND_DATA_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if ((currentFramer->M != 0) &&
		    (currentFramer->M != 1) &&
		    (currentFramer->M != 2) &&
		    (currentFramer->M != 4)) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_M_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if (currentFramer->serializerLanesEnabled > 15) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_LANEEN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}
	}

	if (device->devStateInfo.profilesValid & ORX_PROFILE_VALID) {
		switch(init->obsRx.framerSel) {
		case TAL_FRAMER_A:
			activeFramersMask |= 0x01;
			activeLanesEnabled |= init->jesd204Settings.framerA.serializerLanesEnabled;
			laneInvert |= ((init->jesd204Settings.framerA.serializerLanesEnabled) &
				       (init->jesd204Settings.serInvertLanePolarity));
			currentFramer = &init->jesd204Settings.framerA;
			break;
		case TAL_FRAMER_B:
			activeFramersMask |= 0x02;
			activeLanesEnabled |= init->jesd204Settings.framerB.serializerLanesEnabled;
			laneInvert |= ((init->jesd204Settings.framerB.serializerLanesEnabled) &
				       (init->jesd204Settings.serInvertLanePolarity));
			currentFramer = &init->jesd204Settings.framerB;
			break;
		case TAL_FRAMER_A_AND_B:
			activeFramersMask |= 0x03;
			activeLanesEnabled |= init->jesd204Settings.framerA.serializerLanesEnabled;
			activeLanesEnabled |= init->jesd204Settings.framerB.serializerLanesEnabled;
			laneInvert |= ((init->jesd204Settings.framerA.serializerLanesEnabled) &
				       (init->jesd204Settings.serInvertLanePolarity));
			laneInvert |= ((init->jesd204Settings.framerB.serializerLanesEnabled) &
				       (init->jesd204Settings.serInvertLanePolarity));
			currentFramer = &init->jesd204Settings.framerA;
			break;
		default:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_ORXFRAMER_SEL, retVal, TALACT_ERR_CHECK_PARAM);
		}

		//serAmplitude = init->jesd204Settings.serAmplitude;
		preEmp = init->jesd204Settings.serPreEmphasis;
		enPreEmphasisMask |= (init->jesd204Settings.serPreEmphasis > 0) ?
				     activeLanesEnabled : 0;

		/* TODO: Need error check to make sure that if FRAMER A and B, then make sure each framer has same number of lanes, and M matches */
		for(i = 0; i < 4; i++) {
			obsRxL += ((currentFramer->serializerLanesEnabled >> i) & 0x01);
		}

		if (obsRxL > 0) {
			obsRxLaneRate_kHz = currentFramer->Np * 10 *
					    init->obsRx.orxProfile.orxOutputRate_kHz * currentFramer->M / (8 * obsRxL);
			if ((obsRxLaneRate_kHz != 0) && ((obsRxLaneRate_kHz < 768000)
							 || (obsRxLaneRate_kHz > 12288000))) {
				return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								  TAL_ERR_SER_INV_ORX_LANERATE_PARM, retVal, TALACT_ERR_CHECK_PARAM);
			}
		}

		if ((obsRxL != 0) &&
		    (obsRxL != 1) &&
		    (obsRxL != 2) &&
		    (obsRxL != 4)) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_ORX_L_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if ((currentFramer->Np != 12) &&
		    (currentFramer->Np != 16) &&
		    (currentFramer->Np != 24)) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_ORX_NP_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if ((currentFramer->M != 0) &&
		    (currentFramer->M != 1) &&
		    (currentFramer->M != 2) &&
		    (currentFramer->M != 4)) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_ORX_M_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if (currentFramer->serializerLanesEnabled > 15) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_INV_ORX_LANEEN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}
	}

	if (activeFramersMask == 0x03) {
		/* Verify FramerA and FramerB do not share physical lanes */
		if (init->jesd204Settings.framerA.serializerLanesEnabled &
		    init->jesd204Settings.framerB.serializerLanesEnabled
		   ) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_LANE_CONFLICT_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}
	}

	if (init->jesd204Settings.serAmplitude > 15) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SER_INV_AMP_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (init->jesd204Settings.serPreEmphasis > 4) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SER_INV_PREEMP_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (init->jesd204Settings.serInvertLanePolarity > 15) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SER_INV_LANEPN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Calculate needed JESD204 serializer lane rates */
	if ((rxLaneRate_kHz > 0) &&
	    (obsRxLaneRate_kHz > 0)) {
		fasterLaneRate_kHz = (rxLaneRate_kHz >= obsRxLaneRate_kHz) ?
				     (rxLaneRate_kHz) : (obsRxLaneRate_kHz);
		slowerLaneRate_kHz = (rxLaneRate_kHz >= obsRxLaneRate_kHz) ?
				     (obsRxLaneRate_kHz) : (rxLaneRate_kHz);

		/* Verify that lane rates are integer multiples of each other */
		if (fasterLaneRate_kHz % slowerLaneRate_kHz != 0) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SER_LANE_RATE_CONFLICT_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}
	} else {
		fasterLaneRate_kHz = (rxLaneRate_kHz >= obsRxLaneRate_kHz) ?
				     (rxLaneRate_kHz) : (obsRxLaneRate_kHz);
	}

	/* performing integer multiple check on HS clock and lane rate clock */
	if (fasterLaneRate_kHz == 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SER_LANERATE_ZERO, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((init->clocks.clkPllVcoFreq_kHz % fasterLaneRate_kHz) != 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_HS_AND_LANE_RATE_NOT_INTEGER_MULT, retVal, TALACT_ERR_CHECK_PARAM);
	}

	txser_div = (uint8_t)(init->clocks.clkPllVcoFreq_kHz / fasterLaneRate_kHz);

	switch(txser_div) {
	case 1:
		txser_div_reg = 1;
		break;
	case 2:
		txser_div_reg = 2;
		break;
	case 4:
		txser_div_reg = 2;
		break;
	case 8:
		txser_div_reg = 2;
		break;
	default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SER_INV_TXSER_DIV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Set Tx serializer divider */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_JESD_SYNC_DIV_EN, txser_div_reg*0x5, 0x0000003C, 2);
	IF_ERR_RETURN_U32(retVal);
	//UART_Printf("---------vco:%d--larate:%d-------jesd tx div %d\r\n	rxLaneRate_kHz: %d	obsRxLaneRate_kHz:%d\r\n",init->clocks.clkPllVcoFreq_kHz,fasterLaneRate_kHz, txser_div_reg, rxLaneRate_kHz,obsRxLaneRate_kHz);
	retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_BBPLL_ID, 0x3C), rxLaneRate_kHz >= 8000000? 0:(0x3<<19), 0x3<<19, 0);
	IF_ERR_RETURN_U32(retVal);

#if 0
	/* Power down all lanes */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x1, 0x00004000, 14);
	IF_ERR_RETURN_U32(retVal);
	/* Allow framer data to output to serializer (clear reset) */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x0, 0x00004000, 14);
	IF_ERR_RETURN_U32(retVal);

	/* Toggle Serializer SRESET */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x1, 0x00004000, 14);
	IF_ERR_RETURN_U32(retVal);

	/* Allow framer data to output to serializer (clear reset) */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x0, 0x00004000, 14);
	IF_ERR_RETURN_U32(retVal);
#endif
	return (uint32_t)retVal;
}

uint32_t BR3109_setupDeserializers(br3109Device_t *device, br3109Init_t *init)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t lanesEnabled = 0;
	uint32_t laneRate_kHz = 0;
	uint8_t totalM = 0;
	uint8_t totalNp = 0;
	uint8_t totalL = 0;
	uint8_t i = 0;
	uint8_t laneRateRatio = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setupDeserializers()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if (device->devStateInfo.profilesValid & TX_PROFILE_VALID) {
		if (init->tx.deframerSel == TAL_DEFRAMER_A) {
			lanesEnabled = init->jesd204Settings.deframerA.deserializerLanesEnabled;
			totalM = init->jesd204Settings.deframerA.M;
			totalNp = init->jesd204Settings.deframerA.Np;
		} else if (init->tx.deframerSel == TAL_DEFRAMER_B) {
			lanesEnabled = init->jesd204Settings.deframerB.deserializerLanesEnabled;
			totalM = init->jesd204Settings.deframerB.M;
			totalNp = init->jesd204Settings.deframerB.Np;
		} else if (init->tx.deframerSel == TAL_DEFRAMER_A_AND_B) {
			/* verify deframer A and deframer B do not share common physical lanes */
			if ((init->jesd204Settings.deframerA.deserializerLanesEnabled &
			     init->jesd204Settings.deframerB.deserializerLanesEnabled) > 0) {
				return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								  TAL_ERR_DESER_LANECONFLICT_PARM, retVal, TALACT_ERR_CHECK_PARAM);
			}

			lanesEnabled = init->jesd204Settings.deframerA.deserializerLanesEnabled |
				       init->jesd204Settings.deframerB.deserializerLanesEnabled;

			if (init->jesd204Settings.deframerA.M != init->jesd204Settings.deframerB.M) {
				return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								  TAL_ERR_DESER_M_CONFLICT, retVal, TALACT_ERR_CHECK_PARAM);
			}

			if (init->jesd204Settings.deframerA.M > 2) {
				return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								  TAL_ERR_DESER_INV_DEFAB_M, retVal, TALACT_ERR_CHECK_PARAM);
			}

			if (init->jesd204Settings.deframerA.Np != init->jesd204Settings.deframerB.Np) {
				return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								  TAL_ERR_DESER_NP_CONFLICT, retVal, TALACT_ERR_CHECK_PARAM);
			}

			/* M * 2 since lane rate needs to be calculated with total M for both framers */
			totalM = init->jesd204Settings.deframerA.M * 2;
			totalNp = init->jesd204Settings.deframerA.Np;
		} else {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_DESER_INV_DEFRAMERSEL, retVal, TALACT_ERR_CHECK_PARAM);
		}

		if (lanesEnabled > 0x0F) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_DESER_INV_LANEEN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		}

		//lanesPowerDown = ((~lanesEnabled) & 0x0F);
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESER_TXPROFILE_INV, retVal, TALACT_ERR_CHECK_PARAM);
	}

	for (i = 0; i < 4; i++) {
		totalL += ((lanesEnabled >> i) & 0x01);
	}

	if ((totalL == 0) ||
	    (totalL == 3) ||
	    (totalL > 4)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESER_INV_L_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((totalM != 2) &&
	    (totalM != 4)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESER_INV_M_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((totalNp != 12) &&
	    (totalNp != 16)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESER_INV_NP_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	laneRate_kHz = (init->tx.txProfile.txInputRate_kHz * totalM * 10 * totalNp) /
		       (8 * totalL);

	if ((laneRate_kHz < 614400) ||
	    (laneRate_kHz > 12288000)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESER_INV_LANERATE_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (init->clocks.clkPllVcoFreq_kHz % laneRate_kHz != 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESES_HS_AND_LANE_RATE_NOT_INTEGER_MULT, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	if ((totalNp == 12) &&
	    (init->clocks.clkPllVcoFreq_kHz != laneRate_kHz)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESES_INV_LANE_RATE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	laneRateRatio = (uint8_t)(init->clocks.clkPllVcoFreq_kHz / laneRate_kHz);
	switch (laneRateRatio) {
	case 1:
//		halfRateMode = 1;
//		deserializerDiv = 0; /* divide by 1 */
		break;
	case 2:
//		halfRateMode = 0; /* Full rate mode */
//		deserializerDiv = 0; /* divide by 1 */
		break;
	case 4:
//		halfRateMode = 0; /* Full rate mode */
//		deserializerDiv = 1; /* divide by 2 */
		break;
	case 8:
//		halfRateMode = 0; /* Full rate mode */
//		deserializerDiv = 2; /* divide by 2 */
		break;
	case 16:
//		halfRateMode = 0; /* Full rate mode */
//		deserializerDiv = 3; /* divide by 2 */
		break;
	default: {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESES_INV_LANE_RATE_DIV, retVal, TALACT_ERR_CHECK_PARAM);
	}
	}

	if ((init->jesd204Settings.desEqSetting != 0) &&
	    (init->jesd204Settings.desEqSetting != 1) &&
	    (init->jesd204Settings.desEqSetting != 2)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESER_INV_EQ_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (init->jesd204Settings.desInvertLanePolarity > 15) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_DESER_INV_LANEPN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

#if 0
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x1, 0x00008000, 15);
	IF_ERR_RETURN_U32(retVal);

	/* Allow framer data to output to serializer (clear reset) */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x0, 0x00008000, 15);
	IF_ERR_RETURN_U32(retVal);
#endif
	return (uint32_t)retVal;
}

uint32_t BR3109_setupJesd204bFramer(br3109Device_t *device, br3109Init_t *init,
				    br3109FramerSel_t framerSel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	br3109Jesd204bFramerConfig_t *framer = NULL;
	uint32_t regdata = 0;
	uint8_t i = 0;
	uint8_t L = 0;
	uint8_t CF = 0;
	uint8_t CS = 0;
	uint8_t S = 1;
//	uint8_t JESDVER = 1;
//	uint8_t JESDSUBCLASS = 1;
	uint16_t FK = 0;
	uint8_t HD = 0;
	uint8_t framerLaneXbar = 0xE4;
	uint32_t outputRate_kHz = 0;
	uint8_t pclkDiv = 0;
	uint32_t hsDigClkDiv4or5_Hz = 0;
	uint32_t pclk_Hz = 0;
	uint16_t framerAddrOffset = 0;
	uint8_t scr = 0;

	const uint8_t DUALBANDENABLED = 1;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG,
				 TAL_ERR_OK, "BR3109_setupJesd204bFramer()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif
	
	if (framerSel == TAL_FRAMER_A) {
		framer = &init->jesd204Settings.framerA;
		framerAddrOffset = 0;
	} else if (framerSel == TAL_FRAMER_B) {
		framer = &init->jesd204Settings.framerB;
		framerAddrOffset = BR3109_JESD_FRAMERB_OFFSET;
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_FRAMERSEL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (framer->M == 0) {
		/* Disable Framer link and exit Framer setup */
//		halError = talSpiWriteByte(device->devHalInfo,
//					   (BR3109_ADDR_JESD_FRAMER_CFG_0 + framerAddrOffset), 0x02);
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
						  TALACT_ERR_RESET_SPI);
	} else if ((framer->M == 1)
		   && !(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_INT2_REALIF)
		   && !(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_DEC2_REALIF)
		   && !(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_FILTERONLY_REALIF)
		   && !(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_BYPASS_REALIF)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SER_INV_ZIF_TO_RIF_DATA_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	else if ((framer->M == 8) &&
		 !(device->devStateInfo.rxDualBandEnabled == DUALBANDENABLED)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_DUALBAND_DATA_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((framer->M != 1) &&
	    (framer->M != 2) &&
	    (framer->M != 4) &&
	    (framer->M != 8)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_M_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((framer->Np != 12) &&
	    (framer->Np != 16) &&
	    (framer->Np != 24)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_NP_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (framer->bankId > 15) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_BANKID_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* no need to check deviceId, its range is full uint8_t range */
	if (framer->lane0Id > 31) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_LANEID_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (framer->syncbInSelect > 1) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_SYNCBIN_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	

	/* count number of lanes */
	L = 0;
	for (i = 0; i < 4; i++) {
		L += ((framer->serializerLanesEnabled >> i) & 0x01);
	}
	

	if (L == 0) {
		/* Disable Link, and return from function */
//		halError = talSpiWriteByte(device->devHalInfo,
//					   (BR3109_ADDR_JESD_FRAMER_CFG_0 + framerAddrOffset), 0x02);
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
						  TALACT_ERR_RESET_SPI);
	} else if ((L != 1) &&
		   (L != 2) &&
		   (L != 4)) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_RXFRAMER_INV_L_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check for fractional S value  - invalid settings */
	if (((framer->F * L * 8) % (framer->Np * framer->M)) > 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_S_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	

	S = (framer->F * L * 8) / (framer->Np * framer->M);
	if ((S != 1) &&
	    (S != 2) &&
	    (S != 4) &&
	    (S != 8)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_S_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
		

	FK = ((uint16_t)(framer->K) * (uint16_t)(framer->F));

	if ((FK < 20) ||
	    (FK > 1024) ||
	    (FK % 4 != 0)) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_RXFRAMER_INV_FK_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}		
		
	HD = ((((uint16_t)(framer->F) * 8) % framer->Np) > 0) ? (1) : (0);

	if (framer->enableManualLaneXbar != 0) {
		framerLaneXbar = framer->serializerLaneCrossbar;
	} else {
		switch((framer->serializerLanesEnabled & 0x0F)) {
		/* Lanes 1 and 2 are swapped on laminate, correct here */
		/* Set unused lanes to select value b11 */
		case 1:
			framerLaneXbar = 0xFC;
			break;
		case 2:
			framerLaneXbar = 0xCF;
			break;
		case 3:
			framerLaneXbar = 0xDC;
			break;
		case 4:
			framerLaneXbar = 0xF3;
			break;
		case 5:
			framerLaneXbar = 0xF4;
			break;
		case 6:
			framerLaneXbar = 0xC7;
			break;
		case 8:
			framerLaneXbar = 0x3F;
			break;
		case 9:
			framerLaneXbar = 0x7C;
			break;
		case 10:
			framerLaneXbar = 0x4F;
			break;
		case 12:
			framerLaneXbar = 0xE4;
			break;
		case 15:
			framerLaneXbar = 0xD8;
			break;
		default:
			framerLaneXbar = 0xD8;
			break;
		}
	}
	

//	if (framer->M == 8) {
//		framerAdcXbar = 0x37251301;
//	} else {
//		framerAdcXbar = 0x00001301;
//	}

	/* Determine framer PCLK frequency */
	if (device->devStateInfo.profilesValid & RX_PROFILE_VALID) {
		/* If Rx profile is using this framer, use the Rx IQ data rate */
		if ((init->rx.framerSel == TAL_FRAMER_A_AND_B) ||
		    (init->rx.framerSel == framerSel)) {
			outputRate_kHz = init->rx.rxProfile.rxOutputRate_kHz;
		}
	}

	if (device->devStateInfo.profilesValid & ORX_PROFILE_VALID) {
		/* If ORx profile is using this framer, use the ORx IQ data rate */
		if ((init->obsRx.framerSel == TAL_FRAMER_A_AND_B)
		    || (init->obsRx.framerSel == framerSel)) {
			outputRate_kHz = init->obsRx.orxProfile.orxOutputRate_kHz;
		}
	}

	/* If Rx and ORx share same framer - set PCLK to faster of two rates */
	if ((device->devStateInfo.profilesValid & RX_PROFILE_VALID) &&
	    (device->devStateInfo.profilesValid & ORX_PROFILE_VALID) &&
	    (init->rx.framerSel == init->obsRx.framerSel) &&
	    (init->rx.framerSel == framerSel)) {
		if (init->obsRx.orxProfile.orxOutputRate_kHz >
		    init->rx.rxProfile.rxOutputRate_kHz) {
			outputRate_kHz =  init->obsRx.orxProfile.orxOutputRate_kHz;
		} else {
			outputRate_kHz = init->rx.rxProfile.rxOutputRate_kHz;
		}
	}

	if (outputRate_kHz == 0) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_RXFRAMER_INV_OUTPUT_RATE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	pclk_Hz = (outputRate_kHz * (uint32_t)(framer->F) * 1000) / ((uint32_t)(S) * 4);

	/* Determine PCLK divider */

	hsDigClkDiv4or5_Hz = device->devStateInfo.clocks.hsDigClkDiv4or5_Hz;

	if ((framer->F == 3) ||
	    (framer->F == 6)) {
		pclkDiv = 0;
	} else {
		for (pclkDiv = 0; pclkDiv < 8; pclkDiv++) {
			if ((hsDigClkDiv4or5_Hz >> pclkDiv) == pclk_Hz) {
				break; /* Use current pclk divider */
			}

			if (pclkDiv == 7) {
				return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM,
								  TAL_ERR_RXFRAMER_INV_PCLKFREQ, retVal, TALACT_ERR_CHECK_PARAM);
			}
		}
	}
	
	if (framerSel == TAL_FRAMER_A) {
		framer = &init->jesd204Settings.framerA;
		framerAddrOffset = 0;
	} else if (framerSel == TAL_FRAMER_B) {
		framer = &init->jesd204Settings.framerB;
		framerAddrOffset = BR3109_JESD_FRAMERB_OFFSET;
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_FRAMERSEL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	regdata =  0x00;
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_CONFIG4_0+ framerAddrOffset, regdata, 0x10, 0);
	/* Disable JESD204b link enable */
	regdata = (init->jesd204Settings.framerA.syncbInSelect & 0x01) | ((init->jesd204Settings.sysrefLvdsMode & 0x1) << 3)
			| ((init->jesd204Settings.sysrefLvdsPnInvert & 0x1)<<4);
	retVal = BR3109_ArmWriteField(device,  BR3109_ADDR_JESD_FRAMER_CONFIG_0, regdata, 0x00000019, 0);
	IF_ERR_RETURN_U32(retVal);
	/* enabling the SYSREF for relink if newSysrefOnRelink is set */
	// if (framer->newSysrefOnRelink > 0) {
	// 	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_CONFIG_0, 0x01, 0xff00, 8);
	// 	IF_ERR_RETURN_U32(retVal);
	// }
	// regdata  = 0x2;//sysref oneshot
	// retVal = BR3109_ArmWriteField(device,  BR3109_ADDR_JESD_FRAMER_CONFIG_1, regdata, 0x02, 1);
	// IF_ERR_RETURN_U32(retVal);
	// framer->bankId = 0;
	scr = framer->scramble;		//number of lanes -1 //scramble enable
	regdata = (framer->deviceId&0xff)|((framer->bankId & 0x0F)<<8)|((framer->lane0Id & 0x1F)<<16) | (scr << 31) | (((L-1) & 0x1F) << 24); /* DID /BID/LID/L/SCR*/
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_CONFIG1_0+ framerAddrOffset, regdata , 0x9F1FFFFF, 0);
	IF_ERR_RETURN_U32(retVal);
	regdata = ((framer->F - 1) & 0xff)|(((framer->K - 1) & 0x1f) << 8)|(((framer->M - 1) & 0xFF) << 16)|(((framer->Np-1) & 0x1F) << 24)|((CS & 3) << 30);
	retVal = BR3109_armMemoryCmd_blk_write(device,BR3109_ADDR_JESD_FRAMER_CONFIG2_0+ framerAddrOffset, &regdata, 1);
	IF_ERR_RETURN_U32(retVal);
//	HD = 1;
	regdata = ((framer->Np & 0x1F) - 1) | ((S - 1) << 8) | ((CF & 0x1F) << 16) | (HD << 23) | (0<<24); /* 4 multiframes in ILAS */
	retVal = BR3109_armMemoryCmd_blk_write(device,BR3109_ADDR_JESD_FRAMER_CONFIG3_0+ framerAddrOffset, &regdata, 1);
	IF_ERR_RETURN_U32(retVal);
	
	 /* power up desired lanes */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_CONFIG4_0+ framerAddrOffset, framer->serializerLanesEnabled, 0x0000000F, 0);
	IF_ERR_RETURN_U32(retVal);

	/* Sample Crossbar */
//	regdata =framerAdcXbar;//
//	retVal = BR3109_armMemoryCmd_blk_write(device,BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_0123_0, &regdata, 1);
//	IF_ERR_RETURN_U32(retVal);

	/* Framer: Set Lane Crossbar */
	regdata =framerLaneXbar;//
	retVal = BR3109_armMemoryCmd_blk_write(device,BR3109_ADDR_JESD_FRAMER_LANE_XBAR, &regdata, 1);
	IF_ERR_RETURN_U32(retVal);

	
	//tx cfg brats per multiframe
	//tx cfg lmfc offset
	FK = FK/4;		
	regdata = ((FK - 1) & 0xFF) | ((framer->lmfcOffset & 0xFF) << 16);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_CONFIG5_0+ framerAddrOffset, regdata, 0xFF00FF, 0);
	IF_ERR_RETURN_U32(retVal);
	/* Enable Framer link */
//	regdata =(framerSel == TAL_FRAMER_B)?0:1;
//	retVal = BR3109_enableFramerLink(device, framerSel, regdata);
//	IF_ERR_RETURN_U32(retVal);

//	/* Toggle Serializer SRESET */
//	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x1, 0x00004000, 14);
//	IF_ERR_RETURN_U32(retVal);
//
//	/* Allow framer data to output to serializer (clear reset) */
//	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x0, 0x00004000, 14);
//	IF_ERR_RETURN_U32(retVal);
	return (uint32_t)retVal;
}

uint32_t BR3109_setupJesd204bDeframer(br3109Device_t *device, br3109Init_t *init, br3109DeframerSel_t deframerSel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	br3109Jesd204bDeframerConfig_t *deframer = NULL;
	uint8_t i = 0;
	// uint8_t newSysrefOnRelink = 0;
	uint8_t scr = 0;
	uint8_t L = 0;
	uint8_t F = 0;
	uint8_t S = 1;
	uint32_t pclk_kHz = 0;
	uint16_t FK = 0;
	uint32_t hsDigClkDiv4or5_Hz = 0;
	uint8_t pclkDiv = 0;
	uint8_t pclkReg = 0;
	// uint8_t extSysref = 0;
	uint8_t syncBarSel = 1;
	uint8_t deframerInput = 0;
	uint8_t lane = 0;
	uint8_t deframerlaneXbar = 0;
	uint32_t regdat = 0;

	if (device->devStateInfo.profilesValid & TX_PROFILE_VALID) {
		if (deframerSel == TAL_DEFRAMER_A) {
			deframer = &init->jesd204Settings.deframerA;
		} else if (deframerSel == TAL_DEFRAMER_B) {
			return retVal;
			deframer = &init->jesd204Settings.deframerB;
		} else {
			return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_DEFSEL, retVal, TALACT_ERR_CHECK_PARAM);
		}
	} else {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_TXPROFILE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	// newSysrefOnRelink = (deframer->newSysrefOnRelink > 0) ? 4 : 0;

	scr = (deframer->scramble > 0) ? 0 : 1;

	if (deframer->deserializerLanesEnabled > 15) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_LANESEN, retVal, TALACT_ERR_CHECK_PARAM);
	}

	for (i = 0; i < 4; i++) {
		L += ((deframer->deserializerLanesEnabled >> i) & 0x01);
	}

	if ((L != 1) &&
	    (L != 2) &&
	    (L != 4)) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_L, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((deframer->M != 2) &&
	    (deframer->M != 4)) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_M, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((deframer->Np != 12) &&
	    (deframer->Np != 16)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_NP, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check for fractional F - invalid settings */
	if(device->devStateInfo.swTest > 0) {
		/*For a software test bypass bounds checking on F and assign a valid value*/
		F = 2;
	} else {
		if (((deframer->Np * deframer->M * S) % (8 * L)) > 0) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_F, retVal, TALACT_ERR_CHECK_PARAM);
		}

		F = (deframer->Np * deframer->M * S) / (8 * L);
	}

	if ((F < 1) ||
	    (F > 32)) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_F, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((deframer->K < 1) ||
	    (deframer->K > 32)) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_K, retVal, TALACT_ERR_CHECK_PARAM);
	}

	FK = (uint16_t)F * (uint16_t)deframer->K;
	if (((FK % 4) != 0) ||
	    (FK < 20) ||
	    (FK > 1024)) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_FK, retVal, TALACT_ERR_CHECK_PARAM);
	}

	pclk_kHz = init->tx.txProfile.txInputRate_kHz * (uint32_t)(F) / (uint32_t)(
			   S) / 4;
	//HD = ((((uint16_t)(F) * 8) % deframer->Np) > 0) ? 1 : 0;

	hsDigClkDiv4or5_Hz = device->devStateInfo.clocks.hsDigClkDiv4or5_Hz;

	if ((F == 3) ||
	    (F == 6)) {
		pclkReg = 0;
	} else {
		if (((hsDigClkDiv4or5_Hz / 1000) % pclk_kHz) != 0) {
			return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_PCLK, retVal, TALACT_ERR_CHECK_PARAM);
		}

		pclkDiv = (uint8_t)(hsDigClkDiv4or5_Hz / 1000 / pclk_kHz);
	//UART_Printf("hsDigClkDiv4or5_Hz / %d KHz / pclk_kHz/%d KHz \r\n",hsDigClkDiv4or5_Hz,pclk_kHz);
		switch (pclkDiv) {
		case 1:
			pclkReg = 0; /* PCLK = HsDigClk /4 */
			break;
		case 2:
			pclkReg = 1; /* PCLK = HsDigClk /8 */
			break;
		case 4:
			pclkReg = 2; /* PCLK = HsDigClk /16 */
			break;
		case 8:
			pclkReg = 3; /* PCLK = HsDigClk /32 */
			break;
		case 16:
			pclkReg = 3; /* PCLK = HsDigClk /64 */
			break;
		case 32:
			pclkReg = 3; /* PCLK = HsDigClk /128 */
			break;
		default: {
			return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_PCLKDIV, retVal, TALACT_ERR_CHECK_PARAM);
		}
		}
	}

	if (deframer->bankId > 15) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_BANKID, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (deframer->lane0Id > 31) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ERR_DEFRAMER_INV_LANEID, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (deframer->lmfcOffset >= deframer->K) {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFRAMER_INV_LMFC_OFFSET, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Disable deframer link */
	retVal = (talRecoveryActions_t)BR3109_enableDeframerLink(device, deframerSel, 0);
	IF_ERR_RETURN_U32(retVal);

	/* checksum mode - clear [7] for correct mode */
//	halError = brSpiWriteField(device->devHalInfo,
//				    (BR3109_ADDR_JESD_DEFRAMER_CFG3_0 + deframerAddrOffset), 0, 0x80, 7);
//	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
//				  TALACT_ERR_RESET_SPI);
//	IF_ERR_RETURN_U32(retVal);

	/* Disable Deframer CMM IRQ Interrupt */
//	halError = brSpiWriteField(device->devHalInfo,
//				    (BR3109_ADDR_JESD_DEFRAMER_IP_CFG3_0 + deframerAddrOffset), DFRMRCMMIRQMASK,
//				    0x80, 0);
//	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
//				  TALACT_ERR_RESET_SPI);
//	IF_ERR_RETURN_U32(retVal);

	if (deframer->enableManualLaneXbar != 0) {
		deframerlaneXbar = deframer->deserializerLaneCrossbar;
	} else {
		/* Lane crossbar  - Allow user to reorder lanes in deframer->deserializerLaneCrossbar, but this code still */
		/* maps used lanes to deframer inputs */
		deframerInput = 0;
		for (lane = 0; lane < 4; lane++) {
			if ((deframer->deserializerLanesEnabled >> lane) & 1) {
				deframerlaneXbar |= (((deframer->deserializerLaneCrossbar >>
						       (lane << 1)) & 3) << (deframerInput << 1));
				deframerInput += 1;
			}
		}
	}
	/* select SYNCOUTB 0 or 1 pin */
	syncBarSel = 0x3;
	/* Set LMFC Offset */
	regdat = (deframerlaneXbar & 0xFF) | ((syncBarSel & 0x3) << 26);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_CONFIG, regdat, 0xCFF1FFF, 0);
	IF_ERR_RETURN_U32(retVal);
	regdat = deframer->deserializerLanesEnabled;
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_CONFIG_0, regdat, 0xf, 0);
	IF_ERR_RETURN_U32(retVal);
	regdat = pclkReg-1;
	// //UART_Printf("---pclkReg--:0x%08X-------------%d KHz--------------\r\n",pclkReg, init->tx.txProfile.txInputRate_kHz);
	// retVal = BR3109_ArmWriteField(device, APB_TX_PATH_TOP_VIL_SEL, regdat, 0x00000007, 0);配置jesd tx 带宽
	// IF_ERR_RETURN_U32(retVal);



	//rx cfg octets per frame (also F)
	// rx cfg beats per multiframe
	FK = FK/4;
	regdat = ((deframer->lmfcOffset & 0xFF) << 16) | ((F-1) << 8) | ((FK-1) & 0xFF) ;
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_CONFIG_2, regdat, 0xFFFFFFFF, 0);
	IF_ERR_RETURN_U32(retVal);

	/* set PCLK for deframers */
	//UART_Printf("---RST0x30--pclkReg:0x%08X---------------------------\r\n",pclkReg);
	//retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_JESD_SYNC_DIV_EN, pclkReg&0x3, 0x00000030, 4);
	//IF_ERR_RETURN_U32(retVal);
	

	//scrambar
	regdat = ((scr&0x1)<<11)| 1 << 13;//>BB
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_CONFIG_1, regdat, 0x00002800, 0);
	IF_ERR_RETURN_U32(retVal);

//	/* Enable deframer link */
//	retVal = (talRecoveryActions_t)BR3109_enableDeframerLink(device, deframerSel,1);
//	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}


uint32_t BR3109_enableFramerLink(br3109Device_t *device, br3109FramerSel_t framerSel, uint8_t enable)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t enableLink = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_enableFramerLink()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	enableLink = (enable > 0) ? 1 : 0;

	if ((framerSel != TAL_FRAMER_A) && (framerSel != TAL_FRAMER_B)  && (framerSel != TAL_FRAMER_A_AND_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_RSTFRAMER_INV_FRAMERSEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((framerSel == TAL_FRAMER_A) || (framerSel == TAL_FRAMER_A_AND_B)) {
		// halError = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG_0), 0x1, 0x10000, 16);
		// IF_ERR_RETURN_U32(retVal);
		// halError = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG_0), 0, 0x10000, 16);
		// IF_ERR_RETURN_U32(retVal);
		halError = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG4_0), enableLink, 0x10, 4);
		IF_ERR_RETURN_U32(retVal);
	}

	if ((framerSel == TAL_FRAMER_B) || (framerSel == TAL_FRAMER_A_AND_B)) {
		// retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG_0), 0x1, 0x20000, 17);
		// IF_ERR_RETURN_U32(retVal);
		// retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG_0), 0, 0x20000, 17);
		// IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG4_0 + BR3109_JESD_FRAMERB_OFFSET), enableLink, 0x10, 4);
		IF_ERR_RETURN_U32(retVal);
	}

	return (uint32_t)retVal;
}
uint32_t BR3109_enableDeframerLink(br3109Device_t *device, br3109DeframerSel_t deframerSel, uint8_t enable)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_enableDeframerLink()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if (deframerSel == TAL_DEFRAMER_A) {
	} else if (deframerSel == TAL_DEFRAMER_B) {
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_RSTDEFRAMER_INV_DEFSEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (enable > 0) {
		//retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_DEFRAMER_CONFIG), 0x1, 0x10000, 16);
		//IF_ERR_RETURN_U32(retVal);
		//retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_DEFRAMER_CONFIG), 0, 0x10000, 16);
		//IF_ERR_RETURN_U32(retVal);
		/* Enable the deframer JESD link */
		retVal = BR3109_ArmWriteField(device,BR3109_ADDR_JESD_DEFRAMER_CONFIG_0, 1, 0x10, 4);
		IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_armSpiCmd_Jesd_config(device, JESD_RX);
		IF_ERR_RETURN_U32(retVal);
	} else {
		/* clear deframer link enable bit */
		retVal = BR3109_ArmWriteField(device,BR3109_ADDR_JESD_DEFRAMER_CONFIG_0, 0, 0x10, 4);
		IF_ERR_RETURN_U32(retVal);
	}
	
	return (uint32_t)retVal;
}

uint32_t BR3109_enableSysrefToFramer(br3109Device_t *device, br3109FramerSel_t framerSel, uint8_t enable)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t disableSysref = (enable > 0) ? 0 : 1;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_enableSysrefToFramer()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	if ((framerSel != TAL_FRAMER_A) && (framerSel != TAL_FRAMER_B) && (framerSel != TAL_FRAMER_A_AND_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ENFRAMERSYSREF_INV_FRAMERSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	retVal = BR3109_ArmWriteField(device,BR3109_ADDR_JESD_FRAMER_CONFIG_1, disableSysref, 0x1 << 0, 0);
	IF_ERR_RETURN_U32(retVal);
	return (uint32_t)retVal;
}

uint32_t BR3109_enableSysrefToDeframer(br3109Device_t *device, br3109DeframerSel_t deframerSel, uint8_t enable)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t disableSysref = (enable > 0) ? 0 : 1;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_enableSysrefToDeframer()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	if ((deframerSel != TAL_DEFRAMER_A) && (deframerSel != TAL_DEFRAMER_B) && (deframerSel != TAL_DEFRAMER_A_AND_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ENDEFSYSREF_INV_DEFRAMERSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	retVal = BR3109_ArmWriteField(device,BR3109_ADDR_JESD_DEFRAMER_CONFIG_0, disableSysref, 0x1 << 5, 5);
	IF_ERR_RETURN_U32(retVal);
	return (uint32_t)retVal;
}

uint32_t BR3109_readFramerStatus(br3109Device_t *device, br3109FramerSel_t framerSel, uint32_t *framerStatus)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t sysrefStatus = 0;
	uint32_t clear_irq = 0xfff;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_readFramerStatus()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	if ((framerSel != TAL_FRAMER_A) && (framerSel != TAL_FRAMER_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_FRAMERSTATUS_INV_FRAMERSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (framerStatus == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_FRAMERSTATUS_NULL_FRAMERSTATUS_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	retVal = BR3109_armMemoryCmd_blk_write(device, (BR3109_ADDR_JESD_FRAMER_IRQ ), &clear_irq, 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_armMemoryCmd_blk_read(device, (BR3109_ADDR_JESD_FRAMER_IRQ ), &sysrefStatus, 1);
	IF_ERR_RETURN_U32(retVal);
	*framerStatus = sysrefStatus;
	retVal = BR3109_armMemoryCmd_blk_read(device, (BR3109_ADDR_JESD_FRAMER_SYNC ), &sysrefStatus, 1);
	IF_ERR_RETURN_U32(retVal);
	*framerStatus |= sysrefStatus<<24;
	return (uint32_t)retVal;
}
				 
uint32_t BR3109_readDeframerStatus(br3109Device_t *device, br3109DeframerSel_t deframerSel, uint32_t *deframerStatus)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t eventObs = 0;
	uint32_t clear_irq = 0xF000FFF3;//0xF000FFF3;
	uint32_t clear_count = 0x000F0F0F;

#if BR3109_VERBOSE
	brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_readDeframerStatus()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if (deframerSel == TAL_DEFRAMER_A) {
	} else if (deframerSel == TAL_DEFRAMER_B) {
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFSTATUS_INV_DEFRAMERSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (deframerStatus == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_DEFSTATUS_NULL_DEFRAMERSTATUS_PARAM, retVal, TALACT_ERR_CHECK_PARAM);

	}
	
	retVal = BR3109_armMemoryCmd_blk_write(device, (BR3109_ADDR_JESD_DEFRAMER_NOTINTAB_DISERR_PRBS_COUNT_CLEAR ), &clear_count, 1);
	IF_ERR_RETURN_U32(retVal);
	clear_count = 0;
	retVal = BR3109_armMemoryCmd_blk_write(device, (BR3109_ADDR_JESD_DEFRAMER_NOTINTAB_DISERR_PRBS_COUNT_CLEAR ), &clear_count, 1);
	IF_ERR_RETURN_U32(retVal);
//	delay_ms(10);
	retVal = BR3109_armMemoryCmd_blk_write(device, (BR3109_ADDR_JESD_DEFRAMER_IRQ ), &clear_irq, 1);
	IF_ERR_RETURN_U32(retVal);
	delay_ms(1);
	retVal = BR3109_armMemoryCmd_blk_read(device, (BR3109_ADDR_JESD_DEFRAMER_IRQ ), &eventObs, 1);
	IF_ERR_RETURN_U32(retVal);
	
	*deframerStatus = eventObs;

	return (uint32_t)retVal;
}

static talRecoveryActions_t talCheckDacSampleXbarSelectEnum (
	br3109Device_t *device, br3109DacSampleXbarSelect_t lane)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	if ((lane != TAL_DEFRAMERA_OUT0) && (lane != TAL_DEFRAMERA_OUT1) &&
	    (lane != TAL_DEFRAMERA_OUT2) && (lane != TAL_DEFRAMERA_OUT3) &&
	    (lane != TAL_DEFRAMERB_OUT0) && (lane != TAL_DEFRAMERB_OUT1) &&
	    (lane != TAL_DEFRAMERB_OUT2) && (lane != TAL_DEFRAMERB_OUT3)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_INV_DAC_SAMP_XBAR_SELECT_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	return retVal;
}

uint32_t BR3109_setupDacSampleXbar(br3109Device_t *device,
				   br3109TxChannels_t channelSel, br3109DacSampleXbar_t dacXbar)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t xbarReg[2] = {0};
	static const uint8_t CONV_MASK = 0x1F;
	static const uint8_t CONV_POS  = 0x8;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setupDacSampleXbar()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Check Crossbar for valid settings */
	retVal = talCheckDacSampleXbarSelectEnum (device, dacXbar.dacChanI);
	IF_ERR_RETURN_U32(retVal);
	retVal = talCheckDacSampleXbarSelectEnum (device, dacXbar.dacChanQ);
	IF_ERR_RETURN_U32(retVal);

	/* performing Tx channel selection check */
	switch (channelSel) {
	case TAL_TX1:
//		channelAddr = BR3109_ADDR_JESD_DEFRAMER_CH1_SAMPLE_XBAR;
		break;
	case TAL_TX2:
		break;
	default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_INV_DAC_SAMP_XBAR_CHANNEL_SEL, retVal, TALACT_ERR_CHECK_PARAM);
	}
//	if(dacXbar.dacChanI != 2)
//		return retVal;
	xbarReg[0] =  (((uint8_t)dacXbar.dacChanQ*4) & CONV_MASK) | ((((
			uint8_t)dacXbar.dacChanI*4) & CONV_MASK) << CONV_POS);
	xbarReg[1] =  (((uint8_t)dacXbar.dacChanQ*4+1) & CONV_MASK) | ((((
			uint8_t)dacXbar.dacChanI*4+1) & CONV_MASK) << CONV_POS);

	// UART_Printf("\r\n****I:%d	Q:%d**********dacXbar0x64:0x%08X 0x%08X 0x%08X 0x%08X\r\n",dacXbar.dacChanI, dacXbar.dacChanQ, 
	// 	xbarReg[1]<<(16*(channelSel-1)), xbarReg[0]<<(16*(channelSel-1)),(xbarReg[1]+0x0202)<<(16*(channelSel-1)), (xbarReg[0]+0x0202)<<(16*(channelSel-1)));

	/* setting the DAC sample crossbar */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_0L, xbarReg[1], 0x1F1F<<(16*(channelSel-1)), (16*(channelSel-1)));
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_0H, xbarReg[0], 0x1F1F<<(16*(channelSel-1)), (16*(channelSel-1)));
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_1L, xbarReg[1]+0x0202, 0x1F1F<<(16*(channelSel-1)), (16*(channelSel-1)));
	IF_ERR_RETURN_U32(retVal);
//	if(channelSel > 1)
//	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_1H, xbarReg[0]+0x0202, 0x1F1F<<(16*(channelSel-1)), (16*(channelSel-1)));
//	else		
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_1H, xbarReg[0]+0x0202, 0x1F1F<<(16*(channelSel-1)), (16*(channelSel-1)));
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

static talRecoveryActions_t talCheckAdcSampleXbarSelectEnum (
	br3109Device_t *device, br3109AdcSampleXbarSelect_t conv)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	if ((conv != TAL_ADC_RX1_I) &&
	    (conv != TAL_ADC_RX1_Q) &&
	    (conv != TAL_ADC_RX2_I) &&
	    (conv != TAL_ADC_RX2_Q) &&
	    (conv != TAL_ADC_DUALBAND_RX1_BAND_A_I) &&
	    (conv != TAL_ADC_DUALBAND_RX1_BAND_A_Q) &&
	    (conv != TAL_ADC_DUALBAND_RX2_BAND_A_I) &&
	    (conv != TAL_ADC_DUALBAND_RX2_BAND_A_Q) &&
	    (conv != TAL_ADC_DUALBAND_RX1_BAND_B_I) &&
	    (conv != TAL_ADC_DUALBAND_RX1_BAND_B_Q) &&
	    (conv != TAL_ADC_DUALBAND_RX2_BAND_B_I) &&
	    (conv != TAL_ADC_DUALBAND_RX2_BAND_B_Q)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_INV_ADC_SAMP_XBAR_SELECT_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	return retVal;
}

uint32_t BR3109_setupAdcSampleXbar(br3109Device_t *device,
				   br3109FramerSel_t framerSel, br3109AdcSampleXbar_t adcXbar)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t framerOffset = 0;
	uint32_t xbarReg[2] = {0};
	static const uint8_t CONV_MASK = 0x3F;
	static const uint8_t CONV_POS  = 0x8;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setupAdcSampleXbar()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	switch (framerSel) {
	case TAL_FRAMER_A:
		framerOffset = 0;
		break;
	case TAL_FRAMER_B:
		framerOffset = 0x10;
		break;
	default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_INV_ADC_SAMP_XBAR_FRAMER_SEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check Crossbar for valid settings */
	retVal = talCheckAdcSampleXbarSelectEnum (device, adcXbar.conv0);
	IF_ERR_RETURN_U32(retVal);
	retVal = talCheckAdcSampleXbarSelectEnum (device, adcXbar.conv1);
	IF_ERR_RETURN_U32(retVal);
	retVal = talCheckAdcSampleXbarSelectEnum (device, adcXbar.conv2);
	IF_ERR_RETURN_U32(retVal);
	retVal = talCheckAdcSampleXbarSelectEnum (device, adcXbar.conv3);
	IF_ERR_RETURN_U32(retVal);
	retVal = talCheckAdcSampleXbarSelectEnum (device, adcXbar.conv4);
	IF_ERR_RETURN_U32(retVal);
	retVal = talCheckAdcSampleXbarSelectEnum (device, adcXbar.conv5);
	IF_ERR_RETURN_U32(retVal);
	retVal = talCheckAdcSampleXbarSelectEnum (device, adcXbar.conv6);
	IF_ERR_RETURN_U32(retVal);
	retVal = talCheckAdcSampleXbarSelectEnum (device, adcXbar.conv7);
	IF_ERR_RETURN_U32(retVal);		
	/* writing setting to the ADC sample crossbar */
	xbarReg[0] = (((uint8_t)adcXbar.conv2) & CONV_MASK) | ((((uint8_t)adcXbar.conv3) &	CONV_MASK) << CONV_POS)
			| ((((uint8_t)adcXbar.conv0) &	CONV_MASK) << CONV_POS*2)| ((((uint8_t)adcXbar.conv1) &
			CONV_MASK) << CONV_POS*3);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_0123_0 + framerOffset, xbarReg, 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_FRAMER_SAMPLE_AUX_XBAR_0123_0 + framerOffset, xbarReg, 1);
	IF_ERR_RETURN_U32(retVal);
	// UART_Printf("\r\n-----adcXbar0x4C+%x:0x%08X\r\n",framerOffset,xbarReg[0]);

	xbarReg[1] = (((uint8_t)adcXbar.conv6) & CONV_MASK) | ((((uint8_t)adcXbar.conv7) &	CONV_MASK) << CONV_POS)
			| ((((uint8_t)adcXbar.conv4) &	CONV_MASK) << CONV_POS*2)| ((((uint8_t)adcXbar.conv5) &
			CONV_MASK) << CONV_POS*3);

	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_4567_0 + framerOffset, &xbarReg[1], 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_FRAMER_SAMPLE_AUX_XBAR_4567_0 + framerOffset, &xbarReg[1], 1);
	IF_ERR_RETURN_U32(retVal);
	// UART_Printf("\r\n**************adcXbar0x50+%x:0x%08X\r\n",framerOffset,xbarReg[1]);
	xbarReg[0] += 0x08080808;
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_89AB_0 + framerOffset, &xbarReg[0], 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_FRAMER_SAMPLE_AUX_XBAR_89AB_0 + framerOffset, &xbarReg[0], 1);
	IF_ERR_RETURN_U32(retVal);
	xbarReg[1] += 0x08080808;
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_CDEF_0 + framerOffset, &xbarReg[1], 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_FRAMER_SAMPLE_AUX_XBAR_CDEF_0 + framerOffset, &xbarReg[1], 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_enableFramerTestData(br3109Device_t *device, br3109FramerSel_t framerSelect, br3109FramerDataSource_t testDataSource, br3109FramerInjectPoint_t injectPoint)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_enableFramerTestData()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if (testDataSource > TAL_FTD_CHECKERBOARD) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_FRAMER_INV_TESTDATA_SOURCE_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (injectPoint > TAL_FTD_POST_LANEMAP) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_FRAMER_INV_INJECTPOINT_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if ( ((testDataSource == TAL_FTD_CHECKERBOARD) && (injectPoint != TAL_FTD_POST_LANEMAP)) 
		 || ( (testDataSource != TAL_FTD_CHECKERBOARD) && (injectPoint == TAL_FTD_POST_LANEMAP))){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_INJECTPOINT_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((framerSelect != TAL_FRAMER_A) &&
	    (framerSelect != TAL_FRAMER_B) &&
	    (framerSelect != TAL_FRAMER_A_AND_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_INV_FRAMERSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if((TAL_TEST_DATA_DISABLE == testDataSource)|| (testDataSource == TAL_FTD_ADC_DATA)){
		regdat = 0;
		retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_DIG_MUX_LOOPBACK_FIXDATA, regdat, 0x3 << 12, 12);
		IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_TEST_CFG_0, regdat, 0xFFFFFFFFUL, 0);
		IF_ERR_RETURN_U32(retVal);	
	}else if(testDataSource < TAL_FTD_CHECKERBOARD){
		if (framerSelect != TAL_FRAMER_A_AND_B){
			regdat = (testDataSource | 0x8) | ((testDataSource | 0x8) << 4);
			regdat = regdat << (injectPoint * 16);
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_TEST_CFG_0, regdat, 0xFFFFFFFFUL, (8*(framerSelect))+16);
			IF_ERR_RETURN_U32(retVal);	
		}else{
			regdat = (0x8888|(testDataSource<<0) |(testDataSource<<4)|(testDataSource<<8)|(testDataSource<<12))<< (injectPoint * 16);
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_TEST_CFG_0, regdat, 0xFFFFFFFFUL, 0);
			IF_ERR_RETURN_U32(retVal);	
		}
	}else{//testDataSource == TAL_FTD_CHECKERBOARD
		if (framerSelect != TAL_FRAMER_A_AND_B){
			regdat = 0;
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_TEST_CFG_0, regdat, 0xFFFFFFFFUL, 0);
			IF_ERR_RETURN_U32(retVal);	
			regdat = 1;
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_DIG_MUX_LOOPBACK_FIXDATA, regdat, 0x1 << (12+framerSelect), 12+framerSelect);
			IF_ERR_RETURN_U32(retVal);	
			regdat = 0x12345678;
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_DIG_MUX_JESD_TX_IQ_CH1 + 4 * framerSelect, regdat,0xFFFFFFFFUL, 0);
			IF_ERR_RETURN_U32(retVal);	
		}else{
			regdat = 0;
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_TEST_CFG_0, regdat, 0xFFFFFFFFUL, 0);
			IF_ERR_RETURN_U32(retVal);	
			regdat = 3;
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_DIG_MUX_LOOPBACK_FIXDATA, regdat, 0x3 << 12, 12);
			IF_ERR_RETURN_U32(retVal);
			regdat = 0x12345678;
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_DIG_MUX_JESD_TX_IQ_CH1,regdat, 0xFFFFFFFFUL, 0);
			IF_ERR_RETURN_U32(retVal);	
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_DIG_MUX_JESD_TX_IQ_CH2,regdat, 0xFFFFFFFFUL, 0);
			IF_ERR_RETURN_U32(retVal);	
		}
	}

	return (uint32_t)retVal;
}

uint32_t BR3109_injectFramerTestDataError(br3109Device_t *device, br3109FramerSel_t framerSelect)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_injectFramerTestDataError()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if (framerSelect != TAL_FRAMER_A && framerSelect != TAL_FRAMER_B) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_FRAMER_ERRINJECT_INV_FRAMERSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* reading current PRBS control register contents */
	/* invert test data, then invert back to normal */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_TEST_CFG_0, 0x40, 0x400040<<(8*(framerSelect)), (8*(framerSelect)));
	IF_ERR_RETURN_U32(retVal);	
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_TEST_CFG_0, 0x00, 0x400040<<(8*(framerSelect)), (8*(framerSelect)));
	IF_ERR_RETURN_U32(retVal);	
	return (uint32_t)retVal;
}
		
uint32_t BR3109_enableDeframerPrbsChecker(br3109Device_t *device,
		br3109DeframerPrbsOrder_t polyOrder, br3109DefPrbsCheckLoc_t checkerLocation)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regaddr = 0;
	uint32_t regdata = 0;


#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_enableDeframerPrbsChecker()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	retValWarn = retVal;

	if ((polyOrder >= TAL_PRBS_DISABLE) &&
	    (polyOrder <= TAL_PRBS31) &&
	    (checkerLocation >= TAL_PRBSCHECK_LANEDATA) &&
	    (checkerLocation <= TAL_PRBSCHECK_SAMPLEDATA)) {

		retVal = (talRecoveryActions_t)BR3109_clearDeframerPrbsCounters(device);
		IF_ERR_RETURN_U32(retVal);
		regaddr = (checkerLocation == TAL_PRBSCHECK_LANEDATA)?BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_CFG : BR3109_ADDR_JESD_DEFRAMER_PRBS_CFG;
		regdata = (uint32_t)((polyOrder << 12) | (polyOrder <<8)|(polyOrder << 4)|(polyOrder << 0));
		retVal = BR3109_armMemoryCmd_blk_write(device, regaddr, &regdata, 1);
		IF_ERR_RETURN_U32(retVal);
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_EN_DEFRAMER_PRBS_INV_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* If higher priority retVal has no error, allow possible lower priority warning to be returned */
	if (retVal == TALACT_NO_ACTION) {
		retVal = retValWarn;
	}

	return (uint32_t)retVal;
}
		
uint32_t BR3109_clearDeframerPrbsCounters(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdata = 0xf;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_clearDeframerPrbsCounters()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	retVal = BR3109_armMemoryCmd_blk_write(device,
				   BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_COUNT_CLEAR, &regdata, 1);	//clear raw prbs counter
	IF_ERR_RETURN_U32(retVal);
	regdata = 0xf<<16;
	retVal = BR3109_armMemoryCmd_blk_write(device,
				   BR3109_ADDR_JESD_DEFRAMER_NOTINTAB_DISERR_PRBS_COUNT_CLEAR, &regdata, 1);	//clear prbs counter
	IF_ERR_RETURN_U32(retVal);
	regdata = 0;
	retVal = BR3109_armMemoryCmd_blk_write(device,
				   BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_COUNT_CLEAR, &regdata, 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_armMemoryCmd_blk_write(device,
				   BR3109_ADDR_JESD_DEFRAMER_NOTINTAB_DISERR_PRBS_COUNT_CLEAR, &regdata, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_readDeframerPrbsCounters(br3109Device_t *device, uint8_t lane,
		uint64_t *prbsErrorCount, uint64_t *prbsCheckCount)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t prbsCheckLocation = 0;
	uint32_t regbuf[2];

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_readDeframerPrbsCounters()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if (prbsErrorCount == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_READDFRMPRBS_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (prbsCheckCount == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_READDFRMPRBS_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if(lane >= 4){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_READDFRMPRBS_INV_DEFSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	/* Read back deframer output 0 sample counter */
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_JESD_DEFRAMER_PRBS_CFG, &prbsCheckLocation, 1);
	IF_ERR_RETURN_U32(retVal);

//	/* Save Lane Data Inverted Status */
//	*prbsInvertedStatus =  ((prbsCheckLocation & 0xF0) >> 4);

	/* Mask off the source bit */
	prbsCheckLocation = ((prbsCheckLocation>>4*lane) & 0x7);
	// UART_Printf("raw 0	%d\r\n",prbsCheckLocation);
	if (((br3109DefPrbsCheckLoc_t) prbsCheckLocation) == TAL_PRBSCHECK_LANEDATA) {
		retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_COUNT0 + 0x10*lane, regbuf, 2);	//raw prbs data
		IF_ERR_RETURN_U32(retVal);
		*prbsCheckCount = (((uint64_t)regbuf[1]<<32)|regbuf[0]);
		retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_ERROR_COUNT0 + 0x10*lane, regbuf, 2);
		IF_ERR_RETURN_U32(retVal);
		*prbsErrorCount = (((uint64_t)regbuf[1]<<32)|regbuf[0]);
	} else {
		retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_JESD_DEFRAMER_PRBS_COUNT0 + 0x10*lane, regbuf, 2);		//prbs 
		IF_ERR_RETURN_U32(retVal);
		*prbsCheckCount = (((uint64_t)regbuf[1]<<32)|regbuf[0]);
		retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT0 + 0x10*lane, regbuf, 2);
		IF_ERR_RETURN_U32(retVal);
		*prbsErrorCount = (((uint64_t)regbuf[1]<<32)|regbuf[0]);
	}

	return (uint32_t)retVal;
}

uint32_t BR3109_getDfrmIlasMismatch(br3109Device_t *device,
				    br3109DeframerSel_t deframerSelect, uint32_t *mismatch,
				    br3109Jesd204bLane0Config_t *dfrmCfg, br3109Jesd204bLane0Config_t *dfrmIlas)
{
	static const uint32_t LINK_ENABLE_MASK = 0x01;
	static const uint32_t JESD_SYNC_MASK   = 0x10000;

	talRecoveryActions_t retVal = TALACT_NO_ACTION;     /* Local api error container                                             */
	brHalErr_t halError = BRHAL_OK;       /* Local arm error container                                             */
	uint16_t i = 0;                         /* Local loop counter                                                    */
	uint32_t ilasdata[4] = {0};             /* Local holds lane0 ilas configuration data                             */
	uint32_t zeroCheck = 0x0000;            /* Local used for or'ing to determine if lane configuration is all zeros */
	uint32_t ilasAddress = 0;               /* Local used to assemble ilas addresses on the fly                      */
	uint32_t syncRegister = 0;               /* Local holds contents of syncB register                                */
	uint32_t rxcfgRegister = 0;               /* Local holds contents of syncB register                                */
	br3109Jesd204bLane0Config_t dfrmIlasLocal = {0}; /* Local deframer Ilas settings                                       */

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getDfrmIlasMismatch()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if (mismatch == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_JESD204B_ILAS_MISMATCH_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	*mismatch = 0;

	/* select the deframer to interrogate */
	if (deframerSelect == TAL_DEFRAMER_A) {
		ilasAddress = BR3109_ADDR_JESD_DEFRAMER_IP_OBS0_0;
	} else if (deframerSelect == TAL_DEFRAMER_B) {
		ilasAddress = BR3109_ADDR_JESD_DEFRAMER_IP_OBS1_0;
	} else {
		retVal = talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					  TAL_ERR_JESD204B_ILAS_MISMATCH_INVALID_DEFRAMER, retVal,
					  TALACT_ERR_CHECK_PARAM);
		IF_ERR_RETURN_U32(retVal);
	}

	/* if parameter dfrmCfg is null allocation buffer */
	if (dfrmCfg == NULL) {
		retVal = talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					  TAL_ERR_JESD204B_ILAS_MISMATCH_INVALID_DEFRAMER, retVal,
					  TALACT_ERR_CHECK_PARAM);
		IF_ERR_RETURN_U32(retVal);
	}

	/* if parameter dfrmIlas is null allocation buffer */
	if (dfrmIlas == NULL) {
		dfrmIlas = &dfrmIlasLocal;
	}

	/* check the deframer for link enable */
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_JESD_DEFRAMER_CONFIG_0, &rxcfgRegister, 1);
	IF_ERR_RETURN_U32(retVal);

	if ((syncRegister & LINK_ENABLE_MASK) == 0) {
		retVal = talApiErrHandler(device, TAL_ERRHDL_API_FAIL,
					  TAL_ERR_JESD204B_ILAS_MISMATCH_NO_ACTIVE_LINK, retVal,
					  TALACT_ERR_CHECK_PARAM);
		IF_ERR_RETURN_U32(retVal);
	}

	/* check the deframer for syncB active */
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_JESD_DEFRAMER_RX_CGS_FRAME_SYNC, &syncRegister, 1);
	IF_ERR_RETURN_U32(retVal);

	if ((syncRegister & JESD_SYNC_MASK) == 0) {
		retVal = talApiErrHandler(device, TAL_ERRHDL_API_FAIL,
					  TAL_ERR_JESD204B_ILAS_MISMATCH_SYNC_NOT_DETECTED, retVal,
					  TALACT_ERR_CHECK_PARAM);
		IF_ERR_RETURN_U32(retVal);
	}


	retVal = BR3109_armMemoryCmd_blk_read(device, ilasAddress, ilasdata, 4);
	IF_ERR_RETURN_U32(retVal);


	/* do the zero check of ilas data items */
	zeroCheck = 0x0000;
	for (i = 0; i < 4; i++) {
		zeroCheck = zeroCheck | ilasdata[i];
	}

	/* If ILAS configuration is all zeros then lane0 is not running */
	if (zeroCheck == 0x0000) {
		retVal = talApiErrHandler(device, TAL_ERRHDL_API_FAIL,
					  TAL_ERR_JESD204B_ILAS_MISMATCH_NO_ACTIVE_LINK, retVal,
					  TALACT_ERR_CHECK_PARAM);
		IF_ERR_RETURN_U32(retVal);
	}

	/* loading the structures with the read values for easier reading when doing compares */
	dfrmIlas->DID = ((ilasdata[0]>>16)&0xFF);                 	/* jesd_deframer_IP_obs0  */
	dfrmIlas->BID = ((ilasdata[0]>>24)&0x0F);        			/* jesd_deframer_IP_obs1  */
	dfrmIlas->LID0 = (ilasdata[1] & 0x1F);      				/* jesd_deframer_IP_obs2  */
	dfrmIlas->L = ((ilasdata[1]>>8)&0x0F);          			/* jesd_deframer_IP_obs3  */
	dfrmIlas->SCR = ((ilasdata[1]>>15)&0x01); 					/* jesd_deframer_IP_obs3  */
	dfrmIlas->F = ((ilasdata[1]>>16)&0xFF);                  	/* jesd_deframer_IP_obs4  */
	dfrmIlas->K = ((ilasdata[1]>>24)&0x0F);          			/* jesd_deframer_IP_obs5  */
	dfrmIlas->M = (ilasdata[2]&0xFF);                   		/* jesd_deframer_IP_obs6  */
	dfrmIlas->N = ((ilasdata[2]>>8)&0x0F);          			/* jesd_deframer_IP_obs7  */
	dfrmIlas->CS = ((ilasdata[2]>>14)&0x03);  					/* jesd_deframer_IP_obs7  */
	dfrmIlas->NP = ((ilasdata[2]>>16)&0x0F);        			/* jesd_deframer_IP_obs8  */
	dfrmIlas->S = ((ilasdata[2]>>24)&0x0F);          			/* jesd_deframer_IP_obs9  */
	dfrmIlas->CF = ((ilasdata[3]>>0)&0x1F);        				/* jesd_deframer_IP_obs10 */
	dfrmIlas->HD = ((ilasdata[3]>>7)&0x01); 					/* jesd_deframer_IP_obs10 */
	dfrmIlas->FCHK0 = ((ilasdata[4]>>0)&0x0F);              	/* lane0 Ilas CheckSum    */
	dfrmIlas->FCHK1 = ((ilasdata[4]>>8)&0x0F);              	/* lane1 Ilas CheckSum    */
	dfrmIlas->FCHK2 = ((ilasdata[4]>>16)&0x0F);              	/* lane2 Ilas CheckSum    */
	dfrmIlas->FCHK3 = ((ilasdata[4]>>24)&0x0F);              	/* lane3 Ilas CheckSum    */


//	dfrmCfg->DID = cfgdata[0];                  /* jesd_deframer_LO_cfg0       */
//	dfrmCfg->BID = (cfgdata[1] & 0x0F);         /* jesd_deframer_LO_cfg1       */
//	dfrmCfg->LID0 = (cfgdata[2] & 0x1F);        /* jesd_deframer_LO_cfg2       */
//	dfrmCfg->L = (cfgdata[3] & 0x1F);           /* jesd_deframer_LO_cfg3       */
//	dfrmCfg->SCR = ((cfgdata[3] & 0x80) >> 7);  /* jesd_deframer_LO_cfg3       */
//	dfrmCfg->F = cfgdata[4];                    /* jesd_deframer_LO_cfg4       */
//	dfrmCfg->K = (cfgdata[5] & 0x1F);           /* jesd_deframer_LO_cfg5       */
//	dfrmCfg->M = cfgdata[6];                    /* jesd_deframer_LO_cfg6       */
//	dfrmCfg->N = (cfgdata[7] & 0x1F);           /* jesd_deframer_LO_cfg7       */
//	dfrmCfg->CS = ((cfgdata[7] & 0xC0) >> 6);   /* jesd_deframer_LO_cfg7       */
//	dfrmCfg->NP = (cfgdata[8] & 0x1F);          /* jesd_deframer_LO_cfg8       */
//	dfrmCfg->S = (cfgdata[9] & 0x1F);           /* jesd_deframer_LO_cfg9       */
//	dfrmCfg->CF = (cfgdata[10] & 0x1F);         /* jesd_deframer_LO_cfg10      */
//	dfrmCfg->HD = ((cfgdata[10] & 0x80) >> 7);  /* jesd_deframer_LO_cfg10      */
//	dfrmCfg->FCHK0 = cfgdata[11];               /* lane0 Cfg Computed CheckSum */
//	dfrmCfg->FCHK1 = cfgdata[12];               /* lane1 Cfg Computed CheckSum */
//	dfrmCfg->FCHK2 = cfgdata[13];               /* lane2 Cfg Computed CheckSum */
//	dfrmCfg->FCHK3 = cfgdata[14];               /* lane3 Cfg Computed CheckSum */


	/* performing ILAS mismatch check */
	*mismatch = 0x00000000;
	if (dfrmIlas->DID != dfrmCfg->DID) {
		*mismatch |= 0x00000001;
	}

	if (dfrmIlas->BID != dfrmCfg->BID) {
		*mismatch |= 0x00000002;
	}

	if (dfrmIlas->LID0 != dfrmCfg->LID0) {
		*mismatch |= 0x00000004;
	}

	if (dfrmIlas->L != dfrmCfg->L) {
		*mismatch |= 0x00000008;
	}

	if (dfrmIlas->SCR != dfrmCfg->SCR) {
		*mismatch |= 0x00000010;
	}

	if (dfrmIlas->F != dfrmCfg->F) {
		*mismatch |= 0x00000020;
	}

	if (dfrmIlas->K != dfrmCfg->K) {
		*mismatch |= 0x00000040;
	}

	if (dfrmIlas->M != dfrmCfg->M) {
		*mismatch |= 0x00000080;
	}

	if (dfrmIlas->N != dfrmCfg->N) {
		*mismatch |= 0x00000100;
	}

	if (dfrmIlas->CS != dfrmCfg->CS) {
		*mismatch |= 0x00000200;
	}

	if (dfrmIlas->NP != dfrmCfg->NP) {
		*mismatch |= 0x00000400;
	}

	if (dfrmIlas->S != dfrmCfg->S) {
		*mismatch |= 0x00000800;
	}

	if (dfrmIlas->CF != dfrmCfg->CF) {
		*mismatch |= 0x00001000;
	}

	if (dfrmIlas->HD != dfrmCfg->HD) {
		*mismatch |= 0x00002000;
	}

	/* verify lane ilas checksums equal lane computed checksums */
//	for (i = 11; i < 15; i++) {
//		/* if ilas checksum equal zero then the lane is inactive so suppress lane checksum mismatch flag */
//		if ((ilasdata[i] != cfgdata[i]) && (ilasdata[i] != 0x00)) {
//			*mismatch |= (0x00004000 << (i - 11));
//
//		}
//	}

	return (uint32_t)retVal;
}
					
uint32_t BR3109_setDfrmIrqMask(br3109Device_t *device,
			       br3109DeframerSel_t deframerSelect, uint32_t irqMask)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = irqMask;
	
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setDfrmIrqMask()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* param range check */
	if ((deframerSelect != TAL_DEFRAMER_A) && (deframerSelect != TAL_DEFRAMER_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETDFRMIRQMASK_INV_DEFRAMERSEL_PARAM, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_DEFRAMER_IRQ_MASK_0, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	return ((uint32_t)retVal);
}


uint32_t BR3109_getDfrmIrqMask(br3109Device_t *device,
			       br3109DeframerSel_t deframerSelect, uint32_t *irqMask)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getDfrmIrqMask()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* param range check */
	if (irqMask == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETDFRMIRQMASK_NULL_IRQMASK_PARAM, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	/* param range check */
	if ((deframerSelect != TAL_DEFRAMER_A) &&
	    (deframerSelect != TAL_DEFRAMER_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETDFRMIRQMASK_INV_DEFRAMERSELECT_PARAM, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}
		
	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_JESD_DEFRAMER_IRQ_MASK_0, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	*irqMask = regdat;

	return ((uint32_t)retVal);
}

uint32_t BR3109_getDfrmIrqSource(br3109Device_t *device,
				 br3109DeframerSel_t deframerSelect, uint32_t *irqSourceValue)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getDfrmIrqSource()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* param range check */
	if (irqSourceValue == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETDFRMIRQSRC_NULL_STATUS_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* param range check */
	if ((deframerSelect != TAL_DEFRAMER_A) &&
	    (deframerSelect != TAL_DEFRAMER_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETDFRMIRQSRC_INV_DEFRAMERSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
		
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_DEFRAMER_IRQ, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	*irqSourceValue = regdat;

	return((uint32_t)retVal);
}

uint32_t BR3109_clearDfrmIrq(br3109Device_t *device,
			     br3109DeframerSel_t deframerSelect)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_clearDfrmIrq()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	retValWarn = retVal;

	/* param range check */
	if ((deframerSelect != TAL_DEFRAMER_A) &&
	    (deframerSelect != TAL_DEFRAMER_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_CLRDFRMIRQ_INV_DEFRAMERSEL_PARAM, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}
//	retVal = (talRecoveryActions_t)BR3109_getDfrmIrqMask(device, deframerSelect,
//			&irqMask);
//	IF_ERR_RETURN_U32(retVal);

	/* Clear the IRQ */
	regdat = 0x003111F2;
	retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_JESD_DEFRAMER_IRQ, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);


	/* If higher priority retVal has no error, allow possible lower priority warning to be returned */
	if (retVal == TALACT_NO_ACTION) {
		retVal = retValWarn;
	}

	return ((uint32_t)retVal);
}
#if 0		
talRecoveryActions_t talFindDfrmrLaneCntErr(br3109Device_t *device,
		br3109DeframerSel_t deframer, int32_t *deframerInputsMask)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

	uint8_t msByte = 0;
	uint8_t lsByte = 0;
	uint8_t cntrResetMask = 0;
	uint16_t deframerTcrAddress0 = 0;
	uint16_t deframerTcrAddress1 = 0;
	uint16_t deframerEcntRstAddress0 = 0;
	uint16_t deframerEcntRstAddress1 = 0;

	static const uint16_t ECNT_TCH_LANE0_MASK = 0x0007;
	static const uint16_t ECNT_TCH_LANE1_MASK = 0x0038;
	static const uint16_t ECNT_TCH_LANE2_MASK = 0x0007;
	static const uint16_t ECNT_TCH_LANE3_MASK = 0x0038;

	if (deframer == TAL_DEFRAMER_A) {
		deframerEcntRstAddress0 = BR3109_ADDR_JESD_DEFRAMER_IP_CFG5_0;
		deframerEcntRstAddress1 = BR3109_ADDR_JESD_DEFRAMER_IP_CFG6_0;
		deframerTcrAddress0 = BR3109_ADDR_JESD_DEFRAMER_IP_OBS23_0;
		deframerTcrAddress1 = BR3109_ADDR_JESD_DEFRAMER_IP_OBS24_0;

	} else if (deframer == TAL_DEFRAMER_B) {
		deframerEcntRstAddress0 = BR3109_ADDR_JESD_DEFRAMER_IP_CFG5_1;
		deframerEcntRstAddress1 = BR3109_ADDR_JESD_DEFRAMER_IP_CFG6_1;
		deframerTcrAddress0 = BR3109_ADDR_JESD_DEFRAMER_IP_OBS23_1;
		deframerTcrAddress1 = BR3109_ADDR_JESD_DEFRAMER_IP_OBS24_1;
	} else {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_TALFINDDFRMRLANECNTERROR_INV_DEFRAMERSEL_PARAM, retVal,
					TALACT_ERR_CHECK_PARAM);
	}

	if (deframerInputsMask == NULL) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_TALFINDDFRMRLANECNTERROR_NULL_PARAM, retVal,
					TALACT_ERR_CHECK_PARAM);
	}

	/* Read the TCR registers */
	halError = talSpiReadByte(device->devHalInfo, deframerTcrAddress1, &msByte);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	halError = talSpiReadByte(device->devHalInfo, deframerTcrAddress0, &lsByte);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* Deframer input 0 */
	*deframerInputsMask = 0;
	cntrResetMask = (lsByte & ECNT_TCH_LANE0_MASK);
	if (cntrResetMask > 0) {
		*deframerInputsMask |= 0x01;
	}

	/* Deframer input 1 */
	cntrResetMask = (lsByte & ECNT_TCH_LANE1_MASK);
	if (cntrResetMask > 0) {
		*deframerInputsMask |= 0x02;
	}

	/* Deframer input 2 */
	cntrResetMask = (msByte & ECNT_TCH_LANE2_MASK);
	if (cntrResetMask > 0) {
		*deframerInputsMask |= 0x04;
	}

	/* Deframer input 3 */
	cntrResetMask = (msByte & ECNT_TCH_LANE3_MASK);
	if (cntrResetMask > 0) {
		*deframerInputsMask |= 0x08;
	}

	/* clear the error counter */
	halError = talSpiWriteByte(device->devHalInfo, deframerEcntRstAddress0, 0x00);
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	/* clear the error counter */
	halError = talSpiWriteByte(device->devHalInfo, deframerEcntRstAddress1, 0x00);
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	return (retVal);
}

talRecoveryActions_t talFindDfrmrLaneErr(br3109Device_t *device,
		uint32_t dfrmErrAddress, uint8_t nibbleToUse, int32_t *deframerInputsMask)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

	uint8_t dfrmErrMask =
		0; /* 4 bits in either upper /lower nibble that describe the lanes an error occurred on */

	static const uint8_t HIGH = 1;

	if (deframerInputsMask == NULL) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_TALFINDDFRMRLANEERROR_NULL_PARAM, retVal,
					TALACT_ERR_CHECK_PARAM);
	}

	/* Read error flags from requested deframer register address */
	halError = talSpiReadByte(device->devHalInfo, dfrmErrAddress, &dfrmErrMask);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	if (nibbleToUse >= HIGH) {
		/* Grab lane error mask from upper 4 bits in each register */
		*deframerInputsMask = ((dfrmErrMask >> 4) & 0x0F);
	} else {
		/* Grab lane error mask from lower 4 bits in each register */
		*deframerInputsMask = (dfrmErrMask & 0x0F);
	}

	return (retVal);
}
#endif
uint32_t BR3109_framerSyncbToggle(br3109Device_t *device, br3109FramerSel_t framerSel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_framerSyncbToggle()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if ((framerSel != TAL_FRAMER_A) && (framerSel != TAL_FRAMER_B) && (framerSel != TAL_FRAMER_A_AND_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,  TAL_ERR_FRAMERSYSREFTOGGLE_INV_FRAMERSEL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Save possible warning from logging until end of function */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_CONFIG_0, 0, 0x00000002, 1);
	IF_ERR_RETURN(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_CONFIG_0, 1, 0x00000002, 1);
	IF_ERR_RETURN(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_JESD_FRAMER_CONFIG_0, 0, 0x00000002, 1);
	IF_ERR_RETURN(retVal);

	return (uint32_t)retVal;
}


uint32_t BR3109_FramerRest(br3109Device_t *device, br3109FramerSel_t framerSel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_FramerRest()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if ((framerSel != TAL_FRAMER_A) && (framerSel != TAL_FRAMER_B)  && (framerSel != TAL_FRAMER_A_AND_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_RSTFRAMER_INV_FRAMERSEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((framerSel == TAL_FRAMER_A) || (framerSel == TAL_FRAMER_A_AND_B)) {
		retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG_0), 1<<16, 0x1<<16, 0);//reset
		IF_ERR_RETURN_U32(retVal);
	}

	if ((framerSel == TAL_FRAMER_B) || (framerSel == TAL_FRAMER_A_AND_B)) {
		retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG_0), 1<<17, 0x1<<17, 0);//reset
		IF_ERR_RETURN_U32(retVal);
	}
	retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_FRAMER_CONFIG_0), 0<<16, 0x3<<16, 0);//dereset
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_DeframerRest(br3109Device_t *device, br3109DeframerSel_t framerSel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_DeframerRest()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if ((framerSel != TAL_DEFRAMER_A) && (framerSel != TAL_DEFRAMER_B)  && (framerSel != TAL_DEFRAMER_A_AND_B)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_RSTFRAMER_INV_FRAMERSEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_DEFRAMER_CONFIG), 1<<16, 0x1<<16, 0);//reset
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_ArmWriteField(device, (BR3109_ADDR_JESD_DEFRAMER_CONFIG), 0<<16, 0x1<<16, 0);//dereset
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}
