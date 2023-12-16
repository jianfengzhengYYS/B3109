/**
 * \file br3109.c
 * \brief Contains top level functions to support initialization and ctrol of
 *         the Br3109 transceiver device.
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

/**
 * \mainpage Overview
 *
 * This document is intended for use by software engineering professionals
 * and includes detailed information regarding the data types and
 * function calls which comprise the Br3109 ANSI C API
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 *
 */

/**
 * \page Suggested Use
 * The purpose of the Br3109 Application Programming Interface (API) is to abstract the low-level
 * SPI configuration, control, and related calculations required for the Br3109 family of transceiver devices
 *
 * Add the included source code as required to your baseband processor software build. Please reference the integration document
 * as well for further instructions.
 */

/**
 * \page Architecture Br3109 API Software Architecture
 * \image html software_architecture.png
 */

/**
 * \page Folders Br3109 API Folder Structure
 * \image html folder_structure.png
 */

#include "br3109.h"
#include "br3109_reg_addr_macros.h"
#include "br3109_arm.h"
#include "br3109_hal.h"
#include "br3109_user.h"
#include "br3109_error.h"
#include "br3109_version.h"
#include "br3109_rx.h"
#include "br3109_tx.h"
#include "br3109_jesd204.h"
#include "br3109_cals.h"
#include "br3109_gpio.h"
#include "br3109_config.h"

#define BR3109_VERBOSE 1
#define BR3109_LOGGING 0xF      /*LogLevel Set to All*/
#define BR3109_RESET_ON_ERR  1   /*API Reset on Severe Errors*/

uint32_t BR3109_openHw(br3109Device_t *device )
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

	BRHAL_setLogLevel(device->devHalInfo, BR3109_LOGGING);   
	halError = BRHAL_openHw(device->devHalInfo,100);
	if (halError != BRHAL_OK) {
		switch(halError) {
		case BRHAL_SPI_FAIL:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_HAL_SPI,  halError, retVal, TALACT_ERR_RESET_SPI);
		case BRHAL_GPIO_FAIL:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_HAL_GPIO, halError, retVal, TALACT_ERR_RESET_GPIO);
		case BRHAL_TIMER_FAIL:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_HAL_WAIT, halError, retVal, TALACT_ERR_CHECK_TIMER);
		case BRHAL_GEN_SW:
		default:
			return (uint32_t)TALACT_ERR_CHECK_PARAM;
		}
	}

	return (uint32_t)retVal;
}

uint32_t BR3109_closeHw(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_closeHw(device->devHalInfo);

	if (halError != BRHAL_OK) {
		switch(halError) {
		case BRHAL_SPI_FAIL:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
		case BRHAL_GPIO_FAIL:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_HAL_GPIO, halError, retVal, TALACT_ERR_RESET_GPIO);
		case BRHAL_TIMER_FAIL:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_HAL_WAIT, halError, retVal, TALACT_ERR_CHECK_TIMER);
		case BRHAL_GEN_SW:
		default:
			return (uint32_t)TALACT_ERR_CHECK_PARAM;
		}
	}

	return (uint32_t)retVal;
}

uint32_t BR3109_resetDevice(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	
#if BR3109_VERBOSE
		halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_resetDevice()\n");
		retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	
		/* toggle RESETB on device with matching spi chip select index */
		halError = BRHAL_resetHw(device->devHalInfo);
		if (halError == BRHAL_WAIT_TIMEOUT)
		{
			/*Retry*/
			halError = BRHAL_resetHw(device->devHalInfo);
		}
	
		if (halError != BRHAL_OK)
		{
			retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_GPIO, halError, retVal, TALACT_ERR_RESET_GPIO);
		}

	device->devStateInfo.devState = TAL_STATE_POWERONRESET;
//	delay_us(1000);
	return (uint32_t)retVal;
}

uint32_t BR3109_DeResetDevice(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_DeResetDevice()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	halError = brSpiWriteWord(device->devHalInfo, BR3109_ADDR_SPI_INIT_RESET, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);
	delay_us(1000);   
	return (uint32_t)retVal;
}


static talRecoveryActions_t talInitializeDdc(br3109Device_t *device, br3109Init_t *init)
{
//TODO

	uint8_t totalFramerM = 0;
	uint8_t combineBands = 0;
	uint8_t pfirOutDecimation = 0;
	uint8_t ncoPhaseAccumDivideRatio = 0;
	uint8_t rxNco1PhaseAccumDivideRatio = 0;
	uint8_t rxNco2PhaseAccumDivideRatio = 0;
	uint8_t registerValue = 0;
	uint8_t rxNcoControl = 0;
	// brHalErr_t halError = BRHAL_OK;
	talRecoveryActions_t retVal = TALACT_NO_ACTION;

	static const uint8_t RX_DUALBAND_ENABLED = 1;
	static const uint8_t RX_DUALBAND_DISABLED = 0;
	static const uint8_t ENABLE_NCO1 = 0x01;
	static const uint8_t ENABLE_NCO2 = 0x40;
	static const uint8_t ENABLE_NCO1_REALIF = 0x20;
	static const uint8_t ENABLE_NCO2_REALIF = 0x80;
	static const uint8_t DDC_DEC2 = 0x04;    /* Set DDC Mode and enable bit */
	static const uint8_t DDC_INT2 = 0x14;    /* Set DDC Mode and enable bit */
	static const uint8_t DDC_LOWPASS = 0x0C; /* Set DDC Mode and enable bit */

	/* Verify Null pointer to *init */
	if (init == NULL) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_INITIALIZE_DDC_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((device->devStateInfo.profilesValid & RX_PROFILE_VALID) &&
	    (init->rx.rxChannels != TAL_RXOFF)) {
		/* Phase accumulator calculation for NCO : FLOGB2(Decimation at PFIR out) - 2 */
		/* The -2 is because NCO clock is divided version of the div4/5 clock */
		/* For TAL_RXDDC_ZIF_TO_RIF_INT2 mode, Decimation at PFIR should be more than 8 since it is going to be interpolated */
		pfirOutDecimation = init->rx.rxProfile.rxDec5Decimation *
				    init->rx.rxProfile.rhb1Decimation * init->rx.rxProfile.rxFirDecimation;
		if (((pfirOutDecimation < 4)
		     && (init->rx.rxProfile.rxDdcMode != TAL_RXDDC_INT2)) ||
		    ((pfirOutDecimation < 8)
		     && (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_INT2_REALIF)) ||
		    (pfirOutDecimation > 40)) {
			return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						TAL_ERR_INITIALIZE_DDC_INV_DEC_AT_PFIR, retVal, TALACT_ERR_CHECK_PARAM);
		} else {
			pfirOutDecimation >>= 3;
			for (ncoPhaseAccumDivideRatio = 0; ncoPhaseAccumDivideRatio < 3;
			     ncoPhaseAccumDivideRatio++) {
				if (pfirOutDecimation == 0) {
					break;
				} else {
					pfirOutDecimation >>= 1;
				}
			}
		}

		device->devStateInfo.rxDualBandEnabled = RX_DUALBAND_DISABLED;
		device->devStateInfo.rxOutputRate_kHz = init->rx.rxProfile.rxOutputRate_kHz;


		/* Dualband mode */
		if ((init->rx.rxProfile.rxNcoShifterCfg.bandAInputBandWidth_kHz != 0) &&
		    (init->rx.rxProfile.rxNcoShifterCfg.bandBInputBandWidth_kHz != 0)) {
			/* Total number of converters */
			totalFramerM = (((init->rx.framerSel == TAL_FRAMER_A)
					 || (init->rx.framerSel == TAL_FRAMER_A_AND_B)) ?
					init->jesd204Settings.framerA.M : 0) +
				       (((init->rx.framerSel == TAL_FRAMER_B)
					 || (init->rx.framerSel == TAL_FRAMER_A_AND_B)) ?
					init->jesd204Settings.framerB.M : 0);

			/*  If one Rx enabled and M = 2, combine two bands. If M = 4, separate them */
			/*  If two Rx enabled and M = 4, combine two bands. If M = 8, separate them */
			if ((init->rx.rxChannels == TAL_RX1) ||
			    (init->rx.rxChannels == TAL_RX2)) {
				if ((totalFramerM != 2) && (totalFramerM != 4)) {
					return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								TAL_ERR_INITIALIZE_DDC_INV_TOTAL_M_2OR4, retVal, TALACT_ERR_CHECK_PARAM);
				}

				combineBands = (totalFramerM == 2) ? 1 : 0;
			} else { /* TAL_RX1RX2 */
				if ((totalFramerM != 4) && (totalFramerM != 8)) {
					return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								TAL_ERR_INITIALIZE_DDC_INV_TOTAL_M_4OR8, retVal, TALACT_ERR_CHECK_PARAM);
				}

				combineBands = (totalFramerM == 4) ? 1 : 0;
			}

			/* Phase accumulator calculation for NCO1 : FLOGB2(Decimation at PFIR out) - 2 */
			rxNco1PhaseAccumDivideRatio = ncoPhaseAccumDivideRatio;

			/* Phase accumulator calculation for NCO2 : NCO1 + 1 */
			rxNco2PhaseAccumDivideRatio = rxNco1PhaseAccumDivideRatio + 1;

			/* Setup value of NCO1 and NCO2 */
			registerValue = (rxNco2PhaseAccumDivideRatio << 4) |
					rxNco1PhaseAccumDivideRatio;
			device->devStateInfo.rxDualBandEnabled = RX_DUALBAND_ENABLED;
		}/* end dualband */

		/* Not Dualband */
		else {
			/* Phase accumulator calculation for NCO1 : FLOGB2(Decimation at PFIR out) - 2 */
			rxNco1PhaseAccumDivideRatio = ncoPhaseAccumDivideRatio;
			rxNcoControl = 0;

			switch(init->rx.rxProfile.rxDdcMode) {
			case TAL_RXDDC_BYPASS:
			case TAL_RXDDC_BYPASS_REALIF:
				rxNco2PhaseAccumDivideRatio = ncoPhaseAccumDivideRatio;
				/* Leave rxNcoControl = 0 to bypass DDC HB */
				break;

			case TAL_RXDDC_FILTERONLY:
			case TAL_RXDDC_FILTERONLY_REALIF:
				rxNco2PhaseAccumDivideRatio = ncoPhaseAccumDivideRatio;
				rxNcoControl |= DDC_LOWPASS;
				break;

			case TAL_RXDDC_DEC2:
			case TAL_RXDDC_DEC2_REALIF:
				rxNco2PhaseAccumDivideRatio = ncoPhaseAccumDivideRatio + 1;
				rxNcoControl |= DDC_DEC2;
				break;

			case TAL_RXDDC_INT2:
			case TAL_RXDDC_INT2_REALIF:
				rxNco2PhaseAccumDivideRatio = ncoPhaseAccumDivideRatio - 1;
				rxNcoControl |= DDC_INT2;
				break;

			default:

				return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							TAL_ERR_INITIALIZE_DDC_INV_RXDDCMODE, retVal, TALACT_ERR_CHECK_PARAM);
			}

			/* if nco1 shift */
			if (init->rx.rxProfile.rxNcoShifterCfg.bandANco1Freq_kHz != 0) {
				rxNcoControl |= ENABLE_NCO1;

				if ((init->rx.rxProfile.rxDdcMode == TAL_RXDDC_DEC2_REALIF) ||
				    (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_INT2_REALIF) ||
				    (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_FILTERONLY_REALIF) ||
				    (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_BYPASS_REALIF)) {
					rxNcoControl |= ENABLE_NCO1_REALIF;
				}
			}

			/* if nco2 shift */
			if (init->rx.rxProfile.rxNcoShifterCfg.bandANco2Freq_kHz != 0) {
				rxNcoControl |= ENABLE_NCO2;

				if ((init->rx.rxProfile.rxDdcMode == TAL_RXDDC_DEC2_REALIF) ||
				    (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_INT2_REALIF) ||
				    (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_FILTERONLY_REALIF) ||
				    (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_BYPASS_REALIF)) {
					rxNcoControl &= ~ENABLE_NCO1_REALIF;
					rxNcoControl |= ENABLE_NCO2_REALIF;
				}
			}
		} /* end Not Dualband */
	}
	(void)registerValue;
	(void)combineBands;
	return retVal;
}
#define CODE_EN_FOR_RE	1


uint32_t BR3109_initialize(br3109Device_t *device, br3109Init_t *init)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
	brHalErr_t          halError = BRHAL_OK;
    uint8_t txChannelSettings = 0;
    uint8_t rxChannelSettings = 0;
    uint8_t orxChannelSettings = 0;
    uint8_t dacClockRateSelect = 0;
    uint8_t rxFirDecBitField = 0;
    uint8_t orxFirDecBitField = 0;
    uint8_t digDeviceClockDiv = 0;
    uint32_t digRefClock_MHz = 0;
    br3109Info_t clearInfo ={
       // .devState = TAL_STATE_POWERONRESET,
        .initializedChannels = 0,
        .profilesValid = 0,
        .errSource = 0,
        .errCode = 0,
        .clocks = 
        {
            .deviceClock_kHz = 0,
            .clkPllVcoFreq_kHz = 0,
            .clkPllHsDiv = TAL_HSDIV_2,
            .hsDigClkDiv2_Hz = 0,
            .hsDigClkDiv4or5_Hz = 0,
            .rfPllUseExternalLo = 0
        },
        // .gainMode = TAL_MGC,
        // .gainIndexes = 
        // {
        //     .rx1MinGainIndex = 0,
        //     .rx1MaxGainIndex = 0,
        //     .rx2MinGainIndex = 0,
        //     .rx2MaxGainIndex = 0
        // },
        .txAttenStepSize = TAL_TXATTEN_0P05_DB,
        .usedGpiopins = 0,
        .usedGpio3p3pins = 0,
        .rxFramerNp = 0,    
        .orxFramerNp = 0,        
        .rxOutputRate_kHz = 0,    
        .txInputRate_kHz = 0,
		.tx1_s_gain = 0,
		.tx2_s_gain = 0,
        .rxDdcMode = TAL_RXDDC_BYPASS,
        .rxDualBandEnabled = 0,
        .rxTotalM = 0,
        .rxBandwidth_Hz = 0, 
        .txBandwidth_Hz = 0,
        .orxBandwidth_Hz = 0,
        .swTest = 0,
        .deviceSiRev = 0,
        .talErrFunctionTable = 
        {
            .talErrorFunctionTable ={{0},{0}}
        },
        .talFhmFreqRange = 
        {
            .fhmMinFreq_MHz = 0,
         .fhmMaxFreq_MHz = 0
        },
        .talFhmTriggerMode = TAL_FHM_GPIO_MODE,
        .talFhmInitHopFreq_Hz = 0,
        .talFhmMcsSync = 0
    };

    uint32_t agcClock_Hz = 0;
    uint32_t gainUpdateCount = 0;
    br3109AdcSampleXbar_t adcXbar = {(br3109AdcSampleXbarSelect_t)0,(br3109AdcSampleXbarSelect_t)0,(br3109AdcSampleXbarSelect_t)0,(br3109AdcSampleXbarSelect_t)0,(br3109AdcSampleXbarSelect_t)0,(br3109AdcSampleXbarSelect_t)0,(br3109AdcSampleXbarSelect_t)0,(br3109AdcSampleXbarSelect_t)0};
    br3109DacSampleXbar_t dacXbar = {(br3109DacSampleXbarSelect_t)0,(br3109DacSampleXbarSelect_t)0};
    // uint8_t deframerSyncPadCfg = 0;
    // uint16_t syncbPadCfgAddr = 0;
    // uint8_t framerSyncPadCfg = 0;
    uint8_t sysrefCfg = 0;
    uint32_t regData = 0;

	/* Verify pointers to *device and *init are not NULL */
	if (device == NULL) {
		/* Can not write to log since log function requires device data structure */
		return (uint32_t)TALACT_ERR_CHECK_PARAM;
	}

	if (init == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INIT_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_initialize()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    retValWarn = retVal;

    /* clear memory of Br3109 API state container */
	clearInfo.devState = device->devStateInfo.devState;
    device->devStateInfo = clearInfo;
	device->devStateInfo.rxGainCtrl = init->rx.rxGainCtrl;
    device->devStateInfo.txAttenStepSize = init->tx.txAttenStepSize;
	device->devStateInfo.orxBandwidth_SW_Hz = TX_BW_SWTICH_FREQ(init->clocks.deviceClock_kHz * 1000);
	device->devStateInfo.rxBandwidth_SW_Hz = TX_BW_SWTICH_FREQ(init->clocks.deviceClock_kHz * 1000)/2;
	device->devStateInfo.txBandwidth_SW_Hz = TX_BW_SWTICH_FREQ(init->clocks.deviceClock_kHz * 1000);
#if defined(CODE_EN_FOR_RE)&&(CODE_EN_FOR_RE == 1)
	/* Calculate digital clocks based on information in init structure */
	retVal = (talRecoveryActions_t)BR3109_calculateDigitalClocks(device, &init->clocks);
	IF_ERR_RETURN_U32(retVal);

	/* Verify Rx/Tx and ObsRx profiles are valid combinations
	 * and set device->devStateInfo.profilesValid member based on init struct settings
	 */
	retVal = (talRecoveryActions_t)BR3109_verifyProfiles(device, init);
	IF_ERR_RETURN_U32(retVal);
	/* Set 3 or 4-wire SPI mode, MSBFirst/LSBfirst in device, pushes CPOL=0, CPHA=0, longInstWord=1 into device->spiSettings */
	/* Read device silicon revision */
	retVal = (talRecoveryActions_t)BR3109_getDeviceRev(device, &regData);
	device->devStateInfo.deviceSiRev = regData;
	IF_ERR_RETURN_U32(retVal);
#endif
	sysrefCfg = (((init->jesd204Settings.sysrefLvdsMode > 0) ? 1 : 0) << 1)
			| /* Set SYSREF PAD for LVDS mode = 1, or CMOS mode = 0 */
			(((init->jesd204Settings.sysrefLvdsPnInvert > 0) ? 1 : 0) << 2) |
			(((init->jesd204Settings.sysrefLvdsMode > 0) ? 1 : 0) <<
			 4);  /* Set LVDS 100 ohm input termination */
	/* Set Digital device clock divider - provides clock to 1us timer logic, and some other blocks */
	if (init->clocks.deviceClock_kHz <= 200000) {
		digDeviceClockDiv =  0; /* Device clock / 1 */
		digRefClock_MHz = (init->clocks.deviceClock_kHz / 1000);
	} else if ((init->clocks.deviceClock_kHz <= 400000) && (init->clocks.deviceClock_kHz > 200000)) {
		digDeviceClockDiv =  1; /* Device clock / 2 */
		digRefClock_MHz = (((init->clocks.deviceClock_kHz / 1000) + 1) >> 1); /* /2 with round to nearest */
	} else if ((init->clocks.deviceClock_kHz <= 800000) && (init->clocks.deviceClock_kHz > 400000)) {
		digDeviceClockDiv =  2; /* Device clock / 4 */
		digRefClock_MHz = (((init->clocks.deviceClock_kHz / 1000) + 2) >> 2); /* /4 with round to nearest */
	} else if ((init->clocks.deviceClock_kHz <= 1600000) && (init->clocks.deviceClock_kHz > 800000)) {
		digDeviceClockDiv =  3; /* Device clock / 8 */
		digRefClock_MHz = (((init->clocks.deviceClock_kHz / 1000) + 4) >> 3); /* /8 with round to nearest */
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,  TAL_ERR_INIT_INV_DEVCLK, retVal, TALACT_ERR_CHECK_PARAM);
	}
	
	/* Tx Settings */
	if (device->devStateInfo.profilesValid & TX_PROFILE_VALID) {
		if (init->tx.txProfile.thb3Interpolation == 2) {
			txChannelSettings |= 0x10;
		}
		if (init->tx.txProfile.thb2Interpolation == 2) {
			txChannelSettings |= 0x04;
		}
		if (init->tx.txProfile.thb1Interpolation == 2) {
			txChannelSettings |= 0x08;
		}
		switch (init->tx.txProfile.txFirInterpolation) {
			case  1:
				txChannelSettings |= 0x01;
				break;
			case  2:
				txChannelSettings |= 0x02;
				break;
			case  4:
				txChannelSettings |= 0x03;
				break;
		}
		/* Set Rx Dec5 if Int5 is enabled */
		if (init->tx.txProfile.txInt5Interpolation == 5) {
			rxChannelSettings |= 0x02;
		}

		if (init->tx.txProfile.dacDiv == 2) {
			dacClockRateSelect = 1; /* DAC Clock = hsDigClk/2 */
		}
	}

	/* Rx settings */
	if (device->devStateInfo.profilesValid & RX_PROFILE_VALID) {
		if (init->rx.rxProfile.rxDec5Decimation == 5) {
			rxChannelSettings |= 0x02;
		}
		if ( init->rx.rxProfile.rhb1Decimation == 2) {
			rxChannelSettings |= 0x10;
		}
		switch (init->rx.rxProfile.rxFirDecimation) {
			case 1:
				rxFirDecBitField |= 0x01;
				break;
			case 2:
				rxFirDecBitField |= 0x02;
				break;
			case 4:
				rxFirDecBitField |= 0x03;
				break;
		}
	}

	/* Determine ORx channel settings */
	if (device->devStateInfo.profilesValid & ORX_PROFILE_VALID) {
		if ( init->obsRx.orxProfile.rxDec5Decimation == 5) {
			rxChannelSettings |= 0x02;
		}
		/* Enable ORX RHB2 Wideband */
		orxChannelSettings = 0x10;
		if (init->obsRx.orxProfile.rhb1Decimation == 2) {
			orxChannelSettings |= 0x04;
		}
		switch (init->obsRx.orxProfile.rxFirDecimation) {
			case 1:
				orxFirDecBitField |= 0x01;
				break;
			case 2:
				orxFirDecBitField |= 0x02;
				break;
			case 4:
				orxFirDecBitField |= 0x03;
				break;
		}
	}
	/************************************************************************
	* Set channel config settings for Rx/Tx/ and ORx channels
	************************************************************************/
		
#if defined(CODE_EN_FOR_RE)&&(CODE_EN_FOR_RE == 1)&&0
	/* Program FIR filters after CLKPLL locked */
	if (device->devStateInfo.profilesValid & TX_PROFILE_VALID) {
		retVal = (talRecoveryActions_t)BR3109_setTxPfirSyncClk(device, &init->tx.txProfile);
		IF_ERR_RETURN_U32(retVal);

		if (init->tx.txProfile.txFir.numFirCoefs > 0) {
			retVal = (talRecoveryActions_t)BR3109_programFir(device, TAL_TX1TX2_FIR, &init->tx.txProfile.txFir);
			IF_ERR_RETURN_U32(retVal);
		}
	}
	if (((device->devStateInfo.profilesValid & RX_PROFILE_VALID) > 0) ||
		((device->devStateInfo.profilesValid & ORX_PROFILE_VALID) > 0)) {
		retVal = (talRecoveryActions_t)BR3109_setRxPfirSyncClk(device,
				&init->rx.rxProfile, &init->obsRx.orxProfile);
		IF_ERR_RETURN_U32(retVal);
	}
		

	if (device->devStateInfo.profilesValid & RX_PROFILE_VALID) {
		if (init->rx.rxProfile.rxFir.numFirCoefs > 0) {
			retVal = (talRecoveryActions_t)BR3109_programFir(device, TAL_RX1RX2_FIR, &init->rx.rxProfile.rxFir);
			IF_ERR_RETURN_U32(retVal);
			/* Load Rx gain table */
			retVal = (talRecoveryActions_t)BR3109_programRxGainTable(device, &rxGainTable[0], (sizeof(rxGainTable) / sizeof(br3109RxGainTable_t)), TAL_RX1RX2);
			IF_ERR_RETURN_U32(retVal);

			retVal = (talRecoveryActions_t)BR3109_setRxManualGain(device, TAL_RX1, init->rx.rxGainCtrl.rx1GainIndex);
			IF_ERR_RETURN_U32(retVal);

			retVal = (talRecoveryActions_t)BR3109_setRxManualGain(device, TAL_RX2, init->rx.rxGainCtrl.rx2GainIndex);
			IF_ERR_RETURN_U32(retVal);
		}
	}
	if (device->devStateInfo.profilesValid & ORX_PROFILE_VALID) {
		if (init->obsRx.orxProfile.rxFir.numFirCoefs > 0) {
			/* if pointer to orx rxFIR is valid */
			retVal = (talRecoveryActions_t)BR3109_programFir(device, TAL_OBSRX1RX2_FIR, &init->obsRx.orxProfile.rxFir);
			IF_ERR_RETURN_U32(retVal);

			retVal = (talRecoveryActions_t)BR3109_setObsRxManualGain(device, TAL_ORX1, init->obsRx.orxGainCtrl.ORx1Atten_mdB);
			IF_ERR_RETURN_U32(retVal);

			retVal = (talRecoveryActions_t)BR3109_setObsRxManualGain(device, TAL_ORX2, init->obsRx.orxGainCtrl.ORx2Atten_mdB);
			IF_ERR_RETURN_U32(retVal);
		}
	}
#endif
		
#if 1
	// retVal = BR3109_armSpiCmd_Jesd_config(device, JESD_TX);
	// IF_ERR_RETURN_U32(retVal);
	/* If Valid Rx Profile or valid ObsRx profile, setup serializers */
	if ((device->devStateInfo.profilesValid & RX_PROFILE_VALID) || (device->devStateInfo.profilesValid & ORX_PROFILE_VALID)) {
		retVal = (talRecoveryActions_t)BR3109_setupSerializers(device, init);
		IF_ERR_RETURN_U32(retVal);
	}
		
	if ((device->devStateInfo.profilesValid & RX_PROFILE_VALID) && (init->rx.rxChannels != TAL_RXOFF)) {

		/* save the Np states for use in slicer gain compensation management */
		if ((init->rx.framerSel == TAL_FRAMER_A) || (init->rx.framerSel == TAL_FRAMER_A_AND_B)) {
			device->devStateInfo.rxFramerNp = init->jesd204Settings.framerA.Np;
		} else {
			device->devStateInfo.rxFramerNp = init->jesd204Settings.framerB.Np;
		}
		retVal = (talRecoveryActions_t)BR3109_setupJesd204bFramer(device, init, init->rx.framerSel);
		IF_ERR_RETURN_U32(retVal);

		/* set the ADC crossbar: Q/I data assigned to even/odd converters to account for I/Q swap in DUT */
		if (init->rx.rxChannels == TAL_RX2) {
			/* In real IF mode, send Rx2 I data to converter 0 */
			if ((init->rx.rxProfile.rxDdcMode == TAL_RXDDC_INT2_REALIF) || (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_DEC2_REALIF) ||
				(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_FILTERONLY_REALIF) || (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_BYPASS_REALIF)) {
				adcXbar.conv0 = TAL_ADC_RX2_I;
				adcXbar.conv1 = TAL_ADC_RX2_Q;
				adcXbar.conv2 = TAL_ADC_DUALBAND_RX2_BAND_B_I;
				adcXbar.conv3 = TAL_ADC_DUALBAND_RX2_BAND_B_Q;
				adcXbar.conv4 = TAL_ADC_RX1_I;
				adcXbar.conv5 = TAL_ADC_RX1_Q;
				adcXbar.conv6 = TAL_ADC_DUALBAND_RX1_BAND_B_I;
				adcXbar.conv7 = TAL_ADC_DUALBAND_RX1_BAND_B_Q;
			} else {
				adcXbar.conv0 = TAL_ADC_RX2_Q;
				adcXbar.conv1 = TAL_ADC_RX2_I;
				adcXbar.conv2 = TAL_ADC_DUALBAND_RX2_BAND_B_Q;
				adcXbar.conv3 = TAL_ADC_DUALBAND_RX2_BAND_B_I;
				adcXbar.conv4 = TAL_ADC_RX1_Q;
				adcXbar.conv5 = TAL_ADC_RX1_I;
				adcXbar.conv6 = TAL_ADC_DUALBAND_RX1_BAND_B_Q;
				adcXbar.conv7 = TAL_ADC_DUALBAND_RX1_BAND_B_I;
			}
		} else if (init->rx.rxChannels == TAL_RX1) {
			/* In real IF mode, send Rx1 I data to converter 0 */
			if ((init->rx.rxProfile.rxDdcMode == TAL_RXDDC_INT2_REALIF) || (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_DEC2_REALIF) ||
				(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_FILTERONLY_REALIF) || (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_BYPASS_REALIF)) {
				adcXbar.conv0 = TAL_ADC_RX1_I;
				adcXbar.conv1 = TAL_ADC_RX1_Q;
				adcXbar.conv2 = TAL_ADC_DUALBAND_RX1_BAND_B_I;
				adcXbar.conv3 = TAL_ADC_DUALBAND_RX1_BAND_B_Q;
				adcXbar.conv4 = TAL_ADC_RX2_I;
				adcXbar.conv5 = TAL_ADC_RX2_Q;
				adcXbar.conv6 = TAL_ADC_DUALBAND_RX2_BAND_B_I;
				adcXbar.conv7 = TAL_ADC_DUALBAND_RX2_BAND_B_Q;
			} else {
				adcXbar.conv0 = TAL_ADC_RX1_Q;
				adcXbar.conv1 = TAL_ADC_RX1_I;
				adcXbar.conv2 = TAL_ADC_DUALBAND_RX1_BAND_B_Q;
				adcXbar.conv3 = TAL_ADC_DUALBAND_RX1_BAND_B_I;
				adcXbar.conv4 = TAL_ADC_RX2_Q;
				adcXbar.conv5 = TAL_ADC_RX2_I;
				adcXbar.conv6 = TAL_ADC_DUALBAND_RX2_BAND_B_Q;
				adcXbar.conv7 = TAL_ADC_DUALBAND_RX2_BAND_B_I;
			}
		} else {
			/* In real IF mode, send Rx1 I data to converter 0, Rx2 I data on converter 1 */
			if ((init->rx.rxProfile.rxDdcMode == TAL_RXDDC_INT2_REALIF) || (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_DEC2_REALIF) ||
				(init->rx.rxProfile.rxDdcMode == TAL_RXDDC_FILTERONLY_REALIF) || (init->rx.rxProfile.rxDdcMode == TAL_RXDDC_BYPASS_REALIF)) {
				adcXbar.conv0 = TAL_ADC_RX1_I;
				adcXbar.conv1 = TAL_ADC_RX2_I;
				adcXbar.conv2 = TAL_ADC_RX1_Q;
				adcXbar.conv3 = TAL_ADC_RX2_Q;
				adcXbar.conv4 = TAL_ADC_DUALBAND_RX1_BAND_B_I;
				adcXbar.conv5 = TAL_ADC_DUALBAND_RX2_BAND_B_I;
				adcXbar.conv6 = TAL_ADC_DUALBAND_RX1_BAND_B_Q;
				adcXbar.conv7 = TAL_ADC_DUALBAND_RX2_BAND_B_Q;
			} else {
				adcXbar.conv0 = TAL_ADC_RX1_Q;
				adcXbar.conv1 = TAL_ADC_RX1_I;
				adcXbar.conv2 = TAL_ADC_RX2_Q;
				adcXbar.conv3 = TAL_ADC_RX2_I;
				adcXbar.conv4 = TAL_ADC_DUALBAND_RX1_BAND_B_Q;
				adcXbar.conv5 = TAL_ADC_DUALBAND_RX1_BAND_B_I;
				adcXbar.conv6 = TAL_ADC_DUALBAND_RX2_BAND_B_Q;
				adcXbar.conv7 = TAL_ADC_DUALBAND_RX2_BAND_B_I;
			}
		}
		retVal = (talRecoveryActions_t)BR3109_setupAdcSampleXbar(device,
				init->rx.framerSel, adcXbar);
		IF_ERR_RETURN_U32(retVal);

		/* Initialize DDC modes */
		retVal = talInitializeDdc(device, init);
		IF_ERR_RETURN(retVal);
	}
		

	if (device->devStateInfo.profilesValid & ORX_PROFILE_VALID) {
		/* save the Np states for use in slicer gain compensation management */
		if ((init->obsRx.framerSel == TAL_FRAMER_A) ||
			(init->obsRx.framerSel == TAL_FRAMER_A_AND_B)) {
			/* Only Setup JESD Framer if serializer Lanes are enabled for Framer A ORx path */
			if (init->jesd204Settings.framerA.serializerLanesEnabled > 0) {
				device->devStateInfo.orxFramerNp = init->jesd204Settings.framerA.Np;
				retVal = (talRecoveryActions_t)BR3109_setupJesd204bFramer(device, init,
						init->obsRx.framerSel);
				IF_ERR_RETURN_U32(retVal);
			}
		} else {
			/* Only Setup JESD Framer if serializer Lanes are enabled for Framer B ORx path */
			if (init->jesd204Settings.framerB.serializerLanesEnabled > 0) {
				device->devStateInfo.orxFramerNp = init->jesd204Settings.framerB.Np;
				retVal = (talRecoveryActions_t)BR3109_setupJesd204bFramer(device, init,
						init->obsRx.framerSel);
				IF_ERR_RETURN_U32(retVal);
			}
		}

		/* set the ADC crossbar: Q/I data assigned to even/odd converters to account for I/Q swap in DUT */
		adcXbar.conv0 = TAL_ADC_RX1_Q;
		adcXbar.conv1 = TAL_ADC_RX1_I;
		adcXbar.conv2 = TAL_ADC_RX2_Q;
		adcXbar.conv3 = TAL_ADC_RX2_I;
		adcXbar.conv4 = TAL_ADC_DUALBAND_RX1_BAND_B_Q;
		adcXbar.conv5 = TAL_ADC_DUALBAND_RX1_BAND_B_I;
		adcXbar.conv6 = TAL_ADC_DUALBAND_RX2_BAND_B_Q;
		adcXbar.conv7 = TAL_ADC_DUALBAND_RX2_BAND_B_I;
		retVal = (talRecoveryActions_t)BR3109_setupAdcSampleXbar(device, init->obsRx.framerSel, adcXbar);
		IF_ERR_RETURN_U32(retVal);
	}


	if ((device->devStateInfo.profilesValid & TX_PROFILE_VALID) &&
		(init->tx.txChannels != TAL_TXOFF)) {
		retVal = (talRecoveryActions_t)BR3109_setupDeserializers(device, init);
		IF_ERR_RETURN_U32(retVal);

		/* set the DAC sample crossbars */
		if (init->tx.deframerSel == TAL_DEFRAMER_A) {
			if ((init->tx.txChannels == TAL_TX1) || (init->tx.txChannels == TAL_TX1TX2)) {
				dacXbar.dacChanI = TAL_DEFRAMERA_OUT0;
				dacXbar.dacChanQ = TAL_DEFRAMERA_OUT1;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX1, dacXbar);
				IF_ERR_RETURN_U32(retVal);

				dacXbar.dacChanI = TAL_DEFRAMERA_OUT2;
				dacXbar.dacChanQ = TAL_DEFRAMERA_OUT3;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX2, dacXbar);
				IF_ERR_RETURN_U32(retVal);

			}

			if (init->tx.txChannels == TAL_TX2) {
				dacXbar.dacChanI = TAL_DEFRAMERA_OUT2;
				dacXbar.dacChanQ = TAL_DEFRAMERA_OUT3;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX1, dacXbar);
				IF_ERR_RETURN_U32(retVal);

				dacXbar.dacChanI = TAL_DEFRAMERA_OUT0;
				dacXbar.dacChanQ = TAL_DEFRAMERA_OUT1;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX2, dacXbar);
				IF_ERR_RETURN_U32(retVal);
			}
		}

		if (init->tx.deframerSel == TAL_DEFRAMER_B) {
			if ((init->tx.txChannels == TAL_TX1) || (init->tx.txChannels == TAL_TX1TX2)) {
				dacXbar.dacChanI = TAL_DEFRAMERB_OUT0;
				dacXbar.dacChanQ = TAL_DEFRAMERB_OUT1;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX1, dacXbar);
				IF_ERR_RETURN_U32(retVal);

				dacXbar.dacChanI = TAL_DEFRAMERB_OUT2;
				dacXbar.dacChanQ = TAL_DEFRAMERB_OUT3;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX2, dacXbar);
				IF_ERR_RETURN_U32(retVal);
			}

			if (init->tx.txChannels == TAL_TX2) {
				dacXbar.dacChanI = TAL_DEFRAMERB_OUT2;
				dacXbar.dacChanQ = TAL_DEFRAMERB_OUT3;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX1, dacXbar);
				IF_ERR_RETURN_U32(retVal);

				dacXbar.dacChanI = TAL_DEFRAMERB_OUT0;
				dacXbar.dacChanQ = TAL_DEFRAMERB_OUT1;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX2, dacXbar);
				IF_ERR_RETURN_U32(retVal);
			}
		}
		

		if (init->tx.deframerSel == TAL_DEFRAMER_A_AND_B) {
			if ((init->jesd204Settings.deframerA.M == 4) ||
				(init->jesd204Settings.deframerB.M == 4)) {
				return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
								  TAL_ERR_FRAMER_A_AND_B_INV_M_PARM, retVal, TALACT_ERR_CHECK_PARAM);
			} else if ((init->tx.txChannels == TAL_TX1TX2) &&
				   (init->jesd204Settings.deframerA.M == 2) &&
				   (init->jesd204Settings.deframerB.M == 2)) {
				dacXbar.dacChanI = TAL_DEFRAMERA_OUT0;
				dacXbar.dacChanQ = TAL_DEFRAMERA_OUT1;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX1, dacXbar);
				IF_ERR_RETURN_U32(retVal);

				dacXbar.dacChanI = TAL_DEFRAMERB_OUT0;
				dacXbar.dacChanQ = TAL_DEFRAMERB_OUT1;
				retVal = (talRecoveryActions_t)BR3109_setupDacSampleXbar(device, TAL_TX2, dacXbar);
				IF_ERR_RETURN_U32(retVal);
			}
		}
		

		if ((init->tx.deframerSel == TAL_DEFRAMER_A) ||
			(init->tx.deframerSel == TAL_DEFRAMER_A_AND_B)) {
			retVal = (talRecoveryActions_t)BR3109_setupJesd204bDeframer(device, init, TAL_DEFRAMER_A);
			IF_ERR_RETURN_U32(retVal);
		}

		if ((init->tx.deframerSel == TAL_DEFRAMER_B) ||
			(init->tx.deframerSel == TAL_DEFRAMER_A_AND_B)) {
			retVal = (talRecoveryActions_t)BR3109_setupJesd204bDeframer(device, init, TAL_DEFRAMER_B);
			IF_ERR_RETURN_U32(retVal);
		}
#endif
#if 0
		retVal = (talRecoveryActions_t)BR3109_setTxAttenuation(device, TAL_TX1,
				init->tx.tx1Atten_mdB);
		IF_ERR_RETURN_U32(retVal);

		retVal = (talRecoveryActions_t)BR3109_setTxAttenuation(device, TAL_TX2,
				init->tx.tx2Atten_mdB);
		IF_ERR_RETURN_U32(retVal);
#endif
	}


	/* Set gain update counter for 1ms. */
	/* Calculate AGC clock rate - Use same equation as used for txAttenClock_Hz in BR3109_initDigitalClocks*/
	if (device->devStateInfo.clocks.hsDigClkDiv4or5_Hz > 500000000) {
		agcClock_Hz = (device->devStateInfo.clocks.hsDigClkDiv4or5_Hz / 4); /* div 4 */
	} else if (device->devStateInfo.clocks.hsDigClkDiv4or5_Hz > 250000000) {
		agcClock_Hz = (device->devStateInfo.clocks.hsDigClkDiv4or5_Hz / 2); /* div 2 */
	} else {
		agcClock_Hz = device->devStateInfo.clocks.hsDigClkDiv4or5_Hz; /* div 1 */
	}

	gainUpdateCount = (agcClock_Hz / 1000);


	/* Options to disable Tx data if PLL unlocks */
	if ((init->tx.disTxDataIfPllUnlock != TAL_TXDIS_TX_NOT_DISABLED) &&
		(init->tx.disTxDataIfPllUnlock != TAL_TXDIS_TX_ZERO_DATA) &&
		(init->tx.disTxDataIfPllUnlock != TAL_TXDIS_TX_RAMP_DOWN_TO_ZERO)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_VERTXPFILE_INV_DISONPLLUNLOCK, retVal, TALACT_ERR_CHECK_PARAM);
	}


	/* Set Br3109 API state container information */
	device->devStateInfo.devState = (br3109States_t)(device->devStateInfo.devState | TAL_STATE_INITIALIZED);
	device->devStateInfo.initializedChannels = (init->rx.rxChannels & 3);
	device->devStateInfo.initializedChannels |= ((init->obsRx.obsRxChannelsEnable & 3) << 2);
	device->devStateInfo.initializedChannels |= ((init->tx.txChannels & 3) << 4);
	device->devStateInfo.txInputRate_kHz = init->tx.txProfile.txInputRate_kHz;
	device->devStateInfo.rxDdcMode = init->rx.rxProfile.rxDdcMode;

	/* Find the Rx Framer and save the Rx Framer M value */
	if (init->rx.framerSel == TAL_FRAMER_A) {
		device->devStateInfo.rxTotalM = init->jesd204Settings.framerA.M;
	} else if (init->rx.framerSel == TAL_FRAMER_B) {
		device->devStateInfo.rxTotalM = init->jesd204Settings.framerB.M;
	} else if (init->rx.framerSel == TAL_FRAMER_A_AND_B) {
		device->devStateInfo.rxTotalM = init->jesd204Settings.framerA.M + init->jesd204Settings.framerB.M;
	}

	device->devStateInfo.rxBandwidth_Hz = init->rx.rxProfile.rfBandwidth_Hz;
	device->devStateInfo.txBandwidth_Hz = init->tx.txProfile.rfBandwidth_Hz;
	device->devStateInfo.orxBandwidth_Hz = init->obsRx.orxProfile.rfBandwidth_Hz;

	/* If higher priority retVal has no error, allow possible lower priority warning to be returned */
	if (retVal == TALACT_NO_ACTION) {
		retVal = (talRecoveryActions_t)retValWarn;
	}
	(void)sysrefCfg;
	(void)gainUpdateCount;
	(void)digRefClock_MHz;
	(void)digDeviceClockDiv;
	(void)dacClockRateSelect;

//TODO
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_DIG_MUX_LOOPBACK_FIXDATA, 0, (0x1 << 16), 0);
	IF_ERR_RETURN_U32(retVal);
	// uint32_t regdat[4] = {0x00000C00, 0x00000C00, 0x00000C00, 0x00000C00};  //// 2.5dbs 衰减
	// retVal = BR3109_armMemoryCmd_blk_write(device, APB_TX_DGAIN_BASEADDR + 0x0C, regdat, 4);
	// IF_ERR_RETURN_U32(retVal);
//	Globle_conf_t config_data={0};//全局配置下发
//	config_data.rf_pll_Khz = init->clocks.clkPllVcoFreq_kHz;
//	retVal = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_ARM_GLOBLE_CONFIG_DATA, &config_data, (sizeof(config_data)+3)/4);

	return (uint32_t)retVal;
}


uint32_t BR3109_shutdown(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	retVal = (talRecoveryActions_t)BR3109_resetDevice(device);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getMultiChipSyncStatus(br3109Device_t *device,
				       uint8_t *mcsStatus)
{
	talRecoveryActions_t retVal   = TALACT_NO_ACTION;
	brHalErr_t          halError = BRHAL_OK;
	uint32_t regdata;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,  "BR3109_getMultiChipSyncStatus()\n");

	retVal   = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,  TALACT_WARN_RESET_LOG);
#endif
	/* null pointer check */
	if (mcsStatus == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,  TAL_ERR_CHECKGETMCS_STATUS_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_APB_CLK_RST_JESD_SYNC_DIV_EN, &regdata, 1);
		IF_ERR_RETURN_U32(retVal);
		*mcsStatus = (regdata >> 10) & 0x1;//sync one shot done
	}
	return (uint32_t)retVal;
}

uint32_t BR3109_enableMultichipSync(br3109Device_t *device, uint8_t enableMcs,  uint8_t *mcsStatus)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,  "BR3109_enableMultichipSync()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,  TALACT_WARN_RESET_LOG);
#endif

	if (enableMcs > 0) {
		retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_JESD_SYNC_DIV_EN, 1, 0x00000200, 9);
		IF_ERR_RETURN_U32(retVal);
	} else {
		retVal = (talRecoveryActions_t)BR3109_serializerReset(device);
		IF_ERR_RETURN_U32(retVal);
	}

	/* if mcsStatus is a valid pointer, return the MCS status */
	if (mcsStatus != NULL) {
		BR3109_getMultiChipSyncStatus(device, mcsStatus);
	}

	return (uint32_t)retVal;
}

uint32_t BR3109_enableMultichipRfLOPhaseSync(br3109Device_t *device, uint8_t enableDigTestClk)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_enableMultichipRfLOPhaseSync()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
//TODO

	return (uint32_t)retVal;
}

uint32_t BR3109_serializerReset(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_SerializerReset()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x3, 0x0000C000, 14);
	IF_ERR_RETURN_U32(retVal);

	/* Allow framer data to output to serializer (clear reset) */
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_APB_CLK_RST_FUNC_RESET, 0x0, 0x0000C000, 14);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_programFir(br3109Device_t *device, br3109firName_t filterToProgram, br3109Fir_t *firFilter)
{
// every coef = 16bit
// max num of slot = 64
#define DATABUF_SIZE 64 // 64 slots in 16 bit
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint16_t regOffset = 0;
	uint16_t bufOffset = 0;
	uint8_t wrLength = 0; // blk_wr_length, in 32 bit
	uint16_t i= 0;
	uint16_t dataArray[DATABUF_SIZE] = {0};

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_programFir()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* Error checking, #taps less than max number of taps allowed.*/
	if ((firFilter->numFirCoefs == 0) || (DATABUF_SIZE < firFilter->numFirCoefs)){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_PROGRAMFIR_COEFS_NULL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	uint32_t coefBitCnt = firFilter->numFirCoefs / 8; // RANGE [1,8]
	// 余数不为零, 向上取整
	if (firFilter->numFirCoefs % 8 != 0){
		coefBitCnt++;
	}
	uint8_t vldBit = 0;

	// 计算顺序写入的长度和开始地址
	wrLength = coefBitCnt * 4;
	regOffset = (DATABUF_SIZE - wrLength * 2) * 2;
	bufOffset = DATABUF_SIZE - wrLength * 2;

	while (coefBitCnt != 0){
		vldBit |= 1 << (8 - coefBitCnt); // reverse
		coefBitCnt--;
	}

	for (i= 0; i < firFilter->numFirCoefs; i++){
		dataArray[DATABUF_SIZE - 1 - i] = firFilter->coefs[firFilter->numFirCoefs - 1 - i]; // 对齐最高位
	}

	/* firBankSelect and assignment of maxNumTaps, nTapMul and address firAddr.*/
	switch (filterToProgram) {
		case TAL_TX1_FIR:
			// write FIR
			retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_TX_CH1_CFIR_RX_COEF_START + regOffset, (uint32_t*)&dataArray[bufOffset], wrLength);
			IF_ERR_RETURN_U32(retVal);
			// wr vld bits
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TX_CH1_CFIR_RX_COEF_VLD, vldBit, BR3109_ADDR_RX_COEF_VLD_MASK, BR3109_ADDR_RX_COEF_VLD_OFFSET);
			IF_ERR_RETURN_U32(retVal);
			break;
		case TAL_TX2_FIR:
			// write FIR
			retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_TX_CH2_CFIR_ORX_COEF_START + regOffset, (uint32_t*)&dataArray[bufOffset], wrLength);
			IF_ERR_RETURN_U32(retVal);
			// wr vld bits
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TX_CH2_CFIR_ORX_COEF_VLD, vldBit, BR3109_ADDR_ORX_COEF_VLD_MASK, BR3109_ADDR_ORX_COEF_VLD_OFFSET);
			IF_ERR_RETURN_U32(retVal);
			break;
		case TAL_TX1TX2_FIR:
			// write FIR
			retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_TX_CH1_CFIR_RX_COEF_START + regOffset, (uint32_t*)&dataArray[bufOffset], wrLength);
			IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_TX_CH2_CFIR_ORX_COEF_START + regOffset, (uint32_t*)&dataArray[bufOffset], wrLength);
			IF_ERR_RETURN_U32(retVal);
			// wr vld bits
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TX_CH1_CFIR_RX_COEF_VLD, vldBit, BR3109_ADDR_RX_COEF_VLD_MASK, BR3109_ADDR_RX_COEF_VLD_OFFSET);
			IF_ERR_RETURN_U32(retVal);
			// wr vld bits
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TX_CH2_CFIR_ORX_COEF_VLD, vldBit, BR3109_ADDR_ORX_COEF_VLD_MASK, BR3109_ADDR_ORX_COEF_VLD_OFFSET);
			IF_ERR_RETURN_U32(retVal);
			break;
		case TAL_RX1_FIR:
		case TAL_RX2_FIR:
		case TAL_RX1RX2_FIR:
			// write FIR
			retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_RX_CFIR_RX_COEF_START + regOffset, (uint32_t*)&dataArray[bufOffset], wrLength);
			IF_ERR_RETURN_U32(retVal);
			// wr vld bits
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_RX_CFIR_RX_COEF_VLD, vldBit, BR3109_ADDR_RX_COEF_VLD_MASK, BR3109_ADDR_RX_COEF_VLD_OFFSET);
			IF_ERR_RETURN_U32(retVal);
			break;
		case TAL_OBSRX1_FIR:
		case TAL_OBSRX2_FIR:
		case TAL_OBSRX1RX2_FIR:
			// write FIR
			retVal = BR3109_armMemoryCmd_blk_write(device, BR3109_ADDR_RX_CFIR_ORX_COEF_START + regOffset, (uint32_t*)&dataArray[bufOffset], wrLength);
			IF_ERR_RETURN_U32(retVal);
			// wr vld bits
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_RX_CFIR_ORX_COEF_VLD, vldBit, BR3109_ADDR_ORX_COEF_VLD_MASK, BR3109_ADDR_ORX_COEF_VLD_OFFSET);
			IF_ERR_RETURN_U32(retVal);
			break;

		default:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_PROGRAMFIR_INV_FIRNAME_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	return (uint32_t)retVal;
}

uint32_t BR3109_calculateDigitalClocks(br3109Device_t *device,  br3109DigClocks_t *digClocks)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t hsDivTimes10 = 25;
	uint8_t hsClkDivHsDigClk4or5 = 20;
	uint32_t localHsDigClkDiv2_Hz = 0;
	uint32_t localHsDigClk4or5_Hz = 0;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_calculateDigitalClocks()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,  TALACT_WARN_RESET_LOG);
#endif

	device->devStateInfo.clocks.clkPllVcoFreq_kHz = digClocks->clkPllVcoFreq_kHz;
	device->devStateInfo.clocks.clkPllHsDiv = digClocks->clkPllHsDiv;
	device->devStateInfo.clocks.deviceClock_kHz = digClocks->deviceClock_kHz;

	switch(digClocks->clkPllHsDiv) {
		case TAL_HSDIV_2:
			hsDivTimes10 = 20;
			hsClkDivHsDigClk4or5 = 10;
			break;
		case TAL_HSDIV_2P5:
			hsDivTimes10 = 25;
			hsClkDivHsDigClk4or5 = 10;
			break;
		case TAL_HSDIV_3:
			hsDivTimes10 = 30;
			hsClkDivHsDigClk4or5 = 15;
			break;
		case TAL_HSDIV_4:
			hsDivTimes10 = 40;
			hsClkDivHsDigClk4or5 = 20;
			break;
		case TAL_HSDIV_5:
			hsDivTimes10 = 50;
			hsClkDivHsDigClk4or5 = 20;
			break;
		default:
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,  TAL_ERR_CLKPLL_INV_HSDIV, retVal, TALACT_ERR_CHECK_PARAM );
	}

    localHsDigClkDiv2_Hz = (uint32_t)((DIV_U64((((uint64_t)(digClocks->clkPllVcoFreq_kHz) * 1000) >> 1), hsDivTimes10) * 10) >> 1);
	device->devStateInfo.clocks.hsDigClkDiv2_Hz = localHsDigClkDiv2_Hz;
	// localHsDigClk4or5_Hz = (uint32_t)((((DIV_U64)(digClocks->clkPllVcoFreq_kHz) * 1000) >> 1) / hsClkDivHsDigClk4or5);
	localHsDigClk4or5_Hz = (uint32_t)DIV_U64((((uint64_t)(digClocks->clkPllVcoFreq_kHz) * 1000) >> 1), hsClkDivHsDigClk4or5);
	device->devStateInfo.clocks.hsDigClkDiv4or5_Hz = localHsDigClk4or5_Hz;
	return (uint32_t)retVal;
}

/**
 * \brief Verifies the Rx profile members are valid (in range) and calculates HS Dig Clock require for the Rx Profile
 *
 * Private helper function to verify the Rx profile members are valid (in range)
 * and calculates HS Dig Clock require for the Rx Profile
 * If the Rx profile IQ data rate = 0, it is assumed that the Rx profile is
 * not used.  If Rx IQ data rate > 0, and Rx profile members are out of range,
 * the function stores the error code describing the error, and returns a
 * Recovery action to check the configuration settings.
 *
 * \pre This function is private and is not called directly by the user.
 *
 * \dep_begin
 * \dep{device->devHalInfo}
 * \dep_end
 *
 * \param device Structure pointer to Br3109 device data structure
 * \param rxProfile rxProfile settings to be verified
 * \param rxHsDigClk_kHz Return value of the calculated HS Dig Clock required by the Rx profile
 *
 * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval TALACT_NO_ACTION Function completed successfully, no action required
 */
static talRecoveryActions_t talVerifyRxProfile(br3109Device_t *device, br3109RxProfile_t *rxProfile, uint32_t *rxHsDigClk_kHz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint8_t ddcMultiply = 1;
	uint8_t ddcDivide = 1;

	*rxHsDigClk_kHz = 0;

	/********************************/
	/* Check for a valid Rx profile */
	/********************************/
	if ((rxProfile->rxOutputRate_kHz < 15000) || (rxProfile->rxOutputRate_kHz > 500000)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERRXPFILE_INV_IQRATE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((rxProfile->rfBandwidth_Hz < 15000000) || (rxProfile->rfBandwidth_Hz > 250000000)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERRXPFILE_INV_RFBW, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((rxProfile->rhb1Decimation != 1) && (rxProfile->rhb1Decimation != 2)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERRXPFILE_INV_RHB1, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((rxProfile->rxDec5Decimation != 4) && (rxProfile->rxDec5Decimation != 5)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERRXPFILE_INV_DEC5, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((rxProfile->rxFirDecimation != 1) &&(rxProfile->rxFirDecimation != 2) && (rxProfile->rxFirDecimation != 4)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERRXPFILE_INV_FIR, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((rxProfile->rxFir.coefs == NULL) && (rxProfile->rxFirDecimation != 1)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERRXPFILE_INV_COEF, retVal, TALACT_ERR_CHECK_PARAM);
	}

	switch(rxProfile->rxDdcMode) {
		case TAL_RXDDC_BYPASS_REALIF:
		case TAL_RXDDC_BYPASS:
			ddcMultiply = 1;
			ddcDivide = 1;
			break;
		case TAL_RXDDC_DEC2_REALIF:
		case TAL_RXDDC_DEC2:
			ddcMultiply = 2;
			ddcDivide = 1;
			break;
		case TAL_RXDDC_INT2_REALIF:
		case TAL_RXDDC_INT2:
			ddcMultiply = 1;
			ddcDivide = 2;
			break;
		case TAL_RXDDC_FILTERONLY_REALIF:
		case TAL_RXDDC_FILTERONLY:
			ddcMultiply = 1;
			ddcDivide = 1;
			break;
		default:
			return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERRXPFILE_INV_DDC, retVal, TALACT_ERR_CHECK_PARAM );
	}

	*rxHsDigClk_kHz = ( rxProfile->rxOutputRate_kHz * rxProfile->rxFirDecimation * rxProfile->rhb1Decimation * rxProfile->rxDec5Decimation * ddcMultiply ) / ddcDivide;

	return retVal;
}

/**
 * \brief Verifies the ORx profile members are valid (in range) in the init structure
 *
 * If the ORx profile IQ data rate = 0, it is assumed that the ORx profile is
 * not used.  If ORx IQ data rate > 0, and ORx profile members are out of range,
 * the function stores the error code describing the error, and returns a
 * Recovery action to check the configuration settings.
 *
 * \pre This function is private and is not called directly by the user.
 *
 * \dep_begin
 * \dep{device->devHalInfo}
 * \dep_end
 *
 * \param device Structure pointer to Br3109 device data structure
 * \param orxProfile orxProfile settings to be verified
 * \param orxHsDigClk_kHz Return value of the calculated HS Dig Clock required by the ORx profile
 *
 * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval TALACT_NO_ACTION Function completed successfully, no action required
 */
static talRecoveryActions_t talVerifyOrxProfile(br3109Device_t *device, br3109ORxProfile_t *orxProfile, uint32_t *orxHsDigClk_kHz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint8_t ddcMultiply = 1;
	uint8_t ddcDivide = 1;

	*orxHsDigClk_kHz = 0;

	/********************************/
	/* Check for a valid ORx profile */
	/********************************/
	if ((orxProfile->orxOutputRate_kHz < 15000) || (orxProfile->orxOutputRate_kHz > 500000)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERORXPFILE_INV_IQRATE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((orxProfile->rfBandwidth_Hz < 15000000) || (orxProfile->rfBandwidth_Hz > 460000000)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERORXPFILE_INV_RFBW, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((orxProfile->rhb1Decimation != 1) && (orxProfile->rhb1Decimation != 2)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERORXPFILE_INV_RHB1, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((orxProfile->rxDec5Decimation != 4) && (orxProfile->rxDec5Decimation != 5)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERORXPFILE_INV_DEC5, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((orxProfile->rxFirDecimation != 1) && (orxProfile->rxFirDecimation != 2) && (orxProfile->rxFirDecimation != 4)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERORXPFILE_INV_FIR, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((orxProfile->rxFir.coefs == NULL) && (orxProfile->rxFirDecimation != 1)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERORXPFILE_INV_COEF, retVal, TALACT_ERR_CHECK_PARAM);
	}

	switch(orxProfile->orxDdcMode) {
		case TAL_ORXDDC_DISABLED:
			ddcMultiply = 1;
			ddcDivide = 1;
			break;
		case TAL_ORXDDC_SUBSAMPLE_BY2:
			ddcMultiply = 2;
			ddcDivide = 1;
			break;
		default:
			return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERORXPFILE_INV_DDC, retVal, TALACT_ERR_CHECK_PARAM );
	}

	*orxHsDigClk_kHz = (orxProfile->orxOutputRate_kHz * orxProfile->rxFirDecimation *
			    orxProfile->rhb1Decimation * orxProfile->rxDec5Decimation * ddcMultiply) / ddcDivide;

	return retVal;
}

/**
 * \brief Verifies the Tx profile members are valid (in range) in the init structure
 *
 * If the Tx profile IQ data rate = 0, it is assumed that the Tx profile is
 * not used.  If Tx IQ data rate > 0, and Tx profile members are out of range,
 * the function stores the error code describing the error, and returns a
 * Recovery action to check the configuration settings.
 *
 * \pre This function is private and is not called directly by the user.
 *
 * \dep_begin
 * \dep{device->devHalInfo}
 * \dep_end
 *
 * \param device Structure pointer to Br3109 device data structure
 * \param txProfile txProfile settings to be verified
 * \param txHsDigClk_kHz Return value of the calculated HS Dig Clock required by the Tx profile
 *
 * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval TALACT_NO_ACTION Function completed successfully, no action required
 */
static talRecoveryActions_t talVerifyTxProfile(br3109Device_t *device, br3109TxProfile_t *txProfile, uint32_t *txHsDigClk_kHz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;

	*txHsDigClk_kHz = 0;
	/********************************/
	/* Check for a valid Tx profile */
	/********************************/

	if ((txProfile->txInputRate_kHz < 15000) || (txProfile->txInputRate_kHz > 500000)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERTXPFILE_INV_IQRATE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((txProfile->rfBandwidth_Hz < 15000000) || (txProfile->rfBandwidth_Hz > 460000000)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERTXPFILE_INV_RFBW, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((txProfile->thb1Interpolation != 1) && (txProfile->thb1Interpolation != 2)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERTXPFILE_INV_THB1, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((txProfile->thb2Interpolation != 1) && (txProfile->thb2Interpolation != 2)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERTXPFILE_INV_THB2, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((txProfile->thb3Interpolation != 1) && (txProfile->thb3Interpolation != 2)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERTXPFILE_INV_THB3, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((txProfile->txInt5Interpolation != 1) && (txProfile->txInt5Interpolation != 5)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERTXPFILE_INV_INT5, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Int5 and thb1/2/3 should be mutually exclusive */
	if (((txProfile->txInt5Interpolation == 5) && ((txProfile->thb1Interpolation != 1) ||
	      (txProfile->thb2Interpolation != 1) || (txProfile->thb3Interpolation != 1))) ||
	    ((txProfile->txInt5Interpolation == 1) && (txProfile->thb1Interpolation == 1) &&
	     (txProfile->thb2Interpolation == 1) && (txProfile->thb3Interpolation == 1))) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERTXPFILE_INV_HBMUTEX, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if ((txProfile->txFirInterpolation != 1) && (txProfile->txFirInterpolation != 2) && (txProfile->txFirInterpolation != 4)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERTXPFILE_INV_FIRIPL, retVal, TALACT_ERR_CHECK_PARAM);

	}

	if ((txProfile->txFir.coefs == NULL) && (txProfile->txFirInterpolation != 1)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERTXPFILE_INV_COEF, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if((txProfile->dacDiv != 1) && (txProfile->dacDiv != 2)) {
		return talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
					TAL_ERR_VERTXPFILE_INV_DACDIV, retVal, TALACT_ERR_CHECK_PARAM);
	}

	*txHsDigClk_kHz = (txProfile->txInputRate_kHz * txProfile->txFirInterpolation * txProfile->thb1Interpolation * txProfile->thb2Interpolation *
			   txProfile->thb3Interpolation * txProfile->txInt5Interpolation *  txProfile->dacDiv);

	return retVal;
}

uint32_t BR3109_verifyProfiles(br3109Device_t *device, br3109Init_t *init)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t rxHsDigClk_kHz = 0;
	uint32_t orxHsDigClk_kHz = 0;
	uint32_t txHsDigClk_kHz = 0;
	uint32_t hsDigClkDiv2_Hz = 0;
	uint32_t hsDigClk_kHz = 0;
	br3109RxProfile_t *rxProfile  = NULL;
	br3109TxProfile_t *txProfile = NULL;
	br3109ORxProfile_t *orxProfile  = NULL;

	if (device == NULL) {
		/* Can not write to log since log function requires device data structure */
		return (uint32_t)TALACT_ERR_CHECK_PARAM;
	}

	if (init == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_INV_NULL_INIT_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	device->devStateInfo.profilesValid = 0;

	if(init->tx.txChannels != TAL_TXOFF) {
		txProfile = &init->tx.txProfile;
		retVal = talVerifyTxProfile(device, txProfile, &txHsDigClk_kHz);
		IF_ERR_RETURN_U32(retVal);
	}

	if(init->rx.rxChannels != TAL_RXOFF) {
		rxProfile = &init->rx.rxProfile;
		retVal = talVerifyRxProfile(device, rxProfile, &rxHsDigClk_kHz);
		IF_ERR_RETURN_U32(retVal);
	}

	if((init->obsRx.obsRxChannelsEnable != TAL_ORXOFF) || (init->tx.txChannels != TAL_TXOFF)) {
		orxProfile = &init->obsRx.orxProfile;
		retVal = talVerifyOrxProfile(device, orxProfile, &orxHsDigClk_kHz);
		IF_ERR_RETURN_U32(retVal);
	}

	if ((init->clocks.rfPllPhaseSyncMode != TAL_RFPLLMCS_NOSYNC)  && (init->clocks.rfPllPhaseSyncMode != TAL_RFPLLMCS_INIT_AND_SYNC)
	    && (init->clocks.rfPllPhaseSyncMode != TAL_RFPLLMCS_INIT_AND_1TRACK) && (init->clocks.rfPllPhaseSyncMode != TAL_RFPLLMCS_INIT_AND_CONTTRACK)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_VERPFILE_INV_RFPLLMCSMODE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Verify HsDigClk needed by Rx/Tx/Orx profiles is possible based on CLKPLL
	 * settings
	 */

	hsDigClkDiv2_Hz = device->devStateInfo.clocks.hsDigClkDiv2_Hz;

	hsDigClk_kHz = (hsDigClkDiv2_Hz / 500); /* time 2 and convert from Hz to kHz */
	/* Verify Tx profile is valid */
	if (txHsDigClk_kHz > 0) {
		// if ((rxHsDigClk_kHz > 0) && (txHsDigClk_kHz != rxHsDigClk_kHz)) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_TXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		// if ((orxHsDigClk_kHz > 0) && (txHsDigClk_kHz != orxHsDigClk_kHz)) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_TXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		// if (hsDigClk_kHz != txHsDigClk_kHz) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_TXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		device->devStateInfo.profilesValid |= TX_PROFILE_VALID;
	}

	/* Verify Rx profile is valid */
	if (rxHsDigClk_kHz > 0) {
		// if ((txHsDigClk_kHz > 0) && (rxHsDigClk_kHz != txHsDigClk_kHz)) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_RXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		// if ((orxHsDigClk_kHz > 0) && (rxHsDigClk_kHz != orxHsDigClk_kHz)) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_RXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		// if (hsDigClk_kHz != rxHsDigClk_kHz) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_RXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		device->devStateInfo.profilesValid |= RX_PROFILE_VALID;
	}

	/* Verify ORx profile is valid */
	if (orxHsDigClk_kHz > 0) {
		// if ((txHsDigClk_kHz > 0) && (orxHsDigClk_kHz != txHsDigClk_kHz)) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_ORXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		// if ((rxHsDigClk_kHz > 0) && (orxHsDigClk_kHz != rxHsDigClk_kHz)) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_ORXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		// if (hsDigClk_kHz != orxHsDigClk_kHz) {
		// 	return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_VERPFILE_ORXHSCLK, retVal, TALACT_ERR_CHECK_PARAM);
		// }

		device->devStateInfo.profilesValid |= ORX_PROFILE_VALID;
	}
	return (uint32_t)retVal;
}

uint32_t BR3109_setSpiSettings(br3109Device_t *device,br3109SpiSettings_t *spi)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	// uint32_t data = 0;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "TALISE_setSpiSettings()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if (spi->MSBFirst == 0) {
		(uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERRHDL_HAL_SPI, retVal, TALACT_ERR_CHECK_PARAM);/* SPI bit is 1, only MSB*/
	}

	if (spi->autoIncAddrUp == 0) {
		(uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERRHDL_HAL_SPI, retVal, TALACT_ERR_CHECK_PARAM);/* SPI bit is 1, only autoIncAddrUp*/
	}

	retVal = BR3109_SetSPIPinMode(device, SPI1_SEL, spi->fourWireMode > 0? 1:0);
	IF_ERR_RETURN_U32(retVal);

	if (( spi->cmosPadDrvStrength != TAL_CMOSPAD_DRV_1X) &&
	    ( spi->cmosPadDrvStrength != TAL_CMOSPAD_DRV_2X) &&
	    ( spi->cmosPadDrvStrength != TAL_CMOSPAD_DRV_3X) &&
	    ( spi->cmosPadDrvStrength != TAL_CMOSPAD_DRV_4X) &&
	    ( spi->cmosPadDrvStrength != TAL_CMOSPAD_DRV_5X) &&
	    ( spi->cmosPadDrvStrength != TAL_CMOSPAD_DRV_6X) &&
	    ( spi->cmosPadDrvStrength != TAL_CMOSPAD_DRV_8X) &&
	    ( spi->cmosPadDrvStrength != TAL_CMOSPAD_DRV_10X)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETSPI_INV_CMOS_DRV_STR, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		/* Extract fields from this encoding when writing: {non_gpio_drv , spi_cmos_drv_select[3:0]} */
	}

	/* Check if SPI reads and writes work after changing settings */
	retVal = (talRecoveryActions_t)BR3109_verifySpiReadWrite (device);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_verifySpiReadWrite (br3109Device_t *device)
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	int i = 0;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_verifySpiReadWrite()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	uint32_t wbuf[5] = {0x12345678, 0x2, 0x3, 0x4, 0x5};
	uint32_t rbuf[5] = {0};
	uint32_t addr = 0x80000000;
    retVal = BR3109_writeArmMem(device, addr, wbuf, 5);
    IF_ERR_RETURN_U32(retVal);
    retVal = BR3109_readArmMem(device, addr, rbuf, 5, 1);
    IF_ERR_RETURN_U32(retVal);
    for(i = 0; i < 5; i++){
    	if(rbuf[i] != wbuf[i]){
    		// UART_Printf("[%d] rbuf	0x%08X,	wbuf	0x%08X\r\n",i, rbuf[i], wbuf[i]);
			return BRHAL_SPI_FAIL;
    	}
    }
	return BRHAL_OK;
}

uint32_t BR3109_initDigitalClocks(br3109Device_t *device, br3109DigClocks_t *clockSettings)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regda = 0;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_initDigitalClocks()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	if (clockSettings->deviceClock_kHz > 10000 && clockSettings->deviceClock_kHz <= 1000000) {
		regda = clockSettings->deviceClock_kHz*1000;
		retVal = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_REFCLK_FREQ,  &regda, 1);
		IF_ERR_RETURN_U32(retVal);
	} else {
		return (uint32_t)talApiErrHandler(device,TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_SCALEDDEVCLK_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	return (uint32_t)retVal;
}

uint32_t BR3109_setTxPfirSyncClk(br3109Device_t *device,  br3109TxProfile_t *txProfile)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	// brHalErr_t halError = BRHAL_OK;
	// static const uint8_t NUM_MULT_PER_ROW = 20;
	// static const uint8_t MAX_NUM_TAPS = 80;
	// uint8_t effectiveRows = 0;
	// uint8_t numRows = 0;
	// uint32_t dpClk_kHz = 0;
	// uint32_t syncClk_kHz = 0;
	// uint32_t hsDigClkDiv4or5_kHz = 0;
	// uint8_t syncDiv = 0;
//TODO

	return (uint32_t)retVal;
}

uint32_t BR3109_setRxPfirSyncClk(br3109Device_t *device,  br3109RxProfile_t *rxProfile, br3109ORxProfile_t *orxProfile)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	/* Write Rx/Orx PFIR SYNC Clock divider */
//TODO
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getApiVersion (br3109Device_t *device, uint32_t *siVer,  uint32_t *majorVer, uint32_t *minorVer, uint32_t *buildVer)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,  "BR3109_getApiVersion()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,  TALACT_WARN_RESET_LOG);
#endif

	if ((siVer == NULL) || (majorVer == NULL) || (minorVer == NULL) || (buildVer == NULL)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,  TAL_ERR_GETAPIVERSION_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	*siVer = TAL_CURRENT_SI_VERSION;
	*majorVer = TAL_CURRENT_MAJOR_VERSION;
	*minorVer = TAL_CURRENT_MINOR_VERSION;
	*buildVer = TAL_CURRENT_BUILD_VERSION;
	return (uint32_t)retVal;
}

uint32_t BR3109_getDeviceRev(br3109Device_t *device, uint32_t *revision)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getDeviceRev()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if (revision == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETDEVICEREV_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	halError = brSpiReadWord(device->devHalInfo, BR3109_ADDR_HW_REVISION, revision );
    retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
//TODO
	return (uint32_t)retVal;
}

uint32_t BR3109_getProductId(br3109Device_t *device, uint8_t *productId)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getProductId()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	if (productId == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETPRODUCTID_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	halError = brSpiReadWord(device->devHalInfo, BR3109_ADDR_HW_REVISION, &regdat );
    retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
	*productId = regdat;
	return (uint32_t)retVal;
}
