/*!
 * \file br3109_types.h
 * \brief Contains Br3109 API configuration and run-time type definitions
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef BR3109_TYPES_H_
#define BR3109_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/kernel.h>
#include "br3109_rx_types.h"
#include "br3109_jesd204_types.h"
#include "br3109_tx_types.h"
#include "br3109_radioctrl_types.h"

int16_t GetIndexFromAtt(int16_t mdb);
int16_t GetAttFromIndex(int16_t index);
/**
 *  \brief Enum of possible HS divider settings
 */
typedef enum {
	TAL_HSDIV_2   = 0x0,    /*!< High speed clock divide by 2 setting */
	TAL_HSDIV_2P5 = 0x1,    /*!< High speed clock divide by 2.5 setting */
	TAL_HSDIV_3   = 0x2,    /*!< High speed clock divide by 3 setting */
	TAL_HSDIV_4   = 0x3,    /*!< High speed clock divide by 4 setting */
	TAL_HSDIV_5   = 0x4     /*!< High speed clock divide by 5 setting */
} br3109HsDiv_t;

/**
 * \brief Enum of ORx PLL selections
 */
typedef enum {
	TAL_OBSLO_AUX_PLL = 0,               /*!< Selects AUX PLL for Observation LO */
	TAL_OBSLO_RF_PLL,           /*!< Selects RF PLL for Observation LO */
} br3109ObsRxLoSource_t;

/**
 *  \brief Enum to set the desired FIR filter type for related functions
 */
typedef enum {
	TAL_TX1_FIR = 1,                        /*!< Tx1 FIR filter select */
	TAL_TX2_FIR = 2,                        /*!< Tx2 FIR filter select */
	TAL_TX1TX2_FIR = 3,                     /*!< Tx1 + Tx2 FIR filter select */
	TAL_RX1_FIR = 4,                        /*!< Rx1 FIR filter select */
	TAL_RX2_FIR = 8,                        /*!< Rx2 FIR filter select */
	TAL_RX1RX2_FIR = 12,                    /*!< Rx1 + Rx2 FIR filter select */
//	TAL_RX1RX2_LOOPBACK_FIR = 140,          /*!< Rx1 + Rx2 loopback FIR filter select */
	TAL_OBSRX1_FIR = 16,                    /*!< ORx1 FIR filter select */
	TAL_OBSRX2_FIR = 32,                    /*!< ORx2 FIR filter select */
	TAL_OBSRX1RX2_FIR = 48,                 /*!< ORx1 + ORx2 FIR filter select */
//	TAL_OBSRX1RX2_LOOPBACK_FIR = 176,       /*!< ORx1 + ORx2 loopback FIR filter select */
//	TAL_LOOPBACK_FIR = 128                  /*!< Loopback FIR filter select */
} br3109firName_t;

/**
 *  \brief Enum to select the proper gain for the FIR
 */
typedef enum {
	TAL_FIR_GAIN_NEG12_DB = -12, /*!< Br3109 FIR gain -12 */
	TAL_FIR_GAIN_NEG6_DB = -6,   /*!< Br3109 FIR gain -6 */
	TAL_FIR_GAIN_0_DB = 0,       /*!< Br3109 FIR gain 0 */
	TAL_FIR_GAIN_POS6_DB = 6     /*!< Br3109 FIR gain 6 */
} br3109FirGain_t;

/**
 *  \brief Data structure to hold Br3109 States
 */
typedef enum {
	TAL_STATE_POWERONRESET = 0x00,
	TAL_STATE_INITIALIZED  = 0x01,
	TAL_STATE_STREAMLOADED = 0x02,
	TAL_STATE_ARMLOADED    = 0x04,
	TAL_STATE_CALS_RUN     = 0x08,
	TAL_STATE_RADIOON      = 0x10
} br3109States_t;

/**
 *  \brief Enum of possible DDC Modes
 */
typedef enum {
	TAL_RXDDC_BYPASS = 0,      /*!< No Half Band Enabled */
	TAL_RXDDC_FILTERONLY,      /*!< Half Band Filters only */
	TAL_RXDDC_INT2,            /*!< Half Band Interpolation by 2 */
	TAL_RXDDC_DEC2,            /*!< Half Band Decimate by 2 */
	TAL_RXDDC_BYPASS_REALIF,   /*!< No Half Band Enabled */
	TAL_RXDDC_FILTERONLY_REALIF,     /*!< Half Band Filters only */
	TAL_RXDDC_INT2_REALIF,     /*!< Half Band Interpolation by 2 */
	TAL_RXDDC_DEC2_REALIF      /*!< Half Band Decimate by 2 */
} br3109RxDdc_t;

/**
 * \brief Enumerated list of DDC Modes in ORx datapath
 */
typedef enum {
	TAL_ORXDDC_DISABLED = 0x00,          /*!< ZIF Observation Receiver */
	TAL_ORXDDC_SUBSAMPLE_BY2             /*!< Subsample Received Observation signal */
} br3109OrxDdc_t;

/**
 * \brief Enumerated list of CMOS pads drive strength options
 */
typedef enum {
	/* Values encoded as {non_gpio_drv , spi_cmos_drv_select[3:0]} */
	TAL_CMOSPAD_DRV_1X  = 0x00,       /*!<  2.5pF load @ 65MHz */
	TAL_CMOSPAD_DRV_2X  = 0x10,       /*!<    5pF load @ 65MHz */
	TAL_CMOSPAD_DRV_3X  = 0x03,       /*!<  7.5pF load @ 65MHz */
	TAL_CMOSPAD_DRV_4X  = 0x11,       /*!<   10pF load @ 65MHz */
	TAL_CMOSPAD_DRV_5X  = 0x0F,       /*!< 12.5pF load @ 65MHz */
	TAL_CMOSPAD_DRV_6X  = 0x13,       /*!<   15pF load @ 65MHz */
	TAL_CMOSPAD_DRV_8X  = 0x17,       /*!<   20pF load @ 65MHz */
	TAL_CMOSPAD_DRV_10X = 0x1F        /*!<   25pF load @ 65MHz */
} br3109CmosPadDrvStr_t;

/**
 * \brief Enumerated list of options to disable Tx data when PLL unlocks
 */
typedef enum {
	TAL_TXDIS_TX_NOT_DISABLED       = 0x0,      /*!< Tx data is not disabled when PLL unlocks */
	TAL_TXDIS_TX_ZERO_DATA          = 0x1,      /*!< Tx data is zeroed immediately when PLL unlocks */
	TAL_TXDIS_TX_RAMP_DOWN_TO_ZERO  = 0x2       /*!< Tx data is ramped down to zero when PLL unlocks */
} br3109TxDataIfPllUnlock_t;

/**
 * \brief Enumerated list of RFPLL phase synchronization modes
 *
 * RFPLL Phase sync requires extra time to sync each time the RFPLL frequency
 * is changed. If RFPLL phase sync is not required, it may be desired to
 * disable the feature to allow the RFPLL to lock faster.
 *
 * Depending on the desired accuracy of the RFPLL phase sync, several options
 * are provided.
 */
typedef enum {
	TAL_RFPLLMCS_NOSYNC = 0,                /*!< Disable RFPLL phase synchronization */
	TAL_RFPLLMCS_INIT_AND_SYNC = 1,         /*!< Enable RFPLL phase sync init only */
	TAL_RFPLLMCS_INIT_AND_1TRACK = 2,       /*!< Enable RFPLL phase sync init and track once  */	//todo
	TAL_RFPLLMCS_INIT_AND_CONTTRACK = 3     /*!< Enable RFPLL phase sync init and track continuously */		//todo
} br3109RfPllMcs_t;

/**
 *  \brief Data structure to hold Br3109 FIR filter settings
 */
typedef struct {
	int8_t gain_dB;         /*!< Filter gain in dB */
	uint8_t numFirCoefs;    /*!< Number of coefficients in the FIR filter */
	int16_t *coefs;         /*!< A pointer to an array of filter coefficients */
} br3109Fir_t;

/**
 *  \brief Data structure to hold Br3109 Rx NCO shifter parameters for both BandA and BandB
 *  configurations.  Settings all four element values to zero for a band will disable the NCO
 *  for that shift.
 */
typedef struct {
	uint32_t bandAInputBandWidth_kHz;  /*!< BandWidth in khz of the BandA input signal */
	int32_t bandAInputCenterFreq_kHz;      /*!< Center Frequency in khz of the BandA input signal */
	int32_t bandANco1Freq_kHz;         /*!< BandA NCO1 Frequency shift in khz */
	int32_t bandANco2Freq_kHz;         /*!< BandA NCO2 Frequency shift in khz */

	uint32_t bandBInputBandWidth_kHz;  /*!< BandWidth in khz of the BandB input signal */
	int32_t bandBInputCenterFreq_kHz;  /*!< Center Frequency in khz of the BandB input signal */
	int32_t bandBNco1Freq_kHz;         /*!< BandB NCO1 Frequency shift in khz */
	int32_t bandBNco2Freq_kHz;         /*!< BandB NCO2 Frequency shift in khz */

} br3109RxNcoShifterCfg_t;

/**
 *  \brief Data structure to hold settings for the current Rx specific use case profile
 */
typedef struct {
	br3109Fir_t rxFir;			        /*!< Rx FIR filter structure */ //todo rx cfir
	uint8_t rxFirDecimation;            /*!< Rx FIR decimation (1,2,4) */	//todo rx 2^(rcfirout-rcfirin_sel)
	uint8_t rxDec5Decimation;           /*!< Decimation of Dec5 or Dec4 filter (5,4) */	//todo clk_ret_reg 0x3C 2[1:0] //rx path 0x1c  dec5 0 rhb3-1 1
	uint8_t rhb1Decimation;             /*!< RX Halfband1 (HB1) decimation. Can be either 1 or 2 */	//todo delay^^^^
	uint32_t rxOutputRate_kHz;          /*!< Rx Output data rate in kHz */	//todo delay^^^^
	uint32_t rfBandwidth_Hz;            /*!< Rx RF passband bandwidth for the profile */	//数字带宽
	uint32_t rxBbf3dBCorner_kHz;        /*!< Rx BBF (TIA) 3dB corner in kHz */		//todo 模拟rx带宽
	br3109RxDdc_t rxDdcMode;            /*!< Rx DDC mode */	//todo delay^^^^
	br3109RxNcoShifterCfg_t
	rxNcoShifterCfg; /*!< Rx NCO Shift parameters used for ZIF->RIF, CIF->ZIF */	//todo rx ifconv
} br3109RxProfile_t;

/**
 *  \brief Data structure to hold settings for the current ORx specific use case profile
 */
typedef struct {
	br3109Fir_t rxFir;                  /*!< ORx FIR filter structure */
	uint8_t rxFirDecimation;            /*!< ORx FIR decimation (1,2,4) */
	uint8_t rxDec5Decimation;           /*!< Decimation of Dec5 or Dec4 filter (5,4) */
	uint8_t rhb1Decimation;             /*!< ORX Halfband1 (HB1) decimation. Can be either 1 or 2 */
	uint32_t orxOutputRate_kHz;         /*!< ORx Output data rate in kHz */
	uint32_t rfBandwidth_Hz;            /*!< ORx RF passband bandwidth for the profile */
	uint32_t rxBbf3dBCorner_kHz;        /*!< ORx BBF (TIA) 3dB corner in kHz */
	br3109OrxDdc_t orxDdcMode;          /*!< ORx DDC mode */
} br3109ORxProfile_t;

/**
 *  \brief Data structure to hold settings for the current Tx specific use case profile
 */
typedef struct {
	uint8_t dacDiv;                     /*!< The divider used to generate the DAC clock (1,2)*/	//todo delate
	br3109Fir_t txFir;                  /*!< Tx FIR filter structure */
	uint8_t txFirInterpolation;         /*!< The TX digital FIR filter interpolation (1,2,4) */	//todo txpath 0xc cfir 2^(in -out)
	uint8_t thb1Interpolation;          /*!< Tx Halfband1 (HB1) filter interpolation (1,2) */	//todo thb1 bypass
	uint8_t thb2Interpolation;          /*!< Tx Halfband2 (HB2) filter interpolation (1,2) */	//todo thb2 bypass
	uint8_t thb3Interpolation;          /*!< Tx Halfband3 (HB3) filter interpolation (1,2) */	//todo thb3 bypass
	uint8_t txInt5Interpolation;        /*!< Tx Int5 filter interpolation (1,5) */	//todo 与rx dec5 相同
	uint32_t txInputRate_kHz;           /*!< Tx input data rate in kHz */	//todo thpath 0x8 din_vld_sel[2:0]
	uint32_t primarySigBandwidth_Hz;    /*!< Tx primary signal BW */	//todo delay^^^^^	
	uint32_t rfBandwidth_Hz;            /*!< Tx RF passband bandwidth for the profile */	//todo delay^^^^^
	uint32_t txBbf3dBCorner_kHz;        /*!< Tx BBF 3dB corner in kHz */	// tx 模拟带宽
} br3109TxProfile_t;

/**
 *  \brief Data structure to hold ORx gain control settings for initialization and during use
 */
typedef struct {
	uint16_t ORx1Atten_mdB;                    /*!< Initial and current ORx1 Attenuation */
	uint16_t ORx2Atten_mdB;                    /*!< Initial and current ORx2 Attenuation */
} br3109ORxGainControl_t;

/**
 *  \brief Data structure to hold Rx gain control settings for initialization and during use
 */
typedef struct {
	br3109GainMode_t gainMode;  /*!< Current Rx gain control mode setting */
	uint8_t rx1GainIndex;       /*!< Rx1 Gain Index, can be used in different ways for manual and AGC gain control */
	uint8_t rx2GainIndex;       /*!< Rx2 Gain Index, can be used in different ways for manual and AGC gain control */
	uint8_t rx1MaxGainIndex;    /*!< Max gain index for the currently loaded Rx1 Gain table */
	uint8_t rx1MinGainIndex;    /*!< Min gain index for the currently loaded Rx1 Gain table */
	uint8_t rx2MaxGainIndex;    /*!< Max gain index for the currently loaded Rx2 Gain table */
	uint8_t rx2MinGainIndex;    /*!< Min gain index for the currently loaded Rx2 Gain table */
} br3109RxGainControl_t;

/**
 *  \brief Data structure to hold Tx data path settings
 */
typedef struct {
	br3109TxProfile_t	txProfile;              /*!< Tx datapath profile, 3dB corner frequencies, and digital filter enables */
	br3109DeframerSel_t	deframerSel;          /*!< Br3109 JESD204b deframer select (Deframer A or B, or both) */
	br3109TxChannels_t	txChannels;            /*!< The desired Tx channels to enable during initialization */
	br3109TxAttenStepSize_t txAttenStepSize;  /*!< Tx Attenuation step size */
	uint16_t tx1Atten_mdB;                    /*!< Initial and current Tx1 Attenuation */
	uint16_t tx2Atten_mdB;                    /*!< Initial and current Tx2 Attenuation */
	br3109TxDataIfPllUnlock_t  disTxDataIfPllUnlock;   /*!< Options to disable Transmit data when the RFPLL unlocks */
} br3109TxSettings_t;

/**
 *  \brief Data structure to hold Rx data path settings
 */
typedef struct {
	br3109RxProfile_t	  rxProfile;        /*!< Rx datapath profile, 3dB corner frequencies, and digital filter enables */
	br3109FramerSel_t	  framerSel;        /*!< Rx JESD204b framer configuration enum */
	br3109RxGainControl_t rxGainCtrl;   /*!< Rx Gain control settings structure */
	br3109RxChannels_t    rxChannels;      /*!< The desired Rx Channels to enable during initialization */
} br3109RxSettings_t;

/**
 *  \brief Data structure to hold ORx data path settings
 */
typedef struct {
	br3109ORxProfile_t	orxProfile;               /*!< ORx datapath profile, 3dB corner frequencies, and digital filter enables. */
	br3109ORxGainControl_t	orxGainCtrl;         /*!< ObsRx gain control settings structure */
	br3109FramerSel_t	framerSel;                /*!< ObsRx JESD204b framer configuration structure */
	br3109ObsRxChannels_t	obsRxChannelsEnable;  /*!< The desired ObsRx Channel to enable during initialization */
	br3109ObsRxLoSource_t	obsRxLoSource;        /*!< Field not used, reserved for future use. The ORx mixers can use the RF_PLL or Aux_PLL */
} br3109ObsRxSettings_t;

/**
 *  \brief Data structure to hold digital clock settings
 */
typedef struct {
	uint32_t deviceClock_kHz;       /*!< CLKPLL and device reference clock frequency in kHz */
	uint32_t clkPllVcoFreq_kHz;     /*!< CLKPLL VCO frequency in kHz */	//todo delate
	br3109HsDiv_t  clkPllHsDiv;     /*!< CLKPLL high speed clock divider */	//todo delate
	uint8_t rfPllUseExternalLo;     /*!< 1= Use external LO input for RF PLL, 0 = use internal LO generation for RF PLL */	//todo 孙诗岩
	br3109RfPllMcs_t rfPllPhaseSyncMode;   /*!< Set RF PLL phase synchronization mode. Adds extra time to lock RF PLL when PLL frequency changed. See enum for options */	//todo delay^^^^
} br3109DigClocks_t;

/**
 * \brief Data structure used by the API run time state structure to hold the calculated
 *        digital clocks for the selected init profiles
 */
typedef struct {
	uint32_t deviceClock_kHz;       /*!< CLKPLL and device reference clock frequency in kHz */
	uint32_t clkPllVcoFreq_kHz;     /*!< CLKPLL VCO frequency in kHz */
	br3109HsDiv_t  clkPllHsDiv;     /*!< CLKPLL high speed clock divider */
	uint32_t hsDigClkDiv2_Hz;       /*!< Calculated in BR3109_initialize() digital clock used throughout API functions */
	uint32_t hsDigClkDiv4or5_Hz;    /*!< Calculated in BR3109_initialize() digital clock used throughout API functions */
	uint8_t rfPllUseExternalLo;     /*!< Stored version of the init struct setting (Ext LO input) to keep in the device data structure, 1= Use external LO input for RF PLL, 0 = use internal LO generation for RF PLL */
} br3109Clocks_t;

/**
 * \brief Data structure to hold SPI settings for all system device types
 */
typedef struct {
	uint8_t MSBFirst;                           /*!< 1 = MSBFirst*/
	uint8_t enSpiStreaming;                     /*!< Not Recommended - most registers in Br3109 API are not consecutive */
	uint8_t autoIncAddrUp;                      /*!< For SPI Streaming, set address increment direction. 1= next addr = addr+1, 0:addr = addr-1 */
	uint8_t fourWireMode;                       /*!< 1: Use 4-wire SPI, 0: 3-wire SPI (SDIO pin is bidirectional). NOTE: ADI's FPGA platform always uses 4-wire mode */
	br3109CmosPadDrvStr_t cmosPadDrvStrength;   /*!< Drive strength of CMOS pads when used as outputs (SDIO, SDO, GP_INTERRUPT, GPIO 1, GPIO 0) */	//TODO ???
} br3109SpiSettings_t;

/**
 * \brief Data structure to hold Br3109 device instance initialization settings
 */
typedef struct {
	br3109SpiSettings_t   spiSettings;     /*!< SPI settings data structure */
	br3109RxSettings_t    rx;              /*!< Rx settings data structure */
	br3109TxSettings_t    tx;              /*!< Tx settings data structure */
	br3109ObsRxSettings_t obsRx;           /*!< ObsRx settings data structure */
	br3109DigClocks_t     clocks;          /*!< Holds settings for CLKPLL and reference clock */
	br3109JesdSettings_t  jesd204Settings; /*!< Holds the JESD204B data link settings */
} br3109Init_t;

/**
 * \brief Data structure to hold Br3109 error callback function
 */
typedef struct {
	uint32_t errSrc;
	const char* (*callbackFunction)(uint32_t errSrc, uint32_t errCode);
} talErrorFunction_t;

#define BR3109_ERRORFUNCTIONTABLEMAX 2

/**
 * \brief Function jump table to hold a registry of error message functions for
 *        add on features.
 */
typedef struct {
	talErrorFunction_t talErrorFunctionTable[BR3109_ERRORFUNCTIONTABLEMAX];
} talErrFunctionTable_t;

/**
 * \brief Structure to hold FHM mode frequency range
 */
typedef struct {
	uint32_t fhmMinFreq_MHz; /*!< Lower limit for FHM hop frequency in MHz */
	uint32_t fhmMaxFreq_MHz; /*!< Upper limit for FHM hop frequency in MHz */
} talFrequencyHoppingRange_t;

/**
 * \brief Data structure to hold a Br3109 device instance status information
 */
typedef struct {
	br3109States_t	devState;          			/*!< Current device state of the part, i.e., radio on, radio off, arm loaded, etc., defined by deviceState enum */
	uint8_t initializedChannels;                /*!< Holds Rx/ORx/Tx channels that were initialized and calibrated for the current device */
	uint8_t profilesValid;            			/*!< Current device profilesValid bit field for use notification, i.e., Tx = 0x01, Rx = 0x02, Orx = 0x04 */
	uint32_t errSource;               			/*!< Current source of error returned */
	uint32_t errCode;                 			/*!< Current error code returned */
	br3109Clocks_t	clocks;            			/*!< Currently calculated Br3109 digital clocks for the selected init profiles */
	br3109RxGainControl_t rxGainCtrl;			/*!< Current device gain mode,which can be MGC, AGCFAST, and index */
	br3109TxAttenStepSize_t txAttenStepSize; 	/*!< Current tx Atten step size */
	uint32_t usedGpiopins;                      /*!< Each bit position 'N' indicates whether the GPIO 'N' is assigned to some function (if 1) or not (if 0) */
	uint16_t usedGpio3p3pins;                   /*!< Each bit position 'N' indicates whether the GPIO3.3 'N' is assigned to some function (if 1) or not (if 0) */
	uint8_t rxFramerNp;                         /*!< Rx Framer Np - converter sample resolution (12, 16, 24) */
	uint8_t orxFramerNp;                        /*!< ORx Framer Np - converter sample resolution (12, 16, 24) */
	uint32_t rxOutputRate_kHz;                  /*!< Rx Output data rate in kHz */
	uint32_t txInputRate_kHz;                   /*!< Tx input data rate in kHz */
	br3109RxDdc_t rxDdcMode;                    /*!< DDC Mode saved from initialization  */
	uint8_t rxDualBandEnabled;                  /*!< DualBand Mode saved from initialization for determining dualband status */
	uint8_t rxTotalM;                           /*!< Value of M saved from initialization  */
	uint32_t rxBandwidth_Hz;                    /*!< Rx Bandwidth from the current profile */
	uint32_t txBandwidth_Hz;                    /*!< Tx Bandwidth from the current profile */
	uint16_t tx1_s_gain;                   
	uint16_t tx2_s_gain;                    
	uint32_t orxBandwidth_Hz;                   /*!< ORx Bandwidth from the current profile */
	uint32_t rxBandwidth_SW_Hz;                    /*!< Rx Bandwidth from the current profile */
	uint32_t txBandwidth_SW_Hz;                    /*!< Tx Bandwidth from the current profile */
	uint32_t orxBandwidth_SW_Hz;                   /*!< ORx Bandwidth from the current profile */
	uint32_t swTest;                            /*!< Software testmode signal */
	uint32_t tracking_en;                            /*!< Software tracking enabled */
	uint8_t deviceSiRev;                        /*!< Br3109 silicon rev read during BR3109_initialize */
	talErrFunctionTable_t talErrFunctionTable;  /*!< Br3109  callback function table */
	talFrequencyHoppingRange_t talFhmFreqRange; /*!< Br3109 FHM frequency range */
	br3109FhmTriggerMode_t talFhmTriggerMode;   /*!< Br3109 frequency hop trigger mode */
	uint64_t talFhmInitHopFreq_Hz;              /*!< Initial Br3109 hop frequency */
	uint8_t talFhmMcsSync;                      /*!< Flag to indicate if MCS synchronization is enabled on frequency hopping */
} br3109Info_t;
/**
 * \brief Data structure to hold Br3109 device instance settings
 */
typedef struct {
	void *devHalInfo;               /*!< ADI_HAL Hardware layer settings pointer specific to this Br3109 instance */
	br3109Info_t devStateInfo;      /*!< Br3109 infomration container */
} br3109Device_t;

#ifdef __cplusplus
}
#endif

#endif
