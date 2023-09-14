/**
 * \file br3109_jesd204_types.h
 * \brief Contains Br3109 API JESD data types
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef BR3109_JESD204_TYPES_H_
#define BR3109_JESD204_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief Enum to select desired framer
 */
typedef enum {
	TAL_FRAMER_A = 0,       /*!< Framer A selection */
	TAL_FRAMER_B,           /*!< Framer B selection */
	TAL_FRAMER_A_AND_B      /*!< Used for cases where Rx1 uses one framer, Rx2 uses the second framer */
} br3109FramerSel_t;

/**
 *  \brief Enum to select desired deframer
 */
typedef enum {
	TAL_DEFRAMER_A = 0,    /*!< Deframer A selection */
	TAL_DEFRAMER_B,        /*!< Deframer B selection */
	TAL_DEFRAMER_A_AND_B   /*!< Used for cases where Tx1 uses one deframer, Tx2 uses the second deframer */
} br3109DeframerSel_t;

/**
 * \brief Enumerated list of DAC Sample Xbar options
 */
typedef enum {
	TAL_DEFRAMERA_OUT0 = 0x0,               /*!< Link 0, Deframer out 0 */
	TAL_DEFRAMERA_OUT1 = 0x1,               /*!< Link 0, Deframer out 1 */
	TAL_DEFRAMERA_OUT2 = 0x2,               /*!< Link 0, Deframer out 2 */
	TAL_DEFRAMERA_OUT3 = 0x3,               /*!< Link 0, Deframer out 3 */
	TAL_DEFRAMERB_OUT0 = 0x4,               /*!< Link 1, Deframer out 0 */
	TAL_DEFRAMERB_OUT1 = 0x5,               /*!< Link 1, Deframer out 1 */
	TAL_DEFRAMERB_OUT2 = 0x6,               /*!< Link 1, Deframer out 2 */
	TAL_DEFRAMERB_OUT3 = 0x7                /*!< Link 1, Deframer out 3 */
} br3109DacSampleXbarSelect_t;

/**
 * \brief Enumerated list of ADC Sample Xbar options
 */
typedef enum {
	TAL_ADC_RX1_I = 0x0,                    /*!<  Rx1 I data */
	TAL_ADC_RX1_Q = 0x1,                    /*!<  Rx1 Q data */
	TAL_ADC_RX2_I = 0x2,                    /*!<  Rx2 I data */
	TAL_ADC_RX2_Q = 0x3,                    /*!<  Rx2 Q data */
	TAL_ADC_DUALBAND_RX1_BAND_A_I = 0x0,    /*!<  Dualband Rx1 Band A I data */
	TAL_ADC_DUALBAND_RX1_BAND_A_Q = 0x1,    /*!<  Dualband Rx1 Band A Q data */
	TAL_ADC_DUALBAND_RX2_BAND_A_I = 0x2,    /*!<  Dualband Rx2 Band A I data */
	TAL_ADC_DUALBAND_RX2_BAND_A_Q = 0x3,    /*!<  Dualband Rx2 Band A Q data */
	TAL_ADC_DUALBAND_RX1_BAND_B_I = 0x4,    /*!<  Dualband Rx1 Band B I data */
	TAL_ADC_DUALBAND_RX1_BAND_B_Q = 0x5,    /*!<  Dualband Rx1 Band B Q data */
	TAL_ADC_DUALBAND_RX2_BAND_B_I = 0x6,    /*!<  Dualband Rx2 Band B I data */
	TAL_ADC_DUALBAND_RX2_BAND_B_Q = 0x7     /*!<  Dualband Rx2 Band B Q data */

} br3109AdcSampleXbarSelect_t;

/**
 * \brief Enum of possible Framer Test Data sources
 */
typedef enum {
	TAL_TEST_DATA_DISABLE =0,
	TAL_FTD_PRBS7,           		/*!< Framer test data PRBS7 data source */
	TAL_FTD_PRBS9,              	/*!< Framer test data PRBS9 data source */
	TAL_FTD_PRBS11,                 /*!< Framer test data PRBS11 data source */
	TAL_FTD_PRBS15,                 /*!< Framer test data PRBS15 data source */
	TAL_FTD_PRBS20,                 /*!< Framer test data PRBS20 data source */
	TAL_FTD_PRBS23,                  /*!< Framer test data PRBS23 data source */
	TAL_FTD_PRBS31,                  /*!< Framer test data PRBS31 data source */
	TAL_FTD_ADC_DATA,           /*!< Framer test data ADC data source */
	TAL_FTD_CHECKERBOARD,             /*!< Framer test data checkerboard data source*/

} br3109FramerDataSource_t;

/**
 *  \brief Enum of Framer test data injection points
 */
typedef enum {
	TAL_FTD_FRAMERINPUT = 0,        /*!< Framer test data injection point at framer input */
	TAL_FTD_SERIALIZER,             /*!< Framer test data injection point at serializer input */
	TAL_FTD_POST_LANEMAP            /*!< Framer test data injection point after lane mapping */

} br3109FramerInjectPoint_t;

/**
 * \brief Enum for JESD204B PRBS generated types
 */
typedef enum {
	TAL_PRBS_DISABLE = 0,       /*!< Deframer PRBS pattern disable */
	TAL_PRBS7,                  /*!< Deframer PRBS7 pattern select */
	TAL_PRBS9,                  /*!< Deframer PRBS9 pattern select */
	TAL_PRBS11,                 /*!< Deframer PRBS11 pattern select */
	TAL_PRBS15,                 /*!< Deframer PRBS15 pattern select */
	TAL_PRBS20,                 /*!< Deframer PRBS20 pattern select */
	TAL_PRBS23,                 /*!< Deframer PRBS23 pattern select */
	TAL_PRBS31                  /*!< Deframer PRBS31 pattern select */

} br3109DeframerPrbsOrder_t;

/**
 * \brief Enum for JESD204B deserializer / deframer PRBS selection
 */
typedef enum {
	TAL_PRBSCHECK_LANEDATA = 0, /*!< Check PRBS at deserializer lane output (does not require JESD204b link) */
	TAL_PRBSCHECK_SAMPLEDATA    /*!< Check PRBS at output of deframer (JESD204b deframed sample) */

} br3109DefPrbsCheckLoc_t;

/**
 * \brief Data structure to hold the DAC sample crossbar information
 */
typedef struct {
	br3109DacSampleXbarSelect_t	dacChanI;    /*!< Sample Crossbar select for I channel data*/
	br3109DacSampleXbarSelect_t	dacChanQ;    /*!< Sample Crossbar select for Q channel data*/
} br3109DacSampleXbar_t;

/**
 * \brief Data structure to hold the ADC sample crossbar information
 */
typedef struct {
	br3109AdcSampleXbarSelect_t	conv0;    /*!< Sample Crossbar select for converter 0*/
	br3109AdcSampleXbarSelect_t	conv1;    /*!< Sample Crossbar select for converter 1*/
	br3109AdcSampleXbarSelect_t	conv2;    /*!< Sample Crossbar select for converter 2*/
	br3109AdcSampleXbarSelect_t	conv3;    /*!< Sample Crossbar select for converter 3*/
	br3109AdcSampleXbarSelect_t	conv4;    /*!< Sample Crossbar select for converter 4*/
	br3109AdcSampleXbarSelect_t	conv5;    /*!< Sample Crossbar select for converter 5*/
	br3109AdcSampleXbarSelect_t	conv6;    /*!< Sample Crossbar select for converter 6*/
	br3109AdcSampleXbarSelect_t	conv7;    /*!< Sample Crossbar select for converter 7*/
} br3109AdcSampleXbar_t;

/**
 *  \brief Data structure to hold Br3109 JESD204b DeFramer ILAS configuration settings
 */
typedef struct {
	uint8_t DID;                          /*!< JESD204B Configuration Device ID for ILAS check */
	uint8_t BID;                          /*!< JESD204B Configuration Bank ID for ILAS check */
	uint8_t LID0;                         /*!< JESD204B Configuration starting Lane ID for ILAS check */
	uint8_t L;                            /*!< JESD204B Configuration L = lanes per data converter for ILAS check */
	uint8_t SCR;                          /*!< JESD204B Configuration scramble setting for ILAS check */
	uint8_t F;                            /*!< JESD204B Configuration F = octets per frame for ILAS check */
	uint8_t K;                            /*!< JESD204B Configuration K = frames per multiframe for ILAS check */
	uint8_t M;                            /*!< JESD204B Configuration M = number of data converters for ILAS check */
	uint8_t N;                            /*!< JESD204B Configuration N = data converter sample resolution for ILAS check */
	uint8_t CS;                           /*!< JESD204B Configuration CS = number of control bits transferred per sample per frame for ILAS check */
	uint8_t NP;                           /*!< JESD204B Configuration NP = JESD204B word size based on the highest resolution of the data converter for ILAS check */
	uint8_t S;                            /*!< JESD204B Configuration S = number of samples/data converter/frame for ILAS check */
	uint8_t CF;                           /*!< JESD204B Configuration CF = '0' = control bits appended to each sample, '1' = appended to end of frame for ILAS check */
	uint8_t HD;                           /*!< JESD204B Configuration HD = high density bit - samples are contained within lane (0) or divided over more than one lane (1) for ILAS check */
	uint8_t FCHK0;                        /*!< JESD204B Configuration checksum for ILAS check lane0 */
	uint8_t FCHK1;                        /*!< JESD204B Configuration checksum for ILAS check lane1 */
	uint8_t FCHK2;                        /*!< JESD204B Configuration checksum for ILAS check lane2 */
	uint8_t FCHK3;                        /*!< JESD204B Configuration checksum for ILAS check lane3 */
} br3109Jesd204bLane0Config_t;

/**
 *  \brief Data structure to hold Br3109 JESD204b Framer configuration settings
 */
typedef struct {
	uint8_t bankId;                     /*!< JESD204B Configuration Bank ID extension to Device ID. Range is 0..15 */
	uint8_t deviceId;                   /*!< JESD204B Configuration Device ID link identification number. Range is 0..255 */
	uint8_t lane0Id;                    /*!< JESD204B Configuration starting Lane ID. If more than one lane is used, each lane will increment from the Lane0 ID. Range is 0..31 */
	uint8_t M;                          /*!< Number of ADCs (0, 2, or 4) where 2 ADCs are required per receive chain (I and Q) */
	uint8_t K;                          /*!< Number of frames in a multiframe. Default = 32, F*K must be modulo 4. Where, F=2*M/numberOfLanes */
	uint8_t F;                          /*!< Number of bytes(octets) per frame (Valid 1, 2, 4, 8) */
	uint8_t Np;                         /*!< converter sample resolution (12, 16, 24) */
	uint8_t scramble;                   /*!< Scrambling off if framerScramble = 0, if framerScramble > 0 scrambling is enabled */
	uint8_t externalSysref;             /*!< External SYSREF select. 0 = use internal SYSREF(not currently valid), 1 = use external SYSREF */	//todo clk_reset 0x80 inter_refclk_sel[24]
	uint8_t serializerLanesEnabled;     /*!< Serializer lane select bit field. Where, [0] = Lane0 enabled, [1] = Lane1 enabled, etc */
	uint8_t serializerLaneCrossbar;     /*!< Lane crossbar to map framer lane outputs to physical lanes */
	uint8_t lmfcOffset;                 /*!< LMFC offset value for deterministic latency setting.  Range is 0..31 */
	uint8_t newSysrefOnRelink;          /*!< Flag for determining if SYSREF on relink should be set. Where, if > 0 = set, 0 = not set */	//todo delate
	uint8_t syncbInSelect;              /*!< Selects SYNCb input source. Where, 0 = use SYNCBIN0 for this framer, 1 = use SYNCBIN1 for this framer */
	uint8_t overSample;                 /*!< Selects framer bit repeat or oversampling mode for lane rate matching. Where, 0 = bitRepeat mode (changes effective lanerate), 1 = overSample (maintains same lane rate between ObsRx framer and Rx framer and oversamples the ADC samples) */	//todo delate
	uint8_t syncbInLvdsMode;            /*!< 1 - enable LVDS input pad with 100ohm internal termination, 0 - enable CMOS input pad */	//todo delate
	uint8_t syncbInLvdsPnInvert;        /*!< 0 - syncb LVDS PN not inverted, 1 - syncb LVDS PN inverted */	// 0x10 syncb_in_level[1]
	uint8_t enableManualLaneXbar;       /*!< 0 - Automatic Lane crossbar mapping, 1 - Manual Lane crossbar mapping (use serializerLaneCrossbar value with no checking) */
} br3109Jesd204bFramerConfig_t;

/**
* \brief Data structure to hold the settings for the deframer configuration
*/
typedef struct {
	uint8_t bankId;                     /*!< Extension to Device ID. Range is 0..15 */
	uint8_t deviceId;                   /*!< Link identification number. Range is 0..255 */
	uint8_t lane0Id;                    /*!< Lane0 ID. Range is 0..31 */
	uint8_t M;                          /*!< Number of DACs (0, 2, or 4) - 2 DACs per transmit chain (I and Q) */
	uint8_t K;                          /*!< Number of frames in a multiframe. Default = 32, F*K = modulo 4. Where, F=2*M/numberOfLanes */
	uint8_t scramble;                   /*!< Scrambling off if scramble = 0, if framerScramble > 0 scrambling is enabled */
	uint8_t externalSysref;             /*!< External SYSREF select. 0 = use internal SYSREF, 1 = external SYSREF */	//与rx 共用
	uint8_t deserializerLanesEnabled;   /*!< Deserializer lane select bit field. Where, [0] = Lane0 enabled, [1] = Lane1 enabled, etc */
	uint8_t deserializerLaneCrossbar;   /*!< Lane crossbar to map deframer lane outputs to physical lanes */
	uint8_t lmfcOffset;                 /*!< LMFC offset value to adjust deterministic latency. Range is 0..31 */
	uint8_t newSysrefOnRelink;          /*!< Flag for determining if SYSREF on relink should be set. Where, if > 0 = set, '0' = not set */	//todo delate
	uint8_t syncbOutSelect;             /*!< Selects deframer SYNCBOUT pin (0 = SYNCBOUT0, 1 = SYNCBOUT1) */
	uint8_t Np;                         /*!< converter sample resolution (12, 16) */
	uint8_t syncbOutLvdsMode;           /*!< 1 - enable LVDS output pad, 0 - enable CMOS output pad  */	//todo delate
	uint8_t syncbOutLvdsPnInvert;       /*!< 0 - syncb LVDS PN not inverted, 1 - syncb LVDS PN inverted */	//todo 0x18 syncb_out_level[14]
	uint8_t syncbOutCmosSlewRate;       /*!< 0 - fastest rise/fall times, 3 - slowest rise/fall times */	//todo delate
	uint8_t syncbOutCmosDriveLevel;     /*!< 0 - normal cmos drive level, 1 - double cmos drive level */	//todo delate
	uint8_t enableManualLaneXbar;       /*!< 0 - Automatic Lane crossbar mapping, 1 - Manual Lane crossbar mapping (use deserializerLaneCrossbar value with no checking) */
} br3109Jesd204bDeframerConfig_t;

/**
 *  \brief Data structure to hold Br3109 JESD204b Framer and Deframer configuration information
 */
typedef struct {
	br3109Jesd204bFramerConfig_t	framerA;       /*!< Framer A configuration data structure */
	br3109Jesd204bFramerConfig_t	framerB;       /*!< Framer B configuration data structure */
	br3109Jesd204bDeframerConfig_t	deframerA;   /*!< Deframer A configuration data structure */
	br3109Jesd204bDeframerConfig_t	deframerB;   /*!< Deframer A configuration data structure */	//todo delate
	uint8_t serAmplitude;                       /*!< Serializer amplitude setting. Default = 15. Range is 0..15 */	//todo 贾老师
	uint8_t serPreEmphasis;                     /*!< Serializer pre-emphasis setting. Default = 1 Range is 0..4 */	//todo 贾老师
	uint8_t serInvertLanePolarity;              /*!< Serializer Lane PN inversion select. Default = 0. Where, bit[0] = 1 will invert lane [0], bit[1] = 1 will invert lane 1, etc. */	//todo 贾老师
	uint8_t desInvertLanePolarity;              /*!< Deserializer Lane PN inversion select.  bit[0] = 1 Invert PN of Lane 0, bit[1] = Invert PN of Lane 1, etc */	//todo 贾老师
	uint8_t desEqSetting;                       /*!< Deserializer Equalizer setting. Applied to all deserializer lanes. Range is 0..2 (default 2, 0 = max boost) */	//todo 贾老师
	uint8_t sysrefLvdsMode;                     /*!< 1 - enable LVDS Input pad with 100ohm internal termination, 0 - enable CMOS input pad */	//todo 贾老师
	uint8_t sysrefLvdsPnInvert;                 /*!< 0 - sysref LVDS PN is not inverted, 1 - sysref LVDS PN is inverted */	//todo 贾老师
} br3109JesdSettings_t;


#ifdef __cplusplus
}
#endif

#endif /* BR3109_JESD204_TYPES_H_ */
