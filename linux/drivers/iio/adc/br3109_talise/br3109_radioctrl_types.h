/**
 * \file br3109_radioctrl_types.h
 * \brief Contains Br3109 API Radio Control data types
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef BR3109_RADIOCTRL_TYPES_H_
#define BR3109_RADIOCTRL_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	uint32_t HW_REVISION;			   // 0x00
	uint32_t test_mux;				   // 0x04
	uint32_t gpioSrcCtrl_reg;		   // 0x08
	uint32_t usr_set_gpio_out;		   // 0x0C
	uint32_t usr_set_gpio_oe;		   // 0x10
	uint32_t usr_get_gpio_in;		   // 0x14
	uint32_t arm_gpio_out;			   // 0x18
	uint32_t arm_gpio_oe;			   // 0x1C
	uint32_t arm_gpio_in;			   // 0x20
	uint32_t reserve;				   // 0x24
	uint32_t prg_event_gpio_oe;		   // 0x28
	uint32_t prg_event_gpio_in;		   // 0x2C
	uint32_t gpio_irq_mask;			   // 0x30
	uint32_t gpio_irq;				   // 0x34
	uint32_t gpio_event_edge_type;	   // 0x38
	uint32_t gpio_event_polar;		   // 0x3C
	uint32_t gpio3v3SrcCtrl_reg;	   // 0x40
	uint32_t arm_gpio_3v3_out;		   // 0x44
	uint32_t arm_gpio_3v3_oe;		   // 0x48
	uint32_t arm_gpio_3v3_in;		   // 0x4C
	uint32_t gpio_3v3_irq_mask;		   // 0x50
	uint32_t gpio_3v3_irq;			   // 0x54
	uint32_t gpio_3v3_event_edge_type; // 0x58
	uint32_t gpio_3v3_event_polar;	   // 0x5C
	uint32_t monitor_mask;			   // 0x60
	uint32_t monitor_index;			   // 0x64
	uint32_t spi_sdio_bdir_off;		   // 0x68
	uint32_t orx_tx_sel_pin_sel;	   // 0x6C
	uint32_t rx_mgc_sel;			   // 0x70
	uint32_t orx_mgc_sel;			   // 0x74
	uint32_t tx_attn_sel;			   // 0x78
	uint32_t tx_attn_state_pin_sel;	   // 0x7C
	uint32_t pwm1_period_adj;		   // 0x80
	uint32_t pwm1_enable_adj;		   // 0x84
	uint32_t pwm2_period_adj;		   // 0x88
	uint32_t pwm2_enable_adj;		   // 0x8C
	uint32_t rx_mgc_read_back;		   // 0x90
	uint32_t orx_enable_gpio_sel;	   // 0x94
} BR3109_GPIO_TypeDef;
/**
 *  \brief Enum to select desired low voltage GPIO pin used by the API
 */
typedef enum {
	TAL_GPIO_00 = 0,
	TAL_GPIO_01,
	TAL_GPIO_02,
	TAL_GPIO_03,
	TAL_GPIO_04,
	TAL_GPIO_05,
	TAL_GPIO_06,
	TAL_GPIO_07,
	TAL_GPIO_08,
	TAL_GPIO_09,
	TAL_GPIO_10,
	TAL_GPIO_11,
	TAL_GPIO_12,
	TAL_GPIO_13,
	TAL_GPIO_14,
	TAL_GPIO_15,
	TAL_GPIO_16,
	TAL_GPIO_17,
	TAL_GPIO_18,
	TAL_GPIO_INVALID
} br3109GpioPinSel_t;

/**
 *  \brief Enum to select desired External LO output frequency = RFPLL VCO / 2 / ExtLoDiv
 *  Where ExtLoDiv is a power of 2 from (1, 2, 4, 8, 16, 32, 64)
 */
typedef enum {
	TAL_EXTLO_RFPLLVCO_DIV2 =  0,  /*!< External LO output in frequency range 3000 MHz    - 6000 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV4 =  1,  /*!< External LO output in frequency range 1500 MHz    - 3000 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV8 =  2,  /*!< External LO output in frequency range  750 MHz    - 1500 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV16 = 3,  /*!< External LO output in frequency range  375 Mhz    -  750 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV32 = 4,  /*!< External LO output in frequency range  187.5 MHz  -  375 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV64 = 5   /*!< External LO output in frequency range   93.75 MHz -  187.5 MHz */
} br3109ExtLoDiv_t;
/**
 * \brief Data structure to hold ARM GPIO pin assignments, polarity, and pin enable for the Br3109 ARM
 */
typedef struct {
	br3109GpioPinSel_t gpioPinSel; /*!< Select desired GPIO pin to input into Br3109 (valid 0-15) */
	uint8_t polarity;           /*!< Signal polarity (0 = Normal polarity, 1=Br3109 will invert the signal before using) */
	uint8_t enable;             /*!< 1 = Enable Br3109 ARM use of GPIO signal, 0 = Br3109 ARM uses ARM command to set this signal value */
} br3109ArmGpioPinSettings_t;

/**
 * \brief set gpio pin select, added on 2023.03.15 for 3109ES2, gpio_reg format 5 bit 
 * gpio reg format:
 * --------------------------------------------------------------------------------------
 * |	PIN_SEL_VAL (size: 4 bit, range: 0~15)	|	ENABLE (size: 1 bit, range: 0~1)	|
 * --------------------------------------------------------------------------------------
 */
typedef struct {
	br3109GpioPinSel_t gpioPinSel; /*!< Select desired GPIO pin to input into Br3109 (valid 0-15) */
	uint8_t enable;             /*!< 1 = Enable Br3109 ARM use of GPIO signal, 0 = Br3109 ARM uses ARM command to set this signal value */
} br3109GpioShortPinSel_t;


/**
 * \brief Data structure to hold ARM GPIO pin assignments for each ARM input/output pin.
 */
typedef struct {
	/* Br3109 ARM input GPIO pins -- Only valid if orxPinMode = 1 */
	br3109ArmGpioPinSettings_t orx1TxSel0Pin;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	br3109ArmGpioPinSettings_t orx1TxSel1Pin;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	br3109ArmGpioPinSettings_t orx2TxSel0Pin;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	br3109ArmGpioPinSettings_t orx2TxSel1Pin;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	br3109ArmGpioPinSettings_t enTxTrackingCals;   /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
} br3109ArmGpioConfig_t;

/**
 * \brief Data structure to hold BR3109 GPIO pin assignments for each input/output pin.
 * updated on 2023.03.15 for BR3109ES2
 */
typedef struct {
	/* Br3109 ARM input GPIO pins -- Only valid if orxPinMode = 1 */
	br3109GpioShortPinSel_t p0;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	br3109GpioShortPinSel_t p1;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	br3109GpioShortPinSel_t p2;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	br3109GpioShortPinSel_t p3;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
} br3109GpioShortConfig_t;

/**
 * \brief Data structure to hold BR3109 GPIO pin assignments for each input/output pin.
 * updated on 2023.03.15 for BR3109ES2
 */
typedef struct {
	/* Br3109 ARM input GPIO pins -- Only valid if orxPinMode = 1 */
	br3109GpioShortPinSel_t orx1_enable_gpio_sel;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	uint8_t orx1_enble_sel;      /*!< 0/1 */
	br3109GpioShortPinSel_t orx2_enable_gpio_sel;      /*!< Select desired GPIO pin to input into Br3109(valid 0-15), polarity, enable */
	uint8_t orx2_enble_sel;      /*!< 0/1 */
} br3109GpioConfigOrxGpio_t;


/**
 * \brief Enumerated list of Radio Control Config2 ORx1/ORx2 GPIO pin pair settings
 */
typedef enum {
	TAL_ORX1ORX2_PAIR_01_SEL = 0x00,   /*!< Radio Control Config 2 ORx1/ORx2 GPIO '0,1' pin pair select */
	TAL_ORX1ORX2_PAIR_45_SEL,          /*!< Radio Control Config 2 ORx1/ORx2 GPIO '4,5' pin pair select */
	TAL_ORX1ORX2_PAIR_89_SEL,          /*!< Radio Control Config 2 ORx1/ORx2 GPIO '8,9' pin pair select */
	TAL_ORX1ORX2_PAIR_NONE_SEL         /*!< Radio Control Config 2 ORx1/ORx2 GPIO 'none' pin pair select */

} br3109RadioCtlCfg2_t;

/**
 * \brief Enumerated list of Radio Control Config1 register bits
 */
typedef enum {
	TAL_TXRX_PIN_MODE = 0x01,       /*!< Radio Control Config 1 bit '0' mask */
	TAL_ORX_PIN_MODE = 0x02,        /*!< Radio Control Config 1 bit '1' mask */
	TAL_ORX_USES_RX_PINS = 0x04,    /*!< Radio Control Config 1 bit '2' mask */
	TAL_ORX_SEL = 0x10,             /*!< Radio Control Config 1 bit '4' mask */
	TAL_ORX_SINGLE_CHANNEL = 0x20,  /*!< Radio Control Config 1 bit '5' mask */
	TAL_ORX_ENAB_SEL_PIN = 0x40     /*!< Radio Control Config 1 bit '6' mask */
} br3109RadioCtlCfg1_t;

/**
 *  \brief Enum of possible Rx and ORx stream processor enables to be used with
 *         the BR3109_setRxTxEnable() function
 */
typedef enum {
	TAL_RXOFF_EN    = 0x00,  /*!< All Rx/ORx channels off */
	TAL_RX1_EN      = 0x01,  /*!< Rx1 channel enabled */
	TAL_RX2_EN      = 0x02,  /*!< Rx2 channel enabled */
	TAL_RX1RX2_EN   = 0x03,  /*!< Rx1 + Rx2 channels enabled */
	TAL_ORX1_EN     = 0x04,  /*!< ORx1 channel enabled */
	TAL_ORX2_EN     = 0x08,  /*!< ORx2 channel enabled */
	TAL_ORX1ORX2_EN = 0x0C,	 /*!< ORx1 and ORx2 channels enabled - only allowed if ADC stitching is not enabled */

} br3109RxORxChannels_t;

/**
 * \brief Enumerated list of Tx to ORx mapping selections
 */
typedef enum {
	TAL_MAP_NONE = 0,           /*!< No Tx to ORx mapping select */
	TAL_MAP_TX1_ORX = 0x01,     /*!< Tx1 to ORx mapping select */
	TAL_MAP_TX2_ORX = 0x02      /*!< Tx2 to ORx mapping select */

} br3109TxToOrxMapping_t;
	
/**
 *  \brief Enum of PLL selections
 */
typedef enum {
	TAL_CLK_PLL = 0,                /*!< Selects CLK PLL for Rx and Tx */
	TAL_RF_PLL,                     /*!< Selects RF PLL for Rx and Tx */
	TAL_ORF_PLL                     /*!< Selects AUX PLL for Rx and tx*/

} br3109RfPllName_t;
	
/**
 * \brief Structure to setup/enable ORx LO Selection feature
 */
typedef struct {
	uint8_t disableAuxPllRelocking; /*!< Disables the ARM from automatically relocking the Aux PLL.
                                         Set to 1 when using AuxLO as ORx LO source, 0 = default when RFPLL used as ORx LO source */
	br3109GpioPinSel_t
	gpioSelect;  /*!< TAL_GPIO_INVALID = disable pin mode, GPIO0-15 valid */
} br3109OrxLoCfg_t;

/**
 *  \brief Enum of FHM Exit Mode Selections
 */
typedef enum {
	TAL_FHM_QUICK_EXIT = 0,  /*!< Selects quick exit mode on frequency hopping disable. In this case RF PLL bandwidth is left unchanded */
	TAL_FHM_FULL_EXIT        /*!< Selects full exit mode on frequency hopping disable. RF PLL Loop B/W is restored to narrowband.
                                  RF and Aux PLLs recalibrated and tracking cals resumed. */
} br3109FhmExitMode_t;

/**
 *  \brief Enum of FHM Trigger Mode Selections
 */
typedef enum {
	TAL_FHM_GPIO_MODE = 0,       /*!< Selects FHM trigger mode as GPIO. A low to high pulse triggers frequency hop */
	TAL_FHM_NON_GPIO_MODE,       /*!< Selects FHM trigger mode as non-GPIO. An ARM command triggers frequency hop */
	TAL_FHM_INVALID_TRIGGER_MODE
} br3109FhmTriggerMode_t;

/**
 * \brief Structure to setup br3109 frequency hopping config
 */
typedef struct {
	br3109GpioPinSel_t
	fhmGpioPin; /*!< Maps the Br3109 ARM GPIO pin(TAL_GPIO_0 - TAL_GPIO_15) for frequency hopping. A low to high pulse on this pin triggers freq hopping
                                        Setting fhmGpioPin = TAL_GPIO_INVALID will unassign ARM GPIO pin mapped to Rf Pll Frequency hopping*/
	uint32_t fhmMinFreq_MHz;       /*!< Sets frequency hopping range minimum frequency */
	uint32_t fhmMaxFreq_MHz;       /*!< Sets frequency hopping range maximum frequency */
} br3109FhmConfig_t;

/**
 * \brief Structure to setup br3109 frequency hopping mode settings
 */
typedef struct {
	uint8_t fhmEnable;                      /*!< 0 - Disables Frequency Hopping, 1 - Enables Frequency Hopping */
	uint8_t enableMcsSync;                  /*!< 0 - Disables MCS Synchronization on FHM enable, 1 - Enables MCS Synchronization on FHM enable. Ignored if fhmEnable = 0 */
	br3109FhmTriggerMode_t
	fhmTriggerMode;  /*!< TAL_FHM_GPIO_MODE - Frequency Hop triggered via GPIO low to high pulse
                                                 TAL_FHM_NON_GPIO_MODE - Frequency Hop triggered via ARM command*/
	br3109FhmExitMode_t
	fhmExitMode;        /*!< TAL_FHM_QUICK_EXIT = quick exit on frequency hopping disable,
                                                 TAL_FHM_FULL_EXIT = Full exit on frequency hopping disable. This is ignored if fhmEnable = 1*/
	uint64_t fhmInitFrequency_Hz;           /*!< First hop frequency that Rf Pll is configured to on enabling FHM */
} br3109FhmMode_t;

/**
 * \brief Structure to read br3109 frequency hopping mode status
 */
typedef struct {
	uint16_t currentFhmCmdErrorStatus;     /*!< Current FHM Enter Command Error Status. This is same as the mailbox command error status for FHM */
	uint16_t currentFhmHopErrorStatus;     /*!< Currently active FHM errors during frequency hopping */
	uint32_t numFhmHops;                   /*!< Total no. of Hops since entering FHM */
	uint32_t numFhmNoErrorEvents;          /*!< Total no. of NO FHM error events */
	uint64_t lastFhmNoErrorFreq_Hz;        /*!< Last frequency for which NO Error was encountered */
	uint32_t numFhmHopsOutsideScanRange;   /*!< Total no. of Hops outside FHM scan range */
	uint64_t lastFreqOutsideScanRange_Hz;  /*!< Last frequency which was outside FHM scan range*/
	uint32_t numInvalidFhmHopFrequencies;  /*!< Invalid Hop Freq */
	uint64_t lastInvalidHopFreq_Hz;        /*!< Last Invalid Hop Freq */
	uint32_t compPllError;                 /*!< PLL LO Computation Error */
	uint64_t compPllErrorFreq_Hz;          /*!< PLL LO Computation Error frequency */
	uint32_t rfPllLockFailed;              /*!< RF PLL Lock failed */
	uint64_t rfPllLockFailedFreq_Hz;       /*!< RF PLL Lock failed frequency*/
} br3109FhmStatus_t;

#ifdef __cplusplus
}
#endif

#endif /* BR3109_RADIOCTRL_TYPES_H_ */
