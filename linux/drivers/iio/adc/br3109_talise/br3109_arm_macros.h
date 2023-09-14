/**
 * \file br3109_arm_macros.h
 * \brief Contains Br3109 API miscellaneous macro definitions for ARM
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef _BR3109_ARM_MACROS_H
#define _BR3109_ARM_MACROS_H
#include "br3109_arm_spi_cmd.h"
#ifdef __cplusplus
extern "C" {
#endif

#define TX_PROFILE_VALID    0x01
#define RX_PROFILE_VALID    0x02
#define ORX_PROFILE_VALID   0x04

#define BR3109_JESD_FRAMERB_OFFSET 0x14
#define BR3109_JESD_DEFRAMERB_OFFSET 0x50
/* The mailbox interface 3 bit error flag has a subset of error flags
 * that are common to all ARM commands.
 */
#define TAL_NUM_COMMON_ARMERR_FLAGS                     0x04
#define TAL_NUM_COMMON_SETARMERR_FLAGS                  0x05
#define TAL_NUM_COMMON_GETARMERR_FLAGS                  0x05

#define TX_FIR_COEF_NUMS_MAX							32
#define RX_FIR_COEF_NUMS_MAX							64
#define ORX_FIR_COEF_NUMS_MAX							64
/* ARM Error Flag Source */
#define TAL_ARM_EFSRC_INITCALS                          0x10
#define TAL_ARM_EFSRC_SETRFPLL                          0x20
#define TAL_ARM_EFSRC_SETPENDING                        0x30
#define TAL_ARM_EFSRC_SETGPIOCTRL                       0x40
#define TAL_ARM_EFSRC_BOOTUP                           0x100

//#define BR3109_ARM_ABORT_OPCODE                         0x00
//#define BR3109_ARM_RUNINIT_OPCODE                       0x02
//#define BR3109_ARM_RADIOON_OPCODE                       0x04
//#define BR3109_ARM_WRITECFG_OPCODE                      0x06
//#define BR3109_ARM_READCFG_OPCODE                       0x08
//#define BR3109_ARM_SET_OPCODE                           0x0A
//#define BR3109_ARM_GET_OPCODE                           0x0C

#define BR3109_ARM_ORX1_TX_SEL0_SIGNALID                0x00
#define BR3109_ARM_ORX1_TX_SEL1_SIGNALID                0x01
#define BR3109_ARM_ORX2_TX_SEL0_SIGNALID                0x02
#define BR3109_ARM_ORX2_TX_SEL1_SIGNALID                0x03
#define BR3109_ARM_TXCAL_ENA_SIGNALID                   0x04
#define BR3109_ARM_CAL_UPDATE0_SIGNALID                 0x05
#define BR3109_ARM_CAL_UPDATE1_SIGNALID                 0x06
#define BR3109_ARM_CAL_UPDATE2_SIGNALID                 0x07

#if 0

#define BR3109_ARM_OBJECTID_TXBBF_INIT                  0x00
#define BR3109_ARM_OBJECTID_ADCTUNER_INIT               0x01
#define BR3109_ARM_OBJECTID_TIA_INIT                    0x02
#define BR3109_ARM_OBJECTID_DCOFFSET_INIT               0x03
#define BR3109_ARM_OBJECTID_TXATTENDELAY_INIT           0x04
#define BR3109_ARM_OBJECTID_RXGAINDELAY_INIT            0x05
#define BR3109_ARM_OBJECTID_ADCFLASH_INIT               0x06
#define BR3109_ARM_OBJECTID_PATHDELAY_INIT              0x07
#define BR3109_ARM_OBJECTID_TXLOLINT_INIT               0x08
#define BR3109_ARM_OBJECTID_TXLOLEXT_INIT               0x09
#define BR3109_ARM_OBJECTID_TXQEC_INIT                  0x0A
#define BR3109_ARM_OBJECTID_LBLODELAY_INIT              0x0B
#define BR3109_ARM_OBJECTID_LBRXQEC_INIT                0x0C
#define BR3109_ARM_OBJECTID_RXLODELAY_INIT              0x0D
#define BR3109_ARM_OBJECTID_RXQEC_INIT                  0x0E
#define BR3109_ARM_OBJECTID_ORXLODELAY_INIT             0x10
#define BR3109_ARM_OBJECTID_ORXQEC_INIT                 0x11
#define BR3109_ARM_OBJECTID_TXDAC_INIT                  0x12
#define BR3109_ARM_OBJECTID_ADCSTITCHING_INIT           0x13

#define BR3109_ARM_OBJECTID_RXQEC_TRACKING              0x20
#define BR3109_ARM_OBJECTID_ORXQEC_TRACKING             0x21
#define BR3109_ARM_OBJECTID_TXLOL_TRACKING              0x22
#define BR3109_ARM_OBJECTID_TXQEC_TRACKING              0x23
#define BR3109_ARM_OBJECTID_RXHD2_TRACKING              0x24
#define BR3109_ARM_OBJECTID_TEMP_SENSOR                 0x40
#define BR3109_ARM_OBJECTID_RSSI                        0x41
#define BR3109_ARM_OBJECTID_CAL_STATUS                  0x42
#define BR3109_ARM_OBJECTID_INITCAL_STATUS              0x43
#define BR3109_ARM_OBJECTID_GO_GET_GP_INT_SOURCE        0x46
#define BR3109_ARM_OBJECTID_GET_AUX_ADC_RESULT          0x47
#define BR3109_ARM_OBJECTID_GPIO_CTRL                   0x60
#define BR3109_ARM_OBJECTID_ORX_TXCAL_CTRL              0x61
#define BR3109_ARM_OBJECTID_TDD_TXATTEN_CTRL            0x62
#define BR3109_ARM_OBJECTID_RFPLL_LO_FREQUENCY          0x63
#define BR3109_ARM_OBJECTID_TRACKING_CAL_SUSPEND_RESUME 0x65
#define BR3109_ARM_OBJECTID_TRACKING_CAL_CTRL           0x66
#define BR3109_ARM_OBJECTID_TRACKING_CAL_PENDING        0x67
#define BR3109_ARM_OBJECTID_TRACKING_CAL_UPDATE         0x68
#define BR3109_ARM_OBJECTID_ARM_EXCEPTION               0x69
#define BR3109_ARM_OBJECTID_RFPLL_LOOP_FREQ             0x6B
#define BR3109_ARM_OBJECTID_FAST_FREQ_HOP_MODE          0x6D
#define BR3109_ARM_OBJECTID_FREQ_HOP                    0x6E
#define BR3109_ARM_OBJECTID_GS_AUX_ADC                  0x6F
#define BR3109_ARM_OBJECTID_ENABLE_CALS_GPINT           0x81
#define BR3109_ARM_OBJECTID_CAL_SCHEDULER               0x83
#define BR3109_ARM_OBJECTID_SYTEM_INFO                  0x86
#endif
#if 1
/* ARM memory */
#define BR3109_ADDR_ARM_START_PROG_ADDR					BR3109_RAM_BASE
#define BR3109_ADDR_ARM_END_PROG_ADDR					(BR3109_RAM_BASE+0x35000)
#define BR3109_ADDR_ARM_START_DATA_ADDR					(BR3109_RAM_BASE+0x35000)
//#define BR3109_ADDR_ARM_HOP_FREQ_MEM_ADDR               (BR3109_ADDR_ARM_START_DATA_ADDR+0x1C400)
#define BR3109_ADDR_ARM_END_DATA_ADDR					(BR3109_ADDR_ARM_START_DATA_ADDR+0x2B000)
#define BR3109_ADDR_ARM_VERSION                         BR3109_ADDR_HW_REVISION



/* ARM checksum registers */
#define BR3109_ADDR_ARM_BUILD_CHKSUM_ADDR				BR3109_ADDR_ARM_START_PROG_ADDR
#define BR3109_ADDR_ARM_CALC_CHKSUM_ADDR				BR3109_ADDR_ARM_START_PROG_ADDR
#else

#define BR3109_ADC_PROFILE_SIZE = 42;      /* number of uint16_t words per ADC profile */
#define BR3109_ADCPROFILE_NUM_CAP_COND_CURR_VALUES          16
#define BR3109_ADCPROFILE_NUM_BIAS_VBG_VALUES               22
#define BR3109_ADCPROFILE_NUM_DACG_VALUES                    2
#define BR3109_ADCPROFILE_NUM_ES1_VALUES                     1
#define BR3109_ADCPROFILE_NUM_ALTG_VALUES                    1

/* ARM Radio Status */
#define BR3109_ARM_RADIO_STATUS_POWERUP                 0
#define BR3109_ARM_RADIO_STATUS_READY                   1
#define BR3109_ARM_RADIO_STATUS_IDLE                    2
#define BR3109_ARM_RADIO_STATUS_RADIO_ON                3
#define BR3109_ARM_RADIO_STATUS_PROFILE_ERROR           4
#endif
#ifdef __cplusplus
}
#endif

#endif
