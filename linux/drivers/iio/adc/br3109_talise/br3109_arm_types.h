/**
 * \file br3109_arm_types.h
 * \brief Contains Br3109 ARM data types
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef BR3109_ARM_TYPES_H_
#define BR3109_ARM_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif


#pragma pack(push)
#pragma pack(1)
typedef struct{
	uint32_t rf_pll_Khz;
	uint8_t band;
}Globle_conf_t;


#pragma pack(pop)

#define TAL_ARM_SPI_FLAG		0x80000000
#define TAL_ARM_SPI_ID_POS		8
#define TAL_ARM_SPI_ID_MASK		0xFF00
#define TAL_ARM_SPI_ADDR_MASK	0xFF

typedef enum {
	TALAPI_ARMERR_BOOTUP_TIMEOUT_ERROR, /*!< Timed out waiting for ARM bootup to happen*/
	TALAPI_ARMERR_BOOTUP_IDLE,          /*!< ARM in IDLE mode after bootup*/
	TALAPI_ARMERR_BOOTUP_RADIO_ON,      /*!< ARM in RADIO_ON mode after bootup*/
	TALAPI_ARMERR_BOOTUP_PROFILE_ERROR, /*!< ARM Profile error during bootup*/
	TALAPI_ARMERR_BOOTUP_UNKNOWN_ERROR  /*!< ARM unknown error during bootup*/
} talApiArmErr_t;

typedef enum {
	TAL_ARM_BUILD_DEBUG,                /*!< ARM binary is Debug Object*/
	TAL_ARM_BUILD_TEST_OBJECT,          /*!< ARM binary is Test Object*/
	TAL_ARM_BUILD_RELEASE               /*!< ARM binary is Release*/
} br3109ArmBuildType_t;

typedef struct {
	uint8_t majorVer;                   /*!< The ARM Major revision*/
	uint8_t minorVer;                   /*!< The ARM Minor revision*/
	uint8_t rcVer;                      /*!< The release candidate version (build number)*/
	br3109ArmBuildType_t buildType;        /*!< What type of ARM binary build*/
} br3109ArmVersionInfo_t;

#ifdef __cplusplus
}
#endif

#endif /* BR3109_ARM_TYPES_H_ */
