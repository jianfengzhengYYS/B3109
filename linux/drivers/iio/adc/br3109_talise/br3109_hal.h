/**
 * \file br3109_hal.h
 * \brief Contains prototypes and macro definitions for Private ADI HAL wrapper
 *        functions implemented in br3109_hal.c
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef BR3109_HAL_H_
#define BR3109_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "br_hal.h"

#define HAL_TIMEOUT_DEFAULT 100         /* 100ms */
#define HAL_TIMEOUT_NONE 0x0            /* Non-blocking */
#define HAL_TIMEOUT_INFINITE 0xFFFFFFFF /* Blocking */
#define HAL_TIMEOUT_MULT 2              /* HAL timeout worse-case factor */

/**
 * \brief Private wrapper function for brSpiReadField with error handling
 *
 * This function can be called any time after devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param addr 32-bit SPI address
 * \param fieldVal Pointer to word read-back value
 * \param mask Bit mask for desired field
 * \param startBit Starting bit position for field mask operation
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brSpiReadField(void *devHalInfo, uint32_t addr, uint32_t *fieldVal,
			    uint32_t mask, uint32_t startBit);

/**
 * \brief Private Wrapper function for brSpiWriteField with error handling
 *
 * This function can be called any time after the devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param addr 32-bit SPI address
 * \param fieldVal word value to be written
 * \param mask Bit mask for desired field
 * \param startBit Starting bit position for field mask operation
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brSpiWriteField(void *devHalInfo, uint32_t addr, uint32_t fieldVal,
			     uint32_t mask, uint32_t startBit);

/**
 * \brief Private Wrapper function for ADIHAL_writeToLog with error handling
 *
 * This function can be called any time after the devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param logLevel Enumerated type to distinguish logging level
 * \param errorCode Error code value to be written to log file
 * \param comment String pointer to error reporting comment string
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brWriteToLog(void *devHalInfo, brLogLevel_t logLevel,
			  uint32_t errorCode, const char *comment);

/**
 * \brief Private Wrapper function for brSpiWriteWord with error handling
 *
 * This function can be called any time after the devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param addr 32-bit SPI address
 * \param data word to be written to addr
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brSpiWriteWord(void *devHalInfo, uint32_t addr, uint32_t data);

/**
 * \brief Wrapper function for brSpiReadWord with error handling
 *
 * This function can be called any time after the devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param addr 32-bit SPI address
 * \param readdata Pointer to word value read from addr
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brSpiReadWord(void *devHalInfo, uint32_t addr, uint32_t *readdata);

/**
 * \brief Wrapper function for brSpiWriteWords with error handling
 *
 * This function can be called any time after the devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param addr 32-bit SPI address
 * \param data Pointer to word array to be written starting at addr
 * \param count Number of words to be written
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brSpiWriteWords(void *devHalInfo, uint32_t *addr, uint32_t *data,
			     uint32_t count);

/**
 * \brief Wrapper function for brSpiReadWords with error handling
 *
 * This function can be called any time after the devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param addr 32-bit SPI address
 * \param readdata Pointer to word array for storing read data starting at addr
 * \param count Number of words to be read
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brSpiReadWords(void *devHalInfo, uint32_t *addr, uint32_t *readdata,
			    uint32_t count);

/**
 * \brief Wrapper function for brSpiWriteWords with error handling
 *
 * This function can be called any time after the devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param addr 32-bit SPI address
 * \param data Pointer to word array to be written starting at addr
 * \param count Number of words to be written
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brSpiWriteWordsBlock(void *devHalInfo, uint32_t addr, uint32_t *data,
			     uint32_t count);

/**
 * \brief Wrapper function for brSpiReadWords with error handling
 *
 * This function can be called any time after the devHalInfo has been initialized
 * with valid settings by the user
 *
 * \dep_begin
 * \dep{devHalInfo}
 * \dep_end
 *
 * \param devHalInfo Pointer to device HAL information container
 * \param addr 32-bit SPI address
 * \param readdata Pointer to word array for storing read data starting at addr
 * \param count Number of words to be read
 *
 * \retval Returns brHalErr_t enumerated type
 */
brHalErr_t brSpiReadWordsBlock(void *devHalInfo, uint32_t addr, uint32_t *readdata,
			    uint32_t count);


#ifdef __cplusplus
}
#endif

#endif /* BR3109_HAL_H_ */
