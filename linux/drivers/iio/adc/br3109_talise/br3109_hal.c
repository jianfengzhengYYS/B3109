/**
 * \file br3109_hal.c
 * \brief Contains TALISE API Hardware Abstraction Layer (HAL) functions
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#include "br3109_hal.h"

brHalErr_t brSpiReadWord(void *devHalInfo, uint32_t addr, uint32_t *readdata)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_spiReadWord(devHalInfo, addr, readdata);
	if (halError == BRHAL_WAIT_TIMEOUT) {
		BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT * HAL_TIMEOUT_MULT);
		halError = BRHAL_spiReadWord(devHalInfo, addr, readdata);
	}

	BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT);
	return halError;
}

brHalErr_t brSpiWriteWord(void *devHalInfo, uint32_t addr, uint32_t data)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_spiWriteWord(devHalInfo, addr, data);
	if (halError == BRHAL_WAIT_TIMEOUT) {
		BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT * HAL_TIMEOUT_MULT);
		halError = BRHAL_spiWriteWord(devHalInfo, addr, data);
	}

	BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT);
	return halError;
}

brHalErr_t brSpiReadWordsBlock(void *devHalInfo, uint32_t addr, uint32_t *readdata, uint32_t count)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_spiBlockReadWords(devHalInfo, addr, readdata, count);
	if (halError == BRHAL_WAIT_TIMEOUT) {
		BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT * HAL_TIMEOUT_MULT);
		halError = BRHAL_spiBlockReadWords(devHalInfo, addr, readdata, count);
	}

	BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT);
	return halError;
}

brHalErr_t brSpiWriteWordsBlock(void *devHalInfo, uint32_t addr, uint32_t *data, uint32_t count)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_spiBlockWriteWords(devHalInfo, addr, data, count);
	if (halError == BRHAL_WAIT_TIMEOUT) {
		BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT * HAL_TIMEOUT_MULT);
		halError = BRHAL_spiBlockWriteWords(devHalInfo, addr, data, count);
	}

	BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT);
	return halError;
}

brHalErr_t brSpiWriteWords(void *devHalInfo, uint32_t *addr, uint32_t *data,
			     uint32_t count)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_spiWriteWords(devHalInfo, addr, data, count);
	if (halError == BRHAL_WAIT_TIMEOUT) {
		BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT * HAL_TIMEOUT_MULT);
		halError = BRHAL_spiWriteWords(devHalInfo, addr, data, count);
	}

	BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT);
	return halError;
}

brHalErr_t brSpiReadWords(void *devHalInfo, uint32_t *addr, uint32_t *readdata,
			    uint32_t count)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_spiReadWords(devHalInfo, addr, readdata, count);
	if (halError == BRHAL_WAIT_TIMEOUT) {
		BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT * HAL_TIMEOUT_MULT);
		halError = BRHAL_spiReadWords(devHalInfo, addr, readdata, count);
	}

	BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT);
	return halError;
}

brHalErr_t brSpiReadField(void *devHalInfo, uint32_t addr, uint32_t *fieldVal,
			    uint32_t mask, uint32_t startBit)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_spiReadField(devHalInfo, addr, fieldVal, mask, startBit);
	if (halError == BRHAL_WAIT_TIMEOUT) {
		BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT * HAL_TIMEOUT_MULT);
		halError = BRHAL_spiReadField(devHalInfo, addr, fieldVal, mask, startBit);
	}

	BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT);
	return halError;
}

brHalErr_t brSpiWriteField(void *devHalInfo, uint32_t addr, uint32_t fieldVal,
			     uint32_t mask, uint32_t startBit)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_spiWriteField(devHalInfo, addr, fieldVal, mask, startBit);
	if (halError == BRHAL_WAIT_TIMEOUT) {
		BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT * HAL_TIMEOUT_MULT);
		halError = BRHAL_spiWriteField(devHalInfo, addr, fieldVal, mask, startBit);
	}

	BRHAL_setTimeout(devHalInfo, HAL_TIMEOUT_DEFAULT);
	return halError;
}

brHalErr_t brWriteToLog(void *devHalInfo, brLogLevel_t logLevel,
			  uint32_t errorCode, const char *comment)
{
	brHalErr_t halError = BRHAL_OK;

	halError = BRHAL_writeToLog(devHalInfo, logLevel, errorCode, comment);

	return halError;
}
