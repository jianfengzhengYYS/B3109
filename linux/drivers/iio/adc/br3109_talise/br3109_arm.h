/**
 * \file br3109_arm.h
 * \brief Contains Br3109 ARM related function prototypes for br3109_arm.c
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef BR3109_ARM_H_
#define BR3109_ARM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "br3109_types.h"
#include "br3109_error_types.h"
#include "br3109_arm_types.h"
#include "br3109_arm_spi_cmd.h"
/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/**
 * \brief Resets and performs initialization for the ARM processor
 *
 * Sets ARM Run = 0, Disables parity checks, sets ARM and SPI register clock selects,
 * resets ARM, and enables ARM SPI register access
 *
 * \pre This function is called after the device has been initialized and multichip-sync
 * (MCS) has been completed
 *
 * \dep_begin
 * \dep{device->devHalInfo}
 * \dep{init-> (most members)}
 * \dep_end
 *
 * \param device Pointer to the Br3109 device settings data structure
 * \param init Pointer to the Br3109 initialization setting data structure
 *
 * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
 * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
 * \retval TALACT_NO_ACTION Function completed successfully, no action required
 */
uint32_t BR3109_initArm(br3109Device_t *device, br3109Init_t *init);

/**
 * \brief Writes the Br3109 ARM configuration settings
 *
 * \pre This function is called automatically during BR3109_initArm(), and
 *      this function must be called before loading the ARM firmware.
 *
 * \dep_begin
 * \dep{device->devHalInfo}
 * \dep{init-> (most members)}
 * \dep_end
 *
 * \param device Pointer to the Br3109 device settings data structure
 * \param init Pointer to the Br3109 initialization settings data structure
 *
 * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
 * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
 * \retval TALACT_NO_ACTION Function completed successfully, no action required
 */
uint32_t BR3109_writeArmProfile(br3109Device_t *device, br3109Init_t *init);

/**
 * \brief Loads binary array into ARM program memory
 *
 * This function sets the ARM DMA control register bits for an ARM memory write in legacy mode, auto-incrementing
 * the address. Valid memory addresses are: Program Memory (0x01000000 - 0x0101C000)
 *
 * The top level application reads the binary file, parsing it into any array, starting at the first data byte
 * on the first line of the file. The array is passed to this function and writes it to ARM program memory. Any non-data
 * bytes should be excluded from the binary array when parsing.
 *
 * \pre This function is called after the device has been initialized, PLL lock status has been verified, and
 * the stream binary has been loaded
 *
 * \dep_begin
 * \dep{device->devHalInfo}
 * \dep_end
 *
 * \param device Pointer to the Br3109 device data structure containing settings
 * \param binary word array containing all valid ARM file data bytes
 * \param count The number of words in the binary array
 *
 * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
 * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
 * \retval TALACT_NO_ACTION Function completed successfully, no action required
 */
uint32_t BR3109_loadArmFromBinary(br3109Device_t *device, uint32_t *binary, uint32_t count);

/****************************************************************************
 * Runtime functions
 ****************************************************************************
 */

/****************************************************************************
 * Helper functions
 ****************************************************************************
 */

 /**
  * \brief Read from the Br3109 ARM program or data memory
  *
  * Valid memory addresses are: Program Memory ,
  * Data Memory .
  *
  * \pre This function is private and is not called directly by the user.
  *
  * \dep_begin
  * \dep{device->devHalInfo}
  * \dep_end
  *
  * \param device Structure pointer to the Br3109 data structure containing settings
  * \param address The 32bit ARM address to read from.
  * \param returnDataWord word(uint32_t) array containing the data read from the ARM memory.
  * \param WordsToRead Number of words in the returnData array.
  * \param autoIncrement is boolean flag to enable or disable auto-increment of ARM register address
  *
  * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
  * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
  * \retval TALACT_NO_ACTION Function completed successfully, no action required
  */
uint32_t BR3109_readArmMem(br3109Device_t *device, uint32_t address, uint32_t *returnDataWord, uint32_t WordsToRead, uint8_t autoIncrement);

 /**
  * \brief Write to the Br3109 ARM program or data memory
  *
  * Valid memory addresses are: Program Memory (0x01000000 - 0x0101C000),
  * Data Memory (0x20000000 - 0x20014000).
  *
  * \pre This function is private and is not called directly by the user.
  *
  * \dep_begin
  * \dep{device->devHalInfo}
  * \dep_end
  *
  * \param device Structure pointer to the Br3109 data structure containing settings
  * \param address The 32-bit ARM address to write
  * \param dataWord word array (uint8_t) containing data to be written to ARM memory
  * \param WordCount Number of words in the data array to be written
  *
  * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
  * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
  * \retval TALACT_NO_ACTION Function completed successfully, no action required
  */
uint32_t BR3109_writeArmMem(br3109Device_t *device, uint32_t address, uint32_t *dataWord, uint32_t WordCount);

 /**
  * \brief Low level helper function used by Br3109 API to write the ARM memory config structures
  *
  * Normally this function should not be required to be used directly by the BBIC.  This is a helper
  * function used by other Br3109 API commands to write settings into the ARM memory.
  *
  * \pre This function is private and is not called directly by the user.
  *
  * \dep_begin
  * \dep{device->devHalInfo}
  * \dep_end
  *
  * \param device Structure pointer to the Br3109 data structure containing settings
  * \param objectId ARM id of a particular structure or setting in ARM memory
  * \param offset Byte offset from the start of the objectId's memory location in ARM memory
  * \param data A byte array containing data to write to the ARM memory buffer.
  * \param byteCount Number of bytes in the data array (Valid size = 1-255 bytes)
  *
  * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
  * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval TALACT_ERR_RESET_ARM Recovery action for ARM reset required
  * \retval TALACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
  * \retval TALACT_NO_ACTION Function completed successfully, no action required
 */
 uint32_t BR3109_writeArmConfig(br3109Device_t *device, uint8_t objectId, uint16_t offset, uint8_t *data, uint8_t byteCount);

 /**
  * \brief Low level helper function used by Br3109 API to read the ARM memory config structures
  *
  * Normally this function should not be required to be used directly by the BBIC.  This is a helper
  * function used by other Br3109 API commands to read settings from the ARM memory.
  *
  * \pre This function is private and is not called directly by the user
  *
  * \dep_begin
  * \dep{device->devHalInfo}
  * \dep_end
  *
  * \param device Structure pointer to the Br3109 data structure containing settings
  * \param objectId ARM id of a particular structure or setting in ARM memory
  * \param offset Byte offset from the start of the objectId's memory location in ARM memory
  * \param data A byte array containing data to write to the ARM memory buffer
  * \param byteCount Number of bytes in the data array (Valid size = 1-255 bytes)
  *
  * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
  * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
  * \retval TALACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
  * \retval TALACT_ERR_RESET_ARM Recovery action for ARM reset required
  * \retval TALACT_NO_ACTION Function completed successfully, no action required
  */
 uint32_t BR3109_readArmConfig(br3109Device_t *device, uint8_t objectId, uint16_t offset, uint8_t *data, uint8_t byteCount);

 /**
  * \brief Reads the Br3109 ARM  command status register and returns an error and status word
  *
  * A 64-bit status register consisting of a pending bit and three-bit error type is read one byte at
  * a time for the first 16 even-numbered opcodes. The function parses the pending bits and error bits into
  * two (2) separate 16-bit words. statusWord contains the status pending bits. errorWord contains
  * a single error bit if the error type > 0 for any three-bit code.
  * Each word is weighted according to the first 16 even-numbered opcodes where,
  *
  * \pre This function is private and is not called directly by the user.
  *
  * \dep_begin
  * \dep{device->devHalInfo}
  * \dep_end
  *
  * \param device Structure pointer to the Br3109 data structure containing settings
  * \param errorWord 32-bit error word comprised of weighted bits according to first 32 even-numbered opcodes
  * The weighted bit is set if any three-bit error type > 0, where '0' = OK
  * \param statusWord 32-bit pending bits word comprised of weighted bits according to first 32 even-numbered opcodes
  * The weighted bit is set if an opcode is pending, where '0' = no pending opcode
  *
  * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
  * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
  * \retval TALACT_NO_ACTION Function completed successfully, no action required
  */
 uint32_t BR3109_readArmCmdStatus(br3109Device_t *device, uint32_t *errorWord, uint32_t *statusWord);

 /**
  * \brief Isolated byte read of the Br3109 ARM  command status register based on the opcode
  *
  * A single byte read is performed on the 64-bit command status register according to
  * the opcode of interest. The pending bit and the error type are extracted from the status
  * register and returned as a single byte in the lower nibble.
  *
  * \pre This function is private and is not called directly by the user.
  *
  * \dep_begin
  * \dep{device->devHalInfo}
  * \dep_end
  *
  * \param device Structure pointer to the Br3109 data structure containing settings
  * \param opCode Opcode of interest where only the first 16 even-numbered integers are valid
  * \param cmdStatword Comprised of cmdStatByte[3:1] = error type, cmdStatByte[0] = pending flag for opCode of interest
  *
  * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
  * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
  * \retval TALACT_NO_ACTION Function completed successfully, no action required
  */
// uint32_t BR3109_readArmCmdStatusByte(br3109Device_t *device, uint8_t opCode, uint8_t *cmdStatByte);
uint32_t BR3109_readArmCmdStatusWord(br3109Device_t *device, uint32_t opCode, uint32_t *cmdStatword);

 /**
  * \brief Br3109 ARM command status wait function polls command status register until opcode of interest completes
  *
  * This function polls the ARM status SPI register at the waitInterval_us rate,
  * blocking the function return until either the ARM command completes, the
  * ARM returns an error, or this function times out.
  *
  * In the event that this function returns an error, such as a timeout error
  * or ARM error, the cmdStatByte parameter is still returned with the ARM
  * command status, which can be used for debug to determine the lower
  * layer ARM error.
  *
  * \pre ARM firmware load and initialization must take place first before attempting to use this function
  *
  * \dep_begin
  * \dep{device->devHalInfo}
  * \dep_end
  *
  * \param device Structure pointer to the Br3109 data structure containing settings
  * \param opCode Opcode of interest where only the first 16 even-numbered integers are valid
  * \param cmdStatword Comprised of cmdStatByte[3:1] = error type, cmdStatByte[0] = pending flag for opCode of interest
  * \param timeout_us Command time-out period in microseconds
  * \param waitInterval_us Wait interval time to thread sleep between each check of the ARM command status to prevent SPI read thrashing
  *
  * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
  * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval TALACT_ERR_RESET_ARM Recovery action for ARM reset required
  * \retval TALACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
  * \retval TALACT_NO_ACTION Function completed successfully, no action required
  */
// uint32_t BR3109_waitArmCmdStatus(br3109Device_t *device, uint8_t opCode, uint8_t *cmdStatByte, uint32_t timeout_us, uint32_t waitInterval_us);
uint32_t BR3109_waitArmCmdStatus(br3109Device_t *device, uint32_t opCode, uint32_t *cmdStatword, uint32_t timeout_us, uint32_t waitInterval_us);

 /**
  * \brief Sends a command to the Br3109 ARM processor interface
  *
  * \pre This function can be called after initializing the ARM processor
  *
  * \dep_begin
  * \dep{device->devHalInfo}
  * \dep_end
  *
  * \param device Structure pointer to the Br3109 data structure containing settings
  * \param opCode Opcode of interest where only the first 16 even-numbered integers are valid
  * \param extendedData A byte array containing extended data to write to the ARM command interface
  * \param extendedDataNumBytes Number of bytes in the extendedData array (Valid size = 0-4 bytes)
  *
  * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
  * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
  * \retval TALACT_ERR_RESET_ARM Recovery action for ARM reset required
  * \retval TALACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
  * \retval TALACT_ERR_RESET_SPI Recovery action for SPI reset required
  * \retval TALACT_NO_ACTION Function completed successfully, no action required
  */
// uint32_t BR3109_sendArmCommand(br3109Device_t *device, uint8_t opCode, const uint8_t *extendedData, uint8_t extendedDataNumBytes);
uint32_t BR3109_sendArmCommand(br3109Device_t *device, uint32_t opCode, const uint32_t *extendedData, uint32_t extendedDataNumWords);


/****************************************************************************
 * Debug functions
 ****************************************************************************
 */

 /**
 * \brief Reads back the version of the ARM binary loaded into the Br3109 ARM memory
 *
 * This function reads the ARM memory to read back the major.minor.releaseCandidate
 * version and the build type for the ARM binary loaded into ARM memory.
 * Note: only 'TAL_ARM_BUILD_RELEASE' build type should be used with production code
 *
 * <B>Dependencies</B>
 * - device->spiSettings->chipSelectIndex
 *
 * \param device is structure pointer to the Br3109 data structure containing settings
 * \param talArmVersionInfo is a pointer to a structure containing the ARM binary version and
 * build type data
 *
 * \retval TALACT_NO_ACTION Function completed successfully
 * \retval TALACT_ERR_CHECK_PARAM One of the function parameters has a NULL pointer or ARM has not been loaded
 */
uint32_t BR3109_getArmVersion_v2(br3109Device_t *device, br3109ArmVersionInfo_t *talArmVersionInfo);

/**
* \deprecated Please see BR3109_getArmVersion_v2 for current implementation
*
* \brief ***DEPRECATED*** Reads back the version of the ARM binary loaded into the Br3109 ARM memory
*
* This function reads the ARM memory to read back the major.minor.releaseCandidate
* version for the ARM binary loaded into ARM memory.
*
* <B>Dependencies</B>
* - device->spiSettings->chipSelectIndex
*
* \param device is structure pointer to the Br3109 data structure containing settings
* \param majorVer The Major version is returned in this parameter
* \param minorVer The Minor version is returned in this parameter
* \param rcVer The release candidate version (build number) is returned in this parameter
*
* \retval TALACT_NO_ACTION Function completed successfully
* \retval TALACT_ERR_CHECK_PARAM One of the function parameters has a NULL pointer
*/
uint32_t BR3109_getArmVersion(br3109Device_t *device, uint8_t *majorVer, uint8_t *minorVer, uint8_t *rcVer);
/*
* \brief BR3109_ArmWriteField used after 3109 is start
* 
* This function write fied  the ARM memory 
* <B>Dependencies</B>
* - device->spiSettings->chipSelectIndex
*
* \param device is structure pointer to the Br3109 data structure containing settings
 *
 * \param addr The 32-bit address to read/modify/write from the arm, notice: addr[bit31] use for spi control in arm's spi device & [bit 8-15] used for arm spi id.

 *
 * \param fieldVal 32-bit value to update in a bitfield (subset) of the SPI register
 *
 * \param mask 32-bit mask of bits to modify (if a bit = 1, that bit can be modified)
 * \param startBit starting bit of the fieldVal in the SPI reg.  Bit shifts
 *                 the fieldVal up to that bit before masking and modifying the SPI
 *                 reg.
*
* \retval TALACT_NO_ACTION Function completed successfully
* \retval TALACT_ERR_CHECK_PARAM One of the function parameters has a NULL pointer
*/
#define BR3109_ARMSPI_ADDR(id, spi_addr)		(TAL_ARM_SPI_FLAG | (id << TAL_ARM_SPI_ID_POS) | (spi_addr & TAL_ARM_SPI_ADDR_MASK))
uint32_t BR3109_ArmWriteField(br3109Device_t *device, uint32_t addr, uint32_t fieldVal,
			     uint32_t mask, uint32_t startBit);

/**
 * \brief Verifies the ARM checksum value
 *
 * The checksum which is written into the binary file is verified with the calculated
 * checksum in the Br3109 ARM after the binary file has been loaded.  This function
 * will wait for a timeout period for the checksum calculation to occur.  The
 * user can adjust the timeout period and SPI read interval in br3109_user.c by
 * adjusting the macros VERIFY_ARM_CHKSUM_TIMEOUTUS and
 * VERIFY_ARM_CHKSUM_INTERVALUS.
 *
 * \pre This function is called after the ARM binary file has been loaded to verify it's
 * checksum in ARM memory
 *
 * \dep_begin
 * \dep{device->devHalInfo}
 * \dep_end
 *
 * \param device is structure pointer to the Br3109 data structure containing settings
 *
 * \retval TALACT_WARN_RESET_LOG Recovery action for log reset
 * \retval TALACT_ERR_CHECK_TIMER Recovery action for timer time-out check required
 * \retval TALACT_ERR_CHECK_PARAM Recovery action for bad parameter check
 * \retval TALACT_ERR_RESET_ARM Recovery action for ARM reset required
 * \retval TALACT_NO_ACTION Function completed successfully, no action required
 */
//uint32_t BR3109_verifyArmChecksum(br3109Device_t *device);
uint32_t BR3109_verifyArmChecksum(br3109Device_t *device, uint32_t checksum);
/**
 * \brief Private Helper function to return ARM error strings to the
 *        BR3109_getErrorMessage function.
 *
 *  This is a private function and is automatically called by the API.  BBIC
 *  should use the BR3109_getErrorMessage function.
 *
 * \param errSrc A value that represents the error source from the Br3109 API
 * \param errCode Error code that along with the error souce allows looking up
 *        a specific error message.
 *
 * \retval Returns a character array with the error message speficied by
 *         errSrc and errCode.
 */
const char* talGetArmErrorMessage(uint32_t errSrc, uint32_t errCode);

/**
 * \brief Private Helper function to process detected errors reported from the
 *        arm to determine recovery action.
 *
 *  This is a private function and is automatically called by the API.
 *
 * \param device Pointer to device data structure identifying desired device instance
 * \param errHdl Error Handler type
 * \param detErr Error detected to be processed by handler (ARM opcode << 16 | ARM object ID <<8 | ARM cmdStatusByte)
 * \param retVal current Recovery Action,
 * \param recAction new Recovery Action to be returned should error handler determine an error
  *
 * \retval uint32_t Value representing the latest recovery Action following processing of detected arm error.
*/
talRecoveryActions_t talArmCmdErrorHandler(br3109Device_t *device, br3109ErrHdls_t errHdl,
        uint32_t detErr, talRecoveryActions_t retVal, talRecoveryActions_t recAction);

#define ARMCMD_ERRCODE(armOpCode, armObjId, armErrorFlag) ((armOpCode << 16) | (armObjId << 8) | armErrorFlag)

#ifdef __cplusplus
}
#endif

#endif /* BR3109_ARM_H_ */
