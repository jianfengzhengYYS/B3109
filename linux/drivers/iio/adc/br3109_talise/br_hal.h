/**
 * \file BR_hal.h
 * \brief Contains Br3109 BR HAL function prototypes type definitions for BR_hal.c
 *
 * Copyright 2022 briradio.
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef _BR_HAL_H_
#define _BR_HAL_H_

/* include standard types and definitions */
#include <linux/kernel.h>
#include <linux/math64.h>"
#include <linux/delay.h>

#define DIV_U64_REM(x,y,r) div64_u64_rem(x, y, r)
#define DIV_U64(x,y) div_u64(x, y)
#define DIV_S64(x,y) div_s64(x, y)

#ifndef UART_Printf
#define  UART_Printf(...)    	dev_err(&phy->spi->dev, __VA_ARGS__)
#endif

#define delay_ms(msecs)	mdelay(msecs)
#define delay_us(us)	udelay(us)
#ifndef SUCCESS
#define SUCCESS    0  
#endif

#ifndef FAILURE
#define FAILURE   -1
#endif

#ifndef pow
int myintpowbase10( int exponent, int subexponent);
#endif

#ifndef log10
int32_t my20log10(int32_t x) ;
#define mymdb20log10(x) (my20log10(1000 * (x))/1000)
#define mygetmdb20log10(x) (my20log10(1000 * (x)))
#endif

// #ifndef sin
// double mysin(double x);
// #define sin mysin
// #endif
// #ifndef cos
// float mycos(float x);
// #define cos mycos
// #endif
/*========================================
 * Enums and structures
 *=======================================*/

struct br_hal {
	// void *reset_gpio;
	// void *spi;
	struct spi_device 	*spi;
	struct gpio_desc	*reset_gpio;
    uint8_t devIndex;
	uint32_t			log_level;
};

/**
 *  \brief Enum of possible Errors Detected by HAL layer to be communicated
 *         to BR APIs.
 */
typedef enum {
	__SYSREF_CONT_ON,
	__SYSREF_CONT_OFF,
	__SYSREF_PULSE,
} SysrefReqMode_t;


/**
 *  \brief Enum of possible 9009 numbers in this desgin
 *         to BR APIs.
 */
typedef enum {
	BRHAL_CHIP0,
	BRHAL_CHIP1,

} brCHIP_t;

/**
 *  \brief Enum of possible Errors Detected by HAL layer to be communicated
 *         to BR APIs.
 */
typedef enum {
	BRHAL_OK = 0,        /*!< HAL function successful. No error Detected */
	BRHAL_SPI_FAIL,      /*!< HAL SPI operation failure. SPI controller Down */
	BRHAL_GPIO_FAIL,     /*!< HAL GPIO function Failure */
	BRHAL_TIMER_FAIL,    /*!< HAL Timer function Failure */
	BRHAL_WAIT_TIMEOUT,  /*!< HAL function Timeout */
	BRHAL_GEN_SW,        /*!< HAL function failed due to general invalid  HAL data*/
	BRHAL_WARNING,       /*!< HAL function warning that non critical error was detected*/
	BRHAL_ERR
} brHalErr_t;

/**
 *  \brief An enumerated type in bit mask format to list the log message
 *         categories or groups.
 */
typedef enum {
	BRHAL_LOG_NONE = 0x0,  /*!< HAL Log enum to represent all types of log messages not selected */
	BRHAL_LOG_MSG  = 0x1,  /*!< HAL Log enum to represent a log message type*/
	BRHAL_LOG_WARN = 0x2,  /*!< HAL Log enum to represent a warning message type*/
	BRHAL_LOG_ERR  = 0x4,  /*!< HAL Log enum to represent a error message type*/
	BRHAL_LOG_SPI  = 0x8,  /*!< HAL Log enum to represent a spi transaction type*/
	BRHAL_LOG_ALL  = 0xF   /*!< HAL Log enum to represent all types of log messages selected */

} brLogLevel_t;

/*========================================
 * Prototypes
 *=======================================*/
#ifdef __cplusplus
extern "C" {
#endif

/* 12 Bytes per SPI transaction * 85 transactions = ~1024 byte buffer size */
/* Minimum HAL_SPIWRITEARRAY_BUFFERSIZE = 18 */
#define HAL_SPIWRITEARRAY_BUFFERSIZE 85

/*============================================================================
 * BriRBRo Ltd. Hardware Control Functions
 *===========================================================================*/

/**
 * \brief Performs a platform hardware initialization for the  BriRBRo Ltd.
 *
 * This function shall initialize all external hardware resources required by
 * BriRBRo Ltd. for correct functionality such as SPI drivers, GPIOs,
 * clocks (if necessary), as per the target platform and target BriRBRo Ltd..
 * At minimum any SPI driver for the BriRBRo Ltd. must be initialized here in order
 * for the SPI writes within the API to function.
 *
 * The API based on the required operation shall set the value timeout parameter.
 * Once this value is set it shall be the timeout requirement for all HAL operations.
 * It is the responsibility of the HAL implementation to maintain this
 * timeout value as a reference. The HAL implementation must use this value
 * to ensure the HAL function do not block longer than this time interval.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This API should be called before calling any other BRHAL functions.
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules defined by devHalInfo
 *
 * \param devHalInfo A void pointer to the targeted device state container.
 *                   The Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param halTimeout_ms A positive integer value to set the HAL function timeout
 *                     interval.
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_SPI_FAIL if function failed to open SPI driver for the device
 * \retval BRHAL_GPIO_FAIL, if function failed to access GPIO resources
 * \retval BRHAL_TIMER_FAIL, if function failed to access timer resources
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t BRHAL_openHw(void *devHalInfo, uint32_t halTimeout_ms);


/**
 * \brief Closes any platform hardware drivers, handles, etc that were necessary
 *        for the BriRBRo Ltd..
 *
 * This function shall shutdown all external hardware resources required by
 * BriRBRo Ltd. for correct functionality such as SPI drivers, GPIOs, clocks as
 * as per the targeted platform and target BriRBRo Ltd..
 * It should close and free any resources assigned in BRHAL_openHw(void *devHalInfo)
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function is called last to terminate the use of device instance and its
 * hardware resources. BRHAL_openHw for the same device instance prior to calling
 * BRHAL_closeHw
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules defined by devHalInfo
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_SPI_FAIL if function failed to close SPI driver for the device
 * \retval BRHAL_GPIO_FAIL, if function failed to release GPIO resources
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t BRHAL_closeHw(void *devHalInfo);


/**
 * \brief Sets the a timeout duration for the HAL functions.
 *
 * This function shall set the timeout duration for the HAL functions. If the
 * HAL operation exceed this time the function shall return with the
 * BRHAL_WAIT_TIMEOUT error. This value shall be set by the API based on the
 * desired required operation. Once this value is set it shall be the timeout
 * requirement for all HAL operations.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre  This function can be called at anytime to change the timeout duration
 * for the HAL functions.  However, it is typically used during initialization.
 *
 * <B>Dependencies</B>
 *  --Application and Platform Specific modules defined by devHalInfo
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 * \param halTimeout_ms A positive integer value to set the HAL function timeout
 *                      interval.
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown. eg devHalInfo
 */
brHalErr_t BRHAL_setTimeout(void *devHalInfo, uint32_t halTimeout_ms);

/**
 * \brief Performs a hardware reset on the BriRBRo Ltd.
 *
 * BriRBRo Ltd.s have a RESETB pin. This function shall toggle the appropriate
 * BriRBRo Ltd. RESETB pin according to the specifications outlined in the BR
 * Device Data Sheet for the targeted Device.
 *
 * BR does not define the structure members in devHalInfo.  The user
 * developing the platform layer of code is required to reset the correct
 * device based on some identifying information within their devHalInfo
 * structure such as a spi chip select or other device id in devHalInfo.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function may be called at any time to reset the device after the required
 * GPIO drivers and resources are opened by the BRHAL_openHw function call.
 * and not after BRHAL_closeHW.
 *
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules defined by devHalInfo
 *
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *        hardware settings describing the device of interest and required
 *        hardware references to toggle RESETB pin of the BR device.
 *
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_GPIO_FAIL if GPIO hardware to toggle BR reset pin unavailable
 * \retval BRHAL_WAIT_TIMEOUT if HAL timeout expired before reset pin toggle could be completed.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t BRHAL_resetHw(void *devHalInfo);


/**
 * \brief Performs a Single SPI write to an BriRBRo Ltd.
 *
 * This function shall perform a single SPI write to an BriRBRo Ltd.. The SPI
 * write implementation must support 32 bit addressing and 32-bit data words.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function may only be used after the required SPI drivers and resources
 * are opened by the BRHAL_openHw() function call and not after BRHAL_closeHW.
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param addr 32-bit SPI register Address (MSB bit in lower layers sets
 *             read/write bit)
 *
 * \param data 32-bit data value to write to the SPI address specified.
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_SPI_FAIL if function failed to complete SPI transaction
 * \retval BRHAL_WAIT_TIMEOUT if HAL timeout expired before SPI transaction could be completed.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t BRHAL_spiWriteWord(void *devHalInfo, uint32_t addr, uint32_t data);

/**
 * \brief Writes an array of SPI writes to an BriRBRo Ltd.
 *
 * This function allows improved performance to write large sets of SPI
 * registers to a single device by passing the SPI Write address and data array to
 * the platform driver instead of many individual calls.
 *
 * If the platform layer SPI driver has no way to write an array to the SPI
 * driver, have this function call BRHAL_spiWriteWord in a for loop.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function may only be used after the required SPI drivers and resources
 * are opened by the BRHAL_openHw() function call and not after BRHAL_closeHW.
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param addr Initial address of SPI register to write.
 *
 * \param data An array of 32-bit data values to write to the SPI addresses, starting from addr.
 *
 * \param count The number of SPI Writes to transfer. Must be smaller or equal
 *              to the size of the data arrays.
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_SPI_FAIL if function failed to complete SPI transaction
 * \retval BRHAL_WAIT_TIMEOUT if HAL timeout expired before SPI transaction could be completed.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t  BRHAL_spiBlockWriteWords(void *devHalInfo, uint32_t addr,
				  uint32_t *data, uint32_t count);
brHalErr_t BRHAL_spiWriteWords(void *devHalInfo,
				 uint32_t *addr, uint32_t *data, uint32_t count);

/**
 * \brief Performs a Single SPI Read from an BriRBRo Ltd.
 *
 * This function shall perform a single SPI read from an BriRBRo Ltd.. The SPI
 * read implementation must support 32 bit addressing and 32-bit data bytes.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function may only be used after the required SPI drivers and resources
 * are opened by the BRHAL_openHw() function call and not after BRHAL_closeHW.
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param addr The 32-bit address to read from the SPI device. 
 *
 * \param readdata The 32-bit data value returned from SPI register addr
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_SPI_FAIL if function failed to complete SPI transaction
 * \retval BRHAL_WAIT_TIMEOUT if HAL timeout expired before SPI transaction could be completed.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t BRHAL_spiReadWord(void *devHalInfo, uint32_t addr,
			       uint32_t *readdata);

/**
 * \brief Performs a Multi SPI Read from an BriRBRo Ltd.
 *
 * This function shall perform multi SPI read from an BriRBRo Ltd.. The SPI
 * read implementation must support 32 bit addressing and 32-bit data bytes.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function may only be used after the required SPI drivers and resources
 * are opened by the BRHAL_openHw() function call and not after BRHAL_closeHW.
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param addr Initial address of SPI register to read from.
 *
 * \param *readdata The 32-bit data value returned from SPI register addr
 *
 * \param  count The number of SPI Reads to transfer. Must be equal
 *              to the size of the addr and data arrays.
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_SPI_FAIL if function failed to complete SPI transaction
 * \retval BRHAL_WAIT_TIMEOUT if HAL timeout expired before SPI transaction could be completed.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t BRHAL_spiBlockReadWords(void *devHalInfo, uint32_t addr,
				uint32_t *readdata, uint32_t count);
brHalErr_t BRHAL_spiReadWords(void *devHalInfo, uint32_t *addr, uint32_t *readdata, uint32_t count);

/**
 * \brief Performs a write to the specified field in a SPI register.
 *
 * This function shall perform a Read/Modify/Write of a single SPI read from an
 * bits in the register, and writing the value back to the SPI register.
 * BriRBRo Ltd.. The mask and startBit parameters allow modifying some or all
 *
 * The mask is expected to be applied to the complete 32-bit read back SPI
 * register value.  The startBit parameter is used to shift the fieldVal
 * parameter to the correct starting bit in the SPI register.
 *
 * Desired Operation:
 * readVal = Read 32-bit SPI register value
 *
 * Modify
 *   Clear bits in readVal for enabled mask bits (readVal & ~mask)
 *   Set bits in field based on mask and startBit ((fieldVal << startBit) & mask)
 *
 * Write the modified value back to the SPI register
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre  This function may only be used after the required SPI drivers and resources
 * are opened by the BRHAL_openHw() function call and not after BRHAL_closeHW.
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param addr The 32-bit address to read/modify/write from the SPI device.

 *
 * \param fieldVal 32-bit value to update in a bitfield (subset) of the SPI register
 *
 * \param mask 32-bit mask of bits to modify (if a bit = 1, that bit can be modified)
 * \param startBit starting bit of the fieldVal in the SPI reg.  Bit shifts
 *                 the fieldVal up to that bit before masking and modifying the SPI
 *                 reg.
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_SPI_FAIL if function failed to complete SPI transaction
 * \retval BRHAL_WAIT_TIMEOUT if HAL timeout expired before SPI transaction could be completed.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t BRHAL_spiWriteField(void *devHalInfo, uint32_t addr,
				 uint32_t  fieldVal, uint32_t mask, uint8_t startBit);

/**
 * \brief Performs a read from a particular bit field in a SPI register.
 *
 * This function shall perform a single SPI read from an BriRBRo Ltd.. Before
 * the SPI read value is returned, it is masked and bit-shifted to return
 * only a desired subset of bits - the desired bit field.
 *
 * Any necessary SPI drivers or resources are expected to have already been
 * opened by the BRHAL_openHw() function call.
 *
 * The mask parameter is applied to the entire 32-bit SPI read back value. The
 * startBit parameter shifts the desired subset of bits down to the zero bit
 * position.
 *
 * Desired Operation:
 * readVal = SPI read of a 32-bit SPI register
 * return (readVal & mask) >> startBit
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function may only be used after the required SPI drivers and resources
 * are opened by the BRHAL_openHw() function call and not after BRHAL_closeHW.
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param addr The 32-bit address to read/modify/write from the SPI device.

 *
 * \param fieldVal 32-bit value: Readback value of a bitfield (subset) of the
 *                 SPI register masked and shifted down to the bit 0 position
 *
 * \param mask 32-bit mask of bits to readback (if a bit = 1, that bit can be read)
 * \param startBit starting LSB bit of the fieldVal in the SPI reg.  Bit shifts
 *                 the fieldVal down to that bit before returning in fieldVal
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_SPI_FAIL if function failed to complete SPI transaction
 * \retval BRHAL_WAIT_TIMEOUT if HAL timeout expired before SPI transaction could be completed.
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 */
brHalErr_t BRHAL_spiReadField(void *devHalInfo, uint32_t addr,
				uint32_t *fieldVal, uint32_t mask, uint8_t startBit);

/**
 * \brief Delay or sleep for the specified number of microseconds.
 *
 * Performs a thread blocking/sleeping delay of the specified time in us.
 * Function should wait at least for the specified amount of time.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function can be used at any time to create a delay
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param time_us The amount of time in microseconds to block - thread sleep for
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_TIMER_FAIL if function delay function failed to complete
 * \retval BRHAL_WARNING if function failed to access non-critical resource eg logging
 *
 */
brHalErr_t BRHAL_wait_us(void *devHalInfo, uint32_t time_us);

/*========================================
 * Optional Logging Functions
 *
 *=======================================*/
/**
 * \brief Writes a character array to a log file to facilitate debugging
 *
 * This function uses the logLevel parameter to specify what type of
 * message / warning / error to write to the log file. This function
 * uses enums in BRLogLevel_t to set the log level.
 * BRHAL_LOG_NONE = 0x0,
 * BRHAL_LOG_MSG  = 0x1,
 * BRHAL_LOG_WARN = 0x2,
 * BRHAL_LOG_ERR  = 0x4,
 * BRHAL_LOG_SPI  = 0x8,
 * BRHAL_LOG_ALL  = 0xF
 *
 * The user implementation may choose to allow a feature to filter
 * the level of message to log.  This could allow only logging errors
 * for instance.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function should be used when setting up and configuring the system
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param halLogLevel The type of message to log.
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 */
brHalErr_t BRHAL_setLogLevel(void *devHalInfo, uint16_t halLogLevel);

/**
 * \brief Writes a character array to a log file to facilitate debugging
 *
 * This function uses the logLevel parameter to specify what type of
 * message / warning / error to write to the log file. This function
 * uses enums in BRLogLevel_t to set the log level.
 *
 * The user implementation may choose to allow a feature to filter
 * the level of message to log.  This could allow only logging errors
 * for instance.
 *
 * Returns an error of type brHalErr_t. Error returned will depend on platform
 * specific implementation. API expects BRHAL_OK if function completed successfully.
 * Any other value represents an error or warning to the API. Error return list
 * below provides possible errors from customer implementation.
 *
 * \pre This function can be used at anytime once the required logging resources
 * are opened by the BRHAL_openHw() function call and not after BRHAL_closeHW.
 * The log level should also be set prior to calling this function via
 * BRHAL_setLogLevel.
 *
 * <B>Dependencies</B>
 * --Application and Platform Specific modules
 *
 * \param devHalInfo Pointer to Platform HAL defined structure containing
 *                   hardware settings describing the device of interest.
 *
 * \param logLevel The type of message to log.  BRHAL_LOG_MSG,
 *        BRHAL_LOG_WARN, BRHAL_LOG_ERR, BRHAL_LOG_SPI,
 *
 * \param errorCode An error code value to designate a particular error
 * \param comment The character array describing the error
 *
 * \retval BRHAL_OK if function completed successfully.
 * \retval BRHAL_GEN_SW if device references is unknown.
 * \retval BRHAL_WARNING if log message cannot be completed
 */
brHalErr_t BRHAL_writeToLog(void *devHalInfo, brLogLevel_t logLevel,
			      uint32_t errorCode, const char *comment);


#ifdef __cplusplus
}
#endif
#endif
