/**
 * \file br3109_gpio.c
 * \brief Br3109 GPIO functions
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#include "br3109_cals.h"
#include "br3109_gpio.h"
#include "br3109_jesd204.h"
#include "br3109_reg_addr_macros.h"
#include "br3109_hal.h"
#include "br3109_error.h"
#include "br3109_arm.h"
#include "br3109_user.h" /* Defines BR3109_VERBOSE */


uint32_t  BR3109_setGpioOe (br3109Device_t *device, uint32_t gpioOutEn, uint32_t gpioUsedMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t readreg = 0;

    static const uint32_t GPIO_OE_MASK = 0x7FFFF;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpioOe()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* performing gpioOutEn range check */
    if (gpioOutEn > GPIO_OE_MASK)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GPIO_OE_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    /* Br3109 SPI regs to set GPIO OE direction */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, usr_set_gpio_oe), &readreg, 1);
    IF_ERR_RETURN_U32(retVal);
	readreg = (readreg &(~gpioUsedMask))|(gpioOutEn&gpioUsedMask);
	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, usr_set_gpio_oe), &readreg, 1);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}


uint32_t  BR3109_getGpioOe (br3109Device_t *device, uint32_t *gpioOutEn) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	
#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpioOe()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* null pointer check */
    if (gpioOutEn == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIO_OE_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, usr_set_gpio_oe), gpioOutEn, 1);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}

uint32_t  BR3109_setGpioSourceCtrl (br3109Device_t *device, uint32_t gpioSrcCtrl) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
    static const uint32_t GPIO_SRC_MASK = 0x000FFFFF;
	int i = 0;
	uint32_t src_mask = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpioSourceCtrl()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* performing gpioSrcCtrl range check */
    if (gpioSrcCtrl > GPIO_SRC_MASK)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GPIO_SRC_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }
	for(i = 0; i < 5; i++){
		if((gpioSrcCtrl >> i*4)&0xF){
			src_mask |= (0xF<<(i * 4)); 
		}
	}
    /* writing GPIO configuration registers */
	retVal = BR3109_ArmWriteField(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpioSrcCtrl_reg), gpioSrcCtrl, src_mask, 0);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}


uint32_t  BR3109_getGpioSourceCtrl (br3109Device_t *device, uint32_t *gpioSrcCtrl) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpioSourceCtrl()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (gpioSrcCtrl == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIO_SRC_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Reading GPIO source control registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpioSrcCtrl_reg), gpioSrcCtrl, 1);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}


uint32_t  BR3109_setGpioPinLevel (br3109Device_t *device, uint32_t gpioPinLevel) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;

    static const uint32_t GPIO_PIN_MASK = 0x0007FFFF;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpioPinLevel()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* performing range check on gpioPinLevel */
    if (gpioPinLevel > GPIO_PIN_MASK)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GPIO_LEVEL_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* writing GPIO configuration registers */
	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, usr_set_gpio_out), &gpioPinLevel, 1);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}

uint32_t  BR3109_getGpioPinLevel (br3109Device_t *device, uint32_t *gpioPinLevel) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpioPinLevel()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (gpioPinLevel == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIO_LEVEL_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* reading the registers into three-byte array */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, usr_get_gpio_in), gpioPinLevel, 1);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}


uint32_t  BR3109_getGpioSetLevel (br3109Device_t *device, uint32_t *gpioPinSetLevel) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpioSetLevel()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (gpioPinSetLevel == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIO_SETLEVEL_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* reading the registers into three-byte array */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, usr_set_gpio_out), gpioPinSetLevel, 1);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}


uint32_t  BR3109_setGpioMonitorOut (br3109Device_t *device, uint8_t monitorIndex, uint8_t monitorMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
    static const uint8_t INDEX_MASK = 0x35;
	uint32_t data=monitorIndex;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpioMonitorOut()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* performing index range check */
    if (monitorIndex > INDEX_MASK)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_MONITOR_OUT_INDEX_RANGE, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Set the GPIO monitor index and the required pin configuration */
	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, monitor_index), &data, 1);
    IF_ERR_RETURN_U32(retVal);
	data = monitorMask;
	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, monitor_mask), &data, 1);
    IF_ERR_RETURN_U32(retVal);


    return (uint32_t)retVal;
}


uint32_t  BR3109_getGpioMonitorOut (br3109Device_t *device, uint8_t *monitorIndex, uint8_t *monitorMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t data = 0;
	
#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpioMonitorOut()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* Checking for null parameters */
    if (monitorIndex == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIOMON_INDEX_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    if (monitorMask == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIOMON_MONITORMASK_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Get the GPIO monitor index and the required pin configuration */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, monitor_index), &data, 1);
    IF_ERR_RETURN_U32(retVal);
	*monitorIndex = data&0xff;
    /* Get GPIO monitor out enable */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, monitor_mask), &data, 1);
    IF_ERR_RETURN_U32(retVal);
	*monitorMask = data&0xff;
    return (uint32_t)retVal;
}


uint32_t  BR3109_setGpIntMask (br3109Device_t *device, uint16_t gpIntMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t data = gpIntMask&(TAL_GPMASK_LSB|TAL_GPMASK_MSB);

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpIntMask()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* checking for valid mask setting */
    if (gpIntMask & ~(TAL_GPMASK_MSB | TAL_GPMASK_LSB))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_GP_INT_MASK_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    else
    {
		retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio_irq_mask), &data, 1);
	    IF_ERR_RETURN_U32(retVal);
    }

    return (uint32_t)retVal;
}


uint32_t  BR3109_getGpIntMask (br3109Device_t *device, uint16_t *gpIntMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t data = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpIntMask()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
    
    /* checking for null pointer */
    if (gpIntMask == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_GP_INT_MASK_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    else
    {
        /* Read the current GpInt Mask */
		retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio_irq_mask), &data, 1);
	    IF_ERR_RETURN_U32(retVal);
        
        *gpIntMask = data&(TAL_GPMASK_LSB|TAL_GPMASK_MSB);
    }

    return (uint32_t)retVal;
}

uint32_t  BR3109_getGpIntStatus (br3109Device_t *device, uint16_t *gpIntStatus) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t data = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpIntStatus()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* checking for null pointer */
    if (gpIntStatus == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GP_INT_STATUS_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    else
    {
		retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio_irq), &data, 1);
	    IF_ERR_RETURN_U32(retVal);

        *gpIntStatus = data&(TAL_GPMASK_LSB|TAL_GPMASK_MSB);
    }

    return (uint32_t)retVal;
}

uint32_t  BR3109_getTemperature (br3109Device_t *device, int16_t *temperatureDegC) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t data = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getTemperature()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
    /* checking for null pointer */
    if (temperatureDegC == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GP_INT_STATUS_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    else
    {
		// data = (1 << 0) | (0 << 1) | (0x3 << 2) | (8 << 4) ;
		// retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_DAC_IQ_L0_ID, 0x30), data, 0xFF, 0);
		// // data = 1<<24;
		// // retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_DAC_IQ_L0_ID, 0x08), data, 0x1 << 24, 0);
		// IF_ERR_RETURN_U32(retVal);
		// data = 4;//4:adc L0 5:adc L1 6:dac L0 7 dac L1 8: jesd tx 9:jesd rx 15-17:bandgap
		// retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_AUX_ADDA_IVREF_ID, 0x08), data, 0x0000001F, 0);
		// IF_ERR_RETURN_U32(retVal);
		// //RST
		// retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_AUX_ADDA_IVREF_ID, 0x0C), 0, 0x00000001, 0);
		// IF_ERR_RETURN_U32(retVal);
		// retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_AUX_ADDA_IVREF_ID, 0x0C), 1, 0x00000001, 0);
		// IF_ERR_RETURN_U32(retVal);
		// retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_AUX_ADDA_IVREF_ID, 0x0C), 0, 0x00000001, 0);
		// IF_ERR_RETURN_U32(retVal);
		//read adc
		retVal = BR3109_armSpiCmd_SPI_blk_read(device,SPI_AUX_ADDA_IVREF_ID, 0x7C, &data, 1);
	    IF_ERR_RETURN_U32(retVal);
        // *temperatureDegC = (int16_t)(-0.2376f * data + 736.99);
        *temperatureDegC = (int16_t)((-2376 * data + 7369900)/10000);
    }

    return (uint32_t)retVal;
}

uint32_t BR3109_setupAuxDacs(br3109Device_t *device, br3109AuxDac_t *auxDac)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	const uint8_t dac_addr[10]={0x34, 0x64, 0x38, 0x60, 0x3c, 0x5c, 0x30, 0x00, 0x2c, 0x04};
	int i = 0;
//	uint8_t i = 0;
	// uint8_t auxDacConfig = 0x00; //暂时没有该变量用到注释掉了

//	static const uint16_t DAC_10_BIT_MAX_VALUE = 1023;
//	static const uint16_t DAC_12_BIT_MAX_VALUE = 4095;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setupAuxDacs()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if (auxDac == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_SETUPAUXDAC_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	// config analog mode
	for(i = 0; i < 10; i+= 4){
		if(auxDac->auxDacEnables &(0xF<<i)){
			retVal = BR3109_setGpio3v3SourceCtrl(device, TAL_GPIO3V3_ANALOG_MODE <<i);
			IF_ERR_RETURN_U32(retVal);
		}
	}
	retVal = BR3109_setGpio3v3AnaEn(device, auxDac->auxDacEnables, auxDac->auxDacEnables);
	IF_ERR_RETURN_U32(retVal);
	for(i = 0; i < 10; i++){
		if(auxDac->auxDacEnables &(1<<i)){
			retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_AUX_ADDA_IVREF_ID, dac_addr[i]), auxDac->auxDacValues[i], 0x000003FF, 0);
			IF_ERR_RETURN_U32(retVal);
		}
	}
	return (uint32_t)retVal;
}

uint32_t BR3109_writeAuxDac(br3109Device_t *device, uint8_t auxDacIndex, uint16_t auxDacCode)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	const uint8_t dac_addr[10]={0x34, 0x64, 0x38, 0x60, 0x3c, 0x5c, 0x30, 0x00, 0x2c, 0x04};
	uint32_t regdat = auxDacCode;
	static const uint16_t DAC_10_BIT_MAX_VALUE = 1023;
//	static const uint16_t DAC_12_BIT_MAX_VALUE = 4095;
	static const uint8_t DAC_MAX_INDEX = 9;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_writeAuxDac()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if (auxDacIndex > DAC_MAX_INDEX) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_WRITEAUXDAC_INV_AUXDACINDEX, retVal, TALACT_ERR_CHECK_PARAM);
	} else if (auxDacCode > DAC_10_BIT_MAX_VALUE){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_WRITEAUXDAC_INV_12BIT_AUXDACCODE, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		/* Write DAC configuration and code for 10-bit DACs */
		retVal = BR3109_armSpiCmd_SPI_blk_write(device, SPI_AUX_ADDA_IVREF_ID, dac_addr[auxDacIndex], &regdat, 1);
		IF_ERR_RETURN_U32(retVal);
	}

	/* Write enable bit to latch the DAC code into the DAC */

	return (uint32_t)retVal;
}
/**
 * SPI2 仅对GPIO[3:0]这个组有效，
	GPIO_0: SPI2_DIO/MOSI (支持双向模式)
	GPIO_1: SPI2_DOUT
	GPIO_2: SPI2_CLK
	GPIO_3: SPI2_CS_N
*/
uint32_t  BR3109_setSpi2Enable (br3109Device_t *device, uint8_t spi2Enable) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setSpi2Enable()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	if(spi2Enable > 0){		
		retVal = BR3109_setGpioSourceCtrl(device, TAL_GPIO_SPI2_MODE_14 << 0);
		IF_ERR_RETURN_U32(retVal);
	}

    return (uint32_t)retVal;
}


uint32_t  BR3109_getSpi2Enable (br3109Device_t *device, uint8_t *spi2Enable) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t readreg = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setSpi2Enable()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif	
	retVal = BR3109_getGpioSourceCtrl(device, &readreg);
	IF_ERR_RETURN_U32(retVal);
	if((readreg & 0xF) == TAL_GPIO_SPI2_MODE_14){
		*spi2Enable = 1;
	}else{
		*spi2Enable = 0;
	}
    return (uint32_t)retVal;
}

uint32_t  BR3109_setGpio3v3Oe (br3109Device_t *device, uint16_t gpio3v3OutEn, uint16_t gpio3v3UsedMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t readreg = 0;
    static const uint16_t GPIO_OE_MASK = 0xFFF;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpio3v3Oe()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* performing gpioOutEn range check */
    if (gpio3v3OutEn > GPIO_OE_MASK)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPIO3V3_OE_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Br3109 SPI regs to set GPIO OE direction */
	retVal = BR3109_ArmWriteField(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, arm_gpio_3v3_oe), gpio3v3OutEn, gpio3v3UsedMask,0);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}

uint32_t  BR3109_setGpio3v3AnaEn (br3109Device_t *device, uint16_t gpio3v3AnaEn, uint16_t gpio3v3UsedMask) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t readreg = 0;
    static const uint16_t GPIO_OE_MASK = 0xFFF;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpio3v3AnaEn()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* performing gpioOutEn range check */
    if (gpio3v3AnaEn > GPIO_OE_MASK)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPIO3V3_OE_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Br3109 SPI regs to set GPIO OE direction */
	retVal = BR3109_ArmWriteField(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, arm_gpio_3v3_oe), gpio3v3AnaEn << 16 , gpio3v3UsedMask << 16,0);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}

uint32_t  BR3109_getGpio3v3Oe (br3109Device_t *device, uint16_t *gpio3v3OutEn) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t readreg = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpio3v3Oe()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* null pointer check */
    if (gpio3v3OutEn == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GETGPIO3V3_OE_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, arm_gpio_3v3_oe), &readreg, 1);
    IF_ERR_RETURN_U32(retVal);

    /* reconstructing byte reads into gpioOutEn word */
	*gpio3v3OutEn = readreg;

    return (uint32_t)retVal;
}

uint32_t  BR3109_setGpio3v3SourceCtrl (br3109Device_t *device, uint16_t gpio3v3SrcCtrl) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t regdata = gpio3v3SrcCtrl;
	uint32_t src_mask;
    static const uint16_t GPIO_SRC_MASK = 0x0FFF;
	int i = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpio3v3SourceCtrl()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	for(i = 0; i < 5; i++){
		if((gpio3v3SrcCtrl >> i*4)&0xF){
			src_mask |= (0xF<<(i * 4)); 
		}
	}
    /* performing gpio3v3SrcCtrl range check */
    if (gpio3v3SrcCtrl > GPIO_SRC_MASK)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPIO3V3_SRC_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Writing 3.3V GPIO configuration registers */
	retVal = BR3109_ArmWriteField(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio3v3SrcCtrl_reg), gpio3v3SrcCtrl, src_mask, 0);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}


uint32_t  BR3109_getGpio3v3SourceCtrl (br3109Device_t *device, uint16_t *gpio3v3SrcCtrl) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t regdata = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpio3v3SourceCtrl()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (gpio3v3SrcCtrl == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIO3V3_SRC_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* Reading GPIO source control registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio3v3SrcCtrl_reg), &regdata, 1);
    IF_ERR_RETURN_U32(retVal);

    /* reconstructing byte reads into gpioSrcCtrl word */
    *gpio3v3SrcCtrl = ((uint16_t)regdata);

    return (uint32_t)retVal;
}


uint32_t  BR3109_setGpio3v3PinLevel (br3109Device_t *device, uint16_t gpio3v3PinLevel) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t regdata = gpio3v3PinLevel;
    

    static const uint16_t GPIO_PIN_MASK = 0x0FFF;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setGpio3v3PinLevel()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* performing range check on gpioPinLevel */
    if (gpio3v3PinLevel > GPIO_PIN_MASK)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GPIO3V3_LEVEL_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* writing GPIO configuration registers */
	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, arm_gpio_3v3_out), &regdata, 1);
    IF_ERR_RETURN_U32(retVal);

    return (uint32_t)retVal;
}


uint32_t  BR3109_getGpio3v3PinLevel (br3109Device_t *device, uint16_t *gpio3v3PinLevel) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t regdata = 0;

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpio3v3PinLevel()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (gpio3v3PinLevel == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIO3V3_LEVEL_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* reading the registers into 2-byte array */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, arm_gpio_3v3_in), &regdata, 1);
    IF_ERR_RETURN_U32(retVal);
	
    /* reconstructing byte reads into gpioPinLevel word */
    *gpio3v3PinLevel = ((uint16_t)regdata);

    return (uint32_t)retVal;
}


uint32_t  BR3109_getGpio3v3SetLevel (br3109Device_t *device, uint16_t *gpio3v3PinSetLevel) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t regdata = 0;
	
#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpio3v3SetLevel()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    if (gpio3v3PinSetLevel == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
                BR3109_ERR_GETGPIO3V3_SETLEVEL_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }

    /* reading the registers into 2-byte array */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, arm_gpio_3v3_out), &regdata, 1);
    IF_ERR_RETURN_U32(retVal);
	
    /* reconstructing byte reads into gpioPinSetLevel word */
    *gpio3v3PinSetLevel = ((uint16_t)regdata);

    return (uint32_t)retVal;
}
#if 0
void talSimGpSources(br3109Device_t *device, uint32_t *gpIntStatus,  uint32_t *gpIntDeframerSources, uint16_t *gpIntFramerSources)
{
	/* GP_INT source masks */
	static const uint32_t GPINT_FRMR_ERROR = 0x00000010;
	static const uint32_t GPINT_DEFRMR_ERROR = 0x00000020;
	static const uint32_t GPINT_PA_PROTECT_TX1_ERROR = 0x00000040;
	static const uint32_t GPINT_PA_PROTECT_TX2_ERROR = 0x00000080;
	static const uint32_t GPINT_STREAM_ERROR = 0x000001000;

	/* deframer subsystem GP_INT sources masks */
	static const uint32_t DEFRAMER_A_BD_ERROR = 0x00000001;
	static const uint32_t DEFRAMER_A_NIT_ERROR = 0x00000002;
	static const uint32_t DEFRAMER_A_UEK_ERROR = 0x00000004;
	static const uint32_t DEFRAMER_A_ILD_ERROR = 0x00000008;
	static const uint32_t DEFRAMER_A_ILS_ERROR = 0x00000010;
	static const uint32_t DEFRAMER_A_GCS_ERROR = 0x00000020;
	static const uint32_t DEFRAMER_A_FS_ERROR = 0x00000040;
	static const uint32_t DEFRAMER_A_CSG_ERROR = 0x00000080;
	static const uint32_t DEFRAMER_A_POINTER_ALIGN_ERROR = 0x00000200;
	static const uint32_t DEFRAMER_A_SYSREF_ALIGN_ERROR = 0x00000400;
	static const uint32_t DEFRAMER_B_BD_ERROR = 0x00000800;
	static const uint32_t DEFRAMER_B_NIT_ERROR = 0x00001000;
	static const uint32_t DEFRAMER_B_UEK_ERROR = 0x00002000;
	static const uint32_t DEFRAMER_B_ILD_ERROR = 0x00004000;
	static const uint32_t DEFRAMER_B_ILS_ERROR = 0x00008000;
	static const uint32_t DEFRAMER_B_GCS_ERROR = 0x00010000;
	static const uint32_t DEFRAMER_B_FS_ERROR = 0x00020000;
	static const uint32_t DEFRAMER_B_CSG_ERROR = 0x00040000;
	static const uint32_t DEFRAMER_B_POINTER_ALIGN_ERROR = 0x00100000;
	static const uint32_t DEFRAMER_B_SYSREF_ALIGN_ERROR = 0x00200000;

	/* framer subsystem GP_INT sources masks */
	static const uint16_t FRAMER_A_FIFO_POINTER_OFFSET_ERROR = 0x00000001;
	static const uint16_t FRAMER_A_LMFC_ALIGN_ERROR = 0x00000002;
	static const uint16_t FRAMER_B_FIFO_POINTER_OFFSET_ERROR = 0x00000004;
	static const uint16_t FRAMER_B_LMFC_ALIGN_ERROR = 0x00000008;

	/* swTest Flags */
	static const uint32_t SWTST_NO_PENDING = 3;

	static const uint32_t SWTST_FRAMER_A_FIFOPTR = 6;
	static const uint32_t SWTST_FRAMER_A_LMFCALIGN = 7;
	static const uint32_t SWTST_FRAMER_B_FIFOPTR = 8;
	static const uint32_t SWTST_FRAMER_B_LMFCALIGN = 9;

	static const uint32_t SWTST_DFRAMR_A_BDERR = 10;
	static const uint32_t SWTST_DFRAMR_A_NITERR = 11;
	static const uint32_t SWTST_DFRAMR_A_UEKERR = 12;
	static const uint32_t SWTST_DFRAMR_B_BDERR = 13;
	static const uint32_t SWTST_DFRAMR_B_NITERR = 14;
	static const uint32_t SWTST_DFRAMR_B_UEKERR = 15;
	static const uint32_t SWTST_DFRAMR_A_ILDERR = 16;
	static const uint32_t SWTST_DFRAMR_B_ILDERR = 17;
	static const uint32_t SWTST_DFRAMR_A_ILSERR = 18;
	static const uint32_t SWTST_DFRAMR_B_ILSERR = 19;
	static const uint32_t SWTST_DFRAMR_A_GCSERR = 20;
	static const uint32_t SWTST_DFRAMR_B_GCSERR = 21;
	static const uint32_t SWTST_DFRAMR_A_FSERR = 22;
	static const uint32_t SWTST_DFRAMR_B_FSERR = 23;
	static const uint32_t SWTST_DFRAMR_A_CSGERR = 24;
	static const uint32_t SWTST_DFRAMR_B_CSGERR = 25;
	static const uint32_t SWTST_DFRAMR_A_FIFOERR = 26;
	static const uint32_t SWTST_DFRAMR_B_FIFOERR = 27;
	static const uint32_t SWTST_DFRAMR_A_SYSREFERR = 28;
	static const uint32_t SWTST_DFRAMR_B_SYSREFERR = 29;
	static const uint32_t SWTST_PA1_PROTECT = 30;
	static const uint32_t SWTST_PA2_PROTECT = 31;
	static const uint32_t SWTST_STREAM = 32;

	if (device->devStateInfo.swTest == SWTST_NO_PENDING) {
		*gpIntStatus = 0;
		*gpIntFramerSources = 0;
	} else if (device->devStateInfo.swTest == SWTST_FRAMER_A_FIFOPTR) {
		*gpIntStatus = GPINT_FRMR_ERROR;
		*gpIntFramerSources = FRAMER_A_FIFO_POINTER_OFFSET_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_FRAMER_A_LMFCALIGN) {
		*gpIntStatus = GPINT_FRMR_ERROR;
		*gpIntFramerSources = FRAMER_A_LMFC_ALIGN_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_FRAMER_B_FIFOPTR) {
		*gpIntStatus = GPINT_FRMR_ERROR;
		*gpIntFramerSources = FRAMER_B_FIFO_POINTER_OFFSET_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_FRAMER_B_LMFCALIGN) {
		*gpIntStatus = GPINT_FRMR_ERROR;
		*gpIntFramerSources = FRAMER_B_LMFC_ALIGN_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_BDERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_BD_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_NITERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_NIT_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_UEKERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_UEK_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_BDERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_BD_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_NITERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_NIT_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_UEKERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_UEK_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_ILDERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_ILD_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_ILDERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_ILD_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_ILSERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_ILS_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_ILSERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_ILS_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_GCSERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_GCS_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_GCSERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_GCS_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_FSERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_FS_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_FSERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_FS_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_CSGERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_CSG_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_CSGERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_CSG_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_FIFOERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_POINTER_ALIGN_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_FIFOERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_POINTER_ALIGN_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_A_SYSREFERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_A_SYSREF_ALIGN_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_DFRAMR_B_SYSREFERR) {
		*gpIntStatus = GPINT_DEFRMR_ERROR;
		*gpIntDeframerSources = DEFRAMER_B_SYSREF_ALIGN_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_PA1_PROTECT) {
		*gpIntStatus = GPINT_PA_PROTECT_TX1_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_PA2_PROTECT) {
		*gpIntStatus = GPINT_PA_PROTECT_TX2_ERROR;
	} else if (device->devStateInfo.swTest == SWTST_STREAM) {
		*gpIntStatus = GPINT_STREAM_ERROR;
	}
}

uint32_t  BR3109_gpIntHandler (br3109Device_t *device, uint32_t *gpIntStatus, br3109GpIntInformation_t *gpIntDiag) 
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

	uint32_t addressDefrmrA = 0;
	uint32_t addressDefrmrB = 0;
	uint32_t gpIntDeframerSources = 0;
	uint32_t gpIntFramerSources = 0;
	uint32_t gpIntMask = 0;
	uint32_t regdat = 0;
	uint8_t lsByte = 0;
	uint8_t msByte = 0;
	uint8_t data[8] = {0};
	uint16_t loopCnt = 0;

	static const uint8_t HIGH = 1;
	static const uint8_t LOW = 0;

	static const uint8_t JESD_SAMPLE_DATA_MASK = 0x03;
	static const uint8_t TXPOWER_RAMP_DOWN_MASK = 0x02;
	static const uint8_t DISTAMP_RAMP_DOWN_MASK = 0x29;
	static const uint8_t UPCONVERT_RAMP_DOWN_MASK = 0x39;
	static const uint8_t ARM_ECC_MASK = 0x01;
	static const uint32_t GPINT_CLKPLL_ERROR = 0x000000008;
	static const uint32_t GPINT_RFPLL_ERROR = 0x000000002;
	static const uint32_t GPINT_AUXPLL_ERROR = 0x000000004;
	static const uint32_t GPINT_WATCHDOG_TIMEOUT = 0x000000100;
	static const uint32_t GPINT_ARM_FORCE_GP_INTERRUPT = 0x000000200;
	static const uint32_t GPINT_ARM_SYSTEM_ERROR = 0x000000400;
	static const uint32_t GPINT_ARM_CAL_ERROR = 0x000000800;
	static const uint32_t GPINT_FRMR_ERROR = 0x00000010;
	static const uint32_t GPINT_DEFRMR_ERROR = 0x00000020;
	static const uint32_t GPINT_PA_PROTECT_TX1_ERROR = 0x00000040;
	static const uint32_t GPINT_PA_PROTECT_TX2_ERROR = 0x00000080;
	static const uint32_t GPINT_STREAM_ERROR = 0x000001000;
	static const uint32_t GPINT_ARM_PARITY_ERROR = 0x000002000;

	/* deframer subsystem GP_INT sources masks */
	static const uint32_t DEFRAMER_A_BD_ERROR = 0x00000001;
	static const uint32_t DEFRAMER_A_NIT_ERROR = 0x00000002;
	static const uint32_t DEFRAMER_A_UEK_ERROR = 0x00000004;
	static const uint32_t DEFRAMER_A_ILD_ERROR = 0x00000008;
	static const uint32_t DEFRAMER_A_ILS_ERROR = 0x00000010;
	static const uint32_t DEFRAMER_A_GCS_ERROR = 0x00000020;
	static const uint32_t DEFRAMER_A_FS_ERROR = 0x00000040;
	static const uint32_t DEFRAMER_A_CSG_ERROR = 0x00000080;
	static const uint32_t DEFRAMER_A_POINTER_ALIGN_ERROR = 0x00000200;
	static const uint32_t DEFRAMER_A_SYSREF_ALIGN_ERROR = 0x00000400;
	static const uint32_t DEFRAMER_B_BD_ERROR = 0x00000800;
	static const uint32_t DEFRAMER_B_NIT_ERROR = 0x00001000;
	static const uint32_t DEFRAMER_B_UEK_ERROR = 0x00002000;
	static const uint32_t DEFRAMER_B_ILD_ERROR = 0x00004000;
	static const uint32_t DEFRAMER_B_ILS_ERROR = 0x00008000;
	static const uint32_t DEFRAMER_B_GCS_ERROR = 0x00010000;
	static const uint32_t DEFRAMER_B_FS_ERROR = 0x00020000;
	static const uint32_t DEFRAMER_B_CSG_ERROR = 0x00040000;
	static const uint32_t DEFRAMER_B_POINTER_ALIGN_ERROR = 0x00100000;
	static const uint32_t DEFRAMER_B_SYSREF_ALIGN_ERROR = 0x00200000;

	/* framer subsystem GP_INT sources masks */
	static const uint16_t FRAMER_A_FIFO_POINTER_OFFSET_ERROR = 0x00000001;
	static const uint16_t FRAMER_A_LMFC_ALIGN_ERROR = 0x00000002;
	static const uint16_t FRAMER_B_FIFO_POINTER_OFFSET_ERROR = 0x00000004;
	static const uint16_t FRAMER_B_LMFC_ALIGN_ERROR = 0x00000008;

	static const uint32_t SWTST_DISABLE = 0;
	static const uint32_t SWTST_DATA_PARITY = 4;
	static const uint32_t SWTST_PROGRAM_PARITY = 5;

	static const uint32_t SWTST_DFRAMR_A_FIFOERR = 26;
	static const uint32_t SWTST_DFRAMR_B_FIFOERR = 27;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_gpIntHandler()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (gpIntStatus == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPINT_STATUS_NULL_PARM,
						  retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* initialize gpIntDiag to all zeors if requested */
	if (gpIntDiag != NULL) {
		for (loopCnt = 0; loopCnt < 8; loopCnt++) {
			gpIntDiag->data[loopCnt] = 0;
		}
	}

	/* Reading GP_INT source1 */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio_irq), &regdat,1);
	IF_ERR_RETURN(retVal);

	*gpIntStatus = regdat;

	/* Reading GP_INT 3V3 source0 */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio_3v3_irq), &regdat,1);
	IF_ERR_RETURN(retVal);
	IF_ERR_RETURN(retVal);

	*gpIntStatus |= regdat << 20;

	/* Reading GP_INT mask1 */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio_irq_mask), &regdat,1);
	IF_ERR_RETURN(retVal);

	gpIntMask = regdat;

	/* Reading GP_INT 3V3 mask0 */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, gpio_3v3_irq_mask), &regdat,1);
	IF_ERR_RETURN(retVal);

	gpIntMask |= regdat << 20;

	/* mask out all blocked interrupts */
	*gpIntStatus = *gpIntStatus & ~gpIntMask;

	if (device->devStateInfo.swTest > SWTST_DISABLE) {
		talSimGpSources(device, gpIntStatus, &gpIntDeframerSources, &gpIntFramerSources);
	}

	/* order of GP Int Status */
	/* bit14 - 15 Reserved */
	/* bit13  - TAL_GP_MASK_ARM_PARITY_ERROR */
	/* bit12  - TAL_GP_MASK_STREAM_ERROR */
	/* bit11  - TAL_GP_MASK_ARM_CALIBRATION_ERROR */
	/* bit10  - TAL_GP_MASK_ARM_SYSTEM_ERROR */
	/* bit9   - TAL_GP_MASK_ARM_FORCE_INTERRUPT */
	/* bit8   - TAL_GP_MASK_WATCHDOG_TIMEOUT */
	/* bit7   - TAL_GP_MASK_PA_PROTECTION_TX2_ERROR */
	/* bit6   - TAL_GP_MASK_PA_PROTECTION_TX1_ERROR */
	/* bit5   - TAL_GP_MASK_JESD_DEFRMER_IRQ */
	/* bit4   - TAL_GP_MASK_JESD_FRAMER_IRQ */
	/* bit3   - TAL_GP_MASK_CLK_SYNTH_LOCK */
	/* bit2   - TAL_GP_MASK_AUX_SYNTH_LOCK */
	/* bit1   - TAL_GP_MASK_RF_SYNTH_LOCK */
	/* bit0   - Reserved */

	/* is the source the CLK SYNTH LOCK */
	if (((*gpIntStatus & GPINT_CLKPLL_ERROR) > 0) && (device->devStateInfo.swTest == SWTST_DISABLE)) {
		/* set gpIntDiag CLK PLL bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[7] = 0x04;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPINT_CLKPLL_UNLOCKED, retVal, TALACT_ERR_RESET_FULL);
	}

	/* is the source the RF SYNTH LOCK */
	if (((*gpIntStatus & GPINT_RFPLL_ERROR) > 0) && (device->devStateInfo.swTest == SWTST_DISABLE)) {
		/* set gpIntDiag RF PLL bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[7] = 0x10;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPINT_RFPLL_UNLOCKED, retVal, TALACT_ERR_RESET_FULL);
	}

	/* is the source the AUX SYNTH LOCK */
	if (((*gpIntStatus & GPINT_AUXPLL_ERROR) > 0) && (device->devStateInfo.swTest == SWTST_DISABLE)) {
		/* set gpIntDiag AUX PLL bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[7] = 0x08;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPINT_AUXPLL_UNLOCKED, retVal, TALACT_ERR_RESET_FULL);
	}

	/* check for arm crash errors before issuing ARM command */
	/* is the source the WatchDog Timeout */
	if (((*gpIntStatus & GPINT_WATCHDOG_TIMEOUT) > 0) && (device->devStateInfo.swTest == SWTST_DISABLE)) {
		/* set gpIntDiag Arm WatchDog bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[7] = 0x20;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPINT_ARM_WATCHDOG_TIMEOUT, retVal, TALACT_ERR_RESET_ARM);
	}

	/* is the source the ARM FORCE GP_INT*/
	if (((*gpIntStatus & GPINT_ARM_FORCE_GP_INTERRUPT) > 0) && (device->devStateInfo.swTest == SWTST_DISABLE)) {
		/* set gpIntDiag Arm Force GP_INT bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[7] = 0x40;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPINT_ARM_FORCE_GPINT, retVal, TALACT_ERR_RESET_ARM);
	}

	/* is the source the ARM SYSTEM ERROR */
	if (((*gpIntStatus & GPINT_ARM_SYSTEM_ERROR) > 0) && (device->devStateInfo.swTest == SWTST_DISABLE)) {
		/* set gpIntDiag Arm System Error bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[7] = 0x80;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPINT_ARM_SYSTEM_ERROR, retVal, TALACT_ERR_RESET_ARM);
	}

	/* is the source the ARM CALIBRATION ERROR */
	if (((*gpIntStatus & GPINT_ARM_CAL_ERROR) > 0) && (device->devStateInfo.swTest == SWTST_DISABLE)) {
		/* set gpIntDiag RF PLL bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[7] = 0x02;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO, BR3109_ERR_GPINT_ARM_CALIBRATION_ERROR, retVal, TALACT_ERR_RERUN_INIT_CALS);
	}

	/* is the source the ARM DATA PARITY ERROR */
	halError = talSpiReadByte(device->devHalInfo, TALISE_ADDR_ARM_ECC_DATA_READBACK,
				  &lsByte);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	if (((lsByte & ARM_ECC_MASK) > 0) ||
	    (device->devStateInfo.swTest == SWTST_DATA_PARITY)) {
		*gpIntStatus |= GPINT_ARM_PARITY_ERROR;
		/* set gpIntDiag Arm Data Parity Error bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[8] = 0x01;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  TALISE_ERR_GPINT_ARM_DATA_PARITY_ERROR, retVal, TALACT_ERR_RESET_ARM);
	}

	/* is the source the ARM PROGRAM PARITY ERROR */
	halError = talSpiReadByte(device->devHalInfo, TALISE_ADDR_ARM_ECC_PROG_READBACK,
				  &lsByte);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN(retVal);

	if (((lsByte & ARM_ECC_MASK) > 0) ||
	    (device->devStateInfo.swTest == SWTST_PROGRAM_PARITY)) {
		*gpIntStatus |= GPINT_ARM_PARITY_ERROR;

		/* set gpIntDiag Arm Program Parity Error bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[8] = 0x02;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  TALISE_ERR_GPINT_ARM_PROG_PARITY_ERROR, retVal, TALACT_ERR_RESET_ARM);
	}

	/* The GP_INT is not from an ARM source and clocks are ok */
	/* assume the ARM is running ok */
	/* gather all other GP_INT sources */
	/* sending GET command to ARM */
	retVal = (talRecoveryActions_t)TALISE_sendArmCommand(device,
			TALISE_ARM_GET_OPCODE, &armExtData[0], 1);
	IF_ERR_RETURN_U32(retVal);

	/* waiting for command completion */
	retVal = (talRecoveryActions_t)TALISE_waitArmCmdStatus(device,
			TALISE_ARM_GET_OPCODE, &cmdStatusByte, GETGPINTSRC_TIMEOUT_US,
			GETGPINTSRC_INTERVAL_US);
	if((cmdStatusByte >> 1) > 0) {
		/* set gpIntDiag Arm Command Wait TimeOut bit if requested */
		if (gpIntDiag != NULL) {
			gpIntDiag->data[8] = 0x04;
		}

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_ARM_CMD_ERR,
						  ARMCMD_ERRCODE(TALISE_ARM_GET_OPCODE, TALISE_ARM_OBJECTID_GO_GET_GP_INT_SOURCE,
								  cmdStatusByte), retVal, TALACT_ERR_RESET_ARM);
	} else {
		IF_ERR_RETURN_U32(retVal);
	}

	/* read out the accumulated sources */
	retVal = (talRecoveryActions_t)TALISE_readArmMem(device,
			TALISE_ADDR_ARM_START_DATA_ADDR, &data[0], 8, 0);
	IF_ERR_RETURN_U32(retVal);

	if (device->devStateInfo.swTest > 0) {
		if (gpIntDiag == NULL) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_GPINTDIAG_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
		} else {
			for (loopCnt = 0; loopCnt < sizeof(data); loopCnt++) {
				data[loopCnt] = gpIntDiag->data[loopCnt];
			}
		}

	} else {
		/* save all source bits if diagnostic requested */
		if (gpIntDiag != NULL) {
			for (loopCnt = 0; loopCnt < 8; loopCnt++) {
				gpIntDiag->data[loopCnt] = data[loopCnt];
			}
		}
	}

	/* assemble sources by subsystems */
	if (device->devStateInfo.swTest == 0) {
		gpIntDeframerSources = (uint32_t)(data[3] & 0xE0) >> 5 |
				       (uint32_t)data[4] << 3 | (uint32_t) data[5] << 11 | (uint32_t) (
					       data[6] & 0x03) << 19;
		gpIntFramerSources = (uint16_t)(data[6] & 0x78) >> 3;
	}

	/* is the source the framer */
	if ((*gpIntStatus & GPINT_FRMR_ERROR) > 0) {
		/* framer handler here */
		if (((gpIntFramerSources & FRAMER_A_FIFO_POINTER_OFFSET_ERROR) > 0) ||
		    ((gpIntFramerSources & FRAMER_A_LMFC_ALIGN_ERROR) > 0)) {
			/* Clear Framer A Fifo error */
			halError = brSpiWriteField(device->devHalInfo,
						    TALISE_ADDR_JESD_FRAMER_ASYNC_PTR_DBG_0, 0, 0x80, 7);
			retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
						  TALACT_ERR_RESET_SPI);
			IF_ERR_RETURN(retVal);

			if (gpIntDiag != NULL) {
				gpIntDiag->framer = TAL_FRAMER_A;
			}

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_FRAMERA, retVal, TALACT_ERR_RESET_JESD204FRAMERA);
		} else if (((gpIntFramerSources & FRAMER_B_FIFO_POINTER_OFFSET_ERROR) > 0) ||
			   ((gpIntFramerSources & FRAMER_B_LMFC_ALIGN_ERROR) > 0)) {
			/* Clear Framer A Fifo error */
			halError = brSpiWriteField(device->devHalInfo,
						    TALISE_ADDR_JESD_FRAMER_ASYNC_PTR_DBG_1, 0, 0x80, 7);
			retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
						  TALACT_ERR_RESET_SPI);
			IF_ERR_RETURN(retVal);

			if (gpIntDiag != NULL) {
				gpIntDiag->framer = TAL_FRAMER_B;
			}

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_FRAMERB, retVal, TALACT_ERR_RESET_JESD204FRAMERB);
		}
	}

	/* is the source the deframer */
	if ((*gpIntStatus & GPINT_DEFRMR_ERROR) > 0) {
		/* Deframer A */
		if (((gpIntDeframerSources & DEFRAMER_A_BD_ERROR) > 0) ||
		    ((gpIntDeframerSources & DEFRAMER_A_NIT_ERROR) > 0) ||
		    ((gpIntDeframerSources & DEFRAMER_A_UEK_ERROR) > 0)) {
			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_A;
				retVal = talFindDfrmrLaneCntErr(device, TAL_DEFRAMER_A,
								&gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_A);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERA, retVal, TALACT_ERR_BBIC_LOG_ERROR);
		} /* end DEFRAMER_A */

		/* Deframer B */
		else if (((gpIntDeframerSources & DEFRAMER_B_BD_ERROR) > 0) ||
			 ((gpIntDeframerSources & DEFRAMER_B_NIT_ERROR) > 0) ||
			 ((gpIntDeframerSources & DEFRAMER_B_UEK_ERROR) > 0)) {
			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_B;
				retVal = talFindDfrmrLaneCntErr(device, TAL_DEFRAMER_B,
								&gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_B);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERB, retVal, TALACT_ERR_BBIC_LOG_ERROR);
		} /* end DEFRAMER_B */

		/* Deframer A */
		else if ((gpIntDeframerSources & DEFRAMER_A_ILD_ERROR) > 0) {
			addressDefrmrA = TALISE_ADDR_JESD_DEFRAMER_IP_OBS22_0;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_A;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrA, LOW,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_A);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERA, retVal, TALACT_ERR_RESET_JESD204DEFRAMERA);
		} /* end DEFRAMER_A */

		/* Deframer B */
		else if ((gpIntDeframerSources & DEFRAMER_B_ILD_ERROR) > 0) {
			addressDefrmrB = TALISE_ADDR_JESD_DEFRAMER_IP_OBS22_1;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_B;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrB, LOW,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_B);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERB, retVal, TALACT_ERR_RESET_JESD204DEFRAMERB);
		} /* end DEFRAMER_B */

		/* Deframer A */
		else if ((gpIntDeframerSources & DEFRAMER_A_ILS_ERROR) > 0) {
			addressDefrmrA = TALISE_ADDR_JESD_DEFRAMER_IP_OBS22_0;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_A;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrA, HIGH,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_A);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERA, retVal, TALACT_ERR_RESET_JESD204DEFRAMERA);
		} /* end DEFRAMER_A */

		/* Deframer B */
		else if ((gpIntDeframerSources & DEFRAMER_B_ILS_ERROR) > 0) {
			addressDefrmrB = TALISE_ADDR_JESD_DEFRAMER_IP_OBS22_1;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_B;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrB, HIGH,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_B);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERB, retVal, TALACT_ERR_RESET_JESD204DEFRAMERB);
		} /* end DEFRAMER_B */

		/* GCS Error */
		else if ((gpIntDeframerSources & DEFRAMER_A_GCS_ERROR) > 0) {
			addressDefrmrA = TALISE_ADDR_JESD_DEFRAMER_IP_OBS21_0;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_A;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrA, LOW,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_A);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERA, retVal, TALACT_ERR_RESET_JESD204DEFRAMERA);
		} /* end DEFRAMER_A_GCS_ERROR */

		/* GCS Error */
		else if ((gpIntDeframerSources & DEFRAMER_B_GCS_ERROR) > 0) {
			addressDefrmrB = TALISE_ADDR_JESD_DEFRAMER_IP_OBS21_1;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_B;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrB, LOW,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_B);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERB, retVal, TALACT_ERR_RESET_JESD204DEFRAMERB);
		} /* end DEFRAMERB_GCS_ERROR */

		/* FS Error */
		else if ((gpIntDeframerSources & DEFRAMER_A_FS_ERROR) > 0) {
			addressDefrmrA = TALISE_ADDR_JESD_DEFRAMER_IP_OBS21_0;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_A;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrA, HIGH,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_A);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERA, retVal, TALACT_ERR_RESET_JESD204DEFRAMERA);
		} /* end DEFRAMER_A_FS_ERROR */

		/* FS Error */
		else if ((gpIntDeframerSources & DEFRAMER_B_FS_ERROR) > 0) {
			addressDefrmrB = TALISE_ADDR_JESD_DEFRAMER_IP_OBS21_1;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_B;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrB, HIGH,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_B);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERB, retVal, TALACT_ERR_RESET_JESD204DEFRAMERB);
		} /* end DEFRAMER_B_FS_ERROR */

		/* CSG Error */
		else if ((gpIntDeframerSources & DEFRAMER_A_CSG_ERROR) > 0) {
			addressDefrmrA = TALISE_ADDR_JESD_DEFRAMER_IP_OBS20_0;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_A;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrA, HIGH,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_A);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERA, retVal, TALACT_ERR_RESET_JESD204DEFRAMERA);
		} /* end DEFRAMER_A_CSG_ERROR */

		/* CSG Error */
		else if ((gpIntDeframerSources & DEFRAMER_B_CSG_ERROR) > 0) {
			addressDefrmrB = TALISE_ADDR_JESD_DEFRAMER_IP_OBS20_1;

			if (gpIntDiag != NULL) {
				gpIntDiag->deframer = TAL_DEFRAMER_B;
				retVal = talFindDfrmrLaneErr(device, addressDefrmrB, HIGH,
							     &gpIntDiag->deframerInputsMask);
				IF_ERR_RETURN_U32(retVal);
			}

			retVal = (talRecoveryActions_t)TALISE_clearDfrmIrq(device, TAL_DEFRAMER_B);
			IF_ERR_RETURN_U32(retVal);

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERB, retVal, TALACT_ERR_RESET_JESD204DEFRAMERB);
		} /* end DEFRAMER_A_CSG_ERROR */

		/* FIFO PTR Error */
		else if ((gpIntDeframerSources & DEFRAMER_A_POINTER_ALIGN_ERROR) > 0) {
			if (gpIntDiag != NULL) {
				gpIntDiag->deframerInputsMask = 0;
			}

			for (loopCnt = 0; loopCnt < 4; loopCnt++) {
				/* Check Lane */
				halError = talSpiReadByte(device->devHalInfo,
							  (TALISE_ADDR_JESD_DEFRAMER_IP_OBS42_1 + loopCnt), &lsByte);
				retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
							  TALACT_ERR_RESET_SPI);
				IF_ERR_RETURN(retVal);

				if (((lsByte & 0x01) > 0)
				    || (device->devStateInfo.swTest == SWTST_DFRAMR_A_FIFOERR)) {
					if (gpIntDiag != NULL) {
						gpIntDiag->deframerInputsMask |= (0x01 << loopCnt);
						gpIntDiag->deframer = TAL_DEFRAMER_A;
					}

					/* clear the pointer error interrupt */
					halError = talSpiWriteByte(device->devHalInfo,
								   (TALISE_ADDR_JESD_DEFRAMER_IP_OBS42_1 + loopCnt), 0x00);
					retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
								  TALACT_ERR_RESET_SPI);
					IF_ERR_RETURN_U32(retVal);
				}
			}

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERA, retVal, TALACT_ERR_RESET_JESD204DEFRAMERA);
		} /* end DEFRAMER_A_FIFO_ERROR */

		/* FIFO PTR Error */
		else if ((gpIntDeframerSources & DEFRAMER_B_POINTER_ALIGN_ERROR) > 0) {
			if (gpIntDiag != NULL) {
				gpIntDiag->deframerInputsMask = 0;
			}

			for (loopCnt = 0; loopCnt < 4; loopCnt++) {
				/* Check Lane */
				halError = talSpiReadByte(device->devHalInfo,
							  (TALISE_ADDR_JESD_DEFRAMER_IP_OBS42_1 + loopCnt), &lsByte);
				retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
							  TALACT_ERR_RESET_SPI);
				IF_ERR_RETURN(retVal);

				if (((lsByte & 0x01) > 0)
				    || (device->devStateInfo.swTest == SWTST_DFRAMR_B_FIFOERR)) {
					if (gpIntDiag != NULL) {
						gpIntDiag->deframerInputsMask |= (0x01 << loopCnt);
						gpIntDiag->deframer = TAL_DEFRAMER_B;
					}

					/* clear the pointer error interrupt */
					halError = talSpiWriteByte(device->devHalInfo,
								   (TALISE_ADDR_JESD_DEFRAMER_IP_OBS42_1 + loopCnt), 0x00);
					retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
								  TALACT_ERR_RESET_SPI);
					IF_ERR_RETURN_U32(retVal);
				}
			}

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERB, retVal, TALACT_ERR_RESET_JESD204DEFRAMERB);
		} /* end DEFRAMER_B_FIFO_ERROR */

		/* SYSREF Phase Error */
		else if ((gpIntDeframerSources & DEFRAMER_A_SYSREF_ALIGN_ERROR) > 0) {
			if (gpIntDiag != NULL) {
				gpIntDiag->deframerInputsMask = 0;
				gpIntDiag->deframer = TAL_DEFRAMER_A;
			}

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERA, retVal, TALACT_ERR_RESET_JESD204DEFRAMERA);
		} /* end DEFRAMER_A_SYSREF_PHASE_ERROR */

		/* SYSREF Phase Error */
		else if ((gpIntDeframerSources & DEFRAMER_B_SYSREF_ALIGN_ERROR) > 0) {
			if (gpIntDiag != NULL) {
				gpIntDiag->deframerInputsMask = 0;
				gpIntDiag->deframer = TAL_DEFRAMER_B;
			}

			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_DEFRAMERB, retVal, TALACT_ERR_RESET_JESD204DEFRAMERB);
		} /* end DEFRAMER_A_SYSREF_PHASE_ERROR */

	} /* end deframer */

	/* is the source the PA Protection TX1 */
	/* is the source the PA Protection */
	if (((*gpIntStatus & GPINT_PA_PROTECT_TX1_ERROR) > 0) ||
	    ((*gpIntStatus & GPINT_PA_PROTECT_TX2_ERROR) > 0)) {

		/* Write PA Protect Config */
		halError = brSpiWriteField(device->devHalInfo,
					    TALISE_ADDR_PA_PROTECTION_CONFIGURATION, 0x00, 0x80, 0x80);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		/* is the source the PA Protection TX2 */
		if((*gpIntStatus & GPINT_PA_PROTECT_TX2_ERROR) > 0) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_PA_PROTECT_CH2, retVal, TALACT_ERR_REDUCE_TXSAMPLE_PWR);
		}

		/* is the source the PA Protection TX1 */
		else if((*gpIntStatus & GPINT_PA_PROTECT_TX1_ERROR) > 0) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
							  TALISE_ERR_GPINT_PA_PROTECT_CH1, retVal, TALACT_ERR_REDUCE_TXSAMPLE_PWR);
		}

	}

	/* is the source the stream */
	if (*gpIntStatus & GPINT_STREAM_ERROR) {
		/* disable JESD data for both channels */
		halError = talSpiWriteByte(device->devHalInfo,
					   TALISE_ADDR_JESD_DEFRAMER_SAMPLE_DISABLE_CH1, JESD_SAMPLE_DATA_MASK);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		halError = talSpiWriteByte(device->devHalInfo,
					   TALISE_ADDR_JESD_DEFRAMER_SAMPLE_DISABLE_CH2, JESD_SAMPLE_DATA_MASK);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		/* Ramp down TX power for both channels */
		halError = talSpiWriteByte(device->devHalInfo, TALISE_ADDR_TDD_RAMP_TX1,
					   TXPOWER_RAMP_DOWN_MASK);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		halError = talSpiWriteByte(device->devHalInfo, TALISE_ADDR_TDD_RAMP_TX2,
					   TXPOWER_RAMP_DOWN_MASK);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		/* Ramp down distortion amp, etc */
		halError = talSpiWriteByte(device->devHalInfo, TALISE_ADDR_TX1_PD,
					   DISTAMP_RAMP_DOWN_MASK);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		halError = talSpiWriteByte(device->devHalInfo, TALISE_ADDR_TX2_PD,
					   DISTAMP_RAMP_DOWN_MASK);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		/* power down upconverter */
		halError = talSpiWriteByte(device->devHalInfo, TALISE_ADDR_TX1_PD,
					   UPCONVERT_RAMP_DOWN_MASK);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		halError = talSpiWriteByte(device->devHalInfo, TALISE_ADDR_TX2_PD,
					   UPCONVERT_RAMP_DOWN_MASK);
		retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
					  TALACT_ERR_RESET_SPI);
		IF_ERR_RETURN_U32(retVal);

		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  TALISE_ERR_GPINT_STREAM_ERROR, retVal, TALACT_ERR_RESET_FULL);
	}

	return (uint32_t)retVal;
}
#endif
uint32_t BR3109_setAuxAdcPinModeGpio(br3109Device_t *device, br3109GpioPinSel_t pinModeGpio)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t regdat = 0;
	uint16_t gpio3v3SrcCtrl = 0;

	/*Range check that 1.8v GPIO pin to be assigned for Pin mode Aux ADC start signal is valid*/
	if((pinModeGpio > TAL_GPIO_12) && (pinModeGpio != TAL_GPIO_INVALID)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_SETAUXADCPINMODEGPIO_INV_GPIO, retVal, TALACT_ERR_CHECK_PARAM);
	}

	retVal = BR3109_getGpio3v3SourceCtrl(device, &gpio3v3SrcCtrl);
	IF_ERR_RETURN_U32(retVal);
	if(gpio3v3SrcCtrl &(0xF << (pinModeGpio/4))){//已经使用
		device->devStateInfo.usedGpiopins |= (1<<pinModeGpio);
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_SETAUXADCPINMODEGPIO_INV_GPIO, retVal, TALACT_ERR_CHECK_PARAM);
	}
		
	//set to analog mode
	gpio3v3SrcCtrl = 0x7 << (pinModeGpio/4);
	retVal = BR3109_setGpio3v3SourceCtrl(device, gpio3v3SrcCtrl);
	IF_ERR_RETURN_U32(retVal);
	regdat = (1 << pinModeGpio) << 16;
	retVal = BR3109_ArmWriteField(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, arm_gpio_3v3_oe),
			regdat, regdat, 0);
	device->devStateInfo.usedGpiopins |= (1<<pinModeGpio);

	return (uint32_t)retVal;
}

uint32_t BR3109_getAuxAdcPinModeGpio(br3109Device_t *device, br3109GpioPinSel_t *pinModeGpio)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t regdat = 0;
	uint8_t i = 0;
	uint8_t armGetGpioStatus = 0;
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR+type_OffSet(BR3109_GPIO_TypeDef, arm_gpio_3v3_oe),
				&regdat, 1);
	IF_ERR_RETURN_U32(retVal);
	regdat = (regdat >> 16) & 0x0FFF;
	if(regdat > 0){
		for(i = 0; i < 12; i ++){
			if((regdat >> i) & 0x1){
				break;
			}
		}
	}
	armGetGpioStatus = i;
	if (armGetGpioStatus <= (uint8_t)TAL_GPIO_12) {
		*pinModeGpio = (br3109GpioPinSel_t)(armGetGpioStatus & 0x0F);
	} else {
		*pinModeGpio = TAL_GPIO_INVALID;
	}

	return (uint32_t)retVal;
}

/**-----------------------BR3109ES2 API Update----------------------------------------*/

uint32_t BR3109_setOrxTxPinSel(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setOrxTxPinSel()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* performing gpioOutEn range check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETORXTXPINSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if (config->p0.gpioPinSel > TAL_GPIO_15 ||
		config->p1.gpioPinSel > TAL_GPIO_15 ||
		config->p2.gpioPinSel > TAL_GPIO_15 ||
		config->p3.gpioPinSel > TAL_GPIO_15)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETORXTXPINSEL_INV_PIN, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Br3109 SPI regs to set ORX PIN SEL */
	// unpack config
	writeval = (config->p3.enable & 0x1) << 28 |
			   (config->p3.gpioPinSel & 0xF) << 24 |
			   (config->p2.enable & 0x1) << 20 |
			   (config->p2.gpioPinSel & 0xF) << 16 |
			   (config->p1.enable & 0x1) << 12 |
			   (config->p1.gpioPinSel & 0xF) << 8 |
			   (config->p0.enable & 0x1) << 4 |
			   (config->p0.gpioPinSel & 0xF) << 0;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, orx_tx_sel_pin_sel), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getOrxTxPinSel(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getGpioOe()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETORXTXPINSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, orx_tx_sel_pin_sel), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack config
	config->p0.gpioPinSel = (readval >> 0) & 0xF;
	config->p0.enable = (readval >> 4) & 0x1;
	config->p1.gpioPinSel = (readval >> 8) & 0xF;
	config->p1.enable = (readval >> 12) & 0x1;
	config->p2.gpioPinSel = (readval >> 16) & 0xF;
	config->p2.enable = (readval >> 20) & 0x1;
	config->p3.gpioPinSel = (readval >> 24) & 0xF;
	config->p3.enable = (readval >> 28) & 0x1;

	return (uint32_t)retVal;
}

uint32_t BR3109_setRxMgcSel(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setRxMgcSel()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* performing gpioOutEn range check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETRXMGCSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if (config->p0.gpioPinSel > TAL_GPIO_15 ||
		config->p1.gpioPinSel > TAL_GPIO_15 ||
		config->p2.gpioPinSel > TAL_GPIO_15 ||
		config->p3.gpioPinSel > TAL_GPIO_15)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETRXMGCSEL_INV_PIN, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Br3109 SPI regs to set ORX PIN SEL */
	// unpack config
	writeval = (config->p3.enable & 0x1) << 28 |
			   (config->p3.gpioPinSel & 0xF) << 24 |
			   (config->p2.enable & 0x1) << 20 |
			   (config->p2.gpioPinSel & 0xF) << 16 |
			   (config->p1.enable & 0x1) << 12 |
			   (config->p1.gpioPinSel & 0xF) << 8 |
			   (config->p0.enable & 0x1) << 4 |
			   (config->p0.gpioPinSel & 0xF) << 0;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, rx_mgc_sel), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getRxMgcSel(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getRxMgcSel()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETRXMGCSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, rx_mgc_sel), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack config
	config->p0.gpioPinSel = (readval >> 0) & 0xF;
	config->p0.enable = (readval >> 4) & 0x1;
	config->p1.gpioPinSel = (readval >> 8) & 0xF;
	config->p1.enable = (readval >> 12) & 0x1;
	config->p2.gpioPinSel = (readval >> 16) & 0xF;
	config->p2.enable = (readval >> 20) & 0x1;
	config->p3.gpioPinSel = (readval >> 24) & 0xF;
	config->p3.enable = (readval >> 28) & 0x1;

	return (uint32_t)retVal;
}

uint32_t BR3109_setOrxMgcSel(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setOrxMgcSel()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* performing gpioOutEn range check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETORXMGCSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if (config->p0.gpioPinSel > TAL_GPIO_15 ||
		config->p1.gpioPinSel > TAL_GPIO_15 ||
		config->p2.gpioPinSel > TAL_GPIO_15 ||
		config->p3.gpioPinSel > TAL_GPIO_15)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETORXMGCSEL_INV_PIN, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Br3109 SPI regs to set ORX PIN SEL */
	// unpack config
	writeval = (config->p3.enable & 0x1) << 28 |
			   (config->p3.gpioPinSel & 0xF) << 24 |
			   (config->p2.enable & 0x1) << 20 |
			   (config->p2.gpioPinSel & 0xF) << 16 |
			   (config->p1.enable & 0x1) << 12 |
			   (config->p1.gpioPinSel & 0xF) << 8 |
			   (config->p0.enable & 0x1) << 4 |
			   (config->p0.gpioPinSel & 0xF) << 0;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, orx_mgc_sel), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getOrxMgcSel(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getOrxMgcSel()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETORXMGCSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, orx_mgc_sel), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack config
	config->p0.gpioPinSel = (readval >> 0) & 0xF;
	config->p0.enable = (readval >> 4) & 0x1;
	config->p1.gpioPinSel = (readval >> 8) & 0xF;
	config->p1.enable = (readval >> 12) & 0x1;
	config->p2.gpioPinSel = (readval >> 16) & 0xF;
	config->p2.enable = (readval >> 20) & 0x1;
	config->p3.gpioPinSel = (readval >> 24) & 0xF;
	config->p3.enable = (readval >> 28) & 0x1;

	return (uint32_t)retVal;
}

uint32_t BR3109_setTxAttnSel(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setTxAttnSel()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* performing gpioOutEn range check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETTXATTNSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if (config->p0.gpioPinSel > TAL_GPIO_15 ||
		config->p1.gpioPinSel > TAL_GPIO_15 ||
		config->p2.gpioPinSel > TAL_GPIO_15 ||
		config->p3.gpioPinSel > TAL_GPIO_15)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETTXATTNSEL_INV_PIN, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Br3109 SPI regs to set ORX PIN SEL */
	// unpack config
	writeval = (config->p3.enable & 0x1) << 28 |
			   (config->p3.gpioPinSel & 0xF) << 24 |
			   (config->p2.enable & 0x1) << 20 |
			   (config->p2.gpioPinSel & 0xF) << 16 |
			   (config->p1.enable & 0x1) << 12 |
			   (config->p1.gpioPinSel & 0xF) << 8 |
			   (config->p0.enable & 0x1) << 4 |
			   (config->p0.gpioPinSel & 0xF) << 0;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, tx_attn_sel), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getTxAttnSel(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getTxAttnSel()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETTXATTNSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, tx_attn_sel), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack config
	config->p0.gpioPinSel = (readval >> 0) & 0xF;
	config->p0.enable = (readval >> 4) & 0x1;
	config->p1.gpioPinSel = (readval >> 8) & 0xF;
	config->p1.enable = (readval >> 12) & 0x1;
	config->p2.gpioPinSel = (readval >> 16) & 0xF;
	config->p2.enable = (readval >> 20) & 0x1;
	config->p3.gpioPinSel = (readval >> 24) & 0xF;
	config->p3.enable = (readval >> 28) & 0x1;

	return (uint32_t)retVal;
}

uint32_t BR3109_setTxAttnState(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setTxAttnState()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* performing gpioOutEn range check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETTXATTNSTATE_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if (config->p0.gpioPinSel > TAL_GPIO_15 ||
		config->p1.gpioPinSel > TAL_GPIO_15 ||
		config->p2.gpioPinSel > TAL_GPIO_15)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETTXATTNSTATE_INV_PIN, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Br3109 SPI regs to set ORX PIN SEL */
	// unpack config
	writeval = (config->p2.enable & 0x1) << 20 |
			   (config->p2.gpioPinSel & 0xF) << 16 |
			   (config->p1.enable & 0x1) << 12 |
			   (config->p1.gpioPinSel & 0xF) << 8 |
			   (config->p0.enable & 0x1) << 4 |
			   (config->p0.gpioPinSel & 0xF) << 0;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, tx_attn_state_pin_sel), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getTxAttnState(br3109Device_t *device, br3109GpioShortConfig_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getTxAttnState()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETTXATTNSTATE_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, tx_attn_state_pin_sel), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack config
	config->p0.gpioPinSel = (readval >> 0) & 0xF;
	config->p0.enable = (readval >> 4) & 0x1;
	config->p1.gpioPinSel = (readval >> 8) & 0xF;
	config->p1.enable = (readval >> 12) & 0x1;
	config->p2.gpioPinSel = (readval >> 16) & 0xF;
	config->p2.enable = (readval >> 20) & 0x1;

	return (uint32_t)retVal;
}
/**
 * spi1_2:设置spi选择：0：SPI1 ， 1：SPI2
 * mode : 0: 双向传输，即3线SPI 1：单向传输，即4线SPI
*/
uint32_t BR3109_SetSPIPinMode(br3109Device_t *device,SPISel_t spi1_2, uint32_t mode)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_SetSPIPinMode()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_GPIO);
#endif
	/* Reading GPIO output enable registers */
	retVal = brSpiWriteField(device->devHalInfo, HOST_APB_WRITE + APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, spi_sdio_bdir_off),
			 mode << (spi1_2 == SPI1_SEL ? 0 : 1), 0x1 << (spi1_2 == SPI1_SEL ? 0 : 1), 0);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_setPwm1Period(br3109Device_t *device, uint16_t pwm1_period, uint16_t pwm1_low_count)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setPwm1Period()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/**TODO:perform range check*/

	/* assemble register*/
	writeval = (uint32_t)pwm1_low_count << 16 | pwm1_period;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, pwm1_period_adj), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getPwm1Period(br3109Device_t *device, uint16_t *pwm1_period, uint16_t *pwm1_low_count)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getTxAttnState()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (pwm1_period == NULL || pwm1_low_count == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETPWMPERIOD_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, pwm1_period_adj), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack config
	*pwm1_period = readval & 0xFFFF;
	*pwm1_low_count = readval >> 16 & 0xFFFF;
	return (uint32_t)retVal;
}

uint32_t BR3109_setPwm1Enable(br3109Device_t *device, uint8_t pwm1_enable)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setPwm1Enable()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/*perform range check*/
	if (pwm1_enable != 0 && pwm1_enable != 1)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETPWMENABLE_INV_VAL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* assemble register*/
	writeval = (uint32_t)pwm1_enable;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, pwm1_enable_adj), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getPwm1Enable(br3109Device_t *device, uint8_t *pwm1_enable)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getPwm1Enable()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (pwm1_enable == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETPWMENABLE_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, pwm1_enable_adj), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack info
	*pwm1_enable = (uint8_t)readval;

	return (uint32_t)retVal;
}

uint32_t BR3109_setPwm2Period(br3109Device_t *device, uint16_t pwm2_period, uint16_t pwm2_low_count)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setPwm2Period()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/**TODO:perform range check*/

	/* assemble register*/
	writeval = (uint32_t)pwm2_low_count << 16 | pwm2_period;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, pwm2_period_adj), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getPwm2Period(br3109Device_t *device, uint16_t *pwm2_period, uint16_t *pwm2_low_count)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getPwm2Period()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (pwm2_period == NULL || pwm2_low_count == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETPWMPERIOD_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, pwm2_period_adj), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack config
	*pwm2_period = readval & 0xFFFF;
	*pwm2_low_count = readval >> 16 & 0xFFFF;
	return (uint32_t)retVal;
}

uint32_t BR3109_setPwm2Enable(br3109Device_t *device, uint8_t pwm2_enable)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setPwm2Enable()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/*perform range check*/
	if (pwm2_enable != 0 && pwm2_enable != 1)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETPWMENABLE_INV_VAL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* assemble register*/
	writeval = (uint32_t)pwm2_enable;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, pwm2_enable_adj), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getPwm2Enable(br3109Device_t *device, uint8_t *pwm2_enable)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getPwm2Enable()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (pwm2_enable == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETPWMENABLE_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, pwm2_enable_adj), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack info
	*pwm2_enable = (uint8_t)readval;

	return (uint32_t)retVal;
}

uint32_t BR3109_getRxConfig(br3109Device_t *device, uint32_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getRxConfig()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETRXCFG_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, rx_mgc_read_back), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack info
	*config = readval;

	return (uint32_t)retVal;
}

uint32_t BR3109_setOrxEnableGpio(br3109Device_t *device, br3109GpioConfigOrxGpio_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t writeval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setOrxEnableGpio()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* performing gpioOutEn range check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETORXENABLEGPIOSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if (config->orx1_enable_gpio_sel.gpioPinSel > TAL_GPIO_15 ||
		config->orx2_enable_gpio_sel.gpioPinSel > TAL_GPIO_15)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETORXENABLEGPIOSEL_INV_VAL, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if ((config->orx1_enble_sel != 0 && config->orx1_enble_sel != 1) ||
		(config->orx2_enble_sel != 0 && config->orx2_enble_sel != 1))
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_SETORXENABLEGPIOSEL_INV_VAL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Br3109 SPI regs to set ORX PIN SEL */
	// unpack config
	writeval = (config->orx2_enble_sel & 0x1) << 24 |
			   (config->orx2_enable_gpio_sel.enable & 0x1) << 20 |
			   (config->orx2_enable_gpio_sel.gpioPinSel & 0xF) << 16 |
			   (config->orx1_enble_sel & 0x1) << 8 |
			   (config->orx1_enable_gpio_sel.enable & 0x1) << 4 |
			   (config->orx1_enable_gpio_sel.gpioPinSel & 0xF) << 0;

	retVal = BR3109_armMemoryCmd_blk_write(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, orx_enable_gpio_sel), &writeval, 1);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getOrxEnableGpio(br3109Device_t *device, br3109GpioConfigOrxGpio_t *config)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t readval = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getOrxEnableGpio()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* null pointer check */
	if (config == NULL)
	{
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
										  BR3109_ERR_GETORXENABLEGPIOSEL_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Reading GPIO output enable registers */
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_IOCTRL_BASEADDR + type_OffSet(BR3109_GPIO_TypeDef, orx_enable_gpio_sel), &readval, 1);
	IF_ERR_RETURN_U32(retVal);

	// pack config
	config->orx1_enable_gpio_sel.gpioPinSel = (readval >> 0) & 0xF;
	config->orx1_enable_gpio_sel.enable = (readval >> 4) & 0x1;
	config->orx1_enble_sel = (uint8_t)(readval >> 8) & 0x1;
	config->orx2_enable_gpio_sel.gpioPinSel = (readval >> 16) & 0xF;
	config->orx2_enable_gpio_sel.enable = (readval >> 20) & 0x1;
	config->orx2_enble_sel = (uint8_t)(readval >> 24) & 0x1;

	return (uint32_t)retVal;
}

#if 0
uint32_t BR3109_startAuxAdc(br3109Device_t *device,
			    br3109AuxAdcConfig_t *auxAdcConfig)
{
//	static const uint8_t ARM_AUXADC_CFG_CMD_SIZE_BYTES = 1;
	static const uint16_t MIN_AUXADC_SAMPLES = 1;
	static const uint16_t MAX_AUXADC_SAMPLES = 1000;
	static const uint16_t MIN_AUXADC_SAMPLING_PERIOD_US = 15;
//	static const uint8_t AUXADC_CONFIG_SIZE_BYTES = 6;
	static const uint32_t SWTST_NUMSAMPLES_OUT_OF_RANGE = 1;
	static const uint32_t SWTST_SAMPLING_PERIOD_OUT_OF_RANGE = 2;
	static const uint32_t SWTST_AUXADC_CH_OUT_OF_RANGE = 3;

	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint8_t cmdStatusByte = 0;
	uint8_t armAuxAdcCfgCmd[ARM_AUXADC_CFG_CMD_SIZE_BYTES];
	uint8_t auxAdcConfigByteArr[AUXADC_CONFIG_SIZE_BYTES];

	/*Initialize armAuxAdcCfgCmd C-99 style*/
	armAuxAdcCfgCmd[0] = (uint8_t)BR3109_ARM_OBJECTID_GS_AUX_ADC;

	/*Null Check auxAdcConfig structure*/
	if (auxAdcConfig == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_STARTAUXADC_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*Range check that AuxADC Channel selected is valid*/
	if((auxAdcConfig->auxAdcChannelSel != TAL_AUXADC_CH0) &&
	    (auxAdcConfig->auxAdcChannelSel != TAL_AUXADC_CH1) &&
	    (auxAdcConfig->auxAdcChannelSel != TAL_AUXADC_CH2) &&
	    (auxAdcConfig->auxAdcChannelSel != TAL_AUXADC_CH3)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_STARTAUXADC_INV_CHANNEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*Range check that AuxADC mode is valid*/
	if((auxAdcConfig->auxAdcMode != TAL_AUXADC_NONPIN_MODE) &&
	    (auxAdcConfig->auxAdcMode != TAL_AUXADC_PIN_MODE)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_STARTAUXADC_INV_MODE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*Range check that number of samples for conversion is within range*/
	if((auxAdcConfig->numSamples < MIN_AUXADC_SAMPLES) ||
	    (auxAdcConfig->numSamples > MAX_AUXADC_SAMPLES)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_STARTAUXADC_INV_NUM_SAMPLES, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*Range check that sampling period is at least minimum sampling period for a non-pin mode*/
	if((auxAdcConfig->auxAdcMode == TAL_AUXADC_NONPIN_MODE) &&
	    (auxAdcConfig->samplingPeriod_us < MIN_AUXADC_SAMPLING_PERIOD_US)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_STARTAUXADC_INV_SAMPLING_PERIOD, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*For a pin mode, sampling period member is not used. Transmit a minimum sampling period parameter to ARM*/
	if(auxAdcConfig->auxAdcMode == TAL_AUXADC_PIN_MODE) {
		auxAdcConfig->samplingPeriod_us = MIN_AUXADC_SAMPLING_PERIOD_US;
	}

	if(device->devStateInfo.swTest == SWTST_NUMSAMPLES_OUT_OF_RANGE) {
		/*Aux ADC samples should be in the range 1-1000. Passing 1200 to ARM should throw and ARM exception*/
		auxAdcConfig->numSamples = 1200;
	}

	if(device->devStateInfo.swTest == SWTST_SAMPLING_PERIOD_OUT_OF_RANGE) {
		/*Aux ADC sampling period should be at least 15us. Passing 10us to ARM should throw and ARM exception*/
		auxAdcConfig->samplingPeriod_us = 10;
	}

	if(device->devStateInfo.swTest == SWTST_AUXADC_CH_OUT_OF_RANGE) {
		/*Aux ADC channel should be in the range 0-3. Passing 8 to ARM should throw and ARM exception*/
		auxAdcConfig->auxAdcChannelSel = (br3109AuxAdcChannels_t)8;
	}

	/*Initialize auxAdcConfig Byte Array*/
	auxAdcConfigByteArr[0] = (uint8_t)auxAdcConfig->auxAdcChannelSel;
	auxAdcConfigByteArr[1] = (uint8_t)auxAdcConfig->auxAdcMode;
	auxAdcConfigByteArr[2] = (uint8_t)(auxAdcConfig->numSamples & 0x00FF);
	auxAdcConfigByteArr[3] = (uint8_t)((auxAdcConfig->numSamples >> 8) & 0x00FF);
	auxAdcConfigByteArr[4] = (uint8_t)(auxAdcConfig->samplingPeriod_us & 0x00FF);
	auxAdcConfigByteArr[5] = (uint8_t)((auxAdcConfig->samplingPeriod_us >> 8) &
					   0x00FF);
#if 0//单纯的采集不许要配置，具体情况需要场景需求提出支持
	/* Write AuxADC config to ARM mailbox*/
	retVal = (talRecoveryActions_t)BR3109_writeArmMem(device,
			BR3109_ADDR_ARM_START_DATA_ADDR, &auxAdcConfigByteArr[0],
			AUXADC_CONFIG_SIZE_BYTES);
	IF_ERR_RETURN_U32(retVal);

	/* execute SET Aux ADC config command */
	retVal = (talRecoveryActions_t)BR3109_sendArmCommand(device,
			BR3109_ARM_SET_OPCODE, &armAuxAdcCfgCmd[0], ARM_AUXADC_CFG_CMD_SIZE_BYTES);
	IF_ERR_RETURN_U32(retVal);

	/*Verify that command executed successfully*/
	retVal = (talRecoveryActions_t)BR3109_waitArmCmdStatus(device,
			BR3109_ARM_SET_OPCODE, &cmdStatusByte, SETARMGPIO_TIMEOUT_US,
			SETARMGPIO_INTERVAL_US);

	if((cmdStatusByte >> 1) > 0) {
		return  (uint32_t)talApiErrHandler(device, TAL_ERRHDL_ARM_CMD_ERR,
						   ARMCMD_ERRCODE(BR3109_ARM_SET_OPCODE, armAuxAdcCfgCmd[0], cmdStatusByte),
						   retVal, TALACT_ERR_RESET_ARM);
	} else {
		IF_ERR_RETURN_U32(retVal);
	}
#endif

	return (uint32_t)retVal;
}
#endif
uint32_t BR3109_readAuxAdc(br3109Device_t *device,  br3109AuxAdcResult_t *auxAdcResult)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t regdat = 0;
	
	/*Null check function parameters*/
	if (auxAdcResult == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_GPIO,
						  BR3109_ERR_READAUXADC_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*Initialize Aux ADC Result Command C-99 style*/
	/* Executing the GET Aux ADC result command */
	//reset adc
	retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_AUX_ADDA_IVREF_ID, 0x0C), 0, 0x00000001, 0);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_AUX_ADDA_IVREF_ID, 0x0C), 1, 0x00000001, 0);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_AUX_ADDA_IVREF_ID, 0x0C), 0, 0x00000001, 0);
	IF_ERR_RETURN_U32(retVal);

	/*Read Aux ADC conversion result*/
	retVal = BR3109_armSpiCmd_SPI_blk_read(device, SPI_AUX_ADDA_IVREF_ID, 0x007C, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	auxAdcResult->auxAdcCodeAvg = regdat;
	auxAdcResult->numSamples = 1;
	auxAdcResult->completeIndicator = 1;

	return (uint32_t)retVal;
}

const char *  talGetGpioErrorMessage (uint32_t errSrc, uint32_t errCode) 
{
#ifndef BR3109_VERBOSE
       return "";
#else

    if (errSrc == TAL_ERRSRC_TAL_API_GPIO)
    {
        switch (errCode)
         {
             case BR3109_ERR_GPIO_OK: return "\n";
             case BR3109_ERR_GPIO_OE_INV_PARM: return "Invalid value for gpioOutEn in BR3109_setGpioOe\n";
             case BR3109_ERR_GETGPIO_OE_NULL_PARM: return "Invalid NULL pointer for gpioOutEn in BR3109_getGpioOe\n";
             case BR3109_ERR_GPIO_SRC_INV_PARM: return "Invalid value for gpioSrcCtrl in BR3109_setGpioSourceCtrl\n";
             case BR3109_ERR_GETGPIO_SRC_NULL_PARM: return "Invalid NULL pointer for gpioSrcCtrl in BR3109_getGpioSourceCtrl\n";
             case BR3109_ERR_GPIO_LEVEL_INV_PARM: return "Invalid value for gpioPinLevel in BR3109_setGpioPinLevel\n";
             case BR3109_ERR_GETGPIO_LEVEL_NULL_PARM: return "Invalid NULL pointer for gpioPinLevel in BR3109_getGpioPinLevel\n";
             case BR3109_ERR_GETGPIO_SETLEVEL_NULL_PARM: return "Invalid NULL pointer for gpioPinSetLevel in BR3109_getGpioSetLevel\n";
             case BR3109_ERR_MONITOR_OUT_INDEX_RANGE: return "Invalid value for monitorIndex in BR3109_setGpioMonitorOut\n";
             case BR3109_ERR_GETGPIOMON_INDEX_NULL_PARM: return "Invalid NULL pointer for monitorIndex in BR3109_getGpioMonitorOut\n";
             case BR3109_ERR_GETGPIOMON_MONITORMASK_NULL_PARM: return "Invalid NULL pointer for monitorMask in BR3109_getGpioMonitorOut\n";
             case BR3109_ERR_SETUPAUXDAC_NULL_PARM: return "Invalid NULL pointer for auxDac in BR3109_setupAuxDacs\n";
             case BR3109_ERR_SETUPAUXDAC_INV_10BIT_AUXDACCODE: return "Invalid value for aux10BitDacValue in BR3109_setupAuxDacs\n";
             case BR3109_ERR_SETUPAUXDAC_INV_12BIT_AUXDACCODE: return "Invalid value for aux12BitDacValue in BR3109_setupAuxDacs\n";
             case BR3109_ERR_WRITEAUXDAC_INV_10BIT_AUXDACCODE: return "Invalid value for auxDacCode in BR3109_writeAuxDac\n";
             case BR3109_ERR_WRITEAUXDAC_INV_12BIT_AUXDACCODE: return "Invalid value for auxDacCode in BR3109_writeAuxDac\n";
             case BR3109_ERR_WRITEAUXDAC_INV_AUXDACINDEX: return "Invalid value for auxDacIndex in BR3109_writeAuxDac\n";
             case BR3109_ERR_SETUPAUXDAC_INV_RESOLUTION: return "Invalid AuxDac resolution for 10bit DACs - use ENUM\n";
             case BR3109_ERR_GPIO3V3_OE_INV_PARM: return "BR3109_setGpio3v3Oe(): Invalid value for gpio3v3OutEn\n";
             case BR3109_ERR_GETGPIO3V3_OE_NULL_PARM: return "BR3109_getGpio3v3Oe(): Invalid NULL pointer for gpio3v3OutEn\n";
             case BR3109_ERR_GPIO3V3_SRC_INV_PARM: return "BR3109_setGpio3v3SourceCtrl(): Invalid value for gpio3v3SrcCtrl\n";
             case BR3109_ERR_GETGPIO3V3_SRC_NULL_PARM: return "BR3109_getGpio3v3SourceCtrl(): Invalid NULL pointer for gpio3v3SrcCtrl\n";
             case BR3109_ERR_GPIO3V3_LEVEL_INV_PARM: return "BR3109_setGpio3v3PinLevel(): Invalid value for gpio3v3PinLevel\n";
             case BR3109_ERR_GETGPIO3V3_LEVEL_NULL_PARM: return "BR3109_getGpio3v3PinLevel(): Invalid NULL pointer for gpio3v3PinLevel\n";
             case BR3109_ERR_GETGPIO3V3_SETLEVEL_NULL_PARM: return "BR3109_getGpio3v3SetLevel(): Invalid NULL pointer for gpio3v3PinSetLevel\n";
             case BR3109_ERR_STARTAUXADC_INV_CHANNEL: return "BR3109_startAuxAdc(): Invalid Aux ADC channel selected. Valid Channels Ch0,Ch1,Ch2,Ch3";
             case BR3109_ERR_STARTAUXADC_INV_MODE: return "BR3109_startAuxAdc(): Invalid Aux ADC mode. Valid Modes - Pin Mode, Non-Pin Mode";
             case BR3109_ERR_STARTAUXADC_INV_NUM_SAMPLES: return "BR3109_startAuxAdc(): Aux ADC config number of samples out of range. Valid range 1-1000";
             case BR3109_ERR_STARTAUXADC_INV_SAMPLING_PERIOD : return "BR3109_startAuxAdc(): Aux ADC sampling time less than minimum specified sampling time. Minimum sampling time allowed is 15us";
             case BR3109_ERR_SETAUXADCPINMODEGPIO_INV_GPIO : return "BR3109_setAuxAdcPinModeGpio(): Aux ADC GPIO pin for pin mode ctrl invalid. Allowed GPIO0-GPIO15";
             case BR3109_ERR_SETAUXADCPINMODEGPIO_GPIO_IN_USE : return "BR3109_setAuxAdcPinModeGpio(): Aux ADC pin mode GPIO already in use by another feature";
             case BR3109_ERR_STARTAUXADC_NULL_PARAM : return "BR3109_startAuxAdc():  Null function parameter\n";
             case BR3109_ERR_READAUXADC_NULL_PARAM : return "BR3109_readAuxAdc():  Null function parameter\n";

             /* GP_INT Error Messages */
             case BR3109_ERR_GPINT_CLKPLL_UNLOCKED: return "BR3109_gpIntHandler(): CLK PLL is not locked\n";
             case BR3109_ERR_GPINT_RFPLL_UNLOCKED: return "BR3109_gpIntHandler(): RFCLK PLL is not locked\n";
             case BR3109_ERR_GPINT_AUXPLL_UNLOCKED: return "BR3109_gpIntHandler(): AUXCLK PLL is not locked\n";
             case BR3109_ERR_GPINT_ARM_WATCHDOG_TIMEOUT: return "BR3109_gpIntHandler(): ARM WatchDog Timeout\n";
             case BR3109_ERR_GPINT_ARM_FORCE_GPINT: return "BR3109_gpIntHandler(): ARM Forced a General Purpose Interrupt\n";
             case BR3109_ERR_GPINT_ARM_SYSTEM_ERROR: return "BR3109_gpIntHandler(): ARM reports a system error\n";
             case BR3109_ERR_GPINT_ARM_DATA_PARITY_ERROR: return "BR3109_gpIntHandler(): ARM reports an unrecoverable parity error in data memory\n";
             case BR3109_ERR_GPINT_ARM_PROG_PARITY_ERROR: return "BR3109_gpIntHandler(): ARM reports an unrecoverable parity error in program memory\n";
             case BR3109_ERR_GPINT_ARM_CALIBRATION_ERROR: return "BR3109_gpIntHandler(): ARM reports an Initialization Calibration Error\n";
             case BR3109_ERR_GPINT_FRAMERA: return "BR3109_gpIntHandler(): FramerA reports error\n";
             case BR3109_ERR_GPINT_DEFRAMERA: return "BR3109_gpIntHandler(): DeframerA reports error\n";
             case BR3109_ERR_GPINT_FRAMERB: return "BR3109_gpIntHandler(): FramerB reports error\n";
             case BR3109_ERR_GPINT_DEFRAMERB: return "BR3109_gpIntHandler(): DeframerB reports error\n";
             case BR3109_ERR_GPINT_STATUS_NULL_PARM: return "BR3109_gpIntHandler(): gpStatus variable pointer is NULL\n";
             case BR3109_ERR_GPINT_GPINTDIAG_NULL_PARM: return "BR3109_gpIntHandler(): gpIntDiag structure pointer is NULL\n";
             case BR3109_ERR_GPINT_PA_PROTECT_CH1: return "BR3109_gpIntHandler(): PA Protection Channel1 Reports Error\n";
             case BR3109_ERR_GPINT_PA_PROTECT_CH2: return "BR3109_gpIntHandler(): PA Protection Channel2 Reports Error\n";
             case BR3109_ERR_GPINT_STREAM_ERROR: return "BR3109_gpIntHandler(): Stream Processor Reports Error\n";
             default: return "Invalid GPIO error passed, not in error list\n";
         }
    }
    return "Unknown GPIO Error Source\n";

#endif
}


