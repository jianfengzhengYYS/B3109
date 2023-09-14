/**
 * \file br3109_radioctrl.c
 * \brief Contains functions to support Br3109 radio control and pin control
 *        functions
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#include "br3109_cals.h"
#include "br3109_radioctrl.h"
#include "br3109_gpio.h"
#include "br3109_reg_addr_macros.h"
#include "br3109_arm_macros.h"
#include "br3109_hal.h"
#include "br3109_user.h"
#include "br3109_error.h"
#include "br3109_arm.h"
#include "br3109.h"
#include "br3109_arm_spi_cmd.h"


#if 0
uint32_t  BR3109_loadStreamFromBinary (br3109Device_t *device, uint8_t *binary) 
{
	return 123;
} 
#endif

/*设置sel0和sel1使用gpio*/
uint32_t  BR3109_setArmGpioPins(br3109Device_t *device, br3109ArmGpioConfig_t *armGpio) 
{
	br3109GpioShortConfig_t armpin_config;

	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setArmGpioPins()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /*Check passed pointers for NULL */
	if (armGpio == NULL){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_SET_ARMGPIO_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Update all used GPIO's */	
	armpin_config.p0.gpioPinSel = armGpio->orx1TxSel0Pin.gpioPinSel;
	armpin_config.p0.enable = (armGpio->orx1TxSel0Pin.enable > 0) ? 1 : 0;
	armpin_config.p1.gpioPinSel = armGpio->orx1TxSel1Pin.gpioPinSel;
	armpin_config.p1.enable = (armGpio->orx1TxSel1Pin.enable > 0) ? 1 : 0;
	armpin_config.p2.gpioPinSel = armGpio->orx2TxSel0Pin.gpioPinSel;
	armpin_config.p2.enable = (armGpio->orx2TxSel0Pin.enable > 0) ? 1 : 0;
	armpin_config.p3.gpioPinSel = armGpio->orx2TxSel1Pin.gpioPinSel;
	armpin_config.p3.enable = (armGpio->orx2TxSel1Pin.enable > 0) ? 1 : 0;

	/* Set Br3109 GPIO pins used by ARM */
	retVal = BR3109_setOrxTxPinSel(device, &armpin_config);
	IF_ERR_RETURN_U32(retVal);

	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG1, (0 << 1), (1 << 1), 0);  //orxtx_sel enable 外部控制模式
	IF_ERR_RETURN_U32(retVal);		

	return (uint32_t)retVal;
}


uint32_t  BR3109_setTxRxEnablePinMode(br3109Device_t *device) 
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setTxRxEnablePins()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /*config tx/rx enable pin mode*/
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG0, (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(0 << 7)|(0 << 10)|(0 << 21)|(0 << 22)|(0 << 23)|(0 << 24), (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 7)|(1 << 10)|(1 << 21)|(1 << 22)|(1 << 23)|(1 << 24), 0);  //tx rx pin mode
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG1, (0 << 0), (1 << 0), 0);//enable 外部控制模式
	IF_ERR_RETURN_U32(retVal);	
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG2, (0 << 0)|(0 << 1)|(0 << 4)|(0 << 5)|(1 << 8)|(1 << 9)|(1 << 10)|(1 << 11), (1 << 0)|(1 << 1)|(1 << 4)|(1 << 5)|(1 << 8)|(1 << 9)|(1 << 10)|(1 << 11), 0);  //tx rx pin mode
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG3, (0 << 0)|(0 << 1)|(0 << 2)|(0 << 3)|(1 << 8)|(1 << 9)|(1 << 10)|(1 << 11), (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 8)|(1 << 9)|(1 << 10)|(1 << 11), 0);  //tx rx pin mode
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG4, (0 << 0)|(0 << 1)|(0 << 2)|(0 << 3), (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3), 0);  //tx rx pin mode
	IF_ERR_RETURN_U32(retVal);	
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG5, (0 << 0)|(0 << 4)|(1 << 8)|(1 << 9)|(1 << 10)|(1 << 11), (1 << 0)|(1 << 4)|(1 << 8)|(1 << 9)|(1 << 10)|(1 << 11), 0);  //tx rx pin mode
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}



uint32_t  BR3109_setRadioCtlPinMode (br3109Device_t *device, br3109ObsRxChannels_t orxChannel, br3109GpioShortPinSel_t *orxEnGpioPinSel) 
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	br3109GpioConfigOrxGpio_t config;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setRadioCtlPinMode()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	if(((orxChannel != TAL_ORX1) && (orxChannel != TAL_ORX2)) || orxEnGpioPinSel == NULL){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_RADIO_CTL_MASK_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	/* checking for valid bit mask settings */
	if (orxEnGpioPinSel->gpioPinSel > TAL_GPIO_INVALID) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_INV_RADIO_CTL_MASK_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	/* Disable ability for partial stream processor power down from Rx to
		* ORx or ORx to Rx  until stream processor is updated to support it
		*/
	retVal = BR3109_setGpioOe(device, (0x0 << orxEnGpioPinSel->gpioPinSel), (0x1 << orxEnGpioPinSel->gpioPinSel));
	IF_ERR_RETURN_U32(retVal);	
	retVal = BR3109_setGpioSourceCtrl(device, TAL_GPIO_TX_ATT_CONTROL_MODE_2 << ((orxEnGpioPinSel->gpioPinSel / 4)*4));
	IF_ERR_RETURN_U32(retVal);
	retVal =BR3109_getOrxEnableGpio(device, &config);
	IF_ERR_RETURN_U32(retVal);
	
	if(orxChannel == TAL_ORX1){
		config.orx1_enble_sel = orxEnGpioPinSel->enable > 0 ? 1 : 0;
		config.orx1_enable_gpio_sel.enable = orxEnGpioPinSel->enable > 0 ? 1 : 0;
		config.orx1_enable_gpio_sel.gpioPinSel = orxEnGpioPinSel->gpioPinSel;
		if(orxEnGpioPinSel->enable > 0){			
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG0, (0 << 25)|(1 << 20), (1 << 25)|(1 << 20), 0);//orx 外部控制模式
			IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG1, (0 << 0), (1 << 0), 0);//orx 外部控制模式
			IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG2, (0 << 0)|(1 << 8), (1 << 0)|(1 << 8), 0);//orx adc 外部控制模式
			IF_ERR_RETURN_U32(retVal);			
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG3, (0 << 4)|(1<< 12), (1 << 4)|(1<< 12), 0);//orx dp 外部控制模式
			IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG5, (0 << 1), (1 << 1), 0);//orx dc 外部控制模式
			IF_ERR_RETURN_U32(retVal);
		}
	}else if(orxChannel == TAL_ORX2){
		config.orx2_enble_sel = orxEnGpioPinSel->enable > 0 ? 1 : 0;
		config.orx2_enable_gpio_sel.enable = orxEnGpioPinSel->enable > 0 ? 1 : 0;
		config.orx2_enable_gpio_sel.gpioPinSel = orxEnGpioPinSel->gpioPinSel;
		if(orxEnGpioPinSel->enable > 0){
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG0, (0 << 26)|(1 << 20), (1 << 26)|(1 << 20), 0);//orx 外部控制模式
			IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG1, (0 << 0), (1 << 0), 0);//orx 外部控制模式
			IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG2, (0 << 1)|(1 << 9), (1 << 1)|(1 << 9), 0);//orx adc 外部控制模式
			IF_ERR_RETURN_U32(retVal);	
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG3, (0 << 5)|(1<< 13), (1 << 5)|(1<< 13), 0);//orx dp 外部控制模式
			IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG5, (0 << 5), (1 << 5), 0);//orx dp 外部控制模式
			IF_ERR_RETURN_U32(retVal);
		}
	}
	
	retVal = BR3109_setOrxEnableGpio(device, &config);
	IF_ERR_RETURN_U32(retVal);

	retVal = BR3109_setTxRxEnablePinMode(device);
	IF_ERR_RETURN_U32(retVal);
	
	return (uint32_t)retVal;
}


uint32_t  BR3109_getRadioCtlPinMode (br3109Device_t *device, br3109ObsRxChannels_t orxChannel, br3109GpioShortPinSel_t *orxEnGpioPinSel) 
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	br3109GpioConfigOrxGpio_t config;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getRadioCtlPinMode()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	/* checking for NULL pointers in function parameters */
	if(((orxChannel != TAL_ORX1) && (orxChannel != TAL_ORX2)) || orxEnGpioPinSel == NULL){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETPINMODE_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	retVal =BR3109_getOrxEnableGpio(device, &config);
	IF_ERR_RETURN_U32(retVal);
	if(orxChannel == TAL_ORX1){
		orxEnGpioPinSel->enable = config.orx1_enable_gpio_sel.enable;
		orxEnGpioPinSel->gpioPinSel = config.orx1_enable_gpio_sel.gpioPinSel;
	}else if(orxChannel == TAL_ORX2){
		orxEnGpioPinSel->enable = config.orx2_enable_gpio_sel.enable;
		orxEnGpioPinSel->gpioPinSel = config.orx2_enable_gpio_sel.gpioPinSel;
	}

	return (uint32_t)retVal;
}
#if 0
uint32_t BR3109_setOrxLoCfg(br3109Device_t *device,
			    const br3109OrxLoCfg_t *orxLoCfg)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t enableRelock = 0;
	uint8_t gpio3StreamMaskValue = 0;
	uint8_t gpio3StreamMaskReadValue = 0;
	uint8_t gpioSel = 0;
	uint8_t gpioSelReadVal = 0;
	uint8_t byteOffset = 0;
	uint32_t radioStatus = 0;

	uint32_t usedGpioPins = 0;
	uint32_t freeGpioPins = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setOrxLoCfg()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Perform input paramter NULL and range checks */
	if (orxLoCfg == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETORXLOCFG_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	if (orxLoCfg->gpioSelect == TAL_GPIO_INVALID) {
		gpio3StreamMaskValue = 1;
		gpioSel = 0;
	} else if ((orxLoCfg->gpioSelect >= TAL_GPIO_00)
		   && (orxLoCfg->gpioSelect <= TAL_GPIO_15)) {
		gpio3StreamMaskValue = 0;
		gpioSel = (uint8_t)orxLoCfg->gpioSelect;
	} else {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETORXLOCFG_INVALIDPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* If this feature already enabled for any GPIO pin, free the pin */
	halError = brSpiReadField(device->devHalInfo, BR3109_ADDR_STREAM_CONTROL,
				   &gpio3StreamMaskReadValue, 0x80, 7);
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = brSpiReadField(device->devHalInfo,
				   BR3109_ADDR_STREAM_GPIO_TRIGGER_PIN_SELECT_BYTE1, &gpioSelReadVal, 0xF0, 4);
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	if (gpio3StreamMaskReadValue == 1) {
		/* TAL_GPIO_INVALID */
		/* No GPIO pins to free */
	} else {
		freeGpioPins = (uint32_t)(1 << gpioSelReadVal);
	}

	/* Verify GPIO is not already used */
	if ((orxLoCfg->gpioSelect >= TAL_GPIO_00)
	    && (orxLoCfg->gpioSelect <= TAL_GPIO_15)) {
		usedGpioPins = (uint32_t)(1 << gpioSel);

		/* Check if pin selected is already being used by another feature */
		if ((device->devStateInfo.usedGpiopins & (usedGpioPins & ~freeGpioPins)) > 0) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
							  TAL_ERR_SETORXLOCFG_GPIOUSED, retVal, TALACT_ERR_CHECK_PARAM);
		}
	}

	/* Verify ARM in radio Off state */
	retVal = (talRecoveryActions_t)BR3109_getRadioState(device, &radioStatus);
	IF_ERR_RETURN_U32(retVal);

	if (((radioStatus & 0x07) != BR3109_ARM_RADIO_STATUS_IDLE) &&
	    ((radioStatus & 0x07) != BR3109_ARM_RADIO_STATUS_READY)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETORXLOCFG_INVALID_ARMSTATE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Disable/enable Auto Relocking AuxPLL in ARM */
	/* Invert value before writing to ARM memory */
	enableRelock = (orxLoCfg->disableAuxPllRelocking == 0) ? 1 : 0;
	byteOffset = 0x1C;
	retVal = (talRecoveryActions_t)BR3109_writeArmConfig(device,
			BR3109_ARM_OBJECTID_SYTEM_INFO,
			byteOffset, &enableRelock, 1);
	IF_ERR_RETURN_U32(retVal);

	/* Set Stream GPIO3 pin select and enable the GPIO pin to be able to create
	 * an interrupt to trigger the stream */
	halError = brSpiWriteField(device->devHalInfo, BR3109_ADDR_STREAM_CONTROL,
				    gpio3StreamMaskValue, 0x80, 7);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_STREAM_GPIO_TRIGGER_PIN_SELECT_BYTE1, gpioSel, 0xF0, 4);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Update used GPIO pins in device data structure */
	device->devStateInfo.usedGpiopins &= ~freeGpioPins;
	device->devStateInfo.usedGpiopins |= usedGpioPins;

	return (uint32_t)retVal;
}

uint32_t BR3109_getOrxLoCfg(br3109Device_t *device, br3109OrxLoCfg_t *orxLoCfg)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t radioStatus = 0;
	uint16_t byteOffset = 0;
	uint8_t enableRelock = 0;
	uint8_t gpio3StreamMaskValue = 0;
	uint8_t gpioSel = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getOrxLoCfg()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Perform input paramter NULL and range checks */
	if (orxLoCfg == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETORXLOCFG_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Verify ARM in radio Off state */
	retVal = (talRecoveryActions_t)BR3109_getRadioState(device, &radioStatus);
	IF_ERR_RETURN_U32(retVal);

	if (((radioStatus & 0x07) != BR3109_ARM_RADIO_STATUS_IDLE) &&
	    ((radioStatus & 0x07) != BR3109_ARM_RADIO_STATUS_READY)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETORXLOCFG_INVALID_ARMSTATE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Read back if ARM is auto relocking AuxPll */
	byteOffset = 0x1C;
	retVal = (talRecoveryActions_t)BR3109_readArmConfig(device,
			BR3109_ARM_OBJECTID_SYTEM_INFO, byteOffset, &enableRelock, 1);
	IF_ERR_RETURN_U32(retVal);

	/* Invert relock value */
	orxLoCfg->disableAuxPllRelocking = (enableRelock == 1) ? 0 : 1;

	/* Read back stream GPIO3 pin setup */
	halError = brSpiReadField(device->devHalInfo, BR3109_ADDR_STREAM_CONTROL,
				   &gpio3StreamMaskValue, 0x80, 7);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = brSpiReadField(device->devHalInfo,
				   BR3109_ADDR_STREAM_GPIO_TRIGGER_PIN_SELECT_BYTE1, &gpioSel, 0xF0, 4);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	if (gpio3StreamMaskValue == 1) {
		orxLoCfg->gpioSelect = TAL_GPIO_INVALID;
	} else {
		orxLoCfg->gpioSelect = (br3109GpioPinSel_t)gpioSel;
	}

	return (uint32_t)retVal;
}
#endif

uint32_t BR3109_radioOn(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;

	//select radio param to enable
	static const uint8_t rx1_rf_en = 1;     //0bit
	static const uint8_t rx2_rf_en = 1;     //1bit
	static const uint8_t tx1_rf_en = 1;     //2bit
	static const uint8_t tx2_rf_en = 1;     //3bit
	static const uint8_t orx1_rf_en = 1;    //4bit
	static const uint8_t orx2_rf_en = 1;    //5bit

	uint32_t enable_sel = 0;

	enable_sel = ((rx1_rf_en & 0x1) |
		       ((rx2_rf_en & 0x1) << 1) |
		       ((tx1_rf_en & 0x1) << 2) |
		       ((tx2_rf_en & 0x1) << 3) |
		      ((orx1_rf_en & 0x1) << 4) |
		      ((orx2_rf_en & 0x1) << 5));
	//retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG0, 0x0201330F, 0x0231773F, 0);
	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG0, enable_sel, 0x3F, 0);
	IF_ERR_RETURN_U32(retVal);
	BR3109_armSpiCmd_TddManualsel(device, 1);
	device->devStateInfo.devState = (br3109States_t)(device->devStateInfo.devState | TAL_STATE_RADIOON);
	return (uint32_t)retVal;
}

uint32_t BR3109_radioOff(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;

	retVal = BR3109_ArmWriteField(device, BR3109_ADDR_TDD_RF_CONFIG0, 0x00000000, 0x3F, 0);
	IF_ERR_RETURN_U32(retVal); 
	device->devStateInfo.devState = (br3109States_t)(device->devStateInfo.devState & ~TAL_STATE_RADIOON); 
	return (uint32_t)retVal;
}

uint32_t BR3109_getRadioState(br3109Device_t *device, uint32_t *radioStatus)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t status = 0;

	/* execute only if radioStatus pointer is valid */
	if (radioStatus != NULL) {
		retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_TDD_RF_CONFIG0, &status, 1);
		IF_ERR_RETURN_U32(retVal);

		*radioStatus = (uint32_t)status&0x00000FFF;
	} else {
		/* invalid radioStatus pointer */
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETRADIOSTATE_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	return (uint32_t)retVal;
}

static uint32_t __BR3109_TxBandSet(br3109Device_t *device, br3109TxChannels_t txChannel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	//uint32_t regdat = 0;
    if (device->devStateInfo.txBandwidth_Hz <= device->devStateInfo.txBandwidth_SW_Hz){
		retVal = BR3109_armSpiCmd_tx_bw_set(device,txChannel,0);    //set txchannel bw  0:窄带
		IF_ERR_RETURN_U32(retVal);
    }else{
		retVal = BR3109_armSpiCmd_tx_bw_set(device,txChannel,1);    //set txchannel bw  1:宽带
		IF_ERR_RETURN_U32(retVal);
    }
	return retVal;
}

static uint32_t __BR3109_RXBandSet(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	//uint32_t regdat = 0;
    if(device->devStateInfo.rxBandwidth_Hz <= device->devStateInfo.rxBandwidth_SW_Hz){
		retVal = BR3109_armSpiCmd_orxrx_bw_set(device, 0);   //0:窄带
		IF_ERR_RETURN_U32(retVal);
    }else{
		retVal = BR3109_armSpiCmd_orxrx_bw_set(device, 1);   //1:宽带
		IF_ERR_RETURN_U32(retVal);
	}
	return retVal;
}

static uint32_t __BR3109_ORXBandSet(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	//uint32_t regdat = 0;
    if(device->devStateInfo.orxBandwidth_Hz <= device->devStateInfo.orxBandwidth_SW_Hz){
		retVal = BR3109_armSpiCmd_orxrx_bw_set(device, 0);   //0:窄带
		IF_ERR_RETURN_U32(retVal);
    }else{
		retVal = BR3109_armSpiCmd_orxrx_bw_set(device, 1);   //1:宽带
		IF_ERR_RETURN_U32(retVal);
    }
	return retVal;
}

uint32_t BR3109_setRxTxEnable(br3109Device_t *device,
			      br3109RxORxChannels_t rxOrxChannel, br3109TxChannels_t txChannel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setRxTxEnable()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Only allow valid Rx / ORx cases, return error else */
	switch (rxOrxChannel) {
	case TAL_RXOFF_EN:  /* Fall through to next case */
	case TAL_RX1_EN:    /* Fall through to next case */
	case TAL_RX2_EN:    /* Fall through to next case */
	case TAL_RX1RX2_EN: /* Fall through to next case */
	case TAL_ORX1_EN:   /* Fall through to next case */
	case TAL_ORX2_EN:   /* Fall through to next case */
	case TAL_ORX1ORX2_EN:
		break;
	default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_BBIC_INV_CHN, retVal, TALACT_ERR_CHECK_PARAM);
	}

	switch (txChannel) {
	case TAL_TXOFF: /* Fall through to next case */
	case TAL_TX1:   /* Fall through to next case */
	case TAL_TX2:   /* Fall through to next case */
	case TAL_TX1TX2:
		break;
	default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_BBIC_INV_CHN, retVal, TALACT_ERR_CHECK_PARAM);
	}

//	chEnable = (uint8_t)((txChannel) | rxOrxChannel);
	if(txChannel & TAL_TX1TX2){
		retVal = __BR3109_TxBandSet(device, txChannel);
		IF_ERR_RETURN_U32(retVal);
	}
	retVal = BR3109_armSpiCmd_tx_ch_en(device, txChannel);      //TX通道使能
	IF_ERR_RETURN_U32(retVal);
	
	if(rxOrxChannel & TAL_RX1RX2_EN){
		retVal = __BR3109_RXBandSet(device);
		IF_ERR_RETURN_U32(retVal);
	}

	
	if(rxOrxChannel & TAL_ORX1ORX2_EN){
		retVal = __BR3109_ORXBandSet(device);
		IF_ERR_RETURN_U32(retVal);
	}
	retVal = BR3109_armSpiCmd_orxrx_ch_en(device, rxOrxChannel);       //ORX通道使能
	IF_ERR_RETURN_U32(retVal);

//	/* Check that channels were initialized */
//	if ((chEnable & device->devStateInfo.initializedChannels) != chEnable) {
//		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
//						  TAL_ERR_SETRXTXENABLE_INVCHANNEL, retVal, TALACT_ERR_CHECK_PARAM);
//	}

	return (uint32_t)retVal;
}

uint32_t BR3109_getRxTxEnable(br3109Device_t *device,
			      br3109RxORxChannels_t *rxOrxChannel, br3109TxChannels_t *txChannel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t chEnable = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getRxTxEnable()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if ((txChannel == NULL) ||
	    (rxOrxChannel == NULL)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETRXTXENABLE_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_TDD_RF_CONFIG0, &chEnable, 1);
	IF_ERR_RETURN_U32(retVal);

	*txChannel = (br3109TxChannels_t)((chEnable>>2) & 0x3);
	*rxOrxChannel = (br3109RxORxChannels_t)(chEnable & 0x33);

	return (uint32_t)retVal;
}

uint32_t BR3109_setTxToOrxMapping(br3109Device_t *device, uint8_t txCalEnable,
				  br3109TxToOrxMapping_t oRx1Map, br3109TxToOrxMapping_t oRx2Map)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t bw = device->devStateInfo.txBandwidth_Hz <= device->devStateInfo.txBandwidth_SW_Hz ? 0 : 1;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setTxToOrxMapping()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	retValWarn = retVal;

	/* cmdByte assignment from oRx1Map */
	switch (oRx1Map) {
	case TAL_MAP_NONE:
		break;
	case TAL_MAP_TX1_ORX:
		break;
	case TAL_MAP_TX2_ORX:
		break;
	default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETTXTOORXMAP_INV_ORX1_MAP, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* cmdByte assignment from oRx2Map */
	switch (oRx2Map) {
	case TAL_MAP_NONE:
		break;
	case TAL_MAP_TX1_ORX:
		break;
	case TAL_MAP_TX2_ORX:
		break;
	default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETTXTOORXMAP_INV_ORX2_MAP, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* adding the txCalEnable bit */

	/* write new setting to the command register */
	if(oRx1Map){
		retVal = BR3109_armSpiCmd_setup_tx_loopback(device, oRx1Map, CHANNEL_1, bw, 1, 1);//外部回环。loopback2 orx
		IF_ERR_RETURN_U32(retVal);
		/* adding the txCalEnable bit */
		if (txCalEnable > 0) {
			retVal = BR3109_armSpiCmd_tx_lo_leak_cali(device, oRx1Map, bw);
			IF_ERR_RETURN_U32(retVal);
		}
	}else if(oRx2Map){
		retVal = BR3109_armSpiCmd_setup_tx_loopback(device, oRx2Map, CHANNEL_2, bw, 1, 1);//外部回环。loopback2 orx
		IF_ERR_RETURN_U32(retVal);
		/* adding the txCalEnable bit */
		if (txCalEnable > 0) {
			retVal = BR3109_armSpiCmd_tx_lo_leak_cali(device, oRx2Map, bw);
			IF_ERR_RETURN_U32(retVal);
		}
	}

	/* If higher priority retVal has no error, allow possible lower priority warning to be returned */
	if (retVal == TALACT_NO_ACTION) {
		retVal = retValWarn;
	}

	return (uint32_t)retVal;
}

uint32_t  BR3109_setRfPllFrequency (br3109Device_t *device, br3109RfPllName_t pllName, uint64_t rfPllLoFrequency_Hz) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
	    /* selecting the PLL of interest */
    switch (pllName)
    {
        case TAL_RF_PLL:
            retVal = BR3109_armSpiCmd_setrf_freq(device, (uint32_t)DIV_U64(rfPllLoFrequency_Hz,1000));
        	IF_ERR_RETURN_U32(retVal);
            break;
        case TAL_ORF_PLL:
            retVal = BR3109_armSpiCmd_setorf_freq(device, (uint32_t)DIV_U64(rfPllLoFrequency_Hz,1000));
        	IF_ERR_RETURN_U32(retVal);
            break;
        default:
            return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                    TAL_ERR_SETRFPLL_INV_PLLNAME, retVal, TALACT_ERR_CHECK_PARAM);
    }
    return (uint32_t)retVal;
}

static int intpow(int base, int exponent) 
{
    int result = 1.0;
    int i;

    if (exponent > 0) {
        for (i = 0; i < exponent; i++) {
            result *= base;
        }
    } else if (exponent < 0) {
        for (i = 0; i > exponent; i--) {
            result /= base;
        }
    }

    return result;
}
static uint32_t __GetDiv(uint32_t div_reg)
{
	const uint32_t div_val[]={0x80,0x40,0x60,0x70,0x78,0x7C,0x7E,0x7F};
	uint32_t div = 1;
	uint8_t i = 0, j = 0;
	for(i =0; i<sizeof(div_val)/sizeof(div_val[0]); i++)
	{
		if((div_reg&0xff) == div_val[i])
			break;
	}
	div = intpow(2,i);
	return div;
}
uint32_t  BR3109_getRfPllFrequency (br3109Device_t *device, br3109RfPllName_t pllName, uint64_t *rfPllLoFrequency_Hz) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t reg_data = 0;
	uint32_t refclk = 0;
	uint16_t ref_mul = 1;
	uint32_t fcw = 0;
	uint32_t div = 0;
#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getRfPllFrequency()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	    /* selecting the PLL of interest */
    switch (pllName)
    {
        case TAL_RF_PLL:
			retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_APB_CLK_RST_RF_FCW, &fcw, 1);
       		IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_armSpiCmd_SPI_blk_read(device,SPI_RFPLL_ID, 0x3C, &reg_data, 1);//div
       		IF_ERR_RETURN_U32(retVal);
			div = __GetDiv(reg_data);
			retVal = BR3109_armSpiCmd_SPI_blk_read(device,SPI_RFPLL_ID, 0x34, &reg_data, 1);//ref_div
       		IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_armSpiCmd_SPI_blk_read(device, SPI_LO_TXALLWORK_REFPATH_ID, 0x5C, &reg_data, 1);//div
       		IF_ERR_RETURN_U32(retVal);
			refclk = device->devStateInfo.clocks.deviceClock_kHz * 1000 /intpow(2,(reg_data>>3)&0x1);
			if((reg_data & 0x3E00C) == 0x800C){
				ref_mul = 4;
			}else if((reg_data & 0x3E00C) == 0x08){
				ref_mul = 2;
			}else{
				ref_mul = 1;
			}
			// *rfPllLoFrequency_Hz = (uint64_t)fcw/intpow(2,23)/div*refclk/ref_mul;
			*rfPllLoFrequency_Hz = DIV_U64((uint64_t)fcw, (intpow(2,23)*div/refclk*ref_mul));
            break;
        case TAL_ORF_PLL:
			retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_APB_CLK_RST_ORF_FCW, &fcw, 1);
       		IF_ERR_RETURN_U32(retVal);
			retVal = BR3109_armSpiCmd_SPI_blk_read(device,SPI_ORFPLL_ID, 0x3C, &reg_data, 1);//div
       		IF_ERR_RETURN_U32(retVal);
			div = __GetDiv(reg_data);
			retVal = BR3109_armSpiCmd_SPI_blk_read(device,SPI_ORFPLL_ID, 0x34, &reg_data, 1);//ref_div
       		IF_ERR_RETURN_U32(retVal);
			refclk = device->devStateInfo.clocks.deviceClock_kHz * 1000 /intpow(2,(reg_data>>3)&0x1);
			retVal = BR3109_armSpiCmd_SPI_blk_read(device, SPI_LO_TXALLWORK_REFPATH_ID, 0x5C, &reg_data, 1);//div
       		IF_ERR_RETURN_U32(retVal);
			if((reg_data & 0xf8000C0) == 0x20000C0){
				ref_mul = 4;
			}else if((reg_data & 0xf8000C0) == 0x80){
				ref_mul = 2;
			}else{
				ref_mul = 1;
			}
			// *rfPllLoFrequency_Hz = (uint64_t)fcw/intpow(2,23)/div*refclk/ref_mul;
			*rfPllLoFrequency_Hz = DIV_U64((uint64_t)fcw, (intpow(2,23)*div/refclk*ref_mul));
            break;
        default:
            return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                    TAL_ERR_SETRFPLL_INV_PLLNAME, retVal, TALACT_ERR_CHECK_PARAM);
    }

    return (uint32_t)retVal;
}


uint32_t  BR3109_getPllsLockStatus (br3109Device_t *device, uint8_t *pllLockStatus) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
    uint32_t readData = 0;
	uint32_t rbuf[5];

#if BR3109_VERBOSE
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getPllsLockStatus()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    /* null pointer check */
    if (pllLockStatus == NULL)
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                TAL_ERR_CHECKPLLLOCK_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
    }
    else
    {
        /* clear the pointer contents */
        *pllLockStatus = 0;
		retVal = BR3109_armMemoryCmd_blk_read(device, BR3109_ADDR_APB_CLK_RST_BB_PLL, rbuf, 5);
        IF_ERR_RETURN_U32(retVal);
		//*pllLockStatus = ((rbuf[0]>>28)&0x1)|((rbuf[1]>>8)&0x2)|((rbuf[3]>>7)&0x4);
		readData = (rbuf[0]>>28)&0x1;
        *pllLockStatus = readData;
		readData = (rbuf[1]>>9)&0x1;
        *pllLockStatus = *pllLockStatus | (readData << 1);
		readData = (rbuf[3]>>9)&0x1;
        *pllLockStatus = *pllLockStatus | (readData << 2);
    }

    return (uint32_t)retVal;
}
#if 0
uint32_t  BR3109_setRfPllLoopFilter (br3109Device_t *device, uint16_t loopBandwidth_kHz, uint8_t stability) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;

#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setRfPllLoopFilter()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    retValWarn = retVal;

    retVal = (talRecoveryActions_t)BR3109_setPllLoopFilter(device, TAL_RF_PLL, loopBandwidth_kHz, stability);
    IF_ERR_RETURN_U32(retVal);

    /* If higher priority retVal has no error, allow possible lower priority warning to be returned */
    if (retVal == TALACT_NO_ACTION)
    {
        retVal = retValWarn;
    }

    return (uint32_t)retVal;
}


uint32_t  BR3109_getRfPllLoopFilter (br3109Device_t *device, uint16_t *loopBandwidth_kHz, uint8_t *stability) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;


#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
    halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getRfPllLoopFilter()\n");
    retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

    retValWarn = retVal;

    retVal = (talRecoveryActions_t)BR3109_getPllLoopFilter(device, TAL_RF_PLL, loopBandwidth_kHz, stability);
    IF_ERR_RETURN_U32(retVal);

    if (retVal == TALACT_NO_ACTION)
    {
        retVal = retValWarn;
    }

    return (uint32_t)retVal;
}


uint32_t  BR3109_setPllLoopFilter (br3109Device_t *device, br3109RfPllName_t pllName, uint16_t loopBandwidth_kHz, uint8_t stability) 
{
    talRecoveryActions_t retVal = TALACT_NO_ACTION;
    talRecoveryActions_t retValWarn = TALACT_NO_ACTION;
	uint16_t band = 0;
#if BR3109_VERBOSE
    brHalErr_t halError = BRHAL_OK;
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setPllLoopFilter()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
    /* Range check for loopBandwidth - Should be between 50kHz and 500000kHz */
    if ((loopBandwidth_kHz < 50) ||
        (loopBandwidth_kHz > 500000))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                TAL_ERR_SETRFPLL_LOOPFILTER_INV_LOOPBANDWIDTH, retVal, TALACT_ERR_CHECK_PARAM);
    }
		    /* Range check for stability - Should be between 3 and 15 */
    if ((stability < 3) ||
        (stability > 15))
    {
        return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                TAL_ERR_SETRFPLL_LOOPFILTER_INV_STABILITY, retVal, TALACT_ERR_CHECK_PARAM);
    }
	if(loopBandwidth_kHz>125000)
		band = 1;
	
	    /* selecting the PLL of interest */
    switch (pllName)
    {
        case TAL_RF_PLL:
			//////////////////////////////////////////////////////////////TODO
            break;
        case TAL_ORF_PLL:
			//////////////////////////////////////////////////////////////TODO
            break;
        default:
            return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
                    TAL_ERR_SETRFPLL_LOOPFILTER_INV_PLLSEL, retVal, TALACT_ERR_CHECK_PARAM);
    }
    /* If higher priority retVal has no error, allow possible lower priority warning to be returned */
    if (retVal == TALACT_NO_ACTION)
    {
        retVal = retValWarn;
    }

    return (uint32_t)retVal;
}


uint32_t  BR3109_getPllLoopFilter (br3109Device_t *device, br3109RfPllName_t pllName, uint16_t *loopBandwidth_kHz, uint8_t *stability) 
{
	return 123;
}
#endif

uint32_t  BR3109_setOrxLoSource (br3109Device_t *device, br3109ObsRxLoSource_t orxLoSource) 
{
	
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setOrxLoSource()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if ((orxLoSource != TAL_OBSLO_RF_PLL) && (orxLoSource != TAL_OBSLO_AUX_PLL)){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETORXLOSRC_INVALIDPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	//retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0), (orxLoSource == TAL_OBSLO_AUX_PLL) ? 0:1,
			//0x80000000, 31);
	//IF_ERR_RETURN_U32(retVal);

    if(orxLoSource == TAL_OBSLO_AUX_PLL)
   	{
   	    //设置ORX1的本振为ORFPLL
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000010), 0x00091000, 0x001FF000UL, 0);
		IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000020), 0x02000B00, 0x02000F80UL, 0);
		IF_ERR_RETURN_U32(retVal);

		//设置ORX2的本振为ORFPLL
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000018), 0x00091000, 0x001FF000UL, 0);
		IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000020), 0x00016000, 0x0001F000UL, 0);
		IF_ERR_RETURN_U32(retVal);

		//设置源为ORFPLL
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000000), 0x00000000, 0x80000000UL, 0);
		IF_ERR_RETURN_U32(retVal);
    }
	else if(orxLoSource == TAL_OBSLO_RF_PLL)
	{
   	    //设置ORX1的本振为RFPLL
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000010), 0x00119000, 0x001FF000UL, 0);
		IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000020), 0x00000A00, 0x02000F80UL, 0);
		IF_ERR_RETURN_U32(retVal);

		//设置ORX2的本振为RFPLL
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000018), 0x00119000, 0x001FF000UL, 0);
		IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000020), 0x00014000, 0x0001F000UL, 0);
		IF_ERR_RETURN_U32(retVal);

		//设置源为RFPLL
		retVal = BR3109_ArmWriteField(device, BR3109_ARMSPI_ADDR(SPI_LO_TXALLWORK_REFPATH_ID, 0x00000000), 0x80000000, 0x80000000UL, 0);
		IF_ERR_RETURN_U32(retVal);
    }

	return (uint32_t)retVal;

}


uint32_t  BR3109_getOrxLoSource (br3109Device_t *device, br3109ObsRxLoSource_t *orx1LoSource, br3109ObsRxLoSource_t *orx2LoSource) 
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regdat = 0;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getOrxLoSource()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Check for NULL */
	if ((orx1LoSource == NULL) ||
	    (orx2LoSource == NULL)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETORXLOSRC_NULLPARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	retVal = BR3109_armSpiCmd_SPI_blk_read(device, SPI_LO_TXALLWORK_REFPATH_ID, 0, &regdat, 1);
	IF_ERR_RETURN_U32(retVal);

	*orx1LoSource = (br3109ObsRxLoSource_t)((regdat >> 31) & 0x01);
	*orx2LoSource = (br3109ObsRxLoSource_t)((regdat >> 31) & 0x01);

	return (uint32_t)retVal;
}

#if 0
uint32_t  BR3109_setFhmConfig (br3109Device_t *device, br3109FhmConfig_t *fhmConfig) 
{
	return 123;
}


uint32_t  BR3109_getFhmConfig (br3109Device_t *device, br3109FhmConfig_t *fhmConfig) 
{
	return 123;
}


uint32_t  BR3109_setFhmMode (br3109Device_t *device, br3109FhmMode_t *fhmMode) 
{
	return 123;
}


uint32_t  BR3109_getFhmMode (br3109Device_t *device, br3109FhmMode_t *fhmMode) 
{
	return 123;
}


uint32_t  BR3109_setFhmHop (br3109Device_t *device, uint64_t nextRfPllFrequency_Hz) 
{
	return 123;
}


uint32_t  BR3109_getFhmRfPllFrequency (br3109Device_t *device, uint64_t *fhmRfPllFrequency_Hz) 
{
	return 123;
}

uint32_t  BR3109_getFhmStatus (br3109Device_t *device, br3109FhmStatus_t *fhmStatus) 
{
	return 123;
}
uint32_t  BR3109_setExtLoOutCfg (br3109Device_t *device, uint8_t enableExtLoOutput, br3109ExtLoDiv_t extLoOutDivide) 
{
	return 123;
}


uint32_t  BR3109_getExtLoOutCfg (br3109Device_t *device, uint8_t *enableExtLoOutput, br3109ExtLoDiv_t *extLoOutDivide) 
{
	return 123;
}

#endif

