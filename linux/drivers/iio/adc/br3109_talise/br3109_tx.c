/**
 * \file br3109_tx.c
 * \brief Contains functions to support Br3109 Tx data path control
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#include "br3109_tx.h"
#include "br3109_reg_addr_macros.h"
#include "br3109_arm_macros.h"
#include "br3109_hal.h"
#include "br3109_user.h"
#include "br3109_error.h"
#include "br3109.h"
#include "br3109_gpio.h"
#include "br3109_arm.h"

static uint32_t __num2bit(uint8_t dnum)
{
    uint32_t nbit = 0;
    if (dnum <= 0)
        return 0;
    while(dnum > 0){
        dnum -= 1;
        nbit |= 1<<dnum;
    }
    return nbit;
}
static uint8_t __bit2num(uint32_t nbit)
{
    uint8_t dnum = 0;
    if (nbit <= 0)
        return 0;
    while(nbit > 0){
		if(nbit&0x1)
        	dnum += 1;
		nbit = nbit >> 1;
    }
    return dnum;
}
/**
txAttenuation_index: 取值范围（0-250），功率关系为20*log10((257-orxAttenuation_index)/257)	
*/
uint32_t BR3109_setTxAttenuation(br3109Device_t *device, br3109TxChannels_t txChannel, uint16_t txAttenuation_mdB)
{
//spi transmitter 0x58
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regData = 0;
	uint32_t dig_setv = 0;
	uint8_t s_sel = 0;
	int16_t anglogpowermdb =0;
	// float txAttenuation_dB = (float)txAttenuation_mdB/1000.0;
#if 1
	// uint32_t rpowerda = 257 - pow(10,-txAttenuation_dB/20)*257;//db = 20lg10((257-pset)/257)
	uint32_t rpowerda = GetIndexFromAtt(txAttenuation_mdB);//db = 20lg10((257-pset)/257)
	uint32_t powerda = rpowerda;
#else
	uint32_t powerda = txAttenuation_index;
#endif
	uint16_t atten_x16 = 0;
	uint8_t atten_x8 = 0;
	uint8_t atten_x4 = 0;

	static const uint32_t txMax = 41951;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setTxAttenuation()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* Check for valid txChannel */
	if ((txChannel != TAL_TX1) && (txChannel != TAL_TX2)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_SETTXATTEN_INV_TXCHANNEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* check input parameter is in valid range */
	if (txAttenuation_mdB >= txMax) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_SETTXATTEN_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}	
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0x8, &regData, 1);
	IF_ERR_RETURN_U32(retVal);
	s_sel = (regData >> 2) & 0x1;
	atten_x16 = __num2bit(powerda / 16) & 0x7FFF;
	if(powerda > 240){//15*16 = 240
		powerda -=  240;
	}else{
		powerda -= (powerda / 16) * 16;
	}
	powerda -= (powerda / 16) * 16;
	atten_x8 = (powerda > 8) ? 1 : 0;
	powerda -= atten_x8 * 8;
	atten_x4 = (powerda > 4) ? 1 : 0;
	powerda -= atten_x4 * 4;	
	regData = atten_x16 | (atten_x8 << 15) | (atten_x4 << 16) | (__num2bit(powerda) << 17);
	if (txChannel == TAL_TX1) {
		retVal = BR3109_armSpiCmd_SPI_blk_write(device, SPI_TRANSMITTER_L0_ID, BR3109_ADDR_TX1_ATTENUATION, &regData, 1);
		IF_ERR_RETURN_U32(retVal);
		if(device->devStateInfo.tx1_s_gain == 0){
			if(s_sel){
				retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0x10, &regData, 1);
				IF_ERR_RETURN_U32(retVal);				
			}else{
				retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0xC, &regData, 1);
				IF_ERR_RETURN_U32(retVal);					
			}
			device->devStateInfo.tx1_s_gain = regData & 0x0000FFFF;
		}
		// anglogpowermdb = (-txAttenuation_dB - 20 * log10((257.0-rpowerda)/257.0) + 20 * log10(device->devStateInfo.tx1_s_gain/4096.0));
		// dig_setv = pow(10,anglogpowermdb/1000.0/20)*0x1000;
		anglogpowermdb = GetAttFromIndex(rpowerda);
		// dig_setv = myintpowbase10((txAttenuation_mdB - anglogpowermdb))
		// if(s_sel){
		// 	retVal = BR3109_armMemoryCmd_blk_write(device, APB_TX_DGAIN_BASEADDR + 0x10, &dig_setv, 1);
		// 	IF_ERR_RETURN_U32(retVal);				
		// }else{
		// 	retVal = BR3109_armMemoryCmd_blk_write(device, APB_TX_DGAIN_BASEADDR + 0xC, &dig_setv, 1);
		// 	IF_ERR_RETURN_U32(retVal);					
		// }
	}
	if (txChannel == TAL_TX2) {
		retVal = BR3109_armSpiCmd_SPI_blk_write(device, SPI_TRANSMITTER_L1_ID, BR3109_ADDR_TX2_ATTENUATION, &regData, 1);
		IF_ERR_RETURN_U32(retVal);
		if(device->devStateInfo.tx2_s_gain == 0){
			if(s_sel){
				retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0x18, &regData, 1);
				IF_ERR_RETURN_U32(retVal);				
			}else{
				retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0x14, &regData, 1);
				IF_ERR_RETURN_U32(retVal);					
			}
			device->devStateInfo.tx2_s_gain = regData & 0x0000FFFF;
		}
		// anglogpowermdb = (-txAttenuation_dB - 20 * log10((257.0-rpowerda)/257.0) + 20 * log10(device->devStateInfo.tx1_s_gain/4096.0));
		// dig_setv = pow(10,(anglogpowermdb/1000.0))/20)*0x1000;
		// if(s_sel){
		// 	retVal = BR3109_armMemoryCmd_blk_write(device, APB_TX_DGAIN_BASEADDR + 0x18, &dig_setv, 1);
		// 	IF_ERR_RETURN_U32(retVal);				
		// }else{
		// 	retVal = BR3109_armMemoryCmd_blk_write(device, APB_TX_DGAIN_BASEADDR + 0x14, &dig_setv, 1);
		// 	IF_ERR_RETURN_U32(retVal);					
		// }
	}
		
	return (uint32_t)retVal;
}

uint32_t BR3109_getTxAttenuation(br3109Device_t *device, br3109TxChannels_t txChannel, uint16_t *txAttenuation_mdB)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint32_t regData = 0;
	uint32_t powerda = 0;
	uint16_t atten_x16 = 0;
	uint8_t atten_x8 = 0;
	uint8_t atten_x4 = 0;
	uint8_t s_sel = 0;
	uint32_t dig_v = 0;
	// float dig_db = 0;
#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getTxAttenuation()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif
	/* check txAttenuation_mdB for null pointer */
	if (txAttenuation_mdB == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETTXATTEN_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0x8, &regData, 1);
	IF_ERR_RETURN_U32(retVal);
	s_sel = (regData >> 2) & 0x1;
	/* Check for a valid txChannel */
	if ((txChannel != TAL_TX1) && (txChannel != TAL_TX2)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETTXATTEN_INV_TXCHANNEL, retVal, TALACT_ERR_CHECK_PARAM);
	} else {
		if (txChannel == TAL_TX1) {
			if(s_sel){
				retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0x10, &regData, 1);
				IF_ERR_RETURN_U32(retVal);				
			}else{
				retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0xC, &regData, 1);
				IF_ERR_RETURN_U32(retVal);					
			}
			if(device->devStateInfo.tx1_s_gain == 0){
				device->devStateInfo.tx1_s_gain = regData & 0x0000FFFF;
			}
			// dig_v = regData & 0x0000FFFF;
			// dig_db = 20 * log10(1.0 * dig_v / device->devStateInfo.tx1_s_gain);
			retVal = BR3109_armSpiCmd_SPI_blk_read(device, SPI_TRANSMITTER_L0_ID, BR3109_ADDR_TX1_ATTENUATION, &regData, 1);
			IF_ERR_RETURN_U32(retVal);
		}

		if (txChannel == TAL_TX2) {
			if(s_sel){
				retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0x18, &regData, 1);
				IF_ERR_RETURN_U32(retVal);				
			}else{
				retVal = BR3109_armMemoryCmd_blk_read(device, APB_TX_DGAIN_BASEADDR + 0x14, &regData, 1);
				IF_ERR_RETURN_U32(retVal);					
			}
			if(device->devStateInfo.tx2_s_gain == 0){
				device->devStateInfo.tx2_s_gain = regData & 0x0000FFFF;
			}
			// dig_v = regData & 0x0000FFFF;
			// dig_db = 20 * log10(1.0 *  dig_v / device->devStateInfo.tx2_s_gain);
			retVal = BR3109_armSpiCmd_SPI_blk_read(device, SPI_TRANSMITTER_L1_ID, BR3109_ADDR_TX2_ATTENUATION, &regData, 1);
			IF_ERR_RETURN_U32(retVal);
		}
	}
	atten_x16  = regData & 0x7fff;
	atten_x8 = (regData >> 15) & 0x1;
	atten_x4 = (regData >> 16) & 0x1;
	powerda = __bit2num((regData >> 17) & 0x1F) + atten_x4 * 4 + atten_x8 * 8 + __bit2num(atten_x16) * 16;	
#if 1
	// *txAttenuation_mdB = -(20 * log10((257.0-powerda)/257.0) + dig_db)*1000;
	*txAttenuation_mdB = GetAttFromIndex(257-powerda);
#else
	*txAttenuation_index = -powerda;
#endif
	return (uint32_t)retVal;
}
#if 0
uint32_t BR3109_setDacFullScale(br3109Device_t *device,
				br3109DacFullScale_t dacFullScale)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t dacFsBoost = 0;
	uint8_t isArmRunning = 0;
	static const uint8_t TXDAC_GAIN_MASK = 0x1F;

	if ((dacFullScale != TAL_DACFS_0DB) &&
	    (dacFullScale != TAL_DACFS_3DB)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_INV_DAC_FULLSCALE_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	halError = brSpiReadField(device->devHalInfo, BR3109_ADDR_ARM_CTL_1,
				   &isArmRunning, 0x01, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	if (isArmRunning == 1) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_API_FAIL,
						  TAL_ERR_DAC_FULLSCALE_INVARMSTATE, retVal, TALACT_ERR_RESET_FULL);
	}

	dacFsBoost = ((uint8_t)dacFullScale) & TXDAC_GAIN_MASK;

	/* Tx1 I data */
	halError = talSpiWriteByte(device->devHalInfo, BR3109_ADDR_TXDAC1_GAIN_I,
				   dacFsBoost );
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Tx1 Q data */
	halError = talSpiWriteByte(device->devHalInfo, BR3109_ADDR_TXDAC1_GAIN_Q,
				   dacFsBoost );
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Tx2 I data */
	halError = talSpiWriteByte(device->devHalInfo, BR3109_ADDR_TXDAC2_GAIN_I,
				   dacFsBoost );
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Tx2 Q data */
	halError = talSpiWriteByte(device->devHalInfo, BR3109_ADDR_TXDAC2_GAIN_Q,
				   dacFsBoost );
	retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}
#endif

uint32_t BR3109_enableTxNco(br3109Device_t *device, br3109TxNcoTestToneCfg_t *txNcoTestToneCfg)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	int32_t txInputRateDiv2_kHz = 0;
	int32_t tx1Ncopower = 0;
	int32_t tx2Ncopower = 0;
	uint32_t tx_ch = 0x3;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_enableTxNco()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,  TALACT_WARN_RESET_LOG);
#endif

	/* Check valid Tx profile */
	if ((device->devStateInfo.profilesValid & TX_PROFILE_VALID) == 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ENABLETXNCO_INV_PROFILE, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check for NULL pointer */
	if ( txNcoTestToneCfg == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ENABLETXNCO_NULL_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	
	if (txNcoTestToneCfg->enable > 0) {
		txInputRateDiv2_kHz = device->devStateInfo.txInputRate_kHz >> 1;
		/* Check Tx1 NCO freq range between Fs/2 and -Fs/2 */
		if ((txNcoTestToneCfg->Tone1Freq_kHz > (int32_t)txInputRateDiv2_kHz) || (txNcoTestToneCfg->Tone2Freq_kHz < -((int32_t)txInputRateDiv2_kHz))) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ENABLETXNCO_INV_TX1_FREQ, retVal, TALACT_ERR_CHECK_PARAM);
		}

		/* Check Tx2 NCO freq range between Fs/2 and -Fs/2 */
		if ((txNcoTestToneCfg->Tone1Freq_kHz > (int32_t)txInputRateDiv2_kHz) || (txNcoTestToneCfg->Tone2Freq_kHz < -((int32_t)txInputRateDiv2_kHz))) {
			return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_ENABLETXNCO_INV_TX2_FREQ, retVal, TALACT_ERR_CHECK_PARAM);
		}
		if(txNcoTestToneCfg->enable & 0x1){
			/* Tx1 */
			// tx1Ncopower = (uint32_t)(pow(10,((float)txNcoTestToneCfg->Tx1Power/20))*0x8000);
			// tx1Ncopower = (uint32_t)(myintpowbase10( txNcoTestToneCfg->Tx1Power/20, txNcoTestToneCfg->Tx1Power%20)*0x8000/10000);
			if(tx1Ncopower >= 0x8000){
			    tx1Ncopower = 0x7FFF;
			}
		}
		if(txNcoTestToneCfg->enable & 0x2){
			/* Tx2 */
			// tx2Ncopower = (uint32_t)(pow(10,((float)txNcoTestToneCfg->Tx2Power/20))*0x8000);
			// tx1Ncopower = (uint32_t)(myintpowbase10(txNcoTestToneCfg->Tx2Power/20, txNcoTestToneCfg->Tx2Power%20)*0x8000/10000);
			if(tx2Ncopower >= 0x8000){
			    tx2Ncopower = 0x7FFF;
			}
		}
		retVal = BR3109_armSpiCmd_tx_ncopower_set(device, tx_ch, tx1Ncopower, tx2Ncopower);
		IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_armSpiCmd_tx_NCO_set(device,txNcoTestToneCfg->enable&0x3, tx_ch, txNcoTestToneCfg->Tone1Freq_kHz,txNcoTestToneCfg->Tone2Freq_kHz);
		IF_ERR_RETURN_U32(retVal);
	} else {
		/* Disable NCO test mode */
		retVal = BR3109_armSpiCmd_tx_NCO_set(device,0,0, txNcoTestToneCfg->Tone1Freq_kHz,txNcoTestToneCfg->Tone2Freq_kHz);
		IF_ERR_RETURN_U32(retVal);
		//#Enable DP_TX_FIX_EN
		retVal = BR3109_armSpiCmd_tx_ncopower_set(device, 0, 0, 0);
		IF_ERR_RETURN_U32(retVal);
	}

	return (uint32_t)retVal;
}

uint32_t BR3109_setTxAttenCtrlPin(br3109Device_t *device, br3109TxChannels_t txChannel, br3109TxAttenCtrlPin_t *txAttenCtrlPin)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	br3109GpioShortConfig_t attpin_config;

	static const uint8_t TX_INCDEC_MASK = 0x1F;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_setTxAttenCtrlPin()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/*Check passed pointers for NULL */
	if (txAttenCtrlPin == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_SETTXATTENCTRLPIN_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/* Check out of range */
	if ( txAttenCtrlPin->stepSize > TX_INCDEC_MASK) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_SETTXATTENCTRLPIN_INV_PARM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	if((txChannel != TAL_TX1) && (txChannel != TAL_TX2)){
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_SETTXATTENCTRLPIN_INV_CHANNEL, retVal, TALACT_ERR_CHECK_PARAM);
	}
	retVal = BR3109_setGpioOe(device, 0, (0x1 << txAttenCtrlPin->txAttenIncPin) | (0x1 << txAttenCtrlPin->txAttenDecPin));
	IF_ERR_RETURN_U32(retVal);	
	retVal = BR3109_setGpioSourceCtrl(device, TAL_GPIO_TX_ATT_CONTROL_MODE_2 << ((txAttenCtrlPin->txAttenIncPin / 4)*4));
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_getTxAttnSel(device, &attpin_config);
	IF_ERR_RETURN_U32(retVal);
	if(txChannel == TAL_TX1){
		attpin_config.p0.gpioPinSel = txAttenCtrlPin->txAttenIncPin;
		attpin_config.p0.enable = txAttenCtrlPin->enable > 0 ? 1 : 0;
		attpin_config.p1.gpioPinSel = txAttenCtrlPin->txAttenDecPin;
		attpin_config.p1.enable = txAttenCtrlPin->enable > 0 ? 1 : 0;
	}else{
		attpin_config.p2.gpioPinSel = txAttenCtrlPin->txAttenIncPin;
		attpin_config.p2.enable = txAttenCtrlPin->enable > 0 ? 1 : 0;
		attpin_config.p3.gpioPinSel = txAttenCtrlPin->txAttenDecPin;
		attpin_config.p3.enable = txAttenCtrlPin->enable > 0 ? 1 : 0;		
	}
	retVal = BR3109_setTxAttnSel(device, &attpin_config);
	IF_ERR_RETURN_U32(retVal);
	
	return (uint32_t)retVal;
}

uint32_t BR3109_getTxAttenCtrlPin(br3109Device_t *device,  br3109TxChannels_t txChannel, br3109TxAttenCtrlPin_t *txAttenCtrlPin)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	// uint32_t regdat = 0;
	// int i =  0;
	// uint8_t fieldValue;
	br3109GpioShortConfig_t attpin_config;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK, "BR3109_getTxAttenCtrlPin()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal, TALACT_WARN_RESET_LOG);
#endif

	/* Check channel */
	if ((txChannel != TAL_TX1) &&
	    (txChannel != TAL_TX2)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETTXATTENCTRLPIN_INV_CHANNEL, retVal, TALACT_ERR_CHECK_PARAM);
	}

	/*Check passed pointers for NULL */
	if (txAttenCtrlPin == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM, TAL_ERR_GETTXATTENCTRLPIN_NULL_PARAM, retVal, TALACT_ERR_CHECK_PARAM);
	}
	retVal = BR3109_getTxAttnSel(device, &attpin_config);
	IF_ERR_RETURN_U32(retVal);
	if(txChannel == TAL_TX1){
		txAttenCtrlPin->txAttenIncPin = attpin_config.p0.gpioPinSel;
		txAttenCtrlPin->enable = attpin_config.p0.enable;
		txAttenCtrlPin->txAttenDecPin = attpin_config.p1.gpioPinSel;
		txAttenCtrlPin->enable = attpin_config.p1.enable;
	}else{
		txAttenCtrlPin->txAttenIncPin = attpin_config.p2.gpioPinSel;
		txAttenCtrlPin->enable = attpin_config.p2.enable;
		txAttenCtrlPin->txAttenDecPin = attpin_config.p3.gpioPinSel;
		txAttenCtrlPin->enable = attpin_config.p3.enable;	
	}

	// /* Get value of GPIO selects */
	// retVal = BR3109_getGpioSourceCtrl(device, &regdat);
	// IF_ERR_RETURN_U32(retVal);
	// for(i = 0; i < 5; i++)
	// {
	// 	if(((regdat >> (i * 4)) & 0xF) == TAL_GPIO_TX_ATT_CONTROL_MODE_2)
	// 		break;
	// }
	// if(i >= 5){
	// 	txAttenCtrlPin->txAttenIncPin = 0;
	// 	txAttenCtrlPin->txAttenDecPin = 0;
	// 	return retVal;
	// }
	// regdat = ((regdat >> (i * 4)) & 0xF);
	// /* Value of txAttenIncPin */
	// fieldValue = (txChannel == TAL_TX1) ? (regdat & 0x03) : ((
	// 			regdat >> 4) & 0x03);
	// txAttenCtrlPin->txAttenIncPin = (txChannel == TAL_TX1) ? ((
	// 					fieldValue == 0x00) ? TAL_GPIO_04 : ((fieldValue == 0x01) ? TAL_GPIO_12 :
	// 							TAL_GPIO_INVALID))
	// 				: ((fieldValue == 0x00) ? TAL_GPIO_06 : ((fieldValue == 0x01) ? TAL_GPIO_14 :
	// 						TAL_GPIO_INVALID));

	// /* Value of txAttenDecPin */
	// fieldValue = (txChannel == TAL_TX1) ? ((regdat >> 2) & 0x03) : ((
	// 			regdat >> 6) & 0x03);
	// txAttenCtrlPin->txAttenDecPin = (txChannel == TAL_TX1) ? ((
	// 					fieldValue == 0x00) ? TAL_GPIO_05 : ((fieldValue == 0x01) ? TAL_GPIO_13 :
	// 							TAL_GPIO_INVALID))
	// 				: ((fieldValue == 0x00) ? TAL_GPIO_07 : ((fieldValue == 0x01) ? TAL_GPIO_15 :
	// 						TAL_GPIO_INVALID));

	return (uint32_t)retVal;
}
#if 0
uint32_t BR3109_setPaProtectionCfg(br3109Device_t *device,
				   br3109TxPaProtectCfg_t *txPaProtectCfg)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t peakCount = 0;
	uint8_t tx1PeakThreshold = 0;
	uint8_t tx2PeakThreshold = 0;
	uint8_t paProtectCfg = 0;
	uint8_t peakCountMax = 31;

	static const uint8_t ENABLE_PA_PROTECTION_MEASUREMENT = 0x01;
	static const uint8_t ENABLE_PEAK_MEASUREMENT = 0x01;
	static const uint8_t ERROR_FLAGS_STICKY = 0x40;
	static const uint8_t TX_ATTEN_REDUCTION_MAX = 127;
	static const uint16_t POWER_THRESHOLD_MAX = 8191;
	static const uint8_t AVG_DURATION_MAX = 14;
	static const uint8_t SILICON_REV_C0 = 0xC0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_setPaProtectionCfg()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* If Silicon Revision is less than C0 - Max Peak Count is 30 */
	if (device->devStateInfo.deviceSiRev < SILICON_REV_C0) {
		/*Update the max peak count for silicon revisions lower than C0 to 30*/
		peakCountMax = 30;
	}

	if (txPaProtectCfg == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETPAPRO_NULL_PARAM, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	if (txPaProtectCfg->avgDuration > AVG_DURATION_MAX) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETPAPRO_INV_AVGDURATION, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	if (txPaProtectCfg->peakCount > peakCountMax) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETPAPROT_INV_PEAKCNT, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	if (txPaProtectCfg->txAttenStep > TX_ATTEN_REDUCTION_MAX) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETPAPROT_INV_TXATTENSTEP, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	if ((txPaProtectCfg->tx1PowerThreshold == 0) ||
	    (txPaProtectCfg->tx1PowerThreshold > POWER_THRESHOLD_MAX)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETPAPROT_INV_TX1THRESH, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	if ((txPaProtectCfg->tx2PowerThreshold == 0) ||
	    (txPaProtectCfg->tx2PowerThreshold > POWER_THRESHOLD_MAX)) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETPAPROT_INV_TX2THRESH, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	if (txPaProtectCfg->tx1PeakThreshold == 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETPAPROT_INV_TX1PEAKTHRESH, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	if (txPaProtectCfg->tx2PeakThreshold == 0) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_SETPAPROT_INV_TX2PEAKTHRESH, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	/* Disable PA Protection before changing settings paProtectCfg[0]=0 */
	/* Set Tx Power average duration paProtectCfg[4:1] */
	/* Allow error flags to be stick (set until cleared by SPI clear bit paProtectCfg[6]=1 */
	paProtectCfg = ((txPaProtectCfg->avgDuration << 1) | ERROR_FLAGS_STICKY);
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_CONFIGURATION, paProtectCfg);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_PA_PROTECTION_ATTEN_CONTROL, txPaProtectCfg->txAttenStep, 0xFE, 1);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Set Tx1 average power threshold[7:0] */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_THRESHOLD_0,
				   (txPaProtectCfg->tx1PowerThreshold & 0xFF));
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Set Tx1 average power threshold[12:8] */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_THRESHOLD_1,
				   ((txPaProtectCfg->tx1PowerThreshold >> 8) & 0xFF));
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Set Tx2 average power threshold[7:0] */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_THRESHOLD_2,
				   (txPaProtectCfg->tx2PowerThreshold & 0xFF));
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Set Tx2 average power threshold[12:8] */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_THRESHOLD_3,
				   ((txPaProtectCfg->tx2PowerThreshold >> 8) & 0xFF));
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Set Peak mode settings */
	if (txPaProtectCfg->peakCount == 0) {
		peakCount = 0;
		tx1PeakThreshold = 0;
		tx2PeakThreshold = 0;
	} else {
		peakCount = ((txPaProtectCfg->peakCount << 1) | ENABLE_PEAK_MEASUREMENT);
		tx1PeakThreshold = txPaProtectCfg->tx1PeakThreshold;
		tx2PeakThreshold = txPaProtectCfg->tx2PeakThreshold;
	}

	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_PEAK_THRESHOLD_CH1, tx1PeakThreshold);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_PEAK_THRESHOLD_CH2, tx2PeakThreshold);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Set peak count and peak enable bit after peak thresholds */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_PEAK_COUNT, peakCount);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Enable PA Power Protection measurement */
	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_PA_PROTECTION_CONFIGURATION, ENABLE_PA_PROTECTION_MEASUREMENT, 0x01,
				    0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Clear PA Protection Error flags */
	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_PA_PROTECTION_CONFIGURATION, 1, 0x80, 7);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getPaProtectionCfg(br3109Device_t *device,
				   br3109TxPaProtectCfg_t *txPaProtectCfg)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t tx1PowerThresholdLsb = 0;
	uint8_t tx1PowerThresholdMsb = 0;
	uint8_t tx2PowerThresholdLsb = 0;
	uint8_t tx2PowerThresholdMsb = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getPaProtectionCfg()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if (txPaProtectCfg == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETPAPRO_NULL_PARAM, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	halError = brSpiReadField(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_CONFIGURATION, &txPaProtectCfg->avgDuration, 0x1E, 1);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = brSpiReadField(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_ATTEN_CONTROL, &txPaProtectCfg->txAttenStep, 0xFE, 1);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = talSpiReadByte(device->devHalInfo,
				  BR3109_ADDR_PA_PROTECTION_THRESHOLD_0, &tx1PowerThresholdLsb);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = brSpiReadField(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_THRESHOLD_1, &tx1PowerThresholdMsb, 0x1F, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	txPaProtectCfg->tx1PowerThreshold = ((uint16_t)tx1PowerThresholdMsb << 8) |
					    (uint16_t)tx1PowerThresholdLsb;

	halError = talSpiReadByte(device->devHalInfo,
				  BR3109_ADDR_PA_PROTECTION_THRESHOLD_2, &tx2PowerThresholdLsb);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = brSpiReadField(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_THRESHOLD_3, &tx2PowerThresholdMsb, 0x1F, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	txPaProtectCfg->tx2PowerThreshold = ((uint16_t)tx2PowerThresholdMsb << 8) |
					    (uint16_t)tx2PowerThresholdLsb;

	halError = brSpiReadField(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_PEAK_COUNT, &txPaProtectCfg->peakCount, 0x3E, 1);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = talSpiReadByte(device->devHalInfo,
				  BR3109_ADDR_PA_PROTECTION_PEAK_THRESHOLD_CH1,
				  &txPaProtectCfg->tx1PeakThreshold);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = talSpiReadByte(device->devHalInfo,
				  BR3109_ADDR_PA_PROTECTION_PEAK_THRESHOLD_CH2,
				  &txPaProtectCfg->tx2PeakThreshold);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_enablePaProtection(br3109Device_t *device/*, uint8_t enableTxAttenCtrl*/)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t enTxAttenCtrl = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_enablePaProtection()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* TODO: Currently do not allow enabling this feature due to a hardware
	 * issue found.  Calling this function will not enable control of TxAttenuation when
	 * the PA Protection error flag asserts.  BBIC will need to take action to reduce
	 * Tx sample power.
	 */

	/* enTxAttenCtrl = (enableTxAttenCtrl > 0) ? 1 : 0; */
	enTxAttenCtrl = 0;

	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_PA_PROTECTION_ATTEN_CONTROL, enTxAttenCtrl, 0x01, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_getTxSamplePower(br3109Device_t *device,
				 br3109TxChannels_t txChannel, uint16_t *channelPower)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;
	uint8_t paProtectChannelSel = 0;
	uint8_t chPowerLsb = 0;
	uint8_t chPowerMsb = 0;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getTxSamplePower()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if (channelPower == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETTXSAMPLEPWR_NULL_PARAM, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	switch(txChannel) {
	case TAL_TX1:
		paProtectChannelSel = 0;
		break;

	case TAL_TX2:
		paProtectChannelSel = 1;
		break;

	default:
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETTXSAMPLEPWR_INV_TXREADCHAN, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	/* Set channel for power read back */
	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_PA_PROTECTION_CONFIGURATION, paProtectChannelSel, 0x20, 5);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Write power readback reg to force the value to update (write strobe) */
	halError = talSpiWriteByte(device->devHalInfo,
				   BR3109_ADDR_PA_PROTECTION_POWER_0, 0x00);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	/* Read back DAC power for selected channel */
	halError = talSpiReadByte(device->devHalInfo, BR3109_ADDR_PA_PROTECTION_POWER_0,
				  &chPowerLsb);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	halError = talSpiReadByte(device->devHalInfo, BR3109_ADDR_PA_PROTECTION_POWER_1,
				  &chPowerMsb);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	*channelPower = ((uint16_t)chPowerLsb | (uint16_t)(chPowerMsb << 8));

	return (uint32_t)retVal;
}

uint32_t BR3109_getPaProtectErrorFlags(br3109Device_t *device,
				       uint8_t *errorFlags)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_getPaProtectErrorFlags()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	if (errorFlags == NULL) {
		return (uint32_t)talApiErrHandler(device, TAL_ERRHDL_INVALID_PARAM,
						  TAL_ERR_GETPAERRFLAGS_NULL_PARAM, retVal,
						  TALACT_ERR_CHECK_PARAM);
	}

	/* Read back PA Protection Error flags */
	halError = brSpiReadField(device->devHalInfo, BR3109_ADDR_PA_PROTECTION_ERROR,
				   errorFlags, 0x03, 0);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}

uint32_t BR3109_clearPaProtectErrorFlags(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	brHalErr_t halError = BRHAL_OK;

#if BR3109_VERBOSE
	halError = brWriteToLog(device->devHalInfo, BRHAL_LOG_MSG, TAL_ERR_OK,
				 "BR3109_clearPaProtectErrorFlags()\n");
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_LOG, halError, retVal,
				  TALACT_WARN_RESET_LOG);
#endif

	/* Clear PA Protection Error flags */
	halError = brSpiWriteField(device->devHalInfo,
				    BR3109_ADDR_PA_PROTECTION_CONFIGURATION, 1, 0x80, 7);
	retVal = talApiErrHandler(device, TAL_ERRHDL_HAL_SPI, halError, retVal,
				  TALACT_ERR_RESET_SPI);
	IF_ERR_RETURN_U32(retVal);

	return (uint32_t)retVal;
}
#endif
