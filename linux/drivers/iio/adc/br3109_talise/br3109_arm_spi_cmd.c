#include "br3109_arm.h"
#include "br3109.h"
#include "br3109_error.h"
#include "br3109_user.h"
#include "br3109_hal.h"
#include "br3109_arm_spi_cmd.h"
// #include <math.h>

static uint32_t __BR3109_armSpiCmd_blkcpy(br3109Device_t *device, uint32_t srcAdr, uint32_t dstAdr, uint32_t wSz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[3]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = srcAdr&0x7FFFFFFF;
    cmdbuf[1] = dstAdr&0x7FFFFFFF;
    cmdbuf[2] = (wSz)&0xFFFF;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_BLKCPY|TALAPI_ARMSPI_OPCODE_CMDSIZE(3), cmdbuf, 3);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armMemoryCmd_blk_write(br3109Device_t *device, uint32_t startRegAddr, uint32_t *wrDataBuf, uint32_t  wrWordSz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t cmdstatword;
	uint32_t curStAddr = startRegAddr;
    uint32_t readDSz = 0;
    uint16_t curBatchSz = 0;
    while(readDSz<wrWordSz){
        curBatchSz = min(TALAPI_ARM_CPY_MEM_MAX,wrWordSz-readDSz);
		retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	    IF_ERR_RETURN_U32(retVal);
		halError = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_ARM_EXT_CMD_WORD_1, &wrDataBuf[readDSz], curBatchSz);
        retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
		retVal = __BR3109_armSpiCmd_blkcpy(device, BR3109_ADDR_ARM_EXT_CMD_WORD_1, curStAddr, curBatchSz);
		IF_ERR_RETURN_U32(retVal);
        curStAddr = curStAddr + curBatchSz*4;
        readDSz = readDSz + curBatchSz;
    }
	return retVal;

}

uint32_t BR3109_armMemoryCmd_blk_read(br3109Device_t *device, uint32_t startRegAddr, uint32_t *rdDataBuf, uint32_t rdWordSz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t curStAddr = startRegAddr;
    uint32_t readDSz = 0;
    uint16_t curBatchSz = 0;
    while(readDSz<rdWordSz){
        curBatchSz = min(TALAPI_ARM_CPY_MEM_MAX,rdWordSz-readDSz);
		retVal = __BR3109_armSpiCmd_blkcpy(device, curStAddr, BR3109_ADDR_ARM_EXT_CMD_WORD_1, curBatchSz);
		IF_ERR_RETURN_U32(retVal);
		halError = brSpiReadWordsBlock(device->devHalInfo, BR3109_ADDR_ARM_EXT_CMD_WORD_1, &rdDataBuf[readDSz], curBatchSz);
        retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
        curStAddr = curStAddr + curBatchSz*4;
        readDSz = readDSz + curBatchSz;
    }
	return retVal;
}

uint32_t BR3109_armSpiCmd_SPI_blk_write(br3109Device_t *device, uint8_t SPI_select, uint16_t regAddr, uint32_t *wrDataBuf, uint32_t  wrWordSz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t cmdstatword;
	uint32_t curStAddr = regAddr;
    uint32_t readDSz = 0;
    uint16_t curBatchSz = 0;
	uint32_t cmdbuf[3]={0};
    cmdbuf[0] = BR3109_ADDR_ARM_EXT_CMD_WORD_1;//0x3f08
    cmdbuf[1] = 1<<SPI_select;//0x3f0c
    while(readDSz<wrWordSz){
        curBatchSz = min(TALAPI_ARM_CPY_MEM_MAX,wrWordSz-readDSz);
    	cmdbuf[2] = (curStAddr&0x0000ffff)|((curBatchSz<<16)&0x1FFF0000);//0x3f10
		retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	    IF_ERR_RETURN_U32(retVal);
		halError = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_ARM_EXT_CMD_WORD_1, &wrDataBuf[readDSz], curBatchSz);
        retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
		retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SPI_BLK_WRITE|TALAPI_ARMSPI_OPCODE_CMDSIZE(3), cmdbuf, 3);
	    IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	    IF_ERR_RETURN_U32(retVal);
        curStAddr = curStAddr + curBatchSz*4;
        readDSz = readDSz + curBatchSz;
    }
	return retVal;

}

uint32_t BR3109_armSpiCmd_SPI_blk_read(br3109Device_t *device, uint8_t SPI_select, uint16_t regAddr, uint32_t *rdDataBuf, uint32_t rdWordSz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t cmdstatword;
	uint32_t curStAddr = regAddr;
    uint32_t readDSz = 0;
    uint16_t curBatchSz = 0;
	uint32_t cmdbuf[3]={0};
    cmdbuf[0] = 1<<SPI_select ;//0x3f08
    cmdbuf[2] = BR3109_ADDR_ARM_EXT_CMD_WORD_1;//0x3f10
    while(readDSz<rdWordSz){
		retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	    IF_ERR_RETURN_U32(retVal);
        curBatchSz = min(TALAPI_ARM_CPY_MEM_MAX,rdWordSz-readDSz);
    	cmdbuf[1] = (curStAddr&0x0000ffff)|((curBatchSz<<16)&0x1FFF0000);//0x3f0c
    	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SPI_BLK_READ|TALAPI_ARMSPI_OPCODE_CMDSIZE(3), cmdbuf, 3);
	    IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	    IF_ERR_RETURN_U32(retVal);
		halError = brSpiReadWordsBlock(device->devHalInfo, BR3109_ADDR_ARM_EXT_CMD_WORD_1, &rdDataBuf[readDSz], curBatchSz);
        retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
        curStAddr = curStAddr + curBatchSz*4;
        readDSz = readDSz + curBatchSz;
    }
	return retVal;
}

uint32_t BR3109_armSpiCmd_clear_tx_qfir(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_CLEAR_TX_QFIR, NULL, 0);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_clear_tx_cfir(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_CLEAR_TX_CFIR, NULL, 0);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_setup_bb_rf_pll(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SETUP_BB_RF_PLL, NULL, 0);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_puresweep_lb_rx1(br3109Device_t *device, CHANNEL_t sweep_ch, uint8_t bw, uint32_t check_point)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((sweep_ch-1)&0xF) | ((check_point<<4)&0xF0) | ((bw<<8)&0xF00);                   //(sweep_ch-1)为了跟arm程序兼容，arm中做了ch+1操作
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_PURESWEEP_LB_RX1, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);

	return retVal;
}

uint32_t BR3109_armSpiCmd_setonefreq_lb_rx1(br3109Device_t *device, CHANNEL_t freq_ch, uint8_t bw, uint8_t freq_index, uint32_t check_point)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((freq_ch-1)&0xF) | ((check_point<<4)&0xF0) | ((bw<<8)&0xF00) | ((freq_index<<12)&0xFF000);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SETONEFREQ_LB_RX1, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);

	return retVal;
}

uint32_t BR3109_armSpiCmd_calibrationfunc_lb_rx1(br3109Device_t *device, CHANNEL_t cali_ch, uint8_t cali_bw, uint8_t set_fir_flag)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((cali_ch-1)&0xF) | ((cali_bw<<4)&0xF0) | ((set_fir_flag<<8)&0xF00);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_CALIBRATIONFUNC_LB_RX1, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	
	return retVal;
}

uint32_t BR3109_armSpiCmd_puresweep_lb_orx1(br3109Device_t *device,  CHANNEL_t sweep_ch, uint8_t bw, uint32_t check_point)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((sweep_ch-1)&0xF) | ((check_point<<4)&0xF0) | ((bw<<8)&0xF00);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_PURESWEEP_LB_ORX1, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);

	return retVal;
}

uint32_t BR3109_armSpiCmd_setonefreq_lb_orx1(br3109Device_t *device, CHANNEL_t freq_ch, uint8_t freq_index, uint8_t bw,  uint8_t check_point)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((freq_ch-1)&0xF) | ((check_point<<4)&0xF0) | ((bw<<8)&0xF00) | ((freq_index<<12)&0xFF000);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SETONEFREQ_LB_ORX1, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);

	return retVal;
}

uint32_t BR3109_armSpiCmd_calibrationfunc_lb_orx1(br3109Device_t *device, CHANNEL_t cali_ch, uint8_t cali_bw, uint8_t set_fir_flag)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((cali_ch-1)&0xF) | ((cali_bw<<4)&0xF0) | ((set_fir_flag<<8)&0xF00);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_CALIBRATIONFUNC_LB_ORX1, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	
	return retVal;
}

uint32_t BR3109_armSpiCmd_tx_puresweep(br3109Device_t *device, CHANNEL_t cali_ch, uint8_t bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((cali_ch-1)&0xF) | ((bw<<8)&0xF00);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_PURESWEEP, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);

	return retVal;
}

uint32_t BR3109_armSpiCmd_tx_setonefreq(br3109Device_t *device, CHANNEL_t tx_ch, uint8_t freq_bw, uint32_t freqpoint)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((tx_ch-1)&0xF) | ((freq_bw<<8)&0xF00) | ((freqpoint<<12)&0xFF000);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_SETONEFREQ, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);

	return retVal;
}

uint32_t BR3109_armSpiCmd_tx_calibrationfunc(br3109Device_t *device,  CHANNEL_t cali_ch, uint8_t cali_bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((cali_ch-1)&0xF) | ((cali_bw<<4)&0xF0);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_CALIBRATIONFUNC, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	
	return retVal;
}
/**
 * lb_channel: CHANNEL_1 or CHANNEL_2
 * rx_orx_ch: 外部回环时，tx loopback 到rx 或者orx 的channel，只能为CHANNEL_1，CHANNEL_2
 * band: 带宽	0:窄带（rx：100M，orx：200M）	1：宽带（rx：200M，orx：400M）
 * lp_type：回环模式：0：芯片内部回环 1：芯片外部回环
 * loopmode：0：loopback to rx， 1：loopback to orx, 2：loopback to orx（芯片内部回环有用，内部orx另外一条通路），其他值无效错误
*/
uint32_t BR3109_armSpiCmd_setup_tx_loopback(br3109Device_t *device, CHANNEL_t lb_channel, CHANNEL_t rx_orx_ch, uint8_t band, uint8_t lp_type, uint8_t loopmode)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	if(loopmode > 2){
		retVal = TALACT_ERR_CHECK_PARAM;
		return retVal;
	}
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((lb_channel-1)&0xF) | ((band<<4)&0xF0) | ((lp_type<<12)&0xF000) | ((rx_orx_ch & 0x3)<<20) | ((loopmode<<30)&0xC0000000);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SETUP_TX_LOOPBACK, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	
	return retVal;
}

uint32_t BR3109_armSpiCmd_mem_mask_write(br3109Device_t *device, uint32_t val, uint32_t addr, uint32_t mask)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[3]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = val&0x7FFFFFFF;
    cmdbuf[1] = addr&0x7FFFFFFF;
    cmdbuf[2] = mask&0x7FFFFFFF;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_MEM_MASK_WRITE|TALAPI_ARMSPI_OPCODE_CMDSIZE(3), cmdbuf, 3);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_spi_dev_mask_write(br3109Device_t *device, uint32_t mask, uint32_t spi_sel, uint16_t regAddr, uint32_t val)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[4]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = mask&0x7FFFFFFF;
    cmdbuf[1] = spi_sel&0x7FFFFFFF;
    cmdbuf[2] = regAddr&0xFFFF;
	cmdbuf[3] = val&0x7FFFFFFF;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SPI_DEV_MASK_WRITE|TALAPI_ARMSPI_OPCODE_CMDSIZE(3), cmdbuf, 4);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_setup_loopback_lb_rx1(br3109Device_t *device, CHANNEL_t lb_channel, uint8_t bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((lb_channel-1)&0xF) | ((bw<<8)&0xF00);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SETUP_LOOPBACK_LB_RX1, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_setup_loopback_lb_orx1(br3109Device_t *device, CHANNEL_t lb_channel, uint8_t bw, uint8_t lb_src)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((lb_channel-1)&0xF) | ((bw<<8)&0xF00) | (((lb_src-1)<<31)&0x80000000);            //arm中进行了+1操作
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SETUP_LOOPBACK_LB_ORX1, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_tx_lo_leak_cali(br3109Device_t *device, CHANNEL_t set_channel, uint8_t set_bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((set_channel-1)&0xF) | ((set_bw<<4)&0xF0);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_LO_LEAK_CALI, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_rx_lo_leak_cali_addr4(br3109Device_t *device, CHANNEL_t set_channel, uint8_t set_bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((set_channel-1)&0xF) | ((set_bw<<4)&0xF0);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_O_RX_LO_LEAK_CALI_ADDR4, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_orx_lo_leak_cali_addr1c(br3109Device_t *device, CHANNEL_t set_channel, uint8_t set_bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((set_channel-1)&0xF) | ((set_bw<<4)&0xF0);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_O_RX_LO_LEAK_CALI_ADDR1C, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**tx_rx_orx_sel: 0x2: rx tracking, 0x3: tx tracking
**CHANNEL_t ch：0x01:ch1，0x02:ch2
**uint8_t bw：0—窄带200Mhz/245.76Msps，1—宽带450Mhz/491.52Msps
*/
uint32_t BR3109_armSpiCmd_track_config(br3109Device_t *device,uint8_t tx_rx_orx_sel, CHANNEL_t ch, uint8_t bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((ch-1)&0xF) | ((bw<<4)&0xF0) | (tx_rx_orx_sel<<28);
	retVal= BR3109_sendArmCommand(device, SPI_CMD_TRACK_CONFIG, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_tuning_iip(br3109Device_t *device, uint8_t src, CHANNEL_t set_channel, uint8_t step, uint8_t set_bw, uint32_t freqBin_1, uint32_t freqBin_2, uint32_t freqBin_3, uint32_t freqBin_4)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	// uint32_t cmdstatword;
	// uint32_t cmdbuf[5]={0};
	// retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    // IF_ERR_RETURN_U32(retVal);
    // cmdbuf[0] = (src&0xF) | (((set_channel-1)<<4)&0xF0) | ((step<<8)&0xFF00) | ((set_bw<<16)&0xF0000);
	// cmdbuf[1] = freqBin_1;
	// cmdbuf[2] = freqBin_2;
	// cmdbuf[3] = freqBin_3;
	// cmdbuf[4] = freqBin_4;
	// retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TUNING_IIP, cmdbuf, 5);
    // IF_ERR_RETURN_U32(retVal);
	// retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    // IF_ERR_RETURN_U32(retVal);
	return retVal;
}
/** 配置tracking enable
*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint32_t enableMask：详见br3109TrackingCalibrations_t
*/
uint32_t BR3109_armSpiCmd_track_cali_En(br3109Device_t *device, uint32_t enableMask)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = enableMask;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TRACK_CALI, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/** 配置 rx
*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**CHANNEL_t ch：0x01—ch1，0x02—ch2 0x3: ch1 & ch2
**uint8_t index：根据配置最大值最小值进行配置
**uint8_t mode : 0: MGC 1：AGC 模式
*/
uint32_t BR3109_armSpiCmd_config_agc(br3109Device_t *device, CHANNEL_t channel, uint8_t index, uint8_t mode)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	cmdbuf[0] = (channel&0xF) | ((index<<4)&0xFF0) | ((mode<<31)&0x80000000);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_CONFIG_AGC, cmdbuf, 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	return retVal;
}
/**turn off loopback and radio
 * 
*/
uint32_t BR3109_armSpiCmd_turnoff_all(br3109Device_t *device)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TURNOFF_LOOPBACK_LO_ORFPLL, NULL, 0);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_rx_dig_dc_xal(br3109Device_t *device, CHANNEL_t set_channel, uint8_t set_bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((set_channel-1)&0xF) | ((set_bw<<4)&0xF0);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_RX_DIG_DC_XAL, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**CHANNEL_t ch：0x01—ch1，0x02—ch2 0x3: ch1 & ch2，
**tx_ch		:选择tx 校准通道
**rx_ch		:选择rx 校准通道
**orx_ch	:选择orx 校准通道
**tx1_lp2orx_ch	：tx外部回环校准时，orx的通道，0:使用与tx相同的通道进行校准，0x1：txloopback2orx1,0x2：txloopback2orx2， 这里仅外部回环有用,
**tx2_lp2orx_ch	：tx外部回环校准时，orx的通道，0:使用与tx相同的通道进行校准，0x1：txloopback2orx1,0x2：txloopback2orx2， 这里仅外部回环有用,
**uint8_t set_bw：0—窄带200Mhz/245.76Msps，1—宽带450Mhz/491.52Msps
**uint8_t external_lp : 0: 芯片内部回环 1：芯片外部回环
**cali_flag: 参见enum CALI_part_t
*/
uint32_t BR3109_armSpiCmd_Initical_cali(br3109Device_t *device, CHANNEL_t tx_ch, CHANNEL_t rx_ch, CHANNEL_t orx_ch, CHANNEL_t tx1_lp2orx_ch, CHANNEL_t tx2_lp2orx_ch, 
uint8_t set_bw, uint8_t external_lp, uint32_t cali_flag)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[2]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	//bit16 1:可以选择校准选项， 0：全部校准 bit17:1新版本 0：旧版本，tx orx rx通道统一为tx通道
    cmdbuf[0] = ((tx_ch-1)&0x3)| (tx_ch << 2)| (rx_ch << 24) | (orx_ch << 26) | (tx1_lp2orx_ch << 20) | (tx2_lp2orx_ch << 22)| ((set_bw<<4)&0xF0) | ((external_lp<<12)&0xF000) | (0x3<<16);
	cmdbuf[1] = cali_flag;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_INIT_CAL, cmdbuf, 2);
    IF_ERR_RETURN_U32(retVal);
//	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 300000000,WAITINITCALS_INTERVAL_US);
//    IF_ERR_RETURN_U32(retVal);
	return retVal;
}
/**
 * jesdtx_rx: 0x1 只开启jesd tx fifo 0x2：只开启jesd rx fifo 0x3：开启jesd tx rx fifo
*/
uint32_t BR3109_armSpiCmd_Jesd_config(br3109Device_t *device, uint8_t jesdtx_rx)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	cmdbuf[0] = jesdtx_rx;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_JESD_CONFIG, cmdbuf, 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_setrf_freq(br3109Device_t *device, uint32_t freq_KHz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[3]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	cmdbuf[0] = freq_KHz;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SETRF_FREQ_NEW, cmdbuf, 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**ch1_sync_q：0-3 四档位调整
**ch2_sync_q：0-3 四档位调整
**
*/
uint32_t BR3109_armSpiCmd_SyncAdc_Q(br3109Device_t *device, uint8_t ch1_sync_q, uint8_t ch2_sync_q)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (ch1_sync_q&0xf) | ((ch2_sync_q<<4)&0xF0);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SYNC_ADC_Q, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

uint32_t BR3109_armSpiCmd_setorf_freq(br3109Device_t *device, uint32_t freq_KHz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[3]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	cmdbuf[0] = freq_KHz;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_SETORF_FREQ, cmdbuf, 1);
	IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**CHANNEL_t ch：0x01:ch1，0x02:ch2
**uint8_t bw：0—窄带200Mhz/245.76Msps，1—宽带450Mhz/491.52Msps
*/
uint32_t BR3109_armSpiCmd_AGC_CALI(br3109Device_t *device, CHANNEL_t ch, uint8_t bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (bw&0xf) | (((ch-1)<<4)&0x10);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_AGC_CALI, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}


/********************************BR3109_ES2_API SpiCmd update********************************/
/**实现tx/rx/orx通道的平坦度校准**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**CHANNEL_t cali_ch：0x01—ch1，0x02—ch2
**uint8_t cali_bw：0—窄带200Mhz/245.76Msps，1—宽带450Mhz/491.52Msps
**uint8_t tx_rx_orxsel：0x1，0x2:校准rx orx ， 0x4：校准tx
*/
uint32_t BR3109_armSpiCmd_cali_cplxfir(br3109Device_t *device, CHANNEL_t cali_ch, uint8_t cali_bw, uint8_t tx_rx_orxsel)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (cali_ch&0x3) | ((cali_bw<<4)&0xF0) | ((tx_rx_orxsel<<8)&0xF00);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_CALI_CPLXFIR, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**实现捕获模式设置**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t mode：0-play模式, 1-capture模式
**uint8_t continue_mode：0:单次工作模式, 1:连续工作模式
**uint8_t start_pluse:启动信号选择，0-使用manual_start作为启动信号，1-使用TDD控制信号作为启动信号
**uint8_t sel_AB:0x01—capture A, 0x02—capture B, 0x03—capture A&B
**uint8_t div:0-DIV_1, 1-DIV_2, 2-DIV_4, 3-DIV_8, 4-DIV_16, 5-DIV_32, 6-DIV_64
**CHANNEL_t ch：0x01—ch1, 0x02—ch2
**uint16_t src：捕获信号源选择, 0x0~0x1F, 具体见CAPTURE_Src_t
*/
uint32_t BR3109_armSpiCmd_playcap_set(br3109Device_t *device, uint8_t mode, uint8_t continue_mode, uint8_t start_pluse, uint8_t sel_AB, uint8_t div, CHANNEL_t ch, uint16_t src)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (mode&0xf) | ((continue_mode<<4)&0xF0) | ((start_pluse<<5)&0x20) | ((sel_AB<<6)&0xC0) | ((div<<8)&0xF00) | ((ch<<12)&0xF000) | ((src<<16)&0xFF0000);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_PLAYCAP_SET, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**实现捕获模式的选择**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t start_ab：0x01—capture A,0x02—capture B,0x03—capture A&B
*/
uint32_t BR3109_armSpiCmd_playcap_manual_start(br3109Device_t *device, uint8_t start_ab)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((start_ab<<6)&0xC0);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_PLAYCAP_MANUAL_START, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}
/**实现TDD/manual mode**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t mode: 1:TDD 0:manual
*/
uint32_t BR3109_armSpiCmd_TddManualsel(br3109Device_t *device, uint8_t mode)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (mode & 0xF);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TDD_MANUAL_MODE_SET, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**实现rx/orx通道使能**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**br3109RxORxChannels_t ch：0x01—rx1,0x02—rx2,0x03—rx1&rx2,0x04—orx1,0x08—orx2,0x0c—orx1&orx2
*/
uint32_t BR3109_armSpiCmd_orxrx_ch_en(br3109Device_t *device, br3109RxORxChannels_t ch)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (ch & 0xF);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_ORXRX_CH_EN, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**实现rx/orx通道带宽设置**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t bw：0—窄带200Mhz/245.76Msps，1—宽带450Mhz/491.52Msps
*/
uint32_t BR3109_armSpiCmd_orxrx_bw_set(br3109Device_t *device, uint8_t bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((bw<<4)&0x70);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_ORXRX_BW_SET, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**实现rx/orx通道增益设置**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**CHANNEL_t ch：0x1: ch1 0x2:ch2	0x3: ch1&ch2
**uint32_t powerda：设置对应通道的功率值控制字 	DB = 20*log10((257-powerda)/257)
*/
uint32_t BR3109_armSpiCmd_orx_gain_set(br3109Device_t *device, CHANNEL_t ch, uint32_t powerda)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[2]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((ch << 2) & 0xF);
	cmdbuf[1] = powerda;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_ORXCH_GAIN_SET, cmdbuf, 2);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**tx通道使能**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t tx_ch：0x00—all disable，0x01—ch1，0x02—ch2，0x03—ch1&ch2
*/
uint32_t BR3109_armSpiCmd_tx_ch_en(br3109Device_t *device, uint8_t tx_ch)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (tx_ch&0x3);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_CH_EN, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**tx通道ATT设置**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t tx_ch：0x00—all disable，0x01—ch1，0x02—ch2，0x03—ch1&ch2
**uint32_t powerda：功率值控制字 DB = 20*log10((257-powerda)/257)
*/
uint32_t BR3109_armSpiCmd_tx_ch_ATT_set(br3109Device_t *device, uint8_t tx_ch, uint32_t powerda)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[2]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (tx_ch&0xF);
	cmdbuf[1] = powerda;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_CH_ATT_SET, cmdbuf, 2);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**tx带宽选择**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t tx_ch：0x00—all disable，0x01—ch1，0x02—ch2，0x03—ch1&ch2
**uint8_t bw：0—窄带200Mhz/245.76Msps，1—宽带450Mhz/491.52Msps
*/
uint32_t BR3109_armSpiCmd_tx_bw_set(br3109Device_t *device, br3109TxChannels_t tx_ch, uint8_t bw)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (tx_ch&0x3) | ((bw<<4)&0x70);
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_BW_SET, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**设置 tx   NCO**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t nco_ch：0x00—nco disable，0x01—nco cha，0x02—nco chb，0x03—nco cha&chb
**int32_t freq1_khz：第一个频率值，单位KHz
**int32_t freq2_khz：第二个频率值，单位KHz
*/
uint32_t BR3109_armSpiCmd_tx_NCO_set(br3109Device_t *device, uint8_t nco_ch, uint8_t tx_ch, int32_t freq1_khz, int32_t freq2_khz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[3]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = (nco_ch&0x3) | ((tx_ch<<4)&0x30);
	cmdbuf[1] = freq1_khz;
	cmdbuf[2] = freq2_khz;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_NCO_SET, cmdbuf, 3);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**设置 tx   NCO 功率**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**uint8_t tx_ch：0x00—all disable，0x01—ch1，0x02—ch2，0x03—ch1&ch2
**int32_t powerda1：第一个功率值
**int32_t powerda2：第二个功率值
*/
uint32_t BR3109_armSpiCmd_tx_ncopower_set(br3109Device_t *device, uint8_t tx_ch, int32_t powerda1, int32_t powerda2)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[3]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = ((tx_ch<<4)&0x30);
	cmdbuf[1] = powerda1&0xFFFF;
	cmdbuf[2] = powerda2&0xFFFF;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_NCOPOWER_SET, cmdbuf, 3);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}

/**设置 tx lo ripple**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**CHANNEL_t tx_ch：0x01—ch1，0x02—ch2，0x03—ch1&ch2
**int32_t *wrDataBuf：指针变量，要写入到指定地址的32bit数据数组首地址
**uint32_t wrWordSz：32，为32个整型数据，32个平整度系数(对应32个校准频点)乘以10000后的整数值
*/
uint32_t BR3109_armSpiCmd_tx_signal_ripple_param_set(br3109Device_t *device, CHANNEL_t ch, int32_t *wrDataBuf, uint32_t wrWordSz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
    brHalErr_t halError = BRHAL_OK;
	uint32_t cmdstatword;
    uint32_t readDSz = 0;
    uint16_t curBatchSz = 0;
	uint32_t cmdbuf[2]={0};
    cmdbuf[0] = BR3109_ADDR_ARM_EXT_CMD_WORD_1;//0x3f08
    cmdbuf[1] = ch&0x3;//0x3f0c

    if(wrWordSz != RIPPLE_WRITE_LEN){
		return BRHAL_ERR;
    }

    while(readDSz<wrWordSz){
        curBatchSz = min(TALAPI_ARM_CPY_MEM_MAX,wrWordSz-readDSz);
		retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	    IF_ERR_RETURN_U32(retVal);
		halError = brSpiWriteWordsBlock(device->devHalInfo, BR3109_ADDR_ARM_EXT_CMD_WORD_1, (uint32_t *)&wrDataBuf[readDSz], curBatchSz);
        retVal = talApiErrHandler(device,TAL_ERRHDL_HAL_SPI, halError, retVal, TALACT_ERR_RESET_SPI);
        IF_ERR_RETURN_U32(retVal);
		retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TX_LO_RIPPLE_SET|TALAPI_ARMSPI_OPCODE_CMDSIZE(3), cmdbuf, 2);
	    IF_ERR_RETURN_U32(retVal);
		retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
	    IF_ERR_RETURN_U32(retVal);
        readDSz = readDSz + curBatchSz;
    }
	return retVal;
}

/**测试函数**/
/*param:
**br3109Device_t *device：br3109Device_t的指针，使用此结构体解析
**int32_t freq_khz：频率，单位KHz
*/
uint32_t BR3109_armSpiCmd_test_func(br3109Device_t *device, int32_t freq_khz)
{
	talRecoveryActions_t retVal = TALACT_NO_ACTION;
	uint32_t cmdstatword;
	uint32_t cmdbuf[1]={0};
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 10000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
    cmdbuf[0] = freq_khz;
	retVal= BR3109_sendArmCommand(device, TALAPI_ARMSPI_CMD_TEST_FUNC, cmdbuf, 1);
    IF_ERR_RETURN_U32(retVal);
	retVal = BR3109_waitArmCmdStatus(device, 0, &cmdstatword, 100000000,WAITINITCALS_INTERVAL_US);
    IF_ERR_RETURN_U32(retVal);
	return retVal;
}


