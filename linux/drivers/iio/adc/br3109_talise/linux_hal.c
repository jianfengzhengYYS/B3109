#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#include "linux_hal.h"


brHalErr_t BRHAL_writeToLog(void *devHalInfo, brLogLevel_t logLevel, uint32_t errorCode, const char *comment)
{
	struct br3109_hal *devHalData = (struct br3109_hal *)devHalInfo;

	if(devHalInfo == NULL)
	{
		return (BRHAL_GEN_SW);
	}

	if((devHalData->log_level & BRHAL_LOG_ERR) && (logLevel == BRHAL_LOG_ERR))
	{
		dev_err(&devHalData->spi->dev, "ERROR: %d: %s", (int)errorCode, comment);
	}
	else if((devHalData->log_level & BRHAL_LOG_WARN) && (logLevel == BRHAL_LOG_WARN))
	{
		dev_warn(&devHalData->spi->dev, "WARNING: %d: %s", (int)errorCode, comment);
	}
	else if((devHalData->log_level & BRHAL_LOG_MSG) && (logLevel == BRHAL_LOG_MSG))
	{
		dev_info(&devHalData->spi->dev, "MESSAGE: %d: %s",(int)errorCode, comment);
	}
	else
	{
		dev_warn(&devHalData->spi->dev, "Undefined Log Level: 0x%X: %d: %s", logLevel, (int)errorCode, comment);
	}

	return BRHAL_OK;
}

brHalErr_t BRHAL_setTimeout(void *devHalInfo, uint32_t halTimeout_ms)
{
	return BRHAL_OK;
}

brHalErr_t BRHAL_openHw(void *devHalInfo, uint32_t halTimeout_ms)
{
	return BRHAL_OK;
}

brHalErr_t BRHAL_closeHw(void *devHalInfo)
{
	return BRHAL_OK;
}

brHalErr_t BRHAL_resetHw(void *devHalInfo)
{
	struct br3109_hal *devHalData = (struct br3109_hal *)devHalInfo;

	if(devHalInfo == NULL)
	{
		return (BRHAL_GEN_SW);
	}

	dev_err(&devHalData->spi->dev, "BRHAL_resetHw at index");

	gpiod_set_value(devHalData->reset_gpio, 0);
	mdelay(1);
	gpiod_set_value(devHalData->reset_gpio, 1);

	return BRHAL_OK;
}

brHalErr_t BRHAL_spiWriteWord(void *devHalInfo,	uint32_t addr, uint32_t data)
{
#define RW_ADDR_OFFSET	1
    struct br_hal *devHalData = (struct br_hal *)devHalInfo;
    uint8_t buf[4+8];
	int ret = 0;

	if (devHalData->log_level & BRHAL_LOG_SPI)
	{
		dev_err(&devHalData->spi->dev, "SPIWrite: ADDR:0x%03X, DATA:0x%02X \n", addr, data);
	}

    buf[0] = (addr >> (24+RW_ADDR_OFFSET)) & 0x7F; //Issue write command
    buf[1] = (addr >> (16+RW_ADDR_OFFSET)) & 0xFF;
    buf[2] = (addr >> ( 8+RW_ADDR_OFFSET)) & 0xFF;
	buf[3] = (addr >> ( 0+RW_ADDR_OFFSET)) & 0xFE; //must be alighed to 32-bit word boundary
	
	buf[4] = (data >> 24) & 0xFF;
    buf[5] = (data >> 16) & 0xFF;
    buf[6] = (data >>  8) & 0xFF;
	buf[7] = (data      ) & 0xFF; //must be alighed to 32-bit word boundary

	ret = spi_write_then_read(devHalData->spi, buf, 12, NULL, 0);
	if (ret < 0) {
		dev_err(&devHalData->spi->dev, "%s: failed (%d)\n", __func__, ret);
		return(BRHAL_SPI_FAIL);
	}

	return BRHAL_OK;
}

brHalErr_t BRHAL_spiBlockWriteWords(void *devHalInfo,
				 uint32_t addr, uint32_t *data, uint32_t count)
{
    brHalErr_t errVal;
    uint32_t i;
    if (devHalInfo == NULL)
        return (BRHAL_GEN_SW);
    for (i = 0; i < count; i++) {
        errVal = BRHAL_spiWriteWord(devHalInfo, addr, data[i]);
        if (errVal)
            return errVal;
		addr+=4;
    }

    return BRHAL_OK;
}

brHalErr_t BRHAL_spiWriteWords(void *devHalInfo,
				 uint32_t *addr, uint32_t *data, uint32_t count)
{
    brHalErr_t errVal;
    uint32_t i;

	for (i = 0; i < count; i++) {
		errVal = BRHAL_spiWriteWord(devHalInfo, addr[i], data[i]);
		if (errVal)
		return errVal;
	}

    return BRHAL_OK;
}

brHalErr_t BRHAL_spiReadWord(void *devHalInfo, uint32_t addr, uint32_t *readdata)
{
	struct br3109_hal *devHalData = (struct br3109_hal *)devHalInfo;
	uint8_t data[4];
	int32_t retval;
    uint8_t wrbuf[4+8];

	if(devHalInfo == NULL)
	{
		return (BRHAL_GEN_SW);
	}

	wrbuf[0] = 0x80| ((addr >> (24+RW_ADDR_OFFSET)) & 0x7F); //Issue read command
    wrbuf[1] = (addr >> (16+RW_ADDR_OFFSET)) & 0xFF;
    wrbuf[2] = (addr >> ( 8+RW_ADDR_OFFSET)) & 0xFF;
	wrbuf[3] = (addr >> ( 0+RW_ADDR_OFFSET)) & 0xFE; //must be alighed to 32-bit word boundary
	
	retval = spi_write_then_read(devHalData->spi, wrbuf, 8, data, 4);
	if (retval < 0) {
		dev_err(&devHalData->spi->dev, "%s: failed (%d)\n", __func__, retval);;
		return(BRHAL_SPI_FAIL);
	} else {
		*readdata = (data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3];
	}

	if (devHalData->log_level & BRHAL_LOG_SPI)
	{
		dev_err(&devHalData->spi->dev, "SPIRead: ADDR:0x%03X, ReadData:0x%08X\n",
			addr, *readdata);
	}

	return BRHAL_OK;
}

brHalErr_t BRHAL_spiReadWords(void *devHalInfo, uint32_t *addr, uint32_t *readdata, uint32_t count)
{
    brHalErr_t errVal;
    uint32_t i;
    if (devHalInfo == NULL)
        return (BRHAL_GEN_SW);
    for (i = 0; i < count; i++) {
        errVal = BRHAL_spiReadWord(devHalInfo, addr[i], &readdata[i]);
        if (errVal)
            return errVal;
    }

    return BRHAL_OK;
}

brHalErr_t BRHAL_spiBlockReadWords(void *devHalInfo, uint32_t addr, uint32_t *readdata, uint32_t count)
{
    brHalErr_t errVal;
    uint32_t i;
    if (devHalInfo == NULL)
        return (BRHAL_GEN_SW);
    for (i = 0; i < count; i++) {
        errVal = BRHAL_spiReadWord(devHalInfo, addr, &readdata[i]);
        if (errVal)
            return errVal;
		addr+=4;
    }

    return BRHAL_OK;
}
	

brHalErr_t BRHAL_spiWriteField(void *devHalInfo,
				 uint32_t addr, uint32_t fieldVal, uint32_t mask, uint8_t startBit)
{
	struct br3109_hal *devHalData = (struct br3109_hal *)devHalInfo;
	brHalErr_t errval;
	uint32_t readVal;

    errval = BRHAL_spiReadWord(devHalInfo, addr, &readVal);
	if (errval < 0)
		return errval;

    readVal = (readVal & ~mask) | ((fieldVal << startBit) & mask);

	if (devHalData->log_level & BRHAL_LOG_SPI)
	{
		dev_err(&devHalData->spi->dev,"SPIWriteField: ADDR:0x%03X, FIELDVAL:0x%02X, MASK:0x%02X, STARTBIT:%d\n",
			addr, fieldVal, mask, startBit);
	}

	return BRHAL_spiWriteWord(devHalInfo, addr, readVal);
}

brHalErr_t BRHAL_spiReadField(void *devHalInfo, uint32_t addr, uint32_t *fieldVal, uint32_t mask, uint8_t startBit)
{
	struct br3109_hal *devHalData = (struct br3109_hal *)devHalInfo;
	brHalErr_t errval;
	uint32_t readVal;

	errval = BRHAL_spiReadWord(devHalInfo, addr, &readVal);
	if (errval < 0)
		return errval;

	*fieldVal = ((readVal & mask) >> startBit);

	if(devHalData->log_level & BRHAL_LOG_SPI)
	{
		dev_err(&devHalData->spi->dev,"SPIReadField: ADDR:0x%03X, MASK:0x%02X, STARTBIT:%d, FieldVal:0x%02X\n",
			addr, mask, startBit, *fieldVal);
	}

	return BRHAL_OK;
}

brHalErr_t  BRHAL_wait_us(void *devHalInfo, uint32_t time_us)
{
	usleep_range(time_us, time_us + 10);

	return BRHAL_OK;
}

brHalErr_t BRHAL_setLogLevel(void *devHalInfo, uint16_t logLevel)
{
	struct br3109_hal *devHalData = (struct br3109_hal *)devHalInfo;

	if(devHalInfo == NULL)
	{
		return (BRHAL_GEN_SW);
	}

	devHalData->log_level = logLevel;

	return BRHAL_OK;
}

int powandlogcoef[40] = {10592,11220,11885,12589,13335,14125,14962,15848,16788,17782,
18836,19952,21134,22387,23713,25118,26607,28183,29853,31622,33496,35481,
37583,39810,42169,44668,47315,50118,53088,56234,
59566,63095,66834,70794,74989,79432,84139,89125,94406};/*系数*10000了，后续需要除以10000*/
int myintpowbase10(int exponent, int subexponent) 
{
    int result = 1;
    int i;
	int base = 10;

    if (exponent > 0) {
        for (i = 0; i < exponent; i++) {
            result *= base;
        }
    } else if (exponent < 0) {
        for (i = 0; i > exponent; i--) {
            result /= base;
        }
    }
	result =result*powandlogcoef[2*subexponent];
    return result;
}

// int32_t mylog(int32_t x)
// {
//     if (x <= 0) {
//         // 输入无效，返回 NaN 或其他错误处理方式
//         return 0.0;
//     }

//     int32_t result = 0.0;
//     int32_t term = (x - 1) / (x + 1);
//     int32_t power = term;
//     int n = 1;

//     while (n < 100) {
//         result += power / n;
//         power *= term * term;
//         n += 2;
//     }

//     return 2 * result;
// }

int32_t my20log10(int32_t x) 
{
    int32_t result;
	int base = 100000;
	int i = 0; 
	int exponent = 0;
	int subexponent = 0;
	int subx = 0;
	for(exponent = 0; exponent < 10; exponent++){
		if(x/base < 1){
			break;
		}
		base *= 10;
	}
	subx = x % base/(base/100000);
	for(i = 1; i < 40; i++){
		if(subx < powandlogcoef[0]){
			i = 0;
			break;
		}
		if(subx >= powandlogcoef[i-1] && subx < powandlogcoef[i]){
			break;
		}
	}
	subexponent = (i + 1)/2;
	result = 20000 * exponent + subexponent*1000;
    return result;
}
// int64_t factorial(int64_t n) {
//     int64_t result = 1.0;
// 	int i = 1;
//     for (i = 1; i <= n; ++i) {
//         result *= i;
//     }

//     return result;
// }

// double mysin(double x)
// {
//     double result = 0.0;
// 	int n = 0;
//     for (n = 0; n <= 10; ++n) {
//         double coefficient = mypow(-1, n) / factorial(2 * n + 1);
//         result += coefficient * mypow(x, 2 * n + 1);
//     }
//     return result;
// }

// int64_t mycos(int64_t x) 
// {
//     int64_t result = 0;
// 	int64_t coefficient = 0;
// 	int n = 0;
//     for (n = 0; n <= 10; ++n) {
//         coefficient = mypow(-1, n) / factorial(2 * n);
//         result += coefficient * mypow(x, 2 * n);
//     }
//     return result;
// }
