/**
 * \file br3109_reg_addr_macros.h
 * \brief Contains Br3109 API address macro definitions
 *
 * Br3109 API version: 1.0.0.6
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef _BR3109_ADDR_MACROS_H
#define _BR3109_ADDR_MACROS_H

#ifdef __cplusplus
extern "C" {
#endif
#if 1

/**
 * *(uint64_t) 的强制转换是为了解决gcc报警告的问题, 64位系统中指针是64位/8字节的, 直接转换为32位的变量会报错,
 * 添加强制转换(uint32_t)(uint64_t)后,该警告消失,
 * 后续可能需要评估该方法是否有意义, BR3109是不是64位还是32位
*/
#define type_OffSet(type, field) 	((uint32_t)(uint64_t)&(((type*)0)->field))
/******************************************************************************/
/*                         APB addrbase                        				  */
/******************************************************************************/
#define APB_CLK_RST_BASEADDR           			(0x40000000UL)
#define APB_IOCTRL_BASEADDR                     (0x4000C000UL)
#define APB_SPI_M_BASEADDR                      (0x40010000UL)
#define APB_DIG_MUX_BASEADDR                    (0x40080000UL)
#define APB_TX_PATH_TOP_BASEADDR                (0x40084000UL)
#define APB_TX_DGAIN_BASEADDR                   (0x40084400UL)
#define APB_TX_IFCONV_BASEADDR                  (0x40084800UL)
#define APB_TX_CPLXCFIR_CH1_BASEADDR            (0x40084C00UL)
#define APB_TX_CPLXCFIR_CH2_BASEADDR            (0x40085000UL)
#define APB_TX_QEC_QFIR_BASEADDR                (0x40085800UL)
#define APB_TX_PHSCMPS_BASEADDR                 (0x40085C00UL)
#define APB_TX_CFIR_BASEADDR                    (0x40086000UL)
#define APB_TX_QEC_CFIR_CH1_BASEADDR            (0x40086400UL)
#define APB_TX_QEC_CFIR_CH2_BASEADDR            (0x40086800UL)
#define APB_RX_PATH_TOP_BASEADDR                (0x40088000UL)
#define APB_RX_RCFIR_BASEADDR     	            (0x40088400UL)
#define APB_RX_AGC_BASEADDR                     (0x40088800UL)
#define APB_RX_ATTN_LUT_BASEADDR                (0x40088C00UL)
#define APB_RX_DIG_GAIN_1_BASEADDR              (0x40089000UL)
#define APB_RX_DIG_GAIN_2_BASEADDR              (0x40089400UL)
#define APB_RX_FE_LUT_BASEADDR  	            (0x40089800UL)
#define APB_RX_PHSCMPS_BASEADDR                 (0x40089C00UL)
#define APB_RX_QEC_QFIR_BASEADDR                (0x4008A000UL)
#define APB_RX_CPLXCFIR_CH1_BASEADDR            (0x4008A800UL)
#define APB_RX_CPLXCFIR_CH2_BASEADDR            (0x4008AC00UL)
#define APB_RX_DC_CORR_BASEADDR                 (0x4008B000UL)
#define APB_RX_IFCONV_BASEADDR                  (0x4008B400UL)
#define APB_RX_QEC_CFIR_CH1_BASEADDR            (0x4008B800UL)
#define APB_RX_QEC_CFIR_CH2_BASEADDR            (0x4008BC00UL)
#define APB_JESD_TX_BASEADDR                    (0x4008C000UL)
#define APB_JESD_RX_BASEADDR                    (0x4008C400UL)
#define APB_TDD_BASEADDR                        (0x40090000UL)
#define APB_PLAYCAP_BASEADDR                    (0x40094000UL)  


#define HOST_SYSCTRL_BASEADDR                   (0x44000000UL)  
#define HOST_APB_WRITE                          (0x80000000UL)  


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Peripheral and SRAM base address */
#define BR3109_FLASH_BASE        (0x00000000UL)  /*!< (FLASH     ) Base Address */
#define BR3109_SRAM_BASE         (0x00000000UL)  /*!< (SRAM      ) Base Address */

#define BR3109_RAM_BASE          (0x00000000UL)
#define BR3109_ADDR_ARM_GLOBLE_CONFIG_DATA 0x37000
/*MEMory*/
#define DRAM_BASE     			 		(0x00025000UL)
#define BR3109_ADDR_CMD_IT_BASE     			 	  	(0x00003f00UL)
#define BR3109_ADDR_ARM_COMMAND     			 	  	(BR3109_ADDR_CMD_IT_BASE+0x04)
#define BR3109_ADDR_ARM_EXT_CMD_WORD_1     			 	(BR3109_ADDR_CMD_IT_BASE+0x20)

#define BR_SPI_SYSCTRL_ADDR_BASE			0x44000000
#define BR3109_ADDR_HW_REVISION				(BR_SPI_SYSCTRL_ADDR_BASE+0X0000)
#define BR3109_ADDR_REFCLK_FREQ				(BR_SPI_SYSCTRL_ADDR_BASE+0X0008)
#define BR3109_ADDR_ARMCLK_DIV				(BR_SPI_SYSCTRL_ADDR_BASE+0X0014)
#define BR3109_ADDR_ARMCLK_DIV_ACTION		(BR_SPI_SYSCTRL_ADDR_BASE+0X001C)
#define BR3109_ADDR_SPI_INIT_RESET			(BR_SPI_SYSCTRL_ADDR_BASE+0X002C)
#define BR3109_ADDR_WAKE_MCU				(BR_SPI_SYSCTRL_ADDR_BASE+0X0030)
#define BR3109_ADDR_WAKE_MCU_BIT			0


/* **********************************APB peripherals*********************************/   
/******************************************************************************/
/*                        APB_CLK_RST		                           		  */
/******************************************************************************/
#define BR3109_ADDR_APB_CLK_RST_BB_PLL					(APB_CLK_RST_BASEADDR+0X000C)
#define BR3109_ADDR_APB_CLK_RST_RF_PLL					(APB_CLK_RST_BASEADDR+0X0010)
#define BR3109_ADDR_APB_CLK_RST_RF_FCW					(APB_CLK_RST_BASEADDR+0X0014)	
#define BR3109_ADDR_APB_CLK_RST_ORF_PLL					(APB_CLK_RST_BASEADDR+0X0018)
#define BR3109_ADDR_APB_CLK_RST_ORF_FCW					(APB_CLK_RST_BASEADDR+0X001C)	
#define BR3109_ADDR_APB_CLK_RST_JESD_SYNC_DIV_EN		(APB_CLK_RST_BASEADDR+0X0030)
#define BR3109_ADDR_APB_CLK_RST_FUNC_RESET				(APB_CLK_RST_BASEADDR+0X003C)	 
/******************************************************************************/
/*                        TX_PATH_TOP		                           		  */
/******************************************************************************/
#define APB_TX_PATH_TOP_VIL_SEL					(APB_TX_PATH_TOP_BASEADDR+0X0008)	
#define APB_TX_PATH_TOP_PLAY_MODE				(APB_TX_PATH_TOP_BASEADDR+0X0038)	

/******************************************************************************/
/*                        APB_RX_IFCONV		                           		  */
/******************************************************************************/

#define BR3109_ADDR_APB_RX_IFCONV_				(APB_RX_IFCONV_BASEADDR+0X003C)	


/******************************************************************************/
/*                        APB_DIG_MUX		                           		  */
/******************************************************************************/

#define BR3109_ADDR_APB_DIG_MUX_LOOPBACK_FIXDATA						(APB_DIG_MUX_BASEADDR+ 0x0008)
#define BR3109_ADDR_APB_DIG_MUX_DP_TX_IQ_CH1							(APB_DIG_MUX_BASEADDR+ 0x000C)
#define BR3109_ADDR_APB_DIG_MUX_DP_TX_IQ_CH2							(APB_DIG_MUX_BASEADDR+ 0x0010)
#define BR3109_ADDR_APB_DIG_MUX_JESD_TX_IQ_CH1							(APB_DIG_MUX_BASEADDR+ 0x0014)
#define BR3109_ADDR_APB_DIG_MUX_JESD_TX_IQ_CH2							(APB_DIG_MUX_BASEADDR+ 0x001C)




/******************************************************************************/
/*                        JESD		                           				  */
/******************************************************************************/
#define BR3109_ADDR_JESD_FRAMER_IRQ								   (APB_JESD_TX_BASEADDR+0x0008)
#define BR3109_ADDR_JESD_FRAMER_CONFIG_0                           (APB_JESD_TX_BASEADDR+0x0010)
#define BR3109_ADDR_JESD_FRAMER_CONFIG_1                           (APB_JESD_TX_BASEADDR+0x001C)
#define BR3109_ADDR_JESD_FRAMER_CONFIG1_0                          (APB_JESD_TX_BASEADDR+0x0020)
#define BR3109_ADDR_JESD_FRAMER_CONFIG2_0                          (APB_JESD_TX_BASEADDR+0x0024)
#define BR3109_ADDR_JESD_FRAMER_CONFIG3_0                          (APB_JESD_TX_BASEADDR+0x0028)
#define BR3109_ADDR_JESD_FRAMER_CONFIG4_0                          (APB_JESD_TX_BASEADDR+0x002C)
#define BR3109_ADDR_JESD_FRAMER_CONFIG5_0                          (APB_JESD_TX_BASEADDR+0x0030)
#define BR3109_ADDR_JESD_FRAMER_LANE_XBAR	                       (APB_JESD_TX_BASEADDR+0x0048)
//#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR		                   (APB_JESD_TX_BASEADDR+0x004C)
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_0123_0                 (APB_JESD_TX_BASEADDR+0x004C)
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_4567_0                 (APB_JESD_TX_BASEADDR+0x0050)
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_89AB_0                 (APB_JESD_TX_BASEADDR+0x0054)
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_CDEF_0                 (APB_JESD_TX_BASEADDR+0x0058)
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_AUX_XBAR_0123_0             (APB_JESD_TX_BASEADDR+0x008C)
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_AUX_XBAR_4567_0             (APB_JESD_TX_BASEADDR+0x0090)
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_AUX_XBAR_89AB_0             (APB_JESD_TX_BASEADDR+0x0094)
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_AUX_XBAR_CDEF_0             (APB_JESD_TX_BASEADDR+0x0098)
#define BR3109_ADDR_JESD_FRAMER_TEST_CFG_0			               (APB_JESD_TX_BASEADDR+0x00EC)
#define BR3109_ADDR_JESD_FRAMER_LANE_PN_CFG			               (APB_JESD_TX_BASEADDR+0x00F0)
#define BR3109_ADDR_JESD_FRAMER_SYNC				               (APB_JESD_TX_BASEADDR+0x00F4)
#define BR3109_ADDR_JESD_DEFRAMER_CONFIG                           (APB_JESD_RX_BASEADDR+0x0008)
#define BR3109_ADDR_JESD_DEFRAMER_IRQ			                   (APB_JESD_RX_BASEADDR+0x000C)
#define BR3109_ADDR_JESD_DEFRAMER_IRQ_MASK_0                       (APB_JESD_RX_BASEADDR+0x0010)
#define BR3109_ADDR_JESD_DEFRAMER_CONFIG_0                         (APB_JESD_RX_BASEADDR+0x0014)
#define BR3109_ADDR_JESD_DEFRAMER_CONFIG_1                         (APB_JESD_RX_BASEADDR+0x0018)
#define BR3109_ADDR_JESD_DEFRAMER_CONFIG_2                         (APB_JESD_RX_BASEADDR+0x001C)
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS0_0                        (APB_JESD_RX_BASEADDR+0x0024)
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS1_0                        (APB_JESD_RX_BASEADDR+0x0034)
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS2_0                        (APB_JESD_RX_BASEADDR+0x0044)
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS3_0                        (APB_JESD_RX_BASEADDR+0x0054)
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_0L                  	(APB_JESD_RX_BASEADDR+0x0064)
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_0H                  	(APB_JESD_RX_BASEADDR+0x0068)
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_1L                  	(APB_JESD_RX_BASEADDR+0x006C)
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_1H                  	(APB_JESD_RX_BASEADDR+0x0070)
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_2L                  	(APB_JESD_RX_BASEADDR+0x0074)
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_2H                  	(APB_JESD_RX_BASEADDR+0x0078)
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_3L                  	(APB_JESD_RX_BASEADDR+0x007C)
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_XBAR_3H                  	(APB_JESD_RX_BASEADDR+0x0080)
#define BR3109_ADDR_JESD_DEFRAMER_RX_CGS_FRAME_SYNC				 	(APB_JESD_RX_BASEADDR+0x0084)
#define BR3109_ADDR_JESD_DEFRAMER_LANE_PN_CFG			            (APB_JESD_RX_BASEADDR+0x0090)
#define BR3109_ADDR_JESD_DEFRAMER_NOTINTAB_DISERR_PRBS_COUNT_CLEAR 	(APB_JESD_RX_BASEADDR+0x00B4)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_CFG                         	(APB_JESD_RX_BASEADDR+0x00B8)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_COUNT0          				(APB_JESD_RX_BASEADDR+0x00BC)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT0                	(APB_JESD_RX_BASEADDR+0x00C4)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_COUNT1          				(APB_JESD_RX_BASEADDR+0x00CC)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT1                	(APB_JESD_RX_BASEADDR+0x00D4)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_COUNT2          				(APB_JESD_RX_BASEADDR+0x00DC)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT2                	(APB_JESD_RX_BASEADDR+0x00E4)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_COUNT3          				(APB_JESD_RX_BASEADDR+0x00EC)
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT3                	(APB_JESD_RX_BASEADDR+0x00F4)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_CFG                         	(APB_JESD_RX_BASEADDR+0x00FC)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_COUNT_CLEAR                 	(APB_JESD_RX_BASEADDR+0x0100)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_COUNT0          				(APB_JESD_RX_BASEADDR+0x0104)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_ERROR_COUNT0                	(APB_JESD_RX_BASEADDR+0x010C)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_COUNT1          				(APB_JESD_RX_BASEADDR+0x0114)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_ERROR_COUNT1                	(APB_JESD_RX_BASEADDR+0x011C)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_COUNT2          				(APB_JESD_RX_BASEADDR+0x0124)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_ERROR_COUNT2                	(APB_JESD_RX_BASEADDR+0x012C)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_COUNT3          				(APB_JESD_RX_BASEADDR+0x0134)
#define BR3109_ADDR_JESD_DEFRAMER_RAW_PRBS_ERROR_COUNT3                	(APB_JESD_RX_BASEADDR+0x013C)

/******************************************************************************/
/*                        TDD		                           				  */
/******************************************************************************/
#define BR3109_ADDR_TDD_RF_CONFIG0				          				(APB_TDD_BASEADDR+0x0008)
#define BR3109_ADDR_TDD_RF_CONFIG1				          			    (APB_TDD_BASEADDR+0x000c)
#define BR3109_ADDR_TDD_RF_CONFIG2				          				(APB_TDD_BASEADDR+0x0010)
#define BR3109_ADDR_TDD_RF_CONFIG3				          				(APB_TDD_BASEADDR+0x016C)
#define BR3109_ADDR_TDD_RF_CONFIG4				          				(APB_TDD_BASEADDR+0x0170)
#define BR3109_ADDR_TDD_RF_CONFIG5				          				(APB_TDD_BASEADDR+0x0214)


/******************************************************************************/
/*                        AGC		                           				  */
/******************************************************************************/

#define BR3109_ADDR_AGC_CONFIG1                                 	(APB_RX_AGC_BASEADDR+0x08)
#define BR3109_ADDR_AGC_GAIN_UPDATE_COUNTER                       	(APB_RX_AGC_BASEADDR+0x08)
#define BR3109_ADDR_SLOWLOOP_CONFIG                                 (APB_RX_AGC_BASEADDR+0x14)
#define BR3109_ADDR_AGC_UNDERRANGE0                                 (APB_RX_AGC_BASEADDR+0x18)
#define BR3109_ADDR_UPPER_LOWER_LEVEL_BLOCKER_THRESHOLD             (APB_RX_AGC_BASEADDR+0x20)
#define BR3109_ADDR_AGC_GAIN_STEP_DEC_OVERLOAD_CONFIG 				(APB_RX_AGC_BASEADDR+0x24)
#define BR3109_ADDR_DEC_OVERLOAD_UPPER_UNDER_RANGE_LOW_THRESHOLD	(APB_RX_AGC_BASEADDR+0x28)
#define BR3109_ADDR_DEC_UNDERRANGE_THRESHOLD		 				(APB_RX_AGC_BASEADDR+0x2C)
#define BR3109_ADDR_AGC_HB2_THRESH_EXCEEDED_CNT		 				(APB_RX_AGC_BASEADDR+0x30)
#define BR3109_ADDR_AGC_HB2_GAIN_STEP				 				(APB_RX_AGC_BASEADDR+0x34)
#define BR3109_ADDR_AGC_HB2_OVERLOAD_POWER			 				(APB_RX_AGC_BASEADDR+0x38)
#define BR3109_ADDR_AGC_LOCK_LEVEL_THRESHOLDS                      	(APB_RX_AGC_BASEADDR+0x3C)
#define BR3109_ADDR_POWER_THRESHOLDS								(APB_RX_AGC_BASEADDR+0x40)
#define BR3109_ADDR_DEC_POWER										(APB_RX_AGC_BASEADDR+0x44)
#define BR3109_ADDR_RX1_UL_SIG_POWER_MEAS_DURATION_DELAY			(APB_RX_AGC_BASEADDR+0x48)
#define BR3109_ADDR_RX1_UL_SIG_POWER_MEAS_DURATION_DELAY_CNT		(APB_RX_AGC_BASEADDR+0x4C)
#define BR3109_ADDR_RX2_UL_SIG_POWER_MEAS_DURATION_DELAY			(APB_RX_AGC_BASEADDR+0x50)
#define BR3109_ADDR_RX2_UL_SIG_POWER_MEAS_DURATION_DELAY_CNT		(APB_RX_AGC_BASEADDR+0x54)
#define BR3109_ADDR_AGC_POWER_OVER_RANGE_HIGH_THRESHOLD				(APB_RX_AGC_BASEADDR+0x58)
#define BR3109_ADDR_AGC_POWER_OVER_RANGE_LOW_THRESHOLD				(APB_RX_AGC_BASEADDR+0x5C)
#define BR3109_ADDR_AGC_POWER_UPPER_LOW_SHIFT_CONFIG				(APB_RX_AGC_BASEADDR+0x60)

#define BR3109_ADDR_AGC_MANUAL_GAIN_SET_FSM_CNT_RESET_RX1_RX2      	(APB_RX_AGC_BASEADDR+0x64)
#define BR3109_ADDR_MANUAL_GAIN_INDEX_DELAY_RX1                     (APB_RX_AGC_BASEADDR+0x70)
#define BR3109_ADDR_MAX_MIN_GAIN_INDEX_RX1                          (APB_RX_AGC_BASEADDR+0x7C)
#define BR3109_ADDR_DEC_POWER_CH1                                   (APB_RX_AGC_BASEADDR+0xD8)
#define BR3109_ADDR_DEC_POWER_CH2                                   (APB_RX_AGC_BASEADDR+0xDC)
#define BR3109_ADDR_MANUAL_GAIN_INDEX_DELAY_RX2                     (APB_RX_AGC_BASEADDR+0x8C)
#define BR3109_ADDR_MANUAL_GAIN_INDEX_ORX1_ORX2                    	(APB_RX_AGC_BASEADDR+0xB0)
#define BR3109_ADDR_MAX_MIN_GAIN_INDEX_RX2                          (APB_RX_AGC_BASEADDR+0x98)
#define BR3109_ADDR_RX1_RX2_GAIN_TAB_IDX                            (APB_RX_AGC_BASEADDR+0xD0)


/******************************************************************************/
/*                                FIR                              			  */
/******************************************************************************/

#define BR3109_ADDR_TX_CH1_CFIR_RX_COEF_START    (APB_TX_CFIR_BASEADDR+0x14) 
#define BR3109_ADDR_TX_CH1_CFIR_RX_COEF_VLD      (APB_TX_CFIR_BASEADDR+0x8) 
#define BR3109_ADDR_TX_CH2_CFIR_ORX_COEF_START   (APB_TX_CFIR_BASEADDR+0x94) 
#define BR3109_ADDR_TX_CH2_CFIR_ORX_COEF_VLD     (APB_TX_CFIR_BASEADDR+0x8) 
    
#define BR3109_ADDR_RX_CFIR_RX_COEF_START   (APB_RX_RCFIR_BASEADDR+0x14) 
#define BR3109_ADDR_RX_CFIR_RX_COEF_VLD     (APB_RX_RCFIR_BASEADDR+0x8) 

#define BR3109_ADDR_RX_CFIR_ORX_COEF_START  (APB_RX_RCFIR_BASEADDR+0x94) 
#define BR3109_ADDR_RX_CFIR_ORX_COEF_VLD    (APB_RX_RCFIR_BASEADDR+0x8) 


#define BR3109_ADDR_ORX_COEF_VLD_MASK           0xFF000000
#define BR3109_ADDR_ORX_COEF_VLD_OFFSET         24
#define BR3109_ADDR_RX_COEF_VLD_MASK            0xFF0000
#define BR3109_ADDR_RX_COEF_VLD_OFFSET          16

/******************************************************************************/
/*                                RX GAIN TABLE                               */
/******************************************************************************/
#define BR3109_ADDR_AGC_RF_ATTN_FE_ATTN_IDX_0       (APB_RX_ATTN_LUT_BASEADDR+0x80)
#define BR3109_ADDR_AGC_RF_ATTN_EXT_RSA_IDX_0       (APB_RX_ATTN_LUT_BASEADDR)
#define BR3109_ADDR_AGC_DIG_LUT_RX1_IDX_0           (APB_RX_DIG_GAIN_1_BASEADDR)
#define BR3109_ADDR_AGC_DIG_LUT_RX2_IDX_0           (APB_RX_DIG_GAIN_2_BASEADDR)

#define BR3109_ADDR_AGC_DIG_LUT_DIG_COMP_Q_OFFSET   16

/******************************************************************************/
/*                                RX DC OFFSRT                                */
/******************************************************************************/

#define BR3109_ADDR_DC_CORR_MAN_MODE_SETTING    (APB_RX_DC_CORR_BASEADDR+0x8)

#define BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH1_EN_MASK       1
#define BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH1_EN_OFFSET     0

#define BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH2_EN_MASK       1<<1
#define BR3109_ADDR_DC_CORR_MAN_MODE_RX_CH2_EN_OFFSET     1

#define BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH1_EN_MASK      1<<2
#define BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH1_EN_OFFSET    2

#define BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH2_EN_MASK      1<<3
#define BR3109_ADDR_DC_CORR_MAN_MODE_ORX_CH2_EN_OFFSET    3


#define BR3109_ADDR_DC_CORR_MAN_RX_CH1_IQ       (APB_RX_DC_CORR_BASEADDR+0x14)
#define BR3109_ADDR_DC_CORR_MAN_ORX_CH1_IQ      (APB_RX_DC_CORR_BASEADDR+0x18)
#define BR3109_ADDR_DC_CORR_MAN_RX_CH2_IQ       (APB_RX_DC_CORR_BASEADDR+0x20)
#define BR3109_ADDR_DC_CORR_MAN_ORX_CH2_IQ      (APB_RX_DC_CORR_BASEADDR+0x24)


/******************************************************************************/
/*          SPI              tansmitter lane 		               			  */
/******************************************************************************/

#define BR3109_ADDR_TX1_ATTENUATION	                              (0x0058)
#define BR3109_ADDR_TX2_ATTENUATION	                              (0x0058)
//#define BR3109_ADDR_TX1_GAIN_0                                     0x0E87
//#define BR3109_ADDR_TX1_GAIN_1                                     0x0E88
//#define BR3109_ADDR_TX1_GAIN_2                                     0x0E89
//#define BR3109_ADDR_TX2_GAIN_0                                     0x0E8A
//#define BR3109_ADDR_TX2_GAIN_1                                     0x0E8B
//#define BR3109_ADDR_TX2_GAIN_2                                     0x0E8C
//#define BR3109_ADDR_TX_INCR_DECR_WORD                              0x0E8D
//#define BR3109_ADDR_TX_TPC_CONFIG                                  0x0E8E
//#define BR3109_ADDR_TDD_RAMP_TX1                                   0x0E92
//#define BR3109_ADDR_TDD_RAMP_TX2                                   0x0E93



/******************************************************************************/
/*                         SPI id                            				  */
/******************************************************************************/

#define SPI_JESD_RX_ID							    0
#define SPI_JESD_TX_ID                 				1
#define SPI_ADC_I_L1_ID                             2
#define SPI_ADC_Q_L1_ID                             3
#define SPI_ADC_I_L0_ID                             4
#define SPI_ADC_Q_L0_ID                             5
#define SPI_DAC_IQ_L1_ID                            6
#define SPI_DAC_IQ_L0_ID                            8
#define SPI_TRANSMITTER_L1_ID                       10
#define SPI_TRANSMITTER_L0_ID                       11
#define SPI_RX_ORX_L1_ID                            12
#define SPI_RX_ORX_L0_ID                            13
#define SPI_ORFPLL_ID                               14
#define SPI_RFPLL_ID                                15
#define SPI_BBPLL_ID                                16
#define SPI_AUX_ADDA_IVREF_ID                       17
#define SPI_LO_TXALLWORK_REFPATH_ID                 18
#define SPI_ID_NOUSED				                0x1F






#else
#define BR3109_ADDR_SPI_INTERFACE_CONFIG_A                         0x0000
#define BR3109_ADDR_SPI_INTERFACE_CONFIG_B                         0x0001
#define BR3109_ADDR_PRODUCT_ID_0                                   0x0004
#define BR3109_ADDR_PRODUCT_ID_1                                   0x0005
#define BR3109_ADDR_SCRATCH_PAD                                    0x000A
#define BR3109_ADDR_VENDOR_ID_0                                    0x000C
#define BR3109_ADDR_VENDOR_ID_1                                    0x000D
#define BR3109_ADDR_GPIO_SLEW_CONTROL_BYTE3                        0x0045
#define BR3109_ADDR_DIGITAL_IO_CONTROL                             0x0048
#define BR3109_ADDR_TX_RX_DATAPATH_INVERSION                       0x004A
#define BR3109_ADDR_SPI2_CONFIGURATION                             0x004B
#define BR3109_ADDR_SPI2_CONFIGURATION2                            0x004C
#define BR3109_ADDR_SYSREF_PAD_CONFIG                              0x0080
#define BR3109_ADDR_TX1_SYNC_PAD_CONFIG                            0x0081
#define BR3109_ADDR_TX2_SYNC_PAD_CONFIG                            0x0082
#define BR3109_ADDR_RX1_SYNC_PAD_CONFIG                            0x0083
#define BR3109_ADDR_RX2_SYNC_PAD_CONFIG                            0x0084
#define BR3109_ADDR_PFIR_COEFF_CTL                                 0x00C0
#define BR3109_ADDR_PFIR_COEFF_DATA                                0x00C2
#define BR3109_ADDR_PFIR_COEFF_ADDR                                0x00C3
#define BR3109_ADDR_CONFIGURATION_CONTROL_1                        0x0100
#define BR3109_ADDR_CONFIGURATION_CONTROL_2                        0x0101
#define BR3109_ADDR_CONFIGURATION_CONTROL_4                        0x0102
#define BR3109_ADDR_CONFIGURATION_CONTROL_ORX                      0x0103
#define BR3109_ADDR_CLOCK_CONTROL_0                                0x0140
#define BR3109_ADDR_CLOCK_CONTROL_1                                0x0141
#define BR3109_ADDR_CLOCK_CONTROL_2                                0x0142
#define BR3109_ADDR_CLOCK_CONTROL_3                                0x0143
#define BR3109_ADDR_CLOCK_CONTROL_4                                0x0144
#define BR3109_ADDR_CLOCK_CONTROL_5                                0x0145
#define BR3109_ADDR_CLOCK_CONTROL_6                                0x0146
#define BR3109_ADDR_CLOCK_CONTROL_7                                0x0147
#define BR3109_ADDR_MCS_CONTROL                                    0x0180
#define BR3109_ADDR_MCS_STATUS                                     0x0181
#define BR3109_ADDR_MCS_CONTROL_2                                  0x0182
#define BR3109_ADDR_CLK_SYNTH_BYTE1                                0x0201
#define BR3109_ADDR_CLK_SYNTH_BYTE2                                0x0202
#define BR3109_ADDR_CLK_SYNTH_BYTE3                                0x0203
#define BR3109_ADDR_CLK_SYNTH_BYTE5                                0x0205
#define BR3109_ADDR_CLK_SYNTH_BYTE6                                0x0206
#define BR3109_ADDR_CLK_SYNTH_BYTE7                                0x0207
#define BR3109_ADDR_CLK_SYNTH_BYTE9                                0x0209
#define BR3109_ADDR_CLK_SYNTH_VCO_VAR_CTR1                         0x020E
#define BR3109_ADDR_CLK_SYNTH_VCO_VAR_CTR2                         0x020F
#define BR3109_ADDR_CLK_SYNTH_DIVIDER_INT_BYTE0                    0x0210
#define BR3109_ADDR_CLK_SYNTH_DIVIDER_INT_BYTE1                    0x0211
#define BR3109_ADDR_CLK_SYNTH_DIVIDER_FRAC_BYTE0                   0x0212
#define BR3109_ADDR_CLK_SYNTH_DIVIDER_FRAC_BYTE1                   0x0213
#define BR3109_ADDR_CLK_SYNTH_DIVIDER_FRAC_BYTE2                   0x0214
#define BR3109_ADDR_CLK_SYNTH_F_VCOTN_BYTE1                        0x0217
#define BR3109_ADDR_CLK_SYNTH_LF_R3                                0x0218
#define BR3109_ADDR_CLK_SYNTH_CAL                                  0x0219
#define BR3109_ADDR_CLK_SYNTH_VCO_CAL_REF                          0x021A
#define BR3109_ADDR_CLK_SYNTH_VCO_BAND_BYTE1                       0x021C
#define BR3109_ADDR_CLK_SYNTH_CAL_CONTROL                          0x021E
#define BR3109_ADDR_RX_NCO_CONTROL_0                               0x0240
#define BR3109_ADDR_RX_NCO_CH1_FTW_BYTE4                           0x0241
#define BR3109_ADDR_RX_NCO_CH1_FTW_BYTE3                           0x0242
#define BR3109_ADDR_RX_NCO_CH1_FTW_BYTE2                           0x0243
#define BR3109_ADDR_RX_NCO_CH1_FTW_BYTE1                           0x0244
#define BR3109_ADDR_RX_NCO_CH2_FTW_BYTE4                           0x0247
#define BR3109_ADDR_RX_NCO_CH2_FTW_BYTE3                           0x0248
#define BR3109_ADDR_RX_NCO_CH2_FTW_BYTE2                           0x0249
#define BR3109_ADDR_RX_NCO_CH2_FTW_BYTE1                           0x024A
#define BR3109_ADDR_RX1_BAND_A_NCO2_FTW_BYTE4                      0x024E
#define BR3109_ADDR_RX1_BAND_A_NCO2_FTW_BYTE3                      0x024F
#define BR3109_ADDR_RX1_BAND_A_NCO2_FTW_BYTE2                      0x0250
#define BR3109_ADDR_RX1_BAND_A_NCO2_FTW_BYTE1                      0x0251
#define BR3109_ADDR_RX1_BAND_B_NCO1_FTW_BYTE4                      0x0254
#define BR3109_ADDR_RX1_BAND_B_NCO1_FTW_BYTE3                      0x0255
#define BR3109_ADDR_RX1_BAND_B_NCO1_FTW_BYTE2                      0x0256
#define BR3109_ADDR_RX1_BAND_B_NCO1_FTW_BYTE1                      0x0257
#define BR3109_ADDR_RX1_BAND_B_NCO2_FTW_BYTE4                      0x025A
#define BR3109_ADDR_RX1_BAND_B_NCO2_FTW_BYTE3                      0x025B
#define BR3109_ADDR_RX1_BAND_B_NCO2_FTW_BYTE2                      0x025C
#define BR3109_ADDR_RX1_BAND_B_NCO2_FTW_BYTE1                      0x025D
#define BR3109_ADDR_RX2_BAND_A_NCO2_FTW_BYTE4                      0x0260
#define BR3109_ADDR_RX2_BAND_A_NCO2_FTW_BYTE3                      0x0261
#define BR3109_ADDR_RX2_BAND_A_NCO2_FTW_BYTE2                      0x0262
#define BR3109_ADDR_RX2_BAND_A_NCO2_FTW_BYTE1                      0x0263
#define BR3109_ADDR_RX2_BAND_B_NCO1_FTW_BYTE4                      0x0266
#define BR3109_ADDR_RX2_BAND_B_NCO1_FTW_BYTE3                      0x0267
#define BR3109_ADDR_RX2_BAND_B_NCO1_FTW_BYTE2                      0x0268
#define BR3109_ADDR_RX2_BAND_B_NCO1_FTW_BYTE1                      0x0269
#define BR3109_ADDR_RX2_BAND_B_NCO2_FTW_BYTE4                      0x026C
#define BR3109_ADDR_RX2_BAND_B_NCO2_FTW_BYTE3                      0x026D
#define BR3109_ADDR_RX2_BAND_B_NCO2_FTW_BYTE2                      0x026E
#define BR3109_ADDR_RX2_BAND_B_NCO2_FTW_BYTE1                      0x026F
#define BR3109_ADDR_RX_BAND_CONTROL                                0x0272
#define BR3109_ADDR_RX_NCO_FTW_UPDATE_CONTROL                      0x027C
#define BR3109_ADDR_BBIC_ENABLES                                   0x0284
#define BR3109_ADDR_RADIO_CONTROL_CONFIG1                          0x028A
#define BR3109_ADDR_RADIO_CONTROL_CONFIG2                          0x028B
#define BR3109_ADDR_TX_DP_MASK_CH1                                 0x028E
#define BR3109_ADDR_TX_DP_MASK_CH2                                 0x028F
#define BR3109_ADDR_REFERENCE_CLOCK_CYCLES                         0x02C0
#define BR3109_ADDR_RF_SYNTHESIZER_POWER_DOWN                      0x0301
#define BR3109_ADDR_CLK_SYNTHESIZER_POWER_DOWN                     0x0302
#define BR3109_ADDR_TX1_PD                                         0x0309
#define BR3109_ADDR_TX2_PD                                         0x030A
#define BR3109_ADDR_MASTERBIAS_CONFIG0                             0x0340
#define BR3109_ADDR_MASTERBIAS_CONFIG1                             0x0341
#define BR3109_ADDR_RCAL_CONTROL                                   0x0384
#define BR3109_ADDR_REF_PAD_CONFIG2                                0x03C1
#define BR3109_ADDR_RF_SYNTH_CAL                                   0x0414
#define BR3109_ADDR_RF_SYNTH_VCO_BAND_BYTE1                        0x0417
#define BR3109_ADDR_EXT_LO_BYTE0                                   0x04C0
#define BR3109_ADDR_EXTLOGEN_BYTE1                                 0x04C1
#define BR3109_ADDR_EXTLOGEN_CONTROL_1                             0x04C2
#define BR3109_ADDR_EXTLO_DIVMODE                                  0x04C3
#define BR3109_ADDR_RX_PFIR_MODE_MAPPING1                          0x0540
#define BR3109_ADDR_RX_PFIR_MODE_MAPPING2                          0x0541
#define BR3109_ADDR_RX_PFIR_BANK_A_SETTINGS                        0x0542
#define BR3109_ADDR_RX_PFIR_BANK_B_SETTINGS                        0x0543
#define BR3109_ADDR_RX_PFIR_BANK_C_SETTINGS                        0x0544
#define BR3109_ADDR_AGC_CONFIG1                                    0x05C0
#define BR3109_ADDR_AGC_CONFIG2                                    0x05C1
#define BR3109_ADDR_MANUAL_GAIN_CONFIG                             0x05C2
#define BR3109_ADDR_AGC_LOCK_LEVEL                                 0x05C3
#define BR3109_ADDR_AGC_OVRG_GAIN_STEP1                            0x05C4
#define BR3109_ADDR_AGC_OVRG_GAIN_STEP2                            0x05C5
#define BR3109_ADDR_AGC_OVRG_GAIN_STEP4                            0x05C7
#define BR3109_ADDR_AGC_OVRG_GAIN_STEP5                            0x05C8
#define BR3109_ADDR_UPPER_LEVEL_BLOCKER_THRESHOLD                  0x05C9
#define BR3109_ADDR_UPPER_LEVEL_BLOCKER_THRESHOLD2                 0x05CA
#define BR3109_ADDR_LOWER_LEVEL_BLOCKER_THRESHOLD                  0x05CB
#define BR3109_ADDR_LOWER_LEVEL_BLOCKER_THRESHOLD2                 0x05CC
#define BR3109_ADDR_AGC_CONFIG_RX1                                 0x05D0
#define BR3109_ADDR_AGC_MANUAL_GAIN_CONTROL_CONFIG_RX1             0x05D1
#define BR3109_ADDR_MANUAL_GAIN_INDEX_RX1                          0x05D2
#define BR3109_ADDR_MANUAL_GAIN_INDEX_ORX_RX1                      0x05D3
#define BR3109_ADDR_AGC_ATTACK_DELAY_RX1                           0x05D6
#define BR3109_ADDR_MAX_GAIN_INDEX_RX1                             0x05D7
#define BR3109_ADDR_MIN_GAIN_INDEX_RX1                             0x05D8
#define BR3109_ADDR_AGC_CONFIG_RX2                                 0x05E0
#define BR3109_ADDR_AGC_MANUAL_GAIN_CONTROL_CONFIG_RX2             0x05E1
#define BR3109_ADDR_MANUAL_GAIN_INDEX_RX2                          0x05E2
#define BR3109_ADDR_MANUAL_GAIN_INDEX_ORX_RX2                      0x05E3
#define BR3109_ADDR_AGC_ATTACK_DELAY_RX2                           0x05E6
#define BR3109_ADDR_MAX_GAIN_INDEX_RX2                             0x05E7
#define BR3109_ADDR_MIN_GAIN_INDEX_RX2                             0x05E8
#define BR3109_ADDR_GAIN_VALID_OVERRIDES                           0x05E9
#define BR3109_ADDR_AGC_LOCK_LEVEL_THRESHOLDS                      0x0640
#define BR3109_ADDR_ULB_COUNT_THRESHOLD                            0x0641
#define BR3109_ADDR_LLB_COUNT_THRESHOLD                            0x0642
#define BR3109_ADDR_ADC_HIGH_OVRG_COUNT_THRESHOLD                  0x0643
#define BR3109_ADDR_ADC_LOW_OVRG_COUNT_THRESHOLD                   0x0644
#define BR3109_ADDR_UPPER0_THRESHOLD_GAIN_STEP                     0x0646
#define BR3109_ADDR_UPPER1_THRESHOLD_GAIN_STEP                     0x0648
#define BR3109_ADDR_LOWER0_THRESHOLD_GAIN_STEP                     0x0647
#define BR3109_ADDR_LOWER1_THRESHOLD_GAIN_STEP                     0x0649
#define BR3109_ADDR_GAIN_UPDATE_COUNTER1                           0x064A
#define BR3109_ADDR_GAIN_UPDATE_COUNTER2                           0x064B
#define BR3109_ADDR_GAIN_UPDATE_COUNTER3                           0x064C
#define BR3109_ADDR_SLOWLOOP_CONFIG                                0x064D
#define BR3109_ADDR_SLOWLOOP_SETTLING_DELAY                        0x064E
#define BR3109_ADDR_POWER_THRESHOLDS                               0x064F
#define BR3109_ADDR_RX1_UL_SIG_POWER_MEAS_DELAY1                   0x0650
#define BR3109_ADDR_RX1_UL_SIG_POWER_MEAS_DELAY2                   0x0651
#define BR3109_ADDR_RX1_UL_SIG_POWER_MEAS_DURATION1                0x0652
#define BR3109_ADDR_RX1_UL_SIG_POWER_MEAS_DURATION2                0x0653
#define BR3109_ADDR_RX2_UL_SIG_POWER_MEAS_DELAY1                   0x0654
#define BR3109_ADDR_RX2_UL_SIG_POWER_MEAS_DELAY2                   0x0655
#define BR3109_ADDR_RX2_UL_SIG_POWER_MEAS_DURATION1                0x0656
#define BR3109_ADDR_RX2_UL_SIG_POWER_MEAS_DURATION2                0x0657
#define BR3109_ADDR_AGC_UNDERRANGE0_0                              0x0658
#define BR3109_ADDR_AGC_UNDERRANGE0_1                              0x0659
#define BR3109_ADDR_AGC_UNDERRANGE1                                0x065A
#define BR3109_ADDR_AGC_UNDERRANGE2                                0x065B
#define BR3109_ADDR_AGC_IP3_OVERRANGE_THRESHOLD                    0x065C
#define BR3109_ADDR_AGC_ADCOVRG_IP3_HIGH_COUNTER                   0x065D
#define BR3109_ADDR_AGC_ADCOVRG_LOW_INT0_COUNTER                   0X065E
#define BR3109_ADDR_AGC_ADCOVRG_LOW_INT1_COUNTER                   0X065F
#define BR3109_ADDR_AGC_OVRG_LOW_INT0_GAIN_STEP                    0x0660
#define BR3109_ADDR_AGC_OVRG_LOW_INT1_GAIN_STEP                    0x0661
#define BR3109_ADDR_SLOWLOOP_CONFIG2                               0x0662
#define BR3109_ADDR_RX1_AGC_DUALBAND_INDEX_X                       0x0663
#define BR3109_ADDR_RX1_AGC_DUALBAND_INDEX_Y                       0x0664
#define BR3109_ADDR_RX1_SLOWLOOP_CONFIG                            0x0665
#define BR3109_ADDR_RX2_AGC_DUALBAND_INDEX_X                       0x0666
#define BR3109_ADDR_RX2_AGC_DUALBAND_INDEX_Y                       0x0667
#define BR3109_ADDR_RX2_SLOWLOOP_CONFIG                            0x0668
#define BR3109_ADDR_AGC_DUALBAND_TOTAL_PWR_MARGIN                  0x066A
#define BR3109_ADDR_AGC_DUALBAND_BAND_PWR_MARGIN                   0x066B
#define BR3109_ADDR_AGC_UPPER_POWER_THRESHOLD                      0x066C
#define BR3109_ADDR_AGC_DUALBAND_HIGH_LNA_THRESHOLD                0x066D
#define BR3109_ADDR_AGC_DUALBAND_LOW_LNA_THRESHOLD                 0x066E
#define BR3109_ADDR_RX1_GAIN_INDEX                                 0x0686
#define BR3109_ADDR_ORX1_GAIN_INDEX                                0x0687
#define BR3109_ADDR_RX2_GAIN_INDEX                                 0x0688
#define BR3109_ADDR_ORX2_GAIN_INDEX                                0x0689
#define BR3109_ADDR_DEC_POWER_CONFIG_1                             0x070B
#define BR3109_ADDR_DEC_POWER_DURATION                             0x070C
#define BR3109_ADDR_DDC_DEC_POWER_CONFIG                           0x0720
#define BR3109_ADDR_DDC_DEC_POWER_MEAS                             0x0721
#define BR3109_ADDR_DIGITAL_GAIN_CONFIG                            0x0780
#define BR3109_ADDR_GAIN_COMPENATION_AND_SLICER_CONFIG             0x0781
#define BR3109_ADDR_RX1_SLICER_GPIO_CONFIG                         0x0782
#define BR3109_ADDR_RX2_SLICER_GPIO_CONFIG                         0x0783
#define BR3109_ADDR_RXDP_SLICER_READBACK                           0x0784
#define BR3109_ADDR_GAIN_COMPENSATION_AND_SLICER_CONFIG2           0x0785
#define BR3109_ADDR_GAIN_TABLE_WORD_ADDRESS                        0x07C0
#define BR3109_ADDR_GAIN_TABLE_WORD_DATA_RXFE                      0x07C1
#define BR3109_ADDR_GAIN_TABLE_WORD_DATA_EXT                       0x07C2
#define BR3109_ADDR_GAIN_TABLE_WORD_DATA_DIG                       0x07C3
#define BR3109_ADDR_GAIN_TABLE_WORD_DATA_DIG2                      0x07C4
#define BR3109_ADDR_GAIN_TABLE_WORD_DATA_DUALBAND                  0x07C5
#define BR3109_ADDR_GAIN_TABLE_WORD_DATA_PHASE_OFFSET              0x07C6
#define BR3109_ADDR_GAIN_TABLE_WORD_DATA_PHASE_OFFSET2             0x07C7
#define BR3109_ADDR_GAIN_TABLE_DATA_OUTPUT_DUALBAND_CH1_BAND_A     0x07CD
#define BR3109_ADDR_GAIN_TABLE_DATA_OUTPUT_DUALBAND_CH1_BAND_B     0x07CE
#define BR3109_ADDR_GAIN_TABLE_DATA_OUTPUT_DUALBAND_CH2_BAND_A     0x07D6
#define BR3109_ADDR_GAIN_TABLE_DATA_OUTPUT_DUALBAND_CH2_BAND_B     0x07D7
#define BR3109_ADDR_GAIN_TABLE_CONFIGURATION                       0x07DA
#define BR3109_ADDR_RXFE1_LOCM                                     0x0800
#define BR3109_ADDR_RXFE2_LOCM                                     0x0801
#define BR3109_ADDR_LO_MUX_CONFIG                                  0x0880
#define BR3109_ADDR_DEC_OVERLOAD_CONFIG1                           0x0943
#define BR3109_ADDR_DEC_OVERLOAD_CONFIG2                           0x0944
#define BR3109_ADDR_DEC_OVERLOAD_UPPER_THRESHOLD                   0x0945
#define BR3109_ADDR_DEC_OVERLOAD_LOWER_THRESHOLD                   0x0946
#define BR3109_ADDR_DEC_UNDERRANGE_INTERVAL0_THRESHOLD             0x0947
#define BR3109_ADDR_DEC_UNDERRANGE_INTERVAL1_THRESHOLD             0x0948
#define BR3109_ADDR_DEC_IP3_OVERRANGE_THRESHOLD                    0x0949
#define BR3109_ADDR_ADC_BIAS_FLASH_VISB                            0x09C4
#define BR3109_ADDR_DIGITAL_DC_OFFSET_SHIFT                        0x0B01
#define BR3109_ADDR_DIGITAL_DC_OFFSET_ORX_SHIFT                    0x0B43
#define BR3109_ADDR_DIGITAL_DC_OFFSET_CONFIG                       0x0B02
#define BR3109_DIGITAL_DC_OFFSET_ORX_LOOPBACK_CONFIG               0x0B40
#define BR3109_ADDR_TX_FILTER_CONFIGURATION                        0x0D45
#define BR3109_ADDR_TX1_ATTENUATION_0_READBACK                     0x0E00
#define BR3109_ADDR_TX1_ATTENUATION_1_READBACK                     0x0E01
#define BR3109_ADDR_TX2_ATTENUATION_0_READBACK                     0x0E03
#define BR3109_ADDR_TX2_ATTENUATION_1_READBACK                     0x0E04
#define BR3109_ADDR_TX_TPC_GPIO_CONFIG                             0x0E96
#define BR3109_ADDR_TXDAC1_GAIN_I                                  0x0EC6
#define BR3109_ADDR_TXDAC1_GAIN_Q                                  0x0EC7
#define BR3109_ADDR_TXDAC2_GAIN_I                                  0x0EC8
#define BR3109_ADDR_TXDAC2_GAIN_Q                                  0x0EC9
#define BR3109_ADDR_TX_ABBF_FREQCAL_NCO_I_UPPER_NIBBLE             0x0F1B
#define BR3109_ADDR_TX_ABBF_FREQCAL_NCO_I_MSBS                     0x0F1C
#define BR3109_ADDR_TX_ABBF_FREQCAL_NCO_I_LSBS                     0x0F1D
#define BR3109_ADDR_TX_ABBF_FREQCAL_NCO_Q_UPPER_NIBBLE             0x0F1E
#define BR3109_ADDR_TX_ABBF_FREQCAL_NCO_Q_MSBS                     0x0F1F
#define BR3109_ADDR_TX_ABBF_FREQCAL_NCO_Q_LSBS                     0x0F20
#define BR3109_ADDR_GPIO_3P3V_DIRECTION_CONTROL_0                  0x1080
#define BR3109_ADDR_GPIO_3P3V_DIRECTION_CONTROL_1                  0x1081
#define BR3109_ADDR_GPIO_3P3V_SPI_SOURCE_0                         0x1082
#define BR3109_ADDR_GPIO_3P3V_SPI_SOURCE_1                         0x1083
#define BR3109_ADDR_GPIO_3P3V_SPI_READ_0                           0x1084
#define BR3109_ADDR_GPIO_3P3V_SPI_READ_1                           0x1085
#define BR3109_ADDR_GPIO_3P3V_LOWER_BYTE_SOURCE_CONTROL            0x1086
#define BR3109_ADDR_GPIO_3P3V_UPPER_BYTE_SOURCE_CONTROL            0x1087
#define BR3109_ADDR_GPIO_DIRECTION_CONTROL_7DOWNTO0                0x10C0
#define BR3109_ADDR_GPIO_DIRECTION_CONTROL_15DOWNTO8               0x10C1
#define BR3109_ADDR_GPIO_DIRECTION_CONTROL_18DOWNTO16              0x10C2
#define BR3109_ADDR_GPIO_SPI_SOURCE_7DOWNTO0                       0x10C3
#define BR3109_ADDR_GPIO_SPI_SOURCE_15DOWNTO8                      0x10C4
#define BR3109_ADDR_GPIO_SPI_SOURCE_18DOWNTO16                     0x10C5
#define BR3109_ADDR_GPIO_SPI_READ_7DOWNTO0                         0x10C6
#define BR3109_ADDR_GPIO_SPI_READ_15DOWNTO8                        0x10C7
#define BR3109_ADDR_GPIO_SPI_READ_18DOWNTO16                       0x10C8
#define BR3109_ADDR_GPIO_LOWER_BYTE_SOURCE_CONTROL                 0x10C9
#define BR3109_ADDR_GPIO_UPPER_BYTE_SOURCE_CONTROL                 0x10CA
#define BR3109_ADDR_GPIO_EXTRA_BITS_SOURCE_CONTROL                 0x10CB
#define BR3109_ADDR_MONITOR_0                                      0x1140
#define BR3109_ADDR_MONITOR_1                                      0x1141
#define BR3109_ADDR_PDAUXDAC_MANUAL_IN_MSB                         0x11A5
#define BR3109_ADDR_PDAUXDAC_MANUAL_IN_LSB                         0x11A6
#define BR3109_ADDR_AUX_DAC_0_WORD_MSB                             0x11D0
#define BR3109_ADDR_AUX_DAC_0_WORD_LSB                             0x11D1
#define BR3109_ADDR_AUX_DAC_12B_0_MSB                              0x11E4
#define BR3109_ADDR_AUX_DAC_12B_0_LSB                              0x11E5
#define BR3109_ADDR_AUX_DAC_LATCH_CONTROL                          0x11E8
#define BR3109_ADDR_DIGITAL_TEST_BYTE                              0x1382
#define BR3109_ADDR_ARM_CTL_1                                      0x13C0
#define BR3109_ADDR_ARM_CLOCK_CONTROL                              0x13C8
#define BR3109_ADDR_AHB_SPI_BRIDGE                                 0x13C9
#define BR3109_ADDR_ARM_BOOT_ADDR_BYTE0                            0x13CA
#define BR3109_ADDR_ARM_BOOT_ADDR_BYTE1                            0x13CB
#define BR3109_ADDR_ARM_BOOT_ADDR_BYTE2                            0x13CC
#define BR3109_ADDR_ARM_BOOT_ADDR_BYTE3                            0x13CD
#define BR3109_ADDR_ARM_STACK_PTR_BYTE_0                           0x13CE
#define BR3109_ADDR_ARM_STACK_PTR_BYTE_1                           0x13CF
#define BR3109_ADDR_ARM_STACK_PTR_BYTE_2                           0x13D0
#define BR3109_ADDR_ARM_STACK_PTR_BYTE_3                           0x13D1
#define BR3109_ADDR_BRIDGE_CLOCK_CONTROL                           0x13D5
#define BR3109_ADDR_ARM_ECC_DATA_READBACK                          0x13D9
#define BR3109_ADDR_ARM_ECC_PROG_READBACK                          0x13E1
#define BR3109_ADDR_ARM_DMA_CTL                                    0x13EE
#define BR3109_ADDR_ARM_DMA_ADDR0                                  0x13EF
#define BR3109_ADDR_ARM_DMA_ADDR1                                  0x13F0
#define BR3109_ADDR_ARM_DMA_DATA0                                  0x13F3
#define BR3109_ADDR_ARM_DMA_DATA3                                  0x13F6
#define BR3109_ADDR_ARM_COMMAND                                    0x1400
#define BR3109_ADDR_ARM_EXT_CMD_BYTE_1                             0x1401
#define BR3109_ADDR_ARM_CMD_STATUS_0                               0x1408
#define BR3109_ADDR_ARM_CMD_STATUS_8                               0x1410
#define BR3109_ADDR_ARM_CMD_STATUS_11                              0x1413
#define BR3109_ADDR_ARM_CMD_STATUS_12                              0x1414
#define BR3109_ADDR_STREAM_CONTROL                                 0x14C0
#define BR3109_ADDR_STREAM_BASE_BYTE0                              0x14C1
#define BR3109_ADDR_STREAM_BASE_BYTE1                              0x14C2
#define BR3109_ADDR_LAST_STREAM_NUM                                0x14C4
#define BR3109_ADDR_TX_RX_EN_READBACK                              0x14D6
#define BR3109_ADDR_STREAM_PROC_DEBUG_REG0                         0x14F5
#define BR3109_ADDR_STREAM_PROC_DEBUG_REG1                         0x14F6
#define BR3109_ADDR_STREAM_PROC_DEBUG_REG2                         0x14F7
#define BR3109_ADDR_STREAM_PROC_DEBUG_REG3                         0x14F8
#define BR3109_ADDR_STREAM_PROC_DEBUG_REG4                         0x14F9
#define BR3109_ADDR_STREAM_PROC_DEBUG_REG5                         0x14FA
#define BR3109_ADDR_STREAM_PROC_DEBUG_REG6                         0x14FB
#define BR3109_ADDR_STREAM_PROC_DEBUG_REG7                         0x14FC
#define BR3109_ADDR_STREAM_GPIO_TRIGGER_PIN_SELECT_BYTE1           0x14FE
#define BR3109_ADDR_JESD_FRAMER_L0_CFG0_0                          0x1520
#define BR3109_ADDR_JESD_FRAMER_L0_CFG1_0                          0x1521
#define BR3109_ADDR_JESD_FRAMER_L0_CFG2_0                          0x1522
#define BR3109_ADDR_JESD_FRAMER_L0_CFG3_0                          0x1523
#define BR3109_ADDR_JESD_FRAMER_L0_CFG4_0                          0x1524
#define BR3109_ADDR_JESD_FRAMER_L0_CFG5_0                          0x1525
#define BR3109_ADDR_JESD_FRAMER_L0_CFG6_0                          0x1526
#define BR3109_ADDR_JESD_FRAMER_L0_CFG7_0                          0x1527
#define BR3109_ADDR_JESD_FRAMER_L0_CFG8_0                          0x1528
#define BR3109_ADDR_JESD_FRAMER_L0_CFG9_0                          0x1529
#define BR3109_ADDR_JESD_FRAMER_L0_CFG10_0                         0x152A
#define BR3109_ADDR_JESD_FRAMER_ILAS_MF_COUNT_0                    0x152B
#define BR3109_ADDR_JESD_FRAMER_ASYNC_PTR_DBG_0                    0x152C
#define BR3109_ADDR_JESD_FRAMER_LANE_XBAR_0                        0x152E
#define BR3109_ADDR_JESD_FRAMER_CFG_0                              0x152F
#define BR3109_ADDR_JESD_FRAMER_CFG2_0                             0x1530
#define BR3109_ADDR_JESD_FRAMER_CFG3_0                             0x1531
#define BR3109_ADDR_JESD_FRAMER_CFG4_0                             0x1532
#define BR3109_ADDR_JESD_FRAMER_CFG5_0                             0x1533
#define BR3109_ADDR_JESD_FRAMER_TEST_CFG_0                         0x1534
#define BR3109_ADDR_JESD_FRAMER_SYSREF_CFG_0                       0x1536
#define BR3109_ADDR_JESD_FRAMER_SYSREF_CFG2_0                      0x1537
#define BR3109_ADDR_JESD_FRAMER_SYSREF_CFG3_0                      0x1538
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_01_0                   0x1540
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_23_0                   0x1541
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_45_0                   0x1542
#define BR3109_ADDR_JESD_FRAMER_SAMPLE_XBAR_67_0                   0x1543
#define BR3109_ADDR_JESD_FRAMER_ILAS_MF_COUNT_1                    0x1553
#define BR3109_ADDR_JESD_FRAMER_ASYNC_PTR_DBG_1                    0x1554
#define BR3109_ADDR_JESD_FRAMER_SRST_CFG                           0x1570
#define BR3109_ADDR_JESD_FRAMER_COMMON_CFG                         0x1579
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG0_0                        0x1580
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG1_0                        0x1581
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG2_0                        0x1582
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG3_0                        0x1583
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG4_0                        0x1584
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG5_0                        0x1585
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG6_0                        0x1586
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG7_0                        0x1587
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG8_0                        0x1588
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG9_0                        0x1589
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG10_0                       0x158A
#define BR3109_ADDR_JESD_DEFRAMER_ILAS_MF_COUNT_0                  0x158B
#define BR3109_ADDR_JESD_DEFRAMER_EVENT_OBS_0                      0x158C
#define BR3109_ADDR_JESD_DEFRAMER_LANE_XBAR_0                      0x158D
#define BR3109_ADDR_JESD_DEFRAMER_CFG_0                            0x158F
#define BR3109_ADDR_JESD_DEFRAMER_CFG2_0                           0x1590
#define BR3109_ADDR_JESD_DEFRAMER_CFG3_0                           0x1591
#define BR3109_ADDR_JESD_DEFRAMER_CFG4_0                           0x1592
#define BR3109_ADDR_JESD_DEFRAMER_SYSREF_CFG_0                     0x1593
#define BR3109_ADDR_JESD_DEFRAMER_SYSREF_CFG2_0                    0x1594
#define BR3109_ADDR_JESD_DEFRAMER_PTR_DBG_DET_LAT_BUF_0            0x1596
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG2_0                        0x1598
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG3_0                        0x1599
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG4_0                        0x159A
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG5_0                        0x159B
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG6_0                        0x159C
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS0_0                        0x15A0
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS13_0                       0x15AD
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS14_0                       0x15AE
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS15_0                       0x15AF
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS20_0                       0x15B4
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS21_0                       0x15B5
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS22_0                       0x15B6
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS23_0                       0x15B7
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS24_0                       0x15B8
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS25_0                       0x15B9
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS35_0                       0x15C3
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS36_0                       0x15C4
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS37_0                       0x15C5
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS38_0                       0x15C6
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS39_0                       0x15C7
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS40_0                       0x15C8
#define BR3109_ADDR_JESD_DEFRAMER_L0_CFG0_1                        0x15D0
#define BR3109_ADDR_JESD_DEFRAMER_CFG_1                            0x15DF
#define BR3109_ADDR_JESD_DEFRAMER_SYSREF_CFG2_1                    0x15E4
#define BR3109_ADDR_JESD_DEFRAMER_PTR_DBG_DET_LAT_BUF_1            0x15E6
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG2_1                        0x15E8
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG3_1                        0x15E9
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG5_1                        0x15EB
#define BR3109_ADDR_JESD_DEFRAMER_IP_CFG6_1                        0x15EC
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS0_1                        0x15F0
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS13_1                       0x15FD
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS14_1                       0x15FE
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS15_1                       0x15FF
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS20_1                       0x1604
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS21_1                       0x1605
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS22_1                       0x1606
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS23_1                       0x1607
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS24_1                       0x1608
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS25_1                       0x1609
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS35_1                       0x1613
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS36_1                       0x1614
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS37_1                       0x1615
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS38_1                       0x1616
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS39_1                       0x1617
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS40_1                       0x1618
#define BR3109_ADDR_JESD_DEFRAMER_IP_OBS42_1                       0x161A
#define BR3109_ADDR_JESD_DEFRAMER_CH1_SAMPLE_XBAR                  0x1620
#define BR3109_ADDR_JESD_DEFRAMER_CH2_SAMPLE_XBAR                  0x1621
#define BR3109_ADDR_JESD_DEFRAMER_COMMON_CFG                       0x1623
#define BR3109_ADDR_JESD_DEFRAMER_COMMON_CFG3                      0x1625
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_DISABLE_CH1               0x1628
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_DISABLE_CH2               0x1629
#define BR3109_ADDR_JESD_DEFRAMER_PTR_DBG_LANE0                    0x162A
#define BR3109_ADDR_JESD_DEFRAMER_PTR_DBG_LANE1                    0x162B
#define BR3109_ADDR_JESD_DEFRAMER_PTR_DBG_LANE2                    0x162C
#define BR3109_ADDR_JESD_DEFRAMER_PTR_DBG_LANE3                    0x162D
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_CFG                         0x162E
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_FLAG                  0x162F
#define BR3109_ADDR_JESD_DEFRAMER_SAMPLE_PRBS_ERROR_COUNT          0x1630
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT0                0x1631
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT1                0x1632
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT2                0x1633
#define BR3109_ADDR_JESD_DEFRAMER_PRBS_ERROR_COUNT3                0x1634
#define BR3109_ADDR_PA_PROTECTION_CONFIGURATION                    0x1660
#define BR3109_ADDR_PA_PROTECTION_ATTEN_CONTROL                    0x1661
#define BR3109_ADDR_PA_PROTECTION_THRESHOLD_0                      0x1662
#define BR3109_ADDR_PA_PROTECTION_THRESHOLD_1                      0x1663
#define BR3109_ADDR_PA_PROTECTION_THRESHOLD_2                      0x1664
#define BR3109_ADDR_PA_PROTECTION_THRESHOLD_3                      0x1665
#define BR3109_ADDR_PA_PROTECTION_PEAK_COUNT                       0x1666
#define BR3109_ADDR_PA_PROTECTION_PEAK_THRESHOLD_CH1               0x1667
#define BR3109_ADDR_PA_PROTECTION_PEAK_THRESHOLD_CH2               0x1668
#define BR3109_ADDR_PA_PROTECTION_POWER_0                          0x1669
#define BR3109_ADDR_PA_PROTECTION_POWER_1                          0x166A
#define BR3109_ADDR_PA_PROTECTION_ERROR                            0x166B
#define BR3109_ADDR_FLOATING_POINT_CONFIG                          0x16C0
#define BR3109_ADDR_FLOATING_POINT_CONTROL                         0x16C1
#define BR3109_ADDR_FLOATING_POINT_ENABLE                          0x16C2
#define BR3109_ADDR_INTEGER_FORMAT_CONFIG                          0x16C3
#define BR3109_ADDR_GP_INTERRUPT_MASK_1                            0x1700
#define BR3109_ADDR_GP_INTERRUPT_MASK_0                            0x1701
#define BR3109_ADDR_GP_INTERRUPT_READ_1                            0x1702
#define BR3109_ADDR_GP_INTERRUPT_READ_0                            0x1703
#define BR3109_ADDR_AUX_SYNTH_VCO_BAND_BYTE1                       0x1792
#define BR3109_ADDR_AUX_SYNTH_CAL                                  0x1798
#define BR3109_ADDR_SER_PHY_TXCTRL_0                               0x1850
#define BR3109_ADDR_SER_PHY_TXCTRL_1                               0x1851
#define BR3109_ADDR_SER_PHY_PD                                     0x1853
#define BR3109_ADDR_SER_PHY_ENABLES                                0x1854
#define BR3109_ADDR_SER_PREEMPH_CTRL                               0x1855
#define BR3109_ADDR_DES_PHY_PWR_DWN_0                              0x1869
#define BR3109_ADDR_DES_PHY_PWR_DWN_4                              0x186D
#define BR3109_ADDR_DES_PHY_CLK_CTL_0                              0x1878
#define BR3109_ADDR_DES_PHY_CLK_CTL_1                              0x1879
#define BR3109_ADDR_DES_PHY_GENERAL_CTL_0                          0x187D
#define BR3109_ADDR_DES_PHY_GENERAL_CTL_1                          0x187E
#define BR3109_ADDR_DES_PHY_EQ_CONTROL2                            0x18B0
#define BR3109_ADDR_PHM_CNTRL                                      0x18C0 /* C0 Only */
#define BR3109_ADDR_PHM_PERIOD_BYTE0                               0x18C1 /* C0 Only */
#define BR3109_ADDR_PHM_PERIOD_BYTE1                               0x18C2 /* C0 Only */
#define BR3109_ADDR_PHM_PERIOD_BYTE2                               0x18C3 /* C0 Only */
#define BR3109_ADDR_FOVR_CNTRL                                     0x18CC /* C0 Only */
#define BR3109_ADDR_FOVR_RX_UP_THRESH_BYTE0                        0x18CD /* C0 Only */
#define BR3109_ADDR_FOVR_RX_UP_THRESH_BYTE1                        0x18CE /* C0 Only */
#define BR3109_ADDR_FOVR_RX_LOW_THRESH_BYTE0                       0x18CF /* C0 Only */
#define BR3109_ADDR_FOVR_RX_LOW_THRESH_BYTE1                       0x18D0 /* C0 Only */
#define BR3109_ADDR_FOVR_RX_DWELL_THRESH_BYTE0                     0x18D1 /* C0 Only */
#define BR3109_ADDR_FOVR_RX_DWELL_THRESH_BYTE1                     0x18D2 /* C0 Only */
#define BR3109_ADDR_PHM_FOVR_ON_JESD                               0x18D3 /* C0 Only */
#define BR3109_ADDR_PCA_CNTRL                                      0x18D4 /* C0 Only */
#define BR3109_ADDR_PCA_RX1_GAIN_INDEX_OFFSET                      0x18D5 /* C0 Only */
#define BR3109_ADDR_PCA_RX2_GAIN_INDEX_OFFSET                      0x18D6 /* C0 Only */
#define BR3109_ADDR_PCA_RX1_GAIN_STEP                              0x18D7 /* C0 Only */
#define BR3109_ADDR_PCA_RX2_GAIN_STEP                              0x18D8 /* C0 Only */
#define BR3109_ADDR_SCRATCH_PAD_UPPER_ADDRESS_SPACE                0x3EE0
#define BR3109_ADDR_SCRATCH_PAD_READ_ONLY_UPPER_ADDRESS_SPACE      0x3EE1
#endif
#ifdef __cplusplus
}
#endif

#endif
