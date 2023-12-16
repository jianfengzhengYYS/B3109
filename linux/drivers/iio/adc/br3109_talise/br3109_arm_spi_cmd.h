
#ifndef BR3109_ARM_SPI_CMD_H_
#define BR3109_ARM_SPI_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "br3109_reg_addr_macros.h"

#define min(a,b)		(((a) < (b))?(a):(b))


#define TALAPI_ARMSPI_OPCODE_CMDSIZE_POS		24
#define TALAPI_ARMSPI_OPCODE_CMDSIZE(Sz)		((Sz)<<TALAPI_ARMSPI_OPCODE_CMDSIZE_POS)
#define TALAPI_ARM_CPY_MEM_MAX					((0x3F80-BR3109_ADDR_ARM_EXT_CMD_WORD_1))

#define RX_PATH             1
#define RIPPLE_WRITE_LEN    32

typedef enum {
	TALAPI_ARMSPI_BUSY		= 0x00D0C0DEUL,
	TALAPI_ARMSPI_READY		= 0xC0DE00D0UL,
} talApiArmSpiSta;


enum CALI_part_t{
	CALI_PART_ONLY_SWEEP		= 1<<0,
	CALI_PART_QEC_CFIR			= 1<<1,
	CALI_PART_IQCALI			= 1<<2,
	CALI_PART_CPLxFIR			= 1<<3,
	CALI_PART_TX_NCO_SWEEP		= 1<<4,
	CALI_PART_DC_OFFSET			= 1<<5,	
};

typedef enum {	
	TALAPI_ARMSPI_CMD_BLKCPY							= 0x0001,
	TALAPI_ARMSPI_CMD_SPI_BLK_WRITE                     = 0x0002,
	TALAPI_ARMSPI_CMD_SPI_BLK_READ                      = 0x0003,
	TALAPI_ARMSPI_CMD_CLEAR_TX_QFIR                 	= 0x0004,
	TALAPI_ARMSPI_CMD_CLEAR_TX_CFIR                 	= 0x0005,
	TALAPI_ARMSPI_CMD_SETUP_BB_RF_PLL               	= 0x0006,
	TALAPI_ARMSPI_CMD_PURESWEEP_LB_RX1              	= 0x0007,
	TALAPI_ARMSPI_CMD_SETONEFREQ_LB_RX1             	= 0x0008,
	TALAPI_ARMSPI_CMD_CALIBRATIONFUNC_LB_RX1        	= 0x0009,
	TALAPI_ARMSPI_CMD_PURESWEEP_LB_ORX1             	= 0x000A,
	TALAPI_ARMSPI_CMD_SETONEFREQ_LB_ORX1            	= 0x000B,
	TALAPI_ARMSPI_CMD_CALIBRATIONFUNC_LB_ORX1       	= 0x000C,
	TALAPI_ARMSPI_CMD_TX_PURESWEEP                  	= 0x000D,
	TALAPI_ARMSPI_CMD_TX_SETONEFREQ                 	= 0x000E,
	TALAPI_ARMSPI_CMD_TX_CALIBRATIONFUNC            	= 0x000F,
	TALAPI_ARMSPI_CMD_SETUP_TX_LOOPBACK             	= 0x0010,
	TALAPI_ARMSPI_CMD_MEM_MASK_WRITE            	    = 0x0011,
	TALAPI_ARMSPI_CMD_SPI_DEV_MASK_WRITE              	= 0x0012,
	TALAPI_ARMSPI_CMD_SETUP_LOOPBACK_LB_RX1         	= 0x0013,
	TALAPI_ARMSPI_CMD_SETUP_LOOPBACK_LB_ORX1        	= 0x0014,
	TALAPI_ARMSPI_CMD_LO_LEAK_CALI                  	= 0x0015,
	TALAPI_ARMSPI_CMD_O_RX_LO_LEAK_CALI_ADDR4       	= 0x0016,
	TALAPI_ARMSPI_CMD_O_RX_LO_LEAK_CALI_ADDR1C      	= 0x0017,
	SPI_CMD_TRACK_CONFIG				              	= 0x0018,
	TALAPI_ARMSPI_CMD_TUNING_IIP                    	= 0x0019,
	TALAPI_ARMSPI_CMD_TRACK_CALI                 		= 0x001A,
	TALAPI_ARMSPI_CMD_CONFIG_AGC                    	= 0x001B,
	TALAPI_ARMSPI_CMD_TURNOFF_LOOPBACK_LO_ORFPLL    	= 0x001C,
	TALAPI_ARMSPI_CMD_SETRF_FREQ                   		= 0x001D,
	TALAPI_ARMSPI_CMD_RX_DIG_DC_XAL                   	= 0x0020,
	TALAPI_ARMSPI_CMD_INIT_CAL							= 0x0021,
	TALAPI_ARMSPI_CMD_JESD_CONFIG						= 0x0022,
	TALAPI_ARMSPI_CMD_SETRF_FREQ_NEW					= 0x0023,
	TALAPI_ARMSPI_CMD_SYNC_ADC_Q						= 0x0024,
	TALAPI_ARMSPI_CMD_SETORF_FREQ						= 0x0025,
	TALAPI_ARMSPI_CMD_AGC_CALI							= 0x0026,
	TALAPI_ARMSPI_CMD_JESD_STATE_SET					= 0x0027,
    TALAPI_ARMSPI_CMD_CALI_CPLXFIR					    = 0x1001,
	TALAPI_ARMSPI_CMD_PLAYCAP_SET						= 0x1101,
	TALAPI_ARMSPI_CMD_PLAYCAP_MANUAL_START			    = 0x1102,
	TALAPI_ARMSPI_CMD_TDD_MANUAL_MODE_SET				= 0x1200,
	TALAPI_ARMSPI_CMD_ORXRX_CH_EN						= 0x1201,
	TALAPI_ARMSPI_CMD_ORXRX_BW_SET					    = 0x1202,
	TALAPI_ARMSPI_CMD_ORXCH_GAIN_SET					= 0x1203,
	TALAPI_ARMSPI_CMD_TX_CH_EN						    = 0x1204,
	TALAPI_ARMSPI_CMD_TX_CH_ATT_SET					    = 0x1205,
	TALAPI_ARMSPI_CMD_TX_BW_SET						    = 0x1206,
	TALAPI_ARMSPI_CMD_TX_NCO_SET						= 0x1207,
	TALAPI_ARMSPI_CMD_TX_NCOPOWER_SET					= 0x1208,
	TALAPI_ARMSPI_CMD_TX_LO_RIPPLE_SET                  = 0x1209,
	TALAPI_ARMSPI_CMD_TEST_FUNC						    = 0xFFFF,
} talApiArmSpiCMD;

/**
 * \brief Enumerated list of channel
 */
typedef enum{
	CHANNEL_1	= 0x1,
	CHANNEL_2	= 0x2,
	CHANNEL_1_2	= 0x3,
}CHANNEL_t;


typedef enum{
	JESD_TX	= 0x1,
	JESD_RX	= 0x2,
	JESD_TX_RX	= 0x3,
}JESDSerializer_Tpye_t;

//uint32_t BR3109_armSpiCmd_blkcpy(br3109Device_t *device, uint32_t srcAdr, uint32_t dstAdr, uint32_t wSz);
uint32_t BR3109_armMemoryCmd_blk_write(br3109Device_t *device, uint32_t startRegAddr, uint32_t *wrDataBuf, uint32_t  wrWordSz);
uint32_t BR3109_armMemoryCmd_blk_read(br3109Device_t *device, uint32_t startRegAddr, uint32_t *rdDataBuf, uint32_t rdWordSz);
uint32_t BR3109_armSpiCmd_SPI_blk_write(br3109Device_t *device, uint8_t SPI_select, uint16_t regAddr, uint32_t *wrDataBuf, uint32_t  wrWordSz);
uint32_t BR3109_armSpiCmd_SPI_blk_read(br3109Device_t *device, uint8_t SPI_select, uint16_t regAddr, uint32_t *rdDataBuf, uint32_t rdWordSz); 
uint32_t BR3109_armSpiCmd_clear_tx_qfir(br3109Device_t *device);
uint32_t BR3109_armSpiCmd_clear_tx_cfir(br3109Device_t *device);
uint32_t BR3109_armSpiCmd_setup_bb_rf_pll(br3109Device_t *device);
uint32_t BR3109_armSpiCmd_puresweep_lb_rx1(br3109Device_t *device, CHANNEL_t sweep_ch, uint8_t bw, uint32_t check_point);
uint32_t BR3109_armSpiCmd_setonefreq_lb_rx1(br3109Device_t *device, CHANNEL_t freq_ch, uint8_t bw, uint8_t freq_index, uint32_t check_point);
uint32_t BR3109_armSpiCmd_calibrationfunc_lb_rx1(br3109Device_t *device, CHANNEL_t cali_ch, uint8_t cali_bw, uint8_t set_fir_flag);
uint32_t BR3109_armSpiCmd_puresweep_lb_orx1(br3109Device_t *device,  CHANNEL_t sweep_ch, uint8_t bw, uint32_t check_point);
uint32_t BR3109_armSpiCmd_setonefreq_lb_orx1(br3109Device_t *device, CHANNEL_t freq_ch, uint8_t freq_index, uint8_t bw,  uint8_t check_point);
uint32_t BR3109_armSpiCmd_calibrationfunc_lb_orx1(br3109Device_t *device, CHANNEL_t cali_ch, uint8_t cali_bw, uint8_t set_fir_flag);
uint32_t BR3109_armSpiCmd_tx_puresweep(br3109Device_t *device, CHANNEL_t cali_ch, uint8_t bw);
uint32_t BR3109_armSpiCmd_tx_setonefreq(br3109Device_t *device, CHANNEL_t tx_ch, uint8_t freq_bw, uint32_t freqpoint);
uint32_t BR3109_armSpiCmd_tx_calibrationfunc(br3109Device_t *device,  CHANNEL_t cali_ch, uint8_t cali_bw);
uint32_t BR3109_armSpiCmd_setup_tx_loopback(br3109Device_t *device, CHANNEL_t lb_channel, CHANNEL_t rx_orx_ch, uint8_t band, uint8_t lp_type, uint8_t loopmode);
uint32_t BR3109_armSpiCmd_mem_mask_write(br3109Device_t *device, uint32_t val, uint32_t dstAdr, uint32_t mask);
uint32_t BR3109_armSpiCmd_spi_dev_mask_write(br3109Device_t *device, uint32_t mask, uint32_t spi_sel, uint16_t regAddr, uint32_t val);
uint32_t BR3109_armSpiCmd_setup_loopback_lb_rx1(br3109Device_t *device, CHANNEL_t lb_channel, uint8_t bw);
uint32_t BR3109_armSpiCmd_setup_loopback_lb_orx1(br3109Device_t *device, CHANNEL_t lb_channel, uint8_t bw, uint8_t lb_src);
uint32_t BR3109_armSpiCmd_tx_lo_leak_cali(br3109Device_t *device, CHANNEL_t set_channel, uint8_t set_bw);
uint32_t BR3109_armSpiCmd_rx_lo_leak_cali_addr4(br3109Device_t *device, CHANNEL_t set_channel, uint8_t set_bw);
uint32_t BR3109_armSpiCmd_orx_lo_leak_cali_addr1c(br3109Device_t *device, CHANNEL_t set_channel, uint8_t set_bw);
uint32_t BR3109_armSpiCmd_track_config(br3109Device_t *device,uint8_t tx_rx_orx_sel, CHANNEL_t ch, uint8_t bw);
uint32_t BR3109_armSpiCmd_tuning_iip(br3109Device_t *device, uint8_t src, CHANNEL_t set_channel,uint8_t step, uint8_t set_bw, uint32_t freqBin_1, uint32_t freqBin_2, uint32_t freqBin_3, uint32_t freqBin_4);
uint32_t BR3109_armSpiCmd_track_cali_En(br3109Device_t *device, uint32_t enableMask);
uint32_t BR3109_armSpiCmd_config_agc(br3109Device_t *device, CHANNEL_t channel, uint8_t index, uint8_t mode);
uint32_t BR3109_armSpiCmd_turnoff_all(br3109Device_t *device);
uint32_t BR3109_armSpiCmd_rx_dig_dc_xal(br3109Device_t *device, CHANNEL_t set_channel, uint8_t set_bw);
uint32_t BR3109_armSpiCmd_Initical_cali(br3109Device_t *device, CHANNEL_t tx_ch, CHANNEL_t rx_ch, CHANNEL_t orx_ch, CHANNEL_t tx1_lp2orx_ch, CHANNEL_t tx2_lp2orx_ch, uint8_t set_bw, uint8_t external_lp, uint32_t cali_flag);
uint32_t BR3109_armSpiCmd_Jesd_config(br3109Device_t *device, uint8_t jesdtx_rx);
uint32_t BR3109_armSpiCmd_Jesd_state_autorelink(br3109Device_t *device, uint8_t state);
uint32_t BR3109_armSpiCmd_setrf_freq(br3109Device_t *device, uint32_t freq_KHz);
uint32_t BR3109_armSpiCmd_SyncAdc_Q(br3109Device_t *device, uint8_t ch1_sync_q, uint8_t ch2_sync_q);
uint32_t BR3109_armSpiCmd_setorf_freq(br3109Device_t *device, uint32_t freq_Khz);
uint32_t BR3109_armSpiCmd_AGC_CALI(br3109Device_t *device, CHANNEL_t ch, uint8_t bw);
uint32_t BR3109_armSpiCmd_cali_cplxfir(br3109Device_t *device, CHANNEL_t cali_ch, uint8_t cali_bw, uint8_t tx_rx_orxsel);
uint32_t BR3109_armSpiCmd_playcap_set(br3109Device_t *device, uint8_t mode, uint8_t continue_mode, uint8_t start_pluse, uint8_t sel_AB, uint8_t div, CHANNEL_t ch, uint16_t src);
uint32_t BR3109_armSpiCmd_playcap_manual_start(br3109Device_t *device, uint8_t start_ab);
uint32_t BR3109_armSpiCmd_TddManualsel(br3109Device_t *device, uint8_t mode);
uint32_t BR3109_armSpiCmd_orxrx_ch_en(br3109Device_t *device, br3109RxORxChannels_t ch);
uint32_t BR3109_armSpiCmd_orxrx_bw_set(br3109Device_t *device, uint8_t bw);
uint32_t BR3109_armSpiCmd_orx_gain_set(br3109Device_t *device, CHANNEL_t ch, uint32_t powerda);
uint32_t BR3109_armSpiCmd_tx_ch_en(br3109Device_t *device, uint8_t tx_ch);
uint32_t BR3109_armSpiCmd_tx_ch_ATT_set(br3109Device_t *device, uint8_t tx_ch, uint32_t powerda);
uint32_t BR3109_armSpiCmd_tx_bw_set(br3109Device_t *device, br3109TxChannels_t tx_ch, uint8_t bw);
uint32_t BR3109_armSpiCmd_tx_NCO_set(br3109Device_t *device, uint8_t nco_ch, uint8_t tx_ch, int32_t freq1_khz, int32_t freq2_khz);
uint32_t BR3109_armSpiCmd_tx_ncopower_set(br3109Device_t *device, uint8_t tx_ch, int32_t powerda1, int32_t powerda2);
uint32_t BR3109_armSpiCmd_tx_signal_ripple_param_set(br3109Device_t *device, CHANNEL_t tx_ch, int32_t *wrDataBuf, uint32_t wrWordSz);







#ifdef __cplusplus
}
#endif

#endif


