/**
 * \file br3109_config.h
 * \brief Br3109 configuration external declarations
 *
 * \brief Contains structure definitions for tal_config.c
 *
 * Copyright 2022 briradio..
 * Released under the BR3109 API license, for more information see the "LICENSE.txt" file in this zip file.
 *
 * The top level structure br3109Device_t talDevice uses keyword
 * extern to allow the application layer main() to have visibility
 * to these settings.
 */

#ifndef TAL_INIT_H_
#define TAL_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TX_BW_SWTICH_FREQ(ref_freq_hz)   (ref_freq_hz > 81250000 ? (ref_freq_hz > 200000000 ? (ref_freq_hz) : (ref_freq_hz*2)) : (ref_freq_hz*4))
extern br3109Init_t talInit;

#ifdef __cplusplus
}
#endif

#endif
