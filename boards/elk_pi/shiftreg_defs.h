// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Modern Ancient Instruments Networked AB, dba Elk Stockholm
 *
 * Defitions which describe input shiftregisters SN74HC165,output shift
 * registers SN74HC595, adc ADS7041 and the hardware configuration and layout of
 * the ElkPi hat for the raspberry pi.
 *
 *
 * The schematic for the ElkPi Hat can be found here:
 * https://github.com/elk-audio/elk-pi-hardware/blob/master/ElkPi_schematics.pdf
 *
 * Data sheets for the shift registers and adc:
 * http://www.ti.com/lit/ds/symlink/sn74hc595.pdf
 * http://www.ti.com/lit/ds/symlink/sn74hc165.pdf
 * http://www.ti.com/lit/ds/symlink/ads7041.pdf
 */
#ifndef SHIFTREG_DEFS_H_
#define SHIFTREG_DEFS_H_

#define SHIFTREG_NUM_PINS_PER_REG	8

#define SHIFTREG_NUM_OUTPUT_SHIFTREG	4
#define SHIFTREG_NUM_OUTPUT_PINS	32
#define SHIFTREG_LOAD_OUT_GPIO_NUM	5
#define SHIFTREG_ENABLE_LOAD_OUT_VAL	0
#define SHIFTREG_DISABLE_LOAD_OUT_VAL	1
#define SHIFTREG_LOAD_OUT_PERIOD_NS	200

#define SHIFTREG_NUM_INPUT_SHIFTREG	4
#define SHIFTREG_NUM_INPUT_PINS		32
#define SHIFTREG_LOAD_IN_GPIO_NUM	6
#define SHIFTREG_ENABLE_LOAD_IN_VAL	0
#define SHIFTREG_DISABLE_LOAD_IN_VAL	1
#define SHIFTREG_INPUT_SHIFTREG_CS	0
#define SHIFTREG_LOAD_IN_PERIOD_NS	200

#define SHIFTREG_NUM_ADC_MUX_SELECTS	4
#define SHIFTREG_NUM_ADC_PINS		16
#define SHIFTREG_ADC_LOAD_GPIO_NUM	13
#define SHIFTREG_ENABLE_ADC_LOAD_VAL	0
#define SHIFTREG_DISABLE_ADC_LOAD_VAL	1
#define SHIFTREG_ADC_CS			1
#define SHIFTREG_ADC_LOAD_PERIOD_NS	200
#define SHIFTREG_ADC_ACQ_PERIOD_NS	30000
#define SHIFTREG_ADC_SPI_LEN_BYTES	2
#define SHIFTREG_ADC_RES		10
#define SHIFTREG_ADC_CHANS_PER_TICK	4

#define SHIFTREG_SPI_DATA_MODE		0
#define SHIFTREG_SPI_CLOCK_DIV		32 // 7.8125MHz according to bcm283x
#define SHIFTREG_SPI_CS_ON_VAL		0

#endif //SHIFTREG_DEFS_H_
