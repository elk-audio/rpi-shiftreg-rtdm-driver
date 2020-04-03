/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Modern Ancient Instruments Networked AB, dba Elk Stockholm
 *
 * Helper library which allows for low level access to the bcm2835 and bcm2838
 * SPI0 peripherals. This library is heavily based on and inspired by the
 * bcm2835 peripheral library by Mike McCauley's found here:
 * https://www.airspayce.com/mikem/bcm2835/
 *
 */
#ifndef BCM2835_PERIPH_H_
#define BCM2835_PERIPH_H_

/*--------  Available Chip select definitions  --------*/
#define BCM283X_SPI_CS0			0
#define BCM283X_SPI_CS1			1
#define BCM283X_SPI_CS2			2
#define BCM283X_SPI_CS_NONE		3

/**
 * @brief Initialize library.
 *
 * @return int 0 on success, error code if not
 */
int bcm283x_periph_init(void);

/**
 * @brief Close library and deinit
 */
void bcm283x_periph_close(void);

/**
 * @brief Initialize SPI0 peripheral
 */
void bcm283x_spi_begin(void);

/**
 * @brief Deinit SPI0 Peripheral and set its pin back to default function.
 *
 */
void bcm283x_spi_end(void);

/**
 * @brief Set the SPI0 clock division value
 *
 * @param divider_val The clock division value from 0 to 32768
 */
void bcm283x_spi_set_clock_div(uint16_t divider_val);

/**
 * @brief Choose which chip select is used.
 *
 * @param chip_sel either 0 or 1 for chip select 0 and 1 respectively
 */
void bcm283x_spi_chip_sel(uint8_t chip_sel);

/**
 * @brief Set the what value the chip select has when it is asserted
 *
 * @param chip_sel The chip select number
 * @param enable_val The value it should have when it asserts
 */
void bcm283x_spi_set_chip_sel_pol(uint8_t chip_sel, uint8_t enable_val);

/**
 * @brief Sets the SPI data mode, ranging from 0 - 3.
 *
 * @param mode The SPI data mode
 */
void bcm283x_spi_set_data_mode(uint8_t mode);

/**
 * @brief Write byte data to the SPI by polling method
 *
 * @param buf The buffer with the data
 * @param len The num of bytes to send
 */
void bcm283x_spi_write_bytes_polling(uint8_t *buf, uint32_t len);

/**
 * @brief Read byte data from the SPI by polling method
 *
 * @param buf The buffer to store the incoming data
 * @param len The num of bytes to store
 */
void bcm283x_spi_read_bytes_polling(uint8_t *buf, uint32_t len);

/**
 * @brief Set a RPI B+ Pin
 *
 * @param pin The pin to set.
 */
void bcm283x_gpio_set(uint32_t pin);

/**
 * @brief Clear a RPi B+ Pin
 *
 * @param pin The pin to clear
 */
void bcm283x_gpio_clr(uint32_t pin);

#endif // BCM2835_PERIPH_H_
