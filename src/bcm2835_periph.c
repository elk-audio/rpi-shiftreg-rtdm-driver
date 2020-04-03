// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Modern Ancient Instruments Networked AB, dba Elk Stockholm
 *
 * Helper library which allows for low level access to the bcm2835 and bcm2838
 * SPI0 peripherals. This library is heavily based on and inspired by the
 * bcm2835 peripheral library by Mike McCauley's found here:
 * https://www.airspayce.com/mikem/bcm2835/
 */
#include <linux/of.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/mman.h>
#include <asm/io.h>
#include <asm/barrier.h>

#include "bcm2835_periph.h"

/*--------  Common definitions  --------*/
#define BCM283X_GPIO_OFFSET		0x200000
#define BCM283X_SPI0_OFFSET		0x204000
#define BCM283X_GPFSEL0_OFFSET		0x0000
#define BCM283X_GPSET0_OFFSET		0x001c
#define BCM283X_GPCLR0_OFFSET		0x0028

/*--------  Gpio function select masks  --------*/
#define BCM283X_GPIO_FSEL_MASK		0x07
#define BCM283X_GPIO_FSEL_ALT0		0x04
#define BCM283X_GPIO_FSEL_INPT		0x00

/*--------  BCM283X Spi reg offsets  --------*/
#define BCM283X_SPI0_CS			0x0000 // SPI Master Control and Status
#define BCM283X_SPI0_FIFO		0x0004 // Fifo for TX and RX
#define BCM283X_SPI0_CLK		0x0008 // Clock divider reg

/*--------  BCM283X SPI Reg Masks  --------*/
#define BCM283X_SPI0_CLEAR_MASK		0x00000030 // clear rx, tx fifos
#define BCM283X_SPI0_CS_MASK		0x00000003 // select chip select
#define BCM283X_SPI0_TA_MASK		0x00000080 // set tranfer active flag
#define BCM283X_SPI0_TXD_MASK		0x00040000 // TX ready to accepts data
#define BCM283X_SPI0_RXD_MASK		0x00020000 // RX has data
#define BCM283X_SPI0_DONE_MASK		0x00010000 // Transfer done
#define BCM283X_SPI0_CPOL_MASK		0x00000008 // Clock Polarity
#define BCM283X_SPI0_CPHA_MASK		0x00000004 // Clock Polarity

/*--------  Pin Definitions for J8 header in RPI B+ models  --------*/
#define RPI_BPLUS_GPIO_J8_26		7 // Chip select 1
#define RPI_BPLUS_GPIO_J8_24		8 // Chip select 0
#define RPI_BPLUS_GPIO_J8_21		9 // MISO
#define RPI_BPLUS_GPIO_J8_19		10 // MOSI
#define RPI_BPLUS_GPIO_J8_23		11 // CLK

/*--------  RPi 3 definitions  --------*/
#define RPI_3_MODEL_NAME "Raspberry Pi 3"

#define BCM2835_RPI3_PERI_BASE		0x3F000000
#define BCM2835_RPI3_PERI_SIZE		0x01000000

/*--------  RPi 4 definitions  --------*/
#define RPI_4_MODEL_NAME "Raspberry Pi 4"

#define BCM2835_RPI4_PERI_BASE		0xFE000000
#define BCM2835_RPI4_PERI_SIZE		0x01800000



/*--------  Global variables  --------*/
static uint32_t *peripherals_base_addr;
static uint32_t peripherals_mem_size;
static uint32_t *peripherals_addr;

static uint32_t *spi_0_base_addr;
static uint32_t *spi_0_chip_sel_reg;
static uint32_t *spi_0_fifo_reg;
static uint32_t *spi_0_clk_reg;
static uint32_t *bcm283x_gpio_addr;

/*--------  Local helper functions  --------*/

/**
 * @brief Read a register with sync barriers
 *
 * @param uint32_t* Pointer to the register
 * @return uint32_t The val of the register
 */
static inline uint32_t bcm283x_read_reg(volatile uint32_t *reg_addr)
{
	uint32_t reg_val;

	__sync_synchronize();
	reg_val = *reg_addr;
	__sync_synchronize();

	return reg_val;
}

/**
 * @brief Write a value to a register with sync barriers
 *
 * @param uint32_t* the register address
 * @param val The register value
 */
static inline void bcm283x_write_reg(volatile uint32_t *reg_addr,
					const uint32_t val)
{
	__sync_synchronize();
	*reg_addr = val;
	__sync_synchronize();
}

/**
 * @brief Set of a register. Note that it is not an atomic operation
 *
 * @param uint32_t* the register address
 * @param val the value of the bit
 * @param mask the mask for the bit
 */
static inline void bcm283x_set_bits(volatile uint32_t *reg_addr, uint32_t val,
					uint32_t mask)
{
	uint32_t reg_val;

	reg_val = bcm283x_read_reg(reg_addr);
	reg_val = (reg_val & ~mask) | (val & mask);
	bcm283x_write_reg(reg_addr, reg_val);
}

/**
 * @brief Set the function of a gpio register. Used to set a pad to any of its
 *       alternative functions.
 *
 * @param pin The Rpi gpio number
 * @param mode The mode to set the pin to
 */
static inline void bcm283x_gpio_fsel(uint8_t pin, uint8_t mode)
{
	volatile uint32_t *reg_addr;
	uint8_t shift;
	uint32_t mask;
	uint32_t value;

	/* Function selects are 10 pins per 32 bit word, 3 bits per pin */
	reg_addr = bcm283x_gpio_addr + BCM283X_GPFSEL0_OFFSET/4 + (pin/10);
	shift = (pin % 10) * 3;
	mask = BCM283X_GPIO_FSEL_MASK << shift;
	value = mode << shift;
	bcm283x_set_bits(reg_addr, value, mask);
}

int bcm283x_periph_init(void)
{
	struct device_node *dtnode;
	char *full_model_name;

	// initialize statics
	peripherals_mem_size = 0;
	peripherals_base_addr = NULL;
	peripherals_addr = NULL;
	spi_0_base_addr = NULL;
	spi_0_chip_sel_reg = NULL;
	spi_0_fifo_reg = NULL;
	spi_0_clk_reg = NULL;
	bcm283x_gpio_addr = NULL;

	dtnode = of_find_node_by_path("/");
	full_model_name = (char *) of_get_property(dtnode, "model", NULL);

	if (strstr(full_model_name, RPI_4_MODEL_NAME) != NULL) {
		peripherals_base_addr = (uint32_t *) BCM2835_RPI4_PERI_BASE;
		peripherals_mem_size = BCM2835_RPI4_PERI_SIZE;
	} else if (strstr(full_model_name, RPI_3_MODEL_NAME) != NULL) {
		peripherals_base_addr = (uint32_t *) BCM2835_RPI3_PERI_BASE;
		peripherals_mem_size = BCM2835_RPI3_PERI_SIZE;
	} else {
		printk(KERN_ERR "bcm283x_periph: Incompatible RPI model %s!\n",
							full_model_name);
		return -ENODEV;
	}

	// remap peripheral address space into kernel virtual address
	peripherals_addr = ioremap((phys_addr_t) peripherals_base_addr,
					peripherals_mem_size);
	if (peripherals_addr == NULL) {
		printk(KERN_ERR "bcm283x_periph; Failed to " \
			" ioremap peripherals\n");
		return -ENOMEM;
	}

	// get spi0 address
	spi_0_base_addr = peripherals_addr + BCM283X_SPI0_OFFSET/4;
	spi_0_chip_sel_reg = spi_0_base_addr + BCM283X_SPI0_CS/4;
	spi_0_fifo_reg = spi_0_base_addr + BCM283X_SPI0_FIFO/4;
	spi_0_clk_reg = spi_0_base_addr + BCM283X_SPI0_CLK/4;

	// get gpio base address
	bcm283x_gpio_addr = peripherals_addr + BCM283X_GPIO_OFFSET/4;

	return 0;
}

void bcm283x_periph_close(void)
{
	iounmap(peripherals_addr);
	peripherals_base_addr = NULL;
	peripherals_mem_size = 0;
	spi_0_base_addr = NULL;
	bcm283x_gpio_addr = NULL;
}

void bcm283x_spi_begin(void)
{
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_26, BCM283X_GPIO_FSEL_ALT0); // CS1
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_24, BCM283X_GPIO_FSEL_ALT0); // CS0
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_21, BCM283X_GPIO_FSEL_ALT0); // MISO
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_19, BCM283X_GPIO_FSEL_ALT0); // MOSI
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_23, BCM283X_GPIO_FSEL_ALT0); // CLK

	// Clear the fifos
	bcm283x_write_reg(spi_0_chip_sel_reg, 0);

	*spi_0_chip_sel_reg = BCM283X_SPI0_CLEAR_MASK;
}

void bcm283x_spi_end(void)
{
	// Set all the SPI0 pins back to input
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_26, BCM283X_GPIO_FSEL_INPT); // CE1
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_24, BCM283X_GPIO_FSEL_INPT); // CE0
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_21, BCM283X_GPIO_FSEL_INPT); // MISO
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_19, BCM283X_GPIO_FSEL_INPT); // MOSI
	bcm283x_gpio_fsel(RPI_BPLUS_GPIO_J8_23, BCM283X_GPIO_FSEL_INPT); // CLK
}

void bcm283x_spi_set_clock_div(uint16_t divider_val)
{
	bcm283x_write_reg(spi_0_clk_reg, divider_val);
}

void bcm283x_spi_chip_sel(uint8_t chip_sel)
{
	bcm283x_set_bits(spi_0_chip_sel_reg, chip_sel, BCM283X_SPI0_CS_MASK);
}

void bcm283x_spi_set_data_mode(uint8_t mode)
{
	bcm283x_set_bits(spi_0_chip_sel_reg, mode << 2,
			BCM283X_SPI0_CPOL_MASK | BCM283X_SPI0_CPHA_MASK);
}

void bcm283x_spi_set_chip_sel_pol(uint8_t chip_sel, uint8_t enable_val)
{
	uint8_t shift;

	shift = 21 + chip_sel;
	bcm283x_set_bits(spi_0_chip_sel_reg, enable_val << shift, 1 << shift);
}

void bcm283x_spi_write_bytes_polling(uint8_t *buf, uint32_t len)
{
	uint32_t i;
	volatile uint32_t rx_data;

	// Clear TX and RX fifos
	bcm283x_set_bits(spi_0_chip_sel_reg, BCM283X_SPI0_CLEAR_MASK,
			BCM283X_SPI0_CLEAR_MASK);

	// Set transfer active flag
	bcm283x_set_bits(spi_0_chip_sel_reg, BCM283X_SPI0_TA_MASK,
			BCM283X_SPI0_TA_MASK);

	for (i = 0; i < len; i++) {
		// wait for tx fifo to accept data
		while (!(bcm283x_read_reg(spi_0_chip_sel_reg) &
			BCM283X_SPI0_TXD_MASK))
			;

		// write to fifo
		*spi_0_fifo_reg = buf[i];

		// read and discard incoming data
		while (bcm283x_read_reg(spi_0_chip_sel_reg) &
			BCM283X_SPI0_RXD_MASK) {
			rx_data = *spi_0_fifo_reg;
		}
	}

	// wait for done flag to be set
	while (!(*spi_0_chip_sel_reg & BCM283X_SPI0_DONE_MASK))
		while (bcm283x_read_reg(spi_0_chip_sel_reg) &
			BCM283X_SPI0_RXD_MASK)
				rx_data = *spi_0_fifo_reg;

	bcm283x_set_bits(spi_0_chip_sel_reg, 0,
			BCM283X_SPI0_TA_MASK);
}

void bcm283x_spi_read_bytes_polling(uint8_t *buf, uint32_t len)
{
	int num_tx_bytes_sent;
	int num_rx_bytes_recvd;

	// Clear TX and RX fifos
	bcm283x_set_bits(spi_0_chip_sel_reg, BCM283X_SPI0_CLEAR_MASK,
			BCM283X_SPI0_CLEAR_MASK);

	// Set transfer active flag
	bcm283x_set_bits(spi_0_chip_sel_reg, BCM283X_SPI0_TA_MASK,
			BCM283X_SPI0_TA_MASK);

	num_tx_bytes_sent = 0;
	num_rx_bytes_recvd = 0;
	while (num_rx_bytes_recvd < len) {
		while ((bcm283x_read_reg(spi_0_chip_sel_reg) &
			BCM283X_SPI0_TXD_MASK) && (num_tx_bytes_sent < len)) {
				*spi_0_fifo_reg = 0;
				num_tx_bytes_sent++;
			}

		while ((bcm283x_read_reg(spi_0_chip_sel_reg) &
			BCM283X_SPI0_RXD_MASK) && (num_rx_bytes_recvd < len)) {
				buf[num_rx_bytes_recvd] = *spi_0_fifo_reg;
				num_rx_bytes_recvd++;
			}
	}

	/* wait for done flag to be set */
	while (!(*spi_0_chip_sel_reg & BCM283X_SPI0_DONE_MASK))
		;

	bcm283x_set_bits(spi_0_chip_sel_reg, 0,
			BCM283X_SPI0_TA_MASK);
}

void bcm283x_gpio_set(uint32_t pin)
{
	volatile uint32_t *reg_addr;
	uint32_t shift;

	reg_addr = bcm283x_gpio_addr + BCM283X_GPSET0_OFFSET/4 + pin/32;
	shift = pin % 32;
	bcm283x_write_reg(reg_addr, 1 << shift);
}

void bcm283x_gpio_clr(uint32_t pin)
{
	volatile uint32_t *reg_addr;
	uint32_t shift;

	reg_addr = bcm283x_gpio_addr + BCM283X_GPCLR0_OFFSET/4 + pin/32;
	shift = pin % 32;
	bcm283x_write_reg(reg_addr, 1 << shift);
}
