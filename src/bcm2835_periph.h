#ifndef BCM2835_SPI_H_
#define BCM2835_SPI_H_

/*--------  Available Chip select definitions  --------*/
#define BCM283X_SPI_CS0			0
#define BCM283X_SPI_CS1			1
#define BCM283X_SPI_CS2			2
#define BCM283X_SPI_CS_NONE		3

int bcm283x_periph_init(void);

void bcm283x_periph_close(void);

void bcm283x_spi_begin(void);

void bcm283x_spi_end(void);

void bcm283x_spi_set_clock_div(uint16_t divider_val);

void bcm283x_spi_chip_sel(uint8_t chip_sel);

void bcm283x_spi_set_chip_sel_pol(uint8_t chip_sel, uint8_t enable_val);

void bcm283x_spi_set_data_mode(uint8_t mode);

void bcm283x_spi_write_bytes_polling(uint8_t* buf, uint32_t len);

void bcm283x_spi_read_bytes_polling(uint8_t* buf, uint32_t len);

void bcm283x_gpio_set(uint32_t pin);

void bcm283x_gpio_clr(uint32_t pin);

#endif // BCM2835_SPI_H_