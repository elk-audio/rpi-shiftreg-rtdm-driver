#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/gpio.h>

#include <rtdm/rtdm.h>
#include <rtdm/driver.h>

#include "../../../include/shiftreg_rtdm.h"
#include "../include/bcm2835.h"
#include "../include/sika_shiftreg_defs.h"

#define DEVICE_NAME		"shiftreg_rtdm"

struct shiftreg_dev_context {
	bool config_done;
	struct shiftreg_addr_config_t config;
};

static int init_shiftreg_settings(void)
{
	int res;

	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(SIKA_SHIFTREG_SPI_BIT_ORDER);
	bcm2835_spi_setDataMode(SIKA_SHIFTREG_SPI_DATA_MODE);
	bcm2835_spi_setClockDivider(SIKA_SHIFTREG_SPI_CLOCK_DIV);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
	bcm2835_spi_setChipSelectPolarity(SIKA_SHIFTREG_INPUT_SHIFTREG_CS,
					SIKA_SHIFTREG_SPI_CS_ON_VAL);
	bcm2835_spi_setChipSelectPolarity(SIKA_SHIFTREG_ADC_CS,
					SIKA_SHIFTREG_SPI_CS_ON_VAL);

	res = gpio_request(SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM, "load_out_gpio");
	if(res < 0) {
		printk("rpi_shiftreg_rtdm: Error - Failed to get load out gpio"
			" num %d (%d)\n", SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM, res);
		return res;
	}

	res = gpio_direction_output(SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM,
				SIKA_SHIFTREG_DISABLE_LOAD_OUT_VAL);
	if(res < 0) {
		printk("rpi_shiftreg_rtdm: Error - Failed to set load out gpio"
			" num %d direction (%d)\n",
			SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM, res);
		return res;
	}

	res = gpio_request(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM, "load_in_gpio");
	if(res < 0) {
		printk("rpi_shiftreg_rtdm: Error - Failed to get load in gpio"
			" num %d (%d)\n", SIKA_SHIFTREG_LOAD_IN_GPIO_NUM, res);
		return res;
	}

	res = gpio_direction_output(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM,
				SIKA_SHIFTREG_DISABLE_LOAD_IN_VAL);
	if(res < 0) {
		printk("rpi_shiftreg_rtdm: Error - Failed to set load in gpio"
			" num %d direction (%d)\n",
			SIKA_SHIFTREG_LOAD_IN_GPIO_NUM, res);
		return res;
	}

	res = gpio_request(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM, "adc_load_gpio");
	if(res < 0) {
		printk("rpi_shiftreg_rtdm: Error - Failed to get adc gpio"
			" num %d (%d)\n", SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM, res);
		return res;
	}

	res = gpio_direction_output(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM,
				SIKA_SHIFTREG_DISABLE_ADC_LOAD_VAL);
	if(res < 0) {
		printk("rpi_shiftreg_rtdm: Error - Failed to set adc load gpio"
			" num %d direction (%d)\n",
			SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM, res);
		return res;
	}

	return 0;
}

static int rpi_shiftreg_rtdm_open(struct rtdm_fd *fd, int oflags)
{
	struct shiftreg_dev_context *context;

	printk("rpi_shiftreg_rtdm open\n");
	context = (struct shiftreg_dev_context*) rtdm_fd_to_private(fd);

	memset((void*)context, 0, sizeof(struct shiftreg_dev_context));
	return 0;
}

static void rpi_shiftreg_rtdm_close(struct rtdm_fd *fd)
{
	return;
}

static inline int tx_output_shiftreg_data(struct rtdm_fd *fd)
{
	struct shiftreg_dev_context *context;
	int res;
	int i,j, pin_num;
	uint32_t out_pin_data[SIKA_SHIFTREG_NUM_OUTPUT_PINS];
	uint8_t out_shiftreg_data[SIKA_SHIFTREG_NUM_OUTPUT_SHIFTREG];

	context = (struct shiftreg_dev_context *) rtdm_fd_to_private(fd);

	res = rtdm_safe_copy_from_user(fd,
			out_pin_data,
			context->config.user_output_pin_data,
			SIKA_SHIFTREG_NUM_OUTPUT_PINS*sizeof(uint32_t));
	if(res != 0) {
		printk("rpi_shiftreg_rtdm: Error - Cannot copy output pin data"
			" data from user (%d).\n", res);
		return res;
	}

	// interleave
	pin_num = SIKA_SHIFTREG_NUM_OUTPUT_PINS - 1;
	for(i = 0; i < SIKA_SHIFTREG_NUM_OUTPUT_SHIFTREG; i++) {
		out_shiftreg_data[i] = 0;
		for(j = 7; j >= 0; j--) {
			out_pin_data[pin_num] &= 0x1;
			out_pin_data[pin_num] = out_pin_data[pin_num] << j;
			out_shiftreg_data[i] |= out_pin_data[pin_num];
			pin_num--;
		}
	}


	// output shiftregisters do not have a chipselect
	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
	bcm2835_spi_writenb(out_shiftreg_data,
				SIKA_SHIFTREG_NUM_OUTPUT_SHIFTREG);

	gpio_set_value(SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM,
			SIKA_SHIFTREG_ENABLE_LOAD_OUT_VAL);

	rtdm_task_sleep(SIKA_SHIFTREG_LOAD_OUT_PERIOD_NS);

	gpio_set_value(SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM,
			SIKA_SHIFTREG_DISABLE_LOAD_OUT_VAL);

	return 0;
}

static inline int rx_input_shiftreg_data(struct rtdm_fd *fd)
{
	struct shiftreg_dev_context *context;
	int res;
	int i, j, pin_num;
	uint32_t in_pin_data[SIKA_SHIFTREG_NUM_INPUT_PINS];
	uint8_t in_shiftreg_data[SIKA_SHIFTREG_NUM_INPUT_SHIFTREG];

	context = (struct shiftreg_dev_context *) rtdm_fd_to_private(fd);

	// load in gpio
	gpio_set_value(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM,
			SIKA_SHIFTREG_ENABLE_LOAD_IN_VAL);

	rtdm_task_sleep(SIKA_SHIFTREG_LOAD_IN_PERIOD_NS);

	gpio_set_value(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM,
			SIKA_SHIFTREG_DISABLE_LOAD_IN_VAL);

	rtdm_task_sleep(SIKA_SHIFTREG_LOAD_IN_PERIOD_NS);

	// select chipselect for in shift regs
	bcm2835_spi_chipSelect(SIKA_SHIFTREG_INPUT_SHIFTREG_CS);

	// perform rx
	bcm2835_spi_transfern(in_shiftreg_data,
			SIKA_SHIFTREG_NUM_INPUT_SHIFTREG);

	// deinterleave
	pin_num = SIKA_SHIFTREG_NUM_INPUT_PINS - 1;
	for(i = 0; i < SIKA_SHIFTREG_NUM_INPUT_SHIFTREG; i++) {
		for(j = 7; j >= 0; j--) {
			in_pin_data[pin_num] = (in_shiftreg_data[i] >> j) & 0x1;
			pin_num--; 
		}
	}

	// copy to user buf
	res = rtdm_safe_copy_to_user(fd,
			context->config.user_input_pin_data,
			in_pin_data,
			SIKA_SHIFTREG_NUM_INPUT_PINS * sizeof(uint32_t));
	if(res != 0) {
		printk("rpi_shiftreg_rtdm: Error - Cannot copy input pin"
			" data to user (%d).\n", res);
		return res;
	}

	return 0;
}

static inline int rx_adc_data(struct rtdm_fd *fd)
{
	int res;
	struct shiftreg_dev_context *context;
	uint8_t mux_ctrl_shiftreg_data;
	uint32_t adc_data[SIKA_SHIFTREG_NUM_ADC_PINS];
	uint8_t adc_spi_data[SIKA_SHIFTREG_ADC_SPI_LEN_BYTES];
	int pin_num;

	for(pin_num = 0; pin_num < SIKA_SHIFTREG_NUM_ADC_PINS; pin_num++) {
		mux_ctrl_shiftreg_data = pin_num;

		bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
		bcm2835_spi_writenb(&mux_ctrl_shiftreg_data, 1);

		gpio_set_value(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM,
				SIKA_SHIFTREG_ENABLE_ADC_LOAD_VAL);

		rtdm_task_sleep(SIKA_SHIFTREG_ADC_LOAD_PERIOD_NS);

		gpio_set_value(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM,
				SIKA_SHIFTREG_DISABLE_ADC_LOAD_VAL);

		rtdm_task_sleep(SIKA_SHIFTREG_ADC_ACQ_PERIOD_NS);

		bcm2835_spi_chipSelect(SIKA_SHIFTREG_ADC_CS);
		bcm2835_spi_transfern(adc_spi_data,
					SIKA_SHIFTREG_ADC_SPI_LEN_BYTES);

		// readjust bits as data from adc is msb 1st
		adc_data[pin_num] = adc_spi_data[1] >> 4;
		adc_data[pin_num] |= (uint32_t)adc_spi_data[0] << 4; 
	}

	context = (struct shiftreg_dev_context *) rtdm_fd_to_private(fd);
	res = rtdm_safe_copy_to_user(fd,
				context->config.user_adc_pin_data,
				adc_data,
				SIKA_SHIFTREG_NUM_ADC_PINS * sizeof(uint32_t));
	if(res != 0) {
		printk("rpi_shiftreg_rtdm: Error - Cannot copy adc data to"
			" user (%d).\n", res);
		return res;
	}

	return 0;
}

static inline int rpi_shiftreg_xfer(struct rtdm_fd *fd)
{
	struct shiftreg_dev_context *context;
	int res;

	context = (struct shiftreg_dev_context *) rtdm_fd_to_private(fd);

	res = tx_output_shiftreg_data(fd);
	if(res != 0) {
		printk("rpi_shiftreg_rtdm: Failed to send output"
			" shiftreg data.\n");
		return res;
	}

	res = rx_input_shiftreg_data(fd);
	if(res != 0) {
		printk("rpi_shiftreg_rtdm: Failed to get input"
			" shiftreg data.\n");
		return res;
	}

	res = rx_adc_data(fd);
	if(res != 0) {
		printk("rpi_shiftreg_rtdm: Failed to get adc"
			" data.\n");
		return res;
	}

	return 0;
}

static int rpi_shiftreg_rtdm_ioctl_rt(struct rtdm_fd *fd,
					unsigned int request, void __user *arg)
{
	struct shiftreg_dev_context *context;
	int res;

	context = (struct shiftreg_dev_context *)rtdm_fd_to_private(fd);

	// if running perform xfer
	if(context->config_done == true)
		return rpi_shiftreg_xfer(fd);

	switch (request) {
	case SHIFTREG_SET_ADDR_CONFIG:
		res = rtdm_safe_copy_from_user(fd, &context->config,
						arg, sizeof(struct shiftreg_addr_config_t));
		if(res != 0) {
			printk("rpi_shiftreg_rtdm: Error - cannot copy shiftreg"
				"addr config from user (%d)\n", res);
			return res;
		}

		// Finally check if the user buffer address are valid
		if(context->config.user_output_pin_data == NULL) {
			printk("rpi_shiftreg_rtdm: Error - Invalid user address"
				"  for output shiftreg data\n");
			return -EINVAL;
		}

		if(context->config.user_input_pin_data == NULL) {
			printk("rpi_shiftreg_rtdm: Error - Invalid user address"
				" for output shiftreg data\n");
			return -EINVAL;
		}

		if(context->config.user_adc_pin_data == NULL) {
			printk("rpi_shiftreg_rtdm: Error - Invalid user address"
				" for output shiftreg data\n");
			return -EINVAL;
		}

		context->config_done = true;
		return 0;

	case SHIFTREG_XFER:
		if(!context->config_done) {
			printk("rpi_shiftreg_rtdm: Error - shiftreg driver not"
				" configured or invalid confiuration.\n");
			return -EINVAL;
		}

		return rpi_shiftreg_xfer(fd);

	default: /* Unexpected case */
		printk("rpi_shiftreg_rtdm: Error - invalid ioctl\n");
		return -EINVAL;
	}
}

static struct rtdm_driver rpi_shiftreg_driver = {
	.profile_info = RTDM_PROFILE_INFO(gpio,
					RTDM_CLASS_EXPERIMENTAL,
					RTDM_SUBCLASS_GENERIC,
					42),
	.device_flags = RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count = 1,
	.context_size = sizeof(struct shiftreg_dev_context),
	.ops = {
		.open = rpi_shiftreg_rtdm_open,
		.ioctl_rt = rpi_shiftreg_rtdm_ioctl_rt,
		.close = rpi_shiftreg_rtdm_close
	}
};


static struct rtdm_device rpi_shiftreg_device = {
	.driver = &rpi_shiftreg_driver,
	.label = DEVICE_NAME,
};

/**
 * Initializes the spi device using the bcm2835 libary, the gpios for the 
   and shiftregisters.
 */
static int __init rpi_shiftreg_rtdm_init(void) {

	int res;

	if (!realtime_core_enabled()) {
		printk("rpi_shiftreg_rtdm : Error in init: realtime core is not"
			" enabled!\n");
		return -ENODEV;
	}

	printk("rpi_shiftreg_rtdm: Starting Init...\n");
	

	res = bcm2835_init();
	if (res != 1) {
		printk("rpi_shiftreg_rtdm: Error in bcm2835_init (%d).\n", res);
		return res;
	}

	res = init_shiftreg_settings();
	if(res != 0) {
		printk("rpi_shiftreg_rtdm: Failed to init shiftreg settings"
		" (%d).\n", res);
		return res;
	}

	res = rtdm_dev_register(&rpi_shiftreg_device);
	if(res != 0) {
		rtdm_dev_unregister(&rpi_shiftreg_device);
		printk("rpi_shiftreg_rtdm: Failed to init driver (%d).\n", res);
		return res;
	}

	return 0;
}

/**
 * This function is called when the module is unloaded. It unregisters the RTDM device.
 */
static void __exit rpi_shiftreg_rtdm_exit(void) {

	printk("rpi_shiftreg_rtdm: Exiting driver\n");

	rtdm_dev_unregister(&rpi_shiftreg_device);

	bcm2835_spi_end();
	bcm2835_close();

	gpio_free(SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM);
	gpio_free(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM);
	gpio_free(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM);
}

/*
 * Link init and exit functions with driver entry and exit points.
 */
module_init(rpi_shiftreg_rtdm_init);
module_exit(rpi_shiftreg_rtdm_exit);

/*
 * Register module values
 */
MODULE_VERSION("0.1.0");
MODULE_DESCRIPTION("Real-Time Shiftreg driver for the Broadcom BCM283x SoC");
MODULE_AUTHOR("Sharan Yagneswar");
MODULE_LICENSE("GPL v2");