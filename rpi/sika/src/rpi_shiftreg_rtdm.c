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
#define SHIFTREG_RTDM_TASK_PERIOD 1000000
#define SHIFTREG_RTDM_TASK_PRIO 85

/**
 * Module Read only parameters for reference
 */
static uint num_input_pins = SIKA_SHIFTREG_NUM_INPUT_PINS;
module_param(num_input_pins, uint, 0444);

static uint num_output_pins = SIKA_SHIFTREG_NUM_OUTPUT_PINS;
module_param(num_output_pins, uint, 0444);

static uint num_analog_pins = SIKA_SHIFTREG_NUM_ADC_PINS;
module_param(num_analog_pins, uint, 0444);

static uint adc_res = SIKA_SHIFTREG_ADC_RES;
module_param(adc_res, uint, 0444);

/**
 * Adjustable parameter for adc chans sampled per tick
 */
static uint adc_chans_per_tick = SIKA_SHIFTREG_ADC_CHANS_PER_TICK;
module_param(adc_chans_per_tick, uint, 0644);

// dev context
struct shiftreg_dev_context {
	uint32_t* pin_data;
	uint32_t *in_pin_data;
	uint32_t *out_pin_data;
	uint32_t *adc_pin_data;
	uint32_t next_adc_chans_to_sample;
	uint32_t task_period_ns;
	rtdm_event_t irq_event;
	rtdm_task_t *shiftreg_task;
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
		printk("Shiftreg_rtdm: Error - Failed to get load out gpio"
			" num %d (%d)\n", SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM, res);
		return res;
	}

	res = gpio_direction_output(SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM,
				SIKA_SHIFTREG_DISABLE_LOAD_OUT_VAL);
	if(res < 0) {
		printk("Shiftreg_rtdm: Error - Failed to set load out gpio"
			" num %d direction (%d)\n",
			SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM, res);
		return res;
	}

	res = gpio_request(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM, "load_in_gpio");
	if(res < 0) {
		printk("Shiftreg_rtdm: Error - Failed to get load in gpio"
			" num %d (%d)\n", SIKA_SHIFTREG_LOAD_IN_GPIO_NUM, res);
		return res;
	}

	res = gpio_direction_output(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM,
				SIKA_SHIFTREG_DISABLE_LOAD_IN_VAL);
	if(res < 0) {
		printk("Shiftreg_rtdm: Error - Failed to set load in gpio"
			" num %d direction (%d)\n",
			SIKA_SHIFTREG_LOAD_IN_GPIO_NUM, res);
		return res;
	}

	res = gpio_request(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM, "adc_load_gpio");
	if(res < 0) {
		printk("Shiftreg_rtdm: Error - Failed to get adc gpio"
			" num %d (%d)\n", SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM, res);
		return res;
	}

	res = gpio_direction_output(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM,
				SIKA_SHIFTREG_DISABLE_ADC_LOAD_VAL);
	if(res < 0) {
		printk("Shiftreg_rtdm: Error - Failed to set adc load gpio"
			" num %d direction (%d)\n",
			SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM, res);
		return res;
	}

	return 0;
}

static int rpi_shiftreg_rtdm_open(struct rtdm_fd *fd, int oflags)
{
	struct shiftreg_dev_context *context;

	printk("Opening Shiftreg_rtdm ...\n");
	context = (struct shiftreg_dev_context*) rtdm_fd_to_private(fd);

	memset((void*)context, 0, sizeof(struct shiftreg_dev_context));

	context->pin_data = kmalloc(PAGE_SIZE, GFP_USER);
	if(context->pin_data == NULL) {
		printk("Failed to allocate memory for pin data\n");
		return -ENOMEM;
	}

	context->in_pin_data = context->pin_data;
	context->out_pin_data = context->pin_data +
				SIKA_SHIFTREG_NUM_INPUT_PINS;
	context->adc_pin_data = context->out_pin_data +
				SIKA_SHIFTREG_NUM_OUTPUT_PINS;
	context->next_adc_chans_to_sample = 0;

	context->task_period_ns = SHIFTREG_RTDM_TASK_PERIOD;

	context->shiftreg_task = NULL;
	rtdm_event_init(&context->irq_event, 0);

	printk("Shiftreg_rtdm driver opened \n");
	return 0;
}

static void rpi_shiftreg_rtdm_close(struct rtdm_fd *fd)
{
	struct shiftreg_dev_context *context;
	context = (struct shiftreg_dev_context*) rtdm_fd_to_private(fd);

	if (context->shiftreg_task) {
		rtdm_event_destroy(&context->irq_event);
		rtdm_task_destroy(context->shiftreg_task);
		kfree(context->shiftreg_task);
	}
	if(context->pin_data != NULL)
		kfree(context->pin_data);

	printk("Shiftreg rtdm driver closed");
	return;
}

static int rpi_shiftreg_mmap_nrt(struct rtdm_fd *fd, struct vm_area_struct *vma)
{
	struct shiftreg_dev_context *context;
	context = (struct shiftreg_dev_context*) rtdm_fd_to_private(fd);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return rtdm_mmap_kmem(vma, context->pin_data);
}

static inline void tx_output_shiftreg_data(struct shiftreg_dev_context *context)
{
	int i,j, pin_num;
	uint32_t interleaved_val;
	uint8_t out_shiftreg_data[SIKA_SHIFTREG_NUM_OUTPUT_SHIFTREG];

	// interleave
	pin_num = SIKA_SHIFTREG_NUM_OUTPUT_PINS - 1;
	for(i = 0; i < SIKA_SHIFTREG_NUM_OUTPUT_SHIFTREG; i++) {
		out_shiftreg_data[i] = 0;
		for(j = 7; j >= 0; j--) {
			interleaved_val = context->out_pin_data[pin_num] << j;
			out_shiftreg_data[i] |= interleaved_val;
			pin_num--;
		}
	}


	// output shiftregisters do not have a chipselect
	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
	bcm2835_spi_writenb(out_shiftreg_data,
				SIKA_SHIFTREG_NUM_OUTPUT_SHIFTREG);

	bcm2835_gpio_clr(SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM);

	rtdm_task_sleep(SIKA_SHIFTREG_LOAD_OUT_PERIOD_NS);

	bcm2835_gpio_set(SIKA_SHIFTREG_LOAD_OUT_GPIO_NUM);
}

static inline void rx_input_shiftreg_data(struct shiftreg_dev_context *context)
{
	int i, j, pin_num;
	uint8_t in_shiftreg_data[SIKA_SHIFTREG_NUM_INPUT_SHIFTREG];

	// load in gpio
	bcm2835_gpio_clr(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM);

	rtdm_task_sleep(SIKA_SHIFTREG_LOAD_IN_PERIOD_NS);

	bcm2835_gpio_set(SIKA_SHIFTREG_LOAD_IN_GPIO_NUM);

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
			context->in_pin_data[pin_num] =
					(in_shiftreg_data[i] >> j) & 0x1;
			pin_num--;
		}
	}
}

static inline void rx_adc_data(struct shiftreg_dev_context *context)
{
	uint8_t mux_ctrl_shiftreg_data;
	uint8_t adc_spi_data[SIKA_SHIFTREG_ADC_SPI_LEN_BYTES];
	uint32_t pin_num;
	uint32_t next_pin_num;

	bcm2835_spi_chipSelect(SIKA_SHIFTREG_ADC_CS);
	pin_num = context->next_adc_chans_to_sample;
	next_pin_num = pin_num + SIKA_SHIFTREG_ADC_CHANS_PER_TICK;


	while(pin_num < next_pin_num) {
		mux_ctrl_shiftreg_data = pin_num;

		bcm2835_spi_writenb(&mux_ctrl_shiftreg_data, 1);

		bcm2835_gpio_clr(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM);

		rtdm_task_sleep(SIKA_SHIFTREG_ADC_LOAD_PERIOD_NS);

		bcm2835_gpio_set(SIKA_SHIFTREG_ADC_LOAD_GPIO_NUM);

		rtdm_task_sleep(SIKA_SHIFTREG_ADC_ACQ_PERIOD_NS);

		bcm2835_spi_transfern(adc_spi_data,
					SIKA_SHIFTREG_ADC_SPI_LEN_BYTES);

		//readjust bits as data from adc is msb 1st
		context->adc_pin_data[pin_num] = adc_spi_data[1] >> 4;
		context->adc_pin_data[pin_num] |= (uint32_t)adc_spi_data[0] << 4;
		pin_num++;
	}

	if(next_pin_num == SIKA_SHIFTREG_NUM_ADC_PINS)
		next_pin_num = 0;

	context->next_adc_chans_to_sample = next_pin_num;
}

static inline void rpi_shiftreg_intr_handler(void *ctx)
{
	struct shiftreg_dev_context *context;
	uint64_t next_wake_up_time_ns;
	uint64_t exec_time_ns;
	int res;

	context = (struct shiftreg_dev_context *) ctx;

	while (!rtdm_task_should_stop()) {
		next_wake_up_time_ns = rtdm_clock_read_monotonic() +
					context->task_period_ns;

		tx_output_shiftreg_data(context);
		rx_input_shiftreg_data(context);
		rx_adc_data(context);
		rtdm_event_signal(&context->irq_event);

		exec_time_ns = rtdm_clock_read_monotonic();

		if (exec_time_ns >= next_wake_up_time_ns) {
			printk("Shiftreg rtdm task overshot its deadline\n");
			rtdm_task_sleep(context->task_period_ns);
		}
		else {
			res = rtdm_task_sleep_abs(next_wake_up_time_ns,
						RTDM_TIMERMODE_ABSOLUTE);
			if(res != 0) {
				printk("rtdm_task_sleep_abs failed Error" \
				" code: %d\n", res);
			}
		}
	}
}

static int rpi_shiftreg_rtdm_ioctl_rt(struct rtdm_fd *fd, unsigned int request,
					void __user *arg)
{
	int task_period_ns, res;
	struct shiftreg_dev_context *context =
			(struct shiftreg_dev_context *)rtdm_fd_to_private(fd);

	switch(request) {
	case SHIFTREG_RTDM_WAIT_ON_RT_TASK:
		return rtdm_event_wait(&context->irq_event);
		break;

	case SHIFTREG_RTDM_START_RT_TASK:
		return -ENOSYS;
		break;

	case SHIFTREG_RTDM_STOP_RT_TASK:
		return -ENOSYS;
		break;

	case SHIFTREG_RTDM_SET_TICK_PERIOD:
		task_period_ns = SHIFTREG_RTDM_TASK_PERIOD;
		res = rtdm_safe_copy_from_user(fd, &task_period_ns,
						arg, sizeof(int));
		if(res != 0) {
			printk("User failed to set task period. Error %d.", res);
			return res;
		}

		context->task_period_ns = task_period_ns;
		printk("kernel task period set to %d ns\n", task_period_ns);
		return 0;
		break;

	default:
		printk(KERN_WARNING "shiftreg_rtdm : shiftreg_ioctl_rt: " \
			"invalid value %d\n", request);
		return -EINVAL;
	}

	return 0;
}

static int rpi_shiftreg_rtdm_ioctl_nrt(struct rtdm_fd *fd, unsigned int request,
					void __user *arg)
{
	struct shiftreg_dev_context *context =
			(struct shiftreg_dev_context *)rtdm_fd_to_private(fd);
	int result = 0;

	switch(request) {
	case SHIFTREG_RTDM_START_RT_TASK:
		context->shiftreg_task = kcalloc(1,sizeof(rtdm_task_t),
							GFP_KERNEL);
		result = rtdm_task_init(context->shiftreg_task,
					"shiftreg_rtdm_driver_task",
					rpi_shiftreg_intr_handler, context,
					85, 0);
		if (result) {
			printk("shiftreg_rtdm: rtdm_task_init failed\n");
			return -EINVAL;
		}
		break;

	case SHIFTREG_RTDM_STOP_RT_TASK:
		if (context->shiftreg_task) {
			rtdm_event_destroy(&context->irq_event);
			rtdm_task_destroy(context->shiftreg_task);
			kfree(context->shiftreg_task);
			context->shiftreg_task = NULL;
		}
		break;

	default:
		printk(KERN_WARNING "shiftreg_rtdm : shiftreg_ioctl_nrt: " \
			"invalid value %d\n", request);
		return -EINVAL;
	}

	return result;
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
		.ioctl_nrt = rpi_shiftreg_rtdm_ioctl_nrt,
		.close = rpi_shiftreg_rtdm_close,
		.mmap = rpi_shiftreg_mmap_nrt
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
		printk("Shiftreg_rtdm : Error in init: realtime core is not"
			" enabled!\n");
		return -ENODEV;
	}

	printk("Shiftreg_rtdm: Starting Init...\n");
	

	res = bcm2835_init();
	if (res != 1) {
		printk("Shiftreg_rtdm: Error in bcm2835_init (%d).\n", res);
		return res;
	}

	res = init_shiftreg_settings();
	if(res != 0) {
		printk("Shiftreg_rtdm: Failed to init shiftreg settings"
		" (%d).\n", res);
		return res;
	}

	res = rtdm_dev_register(&rpi_shiftreg_device);
	if(res != 0) {
		rtdm_dev_unregister(&rpi_shiftreg_device);
		printk("Shiftreg_rtdm: Failed to init driver (%d).\n", res);
		return res;
	}

	return 0;
}

/**
 * This function is called when the module is unloaded. It unregisters the RTDM device.
 */
static void __exit rpi_shiftreg_rtdm_exit(void) {

	printk("Shiftreg_rtdm: Exiting driver\n");

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