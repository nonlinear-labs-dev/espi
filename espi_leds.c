#include <linux/of_gpio.h>
#include "espi_driver.h"

#define ESPI_LED_DEV_MAJOR		301
#define LED_STATES_SIZE			12
static u8 *led_st;
static u8 *led_new_st;
static DEFINE_MUTEX(led_state_lock);

/*******************************************************************************
    led functions
*******************************************************************************/
static ssize_t led_fops_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	u32 i;
	u8 val, led_id;

	mutex_lock(&led_state_lock);
	for(i=0; i<count; i++) {
		//val = (buf[i] >> 7) & 0x1;
		val = buf[i] >> 7;
		led_id = buf[i] & 0x7F;

		if(val == 0)
			led_new_st[LED_STATES_SIZE - 1 -led_id/8] &= ~(1<<(led_id%8));
		else if(val == 1)
			led_new_st[LED_STATES_SIZE - 1 -led_id/8] |= 1<<(led_id%8);
	}
	mutex_unlock(&led_state_lock);
	status = count;
	return status;
}

static ssize_t led_fops_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static s32 led_fops_open(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static s32 led_fops_release(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	return status;
}

static const struct file_operations led_fops = {
		.owner = 	THIS_MODULE,
		.write = 	led_fops_write,
		.read =		led_fops_read,
		.open =		led_fops_open,
		.release = 	led_fops_release,
		.llseek = 	no_llseek,
};

static struct class *led_class;

s32 espi_driver_leds_setup(struct espi_driver *sb)
{
	s32 i, ret;

	led_st = kcalloc(LED_STATES_SIZE,sizeof(u8), GFP_KERNEL);
	if (!led_st)
		return -ENOMEM;
	led_new_st = kcalloc(LED_STATES_SIZE,sizeof(u8), GFP_KERNEL);
	if (!led_new_st)
		return -ENOMEM;

	for(i=0; i<LED_STATES_SIZE; i++)
		led_new_st[i] = led_st[i] = 0;

	ret = register_chrdev(ESPI_LED_DEV_MAJOR, "spi", &led_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);

	led_class = class_create(THIS_MODULE, "espi-led");
	if(IS_ERR(led_class))
		pr_err("%s: unable to create class\n", __func__);

	device_create(led_class, sb->dev, MKDEV(ESPI_LED_DEV_MAJOR, 0), sb, "espi_led");

	return 0;
}

s32 espi_driver_leds_cleanup(struct espi_driver *sb)
{
	kfree(led_st);
	kfree(led_new_st);

	device_destroy(led_class, MKDEV(ESPI_LED_DEV_MAJOR, 0));
	class_destroy(led_class);
	unregister_chrdev(ESPI_LED_DEV_MAJOR, "spi");

	return 0;
}

void espi_driver_leds_poll(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u32 i, update = 0;

	mutex_lock(&led_state_lock);
	for(i=0; i<LED_STATES_SIZE; i++) {
		if(led_st[i] ^ led_new_st[i]) {
			led_st[i] = led_new_st[i];
			update = 1;
		}
	}
	mutex_unlock(&led_state_lock);

	if(update == 0)
		return;

	xfer.tx_buf = led_st;
	xfer.rx_buf = NULL;
	xfer.len = LED_STATES_SIZE;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;

	espi_driver_scs_select((struct espi_driver*)p, ESPI_SELECTION_PANEL_PORT, ESPI_SELECTION_LEDS_DEVICE);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	gpio_set_value(((struct espi_driver *)p)->gpio_sap, 0);
	gpio_set_value(((struct espi_driver *)p)->gpio_sap, 1);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SELECTION_PANEL_PORT, 0);
}


