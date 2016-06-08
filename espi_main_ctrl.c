#include <linux/poll.h>
#include <linux/of_gpio.h>
#include "espi_driver.h"

#define ESPI_MAIN_CTRL_DEV_MAJOR		321
static u8 main_ctrl;
static u8 update = 0;

/*******************************************************************************
    main ctrl functions
*******************************************************************************/
static ssize_t main_ctrl_fops_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	if(count < 1)
		return -EAGAIN;
	main_ctrl = buf[0];
	update = 1;
	status = count;
	return status;
}

static ssize_t main_ctrl_fops_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static s32 main_ctrl_fops_open(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static s32 main_ctrl_fops_release(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	return status;
}

static const struct file_operations main_ctrl_fops = {
		.owner = 	THIS_MODULE,
		.write = 	main_ctrl_fops_write,
		.read =		main_ctrl_fops_read,
		.open =		main_ctrl_fops_open,
		.release = 	main_ctrl_fops_release,
		.llseek = 	no_llseek,
};

static struct class *main_ctrl_class;

s32 espi_driver_main_ctrl_setup(struct espi_driver *sb)
{
	s32 ret;

	main_ctrl = 0;

	ret = register_chrdev(ESPI_MAIN_CTRL_DEV_MAJOR, "spi", &main_ctrl_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);

	main_ctrl_class = class_create(THIS_MODULE, "espi-main-ctrl");
	if(IS_ERR(main_ctrl_class))
		pr_err("%s: unable to create class\n", __func__);

	device_create(main_ctrl_class, sb->dev, MKDEV(ESPI_MAIN_CTRL_DEV_MAJOR, 0), sb, "espi_main_ctrl");

	return 0;
}

s32 espi_driver_main_ctrl_cleanup(struct espi_driver *sb)
{
	device_destroy(main_ctrl_class, MKDEV(ESPI_MAIN_CTRL_DEV_MAJOR, 0));
	class_destroy(main_ctrl_class);
	unregister_chrdev(ESPI_MAIN_CTRL_DEV_MAJOR, "spi");

	return 0;
}

void espi_driver_main_ctrl_poll(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u8 txbuff[1];
	extern int sck_hz;

	if(update == 0)
		return;
	update = 0;

	txbuff[0] = main_ctrl;

	xfer.tx_buf = txbuff;
	xfer.rx_buf = NULL;
	xfer.len = 1;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = sck_hz;

	espi_driver_scs_select((struct espi_driver*)p, ESPI_MAIN_CONTROL_PORT, ESPI_MAIN_CONTROL_DEVICE);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	gpio_set_value(((struct espi_driver *)p)->gpio_sap, 0);
	gpio_set_value(((struct espi_driver *)p)->gpio_sap, 1);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_MAIN_CONTROL_PORT, 0);
}


