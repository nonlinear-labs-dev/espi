/*
 * espi_driver.c - The simplest and dirty kernel module
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>	// Write data to sysfs
#include <linux/of_gpio.h>  // nni: added for device tree
#include <linux/delay.h>  // nni: added for msleep()
#include <linux/platform_device.h>

#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/hardirq.h>



#if 0 // HW V.2 Rev.A
#define ESPI_ATTENUATOR_PORT		2
#define ESPI_ADC_PORT			3
#define ESPI_BTN_LED_PANELS_PORT	7
#define ESPI_LARGE_DISPLAY_PORT		8
#define ESPI_SMALL_DISPLAY_PORT		5
#define ESPI_RIBBON_LEDS_PORT		6
#endif


#if 1 // HW V.2 Rev.C
#define ESPI_RIBBON_LEDS_PORT		            3
#define ESPI_DEVICE_TOP_COVER_RIBBON_LEDS       2

// alte
#define ESPI_ATTENUATOR_PORT		1
#define ESPI_ADC_PORT			    6
#define ESPI_BTN_LED_PANELS_PORT	7
#define ESPI_LARGE_DISPLAY_PORT		8
#define ESPI_SMALL_DISPLAY_PORT		5
#endif



#if 0 // delete later

#define ESPI_PORT_EDIT_PANEL                    0
#define ESPI_PORT_SELECTION_PANELS              1
#define ESPI_PORT_TOP_COVER                     3
#define ESPI_PORT_SPARE                         2      

#define ESPI_DEVICE_EDIT_PANEL_BUTTONS          1
#define ESPI_DEVICE_EDIT_PANEL_BOLED            2
#define ESPI_DEVICE_EDIT_PANEL_ENCODER          3

#define ESPI_DEVICE_SELECTION_PANELS_BUTTONS    1
#define ESPI_DEVICE_SELECTION_PANELS_LEDS       2

#define ESPI_DEVICE_TOP_COVER_BUTTONS           1

#define ESPI_DEVICE_TOP_COVER_SOLED             3

#endif


#define ESPI_SPI_SPEED	1000000

struct espi_driver {
	struct delayed_work work; // This must be the top entry!
	struct device *dev;
	struct spi_device *spidev;

	// added driver params
	s32 scs_gpios[8];
	s32 sap_gpio;
	s32 dmxs_gpio;

	u8 poll_stage;
};

static struct workqueue_struct *workqueue;




/*******************************************************************************
    defines and variables for the different drivers
*******************************************************************************/


/* Buttons stuff **************************************************************/
#define ESPI_BUTTON_DEV_MAJOR		302
#define BUTTON_BYTES_GENERAL_PANELS	12
#define BUTTON_BYTES_CENTRAL_PANEL	3
#define BUTTON_BYTES_SOLED_PANEL	1	
#define BUTTON_STATES_SIZE		16
static u8 *btn_sm1;
static u8 *btn_st;
#define BUTTON_BUFFER_SIZE		256
static u8 *button_buff;
static u32 btn_buff_head, btn_buff_tail;
static DEFINE_MUTEX(btn_buff_tail_lock);
static DECLARE_WAIT_QUEUE_HEAD(btn_wqueue);

/* LEDs stuff *****************************************************************/
#define ESPI_LED_DEV_MAJOR		301
#define LED_STATES_SIZE			12
static u8 *led_st;
static u8 *led_new_st;
static DEFINE_MUTEX(led_state_lock);

/* RIBBON LEDs stuff **********************************************************/
#define ESPI_RIBBON_LED_DEV_MAJOR	303
#define RIBBON_LED_STATES_SIZE		9
static u8 *rb_led_st;
static u8 *rb_led_new_st;

/* SSD1305 stuff **************************************************************/
#define ESPI_SSD1305_DEV_MAJOR	304

#define SSD1305_DISP_OFF		0xAE
#define SSD1305_DISP_ON			0xAF
#define SSD1305_SET_RATIO_OSC	0xD5
#define SSD1305_SET_COL_ADDR	0x21
#define SSD1305_SET_AREA_COLOR	0xD8
#define SSD1305_SET_SEG_REMAP1	0xA1
#define SSD1305_SET_SCAN_NOR	0xC8
#define SSD1305_SET_OFFSET		0xD3
#define SSD1305_SET_CONTRAST	0x81
#define SSD1305_SET_CHARGE  	0xD9
#define SSD1305_SET_VCOM    	0xDB
#define SSD1305_EON_OFF			0xA4
#define SSD1305_DISP_NOR		0xA6
#define SSD1305_MEM_ADDRESSING 	0x20
#define SSD1305_SET_PAGE		0xB0
#define SSD1305_SET_COL_HI		0x10
#define SSD1305_SET_COL_LO		0x00
#define SSD1305_SET_PAGE_ADDR	0x22

#define SSD1305_BUFF_SIZE		(132*4)
static u8 *ssd1305_buff;
static u8 *ssd1305_tmp_buff;
static DEFINE_MUTEX(ssd1305_tmp_buff_lock);

/* SSD1322 stuff **************************************************************/
#define ESPI_SSD1322_DEV_MAJOR		305
#define ESPI_SSD1322_SPEED		24000000

#define SSD1322_SET_CMD_LOCK		0xFD
#define SSD1322_SET_DISP_OFF		0xAE
#define SSD1322_SET_DISP_ON		0xAF
#define SSD1322_SET_COL_ADDR		0x15
#define SSD1322_SET_ROW_ADDR		0x75
#define SSD1322_SET_DISP_CLK		0xB3
#define SSD1322_SET_MUX_RATIO		0xCA
#define SSD1322_SET_DISP_OFFSET		0xA2
#define SSD1322_SET_START_LINE		0xA1
#define SSD1322_SET_REMAP		0xA0
#define SSD1322_SET_GPIO		0xB5
#define SSD1322_SET_FUNC_SEL		0xAB
#define SSD1322_SET_DISP_ENH_A		0xB4
#define SSD1322_SET_DISP_ENH_B		0xD1
#define SSD1322_SET_CONTRAST_CUR	0xC1
#define SSD1322_SET_MASTER_CUR		0xC7
#define SSD1322_SET_LINEAR_GST		0xB9
#define SSD1322_SET_PHASE_LEN		0xB1
#define SSD1322_SET_PRECH_VOL		0xBB
#define SSD1322_SET_PRECH_PER		0xB6
#define SSD1322_SET_VCOMH		0xBE
#define SSD1322_SET_DISP_MODE		0xA4
#define SSD1322_SET_PARTIAL_DISP	0xA8
#define SSD1322_WRITE_RAM		0x5C

#define SSD1322_BUFF_SIZE		(128*64)
#define SSD1322_BUFF_WIDTH		128
#define SSD1322_BUFF_HEIGHT		64
static u8 *ssd1322_buff;
static u8 *ssd1322_tmp_buff;
static u8 ssd1322_buff_updated;
static DEFINE_MUTEX(ssd1322_tmp_buff_lock);

/* Attenuator stuff ***********************************************************/
#define ESPI_ATTENUATOR_DEV_MAJOR		306
static u8 attenuator_channel_val[3];
static u8 attenuator_channel_updated[3];
static DEFINE_MUTEX(attenuator_lock);

/* ADC stuff ******************************************************************/
#define ESPI_ADC_DEV_MAJOR		307
#define ESPI_ADC_3201			1
#define ESPI_ADC_3204			4
#define ESPI_ADC_3208			8
#define ESPI_ADC_320X			ESPI_ADC_3208
static u16 adc_channel_val[ESPI_ADC_320X];

/* Encoder stuff **************************************************************/
#define ESPI_ENCODER_DEV_MAJOR		308
static s8 encoder_delta;
static DECLARE_WAIT_QUEUE_HEAD(encoder_wqueue);




/*******************************************************************************
    eSPI general driver functions
    @param[in]  port: 1..8
    @param[in]  device: 1..3
                        0 -> all off
*******************************************************************************/
static void espi_driver_scs_select(struct espi_driver *spi, s32 port, s32 device)
{
	s32 s;

	gpio_set_value(spi->dmxs_gpio, 1);	            // dmxs disable: avoid glitches

	if(device == 1 || device == 3)
		s = 0;
	else if(device == 2)
		s = 4;
	else if (device == 0)                           // device 0: all off
	{
		gpio_set_value(spi->scs_gpios[0], 0);		
		gpio_set_value(spi->scs_gpios[1], 0);
		gpio_set_value(spi->scs_gpios[2], 0);
		gpio_set_value(spi->scs_gpios[3], 1);		// disable all ports
		gpio_set_value(spi->scs_gpios[4], 0);		// all off
		gpio_set_value(spi->scs_gpios[5], 0);
		gpio_set_value(spi->scs_gpios[6], 0);
		gpio_set_value(spi->scs_gpios[7], 1);		// disable all ports
		gpio_set_value(spi->dmxs_gpio, 0);	        // dmxs enable
		return;
	} else {
		gpio_set_value(spi->dmxs_gpio, 0);	        // dmxs enable
		return;
	}

	do {
		switch (port)
		{
		case 1:
			gpio_set_value(spi->scs_gpios[0+s], 0);	// Port 1, Device s/4+1 -> ON
			gpio_set_value(spi->scs_gpios[1+s], 0);
			gpio_set_value(spi->scs_gpios[2+s], 0);
			gpio_set_value(spi->scs_gpios[3+s], 0);
			break;
		case 2:
			gpio_set_value(spi->scs_gpios[0+s], 1);	// Port 2, Device s/4+1 -> ON
			gpio_set_value(spi->scs_gpios[1+s], 0);
			gpio_set_value(spi->scs_gpios[2+s], 0);
			gpio_set_value(spi->scs_gpios[3+s], 0);
			break;
		case 3:
			gpio_set_value(spi->scs_gpios[0+s], 0);	// Port 3, Device s/4+1 -> ON
			gpio_set_value(spi->scs_gpios[1+s], 1);
			gpio_set_value(spi->scs_gpios[2+s], 0);
			gpio_set_value(spi->scs_gpios[3+s], 0);
			break;
		case 4:
			gpio_set_value(spi->scs_gpios[0+s], 1);	// Port 4, Device s/4+1 -> ON
			gpio_set_value(spi->scs_gpios[1+s], 1);
			gpio_set_value(spi->scs_gpios[2+s], 0);
			gpio_set_value(spi->scs_gpios[3+s], 0);
			break;
		case 5:
			gpio_set_value(spi->scs_gpios[0+s], 0);	// Port 5, Device s/4+1 -> ON
			gpio_set_value(spi->scs_gpios[1+s], 0);
			gpio_set_value(spi->scs_gpios[2+s], 1);
			gpio_set_value(spi->scs_gpios[3+s], 0);
			break;
		case 6:
			gpio_set_value(spi->scs_gpios[0+s], 1);	// Port 6, Device s/4+1 -> ON
			gpio_set_value(spi->scs_gpios[1+s], 0);
			gpio_set_value(spi->scs_gpios[2+s], 1);
			gpio_set_value(spi->scs_gpios[3+s], 0);
			break;
		case 7:
			gpio_set_value(spi->scs_gpios[0+s], 0);	// Port 7, Device s/4+1 -> ON
			gpio_set_value(spi->scs_gpios[1+s], 1);
			gpio_set_value(spi->scs_gpios[2+s], 1);
			gpio_set_value(spi->scs_gpios[3+s], 0);
			break;
		case 8:
			gpio_set_value(spi->scs_gpios[0+s], 1);	// Port 8, Device s/4+1 -> ON
			gpio_set_value(spi->scs_gpios[1+s], 1);
			gpio_set_value(spi->scs_gpios[2+s], 1);
			gpio_set_value(spi->scs_gpios[3+s], 0);
			break;
		}
		s += 4;
	} while( (s <= 4) && (device == 3));
	
	gpio_set_value(spi->dmxs_gpio, 0);	            // dmxs enable
}



static s32 espi_driver_transfer(struct spi_device *dev, struct spi_transfer *xfer)
{
	struct spi_message msg;
	s32 status;
	
	xfer->tx_nbits = xfer->rx_nbits = SPI_NBITS_SINGLE;

	spi_message_init(&msg);
	spi_message_add_tail(xfer, &msg);

	status = spi_sync(dev, &msg);

	return status;
}



static s32 espi_driver_set_mode(struct spi_device *dev, u16 mode)
{
	s32 status;
	struct spi_transfer xfer;
	u8 tx = 0;

	xfer.tx_buf = &tx;
	xfer.rx_buf = NULL;
	xfer.len = 1;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;

	dev->mode = mode;
	status = spi_setup(dev);

	espi_driver_transfer(dev, &xfer);
	return status;
}





/*******************************************************************************
    encoder functions
*******************************************************************************/
static ssize_t espiencoder_write(   struct file *filp, 
                                    const char __user *buf, 
                                    size_t count, 
                                    loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}



static ssize_t espiencoder_read(    struct file *filp, 
                                    char __user *buf, 
                                    size_t count, 
                                    loff_t *f_pos)
{
	ssize_t status = 0;

	/* If in non-blocking mode and no data to read, return */
	if (filp->f_flags & O_NONBLOCK && encoder_delta == 0)
		return -EAGAIN;

	/* Sleep until there is data to read */
	if ((status = wait_event_interruptible(encoder_wqueue, encoder_delta != 0)))
		return status;

    /**@todo:   find a solution for cases, where playground crashes, 
                user turn on rotary like crazy, playground restarts and e.g. 
                volume jumps to max. */
	
    buf[0] = encoder_delta;
	encoder_delta = 0;
	status = 1;

	return status;
}

static s32 espiencoder_open(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static s32 espiencoder_release(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	return status;
}

static unsigned int espiencoder_poll(struct file *filp, poll_table *wait)
{

	unsigned int mask = 0;
	poll_wait(filp, &encoder_wqueue, wait);

	/* If there is data in buffer, reading is allowed
	 * Writing to buttons never makes sense, so dissallow */

	if (encoder_delta != 0)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations espiencoder_fops = {
		.owner = 	THIS_MODULE,
		.write = 	espiencoder_write,
		.read =		espiencoder_read,
		.open =		espiencoder_open,
		.release = 	espiencoder_release,
		.llseek = 	no_llseek,
		.poll =		espiencoder_poll,
};

static struct class *espiencoder_class;

static s32 espi_driver_encoder_setup(struct espi_driver *sb)
{
	s32 ret;
	
	encoder_delta = 0;

	ret = register_chrdev(ESPI_ENCODER_DEV_MAJOR, "spi", &espiencoder_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);

	espiencoder_class = class_create(THIS_MODULE, "espi-encoder");
	if(IS_ERR(espiencoder_class))
		pr_err("%s: unable to create class\n", __func__);

	device_create(espiencoder_class, sb->dev, MKDEV(ESPI_ENCODER_DEV_MAJOR, 0), sb, "espi_encoder");

	return 0;
}

static s32 espi_driver_encoder_cleanup(struct espi_driver *sb)
{
	device_destroy(espiencoder_class, MKDEV(ESPI_ENCODER_DEV_MAJOR, 0));
	class_destroy(espiencoder_class);
	unregister_chrdev(ESPI_ENCODER_DEV_MAJOR, "spi");

	return 0;
}

static void espi_driver_encoder_poll(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u8 rx_buff[3];
	u8 tx_buff[3];

	tx_buff[0] = 0xAA;
	
	xfer.tx_buf = tx_buff;
	xfer.rx_buf = rx_buff;
	xfer.len = 3;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;
	
	espi_driver_scs_select((struct espi_driver*)p, ESPI_LARGE_DISPLAY_PORT, 3);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_LARGE_DISPLAY_PORT, 0);
	
	if(rx_buff[2] != 0 ) {
		if (!(encoder_delta + (s8)rx_buff[2] > 127) &&
		    !(encoder_delta + (s8)rx_buff[2] < -128)) {
			encoder_delta += (s8) rx_buff[2];

			//printk("encoder delta: %d\n", (s8)encoder_delta);
			wake_up_interruptible(&encoder_wqueue);
		}
	}
}




/*******************************************************************************
    adc functions
*******************************************************************************/
static ssize_t espiadc_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static ssize_t espiadc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	int i;
	u16* vals = (u16*) buf;
	
	if(count >= (ESPI_ADC_320X * sizeof(u16))) {
		for(i=0; i < ESPI_ADC_320X; i++)
			vals[i] = adc_channel_val[i];
		status = ESPI_ADC_320X;
	}
	
	return status;
}

static s32 espiadc_open(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static s32 espiadc_release(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	return status;
}

static const struct file_operations espiadc_fops = {
		.owner = 	THIS_MODULE,
		.write = 	espiadc_write,
		.read =		espiadc_read,
		.open =		espiadc_open,
		.release = 	espiadc_release,
		.llseek = 	no_llseek,
};

static struct class *espiadc_class;

static s32 espi_driver_adc_setup(struct espi_driver *sb)
{
	s32 i, ret;

	for(i=0; i < ESPI_ADC_320X; i++) {
		adc_channel_val[i] = 0x0000;
	}
	
	ret = register_chrdev(ESPI_ADC_DEV_MAJOR, "spi", &espiadc_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);

	espiadc_class = class_create(THIS_MODULE, "adc-320x");
	if(IS_ERR(espiadc_class))
		pr_err("%s: unable to create class\n", __func__);

	device_create(espiadc_class, sb->dev, MKDEV(ESPI_ADC_DEV_MAJOR, 0), sb, "mcp320x-adc");

	return 0;
}

static s32 espi_driver_adc_cleanup(struct espi_driver *sb)
{
	device_destroy(espiadc_class, MKDEV(ESPI_ADC_DEV_MAJOR, 0));
	class_destroy(espiadc_class);
	unregister_chrdev(ESPI_ADC_DEV_MAJOR, "spi");

	return 0;
}

static void espi_driver_adc_poll(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u8 rx_buff[3];
	u8 tx_buff[3];
	u8 i;
	u16 tmp;

	for(i=0; i < ESPI_ADC_320X; i++) {
		tx_buff[0] = 0x6 | (i >> 2);
		tx_buff[1] = i << 6;
		tx_buff[2] = 0;
		
		xfer.tx_buf = tx_buff;
		xfer.rx_buf = rx_buff;
		xfer.len = 3;
		xfer.bits_per_word = 8;
		xfer.delay_usecs = 0;
		xfer.speed_hz = ESPI_SPI_SPEED;
		
		espi_driver_scs_select((struct espi_driver*)p, ESPI_ADC_PORT, 1);
		espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
		espi_driver_scs_select((struct espi_driver*)p, ESPI_ADC_PORT, 0);
		
		tmp = ((rx_buff[1] & 0xF)<<8) | rx_buff[2];
		adc_channel_val[i] = tmp;
	}
}


/*******************************************************************************
    attenuator functions
*******************************************************************************/
static ssize_t attenuator_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;

	if(buf[0] >= 3 && count != 2)
		return -EFAULT;
		
	if(buf[1] != attenuator_channel_val[(const unsigned char)buf[0]]) {
		mutex_lock(&attenuator_lock);
		attenuator_channel_val[(const unsigned char)buf[0]] = buf[1];
		attenuator_channel_updated[(const unsigned char)buf[0]] = 1;
		status = 2;
		mutex_unlock(&attenuator_lock);
	}
	
	return status;
}

static ssize_t attenuator_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static s32 attenuator_open(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static s32 attenuator_release(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	return status;
}

static const struct file_operations attenuator_fops = {
		.owner = 	THIS_MODULE,
		.write = 	attenuator_write,
		.read =		attenuator_read,
		.open =		attenuator_open,
		.release = 	attenuator_release,
		.llseek = 	no_llseek,
};

static struct class *attenuator_class;

static s32 espi_driver_attenuator_setup(struct espi_driver *sb)
{
	s32 i, ret;

	for(i=0; i<3; i++) {
		attenuator_channel_updated[i] = 0;
		attenuator_channel_val[i] = 0xFF;
	}
	
	ret = register_chrdev(ESPI_ATTENUATOR_DEV_MAJOR, "spi", &attenuator_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);

	attenuator_class = class_create(THIS_MODULE, "attenuator-LM1972");
	if(IS_ERR(attenuator_class))
		pr_err("%s: unable to create class\n", __func__);

	device_create(attenuator_class, sb->dev, MKDEV(ESPI_ATTENUATOR_DEV_MAJOR, 0), sb, "lm1972-attenuator");

	return 0;
}

static s32 espi_driver_attenuator_cleanup(struct espi_driver *sb)
{
	device_destroy(attenuator_class, MKDEV(ESPI_ATTENUATOR_DEV_MAJOR, 0));
	class_destroy(attenuator_class);
	unregister_chrdev(ESPI_ATTENUATOR_DEV_MAJOR, "spi");

	return 0;
}

static void espi_driver_attenuator_poll(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u8 tmp_buff[2];
	u32 i;

	for(i=0; i<3; i++) {
		if(attenuator_channel_updated[i]) {
			mutex_lock(&attenuator_lock);
			tmp_buff[0] = i;
			tmp_buff[1] = attenuator_channel_val[i];
			attenuator_channel_updated[i] = 0;
			mutex_unlock(&attenuator_lock);
			
			xfer.tx_buf = tmp_buff;
			xfer.rx_buf = NULL;
			xfer.len = 2;
			xfer.bits_per_word = 8;
			xfer.delay_usecs = 0;
			xfer.speed_hz = ESPI_SPI_SPEED;
			
			espi_driver_scs_select((struct espi_driver*)p, ESPI_ATTENUATOR_PORT, 1);
			espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
			espi_driver_scs_select((struct espi_driver*)p, ESPI_ATTENUATOR_PORT, 0);
			
		}
	}
}



/*******************************************************************************
    ssd1322 functions (boled)
*******************************************************************************/
static ssize_t ssd1322_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	u32 i, j, width, height, x, y, cnt;
	ssize_t status = 0;
	
	if(count < 4)
		return -EFAULT;
	
	x = buf[0];
	y = buf[1];
	width = buf[2];
	height = buf[3];
	
	if(count < (4 + 2*width*height))
		return -EFAULT;
		
	mutex_lock(&ssd1322_tmp_buff_lock);
	cnt = 4;
	for(j = x; j < (x + width); j++)
		for(i= j*2*SSD1322_BUFF_HEIGHT + 2*y; i<(j*2*SSD1322_BUFF_HEIGHT + 2*(y + height)); i++)
			ssd1322_tmp_buff[i] = buf[cnt++];
	ssd1322_buff_updated = 1;
	status = count;
	mutex_unlock(&ssd1322_tmp_buff_lock);
		
	return status;
}

static ssize_t ssd1322_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static int ssd1322_open(struct inode *inode, struct file *filp)
{
    	int status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static int ssd1322_release(struct inode *inode, struct file *filp)
{
    	int status = 0;
	return status;
}

static const struct file_operations ssd1322_fops = {
	.owner = 	THIS_MODULE,
	.write = 	ssd1322_write,
	.read =		ssd1322_read,
	.open =		ssd1322_open,
	.release = 	ssd1322_release,
	.llseek = 	no_llseek,
};

static struct class *ssd1322_class;

static void ssd1322_command(struct espi_driver* sb, u8 cmd, u8* data, u16 len)
{
	struct spi_transfer xfer;
	u8 command = cmd;
	
	xfer.tx_buf = &command;
	xfer.rx_buf = NULL;
	xfer.len = 1;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SSD1322_SPEED;
	
	gpio_set_value(sb->sap_gpio, 0);
	espi_driver_transfer(sb->spidev, &xfer);
	gpio_set_value(sb->sap_gpio, 1);
	
	if(data == NULL)
		return;
		
	xfer.tx_buf = data;
	xfer.rx_buf = NULL;
	xfer.len = len;
	
	espi_driver_transfer(sb->spidev, &xfer);
}

static void ssd1322_data(struct espi_driver* sb, u8* data, u32 len)
{
	struct spi_transfer xfer;
	
	xfer.tx_buf = data;
	xfer.rx_buf = NULL;
	xfer.len = len;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SSD1322_SPEED;
	
	espi_driver_transfer(sb->spidev, &xfer);
}

static s32 espi_driver_ssd1322_setup(struct espi_driver *sb)
{
	s32 ret, i;
	u8 data[2];
	
	ssd1322_buff = kcalloc(SSD1322_BUFF_SIZE,sizeof(u8), GFP_KERNEL);
	if (!ssd1322_buff)
		return -ENOMEM;
		
	ssd1322_tmp_buff = kcalloc(SSD1322_BUFF_SIZE,sizeof(u8), GFP_KERNEL);
	if (!ssd1322_tmp_buff)
		return -ENOMEM;
		
	ssd1322_buff_updated = 0;
	
	for(i=0; i<SSD1322_BUFF_SIZE; i++)
		ssd1322_buff[i] = ssd1322_tmp_buff[i] = 0x00;
	
	/** DISPLAY INITIALIZATION *************/
	espi_driver_scs_select(sb, ESPI_LARGE_DISPLAY_PORT, 2);
	
	data[0] = 0x12;
	ssd1322_command(sb, SSD1322_SET_CMD_LOCK, data, 1);
	ssd1322_command(sb, SSD1322_SET_DISP_OFF, NULL, 0);
	data[0] = 0x1C;
	data[1] = 0x5B;
	ssd1322_command(sb, SSD1322_SET_COL_ADDR, data, 2);
	data[0] = 0x00;
	data[1] = 0x3F;
	ssd1322_command(sb, SSD1322_SET_ROW_ADDR, data, 2);
	data[0] = 0x91;
	ssd1322_command(sb, SSD1322_SET_DISP_CLK, data, 1);
	data[0] = 0x3F;
	ssd1322_command(sb, SSD1322_SET_MUX_RATIO, data, 1);
	data[0] = 0x00;
	ssd1322_command(sb, SSD1322_SET_DISP_OFFSET, data, 1);
	data[0] = 0x00;
	ssd1322_command(sb, SSD1322_SET_START_LINE, data, 1);
	data[0] = 0x07;	//0x06 -> horizontal
	data[1] = 0x11;
	ssd1322_command(sb, SSD1322_SET_REMAP, data, 2);	/** remapping */
	data[0] = 0x00;
	ssd1322_command(sb, SSD1322_SET_GPIO, data, 1);
	data[0] = 0x01;
	ssd1322_command(sb, SSD1322_SET_FUNC_SEL, data, 1);
	data[0] = 0xA0;
	data[1] = 0xFD;
	ssd1322_command(sb, SSD1322_SET_DISP_ENH_A, data, 2);
	data[0] = 0x9F;
	ssd1322_command(sb, SSD1322_SET_CONTRAST_CUR, data, 1);
	data[0] = 0x0F;
	ssd1322_command(sb, SSD1322_SET_MASTER_CUR, data, 1);
	ssd1322_command(sb, SSD1322_SET_LINEAR_GST, NULL, 0);
	data[0] = 0xE2;
	ssd1322_command(sb, SSD1322_SET_PHASE_LEN, data, 1);
	data[0] = 0x20;
	ssd1322_command(sb, SSD1322_SET_DISP_ENH_B, data, 1);
	data[0] = 0x1F;
	ssd1322_command(sb, SSD1322_SET_PRECH_VOL, data, 1);
	data[0] = 0x08;
	ssd1322_command(sb, SSD1322_SET_PRECH_PER, data, 1);
	data[0] = 0x07;
	ssd1322_command(sb, SSD1322_SET_VCOMH, data, 1);
	ssd1322_command(sb, SSD1322_SET_DISP_MODE | 0x02, NULL, 0);
	ssd1322_command(sb, SSD1322_SET_PARTIAL_DISP | 0x01, NULL, 0);
	ssd1322_command(sb, SSD1322_SET_DISP_ON, NULL, 0);
	
	data[0] = 0x1C;
	data[1] = 0x1C + 64 - 1;
	ssd1322_command(sb, SSD1322_SET_COL_ADDR, data, 2);
	data[0] = 0x00;
	data[1] = 0x3F;
	ssd1322_command(sb, SSD1322_SET_ROW_ADDR, data, 2);
	ssd1322_command(sb, SSD1322_WRITE_RAM, NULL, 0);
	ssd1322_data(sb, ssd1322_buff, SSD1322_BUFF_SIZE);
	
	espi_driver_scs_select(sb, ESPI_LARGE_DISPLAY_PORT, 0);
	
	/**** prepare device ***/
	ret = register_chrdev(ESPI_SSD1322_DEV_MAJOR, "spi", &ssd1322_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);
		
	ssd1322_class = class_create(THIS_MODULE, "ssd1322-oled");
	if(IS_ERR(ssd1322_class))
		pr_err("%s: unable to create class\n", __func__);
		
	device_create(ssd1322_class, sb->dev, MKDEV(ESPI_SSD1322_DEV_MAJOR, 0), sb, "ssd1322");
	
	return 0;
}

static s32 espi_driver_ssd1322_cleanup(struct espi_driver *sb)
{
	kfree(ssd1322_buff);
	kfree(ssd1322_tmp_buff);
		
	device_destroy(ssd1322_class, MKDEV(ESPI_SSD1322_DEV_MAJOR, 0));
	class_destroy(ssd1322_class);
	unregister_chrdev(ESPI_SSD1322_DEV_MAJOR, "spi");
	
	return 0;
}

static void espi_driver_poll_boled_force_write(struct espi_driver *p)
{
	espi_driver_scs_select(p, ESPI_LARGE_DISPLAY_PORT, 2);
	ssd1322_data(p, ssd1322_buff, SSD1322_BUFF_SIZE);
	espi_driver_scs_select(p, ESPI_LARGE_DISPLAY_PORT, 0);
}


static void espi_driver_ssd1322_poll(struct espi_driver *p)
{
	u32 i, update = 0;
	
	mutex_lock(&ssd1322_tmp_buff_lock);
	if(ssd1322_buff_updated) {
		for(i=0; i<SSD1322_BUFF_SIZE; i++)
			ssd1322_buff[i] = ssd1322_tmp_buff[i];
		update = 1;
		ssd1322_buff_updated = 0;
	}
	mutex_unlock(&ssd1322_tmp_buff_lock);
	
	if(update == 0)
		return;
	
	espi_driver_scs_select(p, ESPI_LARGE_DISPLAY_PORT, 2);
	ssd1322_data(p, ssd1322_buff, SSD1322_BUFF_SIZE);
	espi_driver_scs_select(p, ESPI_LARGE_DISPLAY_PORT, 0);
}



/*******************************************************************************
    ssd1305 functions (soled)
*******************************************************************************/
static ssize_t ssd1305_write(   struct file *filp, 
                                const char __user *buf,     
                                size_t count, 
                                loff_t *f_pos)
{
	ssize_t status = 0;
	
	if(count != SSD1305_BUFF_SIZE)
		return status;
		
	mutex_lock(&ssd1305_tmp_buff_lock);
	status = copy_from_user(ssd1305_tmp_buff, buf, count);
	mutex_unlock(&ssd1305_tmp_buff_lock);
	
	if(status == 0)
		status = count;
	else
		status = -EFAULT;
		
	return status;
}

static ssize_t ssd1305_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static int ssd1305_open(struct inode *inode, struct file *filp)
{
    	int status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static int ssd1305_release(struct inode *inode, struct file *filp)
{
    	int status = 0;
	return status;
}

static const struct file_operations ssd1305_fops = {
	.owner = 	THIS_MODULE,
	.write = 	ssd1305_write,
	.read =		ssd1305_read,
	.open =		ssd1305_open,
	.release = 	ssd1305_release,
	.llseek = 	no_llseek,
};

static struct class *ssd1305_class;

static s32 espi_driver_ssd1305_setup(struct espi_driver *sb)
{
	struct spi_transfer xfer;
	s32 i, ret;
	
	ssd1305_buff = kcalloc(SSD1305_BUFF_SIZE,sizeof(u8), GFP_KERNEL);
	if (!ssd1305_buff)
		return -ENOMEM;
	ssd1305_tmp_buff = kcalloc(SSD1305_BUFF_SIZE,sizeof(u8), GFP_KERNEL);
	if (!ssd1305_tmp_buff)
		return -ENOMEM;
	
	/** DISPLAY INITIALIZATION *************/
	i = 0;
	ssd1305_buff[i++] = SSD1305_DISP_OFF;
	ssd1305_buff[i++] = SSD1305_SET_RATIO_OSC;
	ssd1305_buff[i++] = 0xA0;
	ssd1305_buff[i++] = SSD1305_SET_COL_ADDR;
	ssd1305_buff[i++] = 0;
	ssd1305_buff[i++] = 131;
	ssd1305_buff[i++] = SSD1305_SET_AREA_COLOR;
	ssd1305_buff[i++] = 0x05;
	ssd1305_buff[i++] = SSD1305_SET_SEG_REMAP1;
	ssd1305_buff[i++] = SSD1305_SET_SCAN_NOR;
	ssd1305_buff[i++] = SSD1305_SET_OFFSET;
	ssd1305_buff[i++] = 0x20;
	ssd1305_buff[i++] = SSD1305_SET_CONTRAST;
	ssd1305_buff[i++] = 0xFF;
	ssd1305_buff[i++] = SSD1305_SET_CHARGE;
	ssd1305_buff[i++] = 0x82;
	ssd1305_buff[i++] = SSD1305_SET_VCOM;
	ssd1305_buff[i++] = 0x3C;
	ssd1305_buff[i++] = SSD1305_EON_OFF;
	ssd1305_buff[i++] = SSD1305_DISP_NOR;
	ssd1305_buff[i++] = SSD1305_MEM_ADDRESSING;
	ssd1305_buff[i++] = 0x00;	// Horizontal Addressing mode
	ssd1305_buff[i++] = SSD1305_DISP_ON;
	ssd1305_buff[i++] = SSD1305_SET_PAGE;
	ssd1305_buff[i++] = SSD1305_SET_PAGE_ADDR;
	ssd1305_buff[i++] = 0x00;
	ssd1305_buff[i++] = 0x03;
	ssd1305_buff[i++] = SSD1305_SET_COL_HI;
	ssd1305_buff[i++] = SSD1305_SET_COL_LO;
	
	xfer.tx_buf = ssd1305_buff;
	xfer.rx_buf = NULL;
	xfer.len = i;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;
	
	gpio_set_value(sb->sap_gpio, 0);
	espi_driver_scs_select(sb, ESPI_SMALL_DISPLAY_PORT, 1);
	espi_driver_transfer(sb->spidev, &xfer);
	espi_driver_scs_select(sb, ESPI_SMALL_DISPLAY_PORT, 0);
	gpio_set_value(sb->sap_gpio, 1);

	for(i=0; i<SSD1305_BUFF_SIZE; i++)
		ssd1305_buff[i] = ssd1305_tmp_buff[i] = 0;
		
	xfer.tx_buf = ssd1305_buff;
	xfer.rx_buf = NULL;
	xfer.len = SSD1305_BUFF_SIZE;
	espi_driver_scs_select(sb, ESPI_SMALL_DISPLAY_PORT, 1);
	espi_driver_transfer(sb->spidev, &xfer);
	espi_driver_scs_select(sb, ESPI_SMALL_DISPLAY_PORT, 0);
	
	/**** prepare device ***/
	ret = register_chrdev(ESPI_SSD1305_DEV_MAJOR, "spi", &ssd1305_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);
		
	ssd1305_class = class_create(THIS_MODULE, "ssd1305-oled");
	if(IS_ERR(ssd1305_class))
		pr_err("%s: unable to create class\n", __func__);
		
	device_create(ssd1305_class, sb->dev, MKDEV(ESPI_SSD1305_DEV_MAJOR, 0), sb, "ssd1305");
	
	return 0;
}

static s32 espi_driver_ssd1305_cleanup(struct espi_driver *sb)
{
	kfree(ssd1305_buff);
	kfree(ssd1305_tmp_buff);
	
	device_destroy(ssd1305_class, MKDEV(ESPI_SSD1305_DEV_MAJOR, 0));
	class_destroy(ssd1305_class);
	unregister_chrdev(ESPI_SSD1305_DEV_MAJOR, "spi");
	
	return 0;
}

static void espi_driver_poll_soled_force_write(struct espi_driver *p)
{
	struct spi_transfer xfer;

	xfer.tx_buf = ssd1305_buff;
	xfer.rx_buf = NULL;
	xfer.len = SSD1305_BUFF_SIZE;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;
	
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SMALL_DISPLAY_PORT, 1);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SMALL_DISPLAY_PORT, 0);
}

static void espi_driver_ssd1305_poll(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u32 i;
	u8 update = 0;

	mutex_lock(&ssd1305_tmp_buff_lock);
	for(i=0; i<SSD1305_BUFF_SIZE; i++) {
		if(ssd1305_buff[i] ^ ssd1305_tmp_buff[i]) {
			ssd1305_buff[i] = ssd1305_tmp_buff[i];
			update = 1;
		}
	}
	mutex_unlock(&ssd1305_tmp_buff_lock);
	
	if(update == 0)
		return;
	
	xfer.tx_buf = ssd1305_buff;
	xfer.rx_buf = NULL;
	xfer.len = SSD1305_BUFF_SIZE;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;
	
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SMALL_DISPLAY_PORT, 1);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SMALL_DISPLAY_PORT, 0);
}


/*******************************************************************************
    ribbon leds functions
*******************************************************************************/
static ssize_t rbled_write( struct file *filp, 
                            const char __user *buf, 
                            size_t count, 
                            loff_t *f_pos)
{
	ssize_t status = 0;
	u32 i;
	u8 val, led_id;
	u8 rot[] = {0,2,1,3};

	for(i=0; i<count; i++) {
		val = buf[i] >> 6;
		led_id = buf[i] & 0x3F;
		val = rot[val & 0x3];

		rb_led_new_st[RIBBON_LED_STATES_SIZE - 1 - led_id/4] &= ~(0x3 << ((led_id%4)*2)); 
		rb_led_new_st[RIBBON_LED_STATES_SIZE - 1 - led_id/4] |= val << ((led_id%4)*2);
	}
	status = count;
	return status;
}

static ssize_t rbled_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static s32 rbled_open(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static s32 rbled_release(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	return status;
}

static const struct file_operations rbled_fops = {
		.owner = 	THIS_MODULE,
		.write = 	rbled_write,
		.read =		rbled_read,
		.open =		rbled_open,
		.release = 	rbled_release,
		.llseek = 	no_llseek,
};

static struct class *rbled_class;

static s32 espi_driver_rb_leds_setup(struct espi_driver *sb)
{
	s32 i, ret;

	rb_led_st = kcalloc(RIBBON_LED_STATES_SIZE,sizeof(u8), GFP_KERNEL);
	if (!led_st)
		return -ENOMEM;
	rb_led_new_st = kcalloc(RIBBON_LED_STATES_SIZE,sizeof(u8), GFP_KERNEL);
	if (!rb_led_new_st)
		return -ENOMEM;

	for(i=0; i<RIBBON_LED_STATES_SIZE; i++)
		rb_led_new_st[i] = rb_led_st[i] = 0;

	ret = register_chrdev(ESPI_RIBBON_LED_DEV_MAJOR, "spi", &rbled_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);

	rbled_class = class_create(THIS_MODULE, "ribbon-led");
	if(IS_ERR(rbled_class))
		pr_err("%s: unable to create class\n", __func__);

	device_create(rbled_class, sb->dev, MKDEV(ESPI_RIBBON_LED_DEV_MAJOR, 0), sb, "ribbon_led");

	return 0;
}

static s32 espi_driver_rb_leds_cleanup(struct espi_driver *sb)
{
	kfree(rb_led_st);
	kfree(rb_led_new_st);

	device_destroy(rbled_class, MKDEV(ESPI_RIBBON_LED_DEV_MAJOR, 0));
	class_destroy(rbled_class);
	unregister_chrdev(ESPI_RIBBON_LED_DEV_MAJOR, "spi");

	return 0;
}

static void espi_driver_rb_leds_poll(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u32 i, update = 0;

	for(i=0; i<RIBBON_LED_STATES_SIZE; i++) {
		if(rb_led_st[i] ^ rb_led_new_st[i]) {
			rb_led_st[i] = rb_led_new_st[i];
			update = 1;
		}
	}

	if(update == 0)
		return;

	xfer.tx_buf = rb_led_st;
	xfer.rx_buf = NULL;
	xfer.len = RIBBON_LED_STATES_SIZE;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;

	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_RIBBON_LEDS_PORT, 1);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_RIBBON_LEDS_PORT, 0);
}



static void espi_driver_rb_leds_poll_force_write(struct espi_driver *p)
{
	struct spi_transfer xfer;

	xfer.tx_buf = rb_led_st;
	xfer.rx_buf = NULL;
	xfer.len = RIBBON_LED_STATES_SIZE;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;

	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_RIBBON_LEDS_PORT, 2);   //ESPI_DEVICE_TOP_COVER_RIBBON_LEDS
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_RIBBON_LEDS_PORT, 0);
}



/*******************************************************************************
    led functions
*******************************************************************************/
static ssize_t espiled_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
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

static ssize_t espiled_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static s32 espiled_open(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static s32 espiled_release(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	return status;
}

static const struct file_operations espiled_fops = {
		.owner = 	THIS_MODULE,
		.write = 	espiled_write,
		.read =		espiled_read,
		.open =		espiled_open,
		.release = 	espiled_release,
		.llseek = 	no_llseek,
};

static struct class *espiled_class;

static s32 espi_driver_leds_setup(struct espi_driver *sb)
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

	ret = register_chrdev(ESPI_LED_DEV_MAJOR, "spi", &espiled_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);

	espiled_class = class_create(THIS_MODULE, "espi-led");
	if(IS_ERR(espiled_class))
		pr_err("%s: unable to create class\n", __func__);

	device_create(espiled_class, sb->dev, MKDEV(ESPI_LED_DEV_MAJOR, 0), sb, "espi_led");

	return 0;
}

static s32 espi_driver_leds_cleanup(struct espi_driver *sb)
{
	kfree(led_st);
	kfree(led_new_st);

	device_destroy(espiled_class, MKDEV(ESPI_LED_DEV_MAJOR, 0));
	class_destroy(espiled_class);
	unregister_chrdev(ESPI_LED_DEV_MAJOR, "spi");

	return 0;
}

static void espi_driver_leds_poll(struct espi_driver *p)
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

	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_BTN_LED_PANELS_PORT, 1);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_BTN_LED_PANELS_PORT, 0);
}



/*******************************************************************************
    buttons functions
*******************************************************************************/
static ssize_t espibtn_write(   struct file *filp, 
                                const char __user *buf, 
                                size_t count, 
                                loff_t *f_pos)
{
	ssize_t status = 0;
	return status;
}

static ssize_t espibtn_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t status = 0;

	/* If in non-blocking mode and no data to read, return */
	if (filp->f_flags & O_NONBLOCK && btn_buff_head == btn_buff_tail)
		return -EAGAIN;

	/* Sleep until there is data to read */
	if ((status = wait_event_interruptible(btn_wqueue, btn_buff_head != btn_buff_tail)))
		return status;

	mutex_lock(&btn_buff_tail_lock);

	// xor 0x80, so we get 1 on btn down and 0 on btn up
	buf[0] = button_buff[btn_buff_tail] ^ 0x80;
	btn_buff_tail = (btn_buff_tail+1)%BUTTON_BUFFER_SIZE;
	status = 1;
	mutex_unlock(&btn_buff_tail_lock);

	return status;
}

static int espibtn_open(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	nonseekable_open(inode, filp);
	return status;
}

static int espibtn_release(struct inode *inode, struct file *filp)
{
	s32 status = 0;
	return status;
}

static unsigned int espibtn_poll(struct file *filp, poll_table *wait)
{

	unsigned int mask = 0;
	poll_wait(filp, &btn_wqueue, wait);

	/* If there is data in buffer, reading is allowed
	 * Writing to buttons never makes sense, so dissallow */

	if (btn_buff_tail != btn_buff_head)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations espibtn_fops = {
		.owner = 	THIS_MODULE,
		.write = 	espibtn_write,
		.read =		espibtn_read,
		.open =		espibtn_open,
		.release = 	espibtn_release,
		.llseek = 	no_llseek,
		.poll =		espibtn_poll,
};

static struct class *espibtn_class;

static s32 espi_driver_buttons_setup(struct espi_driver *sb)
{
	s32 i, ret;

	btn_sm1 = kcalloc(BUTTON_STATES_SIZE,sizeof(u8), GFP_KERNEL);
	if (!btn_sm1)
		return -ENOMEM;

	btn_st = kcalloc(BUTTON_STATES_SIZE,sizeof(u8), GFP_KERNEL);
	if (!btn_st)
		return -ENOMEM;

	for(i=0; i<BUTTON_STATES_SIZE; i++)
		btn_sm1[i] = btn_st[i] = 0xFF;

	button_buff = kcalloc(BUTTON_BUFFER_SIZE,sizeof(u8), GFP_KERNEL);
	if (!button_buff)
		return -ENOMEM;

	btn_buff_head = btn_buff_tail = 0;

	ret = register_chrdev(ESPI_BUTTON_DEV_MAJOR, "spi", &espibtn_fops);
	if (ret < 0)
		pr_err("%s: problem at register_chrdev\n", __func__);

	espibtn_class = class_create(THIS_MODULE, "espi-button");
	if(IS_ERR(espibtn_class))
		pr_err("%s: unable to create class\n", __func__);

	device_create(espibtn_class, sb->dev, MKDEV(ESPI_BUTTON_DEV_MAJOR, 0), sb, "espi_buttons");

	return 0;
}

static s32 espi_driver_buttons_cleanup(struct espi_driver *sb)
{
	kfree(btn_sm1);
	kfree(btn_st);
	kfree(button_buff);

	device_destroy(espibtn_class, MKDEV(ESPI_BUTTON_DEV_MAJOR, 0));
	class_destroy(espibtn_class);
	unregister_chrdev(ESPI_BUTTON_DEV_MAJOR, "spi");

	return 0;
}

static void espi_driver_poll_buttons_selection(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u8 rx[BUTTON_STATES_SIZE];
	u8 i, j, xor, bit, btn_id;

	xfer.tx_buf = NULL;
	xfer.rx_buf = rx;
	xfer.len = BUTTON_BYTES_GENERAL_PANELS;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;

	espi_driver_set_mode(((struct espi_driver*)p)->spidev, SPI_MODE_3);

	/** read general panels */
	espi_driver_scs_select((struct espi_driver*)p, ESPI_BTN_LED_PANELS_PORT, 2);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_BTN_LED_PANELS_PORT, 0);


	espi_driver_set_mode(((struct espi_driver*)p)->spidev, SPI_MODE_0);

}

static void espi_driver_poll_buttons_play(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u8 rx[BUTTON_STATES_SIZE];
	u8 i, j, xor, bit, btn_id;

	xfer.tx_buf = NULL;
	xfer.rx_buf = rx;
	xfer.len = BUTTON_BYTES_GENERAL_PANELS;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;

	espi_driver_set_mode(((struct espi_driver*)p)->spidev, SPI_MODE_3);

	/** read small oled panel */
	xfer.tx_buf = NULL;
	xfer.rx_buf = rx + BUTTON_BYTES_GENERAL_PANELS + BUTTON_BYTES_CENTRAL_PANEL;
	xfer.len = BUTTON_BYTES_SOLED_PANEL;
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SMALL_DISPLAY_PORT, 2);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SMALL_DISPLAY_PORT, 0);
	

	espi_driver_set_mode(((struct espi_driver*)p)->spidev, SPI_MODE_0);

}

static void espi_driver_poll_buttons_edit(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u8 rx[BUTTON_STATES_SIZE];
	u8 i, j, xor, bit, btn_id;

	xfer.tx_buf = NULL;
	xfer.rx_buf = rx;
	xfer.len = BUTTON_BYTES_GENERAL_PANELS;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;

	espi_driver_set_mode(((struct espi_driver*)p)->spidev, SPI_MODE_3);

	/** read central (big oled) panel */
	xfer.tx_buf = NULL;
	xfer.rx_buf = rx + BUTTON_BYTES_GENERAL_PANELS;
	xfer.len = BUTTON_BYTES_CENTRAL_PANEL;
	espi_driver_scs_select((struct espi_driver*)p, ESPI_LARGE_DISPLAY_PORT, 1);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_LARGE_DISPLAY_PORT, 0);


	espi_driver_set_mode(((struct espi_driver*)p)->spidev, SPI_MODE_0);

}

static void espi_driver_pollbuttons(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u8 rx[BUTTON_STATES_SIZE];
	u8 i, j, xor, bit, btn_id;

	xfer.tx_buf = NULL;
	xfer.rx_buf = rx;
	xfer.len = BUTTON_BYTES_GENERAL_PANELS;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;

	espi_driver_set_mode(((struct espi_driver*)p)->spidev, SPI_MODE_3);

	/** read general panels */
	espi_driver_scs_select((struct espi_driver*)p, ESPI_BTN_LED_PANELS_PORT, 2);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_BTN_LED_PANELS_PORT, 0);

	/** read central (big oled) panel */
	xfer.tx_buf = NULL;
	xfer.rx_buf = rx + BUTTON_BYTES_GENERAL_PANELS;
	xfer.len = BUTTON_BYTES_CENTRAL_PANEL;
	espi_driver_scs_select((struct espi_driver*)p, ESPI_LARGE_DISPLAY_PORT, 1);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_LARGE_DISPLAY_PORT, 0);

	/** read small oled panel */
	xfer.tx_buf = NULL;
	xfer.rx_buf = rx + BUTTON_BYTES_GENERAL_PANELS + BUTTON_BYTES_CENTRAL_PANEL;
	xfer.len = BUTTON_BYTES_SOLED_PANEL;
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SMALL_DISPLAY_PORT, 2);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 0);
	gpio_set_value(((struct espi_driver *)p)->sap_gpio, 1);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_SMALL_DISPLAY_PORT, 0);
      
      /***** MASKING ******/
	rx[BUTTON_BYTES_GENERAL_PANELS + 1] |= 0x30;
	rx[BUTTON_BYTES_GENERAL_PANELS + BUTTON_BYTES_CENTRAL_PANEL] |= 0x0F;

	/** check read states */
	for(i=0; i < BUTTON_STATES_SIZE; i++) {
		xor = (btn_sm1[i] & rx[i] & (~btn_st[i])) | (~(btn_sm1[i] | rx[i]) & btn_st[i]);
		if(xor){
			for(j=0; j < 8; j++){
				bit = xor & (1<<j);
				if(bit & btn_st[i])	// change 1->0
					btn_id = i*8 + 7-j;
				else if(bit)		// change 0->1
					btn_id = (i*8 + 7-j) | (1<<7);
				else
					continue;

				btn_st[i] ^= bit;

				if(((btn_buff_head+1)%BUTTON_BUFFER_SIZE) != btn_buff_tail) {
					button_buff[btn_buff_head] = btn_id;
					btn_buff_head = (btn_buff_head+1)%BUTTON_BUFFER_SIZE;
					wake_up_interruptible(&btn_wqueue);
				}
				//printk("button-change = %x\n", btn_id);
			}	
		}
		btn_sm1[i] = rx[i];
	}

	espi_driver_set_mode(((struct espi_driver*)p)->spidev, SPI_MODE_0);
}



static void espi_driver_dbg_scan_scs(struct espi_driver *p)
{
    static u8 port_cnt   = 1;    
    static u8 device_cnt = 1;
    
    espi_driver_scs_select((struct espi_driver *)p, port_cnt + 1, 0);  // all off
    
    device_cnt = ((device_cnt + 1) % 3);
    if( device_cnt == 0)
    {
        port_cnt = ((port_cnt + 1) % 8);
    }
    
    printk("port: %i, device: %i \n", port_cnt+1, device_cnt+1);
    
    espi_driver_scs_select((struct espi_driver *)p, port_cnt + 1, device_cnt);  
}


/*******************************************************************************
    SCHEDULER
*******************************************************************************/
#if 1 // daniels scheduler
static void espi_driver_poll(struct delayed_work *p)
{
	queue_delayed_work(workqueue, p, msecs_to_jiffies(8));
    espi_driver_dbg_scan_scs((struct espi_driver *)p);

#if 0
	espi_driver_rb_leds_poll_force_write((struct espi_driver *)p);// Tut nüscht
    espi_driver_poll_soled_force_write((struct espi_driver *)p);// Tut nüscht
	espi_driver_leds_poll((struct espi_driver *)p);
	espi_driver_poll_buttons_selection((struct espi_driver *)p);
	espi_driver_poll_buttons_play((struct espi_driver *)p);
	espi_driver_poll_buttons_edit((struct espi_driver *)p);
    espi_driver_encoder_poll((struct espi_driver *)p);
#endif

	((struct espi_driver *)p)->poll_stage = (((struct espi_driver *)p)->poll_stage + 1)%8;
}
#endif


#if 0 // nemanjas original scheduler
static void espi_driver_poll(struct delayed_work *p)
{
	queue_delayed_work(workqueue, p, msecs_to_jiffies(8));

	switch(((struct espi_driver *)p)->poll_stage)
	{
	case 0:
	case 2:
	case 4:
	case 6:
		espi_driver_pollbuttons((struct espi_driver *)p);
		espi_driver_adc_poll((struct espi_driver *)p);
		espi_driver_encoder_poll((struct espi_driver *)p);
		break;
	case 1:
	case 5:
		espi_driver_leds_poll((struct espi_driver *)p);
		espi_driver_rb_leds_poll((struct espi_driver *)p);
		espi_driver_attenuator_poll((struct espi_driver *)p);
		break;
	case 3:
		espi_driver_ssd1305_poll((struct espi_driver *)p);
		break;
	case 7:
		espi_driver_ssd1322_poll((struct espi_driver *)p);
		break;
	}

	((struct espi_driver *)p)->poll_stage = (((struct espi_driver *)p)->poll_stage + 1)%8;
}
#endif



/*******************************************************************************
    eSPI driver functions
*******************************************************************************/
static s32 espi_driver_probe(struct spi_device *dev)
{
	s32 nscs, i, ret = 0;
	struct espi_driver *sb;
	struct device_node *dn = dev->dev.of_node; //nni

	printk("espi_driver_probe\n");

	sb = devm_kzalloc(&dev->dev,sizeof(struct espi_driver), GFP_KERNEL);
	if (!sb) {
		dev_err(&dev->dev, "%s: unable to kzalloc for espi_driver\n", __func__);
		return -ENOMEM;
	}

	/****************************************** added by nni  { */

	/** check DT entries */
	nscs = of_gpio_named_count(dn, "scs-gpios");
	if( nscs != 8 || nscs == -ENOENT) {
		dev_err(&dev->dev, "%s: number of scs-gpios in the device tree incorrect\n", __func__);
		return -ENOENT;
	}
	/** fetch gpio numbers */
	for(i=0; i<8; i++) {
		sb->scs_gpios[i] = of_get_named_gpio(dn, "scs-gpios", i);
		if (!gpio_is_valid(sb->scs_gpios[i])) {
			dev_err(&dev->dev, "%s: scs-gpios[%d] in the device tree incorrect\n", __func__, i);
			return -EINVAL;
		}
	}
	sb->sap_gpio = of_get_named_gpio(dn, "sap-gpio", 0);
	if (!gpio_is_valid(sb->sap_gpio)) {
		dev_err(&dev->dev, "%s: sap-gpio in the device tree incorrect\n", __func__);
		return -EINVAL;
	}
	sb->dmxs_gpio = of_get_named_gpio(dn, "dmxs-gpio", 0);
	if (!gpio_is_valid(sb->dmxs_gpio)) {
		dev_err(&dev->dev, "%s: dmxs-gpio in the device tree incorrect\n", __func__);
		return -EINVAL;
	}
	/** request gpios */
	for(i=0; i<8; i++) {
		ret = devm_gpio_request_one(&dev->dev, sb->scs_gpios[i], GPIOF_OUT_INIT_LOW, "scs_gpios");
		if(ret) {
			dev_err(&dev->dev, "%s: failed to request gpio: %d\n", __func__, sb->scs_gpios[i]);
			return ret;
		}
	}
	ret = devm_gpio_request_one(&dev->dev, sb->sap_gpio, GPIOF_OUT_INIT_HIGH, "sap_gpio");
	ret = devm_gpio_request_one(&dev->dev, sb->dmxs_gpio, GPIOF_OUT_INIT_HIGH, "dmxs_gpio");

	/** added by nni } ******************************************************/

	sb->spidev = dev;
	sb->dev = &dev->dev;
	dev_set_drvdata(&dev->dev, (void*)sb);
	dev_info(&dev->dev, "spi registered, item=0x%p\n", (void *)sb);

	sb->poll_stage = 0;
	espi_driver_buttons_setup(sb);
	espi_driver_leds_setup(sb);
	espi_driver_rb_leds_setup(sb);
	espi_driver_ssd1305_setup(sb);
	espi_driver_ssd1322_setup(sb);
	espi_driver_adc_setup(sb);
	espi_driver_attenuator_setup(sb);
	espi_driver_encoder_setup(sb);

	INIT_DELAYED_WORK(&(sb->work), espi_driver_poll);
	queue_delayed_work(workqueue, &(sb->work), msecs_to_jiffies(8));

	return ret;
}


static s32 espi_driver_remove(struct spi_device *spi)
{
	struct espi_driver *sb = (struct espi_driver*)dev_get_drvdata(&spi->dev);

	printk("espi_driver_remove\n");

	cancel_delayed_work(&(sb->work));

	espi_driver_encoder_cleanup(sb);
	espi_driver_attenuator_cleanup(sb);
	espi_driver_adc_cleanup(sb);
	espi_driver_ssd1322_cleanup(sb);
	espi_driver_ssd1305_cleanup(sb);
	espi_driver_rb_leds_cleanup(sb);
	espi_driver_leds_cleanup(sb);
	espi_driver_buttons_cleanup(sb);
	
	return 0;
}


static struct spi_driver espi_driver_driver = {
		.driver = {
				.name = "espi_driver",
				.bus = &spi_bus_type,
				.owner = THIS_MODULE,
		},
		.probe = espi_driver_probe,
		.remove = espi_driver_remove,
		//.suspend = espi_driver_suspend,
		//.resume = espi_driver_resume,
};


// **************************************************************************
static s32 __init espi_driver_init( void )
{
	s32 ret;

	printk("espi_driver_init --\n");

	workqueue = create_workqueue("espi_driver queue");
	if (workqueue == NULL) {
		pr_err("%s: unable to create workqueue\n", __func__);
		return -1;
	}

	ret = spi_register_driver(&espi_driver_driver);
	if (ret)
		pr_err("%s: problem at spi_register_driver\n", __func__);


	printk("Registration done. 2014-09-05-16-11\n");
    

	return ret;
}
module_init(espi_driver_init);


// **************************************************************************
static void __exit espi_driver_exit( void )
{
	printk("espi_driver_exit --\n");

	spi_unregister_driver(&espi_driver_driver);
}
module_exit(espi_driver_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nemanja Nikodijevic");
MODULE_DESCRIPTION("espi_driver");
MODULE_VERSION("0.1");



