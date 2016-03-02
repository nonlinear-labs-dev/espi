#include <linux/of_gpio.h>
#include "espi_driver.h"
#include "espi_fb.h"

#define SSD1305_FB_OFFSET		(256*64)

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

/*******************************************************************************
    ssd1305 functions (soled)
*******************************************************************************/

s32 ssd1305_fb_init(struct oleds_fb_par *par)
{
	struct spi_transfer xfer;
	s32 i;
	struct espi_driver *sb = par->espi;
	
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
	ssd1305_buff[i++] = SSD1305_SET_PAGE;
	ssd1305_buff[i++] = SSD1305_SET_PAGE_ADDR;
	ssd1305_buff[i++] = 0x00;
	ssd1305_buff[i++] = 0x03;
	ssd1305_buff[i++] = SSD1305_SET_COL_HI;
	ssd1305_buff[i++] = SSD1305_SET_COL_LO;
	ssd1305_buff[i++] = SSD1305_DISP_ON;
	
	xfer.tx_buf = ssd1305_buff;
	xfer.rx_buf = NULL;
	xfer.len = i;
	xfer.bits_per_word = 8;
	xfer.delay_usecs = 0;
	xfer.speed_hz = ESPI_SPI_SPEED;
	
	gpio_set_value(sb->gpio_sap, 0);
	espi_driver_scs_select(sb, ESPI_PLAY_PANEL_PORT, ESPI_PLAY_SOLED_DEVICE);
	espi_driver_transfer(sb->spidev, &xfer);
	espi_driver_scs_select(sb, ESPI_PLAY_PANEL_PORT, 0);
	gpio_set_value(sb->gpio_sap, 1);

	for(i=0; i<SSD1305_BUFF_SIZE; i++)
		ssd1305_buff[i] = ssd1305_tmp_buff[i] = 0;
		
	xfer.tx_buf = ssd1305_buff;
	xfer.rx_buf = NULL;
	xfer.len = SSD1305_BUFF_SIZE;
	espi_driver_scs_select(sb, ESPI_PLAY_PANEL_PORT, ESPI_PLAY_SOLED_DEVICE);
	espi_driver_transfer(sb->spidev, &xfer);
	espi_driver_scs_select(sb, ESPI_PLAY_PANEL_PORT, 0);
	
	return 0;
}

void ssd1305_fb_deinit(void)
{
	kfree(ssd1305_buff);
	kfree(ssd1305_tmp_buff);
}

static u8 ssd1305_rgb_to_mono(u16 rgb)
{
	if(ssd1322_rgb_to_mono(rgb) > 0)
		return 1;
	else
		return 0;
}

void ssd1305_update_display(struct oleds_fb_par *par)
{
	u32 i, j, k;
	u16* buf =  (u16*) (par->info->screen_base);
	u32 offset = SSD1305_FB_OFFSET;
	u32 tmp;
	
	mutex_lock(&ssd1305_tmp_buff_lock);
	for(j = 0; j < 4; j++) {
		for(i = 0; i < 128; i++) {
			tmp = j*132 + i + 4;
			ssd1305_tmp_buff[tmp] = 0;
			for(k = 0; k < 8; k++)
				ssd1305_tmp_buff[tmp] |= ssd1305_rgb_to_mono(buf[offset + k*256 + i]) << k;
		}
		offset += 8*256;
	}
	mutex_unlock(&ssd1305_tmp_buff_lock);
}

void espi_driver_ssd1305_poll(struct espi_driver *p)
{
	struct spi_transfer xfer;
	u32 i;
	u8 update = 0;

	ssd1305_update_display(p->oleds);
	
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
	
	gpio_set_value(((struct espi_driver *)p)->gpio_sap, 1);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_PLAY_PANEL_PORT, ESPI_PLAY_SOLED_DEVICE);
	espi_driver_transfer(((struct espi_driver*)p)->spidev, &xfer);
	espi_driver_scs_select((struct espi_driver*)p, ESPI_PLAY_PANEL_PORT, 0);
}

