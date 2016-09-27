#include <linux/string.h>
#include <linux/of_gpio.h>
#include "espi_driver.h"
#include "espi_fb.h"

/* SSD1322 stuff **************************************************************/
#define ESPI_SSD1322_SPEED		5000000//10000000

#define SSD1322_SET_CMD_LOCK 		0xFD
#define SSD1322_SET_DISP_OFF 		0xAE
#define SSD1322_SET_DISP_ON 		0xAF
#define SSD1322_SET_COL_ADDR 		0x15
#define SSD1322_SET_ROW_ADDR 		0x75
#define SSD1322_SET_DISP_CLK 		0xB3
#define SSD1322_SET_MUX_RATIO 		0xCA
#define SSD1322_SET_DISP_OFFSET 	0xA2
#define SSD1322_SET_START_LINE 		0xA1
#define SSD1322_SET_REMAP 		0xA0
#define SSD1322_SET_GPIO 		0xB5
#define SSD1322_SET_FUNC_SEL 		0xAB
#define SSD1322_SET_DISP_ENH_A 		0xB4
#define SSD1322_SET_DISP_ENH_B 		0xD1
#define SSD1322_SET_CONTRAST_CUR 	0xC1
#define SSD1322_SET_MASTER_CUR 		0xC7
#define SSD1322_SET_LINEAR_GST 		0xB9
#define SSD1322_SET_PHASE_LEN 		0xB1
#define SSD1322_SET_PRECH_VOL 		0xBB
#define SSD1322_SET_PRECH_PER 		0xB6
#define SSD1322_SET_VCOMH 		0xBE
#define SSD1322_SET_DISP_MODE 		0xA4
#define SSD1322_SET_PARTIAL_DISP 	0xA8
#define SSD1322_WRITE_RAM 		0x5C

#define SSD1322_BUFF_SIZE 		(128*64)
#define SSD1322_BUFF_WIDTH 		128
#define SSD1322_BUFF_HEIGHT 		64

static u8 *ssd1322_buff;
static u8 *ssd1322_tmp_buff;
static DEFINE_MUTEX(ssd1322_tmp_buff_lock);

/*******************************************************************************
    ssd1322 functions (boled)
*******************************************************************************/
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

	gpio_set_value(sb->gpio_sap, 0);
	espi_driver_transfer(sb->spidev, &xfer);
	gpio_set_value(sb->gpio_sap, 1);

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

s32 ssd1322_fb_init(struct oleds_fb_par *par)
{
	u8 data[2];

	struct espi_driver *sb = par->espi;

	ssd1322_buff = kcalloc(SSD1322_BUFF_SIZE,sizeof(u8), GFP_KERNEL);
	if (!ssd1322_buff)
		return -ENOMEM;

	ssd1322_tmp_buff = kcalloc(SSD1322_BUFF_SIZE,sizeof(u8), GFP_KERNEL);
	if (!ssd1322_tmp_buff)
		return -ENOMEM;

	memset(ssd1322_buff, 0, SSD1322_BUFF_SIZE);
	memset(ssd1322_tmp_buff, 0, SSD1322_BUFF_SIZE);

	/** DISPLAY INITIALIZATION *************/
	espi_driver_scs_select(sb, ESPI_EDIT_PANEL_PORT, ESPI_EDIT_BOLED_DEVICE);

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
	data[0] = 0x06; // -> horizontal
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

	espi_driver_scs_select(sb, ESPI_EDIT_PANEL_PORT, 0);

	return 0;
}

void ssd1322_fb_deinit(struct espi_driver *p)
{

	memset(ssd1322_buff, 0, SSD1322_BUFF_SIZE);
	memset(ssd1322_tmp_buff, 0, SSD1322_BUFF_SIZE);
	espi_driver_ssd1322_poll(p);

	kfree(ssd1322_buff);
	kfree(ssd1322_tmp_buff);
}

u8 ssd1322_rgb_to_mono(u16 rgb)
{
	u16 tmp;
	tmp = 613 * (rgb >> 11) + 601 * (rgb >> 5 & 0x3F) + 233 * (rgb & 0x1F);
	return tmp >> 12;
}

void ssd1322_update_display(struct oleds_fb_par *par)
{
	u32 i;
	u16* buf =  (u16*) (par->info->screen_base);
	u32 offset = 0;

	mutex_lock(&ssd1322_tmp_buff_lock);
	for(i = 0; i < SSD1322_BUFF_SIZE; i++) {
		ssd1322_tmp_buff[i] = ssd1322_rgb_to_mono(buf[offset++]) << 4;
		ssd1322_tmp_buff[i] |= ssd1322_rgb_to_mono(buf[offset++]);
	}
	mutex_unlock(&ssd1322_tmp_buff_lock);

}

void espi_driver_ssd1322_poll(struct espi_driver *p)
{
	u32 i, update = 0;

	ssd1322_update_display(p->oleds);

	mutex_lock(&ssd1322_tmp_buff_lock);
	for(i=0; i<SSD1322_BUFF_SIZE; i++)
		if(ssd1322_buff[i] != ssd1322_tmp_buff[i]) {
			ssd1322_buff[i] = ssd1322_tmp_buff[i];
			update = 1;
		}
	mutex_unlock(&ssd1322_tmp_buff_lock);

	if(update == 0)
		return;

	espi_driver_scs_select(p, ESPI_EDIT_PANEL_PORT, ESPI_EDIT_BOLED_DEVICE);
	ssd1322_data(p, ssd1322_buff, SSD1322_BUFF_SIZE);
	espi_driver_scs_select(p, ESPI_EDIT_PANEL_PORT, 0);
}


