#ifndef _ESPI_FB_H_
#define _ESPI_FB_H_

#include "espi_driver.h"

s32 ssd1322_fb_init(struct oleds_fb_par *par);
s32 ssd1305_fb_init(struct oleds_fb_par *par);
void ssd1322_fb_deinit(void);
void ssd1305_fb_deinit(void);

void ssd1322_update_display(struct oleds_fb_par *par);
void ssd1305_update_display(struct oleds_fb_par *par);

u8 ssd1322_rgb_to_mono(u16 rgb);

void lpc8xx_boled_reset(struct espi_driver *p);

#endif


