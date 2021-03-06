/*
 * arch/arm/mach-meson3/board-m6g24-panel.c
 *
 * from board-m6g33-fuge512-848-panel.c
 * Copyright (C) 2011-2012 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/lm.h>
#include <mach/clock.h>
#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/gpio_data.h>
#include <linux/delay.h>
#include <plat/regops.h>
#include <mach/reg_addr.h>

#include <linux/vout/lcdoutc.h>
#include <linux/aml_bl.h>
#include <mach/lcd_aml.h>

#include "board-softwin.h"

#ifdef CONFIG_AW_AXP
extern int axp_gpio_set_io(int gpio, int io_state);
extern int axp_gpio_get_io(int gpio, int *io_state);
extern int axp_gpio_set_value(int gpio, int value);
extern int axp_gpio_get_value(int gpio, int *value);
#endif

#ifdef CONFIG_AML1212
#include <amlogic/aml1212.h>
#endif

extern Lcd_Config_t m6g24_lcd_config;

//*****************************************
// Define backlight control method
//*****************************************
#define BL_CTL_GPIO	0
#define BL_CTL_PWM	1
#define BL_CTL		BL_CTL_GPIO

//backlight controlled parameters in driver, define the real backlight level
#if (BL_CTL==BL_CTL_GPIO)
#define	DIM_MAX			0x0
#define	DIM_MIN			0xd	
#elif (BL_CTL==BL_CTL_PWM)
#define	PWM_CNT			600			//PWM_CNT <= 65535
#define	PWM_PRE_DIV		0				//pwm_freq = 24M / (pre_div + 1) / PWM_CNT
#define PWM_MAX         (PWM_CNT * 100 / 100)		
#define PWM_MIN         (PWM_CNT * 10 / 100)	
#endif

//brightness level in Android UI menu
#define BL_MAX_LEVEL    	255
#define BL_MIN_LEVEL    	20		
#define DEFAULT_BL_LEVEL	200	//please keep this value the same as uboot

static unsigned bl_level = 0;
static Bool_t data_status = ON;
static Bool_t bl_status = ON;
//*****************************************

#define LCD_BITS		6	//8	//6
static void ttl_ports_ctrl(Bool_t status)
{
    printk(KERN_INFO "%s: %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    if (status) {
        aml_write_reg32(P_PERIPHS_PIN_MUX_1, aml_read_reg32(P_PERIPHS_PIN_MUX_1) | ((1<<14)|(1<<17)|(1<<18)|(1<<19))); //set tcon pinmux
#if (LCD_BITS == 6)		
        aml_write_reg32(P_PERIPHS_PIN_MUX_0, aml_read_reg32(P_PERIPHS_PIN_MUX_0) | ((1<<0)|(1<<2)|(1<<4)));  //enable RGB 18bit
#else
		aml_write_reg32(P_PERIPHS_PIN_MUX_0, aml_read_reg32(P_PERIPHS_PIN_MUX_0) | ((3<<0)|(3<<2)|(3<<4)));  //enable RGB 24bit
#endif		
    }else {
#if (LCD_BITS == 6)
        aml_write_reg32(P_PERIPHS_PIN_MUX_0, aml_read_reg32(P_PERIPHS_PIN_MUX_0) & ~((1<<0)|(1<<2)|(1<<4))); //disable RGB 18bit
		aml_write_reg32(P_PREG_PAD_GPIO1_EN_N, aml_read_reg32(P_PREG_PAD_GPIO1_EN_N) | (0x3f << 2) | (0x3f << 10) | (0x3f << 18));       //GPIOB_0--GPIOB_23  set input
#else
		aml_write_reg32(P_PERIPHS_PIN_MUX_0, aml_read_reg32(P_PERIPHS_PIN_MUX_0) & ~((3<<0)|(3<<2)|(3<<4))); //disable RGB 24bit
		aml_write_reg32(P_PREG_PAD_GPIO1_EN_N, aml_read_reg32(P_PREG_PAD_GPIO1_EN_N) | (0xff << 0) | (0xff << 8) | (0xff << 16));       //GPIOB_0--GPIOB_23  set input
#endif
		aml_write_reg32(P_PERIPHS_PIN_MUX_1, aml_read_reg32(P_PERIPHS_PIN_MUX_1) & ~((1<<14)|(1<<17)|(1<<18)|(1<<19)));  //clear tcon pinmux        		
		aml_write_reg32(P_PREG_PAD_GPIO2_EN_N, aml_read_reg32(P_PREG_PAD_GPIO2_EN_N) | ((1<<18)|(1<<19)|(1<<20)|(1<<23)));  //GPIOD_2 D_3 D_4 D_7 		
    }
}

static void backlight_power_ctrl(Bool_t status)
{ 
	//printk("%s(): bl_status=%s, data_status=%s, bl_level=%u\n", __FUNCTION__, (bl_status ? "ON" : "OFF"), (data_status ? "ON" : "OFF"), bl_level);
    if( status == ON ){
		if ((bl_status == ON) || (data_status == OFF) || (bl_level == 0))
			return;
        aml_set_reg32_bits(P_LED_PWM_REG0, 1, 12, 2);
        msleep(10); 
		//BL_EN -> GPIOD_1: 1
#if (BL_CTL==BL_CTL_GPIO)
		//BL_EN -> GPIOD_1: 1
		gpio_out(PAD_GPIOD_1, 1);
#elif (BL_CTL==BL_CTL_PWM)
		aml_write_reg32(P_PWM_MISC_REG_CD, (aml_read_reg32(P_PWM_MISC_REG_CD) & ~(0x7f<<16)) | ((1 << 23) | (PWM_PRE_DIV<<16) | (1<<1)));  //enable pwm clk & pwm output
		aml_write_reg32(P_PERIPHS_PIN_MUX_2, aml_read_reg32(P_PERIPHS_PIN_MUX_2) | (1<<3));  //enable pwm pinmux
#endif
    }
    else{
		if (bl_status == OFF)
			return;
		aml_write_reg32(P_PREG_PAD_GPIO2_O, (aml_read_reg32(P_PREG_PAD_GPIO2_O) & ~(1<<17)));
		aml_write_reg32(P_PREG_PAD_GPIO2_EN_N, (aml_read_reg32(P_PREG_PAD_GPIO2_EN_N) & ~(1<<17)));		//GPIOD_1 output low
		aml_write_reg32(P_PWM_MISC_REG_CD, aml_read_reg32(P_PWM_MISC_REG_CD) & ~((1 << 23) | (1<<1)));  //disable pwm clk & pwm output
    }
	bl_status = status;
	printk(KERN_INFO "%s() Power %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
}

#define BL_MID_LEVEL    		128
#define BL_MAPPED_MID_LEVEL		102
static void set_backlight_level(unsigned level)
{
    //printk("set_backlight_level: %u, last level: %u\n", level, bl_level);
	level = (level > BL_MAX_LEVEL ? BL_MAX_LEVEL : (level < BL_MIN_LEVEL ? 0 : level));	
    bl_level = level;

	if (level == 0) {
		backlight_power_ctrl(OFF);		
	}
	else {	
		if (level > BL_MID_LEVEL) {
			level = ((level - BL_MID_LEVEL)*(BL_MAX_LEVEL-BL_MAPPED_MID_LEVEL))/(BL_MAX_LEVEL - BL_MID_LEVEL) + BL_MAPPED_MID_LEVEL; 
		} else {
			//level = (level*BL_MAPPED_MID_LEVEL)/BL_MID_LEVEL;
			level = ((level - BL_MIN_LEVEL)*(BL_MAPPED_MID_LEVEL - BL_MIN_LEVEL))/(BL_MID_LEVEL - BL_MIN_LEVEL) + BL_MIN_LEVEL; 
		}		
#if (BL_CTL==BL_CTL_GPIO)
		level = DIM_MIN - ((level - BL_MIN_LEVEL) * (DIM_MIN - DIM_MAX)) / (BL_MAX_LEVEL - BL_MIN_LEVEL);	
		aml_set_reg32_bits(P_LED_PWM_REG0, level, 0, 4);	
#elif (BL_CTL==BL_CTL_PWM)
		level = (PWM_MAX - PWM_MIN) * (level - BL_MIN_LEVEL) / (BL_MAX_LEVEL - BL_MIN_LEVEL) + PWM_MIN;	
		aml_write_reg32(P_PWM_PWM_D, (level << 16) | (PWM_CNT - level));  //pwm	duty	
#endif
		if (bl_status == OFF) 
			backlight_power_ctrl(ON);		
	} 
}

static unsigned get_backlight_level(void)
{
    printk(KERN_DEBUG "%s: %d\n", __FUNCTION__, bl_level);
    return bl_level;
}

static void lcd_power_ctrl(Bool_t status)
{
    printk(KERN_INFO "%s() Power %s\n", __FUNCTION__, (status ? "ON" : "OFF"));
    if (status) {
        //GPIOA27 -> LCD_PWR_EN#: 0  lcd 3.3v
		gpio_out(PAD_GPIOD_8, 0);
        gpio_set_status(PAD_GPIOD_8,gpio_status_out);
        gpio_out(PAD_GPIOA_27, 0);              // open LCD power 
		gpio_set_status(PAD_GPIOA_27,gpio_status_out);
        msleep(10);
        gpio_out(PAD_GPIOD_8, 1);
        msleep(20);
	
        //GPIOC2 -> VCCx3_EN: 0
        //gpio_out(PAD_GPIOC_2, 1);
#ifdef CONFIG_AW_AXP
		axp_gpio_set_io(3,1);     
		axp_gpio_set_value(3, 0); 
#endif			
#ifdef CONFIG_AML1212
        aml_pmu_set_gpio(1, 0);
#endif
		msleep(10);
		
        ttl_ports_ctrl(ON);
		msleep(200);
		data_status = status;
    }
    else {
		data_status = status;
		msleep(200);
        ttl_ports_ctrl(OFF);
		msleep(10);
		
        //GPIOC2 -> VCCx3_EN: 1        
        //gpio_out(PAD_GPIOC_2, 0);
#ifdef CONFIG_AW_AXP
		axp_gpio_set_io(3,0);		
#endif		
#ifdef CONFIG_AML1212
        aml_pmu_set_gpio(1, 1);
#endif
		msleep(10);
		gpio_out(PAD_GPIOD_8, 0);
        //GPIOA27 -> LCD_PWR_EN#: 1  lcd 3.3v
        //gpio_out(PAD_GPIOA_27, 1);
		gpio_set_status(PAD_GPIOA_27,gpio_status_in);
		msleep(100);        //power down sequence, needed
    }
}

static int lcd_suspend(void *args)
{
    args = args;
    
    printk(KERN_INFO "LCD suspending...\n");
	backlight_power_ctrl(OFF);
	lcd_power_ctrl(OFF);
    return 0;
}

static int lcd_resume(void *args)
{
    args = args;
    printk(KERN_INFO "LCD resuming...\n");
	lcd_power_ctrl(ON);
	backlight_power_ctrl(ON);    
    return 0;
}


#define H_ACTIVE		800
#define V_ACTIVE      	480
#define H_PERIOD		1048
#define V_PERIOD		525
#define VIDEO_ON_PIXEL  48
#define VIDEO_ON_LINE   22

Lcd_Config_t m6g24_lcd_config = {

    // Refer to LCD Spec
    .lcd_basic = {
        .h_active = H_ACTIVE,
        .v_active = V_ACTIVE,
        .h_period = H_PERIOD,
        .v_period = V_PERIOD,
    	.screen_ratio_width = 5,
     	.screen_ratio_height = 3,
        .lcd_type = LCD_DIGITAL_TTL,   //LCD_DIGITAL_TTL  //LCD_DIGITAL_LVDS  //LCD_DIGITAL_MINILVDS
        .lcd_bits = LCD_BITS,
    },

    .lcd_timing = {
        .pll_ctrl = 0x1022c, //clk=33MHz, 60Hz
        .div_ctrl = 0x18813, 
        .clk_ctrl = 0x1118,  //[19:16]ss_ctrl, [12]pll_sel, [8]div_sel, [4]vclk_sel, [3:0]xd
        //.sync_duration_num = 501,
        //.sync_duration_den = 10,
    
		.video_on_pixel = VIDEO_ON_PIXEL,
        .video_on_line = VIDEO_ON_LINE,
		 
		.sth1_hs_addr = 1271,
		.sth1_he_addr = 1251,
        .sth1_vs_addr = 0,
        .sth1_ve_addr = V_PERIOD - 1,
        .oeh_hs_addr = 67,
        .oeh_he_addr = 67+H_ACTIVE,
        .oeh_vs_addr = VIDEO_ON_LINE,
        .oeh_ve_addr = VIDEO_ON_LINE+V_ACTIVE-1,
        .vcom_hswitch_addr = 0,
        .vcom_vs_addr = 0,
        .vcom_ve_addr = 0,
        .cpv1_hs_addr = 0,
        .cpv1_he_addr = 0,
        .cpv1_vs_addr = 0,
        .cpv1_ve_addr = 0,
        .stv1_hs_addr = 0,
        .stv1_he_addr = H_PERIOD-1,
		.stv1_vs_addr = 2,
		.stv1_ve_addr = 634,
        .oev1_hs_addr = 0,
        .oev1_he_addr = 0,
        .oev1_vs_addr = 0,
        .oev1_ve_addr = 0,

        .pol_cntl_addr = (0x0 << LCD_CPH1_POL) |(0x1 << LCD_HS_POL) | (0x1 << LCD_VS_POL),
        .inv_cnt_addr = (0<<LCD_INV_EN) | (0<<LCD_INV_CNT),
        .tcon_misc_sel_addr = (1<<LCD_STV1_SEL) | (1<<LCD_STV2_SEL),
        .dual_port_cntl_addr = (1<<LCD_TTL_SEL) | (1<<LCD_ANALOG_SEL_CPH3) | (1<<LCD_ANALOG_3PHI_CLK_SEL) | (0<<LCD_RGB_SWP) | (0<<LCD_BIT_SWP),
    },

    .lcd_effect = {
        .gamma_cntl_port = (1 << LCD_GAMMA_EN) | (0 << LCD_GAMMA_RVS_OUT) | (1 << LCD_GAMMA_VCOM_POL),
        .gamma_vcom_hswitch_addr = 0,
        .rgb_base_addr = 0xf0,
        .rgb_coeff_addr = 0x74a, 
    },

    .lcd_power_ctrl = {
        .cur_bl_level = 0,
        .power_ctrl = lcd_power_ctrl,
        .backlight_ctrl = backlight_power_ctrl,
        .get_bl_level = get_backlight_level,
        .set_bl_level = set_backlight_level,
        .lcd_suspend = lcd_suspend,
        .lcd_resume = lcd_resume,
    },
};

static void lcd_setup_gamma_table(Lcd_Config_t *pConf)
{
    int i;
	
	const unsigned short gamma_adjust[256] = {		
		0,1,2,3,4,5,6,7,9,10,11,12,13,14,15,17,18,19,20,21,22,22,23,24,25,26,27,28,29,29,30,31,
		32,33,34,34,35,36,37,38,39,40,40,41,42,43,44,45,46,47,48,49,50,51,53,54,55,56,57,58,59,60,61,62,
		63,64,65,66,67,68,69,70,71,72,73,75,76,78,79,81,83,84,85,87,88,89,90,92,93,93,94,95,96,97,98,99,
		100,101,102,103,104,105,106,108,109,110,111,113,114,116,117,118,119,120,122,122,123,124,125,126,127,128,129,130,131,131,132,133,
		134,135,136,137,138,139,140,141,142,143,144,145,147,148,149,150,152,153,154,155,156,157,159,160,161,162,163,164,165,166,167,168,
		169,170,171,172,172,173,174,175,176,177,178,179,179,181,182,183,184,185,187,188,189,190,191,192,193,194,196,197,198,199,200,201,
		202,202,203,204,205,206,207,208,209,209,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,
		233,234,234,235,236,237,238,239,239,240,241,242,243,243,244,245,246,246,247,247,248,249,250,250,251,251,252,252,253,254,254,255,
    };

    for (i=0; i<256; i++) {
        pConf->lcd_effect.GammaTableR[i] = gamma_adjust[i] << 2;
        pConf->lcd_effect.GammaTableG[i] = gamma_adjust[i] << 2;
        pConf->lcd_effect.GammaTableB[i] = gamma_adjust[i] << 2;
    }
}

static void lcd_video_adjust(Lcd_Config_t *pConf)
{
	int i;
	
	const signed short video_adjust[33] = { -999, -937, -875, -812, -750, -687, -625, -562, -500, -437, -375, -312, -250, -187, -125, -62, 0, 62, 125, 187, 250, 312, 375, 437, 500, 562, 625, 687, 750, 812, 875, 937, 1000};
	
	for (i=0; i<33; i++)
	{
		pConf->lcd_effect.brightness[i] = video_adjust[i];
		pConf->lcd_effect.contrast[i]   = video_adjust[i];
		pConf->lcd_effect.saturation[i] = video_adjust[i];
		pConf->lcd_effect.hue[i]        = video_adjust[i];
	}
}

static void lcd_sync_duration(Lcd_Config_t *pConf)
{
	unsigned m, n, od, div, xd, pre_div;
	unsigned sync_duration;	

	m = ((pConf->lcd_timing.pll_ctrl) >> 0) & 0x1ff;
	n = ((pConf->lcd_timing.pll_ctrl) >> 9) & 0x1f;
	od = ((pConf->lcd_timing.pll_ctrl) >> 16) & 0x3;
	div = ((pConf->lcd_timing.div_ctrl) >> 4) & 0x7;	
	
	od = (od == 0) ? 1:((od == 1) ? 2:4);
	switch(pConf->lcd_basic.lcd_type)
	{
		case LCD_DIGITAL_TTL:
			xd = ((pConf->lcd_timing.clk_ctrl) >> 0) & 0xf;
			pre_div = 1;
			break;
		case LCD_DIGITAL_LVDS:
			xd = 1;
			pre_div = 7;
			break;
		case LCD_DIGITAL_MINILVDS:
			xd = 1;
			pre_div = 6;
			break;	
		default:
			pre_div = 1;
			break;
	}
	
	sync_duration = m*24*1000/(n*od*(div+1)*xd*pre_div);		
	sync_duration = ((sync_duration * 10000 / H_PERIOD) * 10) / V_PERIOD;
	sync_duration = (sync_duration + 5) / 10;	
	
	pConf->lcd_timing.sync_duration_num = sync_duration;
	pConf->lcd_timing.sync_duration_den = 10;
}

static struct aml_bl_platform_data m6g24_backlight_data =
{
    //.power_on_bl = power_on_backlight,
    //.power_off_bl = power_off_backlight,
    .get_bl_level = get_backlight_level,
    .set_bl_level = set_backlight_level,
    .max_brightness = BL_MAX_LEVEL,
    .dft_brightness = DEFAULT_BL_LEVEL,
};

static struct platform_device m6g24_backlight_device = {
    .name = "aml-bl",
    .id = -1,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &m6g24_backlight_data,
    },
};

static struct aml_lcd_platform __initdata m6g24_lcd_data = {
    .lcd_conf = &m6g24_lcd_config,
};

static struct platform_device __initdata * m6g24_lcd_devices[] = {
    &meson_device_lcd,
//    &meson_device_vout,
    &m6g24_backlight_device,
};


int m6g24_lcd_init(void)
{
    int err;
	lcd_sync_duration(&m6g24_lcd_config);
	lcd_setup_gamma_table(&m6g24_lcd_config);
	lcd_video_adjust(&m6g24_lcd_config);	
    meson_lcd_set_platdata(&m6g24_lcd_data, sizeof(struct aml_lcd_platform));
    err = platform_add_devices(m6g24_lcd_devices, ARRAY_SIZE(m6g24_lcd_devices));
    return err;
}

