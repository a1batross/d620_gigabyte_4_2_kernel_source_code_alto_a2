/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/****************************************
History:

2. lidong.zhou@ragentek.com    2013.4.2
   add lcm compare by adc	

1. lidong.zhou@ragentek.com    2013.3.22
   Description: add mipi  djn otm8018b

zhoulidong  add
---------------------------------------------------------------------------------
VENDOR	|   Driver IC   |    LCD_ID	|  Module N/O	        |   Range	|
---------------------------------------------------------------------------------
DJN	|   ILI9806C	|	0.9V	|90-24025-4177A 	|   0.8-1.0	|	
---------------------------------------------------------------------------------
TRULY   |   OTM8018	|	0.6V	|BSHMDS9401B	        |   0.5-0.7	|
---------------------------------------------------------------------------------
TRUST   |   OTM8018	|	0V	|LCM-T040SWV636MH       |		|
---------------------------------------------------------------------------------
AZET    |   OTM8018	|	1.2V	|MS-A40213N00QO-A       |		|
---------------------------------------------------------------------------------
QJ      |   ILI9806C	|	1.5V	|     			|  1.4-1.6	|
---------------------------------------------------------------------------------
****************************************/

#ifdef BUILD_LK
#include "platform/mt_gpio.h"
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"
// ---------------------------------------------------------------------------
//RGK add
// ---------------------------------------------------------------------------
#include <cust_adc.h>    	// zhoulidong  add for lcm detect
#define MIN_VOLTAGE (800)     // zhoulidong  add for lcm detect
#define MAX_VOLTAGE (1000)     // zhoulidong  add for lcm detect
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0XAB
#define REGFLAG_END_OF_TABLE      							0xAA  // END OF REGISTERS MARKER

#define LCM_ID_OTM8018B									0x8009

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {


	{0x00,1,{0x00}},
	{0xFF,3,{0x80,0x09,0x01}},

	{0x00,1,{0x80}},
	{0xFF,2,{0x80,0x09}}, // enable Orise mode

	{0x00,1,{0x03}},
	{0xff,1,{0x01}},// enable SPI+I2C cmd2 read

	{0x00,1,{0xB4}},
	{0xc0,1,{0x10}},//1+2dot inversion 

	{0x00,1,{0xB9}},
	{0xc4,1,{0x08}}, //reg off	
										 
	{0x00,1,{0xA3}},
	{0xc0,1,{0x00}},//pre-charge //V02





	{0x00,1,{0xB2}},
	{0xc5,1,{0xa3}},//REG-pump23 

	{0x00,1,{0x90}},
	{0xc5,2,{0x96,0x87}},//Pump setting (3x=D6)-->(2x=96)//v02 01/11  d5 87

	{0x00,1,{0x00}},
	{0xd8,2,{0x72,0x72}},//GVDD=4.5V 

	{0x00,1,{0x00}},
	{0xd9,1,{0x61}},// VCOMDC=
	//--------------Gamma+ ----------------------------------------------										
	{0x00,1,{0x00}},
	{0xe1,16,{0x09,0x0a,0x0e,0x0d,0x07,0x18,0x0d,0x0d,0x01,0x04,0x05,0x06,0x0e,0x25,0x22,0x05}},														 
	//-----------Gamma- --------------------------------------------------- 		
	{0x00,1,{0x00}},  
	{0xe2,16,{0x09,0x0a,0x0e,0x0d,0x07,0x18,0x0d,0x0d,0x01,0x04,0x05,0x06,0x0e,0x25,0x22,0x05}},					

	{0x00,1,{0x81}}, 
	{0xc1,1,{0x55}}, //Frame rate 65Hz//V02///66
	// RGB I/F setting VSYNC for OTM8018 0x0e

	{0x00,1,{0xa0}}, 
	{0xc1,1,{0xea}},  //external Vsync(08)  /Vsync,Hsync(0c) /Vsync,Hsync,DE(0e) //V02(0e)  / all	included clk(0f)
	{0x00,1,{0xa2}}, 
	{0xc0,3,{0x04,0x00,0x02}},  

	{0x00,1,{0x80}}, 
	{0xc4,1,{0x30}}, //source bias //V02

	{0x00,1,{0xc0}}, 
	{0xc5,1,{0x00}}, //Pump45


	{0x00,1,{0x8b}}, 
	{0xb0,1,{0x40}}, //DC voltage setting ;[0]GVDD output, default: 0xa8
{0x00,1,{0xb2}},
{0xF5,4,{0x15,0x00,0x15,0x00}},

{0x00,1,{0x81}},
{0xC4,1,{0x83}},

{0x00,1,{0x92}},
{0xc5,2,{0x01,0x03}},

	{0x00,1,{0xb1}}, 
	{0xc5,1,{0xa9}}, //no-display Source output = GND

	{0x00,1,{0x90}}, 
	{0xc0,6,{0x00,0x44,0x00,0x00,0x00,0x03}},

	{0x00,1,{0xa6}}, 
	{0xc1,3,{0x01,0x00,0x00}},		  
	//--------------------------------------------------------------------------------
	//		initial setting 2 < tcon_goa_wave >
	//--------------------------------------------------------------------------------
	// CE8x : vst1, vst2, vst3, vst4\
	{0x00,1,{0x80}}, 
{0x00,1,{0x80}},
	{0xce,12,{0x87,0x03,0x00,0x85,0x03,0x00,0x86,0x03,0x00,0x84,0x03,0x00}},		
																																														
	//CEAx : clka1, clka2
	{0x00,1,{0xa0}},
	{0xce,14,{0x38,0x03,0x03,0x20,0x00,0x00,0x00,0x38,0x02,0x03,0x21,0x00,0x00,0x00}},

	//CEBx : clka3, clka4
	{0x00,1,{0xb0}},
	{0xce,14,{0x38,0x01,0x03,0x22,0x00,0x00,0x00,0x38,0x00,0x03,0x23,0x00,0x00,0x00}},

	//CECx : clkb1, clkb2
	{0x00,1,{0xc0}},
	{0xce,14,{0x30,0x00,0x03,0x24,0x00,0x00,0x00,0x30,0x01,0x03,0x25,0x00,0x00,0x00}},

	//CEDx : clkb3, clkb4
	{0x00,1,{0xd0}},
	{0xce,14,{0x30,0x02,0x03,0x26,0x00,0x00,0x00,0x30,0x03,0x03,0x27,0x00,0x00,0x00}},

	//CFCx :


	//CFDx :
	{0x00,1,{0xc7}},
	{0xcf,1,{0x00}},

	{0x00,1,{0xc9}},
	{0xcf,1,{0x00}},

	{0x00,1,{0xd0}},
	{0xcf,1,{0x00}},
	//--------------------------------------------------------------------------------
	//		initial setting 3 < Panel setting >
	//--------------------------------------------------------------------------------
	// cbcx
	{0x00,1,{0xc0}},
	{0xcb,15,{0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00}},

	// cbdx
	{0x00,1,{0xd0}},
	{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x04}},

{0x00,1,{0x80}},
	{0xcc,10,{0x00,0x00,0x00,0x00,0x0C,0x0A,0x10,0x0E,0x03,0x04}},


	// cc9x
	{0x00,1,{0x90}},
	{0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0B}},


	// ccax
	{0x00,1,{0xa0}},
	{0xcc,15,{0x09,0x0F,0x0D,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},


	// ccbx
	{0x00,1,{0xb0}},
	{0xcc,10,{0x00,0x00,0x00,0x00,0x0D,0x0F,0x09,0x0B,0x02,0x01}},

	// cccx
	{0x00,1,{0xc0}},
	{0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E}},

	// ccdx
	{0x00,1,{0xd0}},
	{0xcc,15,{0x10,0x0A,0x0C,0x04,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
/*
	{0x00,1,{0x82}},
	{0xB2,1,{0x00}},
*/
//	{0x00,1,{0x91}},
//	{0xB3,1,{0x20}},//20  //e0


	{0x35,1,{0x00}}, //open the TE zzf add 

	{0x3A,1,{0x77}},

	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 50, {}},

};

/*

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*
static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

    memset(params, 0, sizeof(LCM_PARAMS));
    
    params->type   = LCM_TYPE_DSI;
    
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    
	//add by yangjuwei start
    params->active_width  = 52;
    params->active_height = 86;
	//add by yangjuwei end

    // enable tearing-free
   // params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
   
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting		
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=800;

    params->dsi.vertical_sync_active				= 4;
    params->dsi.vertical_backporch					= 16;
    params->dsi.vertical_frontporch					= 15;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active				= 6;
    params->dsi.horizontal_backporch				= 37;
    params->dsi.horizontal_frontporch				= 37;
   // params->dsi.horizontal_blanking_pixel				= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    
    // Bit rate calculation

    params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
    params->dsi.fbk_div =0x19;  		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		



}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	
	SET_RESET_PIN(1);     
        MDELAY(10);
        SET_RESET_PIN(0);
        MDELAY(10);
        SET_RESET_PIN(1);
        MDELAY(100);
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

/*
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

*/

// zhoulidong  add for lcm detect (start)
static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;
	
	#ifdef BUILD_LK
		printf("OTM8018B uboot %s \n", __func__);
	       printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("OTM8018B kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif
	   
	return (LCM_ID_OTM8018B == id)?1:0;
}


static int rgk_lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    { 
	#ifdef BUILD_LK
	printf("[adc_uboot]: get data error\n");
	#endif
	return 0;
		   
    }
#endif

    lcm_vol = data[0]*1000+data[1]*10;

    #ifdef BUILD_LK
    printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
    #endif

    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE && lcm_compare_id())
    {
	return 1;
    }

    return 0;
	
}

// zhoulidong  add for lcm detect (end)

// zhoulidong add for eds(start)
static unsigned int lcm_esd_check(void)
{
	#ifdef BUILD_LK
		//printf("lcm_esd_check()\n");
	#else
		//printk("lcm_esd_check()\n");
	#endif 
 #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	if(buffer[0]==0x9c)
	{
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif
		return FALSE;
	}
	else
	{	
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif		 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	
	#ifdef BUILD_LK
		printf("lcm_esd_recover()\n");
	#else
		printk("lcm_esd_recover()\n");
	#endif	
	
	lcm_init();	

	return TRUE;
}
// zhoulidong add for eds(end)
LCM_DRIVER otm8018b_dsi_vdo_djn_lcm_drv = 
{
    .name			= "otm8018b_dsi_vdo_djn",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    =  rgk_lcm_compare_id,
    .esd_check     = lcm_esd_check,
 	.esd_recover   = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
    //.update         = lcm_update,
#endif
};

