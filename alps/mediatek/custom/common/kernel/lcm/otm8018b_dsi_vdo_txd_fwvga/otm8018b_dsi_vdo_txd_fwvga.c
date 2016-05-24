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

1. lidong.zhou@ragentek.com    2013.6.24
   Description: add mipi  txd otm8018b  fwvga

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
#define MIN_VOLTAGE (0)     // zhoulidong  add for lcm detect
#define MAX_VOLTAGE (200)     // zhoulidong  add for lcm detect
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

#define LCM_ID_OTM8018B									0x8009
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
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {


	{0x00,  1 ,{0x00}},
	{0xFF,  3 ,{0x80,0x09,0x01}},
	{0x00,  1 ,{0x80}},
	{0xFF,  2 ,{0x80,0x09}},
	{0x00,  1 ,{0x03}},
	{0xFF,  1 ,{0x01}},





	{0x00,  1 ,{0x00}},
	{0xD8,  2 ,{0x77,0x77}},//6f

	{0x00,  1 ,{0x00}},
	{0xD9,  1 ,{0x1c}},//1e

	{0x00,  1 ,{0xB4}},
	{0xC0,  1 ,{0x50}},

	{0x00,  1 ,{0x82}},
	{0xC5,  1 ,{0xA3}},

	{0x00,  1 ,{0x81}},
	{0xC1,  1 ,{0x77}},

	{0x00,  1 ,{0xA1}},
	{0xC1,  1 ,{0x08}},

	{0x00,  1 ,{0xA3}},
	{0xC0,  1 ,{0x1B}},

	{0x00,  1 ,{0x81}},
	{0xC4,  1 ,{0x83}},

	{0x00,  1 ,{0x90}},
	{0xC5,  3 ,{0x96,0x94,0x01}},

	{0x00,  1 ,{0xB1}},
	{0xC5,  1 ,{0xA9}},

	{0x00,  1 ,{0x90}},
	{0xB3,  1 ,{0x02}},

	{0x00,  1 ,{0x92}},
	{0xB3,  1 ,{0x45}},

	{0x00,  1 ,{0xA1}},
	{0xB3,  1 ,{0x10}},

	{0x00,  1 ,{0xA6}},
	{0xB3,  2 ,{0x20,11}},

	{0x00,  1 ,{0xA0}},
	{0xC1,  1 ,{0xEA}},



	{0x00,  1 ,{0x80}},
	{0xC0,  5 ,{0x00,0x58,0x00,0x14,0x16}},

	{0x00,  1 ,{0x90}},
	{0xC0,  6 ,{0x00,0x44,0x00,0x00,0x00,0x03}},

	{0x00,  1 ,{0x80}},
	{0xF5,  12 ,{0x01,0x14,0x02,0x14,0x10,0x14,0x02,0x14,0x0e,0x14,0x0f,0x20}},
	{0x00,  1 ,{0x90}},
	{0xF5,  10 ,{0x02,0x14,0x08,0x14,0x06,0x14,0x0d,0x14,0x0b,0x14}},

	{0x00,  1 ,{0xA0}},
	{0xF5,  8 ,{0x10,0x14,0x01,0x14,0x14,0x14,0x14,0x14}},

	{0x00,  1 ,{0xB0}},
	{0xF5,  12 ,{0x14,0x14,0x12,0x14,0x13,0x14,0x11,0x14,0x13,0x14,0x00,0x00}},

	{0x00,  1 ,{0x80}},
	{0xC4,  1 ,{0x30}},

	{0x00,  1 ,{0x8A}},
	{0xC4,  1 ,{0x40}},

	{0x00,  1 ,{0xA0}},
	{0xC1,  1 ,{0xEA}},

	{0x00,  1 ,{0xA6}},
	{0xC1,  3 ,{0x01,0x00,0x00}},

	{0x00,  1 ,{0x90}},
	{0xC0,  6 ,{0x00,0x44,0x00,0x00,0x00,0x03}},

	{0x00,  1 ,{0xc0}},
	{0xc5,  1 ,{0x00}},

	{0x00,  1 ,{0x8B}},
	{0xB0,  1 ,{0x40}},


	{0x00,  1 ,{0xb2}},
	{0xf5,  4 ,{0x15,0x00,0x15,0x00}},

	{0x00,  1 ,{0x93}},
	{0xc5,  1 ,{0x03}},

	{0x00,  1 ,{0x80}},
	{0xCE,  6 ,{0x87,0x03,0x00,0x86,0x03,0x00}},
	{0x00,  1 ,{0x90}},
	{0xCE,  6 ,{0x33,0x54,0x00,0x33,0x55,0x00}},
	{0x00,  1 ,{0xA0}},
	{0xCE, 14 ,{0x38,0x03,0x03,0x58,0x00,0x00,0x00,0x38,0x02,0x03,0x59,0x00,0x00,0x00}},
	{0x00,  1 ,{0xB0}},
	{0xCE, 14 ,{0x38,0x01,0x03,0x5A,0x00,0x00,0x00,0x38,0x00,0x03,0x5B,0x00,0x00,0x00}},
	{0x00,  1 ,{0xC0}},
	{0xCE, 14 ,{0x30,0x00,0x03,0x5C,0x00,0x00,0x00,0x30,0x01,0x03,0x5D,0x00,0x00,0x00}},
	{0x00,  1 ,{0xD0}},
	{0xCE, 14 ,{0x38,0x05,0x03,0x5E,0x00,0x00,0x00,0x38,0x04,0x03,0x5F,0x00,0x00,0x00}},
	{0x00,  1 ,{0xC0}},
	{0xCF, 10 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x80,0x00,0x09}},
	{0x00,  1 ,{0xC0}},
	{0xCB, 15 ,{0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00}},
	{0x00,  1 ,{0xD0}},
	{0xCB, 15 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x04,0x04,0x04}},
	{0x00,  1 ,{0xE0}},
	{0xCB, 10 ,{0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,  1 ,{0x80}},
	{0xCC, 10 ,{0x00,0x26,0x25,0x02,0x06,0x00,0x00,0x0A,0x0E,0x0C}},
	{0x00,  1 ,{0x90}},
	{0xCC, 15 ,{0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x25,0x01,0x05}},
	{0x00,  1 ,{0xA0}},
	{0xCC, 15 ,{0x00,0x00,0x09,0x0D,0x0B,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00,  1 ,{0xB0}},
	{0xCC, 10 ,{0x00,0x25,0x26,0x05,0x01,0x00,0x00,0x0D,0x09,0x0B}},
	{0x00,  1 ,{0xC0}},
	{0xCC, 15 ,{0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x26,0x06,0x02}},
	{0x00,  1 ,{0xD0}},
	{0xCC, 15 ,{0x00,0x00,0x0E,0x0A,0x0C,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	//{0x00,  1 ,{0x00}},
	//{0xE1, 16 ,{0x05,0x10,0x17,0x2F,0x29,0x1A,0x0C,0x0C,0x01,0x04,0x02,0x27,0x2D,0x25,0x23,0xE0}},
	//{0x00,  1 ,{0x00}},
	//{0xE2, 16 ,{0x05,0x10,0x17,0x2F,0x29,0x1A,0x0C,0x0C,0x01,0x04,0x02,0x27,0x2D,0x25,0x23,0xE0}},
//Gamma {2.2
{0x00,1,{0x00}},
{0xE1,16,{0x02,0x0B,0x14,0x11,0x0B,0x1D,0x0E,0x0D,0x00,0x05,0x03,0x07,0x0E,0x22,0x1E,0x08}},
{0x00,1,{0x00}},
{0xE2,16,{0x02,0x0B,0x14,0x11,0x0B,0x1D,0x0E,0x0D,0x00,0x05,0x03,0x07,0x0E,0x22,0x1E,0x08}},


	{0x11, 1 ,{0x00}}, 
	{REGFLAG_DELAY, 150, {}},

	{0x29, 1 ,{0x00}}, 
	{REGFLAG_DELAY, 200, {}},

};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
    params->active_width  = 63;
    params->active_height = 111;
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
    params->dsi.fbk_div =0x1F;  		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	/*0x1E-->0x1F for rf desense*/	



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

    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);	

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

    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE)
    {
	return 1;
    }

    return 0;
	
}

// zhoulidong  add for lcm detect (end)

LCM_DRIVER otm8018b_dsi_vdo_txd_fwvga_lcm_drv = 
{
    .name			= "otm8018b_dsi_vdo_txd_fwvga",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = rgk_lcm_compare_id,	
};

