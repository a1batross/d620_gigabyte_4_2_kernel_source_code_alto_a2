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
1. lidong.zhou@ragentek.com    2013.5.30
   fix  lcd  splash

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
#define MIN_VOLTAGE (500)     // zhoulidong  add for lcm detect
#define MAX_VOLTAGE (700)     // zhoulidong  add for lcm detect

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH 			 (320)
#define FRAME_HEIGHT 			 (480)

#define REGFLAG_DELAY             							0XAB
#define REGFLAG_END_OF_TABLE      							0xAA  // END OF REGISTERS MARKER

#define NT33510_LCM_ID									0x55 

#define LCM_DSI_CMD_MODE									1

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

//#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

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

static void nt35510_djn_set_reset_pin(int high){
	mt_set_gpio_mode(GPIO_DISP_LRSTB_PIN, GPIO_MODE_GPIO);
	if(1 == high)
		mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
}
#define SET_RESET_PIN(v)    (nt35510_djn_set_reset_pin(v))

static struct LCM_setting_table lcm_initialization_setting[] = {


	//Enter CMD2
	{0xED, 2,{	0x01, 0xFE}},
        
	//Enter Manufacture Mode 
   
	{0xEE, 2,{	0xDE, 0x21}}, 
        {REGFLAG_DELAY, 120, {}},

	//Sleep Out
	{0x11 ,1,{	0x00}},
        {REGFLAG_DELAY, 200, {}},
  
	{0xC0, 4,{	0x5F,0x5F,0x10,0x10}}, 
	
	//Set VCOM
	{0xC4, 1,{	0x85}},  
        
	//Enter Page1
	{0xBF, 1,{	0xAA}},  
        
	//Smart Color Manufacture Setting
	{0xB0,18,{
			0x0D, 0x00, 0x0D, 0x00, 
			0x11, 0x00, 0x19, 0x00, 
			0x21, 0x00, 0x2D, 0x00, 
			0x3D, 0x00, 0x5D, 0x00, 
			0x5D, 0x00}},
         
	{0xB1, 6,{
			0x80, 0x00, 0x8B, 0x00, 
			0x96, 0x00 }}, 
	
	{0xB2, 6,{
			0x00, 0x00, 0x02, 0x00, 
			0x03, 0x00 }},
	 
	{0xB3,24,{
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00}}, 
	
	{0xB4, 6,{
			0x8B, 0x00, 0x96, 0x00, 
			0xA1, 0x00 }},
 
	{0xB5, 6,{
			0x02, 0x00, 0x03, 0x00, 
			0x04, 0x00 }}, 
	
	{0xB6, 2,{	0x00, 0x00 }},  
	
	{0xB7,22,{
			0x3E,0x00,0x5E,0x00,
			0x9E,0x00,0x74,0x00,
			0x8C,0x00,0xAC,0x00,
			0xDC,0x00,0x70,0x00,
			0xB9,0x00,0xEC,0x00,
			0xDC,0x00}},  

	{0xB8, 8,{
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00}},  
	
	{0xBA, 4,{
			0x24, 0x00, 0x00, 0x00}},
	  

	//CABC Manufacture Setting
	{0xC1, 6,{
			0x20, 0x00, 0x54, 0x00, 
			0xFF, 0x00}},  
	
	{0xC2, 4,{
			0x00, 0x00, 0x04, 0x00}},  
	
	{0xC3,48,{
			0x3C,0x00,0x3A,0x00,
			0x39,0x00,0x37,0x00,
			0x3C,0x00,0x36,0x00,
			0x32,0x00,0x2F,0x00,
			0x2C,0x00,0x29,0x00,
			0x26,0x00,0x24,0x00,
			0x24,0x00,0x23,0x00,
			0x3C,0x00,0x36,0x00,
			0x32,0x00,0x2F,0x00,
			0x2C,0x00,0x29,0x00,
			0x26,0x00,0x24,0x00,
			0x24,0x00,0x23,0x00}},  
		
	{0xC4,26,{
			0x62,0x00,0x05,0x00,
			0x84,0x00,0xF0,0x00,
			0x18,0x00,0xA4,0x00,
			0x18,0x00,0x50,0x00,
			0x0C,0x00,0x17,0x00,
			0x95,0x00,0xF3,0x00,
			0xE6,0x00}},  
		
	{0xC5,10,{
			0x32,0x00,0x44,0x00,
			0x65,0x00,0x76,0x00,
			0x88,0x00}},  
			
	{0xC6,6,{
			0x20,0x00,0x17,0x00,
			0x01,0x00}},  
		
	//CABC Test Mode Setting
	{0xC7,4,{
			0x00,0x00,0x00,0x00 }}, 
		
	{0xC8,4,{
			0x00,0x00,0x00,0x00}}, 
		
	//OSC Manufacture Setting 
	{0xC9,16,{
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00}},  
		
	{0xE0,36,{
			0x01,0x00,0x05,0x00,
			0x10,0x00,0x2F,0x00,
			0x49,0x00,0x5A,0x00,
			0x6F,0x00,0x87,0x00,
			0x95,0x00,0xA1,0x00,
			0xAD,0x00,0xB8,0x00,
			0xC0,0x00,0xC4,0x00,
			0xCC,0x00,0xCF,0x00,
			0xD7,0x00,0xF3,0x00}}, 
		
	{0xE2,36,{
			0x01,0x00,0x05,0x00,
			0x10,0x00,0x2F,0x00,
			0x49,0x00,0x5A,0x00,
			0x6F,0x00,0x87,0x00,
			0x95,0x00,0xA1,0x00,
			0xAD,0x00,0xB8,0x00,
			0xC0,0x00,0xC4,0x00,
			0xCC,0x00,0xCF,0x00,
			0xD7,0x00,0xF3,0x00}}, 
		
	{0xE4,36,{
			0x01,0x00,0x05,0x00,
			0x10,0x00,0x2F,0x00,
			0x49,0x00,0x5A,0x00,
			0x6F,0x00,0x87,0x00,
			0x95,0x00,0xA1,0x00,
			0xAD,0x00,0xB8,0x00,
			0xC0,0x00,0xC4,0x00,
			0xCC,0x00,0xCF,0x00,
			0xD7,0x00,0xF3,0x00}}, 
		      
	{0xE1,36,{
			0x01,0x00,0x05,0x00,
			0x10,0x00,0x2F,0x00,
			0x49,0x00,0x5A,0x00,
			0x6F,0x00,0x87,0x00,
			0x95,0x00,0xA1,0x00,
			0xAD,0x00,0xB8,0x00,
			0xC0,0x00,0xC4,0x00,
			0xCC,0x00,0xCF,0x00,
			0xD7,0x00,0xF3,0x00}}, 
		
	{0xE3,36,{
			0x01,0x00,0x05,0x00,
			0x10,0x00,0x2F,0x00,
			0x49,0x00,0x5A,0x00,
			0x6F,0x00,0x87,0x00,
			0x95,0x00,0xA1,0x00,
			0xAD,0x00,0xB8,0x00,
			0xC0,0x00,0xC4,0x00,
			0xCC,0x00,0xCF,0x00,
			0xD7,0x00,0xF3,0x00}}, 
		
	{0xE5,36,{
			0x01,0x00,0x05,0x00,
			0x10,0x00,0x2F,0x00,
			0x49,0x00,0x5A,0x00,
			0x6F,0x00,0x87,0x00,
			0x95,0x00,0xA1,0x00,
			0xAD,0x00,0xB8,0x00,
			0xC0,0x00,0xC4,0x00,
			0xCC,0x00,0xCF,0x00,
			0xD7,0x00,0xF3,0x00}}, 
	
	{0xE6,36,{
			0x21,0x00,0x55,0x00,
			0x99,0x00,0x77,0x00,
			0x77,0x00,0x76,0x00,
			0x78,0x00,0x98,0x00,
			0xBB,0x00,0x99,0x00,
			0x66,0x00,0x54,0x00,
			0x45,0x00,0x34,0x00,
			0x44,0x00,0x34,0x00}}, 
		
	{0xE7,36,{
			0x21,0x00,0x55,0x00,
			0x99,0x00,0x77,0x00,
			0x77,0x00,0x76,0x00,
			0x78,0x00,0x98,0x00,
			0xBB,0x00,0x99,0x00,
			0x66,0x00,0x54,0x00,
			0x45,0x00,0x34,0x00,
			0x44,0x00,0x34,0x00}}, 
		
	{0xE8,36,{
			0x21,0x00,0x55,0x00,
			0x99,0x00,0x77,0x00,
			0x77,0x00,0x76,0x00,
			0x78,0x00,0x98,0x00,
			0xBB,0x00,0x99,0x00,
			0x66,0x00,0x54,0x00,
			0x45,0x00,0x34,0x00,
			0x44,0x00,0x34,0x00}}, 

	//Manufacture Setting
	{0xE9,4,{
			0xAA, 0x00, 0x00, 0x00}},  
	
	//Enter Page0
	{0x00,1,{
			0xAA }}, 
	
	//Manufacture Test Setting
	{0xCF,17,{
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00}}, 
	 
	{0xF0,5,{
			0x00,0x50,0x00,0x00,
			0x00 }}, 
	
	{0xF1,1,{	0x01 }}, 
	
	{0xEE,2,{	0xDE,0x21}}, 
	
	{0xF3,1,{	0x00 }}, 
	
	//Gate Setting
	{0xF9,4,{	0x06,0x10,0x29,0x00}},
	 
	//Code Lock
	{0xDF,1,{	0x10}}, 
	{REGFLAG_DELAY, 100, {}},
			//TE On
	{0x35,1,{	0x00}}, 
	
	{0x3A,1,{	0x66}}, 
        

	{0x29,1,{	0x00}}, 
	{REGFLAG_DELAY, 50, {}},
	{0x2C,1,{	0x00}},
	

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

	// enable tearing-free
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	params->dsi.mode   = CMD_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_ONE_LANE;

	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB666;

	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_18BIT_RGB666;

	params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
	params->dsi.vertical_active_line=800;
	params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's

	params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
	params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4
	params->dsi.fbk_div =17;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		

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
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}


// zhoulidong  add for lcm detect (start)

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0, id2 = 0;
	unsigned char buffer[2];
	unsigned int data_array[16];


	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);	

	/*	
	data_array[0] = 0x00110500;		// Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	*/

	//*************Enable CMD2 Page1  *******************//
	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000108;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10); 

	data_array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 

	read_reg_v2(0xC5, buffer, 2);
	id = buffer[0]; //we only need ID
	id2= buffer[1]; //we test buffer 1

	return (NT33510_LCM_ID == id)?1:0;

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
//add (end)

LCM_DRIVER nt35310_dsi_cmd_djn_hvga_lcm_drv = 
{
   	.name			= "nt35310_dsi_cmd_djn_hvga",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = rgk_lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
       .update         = lcm_update,
#endif
};

