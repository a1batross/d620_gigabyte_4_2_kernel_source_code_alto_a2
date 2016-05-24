/* Copyright Statement:lisong_new
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

#ifndef _TOUCHPANEL_H__
#define _TOUCHPANEL_H__

#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE

#define TPD_POWER_SOURCE         MT65XX_POWER_LDO_VGP2
#define TPD_I2C_NUMBER           0
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_RES_X                480			//根据LCD分辨率设置
#define TPD_RES_Y                800			//根据LCD分辨率设置

#define TPD_CALIBRATION_MATRIX  {-256, 0, 1310720, 0, 287, 0,0,0};

//#define TPD_HAVE_CALIBRATION

#define TPD_HAVE_BUTTON

#ifdef TPD_HAVE_BUTTON
//#define TPD_KEY_COUNT           2
//#define TPD_KEYS                {KEY_MENU,KEY_BACK}
//#define TPD_KEYS_DIM            {{40,850,60,60},{280,850,60,60}}										//根据LCD分辨率设置
#define TPD_KEY_COUNT           3
#define TPD_KEYS                    {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
#define TPD_KEYS_DIM            {{40,850,80,80},{200,850,80,80},{360,850,80,80}}		//根据LCD分辨率设置
#endif

//#define	TPD_XY_INVERT						//交换X和Y
//#define	TPD_X_INVERT						//X翻转(对称)
//#define	TPD_Y_INVERT						//Y翻转(对称)

//#define	TPD_RSTPIN_1V8						//RESET PIN为1.8V时，要打开这个
#define	GPIO_CTP_RST_PIN_M_GPIO		0			//RESET PIN的IO口编组

//#define TPD_CLOSE_POWER_IN_SLEEP				//是否关闭TP电源,一般不要打开
//#define TP_DEBUG  							//调试信息开关
#define TP_FIRMWARE_UPDATE						//T卡升级功能开关,一般都要打开
//#define TP_PROXIMITY_SENSOR					//贴脸熄屏功能开关,需要时可以打开


#define TPD_HOME_KEY_LONG_PRESS //add by lisong

#endif /* _TOUCHPANEL_H__ */
