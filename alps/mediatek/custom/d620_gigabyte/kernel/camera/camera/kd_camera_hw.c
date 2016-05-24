
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
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
//#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_DBG_FUNC printk

#define DEBUG_CAMERA_HW_K
#ifdef  DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_XLOG_INFO(fmt, args...) \
        do {    \
                    xlog_printk(ANDROID_LOG_INFO, "kd_camera_hw", fmt, ##args); \
        } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif



static void MainCameraDigtalPowerCtrl(kal_bool on){
    if(mt_set_gpio_mode(GPIO_MAIN_CAMERA_12V_POWER_CTRL_PIN,0)){PK_DBG("[[CAMERA SENSOR] Set MAIN CAMERA_DIGITAL POWER_PIN ! \n");}
    if(mt_set_gpio_dir(GPIO_MAIN_CAMERA_12V_POWER_CTRL_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n");}
    if(mt_set_gpio_out(GPIO_MAIN_CAMERA_12V_POWER_CTRL_PIN,on)){PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n");;}
}



/*#ifndef GPIO_MAIN_CAMERA_28V_POWER_CTRL_PIN 
#define GPIO_MAIN_CAMERA_28V_POWER_CTRL_PIN GPIO37
#endif 
static void MainCameraAnalogPowerCtrl(kal_bool on){
    if(mt_set_gpio_mode(GPIO_MAIN_CAMERA_28V_POWER_CTRL_PIN,0)){PK_DBG("[[CAMERA SENSOR] Set MAIN CAMERA_DIGITAL POWER_PIN ! \n");}
    if(mt_set_gpio_dir(GPIO_MAIN_CAMERA_28V_POWER_CTRL_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n");}
    if(mt_set_gpio_out(GPIO_MAIN_CAMERA_28V_POWER_CTRL_PIN,on)){PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n");;}
}*/



int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
    u32 pinSetIdx = 0;//default main sensor
    u32 pinSetIdxTmp = 0;

    #define IDX_PS_CMRST 0
    #define IDX_PS_CMPDN 4

    #define IDX_PS_MODE 1
    #define IDX_PS_ON   2
    #define IDX_PS_OFF  3
    u32 pinSet[2][8] = {
          //for main sensor
          {GPIO_CAMERA_CMRST_PIN,
              GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
              GPIO_OUT_ONE,                   /* ON state */
              GPIO_OUT_ZERO,                  /* OFF state */
           GPIO_CAMERA_CMPDN_PIN,
              GPIO_CAMERA_CMPDN_PIN_M_GPIO,
              GPIO_OUT_ONE,
              GPIO_OUT_ZERO,
          },
          //for sub sensor
          {GPIO_CAMERA_CMRST1_PIN,
           GPIO_CAMERA_CMRST1_PIN_M_GPIO,
              GPIO_OUT_ONE,
              GPIO_OUT_ZERO,
           GPIO_CAMERA_CMPDN1_PIN,
              GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
              GPIO_OUT_ONE, 
              GPIO_OUT_ZERO,
          }
         };



    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }


    PK_DBG("sym kdCISModulePowerOn: 4EC 8AA, currSensorName=%s,pinSetIdx=%d, On=%d\n",currSensorName,pinSetIdx,On);
    //power ON
    if (On) {

       PK_DBG("kdCISModulePowerOn -on:currSensorName=%s;\n",currSensorName);

	   MainCameraDigtalPowerCtrl(1);
	    if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5645_MIPI_YUV,currSensorName)) )
       {
		   PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_OV5645_MIPI_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);
	   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");//2.8 ->1.8
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
		   else
		   	  PK_DBG("[poweron] enable CAMERA_POWER_VCAM_D2\n");	   
		   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
	   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))   //sym modify
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
	   
	   	   /*
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
		   */ 

		   msleep(5);

		   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		   {
			 //PDN pin
			 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}  //sym modify
			 msleep(3);
	   
			 //RST pin
			 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			 msleep(20);
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			 msleep(1);
		   }   
		   
		   //disable inactive sensor
		   if(pinSetIdx == 0) {//disable sub
			   pinSetIdxTmp = 1;
		   }
		   else{
			   pinSetIdxTmp = 0;
		   }
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		   }
	   }
       
       if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_GC2035_YUV,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_GC2035_PL_YUV,currSensorName))))
       {
		   PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_GC2035_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);
	   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");//2.8 ->1.8
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
		   else
		   	  PK_DBG("[poweron] enable CAMERA_POWER_VCAM_D2\n");	   
		   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
	   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))   //sym modify
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
	   
	   	   /*
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
		   */
		   msleep(5);

		   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		   {
			 //PDN pin
			 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}  //sym modify
			 msleep(3);
	   
			 //RST pin
			 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			 msleep(3);
		   }   
		   
		   //disable inactive sensor
		   if(pinSetIdx == 0) {//disable sub
			   pinSetIdxTmp = 1;
		   }
		   else{
			   pinSetIdxTmp = 0;
		   }
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		   }
	   }
       else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5EAYX_YUV,currSensorName)))
       {
          PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_S5K5EAYX_YUV \n");
          PK_DBG("pinSetIdx:%d,IDX_PS_CMPDN=%d,IDX_PS_CMRST=%d \n",pinSetIdx,pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST]);

		  if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
		  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
		  if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}

		  if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		  if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}


	   
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
          {
             printk("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
       
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
          {
             printk("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
       
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
          {
             printk("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
           
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
          {
             printk("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
          
          //PDN/STBY pin
          //if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
          {

			 PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_S5K5EAYX_YUV -Do pwd\n");
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             mdelay(5);
       
			 PK_DBG("kdCISModulePowerOn  SENSOR_DRVNAME_S5K5EAYX_YUV IDX_PS_CMRST]=%d\n",pinSet[pinSetIdx][IDX_PS_CMRST]);
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(5);
          }

	   /*
          //disable inactive sensor
          if(pinSetIdx == 0) {//disable sub
             pinSetIdxTmp = 1;
          }
          else{
             pinSetIdxTmp = 0;
          }
          if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
             if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
             if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
             if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
          }
		  */
       }
	     else  if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC0329_YUV,currSensorName)))
       {
	if(pinSetIdx==1)
	{	
	   PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_GC0329_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);
	   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");//2.8 ->1.8
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
		   else
		   	  PK_DBG("[poweron] enable CAMERA_POWER_VCAM_D2\n");	   
		   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
	   
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))   //sym modify
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
	   
	   	   /*
		   if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
		   {
			   PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			   //return -EIO;
			   goto _kdCISModulePowerOn_exit_;
		   }
		   */
		   msleep(5);

		   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		   {
			 //PDN pin
			 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}  //sym modify
			 msleep(3);
	   
			 //RST pin
			 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			 msleep(3);
		   }   
		   
		   //disable inactive sensor
		   if(pinSetIdx == 0) {//disable sub
			   pinSetIdxTmp = 1;
		   }
		   else{
			   pinSetIdxTmp = 0;
		   }
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		   }
	   }

}
	  else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_SP2518_YUV,currSensorName)))
       {  
		   PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_SP2518_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);

          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
	   
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800/*VOL_2800*/,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
	else
       	  PK_DBG("[poweron] enable CAMERA_POWER_VCAM_D2\n");	 
	

       
         if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
       #if 0
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
       #endif
          //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])  
          {
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(10);
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(5);
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(5);
             //RST pin
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(10);
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(5);
          }
       
          //disable inactive sensor
          if(pinSetIdx == 0) {//disable sub
             pinSetIdxTmp = 1;
          }
          else{
             pinSetIdxTmp = 0;
          }
          if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
             if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
             if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
             if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
          }          
       }
       else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_SP0A19_YUV,currSensorName)))
       {  
		   PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_SPA019_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);
       
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800/*VOL_2800*/,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
	else
       	  PK_DBG("[poweron] enable CAMERA_POWER_VCAM_D2\n");	 
       
          if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
       
         if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }
       
          /*if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
          {
             PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
             //return -EIO;
             goto _kdCISModulePowerOn_exit_;
          }*/
       
          //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])  
          {
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(10);
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(5);
        if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
             msleep(5);
             //RST pin
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(10);
             if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
             mdelay(5);
          }
       
          //disable inactive sensor
          if(pinSetIdx == 0) {//disable sub
             pinSetIdxTmp = 1;
          }
          else{
             pinSetIdxTmp = 0;
          }
          if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
             if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
             if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
             if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
             if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
             if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
          }          
       }
    }
    else {//power OFF
if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_SP2518_YUV,currSensorName)))
{
msleep(5);

		   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		   {
			 //PDN pin
			 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}  //sym modify
			 msleep(3);
	   
			 //RST pin
			 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			 msleep(3);
		   }   
       
          MainCameraDigtalPowerCtrl(0);
     
  if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
           PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }

  
       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
       {
           PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }
       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
           PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }
       
     
  
}
else
{
       PK_DBG("kdCISModulePowerOn -off:currSensorName=%s\n",currSensorName);
	if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5645_MIPI_YUV,currSensorName))){
    		 if ((GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) ||(GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])){
           		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
          		 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
         		  if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
       
        		   if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
         		  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
         		  if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
    		   }
	}else{
       	if ((GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) ||(GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])) {
           if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
           if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
           if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
       
           if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
           if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
           if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
       }
		}

       //if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K4ECGX_MIPI_YUV,currSensorName)))
       {
          MainCameraDigtalPowerCtrl(0);
       }

       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
           PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }
       
       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
           PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }
       
       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
       {
           PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }
       
       if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
       {
           PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
           //return -EIO;
           goto _kdCISModulePowerOn_exit_;
       }
  }
    	}
  return 0;
_kdCISModulePowerOn_exit_:
    return -EIO;
}


EXPORT_SYMBOL(kdCISModulePowerOn);

//!--
//
