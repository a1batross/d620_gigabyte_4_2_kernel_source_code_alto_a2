/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 * 
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H

typedef enum S5K5EAYX_CAMCO_MODE
{
  S5K5EAYX_CAM_PREVIEW=0,//Camera Preview

  S5K5EAYX_CAM_CAPTURE,//Camera Capture

  S5K5EAYX_VIDEO_MPEG4,//Video Mode
  S5K5EAYX_VIDEO_MJPEG,

  S5K5EAYX_WEBCAM_CAPTURE,//WebCam

  S5K5EAYX_VIDEO_MAX
} S5K5EAYX_Camco_MODE;


struct S5K5EAYX_sensor_struct
{
        kal_uint16 sensor_id;

        kal_uint16 Dummy_Pixels;
        kal_uint16 Dummy_Lines;
        kal_uint32 Preview_PClk;

        kal_uint32 Preview_Lines_In_Frame;
        kal_uint32 Capture_Lines_In_Frame;

        kal_uint32 Preview_Pixels_In_Line;
        kal_uint32 Capture_Pixels_In_Line;
        kal_uint16 Preview_Shutter;
        kal_uint16 Capture_Shutter;

        kal_uint16 StartX;
        kal_uint16 StartY;
        kal_uint16 iGrabWidth;
        kal_uint16 iGrabheight;

        kal_uint16 Capture_Size_Width;
        kal_uint16 Capture_Size_Height;
        kal_uint32 Digital_Zoom_Factor;

        kal_uint16 Max_Zoom_Factor;

        kal_uint32 Min_Frame_Rate;
        kal_uint32 Max_Frame_Rate;
        kal_uint32 Fixed_Frame_Rate;
        //kal_bool Night_Mode;
        S5K5EAYX_Camco_MODE Camco_mode;
        AE_FLICKER_MODE_T Banding;

        kal_bool Night_Mode;
};


//Daniel


#define S5K5EAYX_WRITE_ID                     0xAC//0x7A//0x78//0x5A//0xAC (0x78)
#define S5K5EAYX_READ_ID                      0xAD//0x7B//0x79//0x5B//0xAD

#define S5K5EAYX2_WRITE_ID                      0x78
#define S5K5EAYX2_READ_ID                       0x79

#define S5K5EAYX_SENSOR_ID 					0x5EA1

/* SENSOR FULL/PV SIZE */
#define S5K5EAYX_IMAGE_SENSOR_FULL_WIDTH_DRV   (2560-24)
#define S5K5EAYX_IMAGE_SENSOR_FULL_HEIGHT_DRV  (1920-18)
#define S5K5EAYX_IMAGE_SENSOR_PV_WIDTH_DRV     (1280-16)
#define S5K5EAYX_IMAGE_SENSOR_PV_HEIGHT_DRV    (960-12)

#define S5K5EAYX_IMAGE_SENSOR_PV_WIDTH         (S5K5EAYX_IMAGE_SENSOR_PV_WIDTH_DRV)
#define S5K5EAYX_IMAGE_SENSOR_PV_HEIGHT        (S5K5EAYX_IMAGE_SENSOR_PV_HEIGHT_DRV) /* -2 for frame ready done */

#define S5K5EAYX_IMAGE_SENSOR_VIDEO_WIDTH_DRV         (S5K5EAYX_IMAGE_SENSOR_PV_WIDTH_DRV)
#define S5K5EAYX_IMAGE_SENSOR_VIDEO_HEIGHT_DRV        (S5K5EAYX_IMAGE_SENSOR_PV_HEIGHT_DRV) /* -2 for frame ready done */


/* SENSOR START/END POSITION */
#define S5K5EAYX_PV_X_START                     8    // 2
#define S5K5EAYX_PV_Y_START                     8    // 2
#define S5K5EAYX_FULL_X_START                  8
#define S5K5EAYX_FULL_Y_START                  8

/* Flicker factor to calculate tha minimal shutter width step for 50Hz and 60Hz  */
#define MACRO_50HZ                                                      (100)
#define MACRO_60HZ                                                      (120)

#define FACTOR_50HZ                                                     (MACRO_50HZ * 1000)
#define FACTOR_60HZ                                                     (MACRO_60HZ * 1000)



#endif /* __SENSOR_H */
