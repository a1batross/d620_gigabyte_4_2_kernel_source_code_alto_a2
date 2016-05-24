/*************************************************************
1. xiaohui.han@ragentek.com 2013.2.20
   Description:add the file for RPI1040 GSENSOR
*************************************************************/
/* Accelerometer Sensor Driver Header File
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef RPI1040_H
#define RPI1040_H

#include <linux/ioctl.h>

#define RPI1040_AXIS_X				0
#define RPI1040_AXIS_Y				1
#define RPI1040_AXIS_Z				2
#define RPI1040_AXES_NUM				3
#define RPI1040_DATA_LEN				6
#define RPI1040_DEV_NAME				"RPI1040"

#define RPI1040_BUFSIZE				128

#define MAX_SENSOR_NAME				(32)

#endif/* BMA2XX_H */
