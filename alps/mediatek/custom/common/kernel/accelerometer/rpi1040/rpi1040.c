
/*************************************************************
1. xiaohui.han@ragentek.com 2013.2.20
   Description:add the file for RPI1040 GSENSOR
*************************************************************/
/* RPI1040 Accelerometer Sensor Driver
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
 * History: V1.00 --- [2013.01.10]Driver creation
 */

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>

#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE


#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include "cust_gpio_usage.h"
#include "rpi1040.h"

#define GPIO_GSENSOR_EN_PIN   GPIO_4D_GSENSOR_EN_PIN
#define GPIO_GSENSOR_X_PIN    GPIO_4D_GSENSOR_X_PIN
#define GPIO_GSENSOR_Y_PIN    GPIO_4D_GSENSOR_Y_PIN

#define RPI1040_DEV_NAME        "RPI1040"

static const struct i2c_device_id rpi1040_i2c_id[] = {{RPI1040_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_rpi1040={ I2C_BOARD_INFO(RPI1040_DEV_NAME, (0xff>>1))};

static struct i2c_client *rpi1040_i2c_client = NULL;

static struct platform_driver rpi1040_gsensor_driver;

static int rpi1040_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int rpi1040_i2c_remove(struct i2c_client *client);
static int rpi1040_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);




/* rpi1040 data */
struct rpi1040_i2c_data {
    struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert   cvt;
	u8 sensor_name[MAX_SENSOR_NAME];
	struct mutex lock;	
#if defined(CONFIG_HAS_EARLYSUSPEND)
        struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver rpi1040_i2c_driver = {
    .driver = {
//        .owner          = THIS_MODULE,
        .name           = RPI1040_DEV_NAME,
    },
	.probe      	= rpi1040_i2c_probe,
	.remove    	= rpi1040_i2c_remove,
	.id_table = rpi1040_i2c_id,
};

/*----------------------------------------------------------------------------*/

/* log macro */
#define GSE_TAG                  "[gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __func__)
#define GSE_ERR(fmt, args...) \
	printk(KERN_ERR GSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)





static struct rpi1040_i2c_data *obj_rpi1040_data;



static void rpi1040_power(unsigned int on)
{
	static unsigned int power_on;
	GSE_ERR("power begind on =%d \n",on);
	mutex_lock(&obj_rpi1040_data->lock);
		if (power_on == on) {/* power status not change */
			GSE_LOG("ignore power control: %d\n", on);
		} else if (on) {/* power on */
			mt_set_gpio_mode(GPIO_GSENSOR_EN_PIN, 0);
                        mt_set_gpio_dir(GPIO_GSENSOR_EN_PIN, GPIO_DIR_OUT);
	                mt_set_gpio_out(GPIO_GSENSOR_EN_PIN, GPIO_OUT_ONE);
		} else {/* power off */
		mt_set_gpio_mode(GPIO_GSENSOR_EN_PIN, 0);
                 mt_set_gpio_dir(GPIO_GSENSOR_EN_PIN, GPIO_DIR_OUT);
	         mt_set_gpio_out(GPIO_GSENSOR_EN_PIN, GPIO_OUT_ZERO);
		}
	
	power_on = on;
	mutex_unlock(&obj_rpi1040_data->lock);
	GSE_ERR("power end on=%d \n",on);
}

static int rpi1040_read_raw_data(s16 data[RPI1040_AXES_NUM])
{
	s8 buf[RPI1040_DATA_LEN] = {0};
	int err = 0;
    int gpio_x,gpio_y;
	memset(data, 0, sizeof(s16)*RPI1040_AXES_NUM);
	mutex_lock(&obj_rpi1040_data->lock);
	gpio_x = mt_get_gpio_in(GPIO_GSENSOR_X_PIN);
	gpio_y = mt_get_gpio_in(GPIO_GSENSOR_Y_PIN);
GSE_ERR("gpio_x =%d gpio_y=%d  \n ",gpio_x,gpio_y);
     if(gpio_x == 0 && gpio_y == 0)
	{
		data[RPI1040_AXIS_X] = 0x0;
		data[RPI1040_AXIS_Y]= -0x2648;
		data[RPI1040_AXIS_Z] = 0x0;
		
	}else if(gpio_x == 1 && gpio_y == 0)
	{
		data[RPI1040_AXIS_X] = -0x2648;
		data[RPI1040_AXIS_Y] = 0x0;
		data[RPI1040_AXIS_Z] = 0x0;
	}else if(gpio_x == 0 && gpio_y == 1)
	{
		data[RPI1040_AXIS_X] = 0x2648;
		data[RPI1040_AXIS_Y] = 0x0;
		data[RPI1040_AXIS_Z] = 0x0;
		
	}else if(gpio_x == 1 && gpio_y == 1)
	{

		data[RPI1040_AXIS_X] = 0x0;
		data[RPI1040_AXIS_Y] = 0x2648;
		data[RPI1040_AXIS_Z] = 0x0;
	}
	
	mutex_unlock(&obj_rpi1040_data->lock);
	return err;
}

/* rpi1040 setting initialization */
static int rpi1040_init_client(void)
{
	int err = 0;
	GSE_FUN(f);
	mutex_lock(&obj_rpi1040_data->lock);
    mt_set_gpio_mode(GPIO_GSENSOR_EN_PIN, 0);
    mt_set_gpio_dir(GPIO_GSENSOR_EN_PIN, GPIO_DIR_OUT);
	
    mt_set_gpio_mode(GPIO_GSENSOR_X_PIN, 0);
    mt_set_gpio_dir(GPIO_GSENSOR_X_PIN, GPIO_DIR_IN);
    //mt_set_gpio_pull_enable(GPIO_GSENSOR_X_PIN,GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_select(GPIO_GSENSOR_X_PIN,GPIO_PULL_UP);

    mt_set_gpio_mode(GPIO_GSENSOR_Y_PIN, 0);
    mt_set_gpio_dir(GPIO_GSENSOR_Y_PIN, GPIO_DIR_IN);
    //mt_set_gpio_pull_enable(GPIO_GSENSOR_Y_PIN,GPIO_PULL_ENABLE);
   // mt_set_gpio_pull_select(GPIO_GSENSOR_Y_PIN,GPIO_PULL_UP);
	mutex_unlock(&obj_rpi1040_data->lock);
	return 0;
}

/*
*Returns compensated and mapped value. unit is :1000*m/s^2
*/
static int rpi1040_read_sensor_data(
		char *buf, int bufsize)
{

	s16 databuf[RPI1040_AXES_NUM];
	int acc[RPI1040_AXES_NUM];
	int err = 0;
	 struct rpi1040_i2c_data *obj = obj_rpi1040_data;
	memset(databuf, 0, sizeof(s16)*RPI1040_AXES_NUM);
	memset(acc, 0, sizeof(int)*RPI1040_AXES_NUM);

	if (NULL == buf)
		return -1;


	err = rpi1040_read_raw_data(databuf);
	if (err) {
		GSE_ERR("rpi1040 read raw data failed, err = %d\n", err);
		return -3;
	} else {
		
		/* remap coordinate */
		acc[obj->cvt.map[RPI1040_AXIS_X]] =
			obj->cvt.sign[RPI1040_AXIS_X]*databuf[RPI1040_AXIS_X];
		acc[obj->cvt.map[RPI1040_AXIS_Y]] =
			obj->cvt.sign[RPI1040_AXIS_Y]*databuf[RPI1040_AXIS_Y];
		acc[obj->cvt.map[RPI1040_AXIS_Z]] =
			obj->cvt.sign[RPI1040_AXIS_Z]*databuf[RPI1040_AXIS_Z];

		

		sprintf(buf, "%04x %04x %04x",
			acc[RPI1040_AXIS_X], acc[RPI1040_AXIS_Y], acc[RPI1040_AXIS_Z]);
		
	}

	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	
    struct rpi1040_i2c_data *obj = obj_rpi1040_data;

	if (NULL == obj) {
		GSE_ERR("rpi1040 data pointer is null\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", obj->sensor_name);
}

/*
* sensor data format is hex, unit:1000*m/s^2
*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	
    struct rpi1040_i2c_data *obj = obj_rpi1040_data;
	
	char strbuf[RPI1040_BUFSIZE] = "";

	if (NULL == obj) {
		GSE_ERR("rpi1040 data pointer is null\n");
		return 0;
	}

	rpi1040_read_sensor_data(strbuf, RPI1040_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_power_mode_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int gpio_en;
	struct rpi1040_i2c_data *obj = obj_rpi1040_data;

	if (obj == NULL) {
		GSE_ERR("rpi1040 data pointer is null\n");
		return 0;
	}
    gpio_en = mt_get_gpio_in(GPIO_GSENSOR_EN_PIN);
	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n",
		gpio_en);

	return len;
}

static ssize_t store_power_mode_value(struct device_driver *ddri,
		const char *buf, size_t count)
{
	struct rpi1040_i2c_data *obj = obj_rpi1040_data;
	unsigned long power_mode;
	int err;

	if (obj == NULL) {
		GSE_ERR("rpi1040 data pointer is null\n");
		return 0;
	}

	err = strict_strtoul(buf, 10, &power_mode);

	if (err == 0) {
		rpi1040_power(power_mode);
		return count;
	}
	return err;
}





static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(powermode, S_IWUSR | S_IRUGO,
		show_power_mode_value, store_power_mode_value);
static struct driver_attribute *rpi1040_attr_list[] = {
	/* chip information */
	&driver_attr_chipinfo,
	/* sensor  data  information */
	&driver_attr_sensordata,
	/* power status  information */
	&driver_attr_powermode,
};

static int rpi1040_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(rpi1040_attr_list)/sizeof(rpi1040_attr_list[0]));
	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, rpi1040_attr_list[idx]);



		if (err) {
			GSE_ERR("driver_create_file (%s) = %d\n",
				rpi1040_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int rpi1040_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;

	int num = (int)(sizeof(rpi1040_attr_list)/sizeof(rpi1040_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, rpi1040_attr_list[idx]);

	return err;
}

int rpi1040_gsensor_operate(void *self, uint32_t command, void *buff_in, int size_in,
		void *buff_out, int size_out, int *actualout)
{
	int err = 0;
	int value;
	struct rpi1040_i2c_data *priv = (struct rpi1040_i2c_data *)self;
	hwm_sensor_data *gsensor_data;
	char buff[RPI1040_BUFSIZE];

	switch (command) {
	case SENSOR_DELAY:
	
	break;
	case SENSOR_ENABLE:
	if ((buff_in == NULL) || (size_in < sizeof(int))) {
		GSE_ERR("enable sensor parameter error\n");
		err = -EINVAL;
	} else {
		/* value:[0--->suspend, 1--->normal] */
		value = *(int *)buff_in;
		GSE_LOG("sensor enable/disable command: %s\n",
			value ? "enable" : "disable");
		rpi1040_init_client();
		 rpi1040_power(value);
	}
	break;
	case SENSOR_GET_DATA:
	if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data))) {
		GSE_ERR("get sensor data parameter error\n");
		err = -EINVAL;
	} else {
		gsensor_data = (hwm_sensor_data *)buff_out;
		rpi1040_read_sensor_data(buff, RPI1040_BUFSIZE);
		sscanf(buff, "%x %x %x", &gsensor_data->values[0],								
			&gsensor_data->values[1], &gsensor_data->values[2]);
		gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		gsensor_data->value_divide = 1000;
	}
	break;
	default:
	GSE_ERR("gsensor operate function no this parameter %d\n", command);
	err = -1;
	break;
	}

	return err;
}

static int rpi1040_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_rpi1040_data;

	if (file->private_data == NULL) {
		GSE_ERR("null pointer\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int rpi1040_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long rpi1040_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct rpi1040_i2c_data *obj = (struct rpi1040_i2c_data *)file->private_data;
	char strbuf[RPI1040_BUFSIZE] = "";
	s16 raw_data[RPI1040_AXES_NUM] = {0};
	void __user *data;
	long err = 0;
	int cali[RPI1040_AXES_NUM];

	if (obj == NULL)
		return -EFAULT;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
			(void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		GSE_ERR("access error: %08x, (%2d, %2d)\n",
			cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
	rpi1040_power(0);
	rpi1040_init_client();
	rpi1040_power(1);
	break;
	case GSENSOR_IOCTL_READ_CHIPINFO:
	data = (void __user *) arg;
	if (data == NULL) {
		err = -EINVAL;
		break;
	}

	strcpy(strbuf, obj->sensor_name);
	if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
		err = -EFAULT;
		break;
	}
	break;
	case GSENSOR_IOCTL_READ_SENSORDATA:
	data = (void __user *) arg;
	if (data == NULL) {
		err = -EINVAL;
		break;
	}

	rpi1040_read_sensor_data(strbuf, RPI1040_BUFSIZE);
	if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
		err = -EFAULT;
		break;
	}
	break;
	//case GSENSOR_IOCTL_READ_GAIN:
	
	//break;
	case GSENSOR_IOCTL_READ_RAW_DATA:
	data = (void __user *) arg;
	if (data == NULL) {
		err = -EINVAL;
		break;
	}
	err = rpi1040_read_raw_data(raw_data);
	if (err) {
		err = -EFAULT;
		break;
	}
	sprintf(strbuf, "%04x %04x %04x",
			raw_data[RPI1040_AXIS_X],
			raw_data[RPI1040_AXIS_X],
			raw_data[RPI1040_AXIS_X]);

	if (copy_to_user(data, &strbuf, strlen(strbuf) + 1)) {
		err = -EFAULT;
		break;
	}
	break;
	//case GSENSOR_IOCTL_SET_CALI:
	
	//break;
	//case GSENSOR_IOCTL_CLR_CALI:
	
	//break;
	default:
	GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
	err = -ENOIOCTLCMD;
	break;
	}

	return err;
}

static const struct file_operations rpi1040_fops = {
	.owner = THIS_MODULE,
	.open = rpi1040_open,
	.release = rpi1040_release,
	.unlocked_ioctl = rpi1040_unlocked_ioctl,
};

static struct miscdevice rpi1040_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &rpi1040_fops,
};

static void rpi1040_early_suspend(struct early_suspend *h) 
{
	struct rpi1040_i2c_data *obj = container_of(h, struct rpi1040_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	rpi1040_power(0);
}
/*----------------------------------------------------------------------------*/
static void rpi1040_late_resume(struct early_suspend *h)
{
	struct rpi1040_i2c_data *obj = container_of(h, struct rpi1040_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	
	rpi1040_power(1);
}
#ifdef GSENSOR_BE_COMPATIABLE 
extern int kxtik1004_exist;

extern int lis3dh_exist;

#endif
static int rpi1040_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{   
    	struct i2c_client *new_client;
	struct rpi1040_i2c_data  *obj;
	struct hwmsen_object sobj;
	int err = 0;
	 printk("%s \n",__func__,__LINE__);
	#ifdef GSENSOR_BE_COMPATIABLE 
	if( (kxtik1004_exist)||(lis3dh_exist) )
	return 0;
	#endif
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	//obj->sensor_name = "rpi1040";
	strcpy(obj->sensor_name, "rpi1040");
	obj->hw = get_cust_acc_hw();
	#ifdef PROJECT_CUSTOMER_D207
	err = hwmsen_get_convert(4, &obj->cvt);
	#else
	err = hwmsen_get_convert(0, &obj->cvt);
	#endif
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit_hwmsen_get_convert_failed;
	}

	obj_rpi1040_data = obj;
   	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	mutex_init(&obj->lock);

   	rpi1040_i2c_client = new_client;	
	err = rpi1040_init_client();
	if (err)
		goto exit_init_client_failed;

	err = misc_register(&rpi1040_device);
	if (err) {
		GSE_ERR("misc device register failed, err = %d\n", err);
		goto exit_misc_device_register_failed;
	}

	err = rpi1040_create_attr(&rpi1040_gsensor_driver.driver);
	if (err) {
		GSE_ERR("create attribute failed, err = %d\n", err);
		goto exit_create_attr_failed;
	}

	sobj.self = obj;
	sobj.polling = 1;
	sobj.sensor_operate = rpi1040_gsensor_operate;

	err = hwmsen_attach(ID_ACCELEROMETER, &sobj);
	if (err) {
		GSE_ERR("hwmsen attach failed, err = %d\n", err);
		goto exit_hwmsen_attach_failed;
	}

    #ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = rpi1040_early_suspend,
	obj->early_drv.resume   = rpi1040_late_resume,    
	register_early_suspend(&obj->early_drv);
    #endif 
	GSE_LOG("%s: OK\n", __func__);
	return 0;

exit_hwmsen_attach_failed:
	rpi1040_delete_attr(&rpi1040_gsensor_driver.driver);
exit_create_attr_failed:
	misc_deregister(&rpi1040_device);
exit_misc_device_register_failed:
exit_init_client_failed:
exit_hwmsen_get_convert_failed:
	kfree(obj);
exit:
	GSE_ERR("err = %d\n", err);
	return err;
}

static int rpi1040_i2c_remove(struct i2c_client *client)
{
	int err = 0;
	
	err = hwmsen_detach(ID_ACCELEROMETER);
	if (err)
		GSE_ERR("hwmsen_detach failed, err = %d\n", err);

	err = rpi1040_delete_attr(&rpi1040_gsensor_driver.driver);
	if (err)
		GSE_ERR("rpi1040_delete_attr failed, err = %d\n", err);

	err = misc_deregister(&rpi1040_device);
	if (err)
		GSE_ERR("misc_deregister failed, err = %d\n", err);
	
	rpi1040_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
    rpi1040_power(0);
    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int rpi1040_probe(struct platform_device *pdev) 
{
	struct acc_hw *hw = get_cust_acc_hw();
    printk("%s \n",__func__,__LINE__);
	//kxtik1004_force[0] = hw->i2c_num;
	if(i2c_add_driver(&rpi1040_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int rpi1040_remove(struct platform_device *pdev)
{   
    rpi1040_power(0);
    i2c_del_driver(&rpi1040_i2c_driver);
    return 0;
}

static struct platform_driver rpi1040_gsensor_driver = {
	.probe      = rpi1040_probe,
	.remove     = rpi1040_remove,
	.driver     = {
		.name   = "rpi1040_gsensor",
	}
};

static int __init rpi1040_init(void)
{
	struct acc_hw *hw = get_cust_acc_hw();
	 printk("%s \n",__func__,__LINE__);
    i2c_register_board_info(hw->i2c_num, &i2c_rpi1040, 1);
    
	if (platform_driver_register(&rpi1040_gsensor_driver)) {
		GSE_ERR("register gsensor platform driver failed\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit rpi1040_exit(void)
{
	GSE_FUN();
	platform_driver_unregister(&rpi1040_gsensor_driver);
}

module_init(rpi1040_init);
module_exit(rpi1040_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("rpi1040 sensor Driver");
