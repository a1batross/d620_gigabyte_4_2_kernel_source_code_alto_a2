#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/dma-mapping.h>

#include "tpd_custom_ft5206.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "cust_gpio_usage.h"


#define DIJIN_FW_MODULE_ID  0x67
#define DIJIN_FW_ONLY_ID	0x01
#define JD_FW_MODULE_ID 	 0x85
#define JD_FW_ONLY_ID  		0x02

 
 
extern struct tpd_device *tpd;
 
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

static unsigned char g_fw_ver, g_module_id, g_only_id;

#ifdef RGK_TP_AUTO_UPGRADE_SUPPORT
#define AUTO_UPGRADE
#endif

#ifdef RGK_TP_NODE_UPGRAER_SUPPORT
#define NODE_UPGRADE
#endif

 /*register address*/
#define FT6x06_REG_FW_VER		0xA6
#define FT6x06_REG_MODULE_ID		0xA8
#define FT6x06_REG_ONLY_ID		0xB6
#define FT6x06_REG_POINT_RATE	0x88
#define FT6x06_REG_THGROUP	0x80

static u8 *CTPI2CDMABuf_va = NULL;
static u32 CTPI2CDMABuf_pa = NULL;

#if	defined(AUTO_UPGRADE) ||defined(NODE_UPGRADE) 

#define FT_UPGRADE_AA	0xAA
#define FT_UPGRADE_55 	0x55

/*upgrade config of FT6X06*/
#define FT6X06_UPGRADE_AA_DELAY 		100
#define FT6X06_UPGRADE_55_DELAY 		10
#define FT6X06_UPGRADE_ID_1			0x79
#define FT6X06_UPGRADE_ID_2			0x08
#define FT6X06_UPGRADE_READID_DELAY 	10
#define FT6X06_UPGRADE_EARSE_DELAY	2000

// #define FTS_PACKET_LENGTH        128
#define FTS_PACKET_LENGTH        128
#define FTS_SETTING_BUF_LEN        128

#define FTS_UPGRADE_LOOP	10

#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00

#define  DEBUG 1

#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...)
#endif

#ifdef DEBUG
#define dev_dbg(dev, format, arg...)		\
	dev_printk(KERN_DEBUG, dev, format, ##arg)
#endif


static unsigned char CTPM_FW_97_300400319A[] = {
	#include "FT6206_Ragentek_D206_Dijing_0319A_0x1b_20130620_app.i"
};
static unsigned char CTPM_FW_48_10040_0346A[] = {
	#include "FT6206_Ragentek_D206_Dijing_0319A_0x1b_20130620_app.i"
};
static unsigned char CTPM_FW_JD_1189B[] = {
	#include "FT6206_Ragentek_D206_Junda_1190B_0x05_20130620_app.i"
};
static unsigned char CTPM_FW_JD_1190B[] = {
	#include "FT6206_Ragentek_D206_Junda_1190B_0x05_20130620_app.i"
};
static unsigned char CTPM_FW_TRULY_CT1F0467[] = {
	#include "FT6206_Ragentek_D206_TRULY_CT1F0467_0x0d_app.i"
};

#endif


 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);

extern  int  fix_tp_proc_info(void  *tp_data, u8 data_len);

 
static void tpd_eint_interrupt_handler(void);
 
#ifdef MT6575 
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
#endif
#ifdef MT6577
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 

static int tpd_flag = 0;
static int tpd_halt=0;
static int point_num = 0;
static int p_point_num = 0;

//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 3

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5206
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;

	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	//char strbuf[256];
	void __user *data;
	
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;


		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[3];
    int x[3];
    int p[3];
    int id[3];
    int count;
};
 
 static const struct i2c_device_id ft5206_tpd_id[] = {{"ft5206",0},{}};
 //unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
 //static const unsigned short * const forces[] = { force, NULL };
 //static struct i2c_client_address_data addr_data = { .forces = forces, };
 static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO("ft5206", (0x70>>1))};
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "ft5206",//.name = TPD_DEVICE,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = ft5206_tpd_id,
  .detect = tpd_detect,
//  .address_data = &addr_data,
 };
 

static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	if(x > TPD_RES_X)
	{
		TPD_DEBUG("warning: IC have sampled wrong value.\n");;
		return;
	}
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 //printk("D[%4d %4d %4d] ", x, y, p);
	 /* track id Start 0 */
       input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	 input_mt_sync(tpd->dev);
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
       tpd_button(x, y, 1);  
     }
	 if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	 {
         msleep(50);
		 printk("D virtual key \n");
	 }
	 TPD_EM_PRINT(x, y, x, y, p-1, 1);
 }
 
static  void tpd_up(int x, int y,int *count) {
	 //if(*count>0) {
		 //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
		 //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 //printk("U[%4d %4d %4d] ", x, y, 0);
		 input_mt_sync(tpd->dev);
		 TPD_EM_PRINT(x, y, x, y, 0, 0);
	//	 (*count)--;
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
        tpd_button(x, y, 0); 
     }   		 

 }

 static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 {

	int i = 0;
	
	char data[30] = {0};

    u16 high_byte,low_byte;
	u8 report_rate =0;

	p_point_num = point_num;
	mutex_lock(&i2c_access);
	if (tpd_halt)
	{
		mutex_unlock(&i2c_access);
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[24]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &report_rate);
	//TPD_DEBUG("FW version=%x]\n",data[24]);
	
	//TPD_DEBUG("received raw data from touch panel as following:\n");
	//TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
	//TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	//TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);


    //    
	 //we have  to re update report rate
    // TPD_DMESG("report rate =%x\n",report_rate);
	 if(report_rate < 8)
	 {
	   report_rate = 0x8;
	   if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	   {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
	   }
	 }
	 
	mutex_unlock(&i2c_access);
	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if((data[0] & 0x70) != 0) return false; 

	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;
	
	//TPD_DEBUG("point_num =%d\n",point_num);
	
//	if(point_num == 0) return false;

	   //TPD_DEBUG("Procss raw data...\n");

		
		for(i = 0; i < point_num; i++)
		{
			cinfo->p[i] = data[3+6*i] >> 6; //event flag 
                   cinfo->id[i] = data[3+6*i+2]>>4; //touch id
	       /*get the X coordinate, 2 bytes*/
			high_byte = data[3+6*i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i + 1];
			cinfo->x[i] = high_byte |low_byte;

				//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
		
			/*get the Y coordinate, 2 bytes*/
			
			high_byte = data[3+6*i+2];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i+3];
			cinfo->y[i] = high_byte |low_byte;

			  //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
		
			cinfo->count++;
			
		}
		//TPD_DEBUG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
		//TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
		//TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
		  
	 return true;

 };

 static int touch_event_handler(void *unused)
 {
  
    struct touch_info cinfo, pinfo;
	 int i=0;

	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
 
	 do
	 {
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 set_current_state(TASK_INTERRUPTIBLE); 
		  wait_event_interruptible(waiter,tpd_flag!=0);
						 
			 tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);
		 

		  if (tpd_touchinfo(&cinfo, &pinfo)) 
		  {
		    //TPD_DEBUG("point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
			if(point_num >0) 
			{
			    for(i =0; i<point_num && i<3; i++)//only support 3 point
			    {

			#if 0
			    if (cinfo.y[i] <= 800)
			    {
				   	cinfo.x[i] = 480 -cinfo.x[i];
					cinfo.y[i] = 800 -cinfo.y[i];
			    }
				printk("\n@@@cinfo.x[%d] = %d,cinfo.y[%d] = %d \n", i, cinfo.x[i], i , cinfo.y[i]);
				
			  if (cinfo.y[i] > 800)
			  {
			  	printk("\n @##x = %d, y = %d",cinfo.x[i], cinfo.y[i]);
				if (cinfo.x[i] > 0 && cinfo.x[i] < 160)
				{
					mt_set_gpio_mode(GPIO58, GPIO_MODE_00);
					mt_set_gpio_dir(GPIO58, GPIO_DIR_OUT);
					mt_set_gpio_out(GPIO58, GPIO_OUT_ZERO);
					msleep(1);
				}
				if (cinfo.x[i] > 160 && cinfo.x[i] < 320)
				{
					mt_set_gpio_mode(GPIO58, GPIO_MODE_00);
					mt_set_gpio_dir(GPIO58, GPIO_DIR_OUT);
					mt_set_gpio_out(GPIO58, GPIO_OUT_ONE);
					msleep(1);
				}
				if (cinfo.x[i] > 320 && cinfo.x[i] < 480)
				{
					mt_set_gpio_mode(GPIO58, GPIO_MODE_00);
					mt_set_gpio_dir(GPIO58, GPIO_DIR_OUT);
					mt_set_gpio_out(GPIO58, GPIO_OUT_ZERO);
					msleep(1);
				}
			  }
			  #endif
			         tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
			       
			    }
			    input_sync(tpd->dev);
			}

			else  
            {
			    tpd_up(cinfo.x[0], cinfo.y[0], 0);
                //TPD_DEBUG("release --->\n"); 
                //input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }
        }

        if(tpd_mode==12)
        {
           //power down for desence debug
           //power off, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#else
	hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif 
	    msleep(20);
          
        }

 }while(!kthread_should_stop());
 
	 return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	 //TPD_DEBUG("TPD interrupt has been triggered\n");
	 TPD_DEBUG_PRINT_INT;
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }


int ft6x06_i2c_Write(struct i2c_client *client, char *pbt_buf, int dw_len)
{
    
	int i = 0;
	for(i = 0 ; i < dw_len; i++)
	{
		CTPI2CDMABuf_va[i] = pbt_buf[i];
	}

	if(dw_len <= 8)
	{
		client->addr = client->addr & I2C_MASK_FLAG;///hac 
		return i2c_master_send(client, pbt_buf, dw_len);
	}
	else
	{
		//i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;//| I2C_ENEXT_FLAG;//add FAE
                client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;//add FAE
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
		return i2c_master_send(client, CTPI2CDMABuf_pa, dw_len);
	}    
}


int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}


static void get_tp_ic_info(unsigned char * fw_ver, unsigned char *module_id, unsigned char *only_id)
{
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	
	/*get some register information */
	uc_reg_addr = FT6x06_REG_FW_VER;
	ft6x06_i2c_Read(i2c_client, &uc_reg_addr, 1, &uc_reg_value, 1);
	* fw_ver = uc_reg_value;

	uc_reg_addr = FT6x06_REG_MODULE_ID;
	ft6x06_i2c_Read(i2c_client, &uc_reg_addr, 1, &uc_reg_value, 1);
	* module_id = uc_reg_value;

	uc_reg_addr = FT6x06_REG_ONLY_ID;
	ft6x06_i2c_Read(i2c_client, &uc_reg_addr, 1, &uc_reg_value, 1);
	* only_id = uc_reg_value;
}


#if	defined(AUTO_UPGRADE) ||defined(NODE_UPGRADE) 
int ft6x06_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft6x06_i2c_Write(client, buf, sizeof(buf));
}


int ft6x06_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return ft6x06_i2c_Read(client, &regaddr, 1, regvalue, 1);
}


u8 fts_ctpm_get_i_file_ver(void)
{
	u16 ui_sz;

	if (g_module_id == DIJIN_FW_MODULE_ID && g_only_id == DIJIN_FW_ONLY_ID)
	{
		ui_sz = sizeof(CTPM_FW_97_300400319A);
		if (ui_sz > 2)
			return CTPM_FW_97_300400319A[ui_sz - 2];

	}
	else if (g_module_id == JD_FW_MODULE_ID &&  g_only_id == JD_FW_ONLY_ID)
	{
		ui_sz = sizeof(CTPM_FW_JD_1189B);
		if (ui_sz > 2)
			return CTPM_FW_JD_1189B[ui_sz - 2];
	}
	else
	{
			return 0x00;	/*default value */

	}

}

int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	int n;
	int read_id_delay =5;

	printk("\n@@@%s line = %d", __func__, __LINE__);
	
	msleep(500);

	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		#if 0
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xbc */
		
		ft6x06_write_reg(client, 0xbc, FT_UPGRADE_AA);
		msleep(FT6X06_UPGRADE_AA_DELAY);

		/*write 0x55 to register 0xbc */
		ft6x06_write_reg(client, 0xbc, FT_UPGRADE_55);

		msleep(FT6X06_UPGRADE_55_DELAY);
		#else
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
		msleep(30);
		TPD_DMESG(" ft5306 reset\n");
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);	
		#endif

		msleep(30);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		n = 0;
		do {
			n++;
			i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);

			msleep(5);
		} while (i_ret <= 0 && n < 5);

		/*********Step 3:check READ-ID***********************/
		msleep(FT6X06_UPGRADE_READID_DELAY);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =	0x00;
		ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

		printk("@@func= %s, line = %d, CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", __func__, __LINE__,reg_val[0], reg_val[1]);


		if (reg_val[0] == FT6X06_UPGRADE_ID_1
			&& reg_val[1] == FT6X06_UPGRADE_ID_2) {
			//dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				//reg_val[0], reg_val[1]);
			printk("@@@[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&client->dev, "###[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i > FTS_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0xcd;

	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);


	/*Step 4:erase app and panel paramenter area*/
	DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(FT6X06_UPGRADE_EARSE_DELAY);
	/*erase panel parameter area */
	auc_i2c_write_buf[0] = 0x63;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		printk("\n## packet_number[%d] write ##\n", j + 1);
		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft6x06_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH / 6 + 1);
		//DBG("write bytes:0x%04x\n", (j+1) * FTS_PACKET_LENGTH);
		//delay_qt_ms(FTS_PACKET_LENGTH / 6 + 1);
	}
	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft6x06_i2c_Write(client, packet_buf, temp + 6);
		msleep(20);
	}

	/*send the last six byte */
	for (i = 0; i < 6; i++) {
		temp = 0x6ffa + i;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = 1;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		ft6x06_i2c_Write(client, packet_buf, 7);
		msleep(20);
	}


	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) {
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	return 0;
}


int fts_ctpm_auto_clb(struct i2c_client *client)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	ft6x06_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	ft6x06_write_reg(client, 2, 0x4);
	msleep(300);
	for (i = 0; i < 100; i++) {
		ft6x06_read_reg(client, 0, &uc_temp);
		/*return to normal mode, calibration finish */
		if (0x0 == ((uc_temp & 0x70) >> 4))
			break;
	}

	msleep(200);
	/*calibration OK */
	msleep(300);
	ft6x06_write_reg(client, 0, FTS_FACTORYMODE_VALUE);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ft6x06_write_reg(client, 2, 0x5);	/*store CLB result */
	msleep(300);
	ft6x06_write_reg(client, 0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	/*store CLB result OK */
	return 0;
}

#endif

#if	defined(AUTO_UPGRADE) 
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fw_len ;

	if (g_module_id == DIJIN_FW_MODULE_ID && g_only_id == DIJIN_FW_ONLY_ID)
	{
		fw_len = sizeof(CTPM_FW_97_300400319A);
		pbt_buf = CTPM_FW_97_300400319A;
	}
	else if (g_module_id == JD_FW_MODULE_ID &&  g_only_id == JD_FW_ONLY_ID)
	{
		fw_len = sizeof(CTPM_FW_JD_1189B);
		pbt_buf = CTPM_FW_JD_1189B;
	}
	else
	{
		fw_len = 0;
		pbt_buf = NULL;
	}
			
	/*judge the fw that will be upgraded
	* if illegal, then stop upgrade and return.
	*/
	if (fw_len < 8 || fw_len > 32 * 1024) {
		dev_err(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}

	if ((pbt_buf[fw_len - 8] ^ pbt_buf[fw_len - 6]) == 0xFF
		&& (pbt_buf[fw_len - 7] ^ pbt_buf[fw_len - 5]) == 0xFF
		&& (pbt_buf[fw_len - 3] ^ pbt_buf[fw_len - 4]) == 0xFF) {
		/*FW upgrade */
		/*call the upgrade function */

		i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, fw_len);
		if (i_ret != 0)
			dev_err(&client->dev, "%s:upgrade failed. err.\n",
					__func__);
#ifdef AUTO_CLB
		else
			fts_ctpm_auto_clb(client);	/*start auto CLB */
#endif
	} else {
		printk("\n@@@%s line = %d", __func__, __LINE__);

		dev_err(&client->dev, "%s:FW format error\n", __func__);
		return -EBADFD;
	}

	return i_ret;
}


int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
	u8 uc_host_fm_ver = FT6x06_REG_FW_VER;
	u8 uc_tp_fm_ver;
	int i_ret = 0;
	unsigned char  tp_info[512];
	int len;


	ft6x06_read_reg(client, FT6x06_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	
	printk("\n@@i file firmware 0x%x, line = %d ", uc_host_fm_ver, __LINE__);

	// start decide to upgrade or not depend on the TP ic info
	if ((g_module_id == DIJIN_FW_MODULE_ID && g_only_id == DIJIN_FW_ONLY_ID ||  g_module_id == JD_FW_MODULE_ID &&  g_only_id == JD_FW_ONLY_ID) &&
		( g_fw_ver < uc_host_fm_ver))
	// end
	{
		msleep(100);
		dev_dbg(&client->dev, "[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
				uc_tp_fm_ver, uc_host_fm_ver);
		i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
		if (i_ret == 0)	{
			msleep(300);
			uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		
			dev_dbg(&client->dev, "[FTS] upgrade to new version 0x%x\n",
					uc_host_fm_ver);
		} else {
			pr_err("[FTS] upgrade failed ret=%d.\n", i_ret);
		}
	}
	msleep(100);
	get_tp_ic_info(&g_fw_ver, &g_module_id, &g_only_id);
	if (g_module_id == DIJIN_FW_MODULE_ID)
		len = sprintf(tp_info, "TP IC :%s,TP module :%s,TP I2C adr : 0x%x,TP module id :0x%x,TP only id :0x%x,TP firmware :0x%x,","ft6206","DiJin",client->addr,g_module_id,g_only_id,g_fw_ver);
	else if (g_module_id == JD_FW_MODULE_ID)
		len = sprintf(tp_info, "TP IC :%s,TP module :%s,TP I2C adr : 0x%x,TP module id :0x%x,TP only id :0x%x,TP firmware :0x%x,","ft6206","JunDa",client->addr,g_module_id,g_only_id,g_fw_ver);
	else
		len = sprintf(tp_info, "TP IC :%s,TP module :%s,TP I2C adr : 0x%x,TP module id :0x%x,TP only id :0x%x,TP firmware :0x%x,","ft6206","UN KNOW",client->addr,g_module_id,g_only_id,g_fw_ver);

	fix_tp_proc_info(tp_info, len);
	
	return i_ret;
}
static int auto_upgrade_thread(void * date)
{
	  fts_ctpm_auto_upgrade(i2c_client);      
}
#endif


#if	defined(NODE_UPGRADE) 

static struct class *firmware_class;
static struct device *firmware_cmd_dev;

#ifdef RGK_TP_NODE_UPGRAER_WRITE_PERMISSION_OPEN
#define CTP_AUTHORITY 0777//0664
#else
#define CTP_AUTHORITY 0664
#endif


static ssize_t firmware_node_upgrade_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	char *temp;
	u8 current_fw_version;

	get_tp_ic_info(&g_fw_ver, &g_module_id, &g_only_id);
	printk("\n@@line = %d, fw_ver = 0x%x, module_id = 0x%x, only_id = 0x%x", __LINE__, g_fw_ver, g_module_id, g_only_id);

#if 0	
	return sprintf(buf, "###### TP newest supported firmware: \n \n\
DiJin TP [D206 D207][300400319A][firmware:0x20,module id:0x67,only id:0x01]----1\n\n\
DiJin TP [J201 J221][40-0346A]  [firmware:0x20,module id:0x67,only id:0x01]----2\n\n\
JunDa TP [D206]     [jd-1189B]  [firmware:0x05,module id:0x85,only id:0x02]----3\n\n\
JunDa TP [J201 J221][jd-1190B]  [firmware:0x05,module id:0x85,only id:0x02]----4\n\n\ 
Truly TP [D206]	    [CT1F0467]  [firmware:0x0d,module id:xx,only id:xx]----5\n\n\
########################################\n\
###### current  TP IC firmware :\n\n\
--------------------------------[firmware:0x%x,module id:0x%x,only id:0x%x] \n\n\
try [echo n >> upgrade_node] to upgrade TP firmware !\n\n\
compare the corresponding TP newest supported firmware and current TP IC firmware to decide to upgrade or not !\n\n\n "
											, g_fw_ver, g_module_id, g_only_id);
#else
	return sprintf(buf, "###### TP newest firmware: \n \n\
DiJin TP [7230S]     [firmware:0x1b]----1\n\n\
DiJin TP [7122 8122] [firmware:0x1b]----2\n\n\
JunDa TP [7230S]     [firmware:0x05]----3\n\n\
JunDa TP [7122 8122] [firmware:0x05]----4\n\n\ 
########################################\n\
###### current  TP IC firmware :\n\n\
---------------------[firmware:0x%x] \n\n\
try [echo n >> upgrade_node] to upgrade TP firmware !\n\n\
compare the corresponding TP newest supported firmware and current TP IC firmware to decide to upgrade or not !\n\n\n "
										, g_fw_ver);
#endif
										
}

static ssize_t firmware_node_upgrade_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
	int i_ret;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if(buf == NULL)
	{
		printk("'select the project through clicking  'cat ftp_fw_update'");
		return -1;
	}
	printk("ft5x06_store_fw_update\n");

	if(val == 1){
		i_ret = fts_ctpm_fw_upgrade(i2c_client, CTPM_FW_97_300400319A, sizeof(CTPM_FW_97_300400319A));
		if (i_ret != 0)
			dev_err(&i2c_client->dev, "%s:upgrade failed. err.\n",
					__func__);
	
	}else if(val == 2){
		i_ret = fts_ctpm_fw_upgrade(i2c_client, CTPM_FW_48_10040_0346A, sizeof(CTPM_FW_48_10040_0346A));
		if (i_ret != 0)
			dev_err(&i2c_client->dev, "%s:upgrade failed. err.\n",
					__func__);
	
	}else if(val == 3){
	i_ret = fts_ctpm_fw_upgrade(i2c_client, CTPM_FW_JD_1189B, sizeof(CTPM_FW_JD_1189B));
		if (i_ret != 0)
			dev_err(&i2c_client->dev, "%s:upgrade failed. err.\n",
					__func__);
	}else if(val == 4){
	i_ret = fts_ctpm_fw_upgrade(i2c_client, CTPM_FW_JD_1190B, sizeof(CTPM_FW_JD_1190B));
		if (i_ret != 0)
			dev_err(&i2c_client->dev, "%s:upgrade failed. err.\n",
					__func__);
	}else if(val == 5){
	i_ret = fts_ctpm_fw_upgrade(i2c_client, CTPM_FW_TRULY_CT1F0467, sizeof(CTPM_FW_TRULY_CT1F0467));
		if (i_ret != 0)
			dev_err(&i2c_client->dev, "%s:upgrade failed. err.\n",
					__func__);
	}
	
	
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(10);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(300);

	get_tp_ic_info(&g_fw_ver, &g_module_id, &g_only_id);
	printk("\n@@line = %d, fw_ver = 0x%x, module_id = 0x%x, only_id = 0x%x", __LINE__, g_fw_ver, g_module_id, g_only_id);
	
	printk("ft5x06_store_fw_update Firmware Upgrade finish\n");
	
	return size;
}


static DEVICE_ATTR(upgrade_node, CTP_AUTHORITY, firmware_node_upgrade_show, firmware_node_upgrade_store);
#endif


 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;
	int idx;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;

	unsigned char  tp_info[512];
	int len;
	
reset_proc:   
	i2c_client = client;

	if (tpd_load_status)
		return -1;

   
		//power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 


#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);
#else
	
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(1);
	TPD_DMESG(" ft5306 reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 
	msleep(100);
 
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
		   return -1; 
	}

	//set report rate 80Hz
	report_rate = 0x8; 
	if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	{
	    if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	    {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
	    }
		   
	}

	CTPI2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &CTPI2CDMABuf_pa, GFP_KERNEL);
    	if(!CTPI2CDMABuf_va)
	{
    		printk("[TSP] dma_alloc_coherent error\n");
	}

	get_tp_ic_info(&g_fw_ver, &g_module_id, &g_only_id);
	printk("\n@@line = %d, fw_ver = 0x%x, module_id = 0x%x, only_id = 0x%x", __LINE__, g_fw_ver, g_module_id, g_only_id);

#if	defined(NODE_UPGRADE) 
    firmware_class = class_create(THIS_MODULE, "focaltech-touchscreen-ft6206");
    if(IS_ERR(firmware_class))
    {
        pr_err("Failed to create class(firmware)!\n");
    }
	
    firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");
    if(IS_ERR(firmware_cmd_dev))
    {
        pr_err("Failed to create device(firmware_cmd_dev)!\n");
    }
	
    // node upgrade
    if(device_create_file(firmware_cmd_dev, &dev_attr_upgrade_node) < 0)
    {
        pr_err("Failed to create device file(%s)!\n", dev_attr_upgrade_node.attr.name);
    }
#endif

#if	defined(AUTO_UPGRADE)
	thread = kthread_run(auto_upgrade_thread, NULL, "ft6206_fw_update_i_file");
	 if (IS_ERR(thread))
	{ 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(10);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(300);

#endif

	tpd_load_status = 1;

// if auto_upgrade fix tp info in upgrade thread 
#if !defined(AUTO_UPGRADE)
	if (g_module_id == DIJIN_FW_MODULE_ID)
		len = sprintf(tp_info, "TP IC :%s,TP module :%s,TP I2C adr : 0x%x,TP module id :0x%x,TP only id :0x%x,TP firmware :0x%x,","ft6206","DiJin",client->addr,g_module_id,g_only_id,g_fw_ver);
	else if (g_module_id == JD_FW_MODULE_ID)
		len = sprintf(tp_info, "TP IC :%s,TP module :%s,TP I2C adr : 0x%x,TP module id :0x%x,TP only id :0x%x,TP firmware :0x%x,","ft6206","JunDa",client->addr,g_module_id,g_only_id,g_fw_ver);
	else
		len = sprintf(tp_info, "TP IC :%s,TP module :%s,TP I2C adr : 0x%x,TP module id :0x%x,TP only id :0x%x,TP firmware :0x%x,","ft6206","UN KNOW",client->addr,g_module_id,g_only_id,g_fw_ver);

	fix_tp_proc_info(tp_info, len);
#endif	

#ifdef TPD_HAVE_BUTTON

    for (idx = 0; idx < TPD_KEY_COUNT; idx++)
    {
        input_set_capability(tpd->dev, EV_KEY, tpd_keys_local[idx]);
    }
#endif

	#ifdef VELOCITY_CUSTOM_FT5206
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}

	TPD_DMESG("ft5206 Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
   return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 
 {
   
	 TPD_DEBUG("TPD removed\n");

	 if(CTPI2CDMABuf_va)
	{
		dma_free_coherent(NULL, 4096, CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
		CTPI2CDMABuf_va = NULL;
		CTPI2CDMABuf_pa = 0;
	}
 
   return 0;
 }
 
 
 static int tpd_local_init(void)
 {

 
  TPD_DMESG("Focaltech FT5206 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
 
   if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("ft5206 unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("ft5206 add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }

 static void tpd_resume( struct early_suspend *h )
 {
  //int retval = TPD_OK;
  //char data;
 
   TPD_DMESG("TPD wake up\n");
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");

#else

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(1);  
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	msleep(30);
	tpd_halt = 0;
	/* for resume debug
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);

	}
	*/
	tpd_up(0,0,0);
	input_sync(tpd->dev);
	TPD_DMESG("TPD wake up done\n");
	 //return retval;
 }

 static void tpd_suspend( struct early_suspend *h )
 {
	// int retval = TPD_OK;
	 static char data = 0x3;
 	 tpd_halt = 1;
	 TPD_DMESG("TPD enter sleep\n");
	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	 mutex_lock(&i2c_access);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode

#endif
	mutex_unlock(&i2c_access);
        TPD_DMESG("TPD enter sleep done\n");
	 //return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "FT5206",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) {
	 printk("MediaTek FT5206 touch panel driver init\n");
	   i2c_register_board_info(1, &ft5206_i2c_tpd, 1);
		 if(tpd_driver_add(&tpd_device_driver) < 0)
			 TPD_DMESG("add FT5206 driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek FT5206 touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);

