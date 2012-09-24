
/***********************************************************************
*drivers/input/touchsrceen/novatek_touchdriver.c
*
*Novatek  nt1100x TouchScreen driver.
*
*Copyright(c) 2012 Novatek Ltd.
*
*VERSION              DATA               AUTHOR
*   1.0                   2012-02-24         LiuPeng
************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <mach/gpio_data.h>


#include <linux/proc_fs.h>
#include <linux/novatek_ts.h>

#if 0
#include <linux/i2c.h>
#include <linux/input.h>
#include "novatek_ts.h"
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input/mt.h> 	//for Linux3.0
#include <linux/irq.h>
#include <asm/mach/irq.h>
#include <linux/device.h>
#include <linux/slab.h> //for Kzalloc
#include <mach/board.h>
#endif
/*******************************************************/
// Chip Reset define 
#define  HW_RST      0
#define  SW_RST      1

#if 0
#define KFprintk(x...) printk(x)
#else
#define KFprintk(x...) do{} while(0)
#endif
static struct early_suspend novatek_power; //for ealy suspend
static struct i2c_client *this_client;

#if 0
#ifdef _NOVATEK_CAPACITANCEPANEL_BOOTLOADER_FUNCTION_
enum
{
  RS_OK         = 0,
  RS_INIT_ER    = 8,
  RS_ERAS_ER    = 9,
  RS_FLCS_ER    = 10,
  RS_WD_ER      = 11
} ;
#endif
#endif

/*******************************************************	
Description:
	Read data from the i2c slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer
*********************************************************/
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	int retries = 0;

	msgs[0].flags=client->flags;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];
	//msgs[0].scl_rate = NOVATEK_I2C_SCL;

	msgs[1].flags=client->flags | I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];
	//msgs[1].scl_rate = NOVATEK_I2C_SCL;

	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	return ret;
}


/*******************************************************	
Description:
	write data to the i2c slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;
	//msg.scl_rate = NOVATEK_I2C_SCL;
	
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}
	return ret;
}
/*******************************************************
Description:
	novatek touchscreen initialize function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static int novatek_init_panel(struct novatek_ts_data *ts)
{
	
    	ts->abs_x_max = 1280;
	ts->abs_y_max = 800;
	ts->max_touch_num = MAX_FINGER_NUM;
	//ts->int_trigger_type = IRQF_TRIGGER_FALLING | IRQF_DISABLED;	//edge_falling
	//ts->use_irq = 1;
	return 0;

}
/*******************************************************
Description:
	novatek touchscreen driver Get ChipID function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes.  0x01  --NT11003 ChipID.
*******************************************************/

static int novatek_ts_ChipID(struct i2c_client *client)
{
	int ret;
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
	uint8_t Write_data[] ={0xff, 0xf0, 0x00};
    	uint8_t Read_data[2]= {0};

	ret = i2c_write_bytes(ts->client, Write_data, (sizeof(Write_data)/sizeof(Write_data[0])));
	if (ret <= 0)
    	{
			dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
			return ret;
    	}
	ret = i2c_read_bytes(ts->client, Read_data, (sizeof(Read_data)/sizeof(Read_data[0])));
	if (ret <= 0)
    	{
			dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
			return ret;
    	}
	return (Read_data[1]);
}
/*******************************************************
Description:
	novatek touchscreen work function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static void novatek_ts_work_func(struct work_struct *work)
{	
	int ret=-1;
	int tmp = 0;
	uint8_t  point_data[1+ IIC_BYTENUM*MAX_FINGER_NUM]={0};//[(1-READ_COOR_ADDR)+1+2+5*MAX_FINGER_NUM+1]={ 0 };  //read address(1byte)+key index(1byte)+point mask(2bytes)+5bytes*MAX_FINGER_NUM+coor checksum(1byte)
	uint8_t  check_sum = 0;
	uint16_t  finger_current = 0;
	uint16_t  finger_bit = 0;
	unsigned int  count = 0, point_count = 0;
	unsigned int position = 0;	
	uint8_t track_id[MAX_FINGER_NUM] = {0};
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned char index = 0;
	unsigned char touch_num = 0;
	unsigned char touch_check = 0;
	
	struct novatek_ts_data *ts = container_of(work, struct novatek_ts_data, work);

	point_data[0] = READ_COOR_ADDR;		//read coor address
	ret=i2c_read_bytes(ts->client, point_data,  sizeof(point_data)/sizeof(point_data[0]));
	/////////////////////////////////////////////////////////////////////////////////////////////
	#ifdef HAVE_TOUCH_KEY
	touch_check = point_data[1]>>3;  
	if( touch_check > 20 )
		goto handle_key;
	#endif
	////////////////////////////////////////////////////////////////////////////////////////////
  	touch_num = MAX_FINGER_NUM;
  	for(index = 0; index < MAX_FINGER_NUM; index++)
  	{
  		position = 1 + IIC_BYTENUM*index;
		if(point_data[position]&0x3== 0x03)
  		  touch_num--;	
  	}
  	
	if(touch_num)
	{
		for(index=0; index<MAX_FINGER_NUM; index++)
		{
			position = 1 + IIC_BYTENUM*index;
		  #if IC_DEFINE == NT11003
			track_id[index] = (unsigned int)(point_data[position]>>3)-1;
		  #elif IC_DEFINE == NT11002
		  	track_id[index] = (unsigned int)(point_data[position]>>4)-1;
		  #endif
			input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int)( point_data[position+3]>>4);
			input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f); 
		  #if IC_DEFINE == NT11003		 
			input_w =(unsigned int) (point_data[position+4])+127;
		  #elif IC_DEFINE == NT11002
		  	input_w =(unsigned int) 127;
		  #endif	
			//input_x = input_x *KERNEL_SCREEN_MAX_Y/(TOUCH_MAX_HEIGHT);	
			//input_y = input_y *KERNEL_SCREEN_MAX_X/(TOUCH_MAX_WIDTH);

		  	//#if defined(NOVATEK_USE_RAW_DATA)
			//#else
			//input_x = input_x *KERNEL_SCREEN_MAX_Y/(TP_REGISTER_MAX_X);	
			//input_y =input_y *KERNEL_SCREEN_MAX_X/(TP_REGISTER_MAX_Y);
			//#endif

			#ifdef NOVATEK_REVERSE_X
			      input_x =KERNEL_SCREEN_MAX_Y  - input_x *KERNEL_SCREEN_MAX_Y/(TP_REGISTER_MAX_X);	
			#endif
			#ifdef NOVATEK_REVERSE_Y
			      input_y =KERNEL_SCREEN_MAX_X    -  input_y *KERNEL_SCREEN_MAX_X/(TP_REGISTER_MAX_Y);
			#endif

			if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))continue;
			//printk("input_x = %d,input_y = %d, input_w = %d\n", input_x, input_y, input_w);
			
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);			
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 1);	
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, track_id[index]);
			input_mt_sync(ts->input_dev);
			
		}
	}
	else
	{
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);	
		input_mt_sync(ts->input_dev);
	}

	#ifdef HAVE_TOUCH_KEY
	printk("HAVE KEY DOWN!0x%x\n",point_data[1]);
	for(count = 0; count < MAX_KEY_NUM; count++)
	{
		input_report_key(ts->input_dev, touch_key_array[count], !!(point_data[1]&(0x01<<count)));	
	}	   
	#endif
	input_sync(ts->input_dev);

#if defined(INT_PORT)
	if(ts->int_trigger_type> 1)
	{
		msleep(POLL_TIME);
		goto XFER_ERROR;
	}
#endif
	goto END_WORK_FUNC;
handle_key:
#ifdef HAVE_TOUCH_KEY
	//printk("HAVE KEY DOWN!0x%x\n",point_data[1]);
        value =(unsigned int)(point_data[1]>>3);
	switch (value)
			{
			case MENU:
			    	input_report_key(ts->input_dev, KEY_MENU, point_data[1]&0x01);
				break;
			case HOME:
			    	input_report_key(ts->input_dev, KEY_HOME, point_data[1]&0x01);
				break;			
			case BACK:
			    	input_report_key(ts->input_dev, KEY_BACK, point_data[1]&0x01);
				break;
			case VOLUMEDOWN:
			    	input_report_key(ts->input_dev, KEY_VOLUMEDOWN, point_data[1]&0x01);
				break;
			case VOLUMEUP:
			    	input_report_key(ts->input_dev, KEY_VOLUMEUP, point_data[1]&0x01);
				break;
			default:
				break;
			}	   
#endif
	input_sync(ts->input_dev);

NO_ACTION:	

#ifdef HAVE_TOUCH_KEY
	//printk(KERN_INFO"HAVE KEY DOWN!0x%x\n",point_data[1]);
	for(count = 0; count < MAX_KEY_NUM; count++)
	{
		input_report_key(ts->input_dev, touch_key_array[count], !!(point_data[1]&(0x01<<count)));	
	}
	input_sync(ts->input_dev);	   
#endif
END_WORK_FUNC:
XFER_ERROR:
	if(ts->use_irq)
		enable_irq(ts->client->irq);

}

/*******************************************************
Description:
	Timer interrupt service routine.

Parameter:
	timer:	timer struct pointer.
	
return:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart novatek_ts_timer_func(struct hrtimer *timer)
{
	struct novatek_ts_data *ts = container_of(timer, struct novatek_ts_data, timer);
	queue_work(novatek_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
/*******************************************************
Description:
	External interrupt service routine.

Parameter:
	irq:	interrupt number.
	dev_id: private data pointer.
	
return:
	irq execute status.
*******************************************************/
static irqreturn_t novatek_ts_irq_handler(int irq, void *dev_id)
{
	struct novatek_ts_data *ts = dev_id;
	disable_irq_nosync(ts->client->irq);
	queue_work(novatek_wq, &ts->work);
	//printk(">>>koffuxu enter %s\n",__func__);
	return IRQ_HANDLED;
}
/*******************************************************
Description:
	novatek touchscreen probe function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/


static int novatek_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
	struct novatek_i2c_platform_data *pdata = client->dev.platform_data;

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	
	if (pdata->platform_sleep)                              
		pdata->platform_sleep();
	else if(pdata->power_off){
		printk("[Tomy] Power off TP_3V3\n");
		gpio_out(PAD_GPIOA_23,0);//Power off TP
		pdata->power_off();
	}

	//disable_irq(client->irq);
	return 0;
}


static int novatek_resume(struct i2c_client *client)
{
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
	struct novatek_i2c_platform_data *pdata = client->dev.platform_data;

	if(pdata->power_on){
		pdata->power_on();
		msleep(100);
	}
	
	if(pdata->do_reset)
		pdata->do_reset();

	if (ts->use_irq){
		enable_irq(client->irq);
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	
	return 0;
}

static void novatek_ts_early_suspend(struct early_suspend *h)
{
	//novatek_suspend(this_client,PMSG_SUSPEND);

	struct novatek_ts_data *ts;
	ts = container_of(h, struct novatek_ts_data, early_suspend);
	novatek_suspend(ts->client, PMSG_SUSPEND);
}

static void novatek_ts_late_resume(struct early_suspend *h)
{
	//novatek_resume(this_client);

	struct novatek_ts_data *ts;
	ts = container_of(h, struct novatek_ts_data, early_suspend);
	novatek_resume(ts->client);
	
}

static int	nvctp_CheckIsBootloader(struct i2c_client *client);

static int novatek_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry=0;
	struct novatek_ts_data *ts;
	char *version_info = NULL;
	//zy---char test_data = 1;
	uint8_t test_data[7] = {0x00,};

	struct novatek_i2c_platform_data *pdata = client->dev.platform_data;
	printk("Install touch driver.\n");

	
	if(pdata && pdata->power_off && pdata->power_on){
		pdata->power_off();
		mdelay(2);
		pdata->power_on();
		msleep(500);
	}
	

	if (pdata && pdata->do_reset)
		pdata->do_reset();
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk( "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
  

	client->irq=pdata->irq;
	ts->use_irq = 1;
	//i2c_connect_client_novatek = client;
	printk("novatek   irq:%d\n", client->irq);
	INIT_WORK(&ts->work, novatek_ts_work_func);
	this_client = ts->client = client;
	i2c_set_clientdata(client, ts);
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk("Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	novatek_init_panel(ts);
	printk("***** [NOVATEK]touch   ts->abs_x_max = %d, ts->abs_y_max= %d\n", ts->abs_x_max, ts->abs_y_max );


	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);


	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 0, 0, 0);	
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);

    set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(EV_ABS, ts->input_dev->evbit);
#ifdef HAVE_TOUCH_KEY
	for(retry = 0; retry < MAX_KEY_NUM; retry++)
	{
		input_set_capability(ts->input_dev,EV_KEY,touch_key_array[retry]);	
	}
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = "Goodix-TS";
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = pdata->version;	//screen firmware version
	
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk("Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	//ts->bad_data = 0;
	if (client->irq){
		if (pdata->init_irq)
          	   pdata->init_irq();
          	

		ret  = request_irq(client->irq, novatek_ts_irq_handler, IRQF_DISABLED, client->name, ts);   // IRQF_DISABLED
		if (ret < 0) {
			dev_err(&client->dev, "novatek_ts_probe: request irq failed\n");
			goto exit_irq_request_failed;
		}
		disable_irq(client->irq);
		printk("***** [NOVATEK]Reques EIRQ %d succesd\n", client->irq);
	}
		
	if (!ts->use_irq) 
	{
		printk("***** [NOVATEK] using polling mode\n");
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = novatek_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	
	if(ts->use_irq)
		enable_irq(client->irq);

	/****************************************************************/
	// BootLoader Function...
	#ifdef _NOVATEK_CAPACITANCEPANEL_BOOTLOADER_FUNCTION_
	nvctp_CheckIsBootloader(ts->client);
	#endif
	/****************************************************************/

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = novatek_ts_early_suspend;
	ts->early_suspend.resume = novatek_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	printk("Start %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;

exit_irq_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
err_i2c_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
err_create_proc_entry:
	return ret;
}



/*******************************************************
Description:
	novatek touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int novatek_ts_remove(struct i2c_client *client)
{
	struct novatek_ts_data *ts = i2c_get_clientdata(client);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
	#endif

	if (ts && ts->use_irq) {
		free_irq(client->irq, ts);
	}
	else if(ts){
			hrtimer_cancel(&ts->timer);
		}
	dev_notice(&client->dev, "The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	//unregister_early_suspend(&novatek_power);
	kfree(ts);
	return 0;
}

#ifdef _NOVATEK_CAPACITANCEPANEL_BOOTLOADER_FUNCTION_

#define  FW_DATASIZE      (1024*32)
#define  FLASHSECTORSIZE  (FW_DATASIZE/128)
#define  FW_CHECKSUM_ADDR     (FW_DATASIZE - 8)
static unsigned char nvctp_BinaryFile[]=
{
	//#include "COBY_V322_1045_20120532_CSM_7D68.fw"
	#include "COBY_V322_1045_20120601_CSM_8984.fw"
};

/*******************************************************	
Description:
	Read data from the flash slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer
*********************************************************/

static int BootLoader_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	int retries = 0;

	msgs[0].flags=client->flags;
	msgs[0].addr=0x7F;//client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];
	//msgs[0].scl_rate = NOVATEK_I2C_SCL;
	
	msgs[1].flags=client->flags | I2C_M_RD;;
	msgs[1].addr=0x7F;//client->addr;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];
	//msgs[1].scl_rate = NOVATEK_I2C_SCL;
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	return ret;
}

/*******************************************************	
Description:
	write data to the flash slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int BootLoader_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=client->flags;
	msg.addr=0x7F;//client->addr;
	msg.len=len;
	msg.buf=data;		
	//msg.scl_rate = NOVATEK_I2C_SCL;
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}
	return ret;
}

/*******************************************************	
Description:
	Driver Bootloader  function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/


void nvctp_DelayMs(unsigned long wtime)
{
	msleep(wtime);
}
/*************************************************************************************
*Description:
*			Flash Mass Erase Command(7FH --> 30H --> 00H)
*
*Return:
*			Executive Outcomes: 0 -- succeed
*************************************************************************************/
static int novatek_FlashEraseMass(struct i2c_client *client)
{
	uint8_t i,status;
	uint8_t Buffer[4]={0};
	int ret= RS_ERAS_ER;
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
	
    Buffer[0] = 0x00;
    Buffer[1] = 0x33;
	for(i = 5; i > 0; i--)
	{
		Buffer[2] = 0x00;
		
		ret = BootLoader_write_bytes(ts->client, Buffer, 3);
		if (ret <= 0)
    	{
			dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
    	}
		nvctp_DelayMs(25);

		/*Read status*/
   		ret = BootLoader_read_bytes(ts->client, Buffer, 2);
		if (ret <= 0)
    	{
			dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
    	}
   		if(Buffer[1] == 0xAA)
   		{
	   		ret = RS_OK;
			break;
   		}
		
		nvctp_DelayMs(1);
	}
	return ret;


}
/*************************************************************************************
*
*
*
*
*
*************************************************************************************/
static int novatek_FlashEraseSector(struct i2c_client *client, uint16_t wAddress)
{
	uint8_t i,status;
	uint8_t Buffer[4]={0};
	int ret= RS_ERAS_ER;
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
	
    Buffer[0] = 0x00;
    Buffer[1] = 0x30;
	for(i = 5; i > 0; i--)
	{
		Buffer[2] = (uint8_t)(wAddress >> 8);
		Buffer[3] = (uint8_t)(wAddress)&0x00FF;

		ret = BootLoader_write_bytes(ts->client, Buffer, 4);
		if (ret <= 0)
    	{
			dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
    	}
		nvctp_DelayMs(10);

		/*Read status*/
   		ret = BootLoader_write_bytes(ts->client, Buffer, 2);
		if (ret <= 0)
    	{
			dev_err(&(ts->client->dev),"I2C transfer error. Number:%d\n ", ret);
    	}
   		if(Buffer[1] == 0xAA)
   		{
	   		ret = RS_OK;
			break;
   		}
		
		nvctp_DelayMs(2);
	}
	return ret;


}

/*************************************************************************************
*
*
*
*
*
*************************************************************************************/
static int novatek_Initbootloader(uint8_t bType, struct i2c_client *client)
{
	u8 ret = RS_OK;
    u8 status;
	u8 iic_buffer[13];
	u8 write_cmd1[3] = {0xff,0xf1,0x91};
  	u8 write_cmd2[2] = {0x00,0x01};
	
	struct novatek_ts_data *ts = i2c_get_clientdata(client);

	
	//ts->client->Addr = 0x7F;
	iic_buffer[0] = 0x00;
	if(bType == HW_RST)
	 {
		iic_buffer[1] = 0x00;
		BootLoader_write_bytes(ts->client, (uint8_t *)iic_buffer, 2);
		nvctp_DelayMs(2);
		//nvctp_GobalRset(1,2);				
	 }
	else if(bType == SW_RST)
	{
	        iic_buffer[1] = 0xA5;
		BootLoader_write_bytes(ts->client, iic_buffer, 2);
                nvctp_DelayMs(10);
		iic_buffer[1] = 0x00;
		BootLoader_write_bytes(ts->client, (uint8_t *)iic_buffer, 2);
		nvctp_DelayMs(2);
	}

  /*Read status*/
   BootLoader_read_bytes(ts->client,iic_buffer,2);
   	printk("iic_buffer[1] %d\n",iic_buffer[1]);
   if(iic_buffer[1] != 0xAA)
   	{
	   ret = RS_INIT_ER;
   	}
   	printk("ret %d\n",ret);
	
  	//ts->client->Addr = 0x01;
    //i2c_write_bytes(ts->client, write_cmd1, 3); 
	//i2c_write_bytes(ts->client, write_cmd2, 2);
	//ts->client->Addr = 0x7F;
   return ret;
}

/*******************************************************************************************




********************************************************************************************/
unsigned char nvctp_ReadFinalChecksum(unsigned long flash_addr, unsigned long final_checksum,struct i2c_client *client)
{
	unsigned char iic_data_buffer[14]={0};
	unsigned long wValue,DynChecksum;
  	unsigned char ret = RS_OK;
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
	u8 write_cmd1[3] = {0xff,0x8E,0x0E};
  	//u8 write_cmd2[2] = {0x00,0x01};
	
	i2c_write_bytes(ts->client, write_cmd1, 3);
	iic_data_buffer[0] = 0;
	i2c_read_bytes(ts->client,iic_data_buffer,3);
	DynChecksum = (unsigned long)iic_data_buffer[1]<<8|iic_data_buffer[2];
	
	/*inital bootloader 进入bootloader 模式，该模式下i2c读写地址会改变*/
	ret = novatek_Initbootloader(SW_RST,ts->client);
	if(ret != RS_OK)
	 {
		return ret;
	 }
	iic_data_buffer[0]=0x00;		
	iic_data_buffer[1] = 0x99;
	iic_data_buffer[2] = (unsigned char)(flash_addr >> 8);
	iic_data_buffer[3] = flash_addr&0xFF;
	iic_data_buffer[4] = 8;
	BootLoader_write_bytes(ts->client,iic_data_buffer,5);
	nvctp_DelayMs(2);
	BootLoader_read_bytes(ts->client,iic_data_buffer,14);
	wValue = (unsigned long)iic_data_buffer[12]<<8 |iic_data_buffer[13];

	printk("%s,  old version checksum = %x \n",__func__,wValue);

	//TODO
	//if((wValue != final_checksum)||(DynChecksum != final_checksum))
	if((wValue != final_checksum))
	{
		ret = RS_FLCS_ER;
	}

	return ret;
	
}


/*************************************************************************************
*
*
*
*
*
*************************************************************************************/
static int nvctp_WriteDatatoFlash( unsigned char *fw_BinaryData, unsigned long BinaryDataLen,struct i2c_client *client)
{
	uint8_t ret = RS_OK;
  	uint8_t iic_data_buffer[14];
	uint8_t j,k;
	uint8_t iic_buffer[16] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t Checksum[16] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint16_t flash_addr;
	uint16_t sector;
	struct novatek_ts_data *ts = i2c_get_clientdata(client);

	printk("--%s--\n",__func__);
    WFStart:
				iic_data_buffer[0]=0x00;
				sector = 0;
				flash_addr = 0;

				for(sector = 0; sector < FLASHSECTORSIZE; sector++)
				{
					printk("data writing ....... %d \n",sector);
   WFRepeat:	    	 
     	 			flash_addr = 128*sector;  
     	 			for (j = 0; j < 16; j++)
     	 			{
        			/* Write Data to flash*/
                    	iic_data_buffer[1] = 0x55;
	            		iic_data_buffer[2] = (unsigned char)(flash_addr >> 8);
                    	iic_data_buffer[3] = (unsigned char)flash_addr;
		    			iic_data_buffer[4] = 8;
		    
		    			iic_data_buffer[6] = fw_BinaryData[flash_addr + 0];
		    			iic_data_buffer[7] = fw_BinaryData[flash_addr + 1];
		    			iic_data_buffer[8] = fw_BinaryData[flash_addr + 2];
		    			iic_data_buffer[9] = fw_BinaryData[flash_addr + 3];
		    			iic_data_buffer[10]= fw_BinaryData[flash_addr + 4];
		    			iic_data_buffer[11]= fw_BinaryData[flash_addr + 5];
		    			iic_data_buffer[12]= fw_BinaryData[flash_addr + 6];
		    			iic_data_buffer[13]= fw_BinaryData[flash_addr + 7];
        
		    			Checksum[j] = ~(iic_data_buffer[2]+iic_data_buffer[3]+iic_data_buffer[4]+iic_data_buffer[6]+\
		    	        iic_data_buffer[7]+iic_data_buffer[8]+iic_data_buffer[9]+\
		    	  		iic_data_buffer[10]+iic_data_buffer[11]+iic_data_buffer[12]+iic_data_buffer[13]) + 1;
		    			iic_data_buffer[5] = Checksum[j];
	    
		    			BootLoader_write_bytes(ts->client, iic_data_buffer, 14);

						flash_addr += 8;	
     	 			}
					nvctp_DelayMs(20);
					/*Setup force genrate Check sum */
    				flash_addr = 128*sector;  
    				for (j = 0; j < 16; j++)
    				{
                		iic_data_buffer[0] = 0x00;
						iic_data_buffer[1] = 0x99;
						iic_data_buffer[2] = (unsigned char)(flash_addr >> 8);
      	        		iic_data_buffer[3] = (unsigned char)flash_addr & 0xFF;
		        		iic_data_buffer[4] = 8;
						BootLoader_write_bytes(ts->client, iic_data_buffer,5);
						nvctp_DelayMs(2);
						BootLoader_read_bytes(ts->client, iic_data_buffer, 14);
	                  
						if(iic_data_buffer[5] != Checksum[j] )
						{
							printk("checksum error iic_data_buffer[5] = %x,  Checksum[%d] = %x\n", iic_data_buffer[5],j,Checksum[j]);
			  				ret = RS_WD_ER;

                        	//novatek_FlashEraseMass(ts->client);
                        	//goto WFStart;
                        	novatek_FlashEraseSector(ts->client,(unsigned int)(sector*128));
                        	goto WFRepeat;
							
						}
						flash_addr += 8;	
	                    nvctp_DelayMs(2);
	  				}


   }
   return ret;		        
}

/*************************************************************************************
*
*
*
*
*
*************************************************************************************/
static int novatek_Bootloader(unsigned char *nvctp_binaryfile , unsigned long nvctp_binaryfilelength,struct i2c_client *client)
{
	uint8_t i;
	uint8_t ret = RS_OK;
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
        
	printk("--%s--\n",__func__);
	
	/*Erase Sector*/
	ret = novatek_FlashEraseMass(ts->client);

	if(ret != RS_OK)
	{
		printk("nvctp_EraseSector error \n");
		return ret;
	}
	/*Write binary data to flash*/
	ret = nvctp_WriteDatatoFlash(nvctp_binaryfile,nvctp_binaryfilelength,ts->client);
	if(ret != RS_OK)
	{
		return ret;
	}
	
    printk("--%s-- ret = %d\n",__func__,ret);
	return ret;
}

/*************************************************************************************
*
*
*
*
*
*************************************************************************************/
static int	nvctp_CheckIsBootloader(struct i2c_client *client)
{
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
	//struct novatek_platform_data *pdata = client->dev.platform_data;
	struct novatek_i2c_platform_data *pdata = client->dev.platform_data;
	uint16_t FW_CHECKSUM ;
	uint8_t iic_buffer[3];
	printk("nvctp_CheckIsBootloader\n");
		   
	FW_CHECKSUM = (unsigned int)(nvctp_BinaryFile[FW_DATASIZE-2]<< 8 |nvctp_BinaryFile[FW_DATASIZE-1]);
	printk("%s,  new version checksum = %x \n",__func__,FW_CHECKSUM );
	if(nvctp_ReadFinalChecksum(FW_CHECKSUM_ADDR,FW_CHECKSUM,ts->client))
	{
		printk("nvctp_CheckIsBootloader1\n");
		novatek_Bootloader(nvctp_BinaryFile,sizeof(nvctp_BinaryFile),ts->client);
			   
	}
	else
	{
		printk("--%s--, tp version is no change\n",__func__);
		iic_buffer[0] = 0x00;
		iic_buffer[1] = 0xA5;
		BootLoader_write_bytes(ts->client, iic_buffer, 2);										
	}
	
	/*****Hareware reset command******/
	//pdata->platform_wakeup();
	if(pdata->do_reset)
		pdata->do_reset();
	
	
}


/*************************************************************************/

#endif

static const struct i2c_device_id novatek_ts_id[] = {
	{ NOVATEK_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver novatek_ts_driver = {
	.probe		= novatek_ts_probe,
	.remove		= novatek_ts_remove,
//	.resume		= novatek_ts_resume,
//	.suspend		= novatek_ts_suspend,
	.id_table	= novatek_ts_id,
	.driver = {
		.name	= NOVATEK_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

/*******************************************************	
Description:
	Driver Install function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit novatek_ts_init(void)
{
	int ret;
	
	//novatek_wq = create_workqueue("novatek_wq");		//create a work queue and worker thread
	novatek_wq = create_singlethread_workqueue("novatek_wq");
	if (!novatek_wq) {
		printk(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;
		
	}
	ret=i2c_add_driver(&novatek_ts_driver);
	return ret; 
}

/*******************************************************	
Description:
	Driver uninstall function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit novatek_ts_exit(void)
{
	printk(KERN_ALERT "Touchscreen driver of guitar exited.\n");
	i2c_del_driver(&novatek_ts_driver);
	if (novatek_wq)
		destroy_workqueue(novatek_wq);		//release our work queue
}
late_initcall(novatek_ts_init);
module_exit(novatek_ts_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");


