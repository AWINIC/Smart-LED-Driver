/*
 * AWINIC AW2013 LED driver
 *
 * Copyright (C) 2014 AWINIC Technology Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/mt_gpio.h>
#include <mach/mt_gpt.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/fs.h>

#ifndef __HW_CTL_DRVIER__
#define __HW_CTL_DRVIER__
#define HWCTL_DBG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/string.h>
#include <mach/irqs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#define HW_DEVICE_MINOR    (0)
#define HW_DEVICE_COUNT    (1)
#define HW_DEVICE_NAME    "aw2013"

#define HW_CTL_IO_TEST                  _IO('H', 0x00)
#define HW_CTL_IO_ENB_KBD               _IO('H', 0x01)
#define HW_CTL_IO_DIS_KBD               _IO('H', 0x02)
#define HW_CTL_IO_EN_MSG_NTF            _IO('H', 0x03)
#define HW_CTL_IO_DIS_MSG_NTF           _IO('H', 0x04)
#define HW_CTL_IO_EN_CALL_NTF           _IO('H', 0x05)
#define HW_CTL_IO_DIS_CALL_NTF          _IO('H', 0x06)
#define HW_CTL_IO_EN_BAT_NTF            _IO('H', 0x07)
#define HW_CTL_IO_DIS_BAT_NTF           _IO('H', 0x08)
#define HW_CTL_IO_CHARGING_EN_NTF       _IO('H', 0x09)
#define HW_CTL_IO_CHARGING_DIS_NTF      _IO('H', 0x0A)
#define HW_CTL_IO_LEFT_HAND_NTF         _IO('H', 0x0B)
#define HW_CTL_IO_RIGHT_HAND_NTF        _IO('H', 0x0C)
#define HW_CTL_IO_CHARGING_FULL_EN_NTF  _IO('H', 0x0D)
#define HW_CTL_IO_CHARGING_FULL_DIS_NTF _IO('H', 0x0E)
#define HW_CTL_IO_FORCE_REFRESH_LEDS    _IO('H', 0xF0)

#define GPIO_InitIO(dir,pin)     mt_set_gpio_dir(pin,dir)
#define GPIO_WriteIO(level,pin)  mt_set_gpio_out(pin,level)
#define GPIO_ReadIO(pin)         mt_get_gpio_out(pin)

#define AW2013_I2C_MAX_LOOP  50   
#define I2C_delay            2    //i2c speed <= 400k

//LED breath
#define Imax        0x02   //LED Imax,0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x02   //LED rise time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x01   //LED max light time light 0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time   0x02   //LED fall time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time    0x01   //LED off time ,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Delay_time  0x00   //LED Delay time ,0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00   //LED breath period number,0x00=forever,0x01=1,0x02=2.....0x0f=15

typedef enum{
    DEV_NONE_STAGE = 0x0,
    DEV_ALLOC_REGION,
    DEV_ALLOC_CDEV,    
    DEV_ADD_CDEV,
    DEV_ALLOC_CLASS,
    DEV_INIT_ALL
}HWDEV_INIT_STAGE;

typedef struct __HW_DEVICE{
    dev_t hwctl_dev_no;
    struct cdev * hw_cdev;
    struct class *hw_class;
    struct device *hw_device;
    char init_stage;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif                /* CONFIG_HAS_EARLYSUSPEND */    
}HW_DEVICE , * p_HW_DEVICE;
#endif

/*
static ssize_t aw2013_store_led(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t aw2013_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw2013_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);

static DEVICE_ATTR(led, S_IRUGO | S_IWUSR, NULL, aw2013_store_led);
static DEVICE_ATTR(reg, S_IRUGO | S_IWUGO,aw2013_get_reg,  aw2013_set_reg);
*/

struct i2c_client *aw2013_i2c_client;

void AW2013_delay_1us(U16 wTime);
BOOL AW2013_i2c_write_reg(unsigned char reg,unsigned char data);

/*******************************AW2013 LED Program***********************************/
void aw2013_breath_all(int led0,int led1,int led2)  //led on=0x01   ledoff=0x00
{  

    //write_reg(0x00, 0x55);                  // Reset 
    AW2013_i2c_write_reg(0x01, 0x01);         // enable LED         

    AW2013_i2c_write_reg(0x31, Imax|0x70);    //config mode, IMAX = 5mA    
    AW2013_i2c_write_reg(0x32, Imax|0x70);    //config mode, IMAX = 5mA    
    AW2013_i2c_write_reg(0x33, Imax|0x70);    //config mode, IMAX = 5mA    

    AW2013_i2c_write_reg(0x34, 0xff);         // LED0 level,
    AW2013_i2c_write_reg(0x35, 0xff);         // LED1 level,
    AW2013_i2c_write_reg(0x36, 0xff);         // LED2 level,
                                            
    AW2013_i2c_write_reg(0x37, Rise_time<<4 | Hold_time);   //led0                  
    AW2013_i2c_write_reg(0x38, Fall_time<<4 | Off_time);    //led0 
    AW2013_i2c_write_reg(0x39, Delay_time<<4| Period_Num);  //led0 

    AW2013_i2c_write_reg(0x3a, Rise_time<<4 | Hold_time);   //led1                        
    AW2013_i2c_write_reg(0x3b, Fall_time<<4 | Off_time);    //led1 
    AW2013_i2c_write_reg(0x3c, Delay_time<<4| Period_Num);  //led1  

    AW2013_i2c_write_reg(0x3d, Rise_time<<4 | Hold_time);   //led2             
    AW2013_i2c_write_reg(0x3e, Fall_time<<4 | Off_time);    //led2 
    AW2013_i2c_write_reg(0x3f, Delay_time<<4| Period_Num);  //led2

    AW2013_i2c_write_reg(0x30, led2<<2|led1<<1|led0);       //led on=0x01 ledoff=0x00    
    AW2013_delay_1us(8);  //Delay >5us
}


void led_off_aw2013()
{
    unsigned char reg_data;
    unsigned int    reg_buffer[8];

    AW2013_i2c_write_reg(0x30, 0);  //led off    
    AW2013_i2c_write_reg(0x01,0);

    }

void AW2013_delay_1us(U16 wTime)   
{
    udelay(wTime);
}

static BOOL AW2013_i2c_write_reg_org(unsigned char reg,unsigned char data)
{
    BOOL ack=0;
    unsigned char ret;
    unsigned char wrbuf[2];

    wrbuf[0] = reg;
    wrbuf[1] = data;

    ret = i2c_master_send(aw2013_i2c_client, wrbuf, 2);
    if (ret != 2) {
        dev_err(&aw2013_i2c_client->dev,
        "%s: i2c_master_recv() failed, ret=%d\n",
        __func__, ret);
        ack = 1;
    }

    return ack;
}

BOOL AW2013_i2c_write_reg(unsigned char reg,unsigned char data)
{
    BOOL ack=0;
    unsigned char i;
    for (i=0; i<AW2013_I2C_MAX_LOOP; i++)
    {
        ack = AW2013_i2c_write_reg_org(reg,data);
        if (ack == 0) // ack success
            break;
        }
    return ack;
}

unsigned char AW2013_i2c_read_reg(unsigned char regaddr) 
{
    unsigned char rdbuf[1], wrbuf[1], ret, i;

    wrbuf[0] = regaddr;

    for (i=0; i<AW2013_I2C_MAX_LOOP; i++) 
    {
        ret = i2c_master_send(aw2013_i2c_client, wrbuf, 1);
        if (ret == 1)
            break;
    }
    
    ret = i2c_master_recv(aw2013_i2c_client, rdbuf, 1);
    
    if (ret != 1)
    {
        printk("AW2013_i2c_read_reg failed  %s \r\n", __func__);
        dev_err(&aw2013_i2c_client->dev,"%s: i2c_master_recv() failed, ret=%d\n",
        __func__, ret);
    }
    
    return rdbuf[0];
        
}


#if 1
extern struct i2c_adapter * get_mt_i2c_adaptor(int);

int breathlight_master_send(u16 addr, char * buf ,int count)
{
    unsigned char ret;
    
    ret = i2c_master_send(aw2013_i2c_client, buf, count);
    
    if (ret != count) 
    {

        printk("breathlight_master_send failed  %s \r\n", __func__);
    
        dev_err(&aw2013_i2c_client->dev,"%s: i2c_master_recv() failed, ret=%d\n",
            __func__, ret);
    }
    return ret;
}

void led_flash_aw2013_test( unsigned int id )
{
    char buf[2];
    
    AW2013_i2c_read_reg(0x55);
    
    printk("hwctl led_flash_aw2013_test \n");    
    buf[0]=0x01;
    buf[1]=0x01;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x31;
    buf[1]=0x71;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x34;
    buf[1]=0xff;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x37;
    buf[1]=0x53;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x38;
    buf[1]=0x55;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x39;
    buf[1]=0x00;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x30;
    buf[1]=0x01;
    breathlight_master_send(0x45,buf,2);    
}

void led_off_aw2013_test(void)
{
    char buf[2];
    buf[0]=0x30;
    buf[1]=0x00;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x01;
    buf[1]=0x00;
    breathlight_master_send(0x45,buf,2);
}

void led_flash_aw2013( unsigned int id )
{
    char buf[2];

    
    buf[0]=0x01;
    buf[1]=0x01;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x31+id;
    buf[1]=0x71;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x34+id;
    buf[1]=0xff;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x37+id*3;
    buf[1]=0x53;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x38+id*3;
    buf[1]=0x55;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x39+id*3;
    buf[1]=0x00;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x30;
    buf[1]=1<<id;
    breathlight_master_send(0x45,buf,2);
}

void led_flash_aw2013_power_low(void)    //red led
{
    unsigned int id =1;    //red led
    char buf[2];

    buf[0]=0x00;
    buf[1]=0x54;    //reset led module
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x01;
    buf[1]=0x01;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x31+id;
    buf[1]=0x71;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x34+id;
    buf[1]=0xff;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x37+id*3;
    buf[1]=0x00;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x38+id*3;
    buf[1]=0x06;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x39+id*3;
    buf[1]=0x00;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x30;
    buf[1]=1<<id;
    breathlight_master_send(0x45,buf,2);
}

void led_flash_aw2013_charging_full(void)
{
    unsigned int id =2;    //green led

    char buf[2];
    buf[0]=0x00;
    buf[1]=0x54;    //reset led module
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x01;
    buf[1]=0x01;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x31+id;
    buf[1]=0x02;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x34+id;
    buf[1]=0xff;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x30;
    buf[1]=1<<id;
    breathlight_master_send(0x45,buf,2);
}

void led_flash_aw2013_charging(void)
{
    unsigned int id =1;    //red led

    char buf[2];
    buf[0]=0x00;
    buf[1]=0x54;    //reset led module
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x01;
    buf[1]=0x01;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x31+id;
    buf[1]=0x02;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x34+id;
    buf[1]=0xff;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x30;
    buf[1]=1<<id;
    breathlight_master_send(0x45,buf,2);
}

void led_flash_aw2013_unanswer_message_incall(void)    //blue led
{
    unsigned int id =0;    /blue led
    char buf[2];

    buf[0]=0x00;
    buf[1]=0x54;    //reset led module
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x01;
    buf[1]=0x01;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x31+id;
    buf[1]=0x73;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x34+id;
    buf[1]=0xff;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x37+id*3;
    buf[1]=0x32;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x38+id*3;
    buf[1]=0x35;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x39+id*3;
    buf[1]=0x00;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x30;
    buf[1]=1<<id;
    breathlight_master_send(0x45,buf,2);
}

void led_flash_aw2013_power_on(void)
{
    char buf[2];
    unsigned int id =0;    //0 blue led ,1 red,2 green,

    buf[0]=0x01;
    buf[1]=0x01;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x31+id;
    buf[1]=0x73;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x34+id;
    buf[1]=0xff;  //0xc8;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x37+id*3;
    buf[1]=0x34;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x38+id*3;
    buf[1]=0x35;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x39+id*3;
    buf[1]=0x03;
    breathlight_master_send(0x45,buf,2);

    buf[0]=0x30;
    buf[1]=1<<id;
    breathlight_master_send(0x45,buf,2);
}

static unsigned char LED_ON_FLAG = 0x0;

#define TST_BIT(flag,bit)    (flag & (0x1 << bit))
#define CLR_BIT(flag,bit)    (flag &= (~(0x1 << bit)))
#define SET_BIT(flag,bit)    (flag |= (0x1 << bit))

#define MSG_FLAG_BIT           (0x0)
#define CALL_FLAG_BIT          (0x1)
#define BAT_FLAG_BIT           (0x2)
#define CHARGING_FLAG_BIT      (0x3)
#define CHARGING_FULL_FLAG_BIT (0x4)

void Flush_led_data(void)
{
    //first if it's charging situation, we skip the other actions
    if(TST_BIT(LED_ON_FLAG,CHARGING_FLAG_BIT))
    {
        led_flash_aw2013_charging();
        return;
    }
    if(TST_BIT(LED_ON_FLAG,CHARGING_FULL_FLAG_BIT))
    {
        led_flash_aw2013_charging_full();
        return;
    }
    //second the bat infor is the priority
    if(TST_BIT(LED_ON_FLAG,BAT_FLAG_BIT))
    {
        led_flash_aw2013_power_low();
        return;
    }
    //then msg and call
    if(TST_BIT(LED_ON_FLAG,CALL_FLAG_BIT))
    {
        led_flash_aw2013_unanswer_message_incall();
        return;
    }
    if(TST_BIT(LED_ON_FLAG,MSG_FLAG_BIT))
    {
        led_flash_aw2013_unanswer_message_incall();
        return;
    }    
}

void Suspend_led(void)
{
    //first if it's charging situation, we skip the other actions
    led_off_aw2013();
}
#endif

#ifdef HWCTL_DBG
#define DBG_PRINT(x...)    printk(KERN_ERR x)
#else
#define DBG_PRINT(x...)
#endif
static HW_DEVICE g_hwctl_device={.init_stage=DEV_NONE_STAGE,};

static long hwctl_unlock_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    DBG_PRINT("hwctl_ioctl cmd=%d",cmd);

    switch(cmd)
    {
        case HW_CTL_IO_TEST:
            DBG_PRINT("this is test and this module has been tested");
            break;
        case HW_CTL_IO_ENB_KBD:            
            DBG_PRINT("[HWCTL]Enable keyboard");
            break;
        case HW_CTL_IO_DIS_KBD:
            DBG_PRINT("[HWCTL]Disable keyboard");
            break;
        case HW_CTL_IO_EN_MSG_NTF:
            SET_BIT(LED_ON_FLAG,MSG_FLAG_BIT);
        //    Flush_led_data();//
            break;
        case HW_CTL_IO_DIS_MSG_NTF:            
            CLR_BIT(LED_ON_FLAG,MSG_FLAG_BIT);
            break;
        case HW_CTL_IO_EN_CALL_NTF:            
            SET_BIT(LED_ON_FLAG,CALL_FLAG_BIT);
        //    Flush_led_data();//
            break;
        case HW_CTL_IO_DIS_CALL_NTF:            
            CLR_BIT(LED_ON_FLAG,CALL_FLAG_BIT);
            break;
        case HW_CTL_IO_EN_BAT_NTF:            
            SET_BIT(LED_ON_FLAG,BAT_FLAG_BIT);
        //    Flush_led_data();//
            break;
        case HW_CTL_IO_DIS_BAT_NTF:            
            CLR_BIT(LED_ON_FLAG,BAT_FLAG_BIT);
            break;
        case HW_CTL_IO_CHARGING_EN_NTF:    
            SET_BIT(LED_ON_FLAG,CHARGING_FLAG_BIT);
        //    Flush_led_data();///z
            break;
        case HW_CTL_IO_CHARGING_FULL_EN_NTF:    
            SET_BIT(LED_ON_FLAG,CHARGING_FULL_FLAG_BIT);
        //    Flush_led_data();///z
            break;    
        case HW_CTL_IO_CHARGING_FULL_DIS_NTF:    
            CLR_BIT(LED_ON_FLAG,CHARGING_FULL_FLAG_BIT);
            break;
        case HW_CTL_IO_CHARGING_DIS_NTF:    
            CLR_BIT(LED_ON_FLAG,CHARGING_FLAG_BIT);
            break;    
        case HW_CTL_IO_LEFT_HAND_NTF:
            //tpd_toggle_hand_using(0);
            //SET_BIT(LED_ON_FLAG,CHARGING_FLAG_BIT);
            break;
        case HW_CTL_IO_RIGHT_HAND_NTF:
            //tpd_toggle_hand_using(1);
            //CLR_BIT(LED_ON_FLAG,CHARGING_FLAG_BIT);
            break;    

        case HW_CTL_IO_FORCE_REFRESH_LEDS:
            //Suspend_led();
            //Flush_led_data();
            break;
        default:
            break;
    }
    return 0;
}

static int hwctl_open(struct inode *inode, struct file *file)
{ 
    return 0;
}

static int hwctl_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t hwctl_read(struct file * fp, char __user * to, size_t read_size, loff_t * pos)
{
#define VERSION_NO    "Hardware direct control driver"

    ssize_t copy_size = strlen(VERSION_NO);

    DBG_PRINT("hwctl_read ops");

    return copy_to_user(to,VERSION_NO,copy_size);
}

static struct file_operations hwctl_fops = {
    .owner        = THIS_MODULE,
    .unlocked_ioctl    = hwctl_unlock_ioctl,
    .open        = hwctl_open,
    .release    = hwctl_release,
    .read        = hwctl_read,
};

#if 0//defined(CONFIG_HAS_EARLYSUSPEND)
static void aw2013_early_suspend(struct early_suspend *h)
{
    Flush_led_data();
}

static void aw2013_late_resume(struct early_suspend *h)
{
    Suspend_led();
}
#endif


void hwctl_shut_charging_leds_and_dojobs()
{
    Suspend_led();
    CLR_BIT(LED_ON_FLAG,CHARGING_FLAG_BIT);
    CLR_BIT(LED_ON_FLAG,CHARGING_FULL_FLAG_BIT);

    Flush_led_data();
}


static int aw2013_create_sysfs(struct i2c_client *client)
{
    int err;
    struct device *dev = &(client->dev);

    TS_DBG("%s", __func__);
    
    err = device_create_file(dev, &dev_attr_led);
    err = device_create_file(dev, &dev_attr_reg);
    return err;
}


static int __devinit aw2013_i2c_probe(struct i2c_client *client,
                      const struct i2c_device_id *id)
{
    printk("aw2013_i2c_probe:OK");


    aw2013_i2c_client = client;



    DBG_PRINT("aw2013_i2c_probe addr %x   " , aw2013_i2c_client->addr   );

    led_flash_aw2013_test(0);

    
    led_flash_aw2013_power_low();
    led_flash_aw2013_charging_full();
    led_flash_aw2013_charging();
    led_flash_aw2013_unanswer_message_incall();
    aw2013_create_sysfs(client);

    return 0;
}

static int __devexit aw2013_i2c_remove(struct i2c_client *client)
{
    aw2013_i2c_client = NULL;
    return 0;
}

static const struct i2c_device_id AW2013_i2c_id[] = {
    { "AW2013", 0 },
    { }
};



//MODULE_DEVICE_TABLE(i2c, AW2013_i2c_id);

static struct i2c_board_info __initdata aw2013_i2c_hw={ I2C_BOARD_INFO("AW2013", 0x45)};

static struct i2c_driver aw2013_i2c_driver = {
        .driver = 
       {
            // .owner  = THIS_MODULE,
            .name   = "AW2013",
        },

        .probe          = aw2013_i2c_probe,
        .remove         =  __devexit_p(aw2013_i2c_remove),
        .id_table       = AW2013_i2c_id,
};

static int __init hwctl_driver_init(void) 
{
    int ret;


//    #err
    i2c_register_board_info(2, &aw2013_i2c_hw, 1);
         
    ret = i2c_add_driver(&aw2013_i2c_driver);

    
    printk("hwctl_driver_init:start \n");
    
    if(0!=alloc_chrdev_region(&g_hwctl_device.hwctl_dev_no,HW_DEVICE_MINOR,HW_DEVICE_COUNT,HW_DEVICE_NAME))
    {
        printk("hwctl_driver_alloc chrdev region:fail \n");
        goto init_error;
        goto init_error;
        goto init_error;
        goto init_error;
    }    

    
    printk("hwctl_driver_alloc chrdev region:OK\n");
    
    g_hwctl_device.init_stage = DEV_ALLOC_REGION;
    
    printk("hwctl_driver_alloc chrdev region:OK1\n");
    g_hwctl_device.hw_cdev = cdev_alloc();
    printk("hwctl_driver_alloc chrdev region:OK2\n");
        g_hwctl_device.hw_cdev->owner = THIS_MODULE;
        printk("hwctl_driver_alloc chrdev region:OK3\n");
        g_hwctl_device.hw_cdev->ops = &hwctl_fops; 
        printk("hwctl_driver_alloc chrdev region:OK4\n");
        ret = cdev_add(g_hwctl_device.hw_cdev, g_hwctl_device.hwctl_dev_no, 1);//
    printk("hwctl cdev_add_ret_is %d \n",ret);
    if(ret)
    {
        printk("hwctl_driver add cdev error\n");        
        goto init_error;
    }
    g_hwctl_device.init_stage = DEV_ADD_CDEV;
    g_hwctl_device.hw_class = class_create(THIS_MODULE, HW_DEVICE_NAME);
    
    g_hwctl_device.init_stage = DEV_ALLOC_CLASS;

    // if we want auto creat device node, we must call this
    g_hwctl_device.hw_device = device_create(g_hwctl_device.hw_class, NULL, g_hwctl_device.hwctl_dev_no, NULL, HW_DEVICE_NAME); 
    g_hwctl_device.init_stage = DEV_INIT_ALL;
    #if 0
    led_flash_aw2013_test(0);///z
    led_flash_aw2013_power_low();
    led_flash_aw2013_charging_full();
    led_flash_aw2013_charging();
    led_flash_aw2013_unanswer_message_incall();
    #endif

#if 0//def CONFIG_HAS_EARLYSUSPEND
    g_hwctl_device.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 20;
    g_hwctl_device.early_suspend.suspend = aw2013_early_suspend;
    g_hwctl_device.early_suspend.resume = aw2013_late_resume;
    register_early_suspend(&g_hwctl_device.early_suspend);
#endif
    return 0;

init_error:
    if(g_hwctl_device.init_stage == DEV_ALLOC_REGION)
        unregister_chrdev_region(g_hwctl_device.hwctl_dev_no,1);
    return (-1);
}



/* should never be called */
static void __exit hwctl_driver_exit(void) 
{
    if(g_hwctl_device.init_stage == DEV_INIT_ALL)
    {
        device_del(g_hwctl_device.hw_device);
        class_destroy(g_hwctl_device.hw_class);
        cdev_del(g_hwctl_device.hw_cdev);
        unregister_chrdev_region(g_hwctl_device.hwctl_dev_no,1);
    }
    i2c_del_driver(&aw2013_i2c_driver);
}

module_init(hwctl_driver_init);
module_exit(hwctl_driver_exit);

MODULE_AUTHOR("AWINIC");
MODULE_DESCRIPTION("AW2013 Linux HW direct control driver");
MODULE_LICENSE("GPL v2");
