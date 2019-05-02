/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
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



//#include <cust_eint.h>

#include "tpd_custom_it7259.h"
#ifndef TPD_NO_GPIO 
//#include "cust_gpio_usage.h"
#endif
#include <linux/wakelock.h>
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_boot.h>

#include <mach/wd_api.h>

//#include "mt_boot_common.h"
#include <linux/regulator/consumer.h>
#include <linux/of_irq.h>

#include <linux/device.h>

//#define __IT7259_DEBUG_A158_JFSS__ //lshun use tp for A152
#define s32 int
//#define I2C_SUPPORT_RS_DMA 
//extern kal_bool upmu_chr_det(upmu_chr_list_enum chr); // remove
//taikoto 20171108 add
extern struct input_dev *kpd_input_dev;
//taikoto 20171108 add end
unsigned int touch_irq = 0;
#define ABS(x)				((x<0)?-x:x)
#define TPD_OK				0
#define MAX_BUFFER_SIZE		144
#define CTP_NAME			"IT7259"
#define IOC_MAGIC			'd'
#define IOCTL_SET 			_IOW(IOC_MAGIC, 1, struct ioctl_cmd168)
#define IOCTL_GET 			_IOR(IOC_MAGIC, 2, struct ioctl_cmd168)
#define HAS_8_BYTES_LIMIT 

//#define MAX_BUFFER_SIZE        1028//144 
#define MAX_CMD_BUFFER_SIZE        144
#define MAX_FINGER_NUMBER    10
#define MAX_PRESSURE        15
#define DEVICE_NAME            "IT7259"
#define DEVICE_VENDOR        0
#define DEVICE_PRODUCT        0
#define DEVICE_VERSION        0
#define IT7259_X_RESOLUTION    672
#define IT7259_Y_RESOLUTION    672
#define SCREEN_X_RESOLUTION    1440
#define SCREEN_Y_RESOLUTION    768
#define VERSION_ABOVE_ANDROID_20
#define MAX_COUNT            3000
#define COMMAND_SUCCESS        0x0000
#define COMMAND_ERROR         0x0200
#define ERROR_QUERY_TIME_OUT    0x0800

#define QUERY_SUCCESS        0x00
#define QUERY_BUSY        0x01
#define QUERY_ERROR        0x02

#define MAX_FILE_SIZE        0x10000
#define IOC_MAGIC        'd'
#define IOCTL_SET         _IOW(IOC_MAGIC, 1, struct ioctl_cmd168)
#define IOCTL_GET         _IOR(IOC_MAGIC, 2, struct ioctl_cmd168)
#define IOCTL_AP_GET         0xF0
#define IOCTL_AP_SET         0xF1
#define IOCTL_AP_CMD         0xF2
#define IOCTL_AP_WRITE_FW      0xF3
#define IOCTL_AP_COMPARE_FLASH 0xF4
#define IOCTL_AP_SEND_FILE      0xF5

//add by FHH for I2C DMA mode start
#define MAX_I2C_NDMA_LEN 8
#define MAX_I2C_DMA_LEN 240  //should less than 256
static DEFINE_MUTEX(it7259_i2c_mutex);
//static int tpd_i2c_dma_write(struct i2c_client *client, u32 len, u8 const *data);
static int tpd_i2c_dma_read(struct i2c_client *client, u32 len, u8 const *data);
//add by FHH for I2C DMA mode end

extern struct tpd_device *tpd;

static int tpd_flag = 0;
static int tpd_halt=0;
#ifdef I2C_SUPPORT_RS_DMA
static u8 *I2CDMABuf_va = NULL;
static u32 I2CDMABuf_pa = NULL;
#endif
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
#include <linux/fs.h>  
#include <linux/file.h>
#include <linux/types.h>  
#define MY_FILE "/data/spectrum_log.txt"

enum wk_wdt_type {
    WK_WDT_LOC_TYPE,
    WK_WDT_EXT_TYPE,
    WK_WDT_LOC_TYPE_NOLOCK,
    WK_WDT_EXT_TYPE_NOLOCK,
};
extern void mtk_wdt_restart(enum wk_wdt_type type);
 /*
static void mtk_kick_wdt(void)
{
    mtk_wdt_restart(WK_WDT_LOC_TYPE_NOLOCK);
    mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);
}
*/
typedef unsigned char		BOOL;
typedef unsigned char		BYTE;

typedef unsigned char		uint8;
typedef unsigned short		WORD;
typedef unsigned long int	uint32;
typedef unsigned int		UINT;

typedef signed char			int8;
typedef signed short		int16;
typedef signed long int		int32;

//#define MAX_BUFFER_SIZE 256

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

unsigned char APCmdBuffer[MAX_BUFFER_SIZE]; // byte 0: length  byte 1~n: command  byte n+1 : read length

//static bool waitCommandDone(void);
static void tpd_print_version(void);
static void tpd_eint_interrupt_handler(void);
static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
static int tpd_i2c_write(struct i2c_client *client, uint8_t *buf, int len);
static int tpd_i2c_read(struct i2c_client *client, uint8_t *buf, int len , uint8_t addr);
static int gfnIT7259_SpectrumModeTest(unsigned char ucStage);
static ssize_t IT7259_SpectrumTs_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t IT7259_SpectrumTs_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static int i2cDMAReadFromIT7259(struct i2c_client *client, uint16_t addr, uint8_t *buf, int len );
//static int i2cDMAWriteToIT7259(struct i2c_client *client, uint16_t reg, uint8_t *buf, int len);
static int i2cDirectReadFromIT7259(struct i2c_client *client, uint16_t addr, uint8_t *buf, int len );
static int i2cDirectWriteToIT7259(struct i2c_client *client, uint16_t reg, uint8_t *buf, int len);

extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern int mtk_wdt_enable(enum wk_wdt_en en);

//lshun modify 20130615
//extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
//extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
//extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

static struct i2c_client *i2c_client = NULL;

static const struct i2c_device_id tpd_i2c_id[] ={{CTP_NAME,0},{}}; // {{"mtk-tpd",0},{}}; 

static unsigned short force[] = {0, 0x8C, I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};

//static struct i2c_board_info __initdata it7259_i2c_tpd={ I2C_BOARD_INFO(CTP_NAME, (0x8c>>1))};
static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
 static struct i2c_driver tpd_i2c_driver = {
	.driver = {
	.name = CTP_NAME,
#ifdef CONFIG_OF	 	
	.of_match_table = tpd_of_match,
#endif	
	.owner = THIS_MODULE,
	},
    .probe			= tpd_i2c_probe,   
    .remove = tpd_i2c_remove,
    .detect			= tpd_i2c_detect,                           
    .driver.name 	= CTP_NAME, //"mtk-tpd", 
    .id_table 		= tpd_i2c_id,                             
 //   .address_list 	= forces,                        
}; 
struct ite7259_data {
	rwlock_t lock;
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

struct ioctl_cmd168 {
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

static DEVICE_ATTR(spectrumts, S_IWUSR | S_IRUGO, IT7259_SpectrumTs_show, IT7259_SpectrumTs_store);
static struct attribute *ite_attrs[] = {
    &dev_attr_spectrumts.attr,	
    NULL
};

static const struct attribute_group ite_attr_group = {
    .attrs = ite_attrs,
};

//add by FHH for I2C DMA mode start
/*
static int tpd_i2c_dma_write(struct i2c_client *client, u32 len, u8 const *data)
{
    u8* dmaVa = NULL;
    dma_addr_t dmaPa = 0;
    u32 written = 0 ;
    u32 dmaLen = MAX_I2C_DMA_LEN;
    u32 ext_flag;
    int ret;

    if ((len == 0) || (data == NULL) ) {
        return -EIO;
    }

    mutex_lock(&it7259_i2c_mutex);

    if (len < MAX_I2C_NDMA_LEN) {
        client->ext_flag &= (~I2C_DMA_FLAG);
        ret = i2c_master_send(client, data, len);
        if (len != ret) {
            mutex_unlock(&it7259_i2c_mutex);
            pr_err("%s NDMA failed: ret=%d\n", __FUNCTION__, ret);
            return -EIO;
        }
    }
    else {
        client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        dmaVa = dma_alloc_coherent(&(client->dev), MAX_I2C_DMA_LEN, &dmaPa, GFP_KERNEL); 
        ext_flag = client->ext_flag;
        client->ext_flag |= (I2C_ENEXT_FLAG | I2C_DMA_FLAG);

        while (written < len) {
            if (written + MAX_I2C_DMA_LEN > len) {
                dmaLen = len - written;
            }
            memcpy(dmaVa, &data[written], dmaLen);
            ret = i2c_master_send(client, (u8*)dmaPa, dmaLen);
            if (dmaLen != ret) {
                pr_err("%s DMA failed: ret=%d\n", __FUNCTION__, ret);
            }
            written += dmaLen;
        }

        dma_free_coherent(&(client->dev), MAX_I2C_DMA_LEN, dmaVa, dmaPa);
        client->ext_flag = ext_flag;
    }

    mutex_unlock(&it7259_i2c_mutex);

    return 0;
}
*/
static int tpd_i2c_dma_read(struct i2c_client *client, u32 len, u8 const *data)
{
    u8* dmaVa = NULL;
    dma_addr_t dmaPa = 0;
    u32 read = 0 ;
    u32 dmaLen = MAX_I2C_DMA_LEN;
    u32 ext_flag;
    int ret;

    if ((!data) || (len == 0)) {
        pr_err("%s data is null\n", __FUNCTION__);
        return -EINVAL;
    }

    mutex_lock(&it7259_i2c_mutex);

     if (len <= MAX_I2C_NDMA_LEN) {
        client->ext_flag &= (~I2C_DMA_FLAG);
        ret = i2c_master_recv(client, (char*)data, len);
        if (len != ret) {
            mutex_unlock(&it7259_i2c_mutex);
             pr_err("%s NDMA failed: ret=%d, length=%d\n", __FUNCTION__, ret, len);
             return -EIO;
         }
     } else {
        client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        dmaVa = dma_alloc_coherent(&(client->dev), MAX_I2C_DMA_LEN, &dmaPa, GFP_KERNEL); 
        ext_flag = client->ext_flag;
        client->ext_flag |= (I2C_ENEXT_FLAG | I2C_DMA_FLAG);

        while (read < len) {
            if (read + MAX_I2C_DMA_LEN > len) {
                dmaLen = len - read;
            }
            ret = i2c_master_recv(client, (u8*)dmaPa, dmaLen);
            if (dmaLen != ret) {
                pr_err("%s DMA failed: ret=%d\n", __FUNCTION__, ret);
            }
            memcpy((void*)&data[read], dmaVa, dmaLen);
            read += dmaLen;
        }

        dma_free_coherent(&(client->dev), MAX_I2C_DMA_LEN, dmaVa, dmaPa);
        client->ext_flag = ext_flag;
    }

    mutex_unlock(&it7259_i2c_mutex);

    return 0;
}
//add by FHH for I2C DMA mode end

static long ite7259_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	//struct ite7259_data *dev = filp->private_data;
	int retval = 0;
	int i;
	//unsigned char ucQuery;
	unsigned char buffer[MAX_BUFFER_SIZE];
	struct ioctl_cmd168 data;
	unsigned char datalen;
	unsigned char ent[] = {0x60, 0x00, 0x49, 0x54, 0x37, 0x32};
	unsigned char ext[] = {0x60, 0x80, 0x49, 0x54, 0x37, 0x32};

	pr_info("=ite7259_ioctl=\n");
	memset(&data, 0, sizeof(struct ioctl_cmd168));

	switch (cmd) {
	case IOCTL_SET:
		pr_info("=IOCTL_SET=\n");
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}
		buffer[0] = (unsigned char) data.bufferIndex;
		pr_info("%.2X ", buffer[0]);
		for (i = 1; i < data.length + 1; i++) {
			buffer[i] = (unsigned char) data.buffer[i - 1];
			pr_info("%.2X ", buffer[i]);
		}
		if (!memcmp(&(buffer[1]), ent, sizeof(ent))) {

			pr_info("Disabling IRQ.\n");
			//disable_irq(gl_ts->client->irq);
			tpd_halt = 1;
			//mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		}

		if (!memcmp(&(buffer[1]), ext, sizeof(ext))) {

			pr_info("Enabling IRQ.\n");

			//enable_irq(gl_ts->client->irq);
			tpd_halt = 0;
			//	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

		}

		datalen = (unsigned char) (data.length + 1);
		printk("IOCTL_SET: datalen=%d.\n",datalen);
		retval = tpd_i2c_write(i2c_client, &buffer[0], datalen);
		pr_info("SET:retval=%x\n", retval);
		retval = 0;
		break;

	case IOCTL_GET:
		pr_info("=IOCTL_GET=\n");
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}

		retval = tpd_i2c_read(i2c_client, (unsigned char*) buffer, (unsigned char) data.length, (unsigned char) data.bufferIndex);

		pr_info("GET:retval=%x\n", retval);
		retval = 0;
		for (i = 0; i < data.length; i++) {
			data.buffer[i] = (unsigned short) buffer[i];
		}
		pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, buffer[0], buffer[1], buffer[2], buffer[3]);

		if ( copy_to_user((int __user *)arg, &data, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}
		break;

	default:
		retval = -ENOTTY;
		break;
	}

	done:
	pr_info("DONE! retval=%d\n", retval);
	return (retval);
}
/**************************
The output log file format should be like below

Channel = 12
Tx = Off
Minima Frequency = 40
Maxima Frequency = 110
Number of Times = 10

Count = 1
4099
4130
4136
4086
.
.
Count = 2
4099
4130
4136
4086
.
.


**************************/

int i2cReadFromIT7259(struct i2c_client *client, unsigned char writeBuffer[], unsigned short writeLength,
		unsigned char readDataBuffer[], unsigned short readDataLength) 
{
#if 1
    int ret;

    if ((readDataLength > 7) ||(writeLength>7))  {
        return -EINVAL;
    }
    client->addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    memcpy(readDataBuffer, writeBuffer, writeLength);
    ret = i2c_master_send(client, &readDataBuffer[0], (readDataLength << 8 | writeLength));
#else
	int ret;
	struct i2c_msg msgs[2] = { { .addr = client->addr, .flags = 0,
			.len = writeLength, .buf = writeBuffer }, { .addr = client->addr, .flags =
			I2C_M_RD, .len = readDataLength, .buf = readDataBuffer } };
	printk("taikoto====in i2cReadFromIT7259 function===\n");
	memset(readDataBuffer, 0xFF, readDataLength);
	ret = i2c_transfer(client->adapter, msgs, 2);
#endif
	return ret;
}

int i2cWriteToIT7259(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char const dataBuffer[], unsigned short dataLength)
{
#if 1
    int ret;
    char buffer4Write[8];

    if (dataLength > 7) {
        return -EINVAL;
    }
    buffer4Write[0] = bufferIndex;
    memcpy(&(buffer4Write[1]), dataBuffer, dataLength);
    client->addr = client->addr & I2C_MASK_FLAG;
    ret = i2c_master_send(client, buffer4Write, dataLength+1);
    return ret;
#else
	unsigned char buffer4Write[256];
	struct i2c_msg msgs[1] = { { .addr = client->addr, .flags = 0, .len =
			dataLength + 1, .buf = buffer4Write } };
	printk("taikoto====in i2cWriteToIT7259 function===\n");

	buffer4Write[0] = bufferIndex;
	memcpy(&(buffer4Write[1]), dataBuffer, dataLength);
	return i2c_transfer(client->adapter, msgs, 1);
#endif
}


static int i2cAdvancedReadFromIT7259(struct i2c_client *client, unsigned char writeBuffer[], unsigned short writeLength,
		unsigned char readDataBuffer[], unsigned short readDataLength)  {
#if 1
    int ret;

    if ((readDataLength > 7) ||(writeLength>7))  {
        return -EINVAL;
    }
    client->addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    memcpy(readDataBuffer, writeBuffer, writeLength);
    ret = i2c_master_send(client, &readDataBuffer[0], (readDataLength << 8 | writeLength));
#else
	int ret;
	struct i2c_msg msgs[2] = { { .addr = client->addr, .flags = 0,
			.len = writeLength, .buf = writeBuffer }, { .addr = client->addr, .flags =
			I2C_M_RD, .len = readDataLength, .buf = readDataBuffer } };
	printk("taikoto====in read function===\r\n");
	memset(readDataBuffer, 0xFF, readDataLength);
#if 0//def HAS_8_unsigned charS_LIMIT
	//printk("====8 byte===\r\n");
	if(readDataLength >= 8)
	{
		unsigned char ucCurReadIndex = 0;
		unsigned char pucBuffer[128];
		unsigned char pucBufferIndex =0;
		
		memset(pucBuffer, 0xFF, 128);
		if(bufferIndex == 0xA0)
 			pucBufferIndex = 0x05;
		else if(bufferIndex == 0xE0)
			pucBufferIndex = 0x07;
		else if(bufferIndex == 0xC0)
			pucBufferIndex = 0x06;
		else
			pucBufferIndex = 0x05;
		if(IT7259ReadCommandBufferStart(pucBufferIndex))
		{
			unsigned short ucTotalLength = dataLength;
			while(ucCurReadIndex< ucTotalLength)
			{
			 	unsigned char ucReadSize = 8;
				if((ucCurReadIndex + ucReadSize)>=ucTotalLength)
				{
					ret = IT7259ReadCommandBufferContinue((ucTotalLength-ucCurReadIndex),&pucBuffer[ucCurReadIndex],true);
					//pr_info("=read final %x--point[%x][%x][%x][%x][%x][%x][%x][%x]=\n",(ucTotalLength-ucCurReadIndex),
					//pucBuffer[ucCurReadIndex+0],pucBuffer[ucCurReadIndex+1],pucBuffer[ucCurReadIndex+2],
					//pucBuffer[ucCurReadIndex+3],pucBuffer[ucCurReadIndex+4],pucBuffer[ucCurReadIndex+5],pucBuffer[ucCurReadIndex+6],pucBuffer[ucCurReadIndex+7]);
					ucCurReadIndex = ucTotalLength;
					if(ret !=true)
						return false;										
					break;
				}
				else
				{
					ret = IT7259ReadCommandBufferContinue(ucReadSize,&pucBuffer[ucCurReadIndex],false);
					ucCurReadIndex+=ucReadSize;
					if(ret !=true)
						return false;
					//pr_info("=read continue--point[%x][%x][%x][%x][%x][%x][%x][%x]=\n",
					//pucBuffer[0],pucBuffer[1],pucBuffer[2],
					//pucBuffer[3],pucBuffer[4],pucBuffer[5],pucBuffer[6],pucBuffer[7]);
				}
			}
			memcpy(readDataBuffer,pucBuffer,readDataLength);
		}
		else
		{
			return false;
		}						
	}
	else
	{
		//printk("====8 byte 2===\r\n");
	  	ret = i2c_transfer(client->adapter, msgs, 2);
	}	
	
#else
	//printk("====in i2cAdvancedReadFromIT7259 ===");
	ret = i2c_transfer(client->adapter, msgs, 2);
#endif
#endif
    return ret;
}
/*
static int i2cAdvancedWriteToIT7259(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char const dataBuffer[], unsigned short dataLength) 
{
	//printk("====in write function===");
#if 0//def HAS_8_unsigned charS_LIMIT
	int ret;
	unsigned char buffer4Write[256];
	if(dataLength>=7)
	{
		unsigned char ucCurWriteIndex = 0;
		unsigned char pucBufferIndex =0;
		
		memcpy(&(buffer4Write[0]), dataBuffer, dataLength);
		if(bufferIndex == 0x20)
 			pucBufferIndex = 0x01;
		else if(bufferIndex == 0x40)
			pucBufferIndex = 0x02;
		else
			pucBufferIndex = 0x01;
		if(IT7259WriteCommandBufferStart(pucBufferIndex))
		{
			unsigned short ucTotalLength = dataLength;
			while(ucCurWriteIndex< ucTotalLength)
			{
			 	unsigned char ucWriteSize = 5;
				if((ucCurWriteIndex + ucWriteSize)>=ucTotalLength)
				{
					ret = IT7259WriteCommandBufferContinue((ucTotalLength-ucCurWriteIndex),&buffer4Write[ucCurWriteIndex],true);
					//pr_info("=write final %x--point[%x][%x][%x][%x][%x][%x][%x][%x]=\n",(ucTotalLength-ucCurWriteIndex),
					//buffer4Write[ucCurWriteIndex+0],buffer4Write[ucCurWriteIndex+1],buffer4Write[ucCurWriteIndex+2],
					//buffer4Write[ucCurWriteIndex+3],buffer4Write[ucCurWriteIndex+4],buffer4Write[ucCurWriteIndex+5],buffer4Write[ucCurWriteIndex+6],buffer4Write[ucCurWriteIndex+7]);
					ucCurWriteIndex = ucTotalLength;
					if(ret !=true)
					{
						//printk("==============WriteCommandBufferContinue final FAIL ========================");
						return false;
					}										
					break;
				}
				else
				{
					ret = IT7259WriteCommandBufferContinue(ucWriteSize,&buffer4Write[ucCurWriteIndex],false);
					ucCurWriteIndex+=ucWriteSize;
					//pr_info("=write continue--point[%x][%x][%x][%x][%x][%x][%x][%x]=\n",
					//buffer4Write[ucCurWriteIndex],buffer4Write[ucCurWriteIndex+1],buffer4Write[ucCurWriteIndex+2],
					//2buffer4Write[ucCurWriteIndex+3],buffer4Write[ucCurWriteIndex+4],buffer4Write[ucCurWriteIndex+5],buffer4Write[ucCurWriteIndex+6],buffer4Write[ucCurWriteIndex+7]);
					if(ret !=true)
					{
						//printk("==============WriteCommandBufferContinue continue FAIL ========================");
						return false;
					}

				}
			}
		}
		else
		{
			return false;
		}
	}
	else
	{
		struct i2c_msg msgs[1] = { { .addr = client->addr, .flags = 0, .len =
			dataLength + 1, .buf = buffer4Write } };
		buffer4Write[0] = bufferIndex;
		memcpy(&(buffer4Write[1]), dataBuffer, dataLength);
		return i2c_transfer(client->adapter, msgs, 1);
	}

#else

	unsigned char buffer4Write[256];
	struct i2c_msg msgs[1] = { { .addr = client->addr, .flags = 0, .len =
			dataLength + 1, .buf = buffer4Write } };
	buffer4Write[0] = bufferIndex;
	memcpy(&(buffer4Write[1]), dataBuffer, dataLength);
	return i2c_transfer(client->adapter, msgs, 1);

#endif
}
*/
bool IT7259_SendCommand( unsigned char writeBuffer[], unsigned short writeLength,
		unsigned char readDataBuffer[], unsigned short readDataLength) {
	//int i;
	//int tmp;
	unsigned char ucQuery = 0xff;
	unsigned char writeQueryBuffer[1];
	//unsigned char buffer[128];
	//unsigned char ucQueryBuffer[1];
	//struct IT7259_ts_data *ts = gl_ts;
	int ret;
	int count = 0;

	printk("==============IT7259_SendCommand ========================\n");

	 writeQueryBuffer[0] = 0x80;
	// Identify Cap Sensor
	do {
		
		i2cReadFromIT7259(i2c_client, writeQueryBuffer, 1, &ucQuery, 1);
		printk("it7259 : count is %d....",count);
		count ++;
	} while ((ucQuery & 0x01) && (count < MAX_COUNT));
	
	printk("it7259 : ==============step 1: %d ========================\n",count);
	//buffer[0] = 0x00;
	if(count >= MAX_COUNT)
		return false;

	ret = i2cWriteToIT7259(i2c_client, 0x20, writeBuffer, writeLength); 
	if (ret < 0) {
		return false;
	}

	printk("it7259 : ==============step 2:    ========================\n");
	count = 0 ; 
	ucQuery = 0xff;
	do {
		i2cReadFromIT7259(i2c_client, writeQueryBuffer, 1, &ucQuery, 1);
		//i2cReadFromIT7259(ts->client, 0x80, &ucQuery, 1);
		printk("%d....",count);
		count ++;
	} while ((ucQuery & 0x01) && (count < MAX_COUNT));

	if(count >= MAX_COUNT)
		return false;
	printk("it7259 : ==============step 3: %d ========================\n",count);
	//memset(&buffer, 0, sizeof(buffer));

	//unsigned char writeBuffer[], unsigned short writeLength, unsigned char readDataBuffer[], unsigned short readDataLength
	writeQueryBuffer[0] = 0xA0;
	ret = i2cAdvancedReadFromIT7259(i2c_client, writeQueryBuffer, 1 , readDataBuffer, readDataLength);
	//ret = i2cReadFromIT7259(i2c_client, writeQueryBuffer, 1, readDataBuffer, readDataLength);
	if (ret < 0) {
		return false;
	}
	
	pr_err("it7259 : ====Response--[%x][%x][%x][%x][%x][%x]=\n", readDataBuffer[0], readDataBuffer[1],
			readDataBuffer[2], readDataBuffer[3], readDataBuffer[4], readDataBuffer[5]);
	

	return true;
}

int gfnIT7259_SpectrumModeTest(unsigned char ucStage)
{
		//printk("%s():\n", __func__);

        //unsigned char ucQuery = QUERY_BUSY;
        unsigned char pucCommandBuffer[1024];
        unsigned int unWriteLength = 4;
        unsigned int unReadLength = 2;
		unsigned char ucTxMode = 0; //0: Tx Off, 1: Tx On
        unsigned short wTmp=COMMAND_ERROR;
		//unsigned int unCounter = 10;
		unsigned int unMsgLen=0;
		//struct IT7259_ts_data *ts = gl_ts;
		unsigned char msg [128];
		unsigned int i ,j= 0;
		struct file *filp = NULL;
	char receivedByte[9], *p;
	int wAddreses;

		mm_segment_t old_fs; 		
		if(filp == NULL)		
		filp = filp_open(MY_FILE, O_RDWR | O_CREAT, 0666);		
		if (IS_ERR(filp)) {		
			printk("it7259 : Error occured while opening file %s, exiting...\n", MY_FILE);
			set_fs(old_fs);		
			if(filp != NULL)		
				filp_close(filp, NULL);			
			return COMMAND_ERROR;
		}
		
		old_fs = get_fs();		
		set_fs(KERNEL_DS);
		
		//if (!IS_ERR(filp)) {		
		filp->f_op->llseek(filp,0,0); // ?\8A\E6?標移?\B0\E6?案\E9?始\EF??\8D寫??	
		unMsgLen = sprintf(msg,"Channel = %d\r\n", ucStage);
		filp->f_op->write(filp, msg, unMsgLen, &filp->f_pos);
		unMsgLen = sprintf(msg,"Tx = Off\r\n");
		filp->f_op->write(filp, msg, unMsgLen, &filp->f_pos);
		unMsgLen = sprintf(msg,"Minima Frequency = 40\r\n");
		filp->f_op->write(filp, msg, unMsgLen, &filp->f_pos);
		unMsgLen = sprintf(msg,"Maxima Frequency = 110\r\n");
		filp->f_op->write(filp, msg, unMsgLen, &filp->f_pos);
		unMsgLen = sprintf(msg,"Number of Times = 10\r\n");
		filp->f_op->write(filp, msg, unMsgLen, &filp->f_pos);

		
		//unMsgLen = sprintf(msg,"Count = 1\r\n");
		//filp->f_op->write(filp, msg, unMsgLen, &filp->f_pos);

			//filp->f_op->write(filp, (unsigned char *)pucCommandBuffer, 1024, &filp->f_pos);		
		//}

	
	for(i = 1;i<11;i++){
			unsigned int unRetry=0;
			unMsgLen = sprintf(msg,"\r\nCount = %d\r\n", i);
			filp->f_op->write(filp, msg, unMsgLen, &filp->f_pos);

	        pucCommandBuffer[0] = 0x1A;
	        pucCommandBuffer[1] = 0x01;
	        pucCommandBuffer[2] = ucTxMode;
			pucCommandBuffer[3] = ucStage;
	        //send command
	        wTmp = IT7259_SendCommand(pucCommandBuffer,unWriteLength, pucCommandBuffer , unReadLength);  //?^¶?data©??bregister?ìžm
	        if (wTmp != true) {
                    pr_err("msg: %s, wTmp = %d\r\n", msg, wTmp);
	            return COMMAND_ERROR;
                }

	        //read memory
			
			while(( (pucCommandBuffer[0] == 0x00)||(pucCommandBuffer[0] == 0xFF)||(pucCommandBuffer[1] == 0xFF))&&unRetry<10){
		        pucCommandBuffer[0] = 0x1A;
		        pucCommandBuffer[1] = 0x01;
		        pucCommandBuffer[2] = ucTxMode;
				pucCommandBuffer[3] = ucStage;				
				wTmp = IT7259_SendCommand(pucCommandBuffer,unWriteLength, pucCommandBuffer , unReadLength);
				pr_info("i= %d, Retry = %d\r\n",i,unRetry);
				unRetry++;
				if (wTmp != true)
	            	return COMMAND_ERROR;
			}			
			unRetry = 0;			
			
	        wAddreses = (int)((pucCommandBuffer[1]<<8) | pucCommandBuffer[0]);
			//pr_info("wAddreses : %X\r\n",wAddreses);

	        //?¹register?ª512­?word?^š?¡C?\AC\C2?\82\AC@©w­ndirect access¡A¥?read memoryªºcommand¥hšú?^?\AC]¥i¥H¡A?ý?\ACUcommandªº?\ACèŠ¡€@?ž¥u¯??ª128bytes¡C¡A
	        //pr_info("SpectrumMode Read Buffer Start");
	        wTmp = i2cDMAReadFromIT7259(i2c_client, wAddreses,pucCommandBuffer, 1024);        
			//pr_info("SpectrumMode pucCommandBuffer\r\n");
			//for(j=0;j<1024;j++)
			//	pr_info("%X, ",pucCommandBuffer[j]);

			for(j=0;j<1024;j = j+2){
				long intNumber;
				sprintf(receivedByte, "0x%02X%02X", pucCommandBuffer[j+1], pucCommandBuffer[j]);
			//	pr_info("SpectrumMode pucCommandBuffer0: %X\r\n",pucCommandBuffer[j]);
			//	pr_info("SpectrumMode pucCommandBuffer1: %X\r\n",pucCommandBuffer[j+1]);
			//	pr_info("receivedByte: %s\r\n",receivedByte);
				intNumber = simple_strtol(receivedByte, &p, 16);
				unMsgLen = sprintf(msg,"%ld\r\n", intNumber);
				filp->f_op->write(filp, msg, unMsgLen, &filp->f_pos);
			}	
			//filp->f_op->write(filp, (unsigned char *)pucCommandBuffer, 1024, &filp->f_pos);
	}
	
		set_fs(old_fs);		
		if(filp != NULL)		
			filp_close(filp, NULL);

		//reset FW
        pucCommandBuffer[0] = 0x0C;  
		i2cWriteToIT7259(i2c_client, 0x20, pucCommandBuffer, 1);
		mdelay(150);


        if (wTmp != true)
            return COMMAND_ERROR;

		
		if (wTmp != COMMAND_SUCCESS)
			return COMMAND_ERROR;
        else
			return COMMAND_SUCCESS;


		
}


int ite7259_open(struct inode *inode, struct file *filp) {
	int i;
	struct ite7259_data *dev;

	pr_info("taikoto : =ite7259_open=\n");
	dev = kmalloc(sizeof(struct ite7259_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	/* initialize members */
	rwlock_init(&dev->lock);
	for (i = 0; i < MAX_BUFFER_SIZE; i++) {
		dev->buffer[i] = 0xFF;
	}

	filp->private_data = dev;

	return 0; /* success */
}

int ite7259_close(struct inode *inode, struct file *filp) {
	struct ite7259_data *dev = filp->private_data;

	pr_info("taikoto : =ite7259_close=\n");
	if (dev) {
		kfree(dev);
	}

	return 0; /* success */
}

struct file_operations ite7259_fops = {
	.owner		= THIS_MODULE,
	.open		= ite7259_open,
	.release 	= ite7259_close,
	//.ioctl		= ite7259_ioctl,
	.unlocked_ioctl = ite7259_ioctl,
};
		
static struct miscdevice ctp_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= CTP_NAME,
	.fops	= &ite7259_fops,
};

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)  {
  //  strcpy(info->type, "mtk-tpd"); 
  strcpy(info->type, CTP_NAME); 
    return 0;
}

static int tpd_i2c_write(struct i2c_client *client, uint8_t *buf, int len)
{
    int ret = 0;
#ifdef I2C_SUPPORT_RS_DMA
    int i = 0;
    printk("taikoto : start tpd_i2c_write len=%d.\n",len);
    for(i = 0 ; i < len; i++){
        I2CDMABuf_va[i] = buf[i];
    }
    
    if(len < 8){
        client->addr = client->addr & I2C_MASK_FLAG;
        return i2c_master_send(client, buf, len);
    }else{
        client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
        return i2c_master_send(client, I2CDMABuf_pa, len);
    } 
#else
	i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
    ret = i2c_master_send(i2c_client, &buf[0], len);
	if(ret<0)
		printk("%s error\n",__func__);
	
	return ret;
#endif
    return ret;
}

static int tpd_i2c_read(struct i2c_client *client, uint8_t *buf, int len , uint8_t addr)
{
    int ret = 0;
	//printk("start tpd_i2c_read len=%d.\n",len);
#ifdef I2C_SUPPORT_RS_DMA
    int i = 0;
    if(len <= 8){
        buf[0] = addr;
        i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;
        ret = i2c_master_send(i2c_client, &buf[0], (len << 8 | 1));
    }else{
		
		/**
		struct i2c_msg msg;
		
		i2c_smbus_write_byte(i2c_client, addr);
		msg.flags = i2c_client->flags & I2C_M_TEN;
		msg.timing = 100;
		msg.flags |= I2C_M_RD;
		msg.len = len;
		msg.ext_flag = i2c_client->ext_flag;
		if(len <= 8)
		{
			msg.addr = i2c_client->addr & I2C_MASK_FLAG;
			msg.buf = buf;
			ret = i2c_transfer(i2c_client->adapter, &msg, 1);
			return (ret == 1)? len : ret;
		}else{
			client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG ;
			msg.addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
			msg.buf = I2CDMABuf_pa;
			ret = i2c_transfer(i2c_client->adapter, &msg, 1);
			if(ret < 0)
			{
				return ret;
			}
			for(i = 0; i < len; i++)
			{
			
				buf[i] = I2CDMABuf_va[i];
			}
			return ret;
		}
		*/
		//i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;
		
		//client->addr = client->addr & I2C_MASK_FLAG ;//| I2C_WR_FLAG | I2C_RS_FLAG;
		/**
		unsigned char buffer[256];
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;
		do{
			int times = len >> 3;
			int last_len = len & 0x7;
			int ii=0;

			for(ii=0;ii<times;ii++)
			{
				//ret = i2c_smbus_read_i2c_block_data(i2c_client,addr+ (ii<<2), len, (buf+ (ii<<2)));
				buf[ii<<3]=addr+ii<<3;
				
				ret = i2c_master_send(i2c_client, &buf[ii<<3], (8<<8 | 1));
				if(ret < 0)
				{
					printk("read error 380.\n");
					break;
				}
				printk("line 383 ret =%d",ret);
				msleep(20);
			}
			
			if(last_len > 0)
			{
				//ret = i2c_smbus_read_i2c_block_data(i2c_client,addr+ (ii<<2), last_len, (buf+ (ii<<2)));
//				*(buf+ii<<3)=addr+ii<<3;
				buf[ii<<3]=addr+ii<<3;
				ret=i2c_master_send(i2c_client,&buf[ii<<3], (last_len << 8 | 1));
				printk("line 392 ret =%d",ret);
				if(ret<0)
				{
					printk("read error 392.\n");
				}
			}
		}while(0);
		*/
		/**
		I2CDMABuf_va[0] = addr;
    	I2CDMABuf_va[9] = 0xFF;
    	I2CDMABuf_va[8] = 0xFF;
		
        
        //ret = i2c_master_recv(client, I2CDMABuf_pa, ((len+1) << 8 | 1));
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);
    
        if(ret < 0){
			printk("%s:i2c read error.\n", __func__);
            return ret;
        }
    
        for(i = 0; i < len; i++){
            buf[i] = I2CDMABuf_va[i];
        }
		*/
    }
#else
    buf[0] = addr;
    i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = i2c_master_send(i2c_client, &buf[0], (len << 8 | 1));
   // i2c_smbus_read_i2c_block_data(i2c_client,addr,len,buf);	
#endif

    return ret;
}



static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	TPD_DEBUG("Device Tree Tpd_irq_registration!\n");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);

		ret =
		    request_irq(touch_irq, (irq_handler_t) tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING,
				"TOUCH_PANEL-eint", NULL); //IRQF_TRIGGER_FALLING  IRQF_TRIGGER_RISING
		if (ret > 0) {
			ret = -1;
			TPD_DEBUG("tpd request_irq IRQ LINE NOT AVAILABLE!.\n");
		}
		
	} else {
		TPD_DEBUG("tpd request_irq can not find touch eint device node!.\n");
		ret = -1;
	}
	TPD_DEBUG("[%s]irq:%d, debounce:%d-%d:\n", __func__, touch_irq, ints[0], ints[1]);
	return ret;
}


static ssize_t IT7259_SpectrumTs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("%s():\n", __func__);
		
	disable_irq_nosync(touch_irq);
	//unsigned char APCmdBuffer[MAX_BUFFER_SIZE]; // byte 0: length  byte 1~n: command  byte n+1 : read length
	//unsigned char readBuffer[MAX_BUFFER_SIZE];  //size should be fixed by host transfer limit
	
 	
	enable_irq(touch_irq);
	return sprintf(buf, "%d", 1); // no need to show result

}
static ssize_t IT7259_SpectrumTs_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	
	int nWriteLength = 0;
	//int nReadLength = 0;
	//unsigned char buffer[MAX_BUFFER_SIZE];
	char cur[1000];
	//unsigned char cmd;
	int i = 0 ;
	int wTmp = COMMAND_ERROR;
	//char *pch;
	char *token;
	char *tmp = cur;
	char *after;  
	
	//printk("%s():\n", __func__);
	disable_irq_nosync(touch_irq);
	
	//unsigned char APCmdBuffer[MAX_BUFFER_SIZE]; // byte 0: length  byte 1~n: command  byte n+1 : read length
	//unsigned char readBuffer[MAX_BUFFER_SIZE];  //size should be fixed by host transfer limit
	
	
	
	//printk("count = %d\n",count);
	printk("buf:%s\n",buf);
	
	
	
	strcpy(cur, buf);
	while (NULL != (token = strsep(&tmp, " "))) {  
   		printk("token: %s\n", token);  
   		// cmd[i] = atoi(token);
   		
    		APCmdBuffer[i] = simple_strtol(token, &after, 10); // in kernel, use simple_strtol
    		printk("cmd: %d\n", APCmdBuffer[i]); 
		i++;
 	}  

		
	//pr_info("CMD: ");
	nWriteLength = (int)APCmdBuffer[0];
	pr_info("Channel: %d",nWriteLength);
	
	//datalen = (unsigned char) nWriteLength;
	//for (i = 0; i < nWriteLength; i++) {
			
	//	buffer[i] = APCmdBuffer[i+1];
	//	pr_info(" %x ",APCmdBuffer[i+1]);
	//}
		
	//i2cAdvancedWriteToIT7259(gl_ts->client, buffer[0], &(buffer[1]), (unsigned char)(nWriteLength - 1));
	wTmp = gfnIT7259_SpectrumModeTest(nWriteLength);

	if(wTmp != COMMAND_SUCCESS)
		pr_info(" \r\nIT7259_SpectrumTs_store Fail!.");
	else
		pr_info(" \r\nIT7259_SpectrumTs_store Success!.");

	
	enable_irq(touch_irq);
	return count;		

}

/******************************************************************************/
static void tpd_print_version(void)
{
#ifdef I2C_SUPPORT_RS_DMA
    char buffer[9];
#else
    char buffer[8];
#endif
    int ret = -1;

    do
    {
        buffer[0] = 0xFF;
        ret = tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
    } while (buffer[0] & 0x01);

    buffer[0] = 0x20;

    buffer[1] = 0x1;

    buffer[2] = 0x0;

    ret = tpd_i2c_write(i2c_client, buffer, 3);

    if (ret != 3)
    {
        printk("[mtk-tpd] i2c write communcate error in getting FW version : 0x%x\n", ret);
    }

    msleep(10);

    do
    {
        buffer[0] = 0xFF;
        ret = tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
    } while (buffer[0] & 0x01);

#ifdef I2C_SUPPORT_RS_DMA
    ret = tpd_i2c_read(i2c_client, &buffer[0], 9, 0xA0);

    if (ret != 0x901)
    {
        printk("[mtk-tpd] i2c read communcate error in getting FW version : 0x%x\n", ret);
    }
    else
    {
        printk("[mtk-tpd] ITE7260 Touch Panel Firmware Version %x %x %x %x %x %x %x %x %x\n",
               buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8]);
    }

#else
    ret = tpd_i2c_read(i2c_client, &buffer[0], 8, 0xA0);

    if (ret != 0x801)
    {
        printk("[mtk-tpd] i2c read communcate error in getting FW version : 0x%x\n", ret);
    }
    else
    {
        printk("[mtk-tpd] ITE7260 Touch Panel Firmware Version %x %x %x %x %x %x %x %x\n",
               buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
    }

#endif
}

#if 1

/*
typedef enum wk_wdt_en
{
    WK_WDT_DIS,
    WK_WDT_EN,
} WD_CTL;

#define    HAS_8_BYTES_LIMIT

extern int mtk_wdt_enable(enum wk_wdt_en en);
*/
/*
 * compare fw file's version and tp version, when equal return 0, otherwise 1
 */
static int tpd_compare_fw_version(unsigned char f1, unsigned char f2, unsigned char f3, unsigned char f4)
{
#ifdef I2C_SUPPORT_RS_DMA
    char buffer[9];
#else
    char buffer[8];
#endif
    int ret = -1;

    do
    {
        buffer[0] = 0xFF;
        ret = tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
    } while (buffer[0] & 0x01);

    buffer[0] = 0x20;

    buffer[1] = 0x1;

    buffer[2] = 0x0; /* 00-> firmware version, 06->config version */

    ret = tpd_i2c_write(i2c_client, buffer, 3);

    if (ret != 3)
    {
        printk("[mtk-tpd] tpd_compare_fw_version: i2c write communcate error in getting FW version : 0x%x\n", ret);
    }

    msleep(10);

    do
    {
        buffer[0] = 0xFF;
        ret = tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
    } while (buffer[0] & 0x01);

#ifdef I2C_SUPPORT_RS_DMA
    ret = tpd_i2c_read(i2c_client, &buffer[0], 9, 0xA0);

    if (ret != 0x901)
    {
        printk("[mtk-tpd] tpd_compare_fw_version: i2c read communcate error in getting FW version : 0x%x\n", ret);
    }
    else
    {
        printk("[mtk-tpd] tpd_compare_fw_version: ITE7260 Touch Panel Firmware Version %x %x %x %x %x [%x %x %x] %x, f1-f4 [%x %x %x] %x\n",
               buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], f1, f2, f3, f4);
    }

#else
    ret = tpd_i2c_read(i2c_client, &buffer[0], 8, 0xA0);

    if (ret != 0x801)
    {
        printk("[mtk-tpd] tpd_compare_fw_version: i2c read communcate error in getting FW version : 0x%x\n", ret);
    }
    else
    {
        printk("[mtk-tpd] tpd_compare_fw_version: ITE7260 Touch Panel Firmware Version %x %x %x %x %x [%x %x %x], f1-f4 [%x %x %x] %x\n",
               buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], f1, f2, f3, f4);
    }

#endif

    if ((buffer[5] == f1) && (buffer[6] == f2) && (buffer[7] == f3)) /* zgdezu: only compare first three number */
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/*
 * compare config file's version and tp version, when equal return 0, otherwise 1
 */
static int tpd_compare_cfg_version(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4)
{
#ifdef I2C_SUPPORT_RS_DMA
    char buffer[9];
#else
    char buffer[8];
#endif
    int ret = -1;

    do
    {
        buffer[0] = 0xFF;
        ret = tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
    } while (buffer[0] & 0x01);

    buffer[0] = 0x20;

    buffer[1] = 0x1;

    buffer[2] = 0x06; /* 00-> firmware version, 06->config version */

    ret = tpd_i2c_write(i2c_client, buffer, 3);

    if (ret != 3)
    {
        printk("[mtk-tpd] tpd_compare_cfg_version: i2c write communcate error in getting Config version : 0x%x\n", ret);
    }

    msleep(10);

    do
    {
        buffer[0] = 0xFF;
        ret = tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
    } while (buffer[0] & 0x01);

#ifdef I2C_SUPPORT_RS_DMA
    ret = tpd_i2c_read(i2c_client, &buffer[0], 9, 0xA0);

    if (ret != 0x901)
    {
        printk("[mtk-tpd] tpd_compare_cfg_version: i2c read communcate error in getting Config version : 0x%x\n", ret);
    }
    else
    {
        printk("[mtk-tpd] tpd_compare_cfg_version: ITE7260 Touch Panel Config Version %x %x %x %x %x %x %x %x %x, c1-c4 [%x %x %x %x]\n",
               buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], c1, c2, c3, c4);
    }

#else
    ret = tpd_i2c_read(i2c_client, &buffer[0], 8, 0xA0);

    if (ret != 0x801)
    {
        printk("[mtk-tpd] tpd_compare_cfg_version: i2c read communcate error in getting Config version : 0x%x\n", ret);
    }
    else
    {
        printk("[mtk-tpd] tpd_compare_cfg_version: ITE7260 Touch Panel Config Version %x [%x %x %x %x] %x %x %x, c1-c4 [%x %x %x %x]\n",
               buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], c1, c2, c3, c4);
    }

#endif

    if ((buffer[1] == c1) && (buffer[2] == c2) && (buffer[3] == c3) && (buffer[4] == c4)) /* zgdezu: config compare four number */
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

static int i2cInternalReadFromIT7259(struct i2c_client *client, uint8_t *buf, int len , uint8_t addr)
{
    int ret;
    int i;
    uint8_t tmp[255] = {0};

    tmp[0] = 0x70;
    tmp[1] = addr;    
    
    for( i = 0 ; i < len ; i++ )
    {
        tmp[i+2] = buf[i];
    }
    i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = i2c_master_send(i2c_client, tmp, ((len << 8) | 2));

    for( i = 0 ; i < len ; i++ )
    {
        buf[i] = tmp[i];
    }

    return ret;
}

static int i2cInternalWriteToIT7259(struct i2c_client *client, uint8_t reg, uint8_t *buf, int len)
{
    int i;
    int ret;
    uint8_t tmp[255] = {0};

    tmp[0] = 0x70;
    tmp[1] = reg;
    for (i = 0 ; i < len ; i++)
    {
        tmp[i+2] = buf[i];
    }

    i2c_client->addr &= I2C_MASK_FLAG;
    ret = i2c_master_send(i2c_client, tmp, len + 2); 
    return ret;
}

static int i2cDMAReadFromIT7259(struct i2c_client *client, uint16_t addr, uint8_t *buf, int len )
{
    int ret;
    uint8_t regAdd[4];

    regAdd[0] = 0x90;
    regAdd[1] = 0x00;
    regAdd[2] = (addr &0xFF00) >> 8;
    regAdd[3] = addr &0xFF;
    i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
    ret = i2c_master_send(i2c_client, regAdd, 4);
    //i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = tpd_i2c_dma_read(i2c_client, len, buf);

    return ret;
}
/*
static int i2cDMAWriteToIT7259(struct i2c_client *client, uint16_t reg, uint8_t *buf, int len)
{
    int nRet;
    uint8_t regAdd[4];

    regAdd[0] = 0x10;
    regAdd[1] = 0x00;
    regAdd[2] = (reg & 0xFF00) >> 8;
    regAdd[3] = reg & 0xFF;
    i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
    nRet = i2c_master_send(i2c_client, regAdd, 4);
    //i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    nRet = tpd_i2c_dma_write(i2c_client, len, buf);

    return nRet;
}
*/
static int i2cDirectReadFromIT7259(struct i2c_client *client, uint16_t addr, uint8_t *buf, int len )
{
#if 0
    int ret;
    uint8_t regAdd[4];

    regAdd[0] = 0x90;
    regAdd[1] = 0x00;
    regAdd[2] = (addr &0xFF00) >> 8;
    regAdd[3] = addr &0xFF;
    i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
    ret = i2c_master_send(i2c_client, regAdd, 4);
    //i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = tpd_i2c_dma_read(i2c_client, len, buf);
#else
    int ret;
    int i;
    uint8_t tmp[255] = {0};
    tmp[0] = 0x90;
    tmp[1] = 0x00;
    tmp[2] = (addr &0xFF00) >> 8;
    tmp[3] = addr &0xFF;

    for( i = 0 ; i < len ; i++ )
    {
         tmp[i+4] = buf[i];
    }
           
     i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = i2c_master_send(i2c_client, tmp, ((len << 8) | 4));   
    
    for (i = 0 ; i < len; i++)
    {
        buf[i] = tmp[i];
    }
#endif
    return ret;
}

static int i2cDirectWriteToIT7259(struct i2c_client *client, uint16_t reg, uint8_t *buf, int len)
{
#if 0
    int nRet;
    uint8_t regAdd[4];

    regAdd[0] = 0x10;
    regAdd[1] = 0x00;
    regAdd[2] = (reg & 0xFF00) >> 8;
    regAdd[3] = reg & 0xFF;
    i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
    nRet = i2c_master_send(i2c_client, regAdd, 4);
    //i2c_client->addr = (i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    nRet = tpd_i2c_dma_write(i2c_client, len, buf);
#else
    int i = 0;
    int nRet = 0;
    uint8_t tmp[255] = {0};
    
    tmp[0] = 0x10;
    tmp[1] = 0x00;
    tmp[2] = (unsigned char)((reg &0xFF00) >> 8);
    tmp[3] = (unsigned char)(reg &0xFF);
    
    for( i = 0 ; i < len ; i++ )
    {
         tmp[i+4] = buf[i];
    }

    i2c_client->addr &= I2C_MASK_FLAG;
    nRet = i2c_master_send(i2c_client, tmp, len+4);
#endif
    return nRet;
}

//20160612 Tony add
bool gfnIT7259_SwitchCPUClock(unsigned char ucMode)
{
    //////
    //printk("%s(0x%02x):\n", __func__,ucMode);

    //struct IT7259_ts_data *ts = gl_ts;
    unsigned char ucCommandBuffer[1];
    unsigned char ucRetCommandBuffer[1];

    int nErrCount = 0;
    int dwAddress = 0x23;

    ucCommandBuffer[0] = ucMode;
    do
    {
        i2cInternalWriteToIT7259(i2c_client, dwAddress,ucCommandBuffer,1);

        i2cInternalReadFromIT7259(i2c_client, ucRetCommandBuffer, 1, dwAddress);

        nErrCount++;
    } while (((ucRetCommandBuffer[0] & 0x0F)  != ucMode) && nErrCount <= 1000);

    printk("====== gfnIT7259_SwitchCPUClock read/write OK!, nErrCount=%d ===========\n", nErrCount);
    if (nErrCount>1000)
    {
        return false;
    }

    return true;
}

bool gfnIT7259_SPIFCRReady(void)
{
    //printk("###Entry gfnIT7259_SPIFCRReady()\n");
    //struct IT7259_ts_data *ts = gl_ts;
    int nReadCount=0;
    unsigned char ucBuffer[2];

    do
    {
        i2cDirectReadFromIT7259(i2c_client, 0xF400, ucBuffer, 2);//gfnIT7259_DirectReadMemoryRegister
    } while (((ucBuffer[1]& 0x01)!=0x00) && ++nReadCount<20); //nReadCount 3000

    if (nReadCount >=20) //nReadCount 3000
    {
        return false;
    }

    return true;
}

int gfnIT7259_DMAModeWriteFlash(uint16_t wFlashAddress, uint16_t wSRAMAddress,unsigned int dataLength, unsigned char* pData, bool bPollingWait)
{
    //printk("###Entry gfnIT7259_DMAModeWriteFlash(wFlashAddress = %04x, dataLength = %d)\n",wFlashAddress, dataLength);
    //struct IT7259_ts_data *ts = gl_ts;
    int nSector = 0;
    int wAddress;
    int wTmp;
    unsigned char pucCommandBuffer[1024];
    unsigned char pucReadData[2];
    unsigned int LenOffset;
    unsigned char bufTemp[4];
    int i;
    unsigned int wStartAddress = wFlashAddress - (nSector*0x0400);

    //write  to address 0x0000 (SRAM only 6K)
#if 0
    memset(pucCommandBuffer, 0xFF, 1024);
    memcpy(pucCommandBuffer, (unsigned char*)pData, dataLength * sizeof(unsigned char));
    wAddress = wSRAMAddress;//0x0000
    printk("###write  to address 0x0000 (wSRAMAddress = %04x, dataLength = %02x)\n",wSRAMAddress,dataLength);
    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,dataLength);//DirectWriteMemoryRegister

    if (wTmp <= 0)
    {
        //printk("###write  to address 0x0000 fail!\n");
        return COMMAND_ERROR;
    }

#else
    memset(bufTemp, 0xFF, 4);

    wAddress = wSRAMAddress;

    //printk("###write  to address 0x0000 (wSRAMAddress = %04x, dataLength = %02x)\n",wSRAMAddress,dataLength);

    for (LenOffset=0; LenOffset < dataLength; LenOffset+=4) //for 8 byte limit
    {
        for (i = 0; i < 4; i++)
        {
            bufTemp[i] = pData[LenOffset + i];
        }

        wTmp = i2cDirectWriteToIT7259(i2c_client, wAddress, bufTemp, 4);
        //mdelay(1);//for test
        /*if (wTmp <= 0)
        {
            //printk("###write  to address 0x0000 fail!\n");
            return COMMAND_ERROR;
        }*/

        wAddress = wAddress + 4;

        //printk(" ======== wAddress = %04x , LenOffset = %02x ========\n",wAddress,LenOffset);
    }

#endif
    //Select Sector
    memset(pucCommandBuffer, 0xFF, 1024);

    nSector = wFlashAddress/0x0400;

    pucCommandBuffer[0] = (unsigned char)(nSector & 0xFF);

    wAddress = 0xF404;

    printk("###Select Sector(nSector = %02x)\n", pucCommandBuffer[0]);

    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,1);//DirectWriteMemoryRegister
    //mdelay(1);//for test
	printk("======= wTmp = %d =========\n",wTmp);
    /*if (wTmp <= 0)
    {
        printk("###Select Sector fail!");
        return COMMAND_ERROR;
    }*/

    //Wait SPIFCR
    //printk("###Wait SPIFCR\n");

    if (!gfnIT7259_SPIFCRReady())
    {
        return ERROR_QUERY_TIME_OUT;
    }

    //Write Flash strat address
    printk("###Write Flash strat address\n");

    memset(pucCommandBuffer, 0xFF, 1024);

    wAddress = 0xF41A;

    pucCommandBuffer[0] =  wStartAddress & 0x00FF;

    pucCommandBuffer[1] = (wStartAddress & 0xFF00) >> 8 ;

    printk("###Write Flash strat address(pucCommandBuffer[0] = %02x) (pucCommandBuffer[1]= %02x)\n", pucCommandBuffer[0],pucCommandBuffer[1]);
    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,2);//DirectWriteMemoryRegister
    //mdelay(1);//for test

    /*if (wTmp <= 0)
    {
        printk("###Write Flash strat address fail!\n");
        return COMMAND_ERROR;
    }*/

    //Write SARM strat address
    wAddress = 0xF41C;

    memset(pucCommandBuffer, 0xFF, 1024);

    pucCommandBuffer[0] =  wSRAMAddress & 0xFF;

    pucCommandBuffer[1] = (wSRAMAddress & 0xFF00) >> 8 ;

    printk("###Write SARM strat address(pucCommandBuffer[0] = %02x) (pucCommandBuffer[1]= %02x)\n", pucCommandBuffer[0],pucCommandBuffer[1]);

    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,2);//DirectWriteMemoryRegister
    //mdelay(1);//for test

    /*if (wTmp <= 0)
    {
        printk("###Write SARM strat address fail!\n");
        return COMMAND_ERROR;
    }*/

    //write DMA transfer length
    wAddress = 0xF41E;

    pucCommandBuffer[0] =  dataLength & 0xFF;

    pucCommandBuffer[1] = (dataLength & 0xFF00) >> 8 ;

    printk("###Write DMA transfer length(pucCommandBuffer[0] = %02x) (pucCommandBuffer[1]= %02x)\n", pucCommandBuffer[0],pucCommandBuffer[1]);

    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,2);//DirectWriteMemoryRegister
    //mdelay(1);//for test

    /*if (wTmp <= 0)
    {
        //printk("###write DMA transfer length fail!\n");
        return COMMAND_ERROR;
    }*/

    //Write DMA_DIR and DMAEN
    wAddress = 0xF418;

    pucCommandBuffer[0] = 0x0B; //auto erase

    pucCommandBuffer[1] = 0x00;

    printk("###Write DMA_DIR and DMAEN(pucCommandBuffer[0] = %02x) (pucCommandBuffer[1]= %02x)\n", pucCommandBuffer[0],pucCommandBuffer[1]);

    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,2);//DirectWriteMemoryRegister
    //mdelay(1);//for test

    /*if (wTmp <= 0)
    {
        printk("###Write DMA_DIR and DMAEN fail!\n");
        return COMMAND_ERROR;
    }*/

    if (bPollingWait)
    {
        //polling bit 0, until value of bit 0 = 0
        wAddress = 0xF418;
        printk("============= ivan test bPollingWait ===========\n");
        do
        {
            wTmp = i2cDirectReadFromIT7259(i2c_client, wAddress, pucReadData, 2);//gfnIT7259_DirectReadMemoryRegister

            /*if (wTmp <= 0)
            {
                break;
                return COMMAND_ERROR;
            }*/
        } while ((pucReadData[0] & 0x01)!= 0x00);
        printk("============= ivan test bPollingWait 1===========\n");

        //Wait SPIFCR
        if (!gfnIT7259_SPIFCRReady())
        {
            return ERROR_QUERY_TIME_OUT;
        }
    }

    return COMMAND_SUCCESS;
}

unsigned int gfnIT7259_DirectReadFlash(uint16_t wFlashAddress, uint16_t readLength, unsigned char* pData)
{
    //printk("###Entry gfnIT7259_DirectReadFlash()\n");
    //struct IT7259_ts_data *ts = gl_ts;
    int nSector = 0;
    unsigned char pucCommandBuffer[1024];
    int wTmp;
    int wAddress;
    unsigned int LenOffset;
    unsigned char bufTemp[4];
    int i;
    int wOffset = 0;

    nSector = wFlashAddress/0x0400;
    pucCommandBuffer[0] = nSector;

    //Select Sector
    wAddress = 0xF404;
    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,1);

    /*if (wTmp <= 0)
    {
        return COMMAND_ERROR;
    }*/

    //Wait SPIFCR

    if (!gfnIT7259_SPIFCRReady())
    {
        return ERROR_QUERY_TIME_OUT;
    }

    //Read flash
    wOffset = wFlashAddress - (nSector*0x0400);
    wAddress = 0x3000 + wOffset;

#if 0
    wTmp = i2cDirectReadFromIT7259(i2c_client, wAddress, pucCommandBuffer, readLength);

    if (wTmp <= 0)
    {
        return COMMAND_ERROR;
    }

#else
    for (LenOffset = 0; LenOffset < readLength; LenOffset += 4) //for 8 byte limit
    {
        wTmp = i2cDirectReadFromIT7259(i2c_client, wAddress, bufTemp, 4);

        if (wTmp <= 0)
        {
            return COMMAND_ERROR;
        }

        for (i = 0; i < 4; i++)
        {
            pucCommandBuffer[LenOffset + i] = bufTemp[i] ;
        }

        wAddress = wAddress + 4;

        //printk(" ======== wAddress = %04x , LenOffset = %02x ========\n",wAddress,LenOffset);
    }

#endif
    //Wait SPIFCR
    if (!gfnIT7259_SPIFCRReady())
    {
        return ERROR_QUERY_TIME_OUT;
    }

    memcpy((unsigned char*)pData, pucCommandBuffer, readLength * sizeof(unsigned char));

    return COMMAND_SUCCESS;
}

//Tony add
static unsigned int gfnIT7259_GetFWSize(void)
{
    //printk("###Entry gfnIT7259_GetFWSize()\n");
    unsigned int unRet = 0;
    unsigned char arucBuffer[1024];
    uint16_t wAddress;

    wAddress = 0;
    gfnIT7259_DirectReadFlash(wAddress,0x0400,arucBuffer);

    unRet = arucBuffer[0x80+12] + (arucBuffer[0x80+13] << 8);

    return unRet;
}

int gfnIT7259_DirectEraseFlash(unsigned char ucEraseType, int wFlashAddress)
{
    //printk("###Entry gfnIT7259_DirectEraseFlash()\n");
    //struct IT7259_ts_data *ts = gl_ts;
    int nSector = 0;
    unsigned char pucCommandBuffer[1024];
    int wTmp;
    int wAddress;
    nSector = wFlashAddress/0x0400;
    pucCommandBuffer[0] = nSector;

    //Select Sector
    wAddress = 0xF404;
    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,1);

    /*if (wTmp <= 0)
    {
        return COMMAND_ERROR;
    }*/

    //Wait SPIFCR

    if (!gfnIT7259_SPIFCRReady())
    {
        return ERROR_QUERY_TIME_OUT;
    }

    //Read flash
    wAddress = 0xF402;

    pucCommandBuffer[0] = ucEraseType;

    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,1);

    //Wait SPIFCR
    if (!gfnIT7259_SPIFCRReady())
    {
        return ERROR_QUERY_TIME_OUT;
    }

    return COMMAND_SUCCESS;
}

int gfnIT7259_DirectWriteFlash(int wFlashAddress, unsigned int wWriteLength, unsigned char* pData)
{
    //printk("###Entry gfnIT7259_DirectWriteFlash().......wWriteLength = %d\n",wWriteLength);
    //struct IT7259_ts_data *ts = gl_ts;
    int nSector = 0;
    unsigned char pucCommandBuffer[1024];
    int wTmp;
    int wAddress;
    int wOffset = 0;

    nSector = wFlashAddress/0x0400;
    pucCommandBuffer[0] = nSector;

    //Select Sector
    wAddress = 0xF404;
    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,1);

    /*if (wTmp <= 0)
    {
        return COMMAND_ERROR;
    }*/

    //Wait SPIFCR

    if (!gfnIT7259_SPIFCRReady())
    {
        return ERROR_QUERY_TIME_OUT;
    }

    //write flash
	wOffset = wFlashAddress - (nSector*0x0400);
    wAddress = 0x3000 + wOffset;

    memcpy(pucCommandBuffer, (unsigned char*)pData, wWriteLength * sizeof(unsigned char));

    //wTmp = gfnIT7259_DirectWriteMemoryRegister(0x01,wAddress,0x0000,wWriteLength, pucCommandBuffer);
    wTmp = i2cDirectWriteToIT7259(i2c_client,wAddress,pucCommandBuffer,wWriteLength);

    /*if (wTmp <= 0)
    {
        return COMMAND_ERROR;
    }*/

    //Wait SPIFCR

    if (!gfnIT7259_SPIFCRReady())
    {
        return ERROR_QUERY_TIME_OUT;
    }

    return COMMAND_SUCCESS;
}

//Tony add for IT 7259
bool gfnIT7259_FirmwareDownload(unsigned int unFirmwareLength, unsigned char arucFW[], unsigned int unConfigLength, unsigned char arucConfig[])
{
   // printk("%s\n", __func__);
/*
    if ((unFirmwareLength == 0 ) && (unConfigLength == 0 ))
    {
        printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
        return false;
    }
    else
    {
        printk("###001 Entry gfnIT7259_FirmwareDownload()\n");
    }
*/
    int dwAddress;

    unsigned char RetDATABuffer[10];

    unsigned char DATABuffer[10];

    int nSector = 0;

    unsigned int nFillSize = 0;

    int wTmp;

    unsigned int unTmp = gfnIT7259_GetFWSize();

//    unsigned int nConfigSize = unConfigLength;


    //3.7 get address for writing config (in flash)
    //check whether fw and config are in the same sector or not
    //set flash size
    unsigned long dwFlashSize = 0x10000;

    unsigned int nEndFwSector = (unTmp-1) / 1024;

    unsigned int nStartCFGSector = 62 - (unConfigLength-1)/1024;

    unsigned char putFWBuffer[1024];

    unsigned int nRemainderFwSize = 0;
    int nConfigCount = 0;
    unsigned int i = 0;

    if ((unFirmwareLength == 0 ) && (unConfigLength == 0 ))
    {
        printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
        return false;
    }

    //trun off CPU data clock
    if (!gfnIT7259_SwitchCPUClock(0x01))
    {
        printk("###002 gfnIT7259_SwitchCPUClock(0x01) fail!\n");
        return false;
    }

    printk("###002 gfnIT7259_SwitchCPUClock(0x01) OK!\n");

    //Wait SPIFCR
    printk("###003 Wait SPIFCR\n");
    dwAddress = 0xF400;

    do
    {
        i2cDirectReadFromIT7259(i2c_client,dwAddress,RetDATABuffer,2);
    } while ((RetDATABuffer[1] & 0x01)  != 0x00);

    printk("###003 End SPIFCR\n");

    //Erase signature
    printk("###004 Erase signature\n");

    dwAddress = 0xF404;

    DATABuffer[0] = 0x3F;

    i2cDirectWriteToIT7259(i2c_client,dwAddress,DATABuffer,1);

    dwAddress = 0xF402;

    DATABuffer[0] = 0xD7;

    i2cDirectWriteToIT7259(i2c_client,dwAddress,DATABuffer,1);

    //Wait SPIFCR
    printk("###005 Wait SPIFCR\n");

    dwAddress = 0xF400;

    do
    {
        i2cDirectReadFromIT7259(i2c_client,dwAddress,RetDATABuffer,2);
    } while ((RetDATABuffer[1] & 0x01)  != 0x00);

    printk("###005 End SPIFCR\n");

    //Download FW
    printk("###006 Download FW\n");

    for (i  = 0 ; i < unFirmwareLength ; i+=0x0400) //0x0400
    {
        if ((unFirmwareLength - i) >= 0x0400)//0x0400
            nFillSize = 0x0400;//0x0400
        else
            nFillSize = unFirmwareLength - i ;

        wTmp = gfnIT7259_DMAModeWriteFlash(i,0x0000,nFillSize,arucFW+i,true);

        //mdelay(1);//for test

        if (wTmp != COMMAND_SUCCESS)
        {
            //Write Firmware Flash error
            printk("###DMA ModeWrite Firmware Flash error(FlashAddress:%04x)\n",i);
            return false;
        }
    }

    //3. Fw CRC Check

    //check FW CRC
    printk("###007 check FW CRC\n");

    //write start address
    printk("###007 write start address\n");

    dwAddress = 0xF40A;

    DATABuffer[0] = 0x00;

    DATABuffer[1] = 0x00;

    i2cDirectWriteToIT7259(i2c_client,dwAddress,DATABuffer,2);

    //write end address
    printk("###007 write end address\n");

    dwAddress = 0xF40C;

    DATABuffer[0] = (unFirmwareLength-3) & 0x00ff ;

    DATABuffer[1] = ((unFirmwareLength-3) & 0xff00)>>8;

    i2cDirectWriteToIT7259(i2c_client,dwAddress,DATABuffer,2);

    //write CRCCR
    printk("###007 write CRCCR\n");

    dwAddress = 0xF408;

    DATABuffer[0] = 0x01 ;

    i2cDirectWriteToIT7259(i2c_client,dwAddress,DATABuffer,1);

    //wait CRCCR
    printk("###007 wait CRCCR\n");

    dwAddress = 0xF408;

    do
    {
        i2cDirectReadFromIT7259(i2c_client,dwAddress,RetDATABuffer,2);
    } while ((RetDATABuffer[0] & 0x01)  != 0x00);

    //read CRC
    printk("###007 read CRC\n");

    dwAddress = 0xF40E;

    i2cDirectReadFromIT7259(i2c_client,dwAddress,RetDATABuffer,2);
    printk("###007 read CRC RetDATABuffer[0] = 0x%x, RetDATABuffer[1] = 0x%x\n",RetDATABuffer[0],RetDATABuffer[1]);

    //compare FW CRC
    printk("###008 compare FW CRC\n");

    if (RetDATABuffer[0]!= arucFW[unFirmwareLength - 2] || RetDATABuffer[1]!= arucFW[unFirmwareLength - 1])
    {
        printk("###008 FW CRC check fail\n");
        printk("RetDATABuffer[0]:%02x,RetDATABuffer[1]:%02x,FW[Length-2]:%02x,FW[Length-1]:%02x\n",RetDATABuffer[0], RetDATABuffer[1], arucFW[unFirmwareLength - 2], arucFW[unFirmwareLength - 1]);
        return false;//FW CRC check fail
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //download config
    printk("###009 start to download config\n");

    printk("###009-1 start to download config\n");

    if (nEndFwSector == nStartCFGSector)
    {
        int wAddress = nEndFwSector*0x0400;
        nRemainderFwSize = unTmp - nEndFwSector*1024;
        gfnIT7259_DirectReadFlash(wAddress,nRemainderFwSize,putFWBuffer);
    }

    //get config start address
    wTmp = dwFlashSize -1024 - unConfigLength;

    printk("###010 get config start address(%04x)\n",wTmp);

    for (i = wTmp ; i<(dwFlashSize -1024) ; i+=0x0400)
    {
        int nSize = 0;
        int wConfigAddress = 0;

        nSector = i/0x0400;

        if ((nRemainderFwSize!=0) && (nSector == nStartCFGSector))
        {
            int wRemainderFWAddress = nStartCFGSector*0x0400;
            nFillSize = nRemainderFwSize;
            gfnIT7259_DMAModeWriteFlash(wRemainderFWAddress,0x0000,nFillSize,putFWBuffer,true);
        }

        //write config
        nSize = (unConfigLength - (62-nSector)*1024);

        if (nSize >=1024)
        {
            wConfigAddress = nSector * 0x0400;
            nFillSize = 1024;
        }
        else
        {
            wConfigAddress = i;
            nFillSize = nSize;
        }

        wTmp = gfnIT7259_DMAModeWriteFlash(wConfigAddress ,0x0000, nFillSize, arucConfig + nConfigCount , true);

        if (wTmp != COMMAND_SUCCESS)
            return false;

        nConfigCount += nFillSize;
    }

    //Config CRC Check
    printk("###011 Config CRC Check\n");

    //write start address
    printk("###011 write Config start address\n");

    dwAddress = 0xF40A;

    DATABuffer[0] = (0x10000 - unConfigLength -1024) & 0x00ff;

    DATABuffer[1] = ((0x10000- unConfigLength -1024) & 0xff00)>>8;

    i2cDirectWriteToIT7259(i2c_client,dwAddress,DATABuffer,2);

    //write end address
    printk("###011 write Config end address\n");

    dwAddress = 0xF40C;

    DATABuffer[0] = (0x10000 -1024 -3)& 0x00ff;

    DATABuffer[1] = ((0x10000-1024 -3) & 0xff00)>>8;

    i2cDirectWriteToIT7259(i2c_client,dwAddress,DATABuffer,2);

    //write CRCCR
    printk("###011 write CRCCR\n");

    dwAddress = 0xF408;

    DATABuffer[0] = 0x01 ;

    i2cDirectWriteToIT7259(i2c_client,dwAddress,DATABuffer,1);

    //wait CRCCR
    printk("###011 wait CRCCR\n");

    dwAddress = 0xF408;

    do
    {
        i2cDirectReadFromIT7259(i2c_client,dwAddress,RetDATABuffer,2);
    } while ((RetDATABuffer[0] & 0x01)  != 0x00);

    //read CRC
    printk("###011 read CRC\n");

    dwAddress = 0xF40E;

    i2cDirectReadFromIT7259(i2c_client,dwAddress,RetDATABuffer,2);

    //compare Config CRC
    printk("###011 compare Config CRC\n");

    if (RetDATABuffer[0]!= arucConfig[unConfigLength - 2] || RetDATABuffer[1]!= arucConfig[unConfigLength - 1])
    {
        printk("###011 config CRC Check Error\n");
        printk("RetDATABuffer[0]:%02x,RetDATABuffer[1]:%02x,CFG[Length-2]:%02x,CFG[Length-1]:%02x\n",RetDATABuffer[0], RetDATABuffer[1], arucConfig[unConfigLength - 2], arucConfig[unConfigLength - 1]);
        return false;//config CRC Check Error
    }

    DATABuffer[0] = 0x59;

    DATABuffer[1] = 0x72;

    gfnIT7259_DirectEraseFlash(0xD7,(dwFlashSize -1024));
    gfnIT7259_DirectWriteFlash((dwFlashSize -1024),2,DATABuffer);

    DATABuffer[0] = 0x00;
    DATABuffer[1] = 0x00;
    i2cDirectReadFromIT7259(i2c_client,dwAddress,DATABuffer,2);

    //trun on CPU data clock
    printk("###012 trun on CPU data clock\n");

    if (!gfnIT7259_SwitchCPUClock(0x04))
    {
        printk("###012 trun on CPU data clock fail\n");
        return false;
    }

    printk("###gfnIT7259_FirmwareDownload() end.\n");

    return true;
}

static int Upgrade_FW_CFG(void)
{
//    printk("Execute Upgrade_FW_CFG().\n");
    unsigned int fw_size = 0;
    unsigned int config_size = 0;
    return 0;
#if 0
   
    struct file *fw_fd = NULL;

    struct file *config_fd = NULL;
    mm_segment_t fs;

    printk("Upgrade_FW_CFG 1.\n");
//    unsigned char *fw_buf = kzalloc(0x10000, GFP_KERNEL);
//    unsigned char *config_buf = kzalloc(0x500, GFP_KERNEL);

    if (fw_buf  == NULL || config_buf == NULL)
    {
        printk("kzalloc failed.\n");
        return 1;
    }

    printk("Upgrade_FW_CFG 2.\n");

    fs = get_fs();

    printk("Upgrade_FW_CFG 3.\n");
    set_fs(get_ds());

    printk("Upgrade_FW_CFG 4.\n");

    //load fw file
    fw_fd = filp_open("/system/etc/firmware/firmware_ver_1.16.0.0_2016_09_13_14_54_33.bin", O_RDONLY, 0);

    if (fw_fd <= 0)
    {
        //if(IS_ERR(fw_fd)) {
        printk("open /system/etc/firmware/firmware_ver_1.16.0.0_2016_09_13_14_54_33.bin failed, fw_fd=%02X.\n", (unsigned int)fw_fd);
        return 1;
    }

    printk("Upgrade_FW_CFG 5.\n");

    fw_size = fw_fd->f_op->read(fw_fd, fw_buf, 0x10000, &fw_fd->f_pos);
    //printk("fw_ver : %d,%d,%d,%d\n",fw_buf[8], fw_buf[9], fw_buf[10], fw_buf[11]);
    //printk("--------------------- fw_size = %x\n", fw_size);

    printk("Upgrade_FW_CFG 6.\n");
    //load config file
    config_fd = filp_open("/system/etc/firmware/config_ver1.1.5.5.bin", O_RDONLY, 0);

    if (config_fd <= 0)
    {
        //if(IS_ERR(config_fd)) {
        printk("open /system/etc/firmware/config_ver1.1.5.5.bin failed, config_fd=%02X.\n", (unsigned int)config_fd);
        return 1;
    }

    printk("Upgrade_FW_CFG 7.\n");

    config_size = config_fd->f_op->read(config_fd, config_buf, 0x500, &config_fd->f_pos);
    //printk("cfg_ver : %d,%d,%d,%d\n",config_buf[config_size-8], config_buf[config_size-7], config_buf[config_size-6], config_buf[config_size-5]);
    //printk("--------------------- config_size = %x\n", config_size);
    //end

    set_fs(fs);
    filp_close(fw_fd,NULL);
    filp_close(config_fd,NULL);
#else
    fw_size = sizeof(fw_buf1);
    config_size = sizeof(config_buf1);
#endif

    if (0 == tpd_compare_fw_version(fw_buf1[136], fw_buf1[137], fw_buf1[138], fw_buf1[139]))
    {
        printk("IT7259 Firmware equal.\n");

		if (0 == tpd_compare_cfg_version(config_buf1[config_size-8], config_buf1[config_size-7], config_buf1[config_size-6], config_buf1[config_size-5]))
		{
			printk("IT7259 Config equal.\n");
			return 1;
		}
    }

    printk("IT7259 mtk_wdt_enable(WK_WDT_DIS).\n");

#if 1
    mtk_wdt_enable(WK_WDT_DIS);

    printk("IT7259 gfnIT7259_FirmwareDownload().\n");

    if (gfnIT7259_FirmwareDownload(fw_size, fw_buf1, config_size, config_buf1) == false)
    {
        //fail
        printk("Execute gfnIT7259_FirmwareDownload fail.\n");
        mtk_wdt_enable(WK_WDT_EN);
        return 1;
    }
    else
    {
        //success
        printk("Execute gfnIT7259_FirmwareDownload success.\n");
        mtk_wdt_enable(WK_WDT_EN);
        return 0;
    }

#else
    return 1;

#endif
}

#endif

/*******************************************************************************/


static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;//, ret = -1
    unsigned char Wrbuf[2] = { 0x20, 0x07 };
   // unsigned char Rdbuffer[10];
    int retval;
#ifdef I2C_SUPPORT_RS_DMA
    char buffer[9];
#else
   char buffer[8];
#endif
    char i=0;
//	char buffer1[14];

    i2c_client = client;
	// add i2c_device_id
	if (i2c_client->addr != 0x46)
    {
        i2c_client->addr = 0x46;
    }
	
    printk(" taikoto : MediaTek it7259 touch panel i2c probe:%s\n", id->name);
#if 0	//by FHH
//#else
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO); 
	msleep(150);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	// Power Output Low
    //mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
    // mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    //mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN,GPIO_PULL_UP);
    // mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
    // mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);
	// Interrupt Output Low
    msleep(100);
	// Power Output High
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	// Interrupt Input High
    msleep(50);
#endif
	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
	tpd_gpio_output(GTP_RST_PORT, 1); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 0);
	msleep(100);
	tpd_gpio_output(GTP_RST_PORT, 0); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 1);
	msleep(200);
	tpd_gpio_output(GTP_RST_PORT, 1); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 0);
#ifdef I2C_SUPPORT_RS_DMA
    I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
    if(!I2CDMABuf_va)
    {
        printk("it7259 Allocate Touch DMA I2C Buffer failed!\n");
        return -1;
    }
#endif
	//ret = sysfs_create_group(&(client->dev.kobj), &it7259_attr_group);
	err = sysfs_create_group(&client->dev.kobj, &ite_attr_group);

	msleep(100);
	//kthread_run(upgrade_thread_kthread, NULL, "upgrade_thread_kthread");
    /*mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);*/
	tpd_irq_registration();//ugrec_tky
	//enable_irq(touch_irq);
    printk("taikoto : MediaTek it7259 touch panel i2c probe success\n");
    msleep(100);
	
	tpd_print_version();        //read firmware version
	
    tpd_i2c_write(i2c_client, Wrbuf, 2);   // Clean Queue {0x20, 0x07}
    do{
        i++;
		tpd_i2c_read(i2c_client, buffer, 1, 0x80);
        if(i>100)
        {
         i = 0;
         break;
        }
    }while( buffer[0] & 0x01 );
	
    if(tpd_i2c_read(i2c_client, buffer, 2, 0xA0) < 0)
	{
		 TPD_DMESG("taikoto : it7259 I2C transfer error, line: %d\n", __LINE__);
		// return -1; //ysong
	}
  
    thread = kthread_run(touch_event_handler, 0, CTP_NAME);
    if (IS_ERR(thread)) { 
        err = PTR_ERR(thread);
        printk(CTP_NAME " taikoto : it7259 failed to create kernel thread: %d\n", err);
    }
	
    tpd_load_status = 1;  
    printk("DDD_____ 0xA0 : %X, %X\n", buffer[0], buffer[1]);  // add FAE   End
	
	#if 1
    tpd_halt = 1;
    Upgrade_FW_CFG();
    msleep(100);
    tpd_halt = 0;
	#endif
	
    return 0;
}

void tpd_eint_interrupt_handler(void)
{	
    TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	printk("tp_int\n"); 
	wake_up_interruptible(&waiter);
} 

static int tpd_i2c_remove(struct i2c_client *client)
{
#ifdef I2C_SUPPORT_RS_DMA
    if( I2CDMABuf_va ){
		dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;
    }
#endif
    return 0;
}

void tpd_down(int raw_x, int raw_y, int x, int y, int p) {	

    input_report_abs(tpd->dev, ABS_PRESSURE, 128);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 128);
    input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 128);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);

    input_mt_sync(tpd->dev);
    //printk("D[%4d %4d %4d]\n", x, y, p); //by FHH
    TPD_EM_PRINT(raw_x, raw_y, x, y, p, 1);
}

void tpd_up(int raw_x, int raw_y, int x, int y, int p) {

    input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 0);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    //printk("U[%4d %4d %4d]\n", x, y, 0); //by FHH
    TPD_EM_PRINT(raw_x, raw_y, x, y, p, 0);
}

static int x[2] = { (int) -1, (int) -1 };
static int y[2] = { (int) -1, (int) -1 };
static bool finger[2] = { 0, 0 };
static bool flag = 0;

static int touch_event_handler( void *unused )
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; 
    unsigned char pucPoint[14];
	//unsigned char key_temp=0;
#ifndef I2C_SUPPORT_RS_DMA
    unsigned char cPoint[8];
    unsigned char ePoint[6];
	unsigned char Rd_P_buf[3] = { 0x20,0x18, 0x07 };
	unsigned char Rd_keep[4] = { 0x20,0x19, 0x00, 0x06 };
	unsigned char Rd_final[4] = { 0x20,0x19, 0x01, 0x08 };
	unsigned char buffer[1];
#endif
    int ret = 0;
    int xraw, yraw;
    int i = 0;

    sched_setscheduler(current, SCHED_RR, &param); 
    do{
        set_current_state(TASK_INTERRUPTIBLE);
        while (tpd_halt) {tpd_flag = 0; msleep(20);}
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        TPD_DEBUG_SET_TIME;
        set_current_state(TASK_RUNNING); 

        ret = tpd_i2c_read(i2c_client, &pucPoint[0], 1, 0x80);
		//taikoto_test 
        printk("***********************111111pucPoint[0] = %d\n",pucPoint[0]);
        TPD_DEBUG("[mtk-tpd] Query status= 0x%x\n", pucPoint[0]);
		if (!( pucPoint[0] & 0x80 || pucPoint[0] & 0x01 )){
			TPD_DEBUG("[mtk-tpd] No point information\n");
			msleep(10);
			continue;
		}
#ifdef I2C_SUPPORT_RS_DMA
        ret = tpd_i2c_read(i2c_client, &pucPoint[0], 14, 0xE0);
#else        
        //ret = tpd_i2c_read(i2c_client, &cPoint[0], 8, 0xC0);
        //ret += tpd_i2c_read(i2c_client, &ePoint[0], 6, 0xE0);
        
		tpd_i2c_write(i2c_client, Rd_P_buf, 3);
	    do
		{
			buffer[0] = 0xFF;
			tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
		} while (buffer[0] & 0x01);
		
		tpd_i2c_write(i2c_client, Rd_keep, 4);
		do
		{
			buffer[0] = 0xFF;
			tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
		} while (buffer[0] & 0x01);
		
		ret = tpd_i2c_read(i2c_client, &ePoint[0], 6, 0xA0);

		
		tpd_i2c_write(i2c_client, Rd_final, 4);
		do
		{
			buffer[0] = 0xFF;
			tpd_i2c_read(i2c_client, &buffer[0], 1, 0x80);
		} while (buffer[0] & 0x01);
		
		ret += tpd_i2c_read(i2c_client, &cPoint[0], 8, 0xA0);


        for(i=0; i<6; i++) pucPoint[i] = ePoint[i];
        for(i=0; i<8; i++) pucPoint[i+6] = cPoint[i];
#endif
/*
	printk("***********************pucPoint[0] = %d\n",pucPoint[0]);
	printk("***********************pucPoint[1] = %d\n",pucPoint[1]);
	printk("***********************pucPoint[2] = %d\n",pucPoint[2]);
*/
#ifdef I2C_SUPPORT_RS_DMA        
        if (ret == 0xF01) {
#else
        if (ret == 0xE02) {
#endif
	    // gesture
		    if (pucPoint[0] & 0xF0) {
			//printk("-----------------------------------------------pucPoint[0] & 0xF0---------------------.\n");
		    #ifdef TPD_HAVE_PHYSICAL_BUTTON
		    	//physical key
		    	if (  pucPoint[0] & 0x40  && pucPoint[0] & 0x01){
					printk("-----------------------------------------------physical key---------------------.\n");
					printk("***********************pucPoint[1] = %d\n",pucPoint[1]);
		    	    if ( pucPoint[2] ){//taikoto 20171108 add
                                    switch (pucPoint[1]) {
                                           case 1:
                                           {  
						printk("-------------taikoto20171108------------pucPoint[2]-------[HOME]----------.\n");
                                                //tpd_button(tpd_keys_dim_local[pucPoint[1]-1][0], tpd_keys_dim_local[pucPoint[1]-1][1], 1);
                                                input_report_key(kpd_input_dev, KEY_HOME, 1);
                                                input_sync(kpd_input_dev);
                                           }
                                           break;
                                           case 2:
                                           { 
                                                printk("-------------taikoto20171108------------pucPoint[2]-------[BACK]----------.\n");
                                                //tpd_button(tpd_keys_dim_local[pucPoint[1]-1][0], tpd_keys_dim_local[pucPoint[1]-1][1], 1);
                                                input_report_key(kpd_input_dev, KEY_BACK, 1);
                                                input_sync(kpd_input_dev);
		       		           }
                                           break;
                                           case 3:
                                           { 
                                                printk("-------------taikoto20171108------------pucPoint[2]-------[MENU]----------.\n");
                                                //tpd_button(tpd_keys_dim_local[pucPoint[1]-1][0], tpd_keys_dim_local[pucPoint[1]-1][1], 1);
                                                input_report_key(kpd_input_dev, KEY_BACK, 1);
                                                input_sync(kpd_input_dev);
                                           }
                                           break;
                                   }
                             }
			     else {
                                     printk("-------------taikoto20171108------------pucPoint[2]-------down----------.\n");
                                     if ( 1 == pucPoint[1] ) {
                                         input_report_key(kpd_input_dev, KEY_HOME,0);
                                     }
                                     if ( 2 == pucPoint[1] ) {
                                         input_report_key(kpd_input_dev, KEY_BACK,0);
                                     }
                                     if ( 3 == pucPoint[1] ) {
                                         input_report_key(kpd_input_dev, KEY_MENU,0);
                                     }
				     input_sync(kpd_input_dev);
                                     //taikoto 20171108 end
			    }
		    	} else
		    #endif
		    	{
		            TPD_DEBUG("[mtk-tpd] it's a button/gesture\n");
		            continue;
		        }
		    }
		    // palm
		    else if( pucPoint[1] & 0x01 ) {
		        TPD_DEBUG("[mtk-tpd] it's a palm\n");
				continue;
		    }
		    // no more data
		    else if (!(pucPoint[0] & 0x08)) {
				if( finger[0] ){
				    finger[0] = 0;
				    tpd_up(x[0], y[0], x[0], y[0], 0);
				    flag = 1;
				}
				if( finger[1] ){
				    finger[1] = 0;
				    tpd_up(x[1], y[1], x[1], y[1], 0);
				    flag = 1;
				}
				if( flag ){
				    flag = 0;
				    input_sync(tpd->dev);
				}
				TPD_DEBUG("[mtk-tpd] no more data\n");
				continue;
		    }
		    // 3 fingers
		    else if (pucPoint[0] & 0x04) {
		        TPD_DEBUG("[mtk-tpd] we don't support three fingers\n");
				continue;
		    }
			else{
				// finger 1
		        if (pucPoint[0] & 0x01) {
				    //char pressure_point;
		
				    xraw = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
				    yraw = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];
					//pressure_point = pucPoint[5] & 0x0f;
					//TPD_DEBUG("[mtk-tpd] input Read_Point1 x=%d y=%d p=%d\n",xraw,yraw,pressure_point);
					//tpd_calibrate(&xraw, &yraw);			
				    x[0] = xraw; //by FHH
				    y[0] = yraw;
				    finger[0] = 1;
					//printk("***********************x[0] = %d,y[0]=%d\n",x[0],y[0]);
				    tpd_down(x[0], y[0], x[0], y[0], 0);
				    printk("*******************tpd_down:x0=%d,y0=%d\n",x[0],y[0]);
				} 
				else if( finger[0] ){
				    tpd_up(x[0], y[0], x[0], y[0], 0);
				    finger[0] = 0;
				}
	
				// finger 2
				if (pucPoint[0] & 0x02) {
				    //char pressure_point;
				    xraw = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];
				    yraw = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];
				    //pressure_point = pucPoint[9] & 0x0f;
				    //TPD_DEBUG("[mtk-tpd] input Read_Point2 x=%d y=%d p=%d\n",xraw,yraw,pressure_point);
				   // tpd_calibrate(&xraw, &yraw);
		  		    x[1] = xraw;
				    y[1] = yraw;
				    finger[1] = 1;
				    tpd_down(x[1], y[1], x[1], y[1], 1);
				    printk("*******************tpd_down:x1=%d,y1=%d\n",x[1],y[1]);
				} else if (finger[1]){
				    tpd_up(x[1], y[1], x[1], y[1], 0);
				    finger[1] = 0;
				}
			input_sync(tpd->dev);
		    }
		}else{
		    TPD_DEBUG("[mtk-tpd] i2c read communcate error in getting pixels : 0x%x\n", ret);
		}
    } while (!kthread_should_stop()); 
    return 0;
}

int tpd_local_init(void) 
{
    int r;
    int ret;
	///TPD_DMESG("Focaltech it7259 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
		//power on, need confirm with SA
#if 1 //def CONFIG_ARCH_MT6580
		tpd->reg=regulator_get(tpd->tpd_dev, "vtouch"); // get pointer to regulator structure
		if (IS_ERR(tpd->reg)) {
			printk("taikoto : regulator_get() failed!\n");
		}

		ret=regulator_set_voltage(tpd->reg, 2800000, 2800000);	// set 2.8v
		if (ret)
			printk("regulator_set_voltage() failed!\n");
		ret=regulator_enable(tpd->reg);  //enable regulator
		if (ret)
			printk("taikoto : regulator_enable() failed!\n");
#else	
		//hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#endif
    if(i2c_add_driver(&tpd_i2c_driver)!=0) {
        printk("taikoto : unable to add i2c driver.\n");
        return -1;
    }
    
    /* register device (/dev/IT7259 CTP) */
    //ctp_dev.parent = tpd->dev;
    r = misc_register(&ctp_dev);
    if (r) {
        printk("taikoto : register ctp device failed (%d)\n", r);
        return -1;
    }
   


#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
    printk("end %s, %d\n", __FUNCTION__, __LINE__);  

    tpd_type_cap = 1;
    
    return 0;
}

/* Function to manage low power suspend */
 static void tpd_suspend(struct device *h)
{
//    int ret = 0;
//    unsigned char Wrbuf[4] = { 0x20, 0x04, 0x00, 0x02 };
    
 //   tpd_halt = 1;
 //   mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

    TPD_DEBUG("taikoto : IT7259 call suspend\n");    

   // mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    disable_irq(touch_irq);
    tpd_halt = 1;
    tpd_gpio_output(GTP_RST_PORT, 0); 
    msleep(100);
/* //by FHH
    TPD_DEBUG("IT7259 call suspend\n");    
    ret = tpd_i2c_write(i2c_client, Wrbuf, 4);   
    msleep(100);
  
    if(ret != 4){
        TPD_DEBUG("[mtk-tpd] i2c write communcate error during suspend: 0x%x\n", ret);
    }
	msleep(200);
*/
	//hwPowerDown(MT65XX_POWER_LDO_VGP2,  "TP");
}

/* Function to manage power-on resume */
 static void tpd_resume(struct device *h)
{       
    #define TRY_COUNTS 5
	int i=0;
    int ret = 0;
    unsigned char Wrbuf[2] = { 0x20, 0x6F};
    char gsbuffer[2];
#if 0 // 1 //by FHH
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO); 
	msleep(100);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE );
#else
	tpd_gpio_output(GTP_RST_PORT, 1); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 0);
	msleep(100);
	tpd_gpio_output(GTP_RST_PORT, 0); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 1);
	msleep(200);
	tpd_gpio_output(GTP_RST_PORT, 1); //ugrec_tky gpio_direction_output(tpd_rst_gpio_number, 1);
#endif
	for(i=0;i<TRY_COUNTS;i++)
	{
       tpd_i2c_read(i2c_client, gsbuffer, 1, 0x80);
	   msleep(10);
	}
	ret=tpd_i2c_read(i2c_client, gsbuffer, 2, 0xA0);
	msleep(10);
    ret = tpd_i2c_write(i2c_client, Wrbuf, 2);
	
    msleep(300);
	///mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
	enable_irq(touch_irq);
    tpd_halt = 0;
    // liuzhiyong 20120619 update code ++ 
	tpd_up(0, 0, 0, 0, 0);
	tpd_up(0, 0, 0, 0, 0);
	input_sync(tpd->dev);
}
		
static struct tpd_driver_t tpd_device_driver = {
	//.tpd_device_name = "IT7259",
	.tpd_device_name =CTP_NAME,  // TPD_DEVICE,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif 
};

/*
static bool waitCommandDone(void)
{
	unsigned char ucQuery = 0xFF;
	unsigned int count = 0;

	do{
		ucQuery = 0xFF;
		tpd_i2c_read(i2c_client, &ucQuery, 1, 0x80);
		count++;
	}while(ucQuery & 0x01 && count < 500);

	if( !(ucQuery & 0x01) ){
	        return  true;
	}else{
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return  false;
	}
}
*/

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	tpd_get_dts_info();
    printk("taikoto : MediaTek IT7259 touch panel driver init\n");
#if defined(TPD_I2C_NUMBER)	//lshun modify 20130615
   // i2c_register_board_info(TPD_I2C_NUMBER, &it7259_i2c_tpd, 1);
#else
//	i2c_register_board_info(0, &it7259_i2c_tpd, 0);
#endif
	if(tpd_driver_add(&tpd_device_driver) < 0)
		printk("add generic driver failed\n");
       
    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    printk("taikoto : MediaTek IT7259 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}


module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

