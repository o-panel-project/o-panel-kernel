#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include <asm/prom.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/cdev.h>
#include <linux/firmware.h>
#include <asm/uaccess.h>
#include "ft5x06_reg.h"
#include "ft5x06.h"

#define FT5X06_NAME             "ft5x06"
#define VALUE_XY(h, l)          ( (h & 0x0f) << 8 | l)
#define VALUE_ID(yh)            ( yh >> 4 )
#define VALUE_TYPE(xh)          ( xh >> 6 )
#define TOUCH_NUMBER(status)    ( status & 0xf )
#define VALUE_AREA(misc)        ( misc >> 4 )
#define VALUE_DIRECTION(misc)   ( (misc & 0x0c) >> 2 )
#define VALUE_SPEED(misc)       ( misc & 0x03 )
#define TIME_OF_DISCARD         ( msecs_to_jiffies(2000))  //jiffes
#define PROTOCOL_B
enum FT5X06_MODE {
    INVALID_MODE ,
    OPER_MODE ,
    SYS_MODE ,
    TEST_MODE ,
};

enum FT_EVENT_TYPE {
    PRESS_EVENT ,
    RELEASE_EVENT ,
    CONTACT_EVENT ,
    NONE_EVENT,
};

enum FT_TOUCH_STATE {
	TOUCH_NOUSE = -1,
	TOUCH_UP = 0,
	TOUCH_DOWN,
	TOUCH_MOVE,
};

#define printlf() printk("%s   %d\n",__FUNCTION__,__LINE__)
//#define printlf()

//#if defined(LOCAL_GL5202_EVB)
//#define HAS_VIRTUAL_KEY
//#endif


struct ft5x06_point {
    enum FT_EVENT_TYPE  type;
    unsigned int weight;
    unsigned int x;
    unsigned int y;
    unsigned int id;
	enum FT_TOUCH_STATE state;
	unsigned int area;
	unsigned int direction;
	unsigned int speed;
};

struct ft5x06_data {
    struct ft5x06_point points[FT5X06_MAX_POINT];
    u8 rawData[POINT_DATA_LEN*FT5X06_MAX_POINT + 1];
    int validPointCnt;
	int lastValidCnt;
    struct ft5x06_point touches[FT5X06_MAX_POINT];
};

#if CFG_FT_USE_CONFIG
struct ft5x06_cfg_dts {
	unsigned int sirq;
	unsigned int i2cNum;
	unsigned int i2cAddr;
	unsigned int xMax;
	unsigned int yMax;
	unsigned int rotate; 
	unsigned int xRevert;
	unsigned int yRevert;
	unsigned int XYSwap;
	char const *regulator;
	unsigned int vol_max;
	unsigned int vol_min;
};

static struct ft5x06_cfg_dts cfg_dts;
#endif
struct ft5x06_config {
    enum FT5X06_MODE mode;
    struct ft5x06_point max;
    u16 vendor;
    u16 version;
    u16 product;
};

struct ft5x06_device {
    struct input_dev *input_dev;
    struct i2c_client *client;
    struct atc260x_dev *atc260x;
    struct ft5x06_data *ftdata;
    struct ft5x06_config *ftconfig;
    struct work_struct work;
    struct workqueue_struct *workqueue;
	struct regulator *regulator;
    struct mutex lock;
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend es;
#endif
    int irq;
    unsigned long time_discard;
	/* for fw loader */
	struct class *cdev_class;
	struct cdev ft5x06_cdev;
	dev_t ft5x06_dev;
	struct firmware *fw;
};

int debug_switch = 0;
static char ctp_power_name[] = FT5X06_POWER_ID;
static struct regulator *tp_regulator = NULL;
static int ft5x06_reset( struct ft5x06_device *);
static inline void regulator_deinit(struct regulator *);
static struct regulator *regulator_init(const char *, int, int);
static inline void disable_power(struct regulator *);
//volatile int enirq = 1;
volatile int current_val = 0;
//struct regulator *gpower = NULL;
//static struct i2c_client * pClient = NULL;

static unsigned gpio_reset = FT5X06_RESET_PIN;
static unsigned reset_waitms = 50;

/*
  * I2C funcionts:
  * @ ft5x06_get_regs/ft5x06_get_reg
  * @ ft5x06_set_regs/ft5x06_set_reg
  *
  * Sucessed, 0 returned. Failed, 1 returnd
  */
static int ft5x06_get_regs(struct i2c_client *client, 
    u8 addr, u8 *buff, int count)
{
    int err = 0;
    struct i2c_msg msg[2];

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &addr;
    msg[0].len = 1;
    
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = buff;
    msg[1].len = count;
    

    err = i2c_transfer(client->adapter, msg, 2);

    if ( 2 != err ) {
        FT5X06_WARNNING("I2C read failed");
        goto out;
    }
   
out:
    return (( 2 == err )?0:-EFAULT);
}

static int ft5x06_get_reg(struct i2c_client *client,
    u8 addr, u8 *buff)
{
    return ft5x06_get_regs(client, addr, buff, 1);
}

static int ft5x06_set_reg(struct i2c_client *client, 
    u8 addr, u8 *data)
{
    int err = 0;

    struct i2c_msg msg[1];
    char b[] = { addr, *data};

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = b;
    msg[0].len = ARRAY_SIZE(b);

    err = i2c_transfer(client->adapter, msg, 1);
    if ( 1 != err ) {
        FT5X06_WARNNING("I2C write failed");
    }    

    return ( (1 != err)?-EFAULT:0);
}

/*
  * Using this function carefully. Regs address should the firest of data, 
  * to avoid copy when writing datas.
  */
static int ft5x06_set_regs(struct i2c_client *client, u8 addr,
    u8 *data, int len)
{
    struct i2c_msg msg[1];
    int err = 0;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = data;
    msg[0].len = len;

    err = i2c_transfer(client->adapter, msg, 1);
    if ( 1 != err ) {
        FT5X06_WARNNING("I2C write failed(multi)");
    }

    return ( ( 1 != err )?-EFAULT:0);
}


FTS_BOOL i2c_write_interface(struct i2c_client *client,FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

FTS_BOOL i2c_read_interface(struct i2c_client *client,FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[FTS]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

FTS_BOOL byte_write(struct i2c_client *client,FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(client,I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

FTS_BOOL byte_read(struct i2c_client *client,FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(client,I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

FTS_BOOL cmd_write(struct i2c_client *client,FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(client,I2C_CTPM_ADDRESS, write_cmd, num);
}

#if FT5X0X_DOWNLOAD_FIRM
/*down_load fw code add by tuhm*/
#define	MACRO_STRING(MACRO) #MACRO
#define FT_APP_I(INAME) MACRO_STRING(INAME) 
static unsigned char CTPM_FW[]=
{
    #include FT_APP_I(FIRM_I_FILE_NAME)
};

static unsigned char ft5x0x_read_fw_ver(struct i2c_client *client)
{
	unsigned char ver;
	ft5x06_get_reg(client, FT5X0X_REG_FIRMID, &ver);
	return(ver);
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

static int ft5x0x_write_reg(struct i2c_client *client,u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x06_set_regs(client,0,buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}

int fts_ctpm_auto_clb(struct i2c_client *client)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
    ft5x0x_write_reg(client,0, 0x40);  
    delay_qt_ms(100);   //make sure already enter factory mode
    ft5x0x_write_reg(client,2, 0x4);  //write command to start calibration
    delay_qt_ms(300);
    for(i=0;i<100;i++)
    {
        ft5x06_get_reg(client,0,&uc_temp);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        delay_qt_ms(200);
        //printk("[FTS] waiting calibration %d\n",i);
        
    }
    if (i >= 100)
        printk("[FTS] calibration OK?\n");
    else
        printk("[FTS] calibration OK.(%d)\n", i);
    
    msleep(300);
    ft5x0x_write_reg(client,0, 0x40);  //goto factory mode
    delay_qt_ms(100);   //make sure already enter factory mode
    ft5x0x_write_reg(client,2, 0x5);  //store CLB result
    delay_qt_ms(300);
    ft5x0x_write_reg(client,0, 0x0); //return to normal mode 
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

#define    FTS_PACKET_LENGTH        64//128

int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
        int ret = 0;

        struct i2c_msg msgs[] = {
                {
                         .addr = client->addr,
                         .flags = 0,
                         .len = writelen,
                         .buf = writebuf,
                 },
        };

        if(writelen > 0)
        {
                ret = i2c_transfer(client->adapter, msgs, 1);
                if (ret < 0)
                        dev_err(&client->dev, "%s: i2c write error.\n", __func__);
        }


        return ret;
}


int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
        int ret = 0;


        if(readlen > 0)
        {
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
                                dev_err(&client->dev, "%s: i2c read error.\n", __func__);
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
        }


        return ret;
}


#define HIDTOI2C_DISABLE					0
int hidi2c_to_stdi2c(struct i2c_client * client)
{
        u8 auc_i2c_write_buf[5] = {0};
        int bRet = 0;
        #if HIDTOI2C_DISABLE    
                return 0;
        #endif

        auc_i2c_write_buf[0] = 0xeb;
        auc_i2c_write_buf[1] = 0xaa;
        auc_i2c_write_buf[2] = 0x09;
        bRet =fts_i2c_write(client, auc_i2c_write_buf, 3);
        msleep(10);
        auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;
        fts_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

        if(0xeb==auc_i2c_write_buf[0] && 0xaa==auc_i2c_write_buf[1] && 0x08==auc_i2c_write_buf[2])
        {
                pr_info("hidi2c_to_stdi2c successful.\n");
                bRet = 1;
        }
        else
        {
                pr_err("hidi2c_to_stdi2c error.\n");
                bRet = 0;
        }

        return bRet;
}




E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(struct i2c_client *client,FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;
	
        i_ret = hidi2c_to_stdi2c(client);
        if(i_ret == 0)
        {
                printk("[FTS] hid change to i2c fail ! \n");
        }



    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    ft5x0x_write_reg(client,0xfc,0xaa);
    delay_qt_ms(2);
     /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(client,0xfc,0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");
   
    delay_qt_ms(200);   


    /*********Step 2:Enter upgrade mode *****/
    i_ret = hidi2c_to_stdi2c(client);

    if(i_ret == 0)
    {
            printk("[FTS] hid change to i2c fail ! \n");
    }
    msleep(10);

    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x06_set_regs(client,0,auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
    cmd_write(client,0x90,0x00,0x00,0x00,4);
    byte_read(client,reg_val,2);
    printk("config use upgrad id2=0x%x\n",UPGRADE_ID2);
    if (reg_val[0] == 0x54 && reg_val[1] == 0x2c)
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
        printk("[FTS] ERR_READID");
        
        return ERR_READID;
        //i_is_new_protocol = 1;
    }


    cmd_write(client,0xcd,0x0,0x00,0x00,1);
    byte_read(client,reg_val,1);
    printk("[FTS] bootloader version = 0x%x\n", reg_val[0]);

	/*Step 4:erase app and panel paramenter area*/
	printk("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1350);
	for(i = 0;i < 15;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if(0xF0==reg_val[0] && 0xAA==reg_val[1])
		{
			break;
		}
		msleep(50);		
	}
	printk("[FTS][%s] erase app area reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	fts_i2c_write(client, auc_i2c_write_buf, 4);
	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xBF;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) 
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
	{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			printk("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
			msleep(1);			
		}
	}
	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) 
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) 
	{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}	
		fts_i2c_write(client, packet_buf, temp + 6);
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			printk("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			printk("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
			msleep(1);
			
		}
	}

	msleep(50);
	
	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	printk("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1); 
	msleep(300);

	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6); 
	msleep(dw_lenth/256);

	for(i = 0;i < 100;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		dev_err(&client->dev, "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
		if (0xF0==reg_val[0] && 0x55==reg_val[1])
		{
			dev_err(&client->dev, "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
			break;
		}
		msleep(1);
			
	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
				
		return -EIO;
	}
	printk(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);	
	/*********Step 7: reset the new FW***********************/
	printk("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);	
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0) 
	{
		printk("HidI2c change to StdI2c fail ! \n");
	}

    	return ERR_OK;
}


int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
    
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(client,pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
       //error handling ...
       //TBD
   }
   else
   {
       printk("[FTS] upgrade successfully.\n");
       fts_ctpm_auto_clb(client);  //start auto CLB
       fts_ctpm_auto_clb(client);  //start auto CLB
   }

   return i_ret;
}



int fts_ctpm_auto_upg(struct i2c_client *client)
{
    unsigned char uc_host_fm_ver;
    unsigned char uc_tp_fm_ver;
    int           i_ret;

    uc_tp_fm_ver = ft5x0x_read_fw_ver(client);
    //printk("***uc_tp_fm_ver=%x\n",uc_tp_fm_ver);
    //uc_tp_fm_ver -=1;
    
    uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	
    printk("[FTS]uc_tp_fm_ver=%x uc_host_fm_ver=%x\n",uc_tp_fm_ver,uc_host_fm_ver);
    if(uc_host_fm_ver== uc_tp_fm_ver){
    	printk("[FTS]version %d ,don't need upgrade \n",uc_tp_fm_ver);
	return 0;
    }
#if 0
    if ( uc_tp_fm_ver == 0xa6  ||   //the firmware in touch panel maybe corrupted
         uc_tp_fm_ver < uc_host_fm_ver //the firmware in host flash is new, need upgrade
        )
#endif

    {
        msleep(100);
        printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
            uc_tp_fm_ver, uc_host_fm_ver);
        i_ret = fts_ctpm_fw_upgrade_with_i_file(client);    
        if (i_ret == 0)
        {
            msleep(300);
            uc_host_fm_ver = fts_ctpm_get_i_file_ver();
            printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
        }
        else
        {
            printk("[FTS] upgrade failed ret=%d.\n", i_ret);
        }
    }

    return 0;
}

int fts_ctpm_fw_upgrade_with_fwfile(struct i2c_client *client)
{
	struct ft5x06_device *ftdev = i2c_get_clientdata(client);
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret;

	//=========FW upgrade========================*/
	pbt_buf = (FTS_BYTE *)ftdev->fw->data;
#if 0 /* for debug */
	for (i_ret = 0; i_ret < ftdev->fw->size; i_ret++) {
		if (pbt_buf[i_ret] == CTPM_FW[i_ret])
			continue;
		dev_info(&client->dev, "fw data diff %d [%02x]-[%02x]\n",
				i_ret, pbt_buf[i_ret], CTPM_FW[i_ret]);
		break;
	}
	if (i_ret == ftdev->fw->size)
		dev_info(&client->dev, "fw data compare ok.\n");
#endif
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(client, pbt_buf, ftdev->fw->size);
	if (i_ret != 0) {
		dev_err(&client->dev, "fw upgrade failed (%d)\n", i_ret);
		//error handling ...
		//TBD
	} else {
		dev_info(&client->dev, "fw upgrade successfully\n");
		fts_ctpm_auto_clb(client);  //start auto CLB
		fts_ctpm_auto_clb(client);  //start auto CLB
	}

	return i_ret;
}

unsigned char fts_ctpm_get_fwfile_ver(struct i2c_client *client)
{
	struct ft5x06_device *ftdev = i2c_get_clientdata(client);
	unsigned int ui_ver_pos;

	if (ftdev->fw->data && ftdev->fw->size >= 2) {
		ui_ver_pos = ftdev->fw->size - 2;
		return ftdev->fw->data[ui_ver_pos];
	}
	//TBD, error handling?
	return 0xff; //default value
}

int fts_ctpm_manual_upg(struct i2c_client *client, int force)
{
	struct ft5x06_device *ftdev = i2c_get_clientdata(client);
	unsigned char uc_host_fm_ver;
	unsigned char uc_tp_fm_ver;
	int           i_ret;

	if (!ftdev->fw->data) {
		dev_err(&client->dev, "fw data not loaded\n");
		return 0;
	}
	uc_tp_fm_ver = ft5x0x_read_fw_ver(client);
	uc_host_fm_ver = fts_ctpm_get_fwfile_ver(client);
	dev_info(&client->dev,
			"chip fw version=0x%02x, upgrade fw version=0x%02x\n",
			uc_tp_fm_ver, uc_host_fm_ver);
	if ((uc_host_fm_ver == uc_tp_fm_ver) && (force == 0)) {
		dev_info(&client->dev,
				"fw version=%x, don't need upgrade\n", uc_tp_fm_ver);
		return 0;
	}

	msleep(100);
	i_ret = fts_ctpm_fw_upgrade_with_fwfile(client);
	if (i_ret == 0) {
		msleep(300);
		uc_tp_fm_ver = ft5x0x_read_fw_ver(client);
		//uc_host_fm_ver = fts_ctpm_get_fwfile_ver(client);
		dev_info(&client->dev, "fw upgrade to new version 0x%02x\n",
				uc_tp_fm_ver);
	} else {
       dev_err(&client->dev, "fw upgrade failed ret=%d\n", i_ret);
	}

	return 0;
}

/*
 * WJ1307_5526_1280x800_V09_D01_20170502_app.i force load
 */
int fts_ctpm_v09_load(struct i2c_client *client)
{
	struct ft5x06_device *ftdev = i2c_get_clientdata(client);
	unsigned char uc_09_fm_ver;
	unsigned char uc_tp_fm_ver;
	int i_ret;

	if (!ftdev->fw->data) {
		kfree(ftdev->fw->data);
		ftdev->fw->size = 0;
		ftdev->fw->data = NULL;
	}
	uc_09_fm_ver = fts_ctpm_get_i_file_ver();

	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0) {
		msleep(300);
		uc_tp_fm_ver = ft5x0x_read_fw_ver(client);
		if (uc_09_fm_ver == uc_tp_fm_ver) {
			dev_info(&client->dev, "fw V09 load successfully\n");
			return 0;
		}
		dev_info(&client->dev, "fw V09 load failed? now version 0x%02x\n",
				uc_tp_fm_ver);
		i_ret = -1;
	} else {
		dev_err(&client->dev, "fw V09 load failed ret=%d\n", i_ret);
	}

	return i_ret;
}
#endif



/**/

/*
 * /dev/ft5x06 firmware loader ioctl
 */
enum {
	FT5X06_IOCTL_FW_UPDATE = 0xA1,
	FT5X06_IOCTL_FW_UPDATE_FORCE,
	FT5X06_IOCTL_FW_DEFAULT_LOAD,
	FT5X06_IOCTL_GET_VERSION,
};
#define IOCTL_FW_UPDATE     _IO('F', FT5X06_IOCTL_FW_UPDATE)
#define IOCTL_FW_FUPDATE    _IO('F', FT5X06_IOCTL_FW_UPDATE_FORCE)
#define IOCTL_FW_DEFAULT    _IO('F', FT5X06_IOCTL_FW_DEFAULT_LOAD)
#define IOCTL_GET_VERSION   _IO('F', FT5X06_IOCTL_GET_VERSION)

static int ft5x06_fs_open(struct inode *node, struct file *fp)
{
	struct ft5x06_device *ftdev;
	struct i2c_client *client;

	ftdev = container_of(node->i_cdev, struct ft5x06_device, ft5x06_cdev);
	client = ftdev->client;

	fp->private_data = ftdev;

	dev_info(&client->dev, "/dev/ft5x06 opened\n");
	return 0;
}

static int ft5x06_fs_release(struct inode *node, struct file *fp)
{
	struct ft5x06_device *ftdev = fp->private_data;
	struct i2c_client *client = ftdev->client;

	if (ftdev->fw->data) {
		kfree(ftdev->fw->data);
		ftdev->fw->size = 0;
		ftdev->fw->data = NULL;
	}
	dev_info(&client->dev, "/dev/ft5x06 closed\n");
	return 0;
}

static ssize_t ft5x06_fs_read(
		struct file *fp, char *rbuf, size_t cnt, loff_t *fpos)
{
	struct ft5x06_device *ftdev = fp->private_data;
	struct i2c_client *client = ftdev->client;

	dev_info(&client->dev, "/dev/ft5x06 read, nothing to do\n");
	return cnt;
}

static ssize_t ft5x06_fs_write(
		struct file *fp, const char __user *wbuf, size_t cnt, loff_t *fpos)
{
	struct ft5x06_device *ftdev = fp->private_data;
	struct i2c_client *client = ftdev->client;

	if (cnt <= 0) {
		dev_err(&client->dev, "/dev/ft5x06 write, count error\n");
		return -EINVAL;
	}
	ftdev->fw->size = cnt;
	ftdev->fw->data = kmalloc(cnt, GFP_KERNEL);
	if (!ftdev->fw->data) {
		ftdev->fw->size = 0;
		dev_err(&client->dev, "/dev/ft5x06 write, buffer error\n");
		return -ENOMEM;
	}
	if (copy_from_user((void *)ftdev->fw->data, wbuf, cnt)) {
		kfree(ftdev->fw->data);
		ftdev->fw->size = 0;
		ftdev->fw->data = NULL;
		dev_err(&client->dev, "/dev/ft5x06 write, write error\n");
		return -EFAULT;
	}
	dev_info(&client->dev,
			"/dev/ft5x06 write, %d bytes [%02d][%02d][%02d]...\n",
			cnt, ftdev->fw->data[0], ftdev->fw->data[1], ftdev->fw->data[2]);
	return cnt;
}

static long ft5x06_fs_ioctl(struct file *fp, unsigned cmd, unsigned long arg)
{
	struct ft5x06_device *ftdev = fp->private_data;
	struct i2c_client *client = ftdev->client;
	int force_upd = 0;
	long ret = 0;

	switch (cmd) {
	case IOCTL_GET_VERSION:
		dev_info(&client->dev, "fw get version cmd\n");
		ret = ft5x0x_read_fw_ver(client);
		dev_info(&client->dev, "tp fw version 0x%ld\n", ret);
		break;
	case IOCTL_FW_FUPDATE:
		force_upd = 1;
		/* fall through */
	case IOCTL_FW_UPDATE:
		dev_info(&client->dev, "fw %supdate cmd\n", force_upd ? "force " : "");
		ret = fts_ctpm_manual_upg(client, force_upd);
		break;
	case IOCTL_FW_DEFAULT:
		dev_info(&client->dev, "fw V09 force loader cmd\n");
		ret = fts_ctpm_v09_load(client);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct file_operations ft5x06_fops = {
	.open           = ft5x06_fs_open,
	.release        = ft5x06_fs_release,
	.read           = ft5x06_fs_read,
	.write          = ft5x06_fs_write,
	.unlocked_ioctl = ft5x06_fs_ioctl,
};

static int ft5x06_chardev_setup(struct ft5x06_device *ftdev)
{
	struct i2c_client *client = ftdev->client;

	/* file i/o interface */
	if (alloc_chrdev_region(&ftdev->ft5x06_dev, 0, 1, FT5X06_NAME)) {
		dev_err(&client->dev, "failed to allocated device region\n");
		return -ENOMEM;
	}
	cdev_init(&ftdev->ft5x06_cdev, &ft5x06_fops);
	ftdev->ft5x06_cdev.owner = THIS_MODULE;
	if (cdev_add(&ftdev->ft5x06_cdev, ftdev->ft5x06_dev, 1)) {
		dev_err(&client->dev, "failed to add char device\n");
		return -EIO;
	}

	ftdev->cdev_class = class_create(THIS_MODULE, FT5X06_NAME);
	device_create(
			ftdev->cdev_class, NULL, ftdev->ft5x06_dev, NULL, FT5X06_NAME);

	if (!ftdev->fw) {
		ftdev->fw = kmalloc(sizeof(struct firmware), GFP_KERNEL);
		if (!ftdev->fw) {
			dev_err(&client->dev, "failed to alloc fw\n");
			return -ENOMEM;
		}
	}
	ftdev->fw->size = 0;
	ftdev->fw->data = NULL;
	dev_info(&client->dev, "%s done(app_i=%d)\n", __func__, sizeof(CTPM_FW));

	return 0;
}

static void ft5x06_chardev_remove(struct ft5x06_device *ftdev)
{
	struct i2c_client *client = ftdev->client;

	device_destroy(ftdev->cdev_class, ftdev->ft5x06_dev);
	class_destroy(ftdev->cdev_class);
	cdev_del(&ftdev->ft5x06_cdev);
	unregister_chrdev_region(ftdev->ft5x06_dev, 1);
	if (ftdev->fw) {
		kfree(ftdev->fw);
		ftdev->fw = 0;
	}
	dev_info(&client->dev, "%s done\n", __func__);
}

/*
  *  device attributes. Commnication between kernel & user space
  *  Informations exported:
  *    1. work mode
  *    2. ft5x06 device information
  *    3. debug information switch
  *    4. Scan rate.
  */
static ssize_t mode_show(struct device *dev, 
        struct device_attribute *attr,char *buf)
{
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)dev_get_drvdata(dev);
    int ret = 0, count = 0;
    u8 mode;

    ret = ft5x06_get_reg(ftdevice->client, 0x0, &mode);

    count = sprintf(buf, "Current mode is 0x%x\n", mode);
    
    count += sprintf(buf+count, "0: operation, 1: system, 4: test\n");
    
    return count;
}

static ssize_t mode_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    int data = -1;
    u8 mode, mode_t;
    
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)dev_get_drvdata(dev);

    ret = sscanf(buf, "%d", &data);
    mode = data & 0xff;
    switch( mode ) {
        case 0:
        case 1:
        case 4:
            mode_t = mode << 4;
            ft5x06_set_reg(ftdevice->client, 0x0, &mode_t);
            break;
        default:
            FT5X06_WARNNING("Invalid mode");
            break;
    }
    return count;
}

static ssize_t debug_show(struct device *dev, 
        struct device_attribute *attr,char *buf)
{
    int cnt;
    
    cnt = sprintf(buf, "%d\n(Note: 1: open, 0:close)\n", debug_switch);
    return cnt;
}

static ssize_t debug_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    int cnt, tmp;
    cnt = sscanf(buf, "%d", &tmp);
    switch(tmp) {
        case 0:
        case 1:
        case 2:
            debug_switch = tmp;
            break;
        default:
            printk("invalid input\n");
            break;
    }
    return count;
}   

static ssize_t info_show(struct device *dev, 
        struct device_attribute *attr,char *buf)
{
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)dev_get_drvdata(dev);
    u8 firm_version;//, vender_id;
    int ret;

    ret = ft5x06_get_reg(ftdevice->client, OP_IDG_FIRMID, &firm_version);
    if ( ret ) {
        FT5X06_WARNNING("Get firmware id failed");
        goto err;
    }
/*

    ret = ft5x06_get_reg(ftdevice->client, OP_IDG_FT5201ID, &vender_id);
    if ( ret ) {
        FT5X06_WARNNING("get ft5201 id failed");
        goto err;
    }

*/
    ret = sprintf(buf, "%d\n", (int)firm_version);
    return ret;

err:    
    return 0;    
}

static ssize_t info_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static ssize_t rate_show(struct device *dev, 
        struct device_attribute *attr,char *buf)
{
    return 0;
}

static ssize_t rate_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{    
    
    return count;
}

static ssize_t reset_show(struct device *dev, 
        struct device_attribute *attr,char *buf)
{
    return sprintf(buf, "1: reset\n");
}

static ssize_t reset_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    int data;
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)dev_get_drvdata(dev);

    sscanf(buf, "%d", &data);
    if ( data == 1 ) {
        gpio_set_value_cansleep(gpio_reset, 0);
        msleep(reset_waitms);
        gpio_set_value_cansleep(gpio_reset, 1);

        //ft5x06_reset(ftdevice);
    }

    return count;
}

static ssize_t reset_waitms_show(struct device *dev,
        struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d ms\n", reset_waitms);
}

static ssize_t reset_waitms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned data;
	int cnt;

	cnt = sscanf(buf, "%d", &data);
	if (cnt > 0) {
		reset_waitms = data;
	}

	return count;
}

static ssize_t str_show(struct device *dev, 
        struct device_attribute *attr,char *buf)
{
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)dev_get_drvdata(dev);
    u8 power = FT5X06_HIBERNATE_MODE;
    int ret;

    ret = ft5x06_set_reg(ftdevice->client, OP_IDG_PMODE, &power);
    if ( ret ) {
        FT5X06_DEBUG("Set power mode failed");
    }

    return 0;
}

static ssize_t str_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)dev_get_drvdata(dev);
    u8 power = FT5X06_ACTIVE_MODE;
    int ret;

    ret = ft5x06_set_reg(ftdevice->client, OP_IDG_PMODE, &power);
    if ( ret ) {
        FT5X06_DEBUG("Set power mode failed");
        ft5x06_reset(ftdevice);
    }

    return count;
}

static ssize_t reset_check_show(struct device *dev, 
        struct device_attribute *attr,char *buf)
{
    return 0;
}

static ssize_t reset_check_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)dev_get_drvdata(dev);
    int ret;
    u8 err_code = 0;

    ret = ft5x06_get_reg(ftdevice->client, OP_IDG_PMODE, &err_code);
    if ( ret || (err_code == 0x03 || err_code == 0x05 || err_code == 0x1a )) {
        ft5x06_reset(ftdevice);
    }

    return count;
}

static ssize_t reset_nocheck_show(struct device *dev, 
        struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t reset_nocheck_store(struct device *dev, 
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)dev_get_drvdata(dev);
    ft5x06_reset(ftdevice);

    return count;
}

static int reg2read = 0;
static ssize_t set_reg_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    struct ft5x06_device *pdev = (struct ft5x06_device *)dev_get_drvdata(dev);
    u8 data;
    int err = 0, count = 0;

    err = ft5x06_get_reg(pdev->client, reg2read & 0xff, &data);
    if ( err ) {
        count = sprintf(buf, "Dump regs failed\n");
        goto out;
    }

    count = sprintf(buf, "reg-%d: 0x%x\n", reg2read, data);
    
out:    
    return count;
}

static ssize_t set_reg_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
    int cnt;
    int value;
    u8 data;
    struct ft5x06_device *pdev = (struct ft5x06_device *)dev_get_drvdata(dev);    

    cnt = sscanf(buf, "%d:0x%x", &reg2read, &value);
    if ( cnt <= 0 ) {
        reg2read = 0x0;
        goto out;
    }

    data = value & 0xff;
    ft5x06_set_reg(pdev->client, reg2read & 0xff, &data);

out:    
    return count;
}

static int regs_addr = 0x0;
static int reg_count = 10;
static ssize_t dump_regs_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    struct ft5x06_device *pdev = (struct ft5x06_device *)dev_get_drvdata(dev);
    u8 data[reg_count];
    int err = 0, count = 0, i;

    err = ft5x06_get_regs(pdev->client, regs_addr & 0xff, data, reg_count);
    if ( err ) {
        count = sprintf(buf, "Dump regs failed\n");
        goto out;
    }

    for( i = 0; i < reg_count; i++) {
        count += sprintf(buf + count, "reg-%d: 0x%x\n", regs_addr + i, data[i]);
    }
    
out:    
    return count;
}

static ssize_t dump_regs_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
    int cnt;

    cnt = sscanf(buf, "%d:%d", &regs_addr, &reg_count);
    if ( cnt <= 0 ) {
        regs_addr = 0x0;
        reg_count = 10;
    }
    
    return count;
}

static struct device_attribute ft5x06_attr[] = {
    __ATTR(mode, S_IRUSR | S_IWUSR, mode_show, mode_store),
    __ATTR(debug, S_IRUSR | S_IWUSR, debug_show, debug_store),
    __ATTR(firmware_version, S_IRUSR | S_IWUSR, info_show, info_store), //  /sys/devices/b0174000.i2c/i2c-1/1-0038/firmware_version
    __ATTR(scan_rate, S_IRUSR | S_IWUSR, rate_show, rate_store),
    __ATTR(reset, S_IRUSR | S_IWUSR, reset_show, reset_store),
	__ATTR(reset_waitms, S_IRUSR | S_IWUSR,
			reset_waitms_show, reset_waitms_store),
    __ATTR(STR, S_IRUSR | S_IWUSR, str_show, str_store),
    __ATTR(rest_check, S_IRUSR | S_IWUSR, reset_check_show, reset_check_store),
    __ATTR(rest_nocheck, S_IRUSR | S_IWUSR, reset_nocheck_show, reset_nocheck_store),
    __ATTR( set_reg, S_IRUSR | S_IWUSR, set_reg_show, set_reg_store),
    __ATTR( dump_regs,  S_IRUSR | S_IWUSR,  dump_regs_show, dump_regs_store),
};

static inline int ft5x06_valid_cnt(struct ft5x06_data *data)
{
    u8 count = 0;

    if ( !data )
        goto out;

    count = data->rawData[0] & 0xf;
out:
    return (( count > 0 && count <= FT5X06_MAX_POINT )?count:0);
}

static inline int id2code(int id)
{
    return -EEXIST;
}

static inline int position2key(unsigned int x, 
    unsigned int y)
{
    return -1;
}

static int ft5x06_reset( struct ft5x06_device *ftdev)
{
	int err = 0;

	regulator_deinit(tp_regulator);
	tp_regulator = regulator_init(FT5X06_POWER_ID, 
		FT5X06_POWER_MIN_VOL, FT5X06_POWER_MAX_VOL);
	if ( !tp_regulator ) {
		FT5X06_WARNNING("Ft5x06 init power failed");
		err = -EINVAL;
		goto out;
	}
#if 0	
    gpio_set_value_cansleep(gpio_reset, 0);
	msleep(10);
	gpio_set_value_cansleep(gpio_reset, 1);
    msleep(30);
#endif	
out:
	
    return err;
}

static int ft5x06_hw_init(void)
{
	int err = 0;
#if 1
	struct device_node *dts;
	int gpio;
	enum of_gpio_flags flags;
	printlf();
	dts = of_find_node_by_name(NULL, "wpcio-gpio");
	if (!dts) {
		pr_err("%s: Could not find wpcio gpio node\n", __func__);
	} else {
		gpio = of_get_named_gpio_flags(dts, "tp_pwd_rst-gpios", 0, &flags);
		if (gpio_is_valid(gpio)) {
			gpio_reset = gpio;
			pr_info("%s: tp_pwd_rst-gpios=%d\n", __func__, gpio);
		}
		of_node_put(dts);
	}
    err = gpio_request(gpio_reset, FT5X06_NAME);
    if ( err ) {
		err = -EINVAL;
        goto out;
	}

    err = gpio_direction_output(gpio_reset, flags & OF_GPIO_ACTIVE_LOW);
    if ( err ) {
		err = -EINVAL;
        goto out;
	}
#endif 

#ifdef GPIO_TO_IRQ
    err = gpio_request(FT5X06_IRQ_GPIO, FT5x06_IRQ_NAME);
    if ( err ) {
		err = -EINVAL;
        goto out;
	}

    err = gpio_direction_input(FT5X06_IRQ_GPIO);
    if ( err ) {
		err = -EINVAL;
        goto out;
	}
#endif
	tp_regulator = regulator_init(FT5X06_POWER_ID, 
		FT5X06_POWER_MIN_VOL, FT5X06_POWER_MAX_VOL);
	if ( !tp_regulator ) {
		FT5X06_WARNNING("Ft5x06 init power failed");
		err = -EINVAL;
		goto out;
	}
#if 1	
    gpio_set_value_cansleep(gpio_reset, 0);
	msleep(10);
	gpio_set_value_cansleep(gpio_reset, 1);
    msleep(300);
#endif 
out: 
 
    return err;
}

static void ft5x06_hw_deint(void)
{
   
	if ( tp_regulator )
		regulator_deinit(tp_regulator);
#if 1
	gpio_free(gpio_reset);
#endif

#ifdef GPIO_TO_IRQ
	gpio_free(FT5X06_IRQ_GPIO);
#endif
}

static int ft5x06_read_points( struct ft5x06_device *ftdev, 
    struct ft5x06_data *data)
{
    int err = 0, i = 0, cnt = 0, number = 0, pcnt = 0;
    unsigned int x, y;
    enum FT_EVENT_TYPE type;
    
    if ( !ftdev || !data )
        return -EFAULT;

    err = ft5x06_get_regs(ftdev->client, FIRST_READ_ADDR, data->rawData, 
        FIRST_READ_LEN );
    if ( err ) {
        printk(KERN_ERR"Read points data failed");
        goto out;
    }

	number = ft5x06_valid_cnt(data);
    if ( number > 1 ) {
        err = ft5x06_get_regs(ftdev->client, SECOND_READ_ADDR, 
            (data->rawData + FIRST_READ_LEN), POINT_DATA_LEN *(number-1));
        if ( err ) {
            printk(KERN_ERR"Read the 2nd mass data failed");
            goto out;
        }
    }
    //printk(KERN_ERR"number:%d\n", number);
    if ( number ) {
        for ( i = 1, cnt = 0; cnt < number; i += POINT_DATA_LEN, cnt++ ) {           
            type = VALUE_TYPE(data->rawData[i]);
            if ( NONE_EVENT == type )
                continue;

            x = VALUE_XY( data->rawData[i + FT5X06_X_OFFSET], 
                                        data->rawData[i+ FT5X06_X_OFFSET + 1]);
            y = VALUE_XY( data->rawData[i + FT5X06_Y_OFFSET], 
                                        data->rawData[i + FT5X06_Y_OFFSET + 1]);
   // printk("raw:x:%d, y:%d\n", x, y);
#if CFG_FT_USE_CONFIG
            if ( x <= cfg_dts.xMax && y <= cfg_dts.yMax ) {
                if (cfg_dts.XYSwap == 1)
                {
                    int tmp;
                    tmp=x;
                    x=y;
                    y=tmp;
                }
                
                if(cfg_dts.xRevert == 1)
                {   
                    x = cfg_dts.xMax - x;
                }
                
                if(cfg_dts.yRevert == 1)
                {
                    y = cfg_dts.yMax - y;
                }
                
                //if(cfg_dts.rotate == 90){//anticlockwise 90 angle
                //    int tmp;
                //    tmp = x;
                //    x = y;
                //    y = cfg_dts.xMax - tmp;
                //}else if(cfg_dts.rotate == 180){//anticlockwise 180 angle
        	//        x = cfg_dts.xMax - x;
	        //        y = cfg_dts.yMax - y;
                //}else if(cfg_dts.rotate == 270){//anticlockwise 270 angle
        	//        int tmp;
		//            tmp = x;
		//            x = cfg_dts.yMax-y;
		//            y = tmp;
                //} 

            }
#endif
            //printk("end:x:%d, y:%d\n", x, y);

            data->points[pcnt].x = x;
            data->points[pcnt].y = y; 
            data->points[pcnt].type = type;
            data->points[pcnt].id = VALUE_ID( data->rawData[i + FT5X06_Y_OFFSET]);
            data->points[pcnt].weight = data->rawData[i+FT5X06_WEIGHT_OFFSET];
			data->points[pcnt].area =
				VALUE_AREA(data->rawData[i+FT5X06_MISC_OFFSET]);
			data->points[pcnt].direction =
				VALUE_DIRECTION(data->rawData[i+FT5X06_MISC_OFFSET]);
			data->points[pcnt].speed =
				VALUE_SPEED(data->rawData[i+FT5X06_MISC_OFFSET]);
            //printk("[%d]:x:%d, y:%d, type:%d\n", data->points[pcnt].id,
			//		x, y, type);
            pcnt++;
            continue;

        }
		data->lastValidCnt = data->validPointCnt ? data->validPointCnt : pcnt;
        data->validPointCnt = pcnt;
    } else {
		data->validPointCnt = 0;
	}
    
out:

    return err;
}

#define TOUCHES	(ftdev->ftdata->touches)
#define	POINTS	(ftdev->ftdata->points)
static int ft5x06_touch_event(struct ft5x06_device *ftdev)
{
	int need_report = 0, finger, event, found, moved, pos_moved;
    enum FT_EVENT_TYPE  type;

    if( !ftdev || !ftdev->ftdata )
        return -EFAULT;

//	if (ftdev->ftdata->validPointCnt == 0) {
//		/* no touch event, state no change */
//		return 0;
//	}

	for (finger = 0; finger < FT5X06_MAX_POINT; finger++) {
		if (POINTS[finger].id >= FT5X06_MAX_TOUCH) {
			/* touch count limit */
			continue;
		}
		if (TOUCHES[finger].id >= FT5X06_MAX_TOUCH) {
			/* touch count limit */
			continue;
		}
		found = -1;
		for (event = 0; event < ftdev->ftdata->validPointCnt; event++) {
			/* search finger.id event */
			if (POINTS[event].id == TOUCHES[finger].id) {
				found = event;
				break;
			}
		}
		if (found != -1) {
			moved = pos_moved = 0;
			if (TOUCHES[finger].x != POINTS[found].x ||
				TOUCHES[finger].y != POINTS[found].y ||
				TOUCHES[finger].weight != POINTS[found].weight ||
				TOUCHES[finger].area != POINTS[found].area)
				moved = 1;
			if (TOUCHES[finger].x != POINTS[found].x ||
				TOUCHES[finger].y != POINTS[found].y)
				pos_moved = 1;
			/* finger.id touch event found */
			type = POINTS[found].type;
			TOUCHES[finger].type = type;
			TOUCHES[finger].weight = POINTS[found].weight;
			TOUCHES[finger].x = POINTS[found].x;
			TOUCHES[finger].y = POINTS[found].y;
			TOUCHES[finger].id = POINTS[found].id;
			TOUCHES[finger].area = POINTS[found].area;
			TOUCHES[finger].direction = POINTS[found].direction;
			TOUCHES[finger].speed = POINTS[found].speed;
			if ((type == PRESS_EVENT) || (type == CONTACT_EVENT)) {
				/* touch down or move event */
				if (TOUCHES[finger].state == TOUCH_NOUSE) {
					TOUCHES[finger].state = TOUCH_DOWN;
					FT5X06_DEBUG("(%d)[%d] down (%d,%d)(%d,%d,%d,%d)", type,
						finger, TOUCHES[finger].x, TOUCHES[finger].y,
						TOUCHES[finger].weight, TOUCHES[finger].area,
						TOUCHES[finger].direction, TOUCHES[finger].speed);
					need_report++;
				} else {
					TOUCHES[finger].state = TOUCH_MOVE;
					if (moved) {
						FT5X06_DEBUG("(%d)[%d] move (%d,%d)(%d,%d,%d,%d)", type,
							finger, TOUCHES[finger].x, TOUCHES[finger].y,
							TOUCHES[finger].weight, TOUCHES[finger].area,
							TOUCHES[finger].direction, TOUCHES[finger].speed);
						need_report++;
					}
#if 0	/* reduced event cancel */
					if (pos_moved == 0)
						need_report = 0;
#endif
				}
			} else {
				/* touch up event */
				TOUCHES[finger].state = TOUCH_UP;
				FT5X06_DEBUG("(%d)[%d] up", type, finger);
				need_report++;
			}
		} else {
			/* finger.id touch event not found */
			/* if previous event is down or move, then touch up */
			if ((TOUCHES[finger].state == TOUCH_DOWN) ||
				(TOUCHES[finger].state == TOUCH_MOVE)) {
				TOUCHES[finger].weight = 0;	/* PRESSURE = 0 */
				TOUCHES[finger].state = TOUCH_UP;
				FT5X06_DEBUG("( )[%d] up (no event)", finger);
				need_report++;
			}
		}
	}

#if 0
	for (finger = 0; finger < FT5X06_MAX_TOUCH; finger++) {
		if (TOUCHES[finger].state == TOUCH_NOUSE)
			continue;
        printk("[FT5x06] [%d] state = %d (%d,%d,%d)\n", finger,
				TOUCHES[finger].state,
				TOUCHES[finger].x, TOUCHES[finger].y, TOUCHES[finger].weight);
	}
#endif

	return need_report;
}
#undef TOUCHES

#if 1
#define TOUCHES	(ftdev->ftdata->touches)
#define REPORT_MT(x, y, amplitude, width, pressure) \
do {     \
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, amplitude);  \
	input_report_abs(input, ABS_MT_POSITION_X, x);           \
	input_report_abs(input, ABS_MT_POSITION_Y, y);           \
	input_report_abs(input, ABS_MT_WIDTH_MAJOR, width);      \
	input_report_abs(input, ABS_MT_PRESSURE, pressure);      \
} while (0)
static int ft5x06_report_b(struct ft5x06_device *ftdev)
{
	struct input_dev *input;
	int finger;

    if( !ftdev || !ftdev->ftdata )
        return -EFAULT;

	input = ftdev->input_dev;

	for (finger = 0; finger < FT5X06_MAX_POINT; finger++) {
		if (TOUCHES[finger].state == TOUCH_NOUSE)
			continue;
		input_mt_slot(input, TOUCHES[finger].id);
		input_mt_report_slot_state(input,
				MT_TOOL_FINGER, !!TOUCHES[finger].weight);
		if (TOUCHES[finger].state == TOUCH_UP) {
			TOUCHES[finger].state = TOUCH_NOUSE;
			continue;
		}
		REPORT_MT(
				TOUCHES[finger].x, TOUCHES[finger].y,
				TOUCHES[finger].weight, /* touch major */
				TOUCHES[finger].area, /* width major */
				TOUCHES[finger].weight /* pressure*/
				);
		FT5X06_DEBUGV("[%d] report (%d,%d)(%d,%d)",
			finger, TOUCHES[finger].x, TOUCHES[finger].y,
			TOUCHES[finger].weight, TOUCHES[finger].area);
	}
	input_mt_report_pointer_emulation(input, true);
	input_sync(input);
	return 0;
}
#undef TOUCHES
#undef REPORT_MT
#else
static int ft5x06_report_b(struct ft5x06_device *ftdev)
{
		int i = 0;
		struct ft5x06_data *data = ftdev->ftdata;
		struct input_dev *input = ftdev->input_dev;
		struct ft5x06_point*pos = &data->points[i];

    if( !ftdev || !ftdev->ftdata )
        return -EFAULT;
		if ( data->validPointCnt == 0 )
		{
                        input_report_abs(input, ABS_PRESSURE, 0);
                        input_report_key(input, BTN_TOUCH, 0);
                        input_sync(input);
		}	
		//* Modify by Hayden.Hu -- begin
		//for ( i = 0; i < data->validPointCnt; i++ ) 
		else
		{
			if ( pos )
			{
				switch( pos->type ) {
					case PRESS_EVENT:
					case CONTACT_EVENT:
						input_report_abs(input, ABS_X, data->points[i].x);
						input_report_abs(input, ABS_Y, data->points[i].y);
						input_report_abs(input, ABS_PRESSURE, 1);
						input_report_key(input, BTN_TOUCH, 1);
						input_sync(input);
						break;
					case RELEASE_EVENT:
						input_report_abs(input, ABS_PRESSURE, 0);
						input_report_key(input, BTN_TOUCH, 0);
						input_sync(input);
						break;
					default :
						printk( "ft5x06_report_b ohoh\n" );
				}
			}
		}
		//* Modify by Hayden.Hu -- end

    return 0;
}
#endif

static int ft5x06_init_config(struct i2c_client *client, struct ft5x06_config *ftconfig)
{
    int err = 0;
    u8 buff[4] = {0};

    if ( !client || !ftconfig )
        return -EINVAL;

    //config: max(x,y)
    err = ft5x06_get_regs(client, OP_IDG_MAX_XH, buff, 4);
    if ( err ) {
        err = -EAGAIN;
    } else {
#if CFG_FT_USE_CONFIG
        ftconfig->max.x = cfg_dts.xMax;
        ftconfig->max.y = cfg_dts.yMax;
#else        
        ftconfig->max.x = FT5X06_X_MAX;
        ftconfig->max.y = FT5X06_Y_MAX;
#endif
        ftconfig->max.weight = (ftconfig->max.x * ftconfig->max.y) >> 1;
    }

    //config: version & vendor
    err = ft5x06_get_regs(client, OP_IDG_LIB_VER_H, buff, 3);
    if ( err ) {
        err = -EAGAIN;
    } else {
        ftconfig->version = (buff[0] << 8 | buff[1]);
        ftconfig->vendor = buff[2];
    }

    //config: product
    err = ft5x06_get_reg(client, OP_IDG_FIRMID, buff);
    if ( err ) {
        err = -EAGAIN;
    } else {
        ftconfig->product = buff[0];
    }

    buff[0] = 0x1;
    err = ft5x06_set_reg(client, OP_IDG_MODE, &buff[0]);
    if ( err ) {
        err = -EAGAIN;
    }

    return err;
}

// Kernel therad
static void ft5x06_work(struct work_struct *work)
{
    struct ft5x06_device *ftdevice = container_of(work, struct ft5x06_device, work);
    int ret;

    ret = ft5x06_read_points(ftdevice, ftdevice->ftdata);
    //enable_irq(ftdevice->irq);/* binzhang(2012/9/13):  */
//    printk("ret is %d \n",ret);
	if (ret == 0) {
         if (ft5x06_touch_event(ftdevice))
             ft5x06_report_b(ftdevice);
	}
	
}

static irqreturn_t ft5x06_interrupt( int irq, void *devid)
{
    struct ft5x06_device *ftdevice = (struct ft5x06_device *)devid;
    unsigned long now = jiffies;

    //Discard first interrupt for updating after recover from sleep
    //printk(KERN_ERR"%s\n", __func__);
    if ( time_after(now, ftdevice->time_discard) ) 
        goto out;

    disable_irq_nosync(irq);/* binzhang(2012/9/13):  */
	if( !work_pending(&ftdevice->work) )
		queue_work(ftdevice->workqueue, &ftdevice->work);
	enable_irq(irq);
out:
    ftdevice->time_discard = jiffies + TIME_OF_DISCARD;
    return IRQ_HANDLED;
}

static inline void disable_power(struct regulator *power)
{
        regulator_disable(power);
}

static inline void regulator_deinit(struct regulator *power)
{
    regulator_disable(power);
    
    regulator_put(power);
}

static struct regulator *regulator_init(const char *name, int minvol, int maxvol)
{
    struct regulator *power;
    int ret;

    power = regulator_get(NULL, name);
    if (IS_ERR(power)) {
        printlf();
        return NULL;
    }
    //gpower = power;
    
    //printlf();
    if (regulator_set_voltage(power, minvol, maxvol)) {
        regulator_deinit(power);
        printlf();
        return NULL;
    }
    ret = regulator_enable(power);
    return (power);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*
 * In early suspend, it should set tp mode to idle. 
 * Then in early resume, wakeup the panel. But here must reset it. But there is no reset-pin in AT711.
 * So here just stop the work queue, and disable reponse of interrupting.
 */
static void ft5x06_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_device *ftdev = container_of(handler, struct ft5x06_device, es);
//    enirq = 0;//tell work,do not enable irq again
    disable_irq(ftdev->irq);
    flush_workqueue(ftdev->workqueue);
	//cancel_work_sync(&ftdev->work);
	//disable_irq(ftdev->irq);
	//printlf();
}

static void ft5x06_early_resume(struct early_suspend *handler)
{
//    printlf();
	struct ft5x06_device *ftdev = container_of(handler, struct ft5x06_device, es);
    enable_irq(ftdev->irq);
}
#endif

#if CFG_FT_USE_CONFIG
static int tp_of_data_get(void)
{
    struct device_node *of_node;
    enum of_gpio_flags flags;
    unsigned int scope[2];
    int ret = -1;

    of_node = of_find_compatible_node(NULL, NULL, "ft5x06");
    if (of_node==NULL){
        printk(KERN_ERR"%s,%d,find the gsxX680 dts err!\n",__func__, __LINE__);
        return -1;
    }

	/* load tp regulator */
	if (of_find_property(of_node, "tp_vcc", NULL)) {
		ret = of_property_read_string(of_node, "tp_vcc", &cfg_dts.regulator);
		if (ret < 0) {
			printk("can not read tp_vcc power source\n");
			cfg_dts.regulator = ctp_power_name;
		}

		if (of_property_read_u32_array(of_node, "vol_range", scope, 2)) {
			printk(" failed to get voltage range\n");
			scope[0] = FT5X06_POWER_MIN_VOL;
			scope[1] = FT5X06_POWER_MAX_VOL;
		}
		cfg_dts.vol_min=scope[0];
		cfg_dts.vol_max=scope[1];
	}

	/* load irq number */
    cfg_dts.sirq = irq_of_parse_and_map(of_node, 0);
	if (cfg_dts.sirq < 0) {
        printk("No IRQ resource for tp\n");
		return -ENODEV;
	}

	/* load gpio info */
	if (!of_find_property(of_node, "reset_gpios", NULL)) {
		printk("<isp>err: no config gpios\n");
		goto fail;
	}
	gpio_reset = of_get_named_gpio_flags(of_node, "reset_gpios", 0, &flags);

	cfg_dts.i2cNum = FT5X06_I2C_ADAPTER;
    
	/* load tp i2c addr */
	ret = of_property_read_u32(of_node, "reg", &cfg_dts.i2cAddr);
	if (ret) {
		printk(" failed to get i2c_addr\n");
		goto fail;
	}
	
	/* load other options */
	ret = of_property_read_u32(of_node, "x_pixel", &cfg_dts.xMax);
	if (ret) {
		printk("failed to get xMax\r\n,set default:1280");
		cfg_dts.xMax = FT5X06_X_MAX;
	}

	ret = of_property_read_u32(of_node, "y_pixel", &cfg_dts.yMax);
	if (ret) {
		printk("failed to get yMax\r\n,set default:800");
		cfg_dts.yMax = FT5X06_Y_MAX;
	}

	ret = of_property_read_u32(of_node, "x_revert_en", &cfg_dts.xRevert);
	if (ret) {
		printk("failed to get xRevert\r\n,set default:1280");
		cfg_dts.xRevert = 0;
	}

	ret = of_property_read_u32(of_node, "y_revert_en", &cfg_dts.yRevert);
	if (ret) {
		printk("failed to get yRevert\r\n,set default:800");
		cfg_dts.yRevert = 0;
	}

	ret = of_property_read_u32(of_node, "xy_swap_en", &cfg_dts.XYSwap);
	if (ret) {
		printk("failed to get XYSwap, set default:0\r\n");
		cfg_dts.XYSwap = 0;
	}
    
	ret = of_property_read_u32(of_node, "rotate_degree", &cfg_dts.rotate);
	if (ret) {
		printk("failed to get rotate, set default:0\r\n");
		cfg_dts.rotate = 0;
	}

	
	printk("gpio num:%d, reset level:%d, i2c_addr:%02x, irq_number:%d,x_pixel:%d, y_pixel:%d, max_point:%d, rotate:%d, i2cNum:%d\n",
		gpio_reset,
		0,
		cfg_dts.i2cAddr,
		cfg_dts.sirq,
		cfg_dts.xMax,
		cfg_dts.yMax,
		5,
		cfg_dts.rotate,
		cfg_dts.i2cNum);
    
        return 0;

fail:
	return -1;
    
}

/********************TP DEBUG************************/

/**************************************************************************/
static ssize_t tp_rotate_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",cfg_dts.rotate);
}
/**************************************************************************/
static ssize_t tp_rotate_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data=0;
	int error;
    error = strict_strtol(buf, 10, &data);
    if (error)
        return error;
	cfg_dts.rotate=data;
    return count;
}
/**************************************************************************/
static ssize_t tp_xrevert_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",cfg_dts.xRevert);
}
/**************************************************************************/
static ssize_t tp_xrevert_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data=0;
	int error;
    error = strict_strtol(buf, 10, &data);
    if (error)
        return error;
	cfg_dts.xRevert=data;
    return count;
}

/**************************************************************************/
static ssize_t tp_yrevert_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",cfg_dts.yRevert);
}
/**************************************************************************/
static ssize_t tp_yrevert_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data=0;
	int error;
    error = strict_strtol(buf, 10, &data);
    if (error)
        return error;
	cfg_dts.yRevert=data;
    return count;
}

/**************************************************************************/
static ssize_t tp_xyswap_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n",cfg_dts.XYSwap);
}
/**************************************************************************/
static ssize_t tp_xyswap_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long data=0;
	int error;
    error = strict_strtol(buf, 10, &data);
    if (error)
        return error;
	cfg_dts.XYSwap=data;
    return count;
}
/**************************************************************************/
static ssize_t tp_resolution_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%dx%d\n",cfg_dts.xMax, cfg_dts.yMax);
}



static DEVICE_ATTR(tp_rotate, S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP,tp_rotate_show, tp_rotate_store);
static DEVICE_ATTR(tp_xrevert, S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP,tp_xrevert_show, tp_xrevert_store);
static DEVICE_ATTR(tp_yrevert, S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP,tp_yrevert_show, tp_yrevert_store);
static DEVICE_ATTR(tp_xyswap, 	S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP,tp_xyswap_show, tp_xyswap_store);
static DEVICE_ATTR(tp_resolution, S_IRUGO, tp_resolution_show, NULL);
static DEVICE_ATTR(debug_switch, S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP, debug_show, debug_store);

static struct attribute *tp_attributes[] = { 
    &dev_attr_tp_rotate.attr,
	 &dev_attr_tp_xrevert.attr,
	  &dev_attr_tp_yrevert.attr,
	  &dev_attr_tp_xyswap.attr,
	&dev_attr_tp_resolution.attr,
	&dev_attr_debug_switch.attr,
    NULL
};

static const struct attribute_group tp_attr_group = {
    .attrs  = tp_attributes,
};
#endif




static int ft5x06_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct ft5x06_device *ftdev = NULL;
    struct input_dev *input = NULL;
    struct ft5x06_data *ftdata = NULL;
    struct ft5x06_config *ftconfig = NULL;
    struct workqueue_struct *wq = NULL;
    int err = 0;
	int i;

    err = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA);
    if ( !err ) {
        FT5X06_WARNNING("I2c bus dosen't support");
        err = -EFAULT;
        goto i2c_check_failed;
    }

    ftdev = (struct ft5x06_device *)kzalloc( \
        sizeof(struct ft5x06_device), GFP_KERNEL);
    if ( !ftdev ) {
        FT5X06_WARNNING("Create ft5x06 device failed");
        err = -ENOMEM;
        goto create_ft5x06_failed;
    }

    ftdata = (struct ft5x06_data *)kzalloc( \
        sizeof(struct ft5x06_data), GFP_KERNEL);
    if ( !ftdata ) {
        FT5X06_WARNNING("Create ft5x06 data failed");
        err = -ENOMEM;
        goto create_data_failed;
    }
	for (i = 0; i < FT5X06_MAX_POINT; i++) {
		ftdata->touches[i].id = i;	/* touch finger id */
		ftdata->touches[i].state = TOUCH_NOUSE;
	}

    ftconfig = (struct ft5x06_config *)kzalloc( \
        sizeof(struct ft5x06_config), GFP_KERNEL);
    if ( !ftconfig ) {
        FT5X06_WARNNING("Create ft5x06 config failed");
        err = -ENOMEM;
        goto create_config_failed;
    }
    
    input = input_allocate_device();
    if ( !input ) {
        FT5X06_WARNNING("Create input device failed");
        err = -ENOMEM;
        goto create_input_failed;
    }

    wq = create_singlethread_workqueue("ft5x06_touch");
    if ( !wq ) {
        FT5X06_WARNNING("Create workqueue failed");
        goto create_workqueue_failed;
    }

    ftdev->workqueue = wq;
    ftdev->client = client;
    ftdev->ftconfig = ftconfig;
    ftdev->ftdata = ftdata;
    ftdev->input_dev = input;    

#if CFG_FT_USE_CONFIG
    ftdev->irq =cfg_dts.sirq;

	#ifdef GPIO_TO_IRQ
    ftdev->irq = gpio_to_irq(FT5X06_IRQ_GPIO);
	#endif
#else
    ftdev->irq = FT5X06_IRQ;
#endif
    
    ftdev->time_discard = 0;
//binzhang :move earlysuspend  after ft5x06_init_config


    i2c_set_clientdata(client, ftdev);

    INIT_WORK(&ftdev->work, ft5x06_work);
    mutex_init(&ftdev->lock);

    err = ft5x06_init_config(ftdev->client, ftdev->ftconfig);
    if ( err ) {
        FT5X06_WARNNING("read config failed");
        goto init_config_failed;
    }
#ifdef CONFIG_HAS_EARLYSUSPEND	
	ftdev->es.level = 50;
	ftdev->es.resume = ft5x06_early_resume;
	ftdev->es.suspend = ft5x06_early_suspend;
	register_early_suspend(&ftdev->es);	
#endif	
    input->name = FT5X06_NAME;
    input->id.bustype = BUS_HOST;
    input->id.version = ftdev->ftconfig->version;
    input->id.vendor = ftdev->ftconfig->vendor;
    input->id.product = ftdev->ftconfig->product;
    ///////////////
 
#if 1
    __set_bit(EV_ABS, input->evbit);
    __set_bit(EV_SYN, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, ftdev->ftconfig->max.x-1, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, ftdev->ftconfig->max.y-1, 0, 0);

	input_mt_init_slots(input, FT5X06_MAX_POINT, INPUT_MT_DIRECT);
	input_set_abs_params(input,
			ABS_MT_POSITION_X, 0, ftdev->ftconfig->max.x-1, 0, 0);
	input_set_abs_params(input,
			ABS_MT_POSITION_Y, 0, ftdev->ftconfig->max.y-1, 0, 0);
	input_set_abs_params(input,
			ABS_MT_TOUCH_MAJOR, 0, ftdev->ftconfig->max.weight, 0, 0);
	input_set_abs_params(input,
			ABS_MT_WIDTH_MAJOR, 0, 40, 0, 0);
	input_set_abs_params(input,
			ABS_MT_PRESSURE, 0, ftdev->ftconfig->max.weight, 0, 0);
#else
    __set_bit(INPUT_PROP_DIRECT,input->propbit);

    ///////////////
    set_bit(EV_ABS, input->evbit);
//* Modify by Hayden.Hu -- begin
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, 1368, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 768, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 1, 0, 0);
#if CFG_FT_USE_CONFIG
	input_set_abs_params(input, ABS_Y, 0, ftdev->ftconfig->max.y, 0, 0);
	input_set_abs_params(input, ABS_X, 0, ftdev->ftconfig->max.x, 0, 0);
#endif
//* Modify by Hayden.Hu -- end
#endif


#if 0
    printk("ft5x06 probe v2.0 i file,FT5X0X_DOWNLOAD_FIRM=(%d)\n", FT5X0X_DOWNLOAD_FIRM);
	#if (FT5X0X_DOWNLOAD_FIRM==1)
printk("download !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	fts_ctpm_auto_upg(client);
	#endif
#endif
	
    err = input_register_device(ftdev->input_dev);
    if ( err ) {
        FT5X06_WARNNING("Register input failed");
        goto input_register_failed;
    }

    device_enable_async_suspend(&client->dev);
    err = request_irq(ftdev->irq, ft5x06_interrupt, \
        IRQF_DISABLED | IRQF_TRIGGER_FALLING, FT5X06_NAME, ftdev);
    if ( err ) {
        FT5X06_WARNNING("Irq request failed");
        goto irq_request_failed;
    }
    
#if CFG_FT_USE_CONFIG
        if (sysfs_create_group(&ftdev->input_dev->dev.kobj, &tp_attr_group) < 0){
            printk("create tp sysfs group error!");     
        }
#endif

    for ( i = 0; i < ARRAY_SIZE(ft5x06_attr); i++ ) {
        err = device_create_file(&client->dev, &ft5x06_attr[i]);
        if ( err ) {
            FT5X06_WARNNING("Add device file failed");
        }
    }

	ft5x06_chardev_setup(ftdev);

    return 0;

irq_request_failed:
    input_unregister_device(input);
input_register_failed:
init_config_failed:    
    destroy_workqueue(wq);
create_workqueue_failed:
    input_free_device(input);
create_input_failed:
    kfree(ftconfig);
create_config_failed:
    kfree(ftdata);
create_data_failed:
    kfree(ftdev);
create_ft5x06_failed:
i2c_check_failed:
    return err;
}

static int ft5x06_remove(struct i2c_client *client)
{
    struct ft5x06_device *ftdev = i2c_get_clientdata(client);
    
    if ( ftdev ) {
        printk("%s,%d~~~~~~~~\n", __func__, __LINE__);
#if CFG_FT_USE_CONFIG
	    sysfs_remove_group(&ftdev->input_dev->dev.kobj, &tp_attr_group);
#endif
        free_irq(ftdev->irq, ftdev);
        input_unregister_device(ftdev->input_dev);
        destroy_workqueue(ftdev->workqueue);
        input_free_device(ftdev->input_dev);
        ft5x06_chardev_remove(ftdev);
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ftdev->es);
#endif
        kfree(ftdev->ftconfig);
        kfree(ftdev->ftdata);
        kfree(ftdev);
    }

    i2c_set_clientdata(client, NULL);
    return 0;
}

static int ft5x06_suspend(struct i2c_client *client, 
    pm_message_t mesg)
{
//    printlf();
//#ifndef CONFIG_HAS_EARLYSUSPEND
    struct ft5x06_device *ftdev = i2c_get_clientdata(client);
//    enirq = 0;//tell work,do not enable irq again
    disable_irq(ftdev->irq);
    flush_workqueue(ftdev->workqueue);
//    cancel_work_sync(&ftdev->work);
//	disable_irq(ftdev->irq);
//#endif
    gpio_set_value_cansleep(gpio_reset, 0);
    if ( tp_regulator )
    {
        current_val = regulator_get_voltage(tp_regulator);
        printk("current_val is %d \n",current_val);    
        //disable_power(tp_regulator);
        regulator_deinit(tp_regulator);
    }
    return 0;
}

static int ft5x06_resume(struct i2c_client *client)
{
    struct ft5x06_device *ftdev = i2c_get_clientdata(client);
    gpio_set_value_cansleep(gpio_reset, 0);
	msleep(30);
	tp_regulator = regulator_init(FT5X06_POWER_ID,current_val,current_val);
	msleep(20);
	gpio_set_value_cansleep(gpio_reset, 1);
//	msleep(5);
//	gpio_set_value_cansleep(gpio_reset, 0);
//	msleep(10);
//	gpio_set_value_cansleep(gpio_reset, 1);
   	msleep(20);
    enable_irq(ftdev->irq);
    return 0;
}
/*
static int ft5x06_detect(struct i2c_client *client, 
    struct i2c_board_info *info)
{
    strcpy(info->type, FT5X06_NAME);

    return 0;
}
*/

//static unsigned short ft5x06_addresses[] = {
//    FT5X06_I2C_ADDR,
//    I2C_CLIENT_END,
//};

static struct i2c_device_id ft5x06_id[] = {
    {FT5X06_NAME, 0},
    {},
};

static struct of_device_id ft5x06_of_match[] = {
       { .compatible = "ft5x06" },
       { }
};

MODULE_DEVICE_TABLE(i2c, ft5x06_id);

static struct i2c_driver ft5x06_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = FT5X06_NAME,
        .of_match_table        = of_match_ptr(ft5x06_of_match),
    },

    .class = I2C_CLASS_HWMON,
    .probe = ft5x06_probe,
    .remove = ft5x06_remove,
    .suspend = ft5x06_suspend,
    .resume = ft5x06_resume,
   // .detect = ft5x06_detect,
   // .address_list = ft5x06_addresses,
    .id_table = ft5x06_id,
};
static struct i2c_board_info tp_info = {
    .type	= FT5X06_NAME,
};

#if CFG_FT_USE_CONFIG
static int tp_config_init(void)
{
    cfg_dts.rotate=TP_ROTATE_DEFAULT;
    cfg_dts.xMax=FT5X06_X_MAX;
    cfg_dts.yMax=FT5X06_Y_MAX;
    cfg_dts.xRevert=TP_XREVERT;
    cfg_dts.yRevert=TP_YREVERT;
    cfg_dts.XYSwap=TP_XYSWAP;
    return 0;
}
#endif

static int touch_ft5x06_init(void)
{
    int err = 0, i = 0;
    struct i2c_client *client = NULL;

    
    tp_config_init();

#if CFG_FT_USE_CONFIG
    err = tp_of_data_get();
    if (err<0)
    {
        printk("ft get config err!!!");
        return err;
    }
    tp_info.addr = cfg_dts.i2cAddr;
#else
    
    tp_info.addr	= FT5X06_I2C_ADDR;
#endif
    
    ft5x06_hw_init();
    
#if 0
#if CFG_FT_USE_CONFIG
    struct i2c_adapter *adap = i2c_get_adapter(cfg_dts.i2cNum);
#else    
    struct i2c_adapter *adap = i2c_get_adapter(FT5X06_I2C_ADAPTER);
#endif

	
    pClient = i2c_new_device(adap, &tp_info); 
#endif  
    err = i2c_add_driver(&ft5x06_driver);
    
    if ( err ) {
            FT5X06_WARNNING("add i2c driver failed");
            goto out;
    }
    
    list_for_each_entry(client, &(ft5x06_driver.clients), detected ) {
        for ( i = 0; i < ARRAY_SIZE(ft5x06_attr); i++ ) {
            err = device_create_file(&client->dev, &ft5x06_attr[i]);
            if ( err ) {
                FT5X06_WARNNING("Add device file failed");
                goto out;
            }
        }
    }
    
out:
    printlf(); 
    return err;
}

static void touch_ft5x06_exit(void)
{
    int i = 0;
    struct i2c_client *client = NULL;

    list_for_each_entry(client, &(ft5x06_driver.clients), detected ){
        for ( i = 0; i < ARRAY_SIZE(ft5x06_attr); i++ )
            device_remove_file(&client->dev, &ft5x06_attr[i]);
    }

    i2c_del_driver(&ft5x06_driver);
    //i2c_unregister_device(pClient);
    ft5x06_hw_deint();
}

module_init(touch_ft5x06_init);
module_exit(touch_ft5x06_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("lzhou/Actions Semi, Inc");
MODULE_DESCRIPTION("driver for touch pannel of Focal Tech's FT5x06");
