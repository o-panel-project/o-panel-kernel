/*
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.
 * If not, write to the Free Software Foundation,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/pm.h>

#include <asm/uaccess.h>
#include <linux/i2c/twl.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/syscalls.h>

#include "wpcio_pin.h"
#include "wpcio.h"
//#include "twl4030-madc.h"
#include <linux/time.h>
#include <linux/mfd/atc260x/atc260x.h>

#define MY_NAME "WPC_IO"
#define IO_INFO(fmt, arg...) \
	do { \
		printk(KERN_INFO MY_NAME " %s: " fmt "\n" , __FUNCTION__, ##arg); \
	} while (0)
#define IO_ERR(fmt, arg...) \
	do { \
		printk(KERN_ERR  MY_NAME " %s: " fmt "\n" , __FUNCTION__, ##arg); \
	} while (0)

#define IO_DRV_VERSION			"2.0.0 for 3.10.37"
#define MAX_OPEN_COUNT			2

#define MADC_SAMPLE_RATE_MS		50

/*
 * debug print switch
 */
static int debug_switch = 0;
#define IO_DBG(fmt, arg...) \
	do { \
		if (debug_switch == 1) \
		printk(KERN_DEBUG MY_NAME " %s: " fmt "\n" , __FUNCTION__, ##arg); \
	} while (0)
#define IO_DBGV(fmt, arg...) \
	do { \
		if (debug_switch > 1) \
		printk(KERN_DEBUG MY_NAME " %s: " fmt "\n" , __FUNCTION__, ##arg); \
	} while (0)

static ssize_t debug_switch_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n(0:off, 1:on, 2:detail)\n", debug_switch);
}

static ssize_t debug_switch_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int val;
	ssize_t result;
	result = sscanf(buf, "%d", &val);
	if (result != 1)
		return -EINVAL;
	debug_switch = val;
	return count;
}

static struct class_attribute wpcio_class_attrs[] = {
	__ATTR(debug, S_IWUSR | S_IRUGO, debug_switch_show, debug_switch_store),
	__ATTR_NULL,
};

struct class wpcio_class = {
	.name = "wpcio",
	.class_attrs = wpcio_class_attrs,
};

struct _wpcio_data {
	unsigned long open_check;
	atomic_t count;
	spinlock_t acc_lock;
	int usb2_on;
	int usb4_on;
	int usb2_overcurrent;
	int usb4_overcurrent;
	struct timer_list timer;
	struct workqueue_struct *workqueue;
	struct work_struct conversion_work;
	int channel;
	int dc_level;
	int bat1_level;
	int bat2_level;
} wpcio_data = {
	.usb2_on = 0,
	.usb4_on = 0,
	.usb2_overcurrent = 0,
	.usb4_overcurrent = 0,
	.channel = 0,
};

typedef struct _gpio_table {
	int gpio;
	char *property_name;	/* node "wpcio-gpio" of Device tree */
	int flags;
	int requested;
} gpio_table_t;
//static struct _gpio_table {
//	int gpio;
//	char *property_name;	/* node "wpcio-gpio" of Device tree */
//	int flags;
//	int requested;
//} gpio_table[] = {
static gpio_table_t gpio_table[] = {
	{0, "dipswitch1-gpios", GPIOF_DIR_IN, 0},
	{0, "dipswitch2-gpios", GPIOF_DIR_IN, 0},
	{0, "dipswitch3-gpios", GPIOF_DIR_IN, 0},
	{0, "dipswitch4-gpios", GPIOF_DIR_IN, 0},
	{0, "leds_green-gpios", GPIOF_DIR_OUT, 0},
	{0, "leds_red-gpios",   GPIOF_DIR_OUT, 0},
	{0, "leds_orange-gpios",GPIOF_DIR_OUT, 0},
	{0, "usb_memory_pwr-gpios", GPIOF_DIR_OUT, 0},
	{0, "usb_felica_pwr-gpios", GPIOF_DIR_OUT, 0},
	{0, "usb_memory_overcur-gpios", GPIOF_DIR_IN, 0},
	{0, "battery_fast-gpios", GPIOF_DIR_IN, 0},
	{0, "battery_full-gpios", GPIOF_DIR_IN, 0},
	{0, "battery_fault-gpios",GPIOF_DIR_IN, 0},
	{0, "sysen_adc_read-gpios", GPIOF_DIR_OUT, 0},
	{0, "sysreq-gpios", GPIOF_DIR_IN, 1},			/* init on wpc_pwrbutton */
	{0, "soft_poweroff-gpios", GPIOF_DIR_OUT, 1},	/* init on wpc_pwrbutton */
	{0, "wifi_bt_power-gpios", GPIOF_DIR_OUT, 1},	/* init on wlan driver */
	{0, "wifi_en-gpios", GPIOF_DIR_OUT, 1},			/* init on wlan driver */
};

static gpio_table_t user_gpio_table[] = {
	{0, "backlight_en-gpios", GPIOF_DIR_OUT, 1},/* init on backlight driver */
	{0, "lcd_vcc_en-gpios", GPIOF_DIR_OUT, 1},	/* init on dsi driver */
	{0, "tp_pwd_rst-gpios", GPIOF_DIR_OUT, 1},	/* init on tp driver */
	{0, "battery_charge-gpios", GPIOF_DIR_OUT, 0},
};

static int wpcio_get_value(char *property)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(gpio_table); i++) {
		if (strcmp(property, gpio_table[i].property_name) == 0) {
			if (gpio_is_valid(gpio_table[i].gpio)) {
				return gpio_get_value(gpio_table[i].gpio);
			}
		}
	}
	for (i = 0; i < ARRAY_SIZE(user_gpio_table); i++) {
		if (strcmp(property, user_gpio_table[i].property_name) == 0) {
			if (gpio_is_valid(user_gpio_table[i].gpio)) {
				return gpio_get_value(user_gpio_table[i].gpio);
			}
		}
	}
	return 0;
}

static void wpcio_set_value(char *property, int value)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(gpio_table); i++) {
		if (strcmp(property, gpio_table[i].property_name) == 0) {
			if (gpio_is_valid(gpio_table[i].gpio)) {
				gpio_set_value_cansleep(gpio_table[i].gpio, value);
				return;
			}
		}
	}
	for (i = 0; i < ARRAY_SIZE(user_gpio_table); i++) {
		if (strcmp(property, user_gpio_table[i].property_name) == 0) {
			if (gpio_is_valid(user_gpio_table[i].gpio)) {
				gpio_set_value_cansleep(user_gpio_table[i].gpio, value);
				return;
			}
		}
	}
}

static int wpcio_get_gpio(char *property)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(gpio_table); i++) {
		if (strcmp(property, gpio_table[i].property_name) == 0) {
			if (gpio_is_valid(gpio_table[i].gpio)) {
				return gpio_table[i].gpio;
			}
		}
	}
	for (i = 0; i < ARRAY_SIZE(user_gpio_table); i++) {
		if (strcmp(property, user_gpio_table[i].property_name) == 0) {
			if (gpio_is_valid(user_gpio_table[i].gpio)) {
				return user_gpio_table[i].gpio;
			}
		}
	}
	return 0;
}

static char *wpcio_get_property(int gpio)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(gpio_table); i++) {
		if (gpio == gpio_table[i].gpio) {
			return gpio_table[i].property_name;
		}
	}
	for (i = 0; i < ARRAY_SIZE(user_gpio_table); i++) {
		if (gpio == user_gpio_table[i].gpio) {
			return user_gpio_table[i].property_name;
		}
	}
	return NULL;
}

// Colman start: 110412

/*
#if defined(CONFIG_OMAP3WPC)
	#include "../../../arch/arm/mach-omap2/mux.h"
#else
	#error "This driver only work with WPC board"
#endif

#if !defined(CONFIG_OMAP_MUX) || !defined(CONFIG_ARCH_OMAP3430)
	#error "We need CONFIG_OMAP_MUX defined"
#endif

*/
// Colman start, change user gpio default state, before is all input with pull up,
// now defines as below
static struct _user_gpio {
	int gpio;
	char *name;
	int output; // 0 = in, 1 = out
	int level;  // 0 = output low, 1 = output high
	int pull;   // 0 = input pull low, 1 = input pull high, others = no pull up
	int requested;  // internal use
} user_gpio[] = {
//	{54, "GPIO_54", 1, 0, 0, 0 },   // output low - bat2 charger on/off
	//{22, "GPIO_22", 1, 0, 1, 1 },   // output low - lcd power sequence
//	{56, "GPIO_56", 0, 0, 0, 0 },   // Input pull low - Cradle status input
//	{57, "GPIO_57", 0, 0, 0, 0 },   // input pull low - Bat2 detect input
	//{116, "GPIO_116", 1, 0, 0, 1 },   // output low - soft power off trigger,already initilized when board init.
	//{18, "GPIO_18", 1, 0, 0, 0 },   // output low - bat1 charger on/off
//	{148, "GPIO_148", 0, 0, 0, 0 }, // input pull low - no specified yet
};

static void set_mux_gpio(struct _user_gpio *io) {
	/*
	if (io->output) {
		omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT, io->gpio);
	}
	else {
		int val = OMAP_MUX_MODE4 ;
		if (io->pull == 1) val |= OMAP_PIN_INPUT_PULLUP;
		else if (io->pull == 0) val |= OMAP_PIN_INPUT_PULLDOWN;
		else val |= OMAP_PIN_INPUT;
		omap_mux_set_gpio(val, io->gpio);
	}
	*/
}

static void set_dir_gpio(struct _user_gpio *io) {
	if (io->output) gpio_direction_output(io->gpio, io->level);
	else gpio_direction_input(io->gpio);    // default is input
}


static struct _user_gpio * get_gpio(int gpio) {
	int i;
	for (i = 0; i < ARRAY_SIZE(user_gpio); i++) {
		if (user_gpio[i].gpio == gpio) {
			if (user_gpio[i].requested) return &user_gpio[i];
			else {
				printk(MY_NAME": gpio_%d not requested!\n", user_gpio[i].gpio);
				return NULL;
			}
		}
	}
	printk(MY_NAME": gpio_%d not found!\n", gpio);
	return NULL;
}
// Colman end, 110412

static void enable_madc(void) {
	/*
	u8 value;
	// Enable madc use HFCLK
	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &value, TWL4030_INTBR_GPBR1);
	value |= TWL4030_INTBR_GPBR1_MADC_HFCLK_EN;
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, value, TWL4030_INTBR_GPBR1);

	// Enable MADC
	twl_i2c_read_u8(TWL4030_MODULE_MADC, &value, TWL4030_MADC_CTRL1);
	value &= ~TWL4030_MADC_CTRL1_MADCON;
	twl_i2c_write_u8(TWL4030_MODULE_MADC, value, TWL4030_MADC_CTRL1);
	value |= TWL4030_MADC_CTRL1_MADCON;
	twl_i2c_write_u8(TWL4030_MODULE_MADC, value, TWL4030_MADC_CTRL1);
	*/
}

static void start_madc(struct _wpcio_data *wpcio_data) {
	u8 val = 0;
	switch (wpcio_data->channel) {
		case 2:
			val = 0x04;
			//gpio_set_value_cansleep(GPIO_PIN_ADCIN2_SEL, 1);
			break;
		case 1:
			//gpio_set_value_cansleep(GPIO_PIN_ADCIN2_SEL, 0);
			val = 0x04;
			break;
		case 0:
		default:
			val = 0x01;
			break;
	}
	//Enable GPIOB1(SYSEN), for ADC
	//gpio_set_value_cansleep(GPIO_PIN_ADCIN2_SEL, 1);
	wpcio_set_value("sysen_adc_read-gpios", 1);
	/*
	twl_i2c_write_u8(TWL4030_MODULE_MADC, val, TWL4030_MADC_SW1SELECT_LSB);
	twl_i2c_write_u8(TWL4030_MODULE_MADC, 0x00, TWL4030_MADC_SW1SELECT_MSB);
	twl_i2c_write_u8(TWL4030_MODULE_MADC, val, TWL4030_MADC_SW1AVERAGE_LSB);
	twl_i2c_write_u8(TWL4030_MODULE_MADC, 0x00, TWL4030_MADC_SW1AVERAGE_MSB);
	// start the conversion
	twl_i2c_write_u8(TWL4030_MODULE_MADC, TWL_MADC_CTRL_SW1_SW1, TWL4030_MADC_CTRL_SW1);
	*/
	// set a timer to get the value
	mod_timer(&wpcio_data->timer, jiffies + msecs_to_jiffies(MADC_SAMPLE_RATE_MS));
}

// Work handler wpen request
static void conversion_work_handler(void* context) {
	struct _wpcio_data *wpcio_data = container_of(context, struct _wpcio_data, conversion_work);
	u8 ret;
	u32 d;
	ret=atc260x_ex_auxadc_read_by_name("AUX0",&d);
	if(ret < 0)
		IO_ERR("cannot get the AUX0 correct translation data!");
	//printk("AUX0 value=%d\n",d);
	wpcio_data->dc_level = d;

	ret=atc260x_ex_auxadc_read_by_name("AUX2",&d);
	if(ret < 0)
		IO_ERR("cannot get the AUX2 correct translation data!");
	//printk("AUX2 value=%d\n",d);
	wpcio_data->bat1_level = d;

	start_madc(wpcio_data);
	// check conversion end
	/*
	twl_i2c_read_u8(TWL4030_MODULE_MADC, &value,  TWL4030_MADC_CTRL_SW1);
	if (value ) {
		// get the adc value
		switch(wpcio_data->channel) {
			case 1:
			case 2:
				twl_i2c_read_u8(TWL4030_MODULE_MADC, &value, TWL4030_MADC_GPCH2_LSB);
				d = value >> 6;
				twl_i2c_read_u8(TWL4030_MODULE_MADC, &value, TWL4030_MADC_GPCH2_MSB);
				d |= (u32)value << 2;
				d = (d * 2500) / 0x3FF;
				if (wpcio_data->channel == 1) wpcio_data->bat1_level = d;
				else wpcio_data->bat2_level = d;
				break;

			case 0:
			default:
				twl_i2c_read_u8(TWL4030_MODULE_MADC, &value, TWL4030_MADC_GPCH0_LSB);
				d = value >> 6;
				twl_i2c_read_u8(TWL4030_MODULE_MADC, &value, TWL4030_MADC_GPCH0_MSB);
				d |= (u32)value << 2;
				d = (d * 1500) / 0x3FF;
				wpcio_data->dc_level = d;
				break;
		}
		//printk("Colman: %s, channel=%d, val=%dmV\n", __FUNCTION__, wpcio_data->channel, d);
		if (++wpcio_data->channel >= 3) wpcio_data->channel = 0;
		start_madc(wpcio_data);
	}
	else {
		// retry after 10mS
		mod_timer(&wpcio_data->timer, jiffies + msecs_to_jiffies(10));
	}
	*/
}

static void start_conversion(unsigned long context) {
	struct _wpcio_data *wpcio_data = (struct _wpcio_data *)context;
	queue_work(wpcio_data->workqueue, &wpcio_data->conversion_work);
}

static int wpc_io_open(struct inode *inode, struct file *filp) {
    if (!test_and_set_bit(0, &wpcio_data.open_check)) {
		atomic_set(&wpcio_data.count, 0);
    }
	atomic_inc(&wpcio_data.count);
	if (atomic_read(&wpcio_data.count) > MAX_OPEN_COUNT) {
		atomic_dec(&wpcio_data.count);
		return -EACCES;
	}
	return 0;
}

static int wpc_io_release(struct inode *inode, struct file *filp) {
	if (atomic_dec_and_test(&wpcio_data.count)) {
		smp_mb__before_clear_bit();
		clear_bit(0, &wpcio_data.open_check);
		smp_mb__after_clear_bit();
	}
	return 0;
}

static ssize_t wpc_io_read(struct file *file, char __user *data, size_t count, loff_t *ppos) {
	return 0;
}

static ssize_t wpc_io_write(struct file *file, const char __user *data, size_t count, loff_t *ppos) {
	return count;
}

static long wpc_io_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
//	printk(MY_NAME":%s cmd=%x, arg=%ld\n",__FUNCTION__,cmd,arg);
	switch(cmd) {
		int data = 0;
		case WPC_GET_DIPSW:
			if (!arg) return -EFAULT;
			data  = wpcio_get_value("dipswitch1-gpios") ? WPC_DIPSW_1 : 0;
			data |= wpcio_get_value("dipswitch2-gpios") ? WPC_DIPSW_2 : 0;
			data |= wpcio_get_value("dipswitch3-gpios") ? WPC_DIPSW_3 : 0;
			data |= wpcio_get_value("dipswitch4-gpios") ? WPC_DIPSW_4 : 0;
			IO_DBG("Get DIPSW=0x%X", data);
			return copy_to_user((void __user *)arg, &data, sizeof(data))? -EFAULT : 0;

		case WPC_GET_LED:
			if (!arg) return -EFAULT;
			data  = wpcio_get_value("leds_green-gpios") ? WPC_LED_GREEN : 0;
			data |= wpcio_get_value("leds_red-gpios")   ? WPC_LED_RED : 0;
			data |= wpcio_get_value("leds_orange-gpios")? WPC_LED_ORANGE : 0;
			IO_DBG("Get LED=0x%X", data);
			return copy_to_user((void __user *)arg, &data, sizeof(data))? -EFAULT : 0;

		case WPC_SET_LED:
			data = arg;
			wpcio_set_value("leds_green-gpios", (data & WPC_LED_GREEN) ? 1:0);
			wpcio_set_value("leds_red-gpios",   (data & WPC_LED_RED)   ? 1:0);
			wpcio_set_value("leds_orange-gpios",(data & WPC_LED_ORANGE)? 1:0);
			IO_DBG("Set LED=0x%X", data);
			return 0;

		case WPC_RESET_USB_HUB:
			// no usb hub 
			spin_lock(&wpcio_data.acc_lock);
			//gpio_set_value_cansleep(GPIO_PIN_USB_HUB_RESET, 1);
			mdelay(100);
			//gpio_set_value_cansleep(GPIO_PIN_USB_HUB_RESET, 0);
			spin_unlock(&wpcio_data.acc_lock);
			return 0;

		case WPC_SET_USB1_ATTACH:
			data = arg;
			// in o-panel ,don't need to power the cradle by gpio control 
			//gpio_set_value_cansleep(GPIO_PIN_USB1_OE_N, data? 0:1); 
			return 0;

		case WPC_SET_USB2_ONOFF:
			//in schematic,network name is USB0_ON, for microUSB-AB
			spin_lock(&wpcio_data.acc_lock);
			if (arg) {
				//gpio_set_value_cansleep(GPIO_PIN_USB2_OE_N, 0);
				//gpio_set_value_cansleep(GPIO_PIN_USB2_POWER, 1);
				wpcio_set_value("usb_memory_pwr-gpios", 1);
				IO_INFO("gpio usb_memory_pwr output to 1");
				mdelay(200);
				//wpcio_data.usb2_overcurrent = gpio_get_value_cansleep(GPIO_PIN_USB2_OVERCUR_N)? 0:1;
				wpcio_data.usb2_overcurrent = wpcio_get_value("usb_memory_overcur-gpios")? 0:1;
				if (!wpcio_data.usb2_overcurrent) wpcio_data.usb2_on = 1;
				else {
					IO_INFO("USB2 overcurrent, switched off");
					// turn off usb2
					//gpio_set_value_cansleep(GPIO_PIN_USB2_OE_N, 1);
					//gpio_set_value_cansleep(GPIO_PIN_USB2_POWER, 0);
					wpcio_set_value("usb_memory_pwr-gpios", 0);
					IO_INFO("gpio usb_memory_pwr output to 0");
					wpcio_data.usb2_on = 0;
					spin_lock(&wpcio_data.acc_lock);
					return -EFAULT;
				}
			}
			else {
				// turn off usb2
				//gpio_set_value_cansleep(GPIO_PIN_USB2_OE_N, 1);
				//gpio_set_value_cansleep(GPIO_PIN_USB2_POWER, 0);
				wpcio_set_value("usb_memory_pwr-gpios", 0);
				IO_INFO("gpio usb_memory_pwr output to 0");
				wpcio_data.usb2_on = 0;
			}
			spin_unlock(&wpcio_data.acc_lock);
			IO_DBG("USB2 On/Off=%ld", arg);
			return 0;

		case WPC_SET_USB3_ONOFF:
			//in schematic,network name is USB2_ON, for USB FeliCa
			spin_lock(&wpcio_data.acc_lock);
			if (arg) {
			//	gpio_set_value_cansleep(GPIO_PIN_USB3_OE_N, 0);
				//gpio_set_value_cansleep(GPIO_PIN_USB3_POWER, 1);
				wpcio_set_value("usb_felica_pwr-gpios", 1);
				IO_INFO("gpio usb_felica_pwr output to 1");
			}
			else {
				// turn off usb2
			//	gpio_set_value_cansleep(GPIO_PIN_USB3_OE_N, 1);
				//gpio_set_value_cansleep(GPIO_PIN_USB3_POWER, 0);
				wpcio_set_value("usb_felica_pwr-gpios", 0);
				IO_INFO("gpio usb_felica_pwr output to 0");
			}
			spin_unlock(&wpcio_data.acc_lock);
			IO_DBG("USB3 On/Off=%ld", arg);
			return 0;

		case WPC_SET_USB4_ONOFF:
			spin_lock(&wpcio_data.acc_lock);
			if (arg) {
				//gpio_set_value_cansleep(GPIO_PIN_USB4_OE_N, 0);
				//gpio_set_value_cansleep(GPIO_PIN_USB4_POWER, 1);
				mdelay(100);
				//wpcio_data.usb4_overcurrent = gpio_get_value_cansleep(GPIO_PIN_USB4_OVERCUR_N)? 0:1;
				if (!wpcio_data.usb2_overcurrent) wpcio_data.usb4_on = 1;
				else {
					IO_INFO("USB2 overcurrent, switched off");
					// turn off usb2
					//gpio_set_value_cansleep(GPIO_PIN_USB4_OE_N, 1);
					//gpio_set_value_cansleep(GPIO_PIN_USB4_POWER, 0);
					wpcio_data.usb4_on = 0;
					spin_lock(&wpcio_data.acc_lock);
					return -EFAULT;
				}
			}
			else {
				// turn off usb2
				//gpio_set_value_cansleep(GPIO_PIN_USB4_OE_N, 1);
				//gpio_set_value_cansleep(GPIO_PIN_USB4_POWER, 0);
				wpcio_data.usb4_on = 0;
			}
			spin_unlock(&wpcio_data.acc_lock);
			IO_DBG("USB4 On/Off=%ld", arg);
			return 0;

		case WPC_GET_USB2_OVERCUR:
			if (!arg) return -EFAULT;
			if (wpcio_data.usb2_on)
				//wpcio_data.usb2_overcurrent = gpio_get_value_cansleep(GPIO_PIN_USB2_OVERCUR_N)? 0:1;
				wpcio_data.usb2_overcurrent = wpcio_get_value("usb_memory_overcur-gpios")? 0:1;
			else wpcio_data.usb2_overcurrent = -1;
			return copy_to_user((void __user *)arg, &wpcio_data.usb2_overcurrent,
					sizeof(wpcio_data.usb2_overcurrent))? -EFAULT : 0;

		case WPC_GET_USB4_OVERCUR:
			if (!arg) return -EFAULT;
			if (wpcio_data.usb4_on)
				;
				//wpcio_data.usb2_overcurrent = gpio_get_value_cansleep(GPIO_PIN_USB4_OVERCUR_N)? 0:1;
			else wpcio_data.usb4_overcurrent = -1;
			return copy_to_user((void __user *)arg, &wpcio_data.usb4_overcurrent,
					sizeof(wpcio_data.usb4_overcurrent))? -EFAULT : 0;

		case WPC_GET_BAT1_CHARGING_STAT:
			if (!arg) return -EFAULT;
			//data = gpio_get_value_cansleep(GPIO_PIN_BAT1_FASTCHG_N)? 0:WPC_CHARGING_FAST;
			//data |= gpio_get_value_cansleep(GPIO_PIN_BAT1_FULLCHG_N)? 0:WPC_CHARGING_FULL;
			//data |= gpio_get_value_cansleep(GPIO_PIN_BAT1_FAULT_N)? 0:WPC_CHARGING_FAULT;
			data  = wpcio_get_value("battery_fast-gpios") ?0:WPC_CHARGING_FAST;
			data |= wpcio_get_value("battery_full-gpios") ?0:WPC_CHARGING_FULL;
			data |= wpcio_get_value("battery_fault-gpios")?0:WPC_CHARGING_FAULT;
			IO_DBG("Battery1 stat=0x%X", data);
			return copy_to_user((void __user *)arg, &data, sizeof(data))? -EFAULT : 0;

		case WPC_GET_BAT2_CHARGING_STAT:
			if (!arg) return -EFAULT;
			//data = gpio_get_value_cansleep(GPIO_PIN_BAT2_FASTCHG_N)? 0:WPC_CHARGING_FAST;
			//data |= gpio_get_value_cansleep(GPIO_PIN_BAT2_FULLCHG_N)? 0:WPC_CHARGING_FULL;
			//data |= gpio_get_value_cansleep(GPIO_PIN_BAT2_FAULT_N)? 0:WPC_CHARGING_FAULT;
			return copy_to_user((void __user *)arg, &data, sizeof(data))? -EFAULT : 0;

		case WPC_CONNECT_SDIO_WIFI:
#if 0
			if (arg) {
				// Power on onboard wifi
				// Make sure it is low, then high to trigger a card detect irq
				gpio_set_value_cansleep(GPIO_PIN_WIFI_RESET_N, 0);		// assert wifi reset
				gpio_direction_output(GPIO_PIN_WIFI_PD_N, 0);	// assert wifi pd
				udelay(600);
				gpio_direction_output(GPIO_PIN_WIFI_PD_N, 1);	// deassert wifi pd
				udelay(600);
				gpio_set_value_cansleep(GPIO_PIN_WIFI_RESET_N, 1);		// deassert wifi reset
			}
			else {
				// Power off onboard wifi
				gpio_set_value_cansleep(GPIO_PIN_WIFI_RESET_N, 0);	// assert wifi reset
				gpio_direction_output(GPIO_PIN_WIFI_PD_N, 0);		// assert wifi pd
			}
#endif
			return 0;

		case WPC_GET_BAT1_LEVEL:
			IO_DBG("Battery1 level=%d", wpcio_data.bat1_level);
			return copy_to_user((void __user *)arg, &wpcio_data.bat1_level,
					sizeof(wpcio_data.bat1_level))? -EFAULT : 0;
		case WPC_GET_BAT2_LEVEL:
			IO_DBG("Battery2 level=%d", wpcio_data.bat2_level);
			return copy_to_user((void __user *)arg, &wpcio_data.bat2_level,
					sizeof(wpcio_data.bat2_level))? -EFAULT : 0;

		case WPC_GET_DC_LEVEL:
			IO_DBG("Dc level=%d", wpcio_data.dc_level);
			return copy_to_user((void __user *)arg, &wpcio_data.dc_level,
					sizeof(wpcio_data.dc_level))? -EFAULT : 0;

		case WPC_CONNECT_MMC1:
			#if defined(GPIO_PIN_MMC1_ON_N)
#if 0
				spin_lock(&wpcio_data.acc_lock);
				if (arg) {
					gpio_set_value_cansleep(GPIO_PIN_MMC1_ON_N, 0);
				}
				else {
					// force disconnect mmc1
					gpio_set_value_cansleep(GPIO_PIN_MMC1_ON_N, 1);
				}
				spin_unlock(&wpcio_data.acc_lock);
#endif
				return 0;
			#else
				return -EINVAL;
			#endif

		case WPC_SET_TP_ONOFF:
			#if defined(GPIO_PIN_TP_POWER)
				spin_lock(&wpcio_data.acc_lock);
				if (arg) {
					//gpio_set_value_cansleep(GPIO_PIN_TP_POWER, 1);
					wpcio_set_value("tp_pwd_rst-gpios", 1);
					IO_INFO("gpio tp_pwd_rst output to 1");
				}
				else {
					//gpio_set_value_cansleep(GPIO_PIN_TP_POWER, 0);
					wpcio_set_value("tp_pwd_rst-gpios", 0);
					IO_INFO("gpio tp_pwd_rst output to 0");
				}
				spin_unlock(&wpcio_data.acc_lock);
				return 0;
			#else
				return -EINVAL;
			#endif

		case WPC_SET_BATTERY_CHARGE:
		{
			wpcio_set_value("battery_charge-gpios", arg);
			IO_DBG("gpio battery_charge-gpios output to %ld", arg);
			return 0;
		}

		// Colman start, 110412
		case WPC_SET_GPIO_INPUT: {
			struct _user_gpio * gpio = get_gpio(arg);
			if (!gpio) {
				IO_ERR("Set INPUT gpio %ld not found", arg);
				return -EINVAL;
			}
			if (gpio->output != 0) {
				gpio->output = 0;
				gpio->pull = -1;
				set_dir_gpio(gpio);
				set_mux_gpio(gpio);
			}
			else if (gpio->pull != -1) {
				gpio->pull = -1;
				set_mux_gpio(gpio);
			}
			return 0;
		}

		case WPC_SET_GPIO_INPUT_PULLUP: {
			struct _user_gpio * gpio = get_gpio(arg);
			if (!gpio) {
				IO_ERR("Set INPUT_PULLUP gpio %ld not found", arg);
				return -EINVAL;
			}
			if (gpio->output != 0) {
				gpio->output = 0;
				gpio->pull = 1;
				set_dir_gpio(gpio);
				set_mux_gpio(gpio);
			}
			else if (gpio->pull != 1) {
				gpio->pull = 1;
				set_mux_gpio(gpio);
			}
			return 0;
		}

		case WPC_SET_GPIO_INPUT_PULLDOWN: {
			struct _user_gpio * gpio = get_gpio(arg);
			if (!gpio) {
				IO_ERR("Set INPUT_PULLDOWN gpio %ld not found", arg);
				return -EINVAL;
			}
			if (gpio->output != 0) {
				gpio->output = 0;
				gpio->pull = 0;
				set_dir_gpio(gpio);
				set_mux_gpio(gpio);
			}
			else if (gpio->pull != 0) {
				gpio->pull = 0;
				set_mux_gpio(gpio);
			}
			return 0;
		}

		case WPC_SET_GPIO_OUTPUT_HIGH: {
			char *property = wpcio_get_property(arg);
			if (!property) {
				IO_ERR("Set OUTPUT_HIGH gpio %ld not found", arg);
				return -EINVAL;
			}
			wpcio_set_value(property, 1);
			IO_INFO("gpio %s output to 1", property);
#if 0
			struct _user_gpio * gpio = get_gpio(arg);
			if (!gpio) {
				printk(MY_NAME": Set OUTPUT_HIGH gpio %ld not found\n", arg);
				return -EINVAL;
			}
			if (gpio->output != 1) {
				gpio->output = 1;
				gpio->level = 1;
				set_dir_gpio(gpio);
				set_mux_gpio(gpio);
			}
			else if (gpio->level != 1) {
				gpio->level = 1;
				gpio_set_value_cansleep(gpio->gpio, 1);
			}
#endif
			return 0;
		}

		case WPC_SET_GPIO_OUTPUT_LOW: {
			char *property = wpcio_get_property(arg);
			if (!property) {
				IO_ERR("Set OUTPUT_LOW gpio %ld not found", arg);
				return -EINVAL;
			}
			wpcio_set_value(property, 0);
			IO_INFO("gpio %s output to 0", property);
#if 0
			struct _user_gpio * gpio = get_gpio(arg);
			if (!gpio) {
				printk(MY_NAME": Set OUTPUT_LOW gpio %ld not found\n", arg);
				return -EINVAL;
			}
			if (gpio->output != 1) {
				gpio->output = 1;
				gpio->level = 0;
				set_dir_gpio(gpio);
				set_mux_gpio(gpio);
			}
			else if (gpio->level != 0) {
				gpio->level = 0;
				gpio_set_value_cansleep(gpio->gpio, 0);
			}
#endif
			return 0;
		}

		case WPC_GET_GPIO_LEVEL: {
			char *property = wpcio_get_property(arg);
			if (!property) {
				IO_ERR("Get LEVEL gpio %ld not found", arg);
				return -EINVAL;
			}
			data = wpcio_get_value(property);
			IO_INFO("gpio %s level is %d", property, data);
#if 0
			struct _user_gpio * gpio = get_gpio(arg);
			if (!gpio) {
				printk(MY_NAME": Get LEVEL gpio %ld not found\n", arg);
				return -EINVAL;
			}
			// Whatever it is an input or output, we can always get its level
			data = gpio_get_value_cansleep(gpio->gpio);
#endif
			return data;
		}
		// Colman end, 110412

		default:
			IO_ERR("0x%x unknown command", cmd);
			return -EINVAL;
	}
}

static struct file_operations wpc_io_fops = {
	.read    = wpc_io_read,
	.write   = wpc_io_write,
	.unlocked_ioctl   = wpc_io_ioctl,
	.open    = wpc_io_open,
	.release = wpc_io_release,
};

/*
 * call from _atc260x_pm_powerdown()@atc260x-pm.c
 *           _atc260c_9a_pm_do_reboot()@atc260x-pm.c
 */
void wpcio_powerdown_withgpio(void)
{
	IO_ERR("start");

	sys_sync();
	sys_sync();
	sys_sync();
	sys_sync();
	sys_sync();
	mdelay(1000);

	IO_ERR("soft_poweroff-gpios=%d", wpcio_get_gpio("soft_poweroff-gpios"));
	wpcio_set_value("soft_poweroff-gpios", 1);

	while (1) ;

	return;
}
EXPORT_SYMBOL(wpcio_powerdown_withgpio);

struct miscdevice wpc_io_miscdev = {
    .name       = "wpcio",
    .fops       = &wpc_io_fops,
    //.minor      = WPCIO_MINOR,
    .minor      = 222,
};

static struct _pin_table {
	int gpio;
	char *name;
	int output;
	int level;
	int requested;
} pin_table[] = {
	//{ GPIO_PIN_DIPSW1, 			"DIPSW_1", 		0, 0, 0 },
	//{ GPIO_PIN_DIPSW2, 			"DIPSW_2", 		0, 0, 0 },
	//{ GPIO_PIN_DIPSW3, 			"DIPSW_3", 		0, 0, 0 },
	//{ GPIO_PIN_DIPSW4, 			"DIPSW_4", 		0, 0, 0 },
	//{ GPIO_PIN_LED_GREEN, 		"LED_GREEN", 	1, 0, 0 },
	//{ GPIO_PIN_LED_RED,			"LED_RED", 		1, 0, 0 },
	//{ GPIO_PIN_LED_ORANGE, 		"LED_ORANGE",	1, 0, 0 },
//	{ GPIO_PIN_USB_HUB_RESET, 	"HUB_RESET",	1, 1, 0 },
//	{ GPIO_PIN_USB1_OE_N,		"USB1_OEN",		1, 1, 0 },
//	{ GPIO_PIN_USB2_OE_N,		"USB2_OEN",		1, 1, 0 },
//	{ GPIO_PIN_USB3_OE_N,		"USB3_OEN",		1, 1, 0 },
//	{ GPIO_PIN_USB4_OE_N,		"USB4_OEN",		1, 1, 0 },
//	{ GPIO_PIN_USB2_POWER,		"USB2_PWR", 	1, 0, 0 },
	//{ GPIO_PIN_USB3_POWER,		"USB3_PWR", 	1, 0, 0 },
//	{ GPIO_PIN_USB4_POWER,		"USB4_PWR", 	1, 0, 0 },
	//{ GPIO_PIN_USB2_OVERCUR_N,	"USB2_OCN",		0, 0, 0 },
//	{ GPIO_PIN_USB4_OVERCUR_N,	"USB4_OCN",		0, 0, 0 },
	//{ GPIO_PIN_BAT1_FAULT_N,	"BAT1_FAULTN",	0, 0, 0 },
	//{ GPIO_PIN_BAT1_FASTCHG_N,	"BAT1_FASTCN",	0, 0, 0 },
	//{ GPIO_PIN_BAT1_FULLCHG_N,	"BAT1_FULLCN",	0, 0, 0 },
//	{ GPIO_PIN_BAT2_FAULT_N,	"BAT2_FAULTN",	0, 0, 0 },
//	{ GPIO_PIN_BAT2_FASTCHG_N,	"BAT2_FASTCN",	0, 0, 0 },
//	{ GPIO_PIN_BAT2_FULLCHG_N,	"BAT2_FULLCN",	0, 0, 0 },
	//{ GPIO_PIN_WIFI_PD_N,		"WIFI_PDN",		1, 0, 1 },	// Leave mmc3 to request it.
	//{ GPIO_PIN_WIFI_RESET_N,	"WIFI_RESETN",	1, 0, 1 },
	//{ GPIO_PIN_ADCIN2_SEL,		"ADCIN2_SEL",   1, 0, 0 },
//	#if defined(GPIO_PIN_MMC1_ON_N)
//		{ GPIO_PIN_MMC1_ON_N, 	"MMC1_ONN",		1, 0, 0 },
//	#endif
//	#if defined(GPIO_PIN_TP_POWER)
//		{ GPIO_PIN_TP_POWER,	"TP_PWR",		1, 1, 0 },
//	#endif
};

static int __init wpc_io_init(void) {
	int i;
	struct device_node *dts;
	u32 gpio;
	enum of_gpio_flags flags;

	IO_INFO("driver ver %s", IO_DRV_VERSION);

	spin_lock_init(&wpcio_data.acc_lock);

	if (misc_register(&wpc_io_miscdev) < 0) {
        IO_ERR("Can't register misc device with minor %d", wpc_io_miscdev.minor);
        return -EIO;
	}
	IO_INFO("misc dev got minor %i", wpc_io_miscdev.minor);

	dts = of_find_node_by_name(NULL, "wpcio-gpio");
	if (!dts) {
		IO_ERR("Could not find wpcio gpio node");
		return -EIO;
	}
	for (i = 0; i < ARRAY_SIZE(gpio_table); i++) {
		gpio = of_get_named_gpio_flags(
				dts, gpio_table[i].property_name, 0, &flags);
		if (gpio_is_valid(gpio)) {
			gpio_table[i].gpio = gpio;
			gpio_export(gpio, true);
			if (gpio_table[i].requested == 1)
				continue;
			gpio_request(gpio, gpio_table[i].property_name);
			if (gpio_table[i].flags & GPIOF_DIR_IN) {
				gpio_direction_input(gpio);
			} else {
				/*
				 * Output initial value from node "wpcio-gpio" of Device tree
				 */
				gpio_direction_output(gpio, flags & OF_GPIO_ACTIVE_LOW);
			}
			gpio_table[i].requested = 1;
		}
	}
	for (i = 0; i < ARRAY_SIZE(user_gpio_table); i++) {
		gpio = of_get_named_gpio_flags(
				dts, user_gpio_table[i].property_name, 0, &flags);
		if (gpio_is_valid(gpio)) {
			user_gpio_table[i].gpio = gpio;
			gpio_export(gpio, true);
			if (user_gpio_table[i].requested == 1)
				continue;
			gpio_request(gpio, user_gpio_table[i].property_name);
			if (user_gpio_table[i].flags & GPIOF_DIR_IN) {
				gpio_direction_input(gpio);
			} else {
				/*
				 * Output initial value from node "wpcio-gpio" of Device tree
				 */
				gpio_direction_output(gpio, flags & OF_GPIO_ACTIVE_LOW);
			}
			user_gpio_table[i].requested = 1;
		}
	}
	of_node_put(dts);

	// enable specific IO pin
	for (i = 0; i < ARRAY_SIZE(pin_table); i++) {
		if (!gpio_request(pin_table[i].gpio, pin_table[i].name)) {
			pin_table[i].requested = 1;
			if (pin_table[i].output) gpio_direction_output(pin_table[i].gpio, pin_table[i].level);
			else gpio_direction_input(pin_table[i].gpio);
			/* no GPIO_PIN_USB_HUB_RESET
			if (pin_table[i].gpio == GPIO_PIN_USB_HUB_RESET) {
				// pulse the usb hub reset
				mdelay(100);
				gpio_set_value_cansleep(GPIO_PIN_USB_HUB_RESET, 0);
			}
			*/
		}
		else {
			IO_ERR("Unable to request gpio %d, name = %s",
					pin_table[i].gpio, pin_table[i].name);
		}
	}

	// Colman start, 110412
	// enable user gpio pin
	for (i = 0; i < ARRAY_SIZE(user_gpio); i++){
		if (user_gpio[i].requested == 1)
			continue;
		if (!gpio_request(user_gpio[i].gpio, user_gpio[i].name)) {
			user_gpio[i].requested = 1;
			set_dir_gpio(&user_gpio[i]);
			set_mux_gpio(&user_gpio[i]);
		}
		else {
			IO_ERR("Unable to request user gpio %d, name = %s",
					user_gpio[i].gpio, user_gpio[i].name);
		}
	}
	// Colman end, 110412

	class_register(&wpcio_class);

	enable_madc();

	wpcio_data.workqueue = create_singlethread_workqueue("wpcio_workqueue");
	INIT_WORK(&wpcio_data.conversion_work, (work_func_t)conversion_work_handler);
	init_timer(&wpcio_data.timer);
	wpcio_data.timer.data = (unsigned long)&wpcio_data;
	wpcio_data.timer.function = &start_conversion;

	start_madc(&wpcio_data);

	return 0;
}



static void __exit wpc_io_exit(void) {
	int i;
	del_timer(&wpcio_data.timer);
	for (i = 0; i < ARRAY_SIZE(pin_table); i++) {
		if (pin_table[i].requested) {
			gpio_free(pin_table[i].gpio);
			pin_table[i].gpio = 0;
		}
	}

	// Colman start, 110412
	for (i = 1; i < ARRAY_SIZE(user_gpio); i++) {
		if (user_gpio[i].requested) {
			gpio_free(user_gpio[i].gpio);
			user_gpio[i].gpio = 0;
		}
	}
	// Colman end, 110412

	if (misc_deregister(&wpc_io_miscdev) < 0) IO_ERR("deregister fail");
	destroy_workqueue(wpcio_data.workqueue);
	class_unregister(&wpcio_class);
}

module_init(wpc_io_init);
module_exit(wpc_io_exit);

MODULE_AUTHOR("Colman Lai");
MODULE_DESCRIPTION("Driver for WPC IO");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WPCIO_MINOR);
