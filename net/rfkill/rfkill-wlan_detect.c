/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include "../../../drivers/mmc/host/wlan_plat_data.h"
extern struct wlan_plat_data *acts_get_wlan_plat_data(void);
extern int set_wlan_detect_state(int is_ready);
extern unsigned int set_wlan_id(unsigned int id);

#define DRV_NAME "wlan_detect"

struct wlan_detect {
	char strVendor[10];
	char strDevice[10];
};

static struct wlan_detect *wlan_detect_data;

static int wlan_detect_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int wlan_detect_release(struct inode *inode, struct file *file)
{
	return 0;
}

#if 0//(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
int
compat_kernel_read(struct file *file, loff_t offset, char *addr, unsigned long count)
{
	return (int)kernel_read(file, addr, (size_t)count, &offset);
}
int
compat_vfs_write(struct file *file, char *addr, int count, loff_t *offset)
{
	return (int)kernel_write(file, addr, count, offset);
}
#else
int
compat_kernel_read(struct file *file, loff_t offset, char *addr, unsigned long count)
{
	return kernel_read(file, offset, addr, count);
}
int
compat_vfs_write(struct file *file, char *addr, int count, loff_t *offset)
{
	return (int)vfs_write(file, addr, count, offset);
}
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)) */

static int get_wifi_info(void)
{
	#define WIFI_INFO_PATH "/sys/bus/sdio/devices/mmc1:0001:1/uevent"
	#define UEVENT_MAX_SIZE		256
	#define SDIO_WAIT_TIME	20

	int i = SDIO_WAIT_TIME;
	int rdlen,size;
	unsigned int wlan_id;
	char buf[UEVENT_MAX_SIZE];
	char *pstr = NULL;
	struct file* fp=NULL;

	while(i--) {
		fp = filp_open(WIFI_INFO_PATH, O_RDONLY, 0);
		if (!IS_ERR(fp))
			break;
		mdelay(50);
	}

	pr_err("sunlei retry %d times\n", SDIO_WAIT_TIME-i);

	if (IS_ERR(fp)) {
		pr_err("%s: %s is open failed\n", __FUNCTION__, WIFI_INFO_PATH);
		fp = NULL;
		goto err;
	}

	if (!S_ISREG(file_inode(fp)->i_mode)) {
		pr_err("%s: %s is not regular file\n", __FUNCTION__, WIFI_INFO_PATH);
		fp = NULL;
		goto err;
	}

	size = i_size_read(file_inode(fp));
	if (size <= 0) {
		pr_err("%s: %s file size invalid %d\n", __FUNCTION__, WIFI_INFO_PATH, size);
		fp = NULL;
		goto err;
	}

	pr_err("=====i_size_read = %d\n", size);
	size = (size > UEVENT_MAX_SIZE) ? UEVENT_MAX_SIZE : size;
	rdlen = compat_kernel_read(fp, fp->f_pos, buf, size);
	pr_err("=====rdlen = %d\n", rdlen);

	buf[rdlen]="\0";

	if (rdlen > 0) {
		fp->f_pos += rdlen;
	}

	//Parse uevent file, The file content is as follows:
	//SDIO_CLASS=00
	//SDIO_ID=02D0:A804
	//MODALIAS=sdio:c00v02D0dA804
	pstr = strstr(buf, "SDIO_ID=") + 8;
	memset(wlan_detect_data->strVendor, 0, sizeof(wlan_detect_data->strVendor));
	memset(wlan_detect_data->strDevice, 0, sizeof(wlan_detect_data->strDevice));
	strncpy(wlan_detect_data->strVendor, pstr, 4);
	strncpy(wlan_detect_data->strDevice, pstr+5, 4);

	sprintf(buf, "0x%s%s", wlan_detect_data->strVendor, wlan_detect_data->strDevice);
	pr_err("%s: vendor=0x%s device=0x%s, buf=%s\n", __FUNCTION__, wlan_detect_data->strVendor, wlan_detect_data->strDevice, buf);

	wlan_id = (unsigned int)simple_strtol(buf, NULL, 16);

	pr_err("%s: wifi_id->[0x%08X]\n", __FUNCTION__, wlan_id);

	set_wlan_id(wlan_id);

	return rdlen;
err:

	return -1;
}

static int wlan_detect_start(void)
{
	int err = 0;
	struct wlan_plat_data *pdata = acts_get_wlan_plat_data();

	if (!(pdata && pdata->set_init && pdata->set_power && pdata->set_exit)) {
		pr_err("sunlei %s: pdata->set_power==null\n", __FUNCTION__);
		return -1;
	}

	set_wlan_detect_state(0);

	pdata->set_init(pdata);
	pdata->set_power(pdata, 1);

	if (!(pdata && pdata->set_carddetect)) {
		pr_err("sunlei %s: pdata->set_carddetect==null\n", __FUNCTION__);
		return -1;
	}

	pr_err("sunlei %s: start to detect SDIO card!\n", __FUNCTION__);
	err = pdata->set_carddetect(1);

	//read uevent
	mdelay(20);
	get_wifi_info();

	pr_err("sunlei %s: start to remove SDIO card!\n", __FUNCTION__);
	err = pdata->set_carddetect(0);
	mdelay(20);
	pdata->set_power(pdata, 0);
	pdata->set_exit(pdata);
	mdelay(20);
	set_wlan_detect_state(1);
	return 0;
}


static const struct file_operations wlan_detect_ops = {
	.owner = THIS_MODULE,
	.open = wlan_detect_open,
	.release = wlan_detect_release,
};

static ssize_t wifi_vendor_state_show(struct device *dev, struct device_attribute *attr,
                               char *buf)
{
	return sprintf(buf, "%s\n", wlan_detect_data->strVendor);
}

static ssize_t wifi_vendor_state_store(struct device *dev,
                                       struct device_attribute *attr, const char *buf,
                                       size_t n)
{
	return sprintf(buf, "vendor=%s,It is readonly!!\n", wlan_detect_data->strVendor);
}

static ssize_t wifi_device_state_show(struct device *dev, struct device_attribute *attr,
							  char *buf)
{
   return sprintf(buf, "%s\n", wlan_detect_data->strDevice);
}

static ssize_t wifi_device_state_store(struct device *dev,
									  struct device_attribute *attr, const char *buf,
									  size_t n)
{
   return sprintf(buf, "device=%s,It is readonly!!\n", wlan_detect_data->strDevice);
}

static ssize_t wifi_detect_state_show(struct device *dev, struct device_attribute *attr,
                               char *buf)
{
	return sprintf(buf, "wifi_detect is write only!!\n");
}

static ssize_t wifi_detect_state_store(struct device *dev,
                                       struct device_attribute *attr, const char *buf,
                                       size_t n)
{
	wlan_detect_start();
	return n;
}

static DEVICE_ATTR(wifi_vendor, 0664, wifi_vendor_state_show, wifi_vendor_state_store);
static DEVICE_ATTR(wifi_device, 0664, wifi_device_state_show, wifi_device_state_store);
static DEVICE_ATTR(wifi_detect, 0664, wifi_detect_state_show, wifi_detect_state_store);

static struct attribute * wlan_detect_attrs[] = {
	&dev_attr_wifi_vendor.attr,
	&dev_attr_wifi_device.attr,
	&dev_attr_wifi_detect.attr,
	NULL
};

static const struct attribute_group wlan_detect_group = {
	.attrs = wlan_detect_attrs,
};

static int wlan_detect_probe(struct platform_device *pdev)
{
	int ret=-1;
	pr_err("sunlei %s: start pdev->name=%s, id=%d\n", __FUNCTION__, pdev->name, pdev->id);

	return ret;
}

static int wlan_detect_remove(struct platform_device *pdev)
{
	pr_err("sunlei==wlan_detect_remove\n");

	return 0;
}

struct platform_device *pdev=NULL;
static struct platform_driver wlan_detect_driver = {
	.probe  = wlan_detect_probe,
	.remove = wlan_detect_remove,

	.driver = {
		.name   = "rfkill_wlan_detect",
		.owner  = THIS_MODULE,
	},
};

static int __init rfkill_wlan_detect_init(void)
{
	int ret = 0;

	pr_err("sunlei %s ========\n", __func__);
	ret = platform_driver_register(&wlan_detect_driver);
	if (ret) {
		pr_err("register wlan_detect_driver platform driver error!\n");
		return ret;
	}
	pdev = platform_device_register_simple(DRV_NAME, -1, NULL, 0);
	
	wlan_detect_data = devm_kzalloc(&pdev->dev, sizeof(struct wlan_detect), GFP_KERNEL);
	if(!wlan_detect_data){
		printk("wlan_detect: devm_kzalloc wlan_detect_data failed!\n");
		return -ENODEV;
	}

    sprintf(wlan_detect_data->strVendor, "%s", "unknown");
	sprintf(wlan_detect_data->strDevice, "%s", "unknown");

	wlan_detect_start();

	return sysfs_create_group(&pdev->dev.kobj, &wlan_detect_group);
}

static void __exit rfkill_wlan_detect_exit(void)
{
	pr_err("-->%s\n", __func__);

	sysfs_remove_group(&pdev->dev.kobj, &wlan_detect_group);
	
	platform_driver_unregister(&wlan_detect_driver);

}

module_init(rfkill_wlan_detect_init);
module_exit(rfkill_wlan_detect_exit);

MODULE_DESCRIPTION("WLAN module type detect driver");
MODULE_AUTHOR("sunlei <sunlei@huida.com>");
MODULE_LICENSE("GPL");
