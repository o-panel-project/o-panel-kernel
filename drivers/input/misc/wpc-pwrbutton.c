/**
 * wpc-pwrbutton.c - o-panel Power Button Input Driver
 * Based on wpc-pwrbutton.c - J3 Power Button Input Driver
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Peter De Schrijver <peter.de-schrijver@nokia.com>
 * Several fixes by Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/wpc_pwrbutton.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <mach/irqs.h>

static int user_monitor_ready = 0;

static ssize_t user_monitor_ready_show(struct class *class,
				struct class_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%u\n", user_monitor_ready);
}

static ssize_t user_monitor_ready_store(struct class *class,
				struct class_attribute *attr,
				const char *buf,
				size_t count)
{
	int val;
	ssize_t result;
	result = sscanf(buf, "%u", &val);
	if (result != 1)
		return -EINVAL;
	user_monitor_ready = val;
	return count;
}

static struct class_attribute wpc_pwrbutton_attrs[] = {
	__ATTR(monitor_ready, S_IWUSR | S_IRUGO,
			user_monitor_ready_show, user_monitor_ready_store),
	__ATTR_NULL,
};

struct class wpc_pwrbutton_class = {
	.name = WPC_PWRBUTTON_NAME,
	.class_attrs = wpc_pwrbutton_attrs,
};

static irqreturn_t wpc_pwrbutton_irq(int irq, void *_pwr)
{
	struct input_dev *pwr = _pwr;
	struct wpc_pwrbutton_platform_data *pdata = pwr->dev.parent->platform_data;
//	int gpio = (int)pwr->dev.parent->platform_data;
	unsigned int val;
	val = gpio_get_value(pdata->gpio_sys_req);
	dev_notice(pwr->dev.parent,
			"WPC o-panel Power button (gpio-%d) detected (%d).\n",
			pdata->gpio_sys_req, val);
	if (val < 1)
		return IRQ_HANDLED;

	if (!user_monitor_ready) {
		/* Power off right a way */
		dev_err(pwr->dev.parent, "WPC o-panel Power down.\n");
		gpio_set_value_cansleep(pdata->gpio_soft_poweroff, 1);
		return IRQ_HANDLED;
	}

	/* Power button monitor ready */
	input_report_key(pwr, KEY_POWER, 1);
	input_sync(pwr);

	return IRQ_HANDLED;
}

#define GPIO_PWR_SYSREQ  115
#define GPIO_PWR_SOFTPOWEROFF 116

static int __init wpc_pwrbutton_init_hw(void)
{
        int gpio = GPIO_PWR_SYSREQ;
        int err;
        pr_notice("%s\n", __func__);

        /* Setup gpio */
        err = gpio_request(gpio, "wpc_pwrbutton");
        if (err) {
                pr_err("%s: Failed to gpio request GPIO_%d\n", __func__, gpio);
                return -EINVAL;
        }
        gpio_direction_input(gpio);

        gpio = GPIO_PWR_SOFTPOWEROFF;

        err = gpio_request(gpio, "wpc_soft_reset");
        if (err) {
                pr_err("%s: Failed to gpio request GPIO_%d\n", __func__, gpio);
                return -EINVAL;
        }
        gpio_direction_output(gpio, 0);

        return 0;
}



static int wpc_pwrbutton_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	struct wpc_pwrbutton_platform_data *pdata = pdev->dev.platform_data;
	int irq,err;
//	int gpio = pdata->gpio_sys_req;
	wpc_pwrbutton_init_hw();
	irq = gpio_to_irq(pdata->gpio_sys_req);
	
	dev_notice(&pdev->dev, "probe gpio-%d,irq-%d\n",pdata->gpio_sys_req, irq);

	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	pwr->evbit[0] = BIT_MASK(EV_KEY);
	pwr->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	pwr->name = WPC_PWRBUTTON_NAME;
	pwr->phys = WPC_PWRBUTTON_PHYS;
	pwr->dev.parent = &pdev->dev;

	err = request_threaded_irq(irq, NULL, wpc_pwrbutton_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, WPC_PWRBUTTON_NAME, pwr);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get IRQ for pwrbutton: %d\n", err);
		goto free_input_dev;
	}

	err = input_register_device(pwr);
	if (err) {
		dev_err(&pdev->dev, "Can't register power button: %d\n", err);
		goto free_irq;
	}

	platform_set_drvdata(pdev, pwr);
	class_register(&wpc_pwrbutton_class);

	return 0;

free_irq:
	free_irq(irq, NULL);
free_input_dev:
	input_free_device(pwr);
	return err;
}

static int wpc_pwrbutton_remove(struct platform_device *pdev)
{
	struct input_dev *pwr = platform_get_drvdata(pdev);
	struct wpc_pwrbutton_platform_data *pdata = pwr->dev.parent->platform_data;
	int irq = gpio_to_irq(pdata->gpio_sys_req);

	disable_irq(irq);
	class_unregister(&wpc_pwrbutton_class);
	free_irq(irq, pwr);
	input_unregister_device(pwr);

	return 0;
}

static struct platform_driver wpc_pwrbutton_driver = {
	.probe		= wpc_pwrbutton_probe,
	.remove		= __exit_p(wpc_pwrbutton_remove),
	.driver		= {
		.name	= WPC_PWRBUTTON_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init wpc_pwrbutton_init(void)
{
	pr_notice("WPC Power button driver initialize.\n");
	return platform_driver_register(&wpc_pwrbutton_driver);
}
module_init(wpc_pwrbutton_init);

static void __exit wpc_pwrbutton_exit(void)
{
	platform_driver_unregister(&wpc_pwrbutton_driver);
}
module_exit(wpc_pwrbutton_exit);

MODULE_ALIAS("platform:wpc_pwrbutton");
MODULE_DESCRIPTION("WPC o-panel Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter De Schrijver <peter.de-schrijver@nokia.com>");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");
MODULE_AUTHOR("RH <r-hara@wpc>");

