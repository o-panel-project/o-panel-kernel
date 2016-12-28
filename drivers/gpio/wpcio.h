#ifndef __WPCIO_H__
#define __WPCIO_H__


// DIP switch input
#define WPC_GET_DIPSW				0x5701
	#define WPC_DIPSW_1					0x01
	#define WPC_DIPSW_2					0x02
	#define WPC_DIPSW_3					0x04
	#define WPC_DIPSW_4					0x08

// LED control
#define WPC_GET_LED					0x5702
#define WPC_SET_LED					0x5703
	#define WPC_LED_GREEN				0x01
	#define WPC_LED_RED					0x02
	#define WPC_LED_ORANGE				0x04

#define WPC_RESET_USB_HUB			0x5704

// USB ON/OFF
#define WPC_SET_USB1_ATTACH			0x5705	/* cradle kit */
#define WPC_SET_USB2_ONOFF			0x5706	/* memory */
#define WPC_SET_USB3_ONOFF			0x5707	/* Felica */
#define WPC_SET_USB4_ONOFF			0x5708	/* external WIFI */
#define WPC_CONNECT_CRADLE			WPC_SET_USB1_ATTACH
#define WPC_CONNECT_USB_MEMORY		WPC_SET_USB2_ONOFF
#define WPC_CONNECT_USB_FELICA		WPC_SET_USB3_ONOFF
#define WPC_CONNECT_USB_WIFI		WPC_SET_USB4_ONOFF

#define WPC_GET_USB2_OVERCUR		0x5709
#define WPC_GET_USB4_OVERCUR		0x570A
#define WPC_GET_USB_MEMORY_OVERCUR	WPC_GET_USB2_OVERCUR
#define WPC_GET_USB_WIFI_OVERCUR	WPC_GET_USB4_OVERCUR

#define WPC_GET_BAT1_CHARGING_STAT	0x570B
#define WPC_GET_BAT2_CHARGING_STAT	0x570C
	#define WPC_CHARGING_FAST			0x01
	#define WPC_CHARGING_FULL			0x02
	#define WPC_CHARGING_FAULT			0x04

#define WPC_GET_BAT1_LEVEL			0x570D
#define WPC_GET_BAT2_LEVEL			0x570E
#define WPC_GET_DC_LEVEL			0x570F

#define WPC_CONNECT_SDIO_WIFI		0x5710

// for WPC V1 only
#define WPC_CONNECT_MMC1			0x5711	/* MMC1 connect/disconnect, default is connect */
#define WPC_SET_TP_ONOFF			0x5712	/* Touch panel on/off, default is on */

// Add ioctl for non-specific GPIO
#define WPC_SET_GPIO_INPUT			0x5713	/* set pin as input */
#define WPC_SET_GPIO_INPUT_PULLUP	0x5714	/* set pin as input with pullup */
#define WPC_SET_GPIO_INPUT_PULLDOWN	0x5715	/* set pin as input with pulldown */
#define WPC_SET_GPIO_OUTPUT_HIGH	0x5716	/* set pin as output, output level = high */
#define WPC_SET_GPIO_OUTPUT_LOW		0x5717	/* set pin as output, output level = low */
#define WPC_GET_GPIO_LEVEL			0x5718	/* get pin level */

#endif
