#ifndef __WPCIO_PIN_H__
#define __WPCIO_PIN_H__

#include <mach/gpio.h>

#define GPIO_PIN_DIPSW1				124
#define GPIO_PIN_DIPSW2				125
#define GPIO_PIN_DIPSW3				126
#define	GPIO_PIN_DIPSW4				127

#define GPIO_PIN_LED_GREEN			17
#define GPIO_PIN_LED_RED			19
#define GPIO_PIN_LED_ORANGE		        20

#define GPIO_PIN_USB_HUB_RESET		        42  //42~56 are useless pin 

#define GPIO_PIN_USB1_OE_N			43
#define GPIO_PIN_USB2_OE_N			44
#define GPIO_PIN_USB3_OE_N			45
#define GPIO_PIN_USB4_OE_N			46

#define GPIO_PIN_USB2_POWER			87
#define GPIO_PIN_USB3_POWER			88
#define GPIO_PIN_USB4_POWER			47
#define GPIO_PIN_TP_POWER			32

#define GPIO_PIN_MMC1_ON_N			49

#define GPIO_PIN_USB2_OVERCUR_N		34
#define GPIO_PIN_USB4_OVERCUR_N		50

#define GPIO_PIN_BAT1_FAULT_N		16
#define GPIO_PIN_BAT1_FASTCHG_N		15
#define GPIO_PIN_BAT1_FULLCHG_N		14

#define GPIO_PIN_BAT2_FAULT_N           51
#define GPIO_PIN_BAT2_FASTCHG_N		52
#define GPIO_PIN_BAT2_FULLCHG_N		53

#define GPIO_PIN_WIFI_PD_N			120
#define GPIO_PIN_WIFI_RESET_N			21
#define GPIO_PIN_MMC1_CD_N			56

#define GPIO_PIN_ADCIN2_SEL			33

#endif
