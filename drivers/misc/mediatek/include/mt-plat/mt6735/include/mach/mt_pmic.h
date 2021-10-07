#ifndef _CUST_PMIC_H_
#define _CUST_PMIC_H_

/*#define PMIC_VDVFS_CUST_ENABLE*/

#define LOW_POWER_LIMIT_LEVEL_1 15

/*
//Define for disable low battery protect feature, default no define for enable low battery protect.
//#define DISABLE_LOW_BATTERY_PROTECT

//Define for disable battery OC protect
//#define DISABLE_BATTERY_OC_PROTECT

//Define for disable battery 15% protect
//#define DISABLE_BATTERY_PERCENT_PROTECT
*/
/*Define for DLPT*/
/*#define DISABLE_DLPT_FEATURE*/
#define POWER_UVLO_VOLT_LEVEL 2600
#define IMAX_MAX_VALUE 5500

#define POWER_INT0_VOLT 3400
#define POWER_INT1_VOLT 3250
#define POWER_INT2_VOLT 3100

#if defined(CONFIG_ARCH_MT6753)
#define POWER_BAT_OC_CURRENT_H    4670
#define POWER_BAT_OC_CURRENT_L    5500
#define POWER_BAT_OC_CURRENT_H_RE 4670
#define POWER_BAT_OC_CURRENT_L_RE 5500
#else
#define POWER_BAT_OC_CURRENT_H    3400
#define POWER_BAT_OC_CURRENT_L    4000
#define POWER_BAT_OC_CURRENT_H_RE 3400
#define POWER_BAT_OC_CURRENT_L_RE 4000
#endif

//begin removed by liuwenbo@yulong.com for suddenly power off with low battery voltage
//#ifdef CONFIG_V36BML_BATTERY
//#define DLPT_POWER_OFF_EN
//#endif
//end removed by liuwenbo@yulong.com for suddenly power off with low battery voltage
#define POWEROFF_BAT_CURRENT 3000
#define DLPT_POWER_OFF_THD 100

//#define BATTERY_MODULE_INIT
#define MTK_BQ24296_SUPPORT //add by liuwenbo@yulong.com 2015.05.04 to support bq24296

/* ADC Channel Number */
enum {
	AUX_BATSNS_AP =	0x000,
	AUX_ISENSE_AP,
	AUX_VCDT_AP,
	AUX_BATON_AP,
	AUX_TSENSE_AP,
	AUX_TSENSE_MD =	0x005,
	AUX_VACCDET_AP = 0x007,
	AUX_VISMPS_AP =	0x00B,
	AUX_ICLASSAB_AP =	0x016,
	AUX_HP_AP =	0x017,
	AUX_CH10_AP =	0x018,
	AUX_VBIF_AP =	0x019,
	AUX_CH0_6311 = 0x020,
	AUX_CH1_6311 = 0x021,
	AUX_ADCVIN0_MD = 0x10F,
	AUX_ADCVIN0_GPS = 0x20C,
	AUX_CH12 = 0x1011,
	AUX_CH13 = 0x2011,
	AUX_CH14 = 0x3011,
	AUX_CH15 = 0x4011,
};

#endif /* _CUST_PMIC_H_ */
