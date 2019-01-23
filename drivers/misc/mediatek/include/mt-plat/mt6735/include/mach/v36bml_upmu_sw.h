#ifndef _MT_PMIC_UPMU_SW_H_
#define _MT_PMIC_UPMU_SW_H_

#define AUXADC_SUPPORT_IMM_CURRENT_MODE
//#define BATTERY_CDP_WORKAROUND

#if !defined(MTK_PLUG_OUT_DETECTION)
#define MTK_PLUG_OUT_DETECTION
#endif 

//==============================================================================
// Low battery level define
//==============================================================================
typedef enum LOW_BATTERY_LEVEL_TAG 
{
    LOW_BATTERY_LEVEL_0 = 0,
    LOW_BATTERY_LEVEL_1 = 1,
    LOW_BATTERY_LEVEL_2 = 2
} LOW_BATTERY_LEVEL;

typedef enum LOW_BATTERY_PRIO_TAG 
{
    LOW_BATTERY_PRIO_CPU_B      = 0,
    LOW_BATTERY_PRIO_CPU_L      = 1,
    LOW_BATTERY_PRIO_GPU        = 2,    
    LOW_BATTERY_PRIO_MD         = 3,
    LOW_BATTERY_PRIO_MD5        = 4,
    LOW_BATTERY_PRIO_FLASHLIGHT = 5,
    LOW_BATTERY_PRIO_VIDEO      = 6,
    LOW_BATTERY_PRIO_WIFI       = 7,
    LOW_BATTERY_PRIO_BACKLIGHT  = 8
} LOW_BATTERY_PRIO;

extern void (*low_battery_callback)(LOW_BATTERY_LEVEL);
extern void register_low_battery_notify( void (*low_battery_callback)(LOW_BATTERY_LEVEL), LOW_BATTERY_PRIO prio_val );


//==============================================================================
// Battery OC level define
//==============================================================================
typedef enum BATTERY_OC_LEVEL_TAG 
{
    BATTERY_OC_LEVEL_0 = 0,
    BATTERY_OC_LEVEL_1 = 1    
} BATTERY_OC_LEVEL;

typedef enum BATTERY_OC_PRIO_TAG 
{
    BATTERY_OC_PRIO_CPU_B      = 0,
    BATTERY_OC_PRIO_CPU_L      = 1,
    BATTERY_OC_PRIO_GPU        = 2
} BATTERY_OC_PRIO;

extern void (*battery_oc_callback)(BATTERY_OC_LEVEL);
extern void register_battery_oc_notify( void (*battery_oc_callback)(BATTERY_OC_LEVEL), BATTERY_OC_PRIO prio_val );

//==============================================================================
// Battery percent define
//==============================================================================
typedef enum BATTERY_PERCENT_LEVEL_TAG 
{
    BATTERY_PERCENT_LEVEL_0 = 0,
    BATTERY_PERCENT_LEVEL_1 = 1
} BATTERY_PERCENT_LEVEL;

typedef enum BATTERY_PERCENT_PRIO_TAG 
{
    BATTERY_PERCENT_PRIO_CPU_B      = 0,
    BATTERY_PERCENT_PRIO_CPU_L      = 1,
    BATTERY_PERCENT_PRIO_GPU        = 2,    
    BATTERY_PERCENT_PRIO_MD         = 3,
    BATTERY_PERCENT_PRIO_MD5        = 4,
    BATTERY_PERCENT_PRIO_FLASHLIGHT = 5,
    BATTERY_PERCENT_PRIO_VIDEO      = 6,
    BATTERY_PERCENT_PRIO_WIFI       = 7,
    BATTERY_PERCENT_PRIO_BACKLIGHT  = 8
} BATTERY_PERCENT_PRIO;

extern void (*battery_percent_callback)(BATTERY_PERCENT_LEVEL);
extern void register_battery_percent_notify( void (*battery_percent_callback)(BATTERY_PERCENT_LEVEL), BATTERY_PERCENT_PRIO prio_val );

/* ADC Channel Number */
typedef enum {
	//MT6328
	
	MT6328_AUX_BATSNS_AP =		0x000,
	MT6328_AUX_ISENSE_AP,
	MT6328_AUX_VCDT_AP,
	MT6328_AUX_BATON_AP,
	MT6328_AUX_CH4,
	MT6328_AUX_VACCDET_AP,
	MT6328_AUX_CH6,
	MT6328_AUX_TSX,
	MT6328_AUX_CH8,
	MT6328_AUX_CH9,	
	MT6328_AUX_CH10,
	MT6328_AUX_CH11,
	MT6328_AUX_CH12,
	MT6328_AUX_CH13,
	MT6328_AUX_CH14,
	MT6328_AUX_CH15,	
} mt6328_adc_ch_list_enum;

//==============================================================================
// DLPT define
//==============================================================================
typedef enum DLPT_PRIO_TAG 
{
    DLPT_PRIO_PBM        = 0,
    DLPT_PRIO_CPU_B      = 1,
    DLPT_PRIO_CPU_L      = 2,
    DLPT_PRIO_GPU        = 3,    
    DLPT_PRIO_MD         = 4,
    DLPT_PRIO_MD5        = 5,
    DLPT_PRIO_FLASHLIGHT = 6,
    DLPT_PRIO_VIDEO      = 7,
    DLPT_PRIO_WIFI       = 8,
    DLPT_PRIO_BACKLIGHT  = 9
} DLPT_PRIO;

extern void (*dlpt_callback)(unsigned int);
extern void register_dlpt_notify( void (*dlpt_callback)(unsigned int), DLPT_PRIO prio_val);

//==============================================================================
// PMIC LDO define
//==============================================================================



 



#endif // _MT_PMIC_UPMU_SW_H_

