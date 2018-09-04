#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H
#include "v36bml_battery_param.h"


//#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
#define BAT_NTC_10 0
#define BAT_NTC_47 0
#define BAT_NTC_100 1

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             24000
#define NTC_TABLE_SIZE  17
#define ZCV_TABLE_SIZE  78

#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900
#define NTC_TABLE_SIZE  17
#define ZCV_TABLE_SIZE  78

#endif

#if (BAT_NTC_100 == 1)
#define RBAT_PULL_UP_R             100000
#define NTC_TABLE_SIZE  101
#define ZCV_TABLE_SIZE  81
#endif

#define RBAT_PULL_UP_VOLT          1800
#define BATTERY_ID_CHANNEL_NUM 12
#define MTK_GET_BATTERY_ID_BY_AUXADC
#define MTK_MULTI_BAT_PROFILE_SUPPORT

signed int g_battery_id_voltage[] = {500, 700, -1};//0~0.5V for battery 0(ATL), 0.5~0.7V for battery 1(BYD), -1 for the last one (battery 2)
#define TOTAL_BATTERY_NUMBER (sizeof(g_battery_id_voltage) / sizeof(signed int))

/* Qmax for battery  */
signed int g_Q_MAX_POS_50[] = {2150, 2194, 2150};
signed int g_Q_MAX_POS_25[] = {2170, 2194, 2170};
signed int g_Q_MAX_POS_0[] = {1600, 1375, 1600};
signed int g_Q_MAX_NEG_10[] = {926, 850, 926};
signed int g_Q_MAX_POS_50_H_CURRENT[] = {2150, 2194, 2150};
signed int g_Q_MAX_POS_25_H_CURRENT[] = {2170, 2194, 2170};
signed int g_Q_MAX_POS_0_H_CURRENT[] = {1600, 1375, 1600};
signed int g_Q_MAX_NEG_10_H_CURRENT[] = {926, 850, 926};


// ============================================================
// ENUM
// ============================================================

// ============================================================
// structure
// ============================================================

// ============================================================
// typedef
// ============================================================
typedef struct _BATTERY_PROFILE_STRUCT
{
    signed int percentage;
    signed int voltage;
} BATTERY_PROFILE_STRUCT, *BATTERY_PROFILE_STRUCT_P;

typedef struct _R_PROFILE_STRUCT
{
    signed int resistance; // Ohm
    signed int voltage;
} R_PROFILE_STRUCT, *R_PROFILE_STRUCT_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// ============================================================
// External Variables
// ============================================================

// ============================================================
// External function
// ============================================================

// ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================
#if (BAT_NTC_100 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[TOTAL_BATTERY_NUMBER][NTC_TABLE_SIZE] = {
        Batt100_Temperature_Table,
        Batt100_Temperature_Table,
        Batt100_Temperature_Table
    };
#endif

#if (BAT_NTC_47== 1)
    BATT_TEMPERATURE Batt_Temperature_Table[TOTAL_BATTERY_NUMBER][NTC_TABLE_SIZE] = {
         Batt47_Temperature_Table,
         Batt47_Temperature_Table,
         Batt47_Temperature_Table
    };
#endif

#if (BAT_NTC_10== 1)
    BATT_TEMPERATURE Batt_Temperature_Table[TOTAL_BATTERY_NUMBER][NTC_TABLE_SIZE] = {
         Batt10_Temperature_Table,
         Batt10_Temperature_Table,
         Batt10_Temperature_Table
    };
#endif
// T0 -10C
BATTERY_PROFILE_STRUCT battery_profile_t0[TOTAL_BATTERY_NUMBER][ZCV_TABLE_SIZE] =
{
	battery0_profile_t0,
	battery1_profile_t0,
	battery0_profile_t0
};
// T1 0C 
BATTERY_PROFILE_STRUCT battery_profile_t1[TOTAL_BATTERY_NUMBER][ZCV_TABLE_SIZE] =
{
	battery0_profile_t1,
	battery1_profile_t1,
	battery0_profile_t1
};
// T2 25C
BATTERY_PROFILE_STRUCT battery_profile_t2[TOTAL_BATTERY_NUMBER][ZCV_TABLE_SIZE] =
{
	battery0_profile_t2,
	battery1_profile_t2,
	battery0_profile_t2
};
// T3 50C
BATTERY_PROFILE_STRUCT battery_profile_t3[TOTAL_BATTERY_NUMBER][ZCV_TABLE_SIZE] =
{
	battery0_profile_t3,
	battery1_profile_t3,
	battery0_profile_t3
};
// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUCT battery_profile_temperature[] =VALUE_81_ZERO;

// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUCT r_profile_t0[TOTAL_BATTERY_NUMBER][ZCV_TABLE_SIZE] =
{
	battery0_r_profile_t0,
	battery1_r_profile_t0,
	battery0_r_profile_t0
};
// T1 0C
R_PROFILE_STRUCT r_profile_t1[TOTAL_BATTERY_NUMBER][ZCV_TABLE_SIZE] =
{
	battery0_r_profile_t1,
	battery1_r_profile_t1,
	battery0_r_profile_t1
};
// T2 25C
R_PROFILE_STRUCT r_profile_t2[TOTAL_BATTERY_NUMBER][ZCV_TABLE_SIZE] =
{
	battery0_r_profile_t2,
	battery1_r_profile_t2,
	battery0_r_profile_t2
};
// T3 50C
R_PROFILE_STRUCT r_profile_t3[TOTAL_BATTERY_NUMBER][ZCV_TABLE_SIZE] =
{
	battery0_r_profile_t3,
	battery1_r_profile_t3,
	battery0_r_profile_t3
};

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUCT r_profile_temperature[] =VALUE_81_ZERO;

// ============================================================
// function prototype
// ============================================================
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUCT_P fgauge_get_profile(unsigned int temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUCT_P fgauge_get_profile_r_table(unsigned int temperature);

#endif	//#ifndef _CUST_BATTERY_METER_TABLE_H

