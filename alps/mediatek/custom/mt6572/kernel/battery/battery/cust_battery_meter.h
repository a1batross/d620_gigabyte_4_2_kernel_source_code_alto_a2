#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
//#define SOC_BY_HW_FG
#define SOC_BY_SW_FG

/* ADC Channel Number */
#define CUST_TABT_NUMBER 17
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5

/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

#define FG_METER_RESISTANCE 	0

/* Qmax for battery  */
//D620 battery capacity is 2000mah
#if defined (RGK_D620_BATTERY) 
#define Q_MAX_POS_50	1865
#define Q_MAX_POS_25	1950
#define Q_MAX_POS_0		1836
#define Q_MAX_NEG_10	1649

#define Q_MAX_POS_50_H_CURRENT	1836
#define Q_MAX_POS_25_H_CURRENT	1950
#define Q_MAX_POS_0_H_CURRENT	1662
#define Q_MAX_NEG_10_H_CURRENT	1092
#elif defined (RGK_D208_BATTERY)//D208 battery Qmax = 1400mah
#define Q_MAX_POS_50	1320
#define Q_MAX_POS_25	1350
#define Q_MAX_POS_0		1320
#define Q_MAX_NEG_10	1237

#define Q_MAX_POS_50_H_CURRENT	1327
#define Q_MAX_POS_25_H_CURRENT	1300
#define Q_MAX_POS_0_H_CURRENT	1247
#define Q_MAX_NEG_10_H_CURRENT	819
#else
#define Q_MAX_POS_50	1399
#define Q_MAX_POS_25	1499
#define Q_MAX_POS_0		1377
#define Q_MAX_NEG_10	1237

#define Q_MAX_POS_50_H_CURRENT	1377
#define Q_MAX_POS_25_H_CURRENT	1473
#define Q_MAX_POS_0_H_CURRENT	1247
#define Q_MAX_NEG_10_H_CURRENT	819
#endif

/* Discharge Percentage */
#define OAM_D5		 0		//  1 : D5,   0: D2


/* battery meter parameter */
#define CUST_TRACKING_POINT  14
#define CUST_R_SENSE         220//pengjinlong adjust this 200 -> 220 depend on PCB
#define CUST_HW_CC 		     0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#define CAR_TUNE_VALUE		94 //1.00


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			0 // mOhm, base is 20



/* Disable Battery check for HQA */
#ifdef MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

#endif	//#ifndef _CUST_BATTERY_METER_H
