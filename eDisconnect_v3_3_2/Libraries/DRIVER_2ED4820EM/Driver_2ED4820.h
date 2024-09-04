/*
 * commands.h
 *
 *	Library for the EiceDRIVER APD 2ED4820-EM 48 V smart high-side MOSFET gate driver.
 *	Created by Rene Santeler @ MCI EAL 2022 (based on sample code from schwarzg created on 4 Mar 2020)
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#define USE_REG_CACHED 1
#define USE_REG_RENEW  0



// Argument enumerations for set functions. See function descriptions for details
typedef enum {CSAG_10VV, CSAG_15VV, CSAG_20VV, CSAG_25VV, CSAG_31_5VV, CSAG_35VV, CSAG_40VV, CSAG_47_7VV} CurrentSenseAmpGain_state;
typedef enum {CSOCT_0_1, CSOCT_0_2, CSOCT_0_25, CSOCT_0_3} CurrentSenseOverCurrentThres_state;
typedef enum {VURT_1MS, VURT_5MS, VURT_20MS, VURT_50MS} VbatUndervoltageRestartTime_state;
typedef enum {VORT_10US, VORT_50US, VORT_200US, VORT_1000US} VbatOvervoltageRestartTime_state;
typedef enum {DSOT_100MV, DSOT_150MV, DSOT_200MV, DSOT_250MV, DSOT_300MV, DSOT_400MV, DSOT_500MV, DSOT_600MV, DSOT_IGNORE} DrainSourceOvervoltageThres_state;
typedef enum {BT_10US, BT_20US, BT_50US, BT_100US, BT_IGNORE} BlankTime_state;
typedef enum {FT_0_5US, FT_1US, FT_2US, FT_5US, FT_IGNORE} FilterTime_state;
typedef enum {CC_OFF, CC_ON} CrossControl_state;
typedef enum {DSOD_ENABLED, DSOD_DISABLED, DSOD_IGNORE} DrainSourceOvervoltageDeactivation_state;
typedef enum {CSSC_LOWSIDE, CSSC_HIGHSIDE} CurrentSenseShuntConfig_state;
typedef enum {CSLA_LT100PF, CSLA_GT100PF} CurrentSenseLoadAdjust_state;

// Argument enumerations for "is...FlagActive()" functions. Representation of an Flag in register
typedef enum {Fail_VBAT_UNDERVOLTAGE = 1, Fail_VBAT_OVERVOLTAGE = 2, Fail_VDD_UNDERVOLTAGE = 4, Fail_CHIP_OVERTEMPERATURE = 8, Fail_VDS_OVERVOLTAGE_A = 16, Fail_VGS_UNDERVOLTAGE_A = 32, Fail_VDS_OVERVOLTAGE_B = 64, Fail_VGS_UNDERVOLTAGE_B = 128, Fail_OVERCURRENT = 256, Fail_CHARGEPUMP_UNDERVOLTAGE = 512, Fail_SAVESTATE_ENABLED = 1024} FailureFlags;
typedef enum {Warn_LOSSOFGROUND_CP = 1, Warn_LOSSOFGROUND_D = 2, Warn_LOSSOFGROUND_A = 4, Warn_OVERTEMP = 8, Warn_MEMFAIL = 16} WarningFlags;
typedef enum {Monitor_SPI_ADDRESS_NOT_AVAIL = 32, Monitor_SOURCE_OVERVOLTAGE_A = 64, Monitor_SOURCE_OVERVOLTAGE_B = 128, Monitor_CHARGEPUMPR_READY = 256} MonitoringFlags;

// Driver state control
void driverInit(void);
void setEnable(uint8_t state);
void setSafestateEnable(uint8_t state);

// Use this beforehand to utilize "useCachedReg=true" parameter
void readAllRegistersToCache(void);

// Channel control
void switchOn(uint8_t useCachedReg);
void switchOff(uint8_t useCachedReg);
uint8_t switchGetState(uint8_t useCachedReg);
void setChannelA(uint8_t state, uint8_t check, uint8_t useCachedReg);
void setChannelB(uint8_t state, uint8_t check, uint8_t useCachedReg);
void burstChannelA(uint32_t numPulses, uint8_t useCachedReg);
void burstChannelB(uint32_t numPulses, uint8_t useCachedReg);

// Settings
void setCurrentSenseAmpGain(CurrentSenseAmpGain_state CSAG);
void setCurrentSenseOverCurrentThres(CurrentSenseOverCurrentThres_state CSOCT);
void setVbatUndervoltageRestartTime(VbatUndervoltageRestartTime_state VURT);
void setVbatOvervoltageRestartTime(VbatOvervoltageRestartTime_state VORT);
void setDrainSourceOvervoltageThres(DrainSourceOvervoltageThres_state DSOT_ChA, DrainSourceOvervoltageThres_state DSOT_ChB);
void setDrainSourceOvervoltageDeactivation(DrainSourceOvervoltageDeactivation_state DSOD_A, DrainSourceOvervoltageDeactivation_state DSOD_B);
void setBlankTime(BlankTime_state BT_ChA, BlankTime_state BT_ChB);
void setFilterTime(FilterTime_state FT_ChA, FilterTime_state FT_ChB);
void setCrossControl(CrossControl_state CC);
void setCurrentSenseShuntConfig(CurrentSenseShuntConfig_state CSSC);
void setCurrentSenseLoadAdjust(CurrentSenseLoadAdjust_state CSLA);

// Debug functions
void printConfig(uint8_t useCachedReg);
void printAllRegisters(void);

// Used to check if an action was successful (e.g. switchON()) and if failure flags are set
//uint8_t problemDetected;
uint8_t isProblemDetected();
void clearProblemDetected();

// Use these to check if Failures, Warnings or Monitoring Flags are active (e.g. during interrupt routine)
uint32_t getFailureBitmap(uint8_t useCachedReg);
uint32_t getWarningBitmap(uint8_t useCachedReg);
uint32_t getInfoBitmap(uint8_t useCachedReg);

uint8_t isFailureFlagActive(FailureFlags Fail, uint8_t useCachedReg);
uint8_t isWarningFlagActive(WarningFlags Warn, uint8_t useCachedReg);
uint8_t isMonitoringFlagActive(MonitoringFlags Monitor, uint8_t useCachedReg);
void printStatusFlags(uint8_t useCachedReg);
void clearStatusFlags(void);

//// Functions to control pre-charge path [NOT IMPLEMENTED]
//void switchOnWithPrecharge(void);
//void setPrechargeTime(char *arg);
//uint16_t getPrechargeTime(void);
//void ISR_timerPrecharge(void);








#endif /* COMMANDS_H_ */
