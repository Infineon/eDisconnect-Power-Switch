/*
 * displayData.c
 *
 *  Created on: 13 Aug 2023
 */
/* ===========================================================================
** Copyright (C) 2021-2023 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
** ===========================================================================
*/
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "DAVE.h"
#include "DRIVER_2ED4820EM/Driver_2ED4820.h"
#include "DRIVER_2ED4820EM/registers.h"
#include "DRIVER_2ED4820EM/target.h"

#include "displayData.h"
#include "analog.h"
#include "eDisconnectEvent.h"


/***********************************************************************************************************************
 * VARIABLE DECLARATIONS
***********************************************************************************************************************/
uint8_t enableState = 0;

static uint8_t switchBtn = 0;
uint8_t switchState = 0;

uint8_t safeState = 0;

static uint8_t preChargeBtn = 1; //0
uint8_t preChargeState = 1;      //0 

static uint8_t freeWheelBtn = 1;  //0
uint8_t freeWheelState = 1;       //0

static uint8_t	preChargeModeBtn = 0;
uint8_t	preChargeMode = 0;

static uint8_t pulseDurationBtn = 2; //0
uint8_t pulseDurationValue = 2; //0

static uint8_t pwmFreqBtn = 0;
uint8_t	pwmFreqValue = 0;

static uint8_t totalPWMDurationBtn = 0;
uint8_t totalPWMDuration = 0;

static uint32_t freeWheelPeriodBtn = 0;
uint32_t freeWheelPeriod = 0;

static uint32_t freeWheelDutyBtn = 0;
uint32_t freeWheelDuty = 0;


uint8_t Vbat_UV 	= 0;			// This is for GUI display
uint8_t Vbat_OV 	= 0;
uint8_t Vdd_uv 		= 0;
uint8_t OV_temp 	= 0;
uint8_t Vds_OV_A 	= 0;
uint8_t Vgs_UV_A 	= 0;
uint8_t Vds_OV_B 	= 0;
uint8_t Vgs_UV_B 	= 0;
uint8_t OV_current 	= 0;
uint8_t VCMP_UV 	= 0;
uint8_t SAF_EN 		= 0;

//Register list to display
static uint8_t reg0 = 0;					//
static uint8_t reg1 = 0;					//
static uint8_t reg2 = 0;					//
static uint8_t reg3 = 0;					//MOS_CHS_CTRL
static uint8_t reg4 = 0;
static uint8_t reg5 = 0;
static uint8_t reg6 = 0;
static uint8_t reg7 = 0;
static uint8_t reg8 = 0;
static uint8_t reg9 = 0;

//static uint8_t status_Switch = 0;
static uint8_t status_CHR = 0;				// charge status
static uint8_t status_DCHR = 0;				// discharge status
static uint8_t status_CPReady=0;			// Charge pump ready status
static uint8_t status_failure=0;			// failure status

float VbatValue = 0;
float VloadValue = 0;
float IsenseValue = 0;
float HallValue = 0;
int16_t NTCTempValue = 0;

/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
/** Read GUI Data
 *  \param none
 *  \return none
 */
void getGUIdata()
{
	uint32_t cal_period;
	uint32_t cal_duty;
	uint32_t clock = 64000000;

	enableState = 1;	//update Enable button status fix as always on
	switchState = switchBtn;	// update Switch button status
	safeState = 1;	// update safe state button status as always safe

	preChargeState = preChargeBtn;	// Precharge enable button
	freeWheelState = freeWheelBtn;	// freewheel enable button

	preChargeMode = preChargeModeBtn;	//0 = Single pulse , 1 = PWM
	pulseDurationValue = pulseDurationBtn;	// 0 = 100ms, 1 = 250ms, 2 = 500ms

	pwmFreqValue = pwmFreqBtn;		// 0 = 10Hz, 1 = 50Hz, 2 = 100Hz
	totalPWMDuration = totalPWMDurationBtn;	// 0 = 100ms, 1 = 500ms, 2 = 1000ms.


	if (freeWheelPeriodBtn == 0)
		freeWheelPeriodBtn = 1050;

	if (freeWheelDutyBtn == 0)
		freeWheelDutyBtn = 50;

	cal_period = clock * fw_prescaler_value * (freeWheelPeriodBtn * 1e-6); //
	freeWheelPeriod = cal_period -1;

	cal_duty = clock * fw_prescaler_value * (freeWheelDutyBtn * 1e-6); //
	freeWheelDuty = cal_duty -1;

}

/** Send Data to GUI
 *  \param none
 *  \return none
 */
void sendGUIdata()
{

	uint8_t checkVT_fault = 0;

	checkVT_fault = checkFault();
	VbatValue = get_Vbat();
	VloadValue = get_Vload();
	IsenseValue = getBatcurrent();
	NTCTempValue = getNTC_temperature();
	HallValue = getHallcurrent();

	readAllRegistersToCache();

	reg0 = displayRegister(STDIAG);
	reg1 = displayRegister(CHDIAG);
	reg2 = displayRegister(DIAG);
	reg3 = displayRegister(MOS_CHS_CTRL);
	reg4 = displayRegister(FAILURECLEAN);
	reg5 = displayRegister(VDSTHA_B);
	reg6 = displayRegister(MOSFLTBLKA_B);
	reg7 = displayRegister(CSAG_OCTH);
	reg8 = displayRegister(VBATOVUVRST);
	reg9 = displayRegister(RESETS);

	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_A) ==0)
	{ // channel A is off
		status_CHR = 0;
	}
	else
	{
		status_CHR = 1;
	}

	if ((getRegisterCacheData(MOS_CHS_CTRL) & MOSONCH_B) == 0)
	{ // channel B is off
		status_DCHR = 0;
	}
	else
	{
		status_DCHR = 1;
	}


	if ((getRegisterCacheData(STDIAG) & VCP_READY) == 0 || checkVT_fault ==1)
	{
		status_CPReady = 0;
	}
	else
	{
		status_CPReady = 1;
	}

	if ((getRegisterCacheData(STDIAG) & FAILURE) == 0)
	{
		status_failure = 0;
	}
	else
	{
		status_failure = 1;
	}

}

/** Set the Gate Driver current amplifier Gain
 *  \param none
 *  \return none
 */
void guisetCurrentSenseAmpGain()
{
	if (currentGain !=currentGainprev )
	{
		setCurrentSenseAmpGain(currentGain);
		currentGainprev = currentGain;
	}
}

/** Return gate driver fault
 *  \param none
 *  \return status_failure
 */
uint8_t gateDrivefault()
{
	return status_failure;
}

/** Return register values of Gate Driver
 *  \param reg
 *  \return getRegisterCacheData
 */
uint8_t displayRegister(uint8_t reg)
{
	return getRegisterCacheData(reg);
}

/** React to failure from Gate Driver
 *  \param none
 *  \return none
 */
void reactToFailure(){

	readAllRegistersToCache();
	
	if(isFailureFlagActive(Fail_VBAT_UNDERVOLTAGE, USE_REG_CACHED))
	{
		Vbat_UV = 1;
	}
	else
	{
		Vbat_UV    	= 0;
	}

	if(isFailureFlagActive(Fail_VBAT_OVERVOLTAGE, USE_REG_CACHED))
	{
		Vbat_OV = 1;
	}
	else
	{
		Vbat_OV = 0;
	}

	if(isFailureFlagActive(Fail_VDD_UNDERVOLTAGE, USE_REG_CACHED))
	{
		Vdd_uv = 1;
	}
	else
	{
		Vdd_uv = 0;
	}

	if(isFailureFlagActive(Fail_CHIP_OVERTEMPERATURE, USE_REG_CACHED))
	{
		OV_temp = 1;
	}
	else
	{
		OV_temp 	= 0;
	}

	if(isFailureFlagActive(Fail_VDS_OVERVOLTAGE_A, USE_REG_CACHED))
	{
		Vds_OV_A = 1;
	}
	else
	{
		Vds_OV_A = 0;
	}

	if(isFailureFlagActive(Fail_VGS_UNDERVOLTAGE_A, USE_REG_CACHED))
	{
		Vgs_UV_A = 1;
	}
	else
	{
		Vgs_UV_A 	= 0;
	}

	if(isFailureFlagActive(Fail_VDS_OVERVOLTAGE_B, USE_REG_CACHED))
	{
		Vds_OV_B = 1;
	}
	else
	{
		Vds_OV_B = 0;
	}

	if(isFailureFlagActive(Fail_VGS_UNDERVOLTAGE_B, USE_REG_CACHED))
	{
		Vgs_UV_B = 1;
	}
	else
	{
		Vgs_UV_B = 0;
	}

	if(isFailureFlagActive(Fail_OVERCURRENT, USE_REG_CACHED))
	{
		OV_current = 1;

	}
	else
	{
		OV_current 	= 0;
	}

	if(isFailureFlagActive(Fail_CHARGEPUMP_UNDERVOLTAGE, USE_REG_CACHED))
	{
		VCMP_UV = 1;
	}
	else
	{
		VCMP_UV = 0;
	}

	if(isFailureFlagActive(Fail_SAVESTATE_ENABLED, USE_REG_CACHED))
	{
		SAF_EN = 1;
	}
	else
	{
		SAF_EN 	= 0;
	}
}

/** React to warning from Gate Driver
 *  \param none
 *  \return none
 */
void reactToWarningMonitoring(){	// Not implemented in v2.2

	if(isWarningFlagActive(Warn_OVERTEMP, USE_REG_CACHED));

	if(isWarningFlagActive(Warn_MEMFAIL, USE_REG_CACHED));

	if(isWarningFlagActive(Warn_LOSSOFGROUND_CP, USE_REG_CACHED));

	if(isWarningFlagActive(Warn_LOSSOFGROUND_D, USE_REG_CACHED));

	if(isWarningFlagActive(Warn_LOSSOFGROUND_A, USE_REG_CACHED));

	if(isMonitoringFlagActive(Monitor_SPI_ADDRESS_NOT_AVAIL, USE_REG_CACHED));

	if(isMonitoringFlagActive(Monitor_SOURCE_OVERVOLTAGE_A, USE_REG_CACHED));

	if(isMonitoringFlagActive(Monitor_SOURCE_OVERVOLTAGE_B, USE_REG_CACHED));

}

/** Return enable state
 *  \param none
 *  \return enableState
 */
uint8_t getEnableState()
{
	return enableState;
}

/** Return Switch state
 *  \param none
 *  \return switchState
 */
uint8_t getSwitchState()
{
	return switchState;
}

/** Return Safe state
 *  \param none
 *  \return safeState
 */
uint8_t getSafeState()
{
	return safeState;
}

/** Return precharge state
 *  \param none
 *  \return preChargeState
 */
uint8_t getPrechargeState()
{
	return preChargeState;
}

/** Return freewheeling state
 *  \param none
 *  \return freeWheelState
 */
uint8_t getfreeWheelState()
{
	return freeWheelState;
}
/** Return freewheeling period
 *  \param none
 *  \return freeWheelPeriod
 */
uint32_t getfreeWheelperiod()
{
	return freeWheelPeriod;
}
/** Return freewheeling duty cycle
 *  \param none
 *  \return freeWheelDuty
 */
uint32_t getfreeWheelcompare()
{
	return freeWheelDuty;
}
/** Return precharge PWM type, multiple pulse or single pulse
 *  \param none
 *  \return preChargeMode
 */
uint8_t getPreChargeMode()
{
	return preChargeMode;
}
/** Return precharge numbe of pulse
 *  \param none
 *  \return pulseDurationValue
 */
uint8_t getPulseDuration()
{
	return pulseDurationValue;
}
/**  Return PWM frequency duration for precharge
 *  \param none
 *  \return pwmFreqValue
 */
uint8_t getpwmFreq()
{
	return pwmFreqValue;
}
/** Return PWM duration for precharge
 *  \param none
 *  \return totalPWMDuration
 */
uint8_t getPWMDuration()
{
	return totalPWMDuration;
}



