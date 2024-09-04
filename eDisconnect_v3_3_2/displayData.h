/*
 * displayData.h
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

#ifndef DISPLAYDATA_H_
#define DISPLAYDATA_H_

/***********************************************************************************************************************
 * VARIABLE DECLARATIONS
***********************************************************************************************************************/
float version;
float sub_version;

// Curent(A) Setting
uint8_t currentGain;
uint8_t currentGainprev;
uint8_t filterLength;
uint8_t avgLength;
double currentGainshunt;
float currentKI;
double set_r_shunt;
uint8_t currentSelect;
uint8_t currentFilterSelect;
double hall_sensitivity;

uint8_t pre_charge_chk;
uint8_t pre_charge_dbg;
uint8_t free_wheel_dbg;

float fw_prescaler_value;
float dly_prescaler_value;

// System Fault
uint8_t auto_retry;
uint8_t retry_count;
uint8_t max_retry;
float MCU_OV_Vbat;
float MCU_UV_Vbat;
float OC_Vbat;
int16_t MCU_OT_NTCtemperature;
int16_t MCU_UT_NTCtemperature;

//GateDriver Setting
uint8_t set_flag;
uint8_t MOSA_Blank;
uint8_t MOSB_Blank;
uint8_t MOSA_Filter;
uint8_t MOSB_Filter;
uint8_t set_OverCurrentThres;
uint8_t B_DSOV;
uint8_t A_DSOV;
uint8_t UVRT;
uint8_t OVRT;
uint8_t XCTR;

/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
void getGUIdata(void);
void guisetCurrentSenseAmpGain();

void sendGUIdata(void);
uint8_t displayRegister(uint8_t reg);
void reactToFailure(void);
void reactToWarningMonitoring(void);

uint8_t getEnableState(void);
uint8_t getSwitchState(void);
uint8_t getSafeState(void);

uint8_t getPrechargeState(void);
uint8_t getfreeWheelState(void);
uint32_t getfreeWheelperiod(void);
uint32_t getfreeWheelcompare(void);
uint8_t getPreChargeMode(void);
uint8_t getPulseDuration(void);
uint8_t getpwmFreq(void);
uint8_t getPWMDuration(void);
uint16_t getCurrentSliderGain(void);

void fault_SWOFF(void);
uint8_t gateDrivefault(void);



#endif /* DISPLAYDATA_H_ */



