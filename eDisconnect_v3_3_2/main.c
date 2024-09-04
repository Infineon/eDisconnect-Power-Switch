/*
 * main.c
 *
 *  Created on: 2023 May 03 23:05:58
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

#include "DAVE.h"                 //Declarations from DAVE Code Generation (includes SFR declaration)
#include "DRIVER_2ED4820EM/Driver_2ED4820.h"
#include "DRIVER_2ED4820EM/registers.h"
#include "DRIVER_2ED4820EM/target.h"
#include "eDisconnectEvent.h"
#include "displayData.h"

/***********************************************************************************************************************
 * VARIABLE DECLARATIONS
***********************************************************************************************************************/
volatile uint32_t pulsetick_us = 0;			//


uint32_t TimerId_1;
uint32_t TimerId_2;
uint32_t TimerId_3;
uint32_t TimerId_4;

/***********************************************************************************************************************
 * Main ROUTINES
 **********************************************************************************************************************/
/** Program Entry for initialization and loop to check for UART command received
 *  \param none
 *  \return none
 */
int main(void)
{
  DAVE_STATUS_t status;
  status = DAVE_Init();           /* Initialization of DAVE APPs  */
  if (status != DAVE_STATUS_SUCCESS)
  {
    while(1U)
   {

   }

  }

  // set up System Timers
  SYSTIMER_Init(&SYSTIMER_0);
  // set up 10 ms timer for operation handler
  TimerId_1 = (uint32_t)SYSTIMER_CreateTimer(10001,SYSTIMER_MODE_PERIODIC,(void*)operationHandler,NULL);
  // set up 50 ms timer for GUI handler
  TimerId_2 = (uint32_t)SYSTIMER_CreateTimer(50001,SYSTIMER_MODE_PERIODIC,(void*)GUIHandler,NULL);
  // set up 5 ms timer for fault handler
  TimerId_3 = (uint32_t)SYSTIMER_CreateTimer(5001,SYSTIMER_MODE_PERIODIC,(void*)faultHandler,NULL);
  // set up 100usec for pre charge and freewheeling check
  TimerId_4 = (uint32_t)SYSTIMER_CreateTimer(101,SYSTIMER_MODE_PERIODIC,(void*)intervalTimer,NULL);

  // start timers
  if (TimerId_1 != 0U) SYSTIMER_StartTimer(TimerId_1);
  if (TimerId_2 != 0U) SYSTIMER_StartTimer(TimerId_2);
  if (TimerId_3 != 0U) SYSTIMER_StartTimer(TimerId_3);
  if (TimerId_4 != 0U) SYSTIMER_StartTimer(TimerId_4);

  // set firmware version
  version = 3.3;
  sub_version = 1;

  // inital values
  set_init_config();

  eDisconnectInit();	// Initialize PWM, Interrupt and Timer units
  driverInit();
  initConfig();			// Initialize gate driver register and Pre-charge status values
  ADC_MEASUREMENT_StartConversion(&ADC_MEASUREMENT_0);


  while(1U)
  {

  }

} // end of main


/** Function to configure the initial setting
 *  \param none
 *  \return none
 */
void set_init_config (void)
{

	pre_charge_chk = 1;
	pre_charge_dbg = 1; // set to 1 to enable the pre charge monitoring
	free_wheel_dbg = 0;

	currentSelect = 1; // enable TLE4972
	currentFilterSelect =1; // enable filter for TLE4972
	hall_sensitivity = 25; //mill
	filterLength =50; // average filter length
	currentKI = 0.279; // intial gain 0.279 for gate driver current measurement
	set_r_shunt = 0.0002; // shunt value for gate driver current measurement
	set_flag = 0; // enable Gate driver setting in OFF state

	// intial value for Gate Driver current sense.
	set_OverCurrentThres = CSOCT_0_3; //CSOCT_0_3, 2=+-0.25*VDD
	OVRT = VORT_1000US; //VORT_50US
	UVRT = VURT_1MS; // VURT_1MS
	currentGain = 0; // CSAG Driver setting CSAG_10VV
	set_OverCurrentThres = CSOCT_0_3; //CSOCT_0_3, 2=+-0.25*VDD
	B_DSOV = 7; // DSOT_600MV
	A_DSOV = 7; // DSOT_600MV
	MOSA_Blank = 0; // BT_10US
	MOSB_Blank = 0; //BT_10US
	MOSA_Filter = 3; // FT_5US
	MOSB_Filter = 3; // FT_5US
	XCTR = 1; //CC_ON

	// Auto retry setting
	auto_retry = 0; //
	max_retry = 10;

	//Fault limit setting
	MCU_OV_Vbat =60;
	MCU_UV_Vbat =30;
	MCU_OT_NTCtemperature =100;
	MCU_UT_NTCtemperature = -15;


}

/** Function to start a delay count
 *  \param  ms
 *  \return none
 */
void delay_ms(uint32_t ms){
	uint32_t targetMicroSec = SYSTIMER_GetTime() + (ms * 1000);
	while(targetMicroSec > SYSTIMER_GetTime())
		__NOP(); // do nothing
}

/** Function for debug print
 *  \param *p_frm
 *  \return none
 */
void debugPrintf(const char *p_frm, ...) {

}

