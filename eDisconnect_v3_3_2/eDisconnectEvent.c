/*
 * eDisconnectEvent.c
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

#include "DAVE.h"                 //Declarations from DAVE Code Generation (includes SFR declaration)
#include "DRIVER_2ED4820EM/Driver_2ED4820.h"
#include "DRIVER_2ED4820EM/registers.h"
#include "DRIVER_2ED4820EM/target.h"
#include "analog.h"
#include "displayData.h"
#include "eDisconnectEvent.h"

/***********************************************************************************************************************
 * VARIABLE DECLARATIONS
***********************************************************************************************************************/
volatile uint8_t interruptDetected = 0; 	// error interrupt detection from 2ED4820 gate driver
volatile uint32_t led_blink_ms = 0;	 		// LED blinking time in 1 millisecond counter
uint8_t led_state = 0;
uint8_t vfault;
uint8_t tfault;
uint8_t prefault;
uint8_t sysFaultDetected = 0; 	//System fault pin
static uint8_t opState = 0;
uint8_t enableCheck;
uint8_t switchCheck;
uint8_t safeStateCheck;
uint8_t uvFault, ovFault,utFault,otFault;
uint8_t pcComplete = 0;
uint8_t pcStart = 0;
uint8_t pcFlag = 0;

uint8_t redLED	= 0;
uint8_t greenLED = 0;

uint8_t fwComplete=0;
uint8_t fwStart =0;

uint8_t tleFault_OCD2 = 0;

uint8_t tleFault = 0;
uint8_t tleFault_EN= 1;

uint8_t pc_EN;
uint8_t pcMode;
uint8_t pcFreq;
uint8_t pcTime;

uint8_t pcPulse;

uint8_t fwEN;

uint8_t maxPulse;

uint8_t uvLFault=0; // v2.2

uint8_t vSucceed=0; //v2.2

static float cf_Vbat = 0;
static int16_t cf_NTCtemperature = 0;

uint8_t prb_f = 0;
uint8_t gateDriverFault = 0; //v2.2.1

/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/

/** Operation Handler
 * gets called every 10 ms by SYSTIMER
 *  \param none
 *  \return none
 */
void operationHandler()	// call at every 10 ms
{
	measureProcess(); // adc measurement calculation
	operationState(); // different state
}


/** GUI Handler
 * gets called every 50 ms by SYSTIMER
 *  \param none
 *  \return none
 */
void GUIHandler() // call at every 50 ms
{
	getGUIdata();
	sendGUIdata();
}


/** Fault Handler
 * gets called every 5 ms by SYSTIMER
 *  \param none
 *  \return none
 */
void faultHandler() // call at every 5 ms
{
	sysFaultDetected = checkFault();
	led_blink_ms++;
	blinkLED();

	if(interruptDetected==1) //
	{
		interruptDetected = 0; // set interrupt detected to 0
		gateDriverFault = isProblemDetected(); //update gatedriver fault after clear.
		reactToFailure(); // update GUI LED
	}
	else
	{
		gateDriverFault =isProblemDetected(); // update gatedriver status
	}

}

/** intervalTimer handle to check OCD fault pin
 * gets called every 100 us by SYSTIMER
 *  \param none
 *  \return none
 */
void intervalTimer(void) // call at every 100usec
{
	uint32_t OCD2_LOGIC = DIGITAL_IO_GetInput(&OCD2);

	if (OCD2_LOGIC !=  1)
	{
		tleFault_OCD2 = 1;
		switchOff(0);
	}
	else
	{
		tleFault_OCD2 = 0;
	}

	load_voltage_check();
}

/** 2ED4820 Gate Driver fault interrupt pin
 *  \param none
 *  \return none
 */
void Fault_IRQHandler(void)	  // 2ED4820 Gate Driver fault interrupt pin
{
	interruptDetected = 1; // raise trigger that interrupt come from gate driver.
}


/** reset pin interrupt
 *  \param none
 *  \return none
 */
void ResetIRQHandler (void)
{
	NVIC_SystemReset();
}


/** Pre-charge interrupt for pulse count and turn on or off the gate driver
 *  \param none
 *  \return none
 */
void UserIRQHandler(void)	// PC_Ctrl Interrupt
{
	PWM_ClearEvent(&PC_Ctrl, PWM_INTERRUPT_PERIODMATCH);
	pulse_count++; // count after each output pulse.
	float loadVolt = get_Vload();					//load voltage value to GUI
	float batteryVolt = get_Vbat() * 0.98;

	if(pulse_count >= maxPulse) // VL measurement more than that switch on if not fault state.
	{
		pcStart = 0; // reset to prevent monitoring by vload.
		INTERRUPT_Disable(&INTERRUPT_1); // disable this interrupt
		PWM_SetDutyCycle(&PC_Ctrl, 0);
		PWM_Stop(&PC_Ctrl);

		if (pre_charge_chk == 1) // temporary setting to enable/disable pre charge voltage check.
		{
			if (loadVolt >= batteryVolt)
			{
				pcFlag = 2; // detect pre-charge off by which channel
				switchOn(0);
			}
			else // if pre-charge fault happened
			{
				switchOff(0);
				prefault = 1; // trigger fault in check fault
			}
		}
		else // no pre charge voltage check
		{
			switchOn(0);
		}
		pcComplete = 1; // indicate pre-charge complete
		pulse_count = 0; // for pulse count when have pulse effect/
	}
}


/** Freewheeling Interrupt when pin goes high. after blanking time
 *  \param none
 *  \return none
 */
void FWIRQHandler (void)
{
	INTERRUPT_Disable(&FW_INTERRUPT);
	fwStart = 1; // enable monitoring by Vload
	INTERRUPT_Enable(&FW_INTERRUPT);


}

/** Freewheeling Interrupt when end, pin goes low
 *  \param none
 *  \return none
 */
void FWSTOPIRQHandler (void)
{
	INTERRUPT_Disable(&FW_STOP_INTERRUPT);
	PWM_CCU4_Stop(&FW_Ctrl);
	fwStart = 0; // disable monitoring by Vload
	INTERRUPT_Enable(&FW_STOP_INTERRUPT);
}


/** TLE4972 OCD detection
 *  \param none
 *  \return none
 */
void OCDIRQHandler (void)
{
	if (tleFault_EN == 1)
	{
		switchOff(0);
		tleFault = 1;
	}

}

/** Initialize system setting
 *  \param none
 *  \return none
 */
void eDisconnectInit()
{

	  PIN_INTERRUPT_Init(&PIN_INT_DRIVER);
	  PIN_INTERRUPT_Enable(&PIN_INT_DRIVER);

	  PWM_Init(&PC_Ctrl);
	  PWM_SetFreq(&PC_Ctrl, 100U);
	  PWM_SetDutyCycle(&PC_Ctrl, 5000);
	  PWM_Start(&PC_Ctrl);
	  delay_ms(10);
	  PWM_Stop(&PC_Ctrl);

	  INTERRUPT_Init(&INTERRUPT_1);
	  INTERRUPT_Enable(&INTERRUPT_1);
	  delay_ms(10);
	  INTERRUPT_Disable(&INTERRUPT_1);

	  INTERRUPT_Enable(&FW_INTERRUPT);
	  INTERRUPT_Enable(&FW_STOP_INTERRUPT);

	  fw_prescaler_value = 1<<FW_Ctrl.config_ptr->ccu4_cc4_slice_timer_ptr->prescaler_initval;
	  fw_prescaler_value =  1/fw_prescaler_value;
}


/** Initialize Gate Driver setting
 *  \param none
 *  \return none
 */
void initConfig()
{
	  DIGITAL_IO_SetOutputHigh(&PX_LED_R);
	  DIGITAL_IO_SetOutputHigh(&PX_LED_G);

	  setVbatOvervoltageRestartTime(OVRT);  //VORT_10US, VORT_50US, VORT_200US, VORT_1000US
	  setCrossControl(XCTR);
	  currentGainprev = currentGain;
	  setCurrentSenseAmpGain(currentGain); // set using default value
	  setCurrentSenseOverCurrentThres(set_OverCurrentThres); //

	  uvFault = 0;
	  ovFault = 0;
	  utFault = 0;
	  otFault = 0;
	  prefault = 0;

	  opState = 0;
	  pcComplete = 0;
	  pulse_count = 0;
	  fwComplete = 0;
	  pcFlag = 0;
	  retry_count = 0;
}


/** Set Gate Driver
 *  \param none
 *  \return none
 */
void configSetting(void)
{
	if (set_flag ==1)
	{
		set_flag = 0;
		setCurrentSenseOverCurrentThres(set_OverCurrentThres);
		setVbatOvervoltageRestartTime(OVRT);
		setVbatUndervoltageRestartTime(UVRT);
		setDrainSourceOvervoltageThres(A_DSOV,B_DSOV);
		setBlankTime(MOSA_Blank,MOSB_Blank);
		setFilterTime(MOSA_Filter, MOSB_Filter);
	}
}

/** Get state machine state
 *  \param none
 *  \return none
 */
void operationState()
{
	switch(opState)
	{
		case 0:
			op_init_state();
			break;

		case 1:
			op_OFF_state();
			break;

		case 2:
			op_ON_state();
			break;

		case 3:
			op_Fault_state();
			break;

	}
}

/** Init State
 *  \param none
 *  \return none
 */
void op_init_state()	//opState = 0
{
	// In this state, 2ED4820 gate driver is set to sleep when Enable = 0;
	uint8_t EN,SFST;

	prefault = 0;
	EN   = getEnableState();
	SFST = getSafeState();

	prb_f = isProblemDetected();
	led_state = 0;

	if(SFST == 0 ||gateDriverFault == 1||isProblemDetected()==1||sysFaultDetected==1)
	{
		opState = 3;	//go to Fault state
	}
	else	//SFST = 1
	{
		if(EN ==0)
		{
			opState = 0;	//stay in Init state
			DIGITAL_IO_SetOutputLow(&IO_DRIVER_ENABLE);
		}
		else
		{
			opState = 1;	//go to SW OFF State for Switch OFF
			DIGITAL_IO_SetOutputHigh(&IO_DRIVER_ENABLE);
		}
	}
	pcComplete = 0;
}

/** OFF State
 *  \param none
 *  \return none
 */
void op_OFF_state()	//opState = 1
{
	uint8_t EN,SFST,SW;
	uint16_t period, compare;

	EN   = getEnableState();
	SFST = getSafeState();
	SW  = getSwitchState();
	prb_f = isProblemDetected();

	fwEN = getfreeWheelState();
	period =  (uint16_t) getfreeWheelperiod();
	compare = (uint16_t) getfreeWheelcompare();

	guisetCurrentSenseAmpGain(); //update default value during off state base on GUI

	configSetting();

	//MOSFET OFF here ---- Which freeWheeling check
	if(fwEN==1)
	{
		if(fwComplete==0)
		{
			pcFlag = 0;
			PWM_Stop(&PC_Ctrl);
			//switchOff(0);
			INTERRUPT_Enable(&FW_INTERRUPT);
			INTERRUPT_Enable(&FW_STOP_INTERRUPT);
			setChannelA(0,0,0);
			setChannelB(0,0,0);
			XMC_CCU4_SLICE_SetTimerPeriodMatch(FW_Ctrl.ccu4_slice_ptr, period);
			XMC_CCU4_SLICE_SetTimerCompareMatch(FW_Ctrl.ccu4_slice_ptr, compare);
			XMC_CCU4_EnableShadowTransfer(FW_Ctrl.ccu4_module_ptr, FW_Ctrl.shadow_txfr_msk);
			PWM_CCU4_Start(&FW_Ctrl);
			fwStart = 0;
			fwComplete = 1;
			prefault = 0;
		}
		else
		{

		}
	}
	else
	{
		switchOff(0);
		prefault = 0;
	}

	led_state = 1;

	if(SFST == 0 ||gateDriverFault == 1||isProblemDetected()==1||sysFaultDetected==1)
	{
		opState = 3;
	}
	else
	{
		if(EN==0)
		{
			opState = 0;
			DIGITAL_IO_SetOutputLow(&IO_DRIVER_ENABLE);
		}
		else
		{
			DIGITAL_IO_SetOutputHigh(&IO_DRIVER_ENABLE);
			if(SW == 0)
			{
				opState = 1;	// Stay in SW OFF state
			}
			else
			{
				opState = 2;	//Go to SW ON State

			}
		}

	}
	pcComplete = 0;

}


/** ON State
 *  \param none
 *  \return none
 */
void op_ON_state()	//opState = 2
{
	uint8_t EN,SFST,SW;

	EN   = getEnableState();
	SFST = getSafeState();
	SW  = getSwitchState();

	pcMode  = getPreChargeMode();
	pcFreq  = getpwmFreq();
	pcTime  = getPWMDuration();
	pcPulse = getPulseDuration();

	prb_f = isProblemDetected();


	//MOSFET ON here ---- Precharge check
	maxPulse = prechargeConfig(pcMode,pcFreq,pcTime,pcPulse);
	pc_EN = getPrechargeState();


	if(pc_EN == 1)
	{
		if(pcComplete == 0)
		{
			tleFault = 0;
			pcComplete = 1;
			pulse_count = 0; //
			pcStart = 0; // reset to prevent monitoring by vload.
//			auto_retry = 1;
			XMC_GPIO_SetMode(PC_Ctrl.gpio_out_port, PC_Ctrl.gpio_out_pin, XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2);
			INTERRUPT_Enable(&INTERRUPT_1);
			fwStart = 0;
			PWM_CCU4_Stop(&FW_Ctrl);
			PWM_SetDutyCycle(&PC_Ctrl, 5000); // added by SAE Team
			PWM_Start(&PC_Ctrl);
			pcFlag = 0;
			if (pre_charge_dbg == 1)
				pcStart = 1;
		}

	}
	else
	{
		switchOn(0);
	}

	led_state = 2;

	if(SFST == 0 ||gateDriverFault == 1||isProblemDetected()==1||sysFaultDetected==1)
	{
		opState = 3;	// Go to fault state
	}
	else
	{
		if(EN==0)
		{
			opState = 0;	//Go to Init state
			DIGITAL_IO_SetOutputLow(&IO_DRIVER_ENABLE);
		}
		else
		{
			DIGITAL_IO_SetOutputHigh(&IO_DRIVER_ENABLE);
			if(SW==0)
			{
				opState = 1;	// Go to SW OFF state
			}
			else
			{
				opState = 2;	// Stay in SW On state
			}
		}

	}
	fwComplete = 0;
}

/** Fault State
 *  \param none
 *  \return none
 */
void op_Fault_state()	////opState = 2
{
	uint8_t EN,SFST, SW;

	EN   = getEnableState();
	SFST = getSafeState();
	SW  = getSwitchState();
	prb_f = isProblemDetected();

	if(SFST == 0 ||isProblemDetected()==1||sysFaultDetected==1)
	{
		opState = 3;	//go to Fault state
		switchOff(0);
	}

	else if (gateDriverFault == 1)
	{
		opState = 3;	//go to Fault state
	}
	else	//SFST = 1, when no fault found
	{
		if(EN ==0)
		{
			opState = 0;	//Go to Init state
			DIGITAL_IO_SetOutputLow(&IO_DRIVER_ENABLE);
		}
		else
		{
			fwComplete = 1;
			DIGITAL_IO_SetOutputHigh(&IO_DRIVER_ENABLE);
			opState = 1;	//go to SW OFF State for Switch OFF
		}
	}

	if(vfault==0 && tfault==1)
	{
		led_state = 4;
	}
	else if(vfault==1 && tfault == 0)
	{
		led_state = 5;
	}
	else if(vfault==1 && tfault==1)
	{
		led_state = 6;
	}
	else
	{
		led_state = 3;
	}

	pcComplete = 0;
	reactToFailure();
	if (auto_retry == 1)
	{

		clearProblemDetected();
		clearStatusFlags();
		if (PIN_INTERRUPT_GetPinValue(&OCD_INTERRUPT)  == 1)
		{
			tleFault = 0;
		}

	}

	if (SW == 0)
	{
		fwComplete = 1;
		switchOff(0);
		clearProblemDetected();
		clearStatusFlags();
		prefault = 0;
		if (PIN_INTERRUPT_GetPinValue(&OCD_INTERRUPT)  == 1)
		{
			tleFault = 0;
		}
		DIGITAL_IO_SetOutputHigh(&IO_DRIVER_ENABLE);
		opState = 1;	//go to SW OFF State for Switch OFF
	}


}

/** Fault checking for system fault not from gate driver.
 *  \param none
 *  \return none
 */
uint8_t checkFault()
{
	uint8_t faultTrigger;
	uint32_t OCD1_LOGIC = PIN_INTERRUPT_GetPinValue(&OCD_INTERRUPT) ;
	uint32_t OCD2_LOGIC = DIGITAL_IO_GetInput(&OCD2);


	cf_Vbat = get_Vbat();
	cf_NTCtemperature = getNTC_temperature();

	if(cf_Vbat >= MCU_OV_Vbat) // System Over Voltage
	{
		  uvFault = 0;
		  ovFault = 1;
		  vfault = 1;
	}
	else if(cf_Vbat <= MCU_UV_Vbat) // System Under Voltage
	{
		  uvFault = 1;
		  ovFault = 0;
		  vfault = 1;
	}
	else
	{
		  uvFault = 0;
		  ovFault = 0;
		  vfault = 0;
	}

	if(cf_NTCtemperature >= MCU_OT_NTCtemperature) // System OT
	{
		utFault = 0;
		otFault = 1;
		tfault = 1;
	}
	else if(cf_NTCtemperature <=MCU_UT_NTCtemperature) // System UT
	{
		utFault = 1;
		otFault = 0;
		tfault = 1;
	}
	else
	{
		utFault = 0;
		otFault = 0;
		tfault = 0;
	}

	if (OCD1_LOGIC != 1)
	{
		tleFault = 1;
	}
	else
	{
		tleFault = 0;
	}

	if (OCD2_LOGIC !=  1)
	{
		tleFault_OCD2 = 1;
	}
	else
	{
		tleFault_OCD2 = 0;
	}

	// check fault for all condition. add for ocd from tle
	if(vfault ==1 || tfault ==1 || prefault == 1 || tleFault == 1 || tleFault_OCD2 == 1)
	{
		faultTrigger = 1;
	}
	else
	{
		faultTrigger = 0;

	}

	return faultTrigger;
}

/** LED Blink function
 *  \param none
 *  \return none
 */
void blinkLED()
{
	if(led_state == 0)
	{
		if(led_blink_ms > 200)
		{
			DIGITAL_IO_SetOutputHigh(&PX_LED_R);
			DIGITAL_IO_ToggleOutput(&PX_LED_G);
			redLED = 0;
			greenLED = 1;
			led_blink_ms = 0;
		}

	}
	else if(led_state == 1)
	{
		DIGITAL_IO_SetOutputLow(&PX_LED_R); //turn on
		DIGITAL_IO_SetOutputLow(&PX_LED_G); //turn on
		redLED = 1;
		greenLED = 1;
		led_blink_ms = 0;
	}
	else if(led_state == 2)
	{
		DIGITAL_IO_SetOutputLow(&PX_LED_G); // turn on
		DIGITAL_IO_SetOutputHigh(&PX_LED_R); //turn off
		redLED = 0;
		greenLED = 1;
		led_blink_ms = 0;
	}
	else if(led_state == 3)
	{
		DIGITAL_IO_SetOutputLow(&PX_LED_R);
		DIGITAL_IO_SetOutputHigh(&PX_LED_G);
		redLED = 1;
		greenLED = 0;

		led_blink_ms = 0;
	}
	else if(led_state == 4)
	{
		if(led_blink_ms > 200)
		{
			DIGITAL_IO_ToggleOutput(&PX_LED_R);
			DIGITAL_IO_ToggleOutput(&PX_LED_G);
			redLED = 1;
			greenLED = !(greenLED&&0x01);
			led_blink_ms = 0;
		}
	}
	else if(led_state == 5)
	{
		if(led_blink_ms > 200)
		{
			DIGITAL_IO_SetOutputHigh(&PX_LED_G);
			DIGITAL_IO_ToggleOutput(&PX_LED_R);
			redLED = !(redLED&&0x01);
			greenLED = 0;
			led_blink_ms = 0;
		}
	}
	else if(led_state == 6)
	{
		if(led_blink_ms > 5)
		{
			DIGITAL_IO_SetOutputHigh(&PX_LED_G);
			DIGITAL_IO_ToggleOutput(&PX_LED_R);
			redLED = !(redLED&&0x01);
			greenLED = 0;
			led_blink_ms = 0;
		}
	}
	else
	{

		DIGITAL_IO_SetOutputHigh(&PX_LED_R);
		DIGITAL_IO_SetOutputHigh(&PX_LED_G);
		greenLED = 0;
		led_blink_ms = 0;
		led_blink_ms = 0;

	}

}


uint8_t prechargeConfig(uint8_t mode, uint8_t freq, uint8_t pctime, uint8_t singlePulse)
{
	if(mode == 1)
	{
		if(pctime==0)
		{
			if(freq == 0)
			{
				PWM_SetFreq(&PC_Ctrl, 10U);
				maxPulse = 1;
			}
			else if(freq == 1)
			{
				PWM_SetFreq(&PC_Ctrl, 50U);
				maxPulse = 5;
			}
			else
			{
				PWM_SetFreq(&PC_Ctrl, 100U);
				maxPulse = 10;
			}

		}
		else if(pctime==1)
		{
			if(freq == 0)
			{
				PWM_SetFreq(&PC_Ctrl, 10U);
				maxPulse = 5;
			}
			else if(freq == 1)
			{
				PWM_SetFreq(&PC_Ctrl, 50U);
				maxPulse = 25;
			}
			else
			{
				PWM_SetFreq(&PC_Ctrl, 100U);
				maxPulse = 50;
			}
		}
		else if(pctime==2)
		{
			if(freq == 0)
			{
				PWM_SetFreq(&PC_Ctrl, 10U);
				maxPulse = 10;
			}
			else if(freq == 1)
			{
				PWM_SetFreq(&PC_Ctrl, 50U);
				maxPulse = 50;
			}
			else
			{
				PWM_SetFreq(&PC_Ctrl, 100U);
				maxPulse = 100;
			}
		}
	}
	else if(mode==0)
	{
		maxPulse = 1;
		if(singlePulse == 0)
		{
			PWM_SetFreq(&PC_Ctrl, 5U); // set period/freq to 5Hz (100ms)
		}
		else if(singlePulse == 1)
		{
			PWM_SetFreq(&PC_Ctrl, 2U);// set period/freq to 2Hz (250ms)
		}
		else if(singlePulse == 2)
		{
			PWM_SetFreq(&PC_Ctrl, 1U);// set period/freq to 1Hz (500ms)
		}

	}

	return maxPulse;
}


