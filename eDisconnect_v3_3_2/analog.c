/*
 * analog.c
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
#include "DAVE.h"
#include "math.h"
#include "DRIVER_2ED4820EM/Driver_2ED4820.h"
#include "DRIVER_2ED4820EM/registers.h"
#include "DRIVER_2ED4820EM/target.h"
#include "analog.h"
#include "NTC_LUT.h"
#include "displayData.h"
#include "eDisconnectEvent.h"


/***********************************************************************************************************************
 * VARIABLE DECLARATIONS
***********************************************************************************************************************/
XMC_VADC_RESULT_SIZE_t VBAT_raw = 0;		// Vbat ADC value
XMC_VADC_RESULT_SIZE_t VL_raw = 0;			// load voltage ADC value
XMC_VADC_RESULT_SIZE_t Isns_raw = 0;		// Current ADC value
XMC_VADC_RESULT_SIZE_t Aout_raw = 0;		// Current ADC value
XMC_VADC_RESULT_SIZE_t T1_raw = 0;			// Temperature ADC value
XMC_VADC_RESULT_SIZE_t VREF_raw = 0;

static float loadVoltage = 0;					//load voltage value to GUI
static float batteryVoltage = 0;					// battery voltage to GUI
float Isense = 0;					// current sensor value to GUI
double hall_current = 0;					// current sensor value to GUI
double hall_current_raw = 0;					// current sensor value to GUI
double shunt_voltage = 0;
double shunt_current = 0;

double aout_voltage_raw = 0;
double vref_voltage_raw = 0;

float current_offset = 0;
float current_gain = 1;
uint16_t ik[100] = {0};
uint16_t ak[100] = {0};
uint16_t vk[100] = {0};

uint16_t i_avg=0;
double i_fil=0;
double aout_sum=0;
double vref_sum=0;

float voltage_gain = 0.016;

static int16_t NTC_Temperature = 0;
uint16_t T1_index =0;

uint8_t faultDetected = 0;

volatile uint8_t ADCIntDetected = 0;		// interrupt detection from ADC measurement handler

uint8_t meascount = 0;
uint8_t isencount = 0;
uint8_t maxcount = 10;
uint32_t Isns_avg;
uint32_t Isns_fil = 0;
uint16_t Isns_indv[100];
uint32_t Isns_acc = 0;
static double lsb = 3.3/4096;

float preVolt = 0.98 ;
float freeVolt = 0.25;
float freecheckVolt;
float precheckVolt;
uint8_t window_size = 30;

float filteredValue = 0;     // Variable to store the filtered value
float filteredAout = 0;
float filteredVref = 0;
float alpha = 0.03;           // Smoothing factor for PT1 filter
float alphaAout = 0.01;           // Smoothing factor for PT1 filter
float alphaVref = 0.01;           // Smoothing factor for PT1 filter

double cso_voltage;

/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/

/** Function to average the input data
 *  \param input_data
 *  \param data_size
 *  \param window_size
 *  \return filtered_data
 */
uint16_t MovingAverage(const uint16_t input_data[], const uint8_t data_size, const uint8_t window_size)
{
	uint32_t filtered_data = 0;
	int _window_size = window_size;
	int count = data_size - 1;

    if (data_size < window_size) {
        _window_size = data_size;
    } else {
        _window_size = window_size;
    }

    for (int j = 0; j < _window_size; j++) {
    	filtered_data += input_data[count-j];
    }
    filtered_data /= _window_size;

    return (uint16_t)filtered_data;
}

/** ADC interrupt handler to read measured result
 *  \param none
 *  \return none
 */
void Adc_Measurement_Handler()
{
	double aout_voltage = 0;
	double vref_voltage = 0;
	static double gain[8] = {10, 15, 20, 25, 31.5, 35, 40, 47.7}; // gain in V/V


	VBAT_raw  = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_VBAT_SNS);
	VL_raw = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_VL_SNS);
	Isns_raw = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_I_SNS_ADC);
	Aout_raw = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_AOUT_ADC);
	VREF_raw = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_VREF_ADC);
	T1_raw = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_NTC_ADC);

	isencount++;
	if (isencount <=maxcount)
	{
		Isns_acc = (uint32_t)Isns_raw + Isns_acc;
	}
	else
	{
		isencount = 0;
		Isns_avg = (uint16_t)Isns_acc/maxcount;
		Isns_acc = 0;
	}

//	Y(n+1) +=( (X(n+1) – Y(n))>>k)
	if(meascount !=window_size)
	{
		ik[meascount] = Isns_raw;
		ak[meascount] = Aout_raw;
		vk[meascount] = VREF_raw;
		meascount++;
	}
	else
	{
		for (int j = 0; j < window_size - 1; j++)
		{
			ik[j] = ik[j+1];
			ak[j] = ak[j+1];
			vk[j] = vk[j+1];
		}
		ik[window_size-1] = Isns_raw;
		ak[window_size-1] = Aout_raw;
		vk[window_size-1] = VREF_raw;
	}
	i_avg = MovingAverage(ik, meascount,window_size);
	aout_sum = MovingAverage(ak, meascount,window_size);
	vref_sum = MovingAverage(vk, meascount,window_size);

//	Isns_fil += ((i_avg - Isns_fil)>>k);
	filteredValue = alpha * i_avg + (1 - alpha) * filteredValue;
	filteredAout = alphaAout * aout_sum + (1 - alphaAout) * filteredAout;
	filteredVref = alphaVref * vref_sum + (1 - alphaVref) * filteredVref;

	// Current measurement
	cso_voltage = lsb * filteredValue; //i_avg
	currentGainshunt = gain[currentGain];
	shunt_voltage = (cso_voltage - 3.3/2) / currentGainshunt; // get the driver csga value from GUI
	shunt_current = shunt_voltage / set_r_shunt;
	Isense = (float)shunt_current * currentKI;

	// Current measurement
	aout_voltage_raw = aout_voltage = lsb * (double)filteredAout; // convert to voltage aout_sum
	vref_voltage_raw = vref_voltage = lsb * (double)filteredVref; // convert to voltage vref_sum
	hall_current_raw = vref_voltage - aout_voltage;
	hall_current = hall_current_raw/(hall_sensitivity*1e-3); //micro

}

/** update the measured temperature
 *  \param none
 *  \return none
 */
void measureProcess()
{
	// Temperature measurement
	T1_raw = T1_raw;

	T1_index = T1_raw ;
	NTC_Temperature = ntc_temp_lut[T1_index];
}

/** Load voltage check for precharge and freewheel pulse to turn off.
 *  \param none
 *  \return none
 */
void load_voltage_check(void)
{
	float lVoltage;					//load voltage value to GUI
	float bVoltage;					// battery voltage to GUI

	// Voltage measurement
	bVoltage = voltage_gain * VBAT_raw; // adc gain 0.016 for voltage
	lVoltage = voltage_gain * VL_raw;
	freecheckVolt = bVoltage * freeVolt;
	precheckVolt = bVoltage * preVolt;

	if (pre_charge_dbg == 1)
	{
		if (lVoltage >= precheckVolt && pcStart == 1 && pulse_count == 0)
		{
			pcFlag = 1;
			pcStart = 0;
			switchOn(0);
			INTERRUPT_Disable(&INTERRUPT_1);
			PWM_SetDutyCycle(&PC_Ctrl, 0);
			PWM_Stop(&PC_Ctrl);
			XMC_GPIO_SetOutputLow(PC_Ctrl.gpio_out_port, PC_Ctrl.gpio_out_pin);
			XMC_GPIO_SetMode(PC_Ctrl.gpio_out_port, PC_Ctrl.gpio_out_pin, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
			pcComplete = 1;
			pulse_count = 0;
		}
	}
	if (free_wheel_dbg == 1)
	{
		if (lVoltage <= freecheckVolt && fwStart == 1)
		{
			INTERRUPT_Disable(&FW_STOP_INTERRUPT);
			PWM_CCU4_Stop(&FW_Ctrl);
			fwStart = 0;
//			INTERRUPT_Enable(&FW_STOP_INTERRUPT);
		}
	}

}

/** return the Battery Voltage
 *  \param none
 *  \return batteryVoltage
 */
float get_Vbat()
{
	return batteryVoltage;
}

/** return the Load Voltage
 *  \param none
 *  \return loadVoltage
 */
float get_Vload()
{
	return loadVoltage;
}

/** return the Battery current
 *  \param none
 *  \return Isense
 */
float getBatcurrent()
{
	return Isense;
}

/** return the temperature value
 *  \param none
 *  \return NTC_Temperature
 */
uint16_t getNTC_temperature()
{
	return NTC_Temperature;
}

/** return the current sensing from hall sensor
 *  \param none
 *  \return hall_current
 */
float getHallcurrent()
{
	return hall_current;
}

