/*
 * eDisconnectEvent.h
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

#ifndef EDISCONNECTEVENT_H_
#define EDISCONNECTEVENT_H_

/***********************************************************************************************************************
 * VARIABLE DECLARATIONS
***********************************************************************************************************************/

volatile uint32_t pulse_count;
/***********************************************************************************************************************
 * EXTERN DECLARATIONS
***********************************************************************************************************************/
extern uint8_t fwComplete;
extern uint8_t pcComplete;
extern uint8_t prefault;
extern uint8_t pcStart;
extern uint8_t fwStart;
extern uint8_t pcFlag;
extern uint8_t OV_current;

/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
void operationHandler(void);
void GUIHandler(void);
void faultHandler (void);
void intervalTimer(void);
void eDisconnectInit(void);
void initConfig(void);
void operationState(void);

void op_init_state(void);
void op_OFF_state(void);
void op_ON_state(void);
void op_Fault_state(void);

void set_init_config (void);
void configSetting(void);

uint8_t checkFault(void);
void blinkLED(void);

uint8_t prechargeConfig(uint8_t mode, uint8_t freq, uint8_t pctime, uint8_t singlePulse);

void reportFailures(void);

#endif /* EDISCONNECTEVENT_H_ */




