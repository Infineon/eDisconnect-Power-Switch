/**
 * @file pin_interrupt.c
 * @date 2016-03-02
 *
 * NOTE:
 * This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
 *
 * @cond
 ***********************************************************************************************************************
 * PIN_INTERRUPT v4.0.4 - The PIN_INTERRUPT APP invokes user interrupt handler in a response to rising and/or falling
 *                        edge event signal on a pin.
 *
 * Copyright (c) 2016-2020, Infineon Technologies AG
 * All rights reserved.                        
 *                                             
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
 *                                                                              
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes       
 * with Infineon Technologies AG (dave@infineon.com).        
 ***********************************************************************************************************************
 *
 * Change History
 * --------------
 * 2015-12-03:
 *     - Initial version for DAVEv4. <BR>
 * @endcond 
 *
 */

/***********************************************************************************************************************
 * HEADER FILES                                                                                                      
 **********************************************************************************************************************/
#include "pin_interrupt.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

/***********************************************************************************************************************
* API IMPLEMENTATION
***********************************************************************************************************************/
/*  API to retrieve version of the APP */
DAVE_APP_VERSION_t PIN_INTERRUPT_GetAppVersion(void)
{
  DAVE_APP_VERSION_t version;
  version.major = (uint8_t)PIN_INTERRUPT_MAJOR_VERSION;
  version.minor = (uint8_t)PIN_INTERRUPT_MINOR_VERSION;
  version.patch = (uint8_t)PIN_INTERRUPT_PATCH_VERSION;
  return (version);
}

/*
 * API to initialize the PIN_INTERRUPT APP ERU Event Trigger Logic, Output Gating Unit Hardware initialization
 * and NVIC node configuration.
 */
PIN_INTERRUPT_STATUS_t PIN_INTERRUPT_Init(const PIN_INTERRUPT_t *const handle)
{
  XMC_ASSERT("PIN_INTERRUPT_Init: PIN_INTERRUPT APP handle function pointer uninitialized", (handle != NULL));

  /* Initializes input pin characteristics */
  XMC_GPIO_Init(handle->port, handle->pin, &handle->gpio_config);
  /* ERU Event Trigger Logic Hardware initialization based on UI */
  XMC_ERU_ETL_Init(handle->eru, handle->etl, &handle->etl_config);
  /* OGU is configured to generate event on configured trigger edge */
  XMC_ERU_OGU_SetServiceRequestMode(handle->eru, handle->ogu, XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER);
#if (UC_FAMILY == XMC1)
  /* Configure NVIC node and priority */
  NVIC_SetPriority((IRQn_Type)handle->IRQn, handle->irq_priority);
#else
  /* Configure NVIC node, priority and subpriority */
  NVIC_SetPriority((IRQn_Type)handle->IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                    handle->irq_priority, handle->irq_subpriority));
#endif
#if (UC_SERIES == XMC14)
  XMC_SCU_SetInterruptControl((IRQn_Type)handle->IRQn, (XMC_SCU_IRQCTRL_t)handle->irqctrl);
#endif
  if (true == handle->enable_at_init)
  {
    /* Clear pending interrupt before enabling it */
    NVIC_ClearPendingIRQ((IRQn_Type)handle->IRQn);
    /* Enable NVIC node */
    NVIC_EnableIRQ((IRQn_Type)handle->IRQn);
  }
  return (PIN_INTERRUPT_STATUS_SUCCESS);
}

/**
 * This function is used to configure event trigger edge during run time.
 */
void PIN_INTERRUPT_SetEdgeSensitivity(const PIN_INTERRUPT_t *const handle, const PIN_INTERRUPT_EDGE_t edge)
{
  XMC_ASSERT("PIN_INTERRUPT_SetEdgeSensitivity: Handler null pointer", handle != NULL);
  XMC_ERU_ETL_SetEdgeDetection(handle->eru, handle->etl, (XMC_ERU_ETL_EDGE_DETECTION_t)edge);
}

/**
 * This function is used to get the configured event trigger edge during run time.
 */
PIN_INTERRUPT_EDGE_t PIN_INTERRUPT_GetEdgeSensitivity(const PIN_INTERRUPT_t *const handle)
{
  XMC_ASSERT("PIN_INTERRUPT_GetEdgeSensitivity: Handler null pointer", handle != NULL);
  return ((PIN_INTERRUPT_EDGE_t)XMC_ERU_ETL_GetEdgeDetection(handle->eru, handle->etl));
}
