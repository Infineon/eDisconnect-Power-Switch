/*********************************************************************************************************************
* DAVE APP Name : GLOBAL_ADC       APP Version: 4.0.22
*
* NOTE:
* This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
*********************************************************************************************************************/

/**
 * @cond
 ***********************************************************************************************************************
 *
 * Copyright (c) 2015-2020, Infineon Technologies AG
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
 *
 * 2015-02-16:
 *     - Initial version for DAVEv4.<BR>
 *
 * 2015-04-27:
 *     - Configuration structure modified.<BR>
 *     - GLOBAL_ADC_group_t changed to GLOBAL_ADC_GROUP_t.<BR>
 *
 * 2015-06-20:
 *     - Updated the copyright section.<BR>
 *
 * 2015-09-01:
 *     - Added support for XMC14 and XMC48/47.<BR>
 *        
 * 2015-10-01:
 *     - Analog clock default value changed to 26Mhz in XMC42.<BR>
 * 
 * 2015-10-08:
 *     - Added support for XMC4700/XMC4800 devices.<BR>
 *
 * 2015-12-03:
 *     - Added support for XMC4300 devices.<BR>
 *     - Optimized the manifest.<BR>     
 *
 * 2016-03-18:
 *     -Modified the minimum configurable value for Desired analog clock in XMC1x to 1MHz.<BR>
 *
 * 2016-07-08:
 *     - Fixed incorrect case for an included header.<br>
 *
 * 2016-08-18:
 *     - Added support for low frequency operations by configuring DIVS.<BR>
 *
 * @endcond
 *
 */

#ifndef GLOBAL_ADCCONF_H
#define GLOBAL_ADCCONF_H

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
 
/**********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

#define GLOBAL_ADC_HANDLE ((GLOBAL_ADC_t *)(void *) &GLOBAL_ADC_0) /**< Instance handle of the GLOBAL_ADC APP*/

#define GLOBAL_ADC_DIVS_VALUE (0U)

#define GLOBAL_ADC_AREF_VALUE XMC_VADC_GLOBAL_SHS_AREF_EXTERNAL_VDD_UPPER_RANGE

#define GLOBAL_ADC_MAJOR_VERSION (4) /**< Major version number of GLOBAL_ADC APP*/
#define GLOBAL_ADC_MINOR_VERSION (0) /**< Minor version number of GLOBAL_ADC APP*/
#define GLOBAL_ADC_PATCH_VERSION (22) /**< Patch version number of GLOBAL_ADC APP*/

#define GLOBAL_ADC_NUM_INSTANCES (1U)
  
#endif /* GLOBAL_ADCCONF_H_*/


