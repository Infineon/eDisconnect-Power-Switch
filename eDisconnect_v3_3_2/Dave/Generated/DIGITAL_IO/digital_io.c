/**
 * @file digital_io.c
 * @date 2016-07-08
 *
 * NOTE:
 * This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
 *
 * @cond
 ***********************************************************************************************************************
 * DIGITAL_IO v4.0.18 - The DIGITAL_IO APP is used to configure a port pin as digital Input/Output.
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
 * 2015-02-16
 *     - Initial version<br>
 * 2015-12-22
 *     - Added hardware controlled IO feature.
 * 2016-07-08:
 *     - Fixed incorrect case for an included header.<br>
 *
 * @endcond
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "digital_io.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

 /**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/

/**
* @brief Get DIGITAL_IO APP version
* @return DAVE_APP_VERSION_t APP version information (major, minor and patch number)
*/

DAVE_APP_VERSION_t DIGITAL_IO_GetAppVersion(void)
{
  DAVE_APP_VERSION_t version;

  version.major = (uint8_t)DIGITAL_IO_MAJOR_VERSION;
  version.minor = (uint8_t)DIGITAL_IO_MINOR_VERSION;
  version.patch = (uint8_t)DIGITAL_IO_PATCH_VERSION;

  return (version);
}


/**
* @brief Function to initialize the port pin as per UI settings.
* @param handler Pointer pointing to APP data structure.
* @return DIGITAL_IO_STATUS_t DIGITAL_IO APP status.
*/

DIGITAL_IO_STATUS_t DIGITAL_IO_Init(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_Init: handler null pointer", handler != NULL);

  /* Initializes input / output characteristics */
  XMC_GPIO_Init(handler->gpio_port, handler->gpio_pin, &handler->gpio_config);

  /*Configure hardware port control*/
  XMC_GPIO_SetHardwareControl(handler->gpio_port, handler->gpio_pin, handler->hwctrl);

  return (DIGITAL_IO_STATUS_OK);
}
