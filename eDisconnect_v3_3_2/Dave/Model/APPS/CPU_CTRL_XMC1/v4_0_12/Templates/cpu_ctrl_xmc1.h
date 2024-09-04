/**
 * @file cpu_ctrl_xmc1.h
 * @date 2015-10-14
 *
 * NOTE:
 * This file is generated by DAVE. Any manual modification done to this file will be lost when the code is
 * regenerated.
 */
/**
 * @cond
 ***********************************************************************************************************************
 * CPU_CTRL_XMC1 v4.0.12 - Sets the priority grouping for NVIC
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
 *     - Initial version<br>
 *
 * 2021-01-08:
 *     - Added check for minimum XMCLib version
 *
 * @endcond
 *
 */
/**
 * @addtogroup CPU_CTRL_XMC1
 * @{
 */

#ifndef CPU_CTRL_XMC1_H
#define CPU_CTRL_XMC1_H

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "xmc_common.h"
#include "DAVE_Common.h"
#include "cpu_ctrl_xmc1_conf.h"

/***********************************************************************************************************************
 * MACROS
 ***********************************************************************************************************************/
#define CPU_CTRL_XMC1_XMC_LIB_MAJOR_VERSION 2
#define CPU_CTRL_XMC1_XMC_LIB_MINOR_VERSION 0
#define CPU_CTRL_XMC1_XMC_LIB_PATCH_VERSION 0

#if !((XMC_LIB_MAJOR_VERSION > CPU_CTRL_XMC1_XMC_LIB_MAJOR_VERSION) ||\
      ((XMC_LIB_MAJOR_VERSION == CPU_CTRL_XMC1_XMC_LIB_MAJOR_VERSION) && (XMC_LIB_MINOR_VERSION > CPU_CTRL_XMC1_XMC_LIB_MINOR_VERSION)) ||\
      ((XMC_LIB_MAJOR_VERSION == CPU_CTRL_XMC1_XMC_LIB_MAJOR_VERSION) && (XMC_LIB_MINOR_VERSION == CPU_CTRL_XMC1_XMC_LIB_MINOR_VERSION) && (XMC_LIB_PATCH_VERSION >= CPU_CTRL_XMC1_XMC_LIB_PATCH_VERSION)))
#error "CPU_CTRL_XMC1 requires XMC Peripheral Library v2.0.0 or higher"
#endif

/***********************************************************************************************************************
 * ENUMS
 ***********************************************************************************************************************/
/**
 * @ingroup CPU_CTRL_XMC1_enumerations
 * @{
 */
/*
 * @brief enumeration for CPU_CTRL_XMC1 APP
 */
typedef enum CPU_CTRL_XMC1_STATUS
{
  CPU_CTRL_XMC1_STATUS_SUCCESS = 0U,        /**<APP initialization is success */
  CPU_CTRL_XMC1_STATUS_FAILURE = 1U         /**<APP initialization is failure */
} CPU_CTRL_XMC1_STATUS_t;

/**
 * @}
 */

/**********************************************************************************************************************
* DATA STRUCTURES
**********************************************************************************************************************/
/**
 * @ingroup CPU_CTRL_XMC1_datastructures
 * @{
 */
/**
 * @brief Configuration structure for CPU_CTRL_XMC1 APP
 */
typedef struct CPU_CTRL_XMC1
{
  bool initialized;			/**<APP is initialized or not. */
} CPU_CTRL_XMC1_t;

/**
 * @}
 */

/**********************************************************************************************************************
 * API PROTOTYPES
***********************************************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup CPU_CTRL_XMC1_apidoc
 * @{
 */

/**
 * @brief Get CPU_CTRL_XMC1 APP version
 * @return DAVE_APP_VERSION_t APP version information (major, minor and
 *                                                     patch number)
 *
 * \par<b>Description: </b><br>
 * The function can be used to check application software compatibility with a
 * specific version of the APP.
 *
 * Example Usage:
 *
 * @code
 * #include <DAVE.h>
 *
 * int main(void)
 * {
 *   DAVE_APP_VERSION_t version;
 *   init_status = DAVE_Init();
 *
 *   version = CPU_CTRL_XMC1_GetAppVersion();
 *   if (version.major != 4U)
 *   {
 *   }
 *   while(1) {
 *
 *   }
 *   return (0);
 * }
 * @endcode
 */
DAVE_APP_VERSION_t CPU_CTRL_XMC1_GetAppVersion(void);

CPU_CTRL_XMC1_STATUS_t CPU_CTRL_XMC1_Init(CPU_CTRL_XMC1_t *const handler);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif


#include "cpu_ctrl_xmc1_extern.h"


#endif /* CPU_CTRL_XMC1_H */

/**
 * @} (end addtogroup CPU_CTRL_XMC1)
 */
