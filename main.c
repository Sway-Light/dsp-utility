/* ----------------------------------------------------------------------
* Copyright (C) 2010-2012 ARM Limited. All rights reserved.
*
* $Date:         17. January 2013
* $Revision:     V1.4.0
*
* Project:       CMSIS DSP Library
* Title:	     arm_fft_bin_example_f32.c
*
* Description:   Example code demonstrating calculation of Max energy bin of
*                frequency domain of input signal.
*
* Target Processor: Cortex-M4/Cortex-M3
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */

/*********************************************************************************************************//**
 * @file    CMSIS_DSP/arm_fft_bin_example/main.c
 * @version $Rev:: 8            $
 * @date    $Date:: 2019-04-02 #$
 * @brief   Main program.
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/
// <<< Use Configuration Wizard in Context Menu >>>

/* Includes ------------------------------------------------------------------------------------------------*/
#include "ht32.h"
#include "ht32_board.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include "_ht32_project_source.h"

/** @addtogroup HT32_Series_Peripheral_Examples HT32 Peripheral Examples
  * @{
  */

/** @addtogroup CMSIS_DSP
  * @{
  */

/** @addtogroup arm_fft_bin_example
  * @{
  */


/* Private constants ---------------------------------------------------------------------------------------*/
#define TEST_LENGTH_SAMPLES 2048

/* Private function prototypes -----------------------------------------------------------------------------*/
void NVIC_Configuration(void);
void CKCU_Configuration(void);
void GPIO_Configuration(void);

/* Global variables ----------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES/2];

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;

/* Global functions ----------------------------------------------------------------------------------------*/
/*********************************************************************************************************//**
  * @brief  Main program.
  * @retval None
  ***********************************************************************************************************/
int main(void) {
	u32 i = 0;
	arm_status status;
	float32_t maxValue;

	NVIC_Configuration();               /* NVIC configuration                                                 */
	CKCU_Configuration();               /* System Related configuration                                       */
	GPIO_Configuration();               /* GPIO Related configuration                                         */
	RETARGET_Configuration();           /* Retarget Related configuration                                     */

	status = ARM_MATH_SUCCESS;

	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_f32_10khz, ifftFlag, doBitReverse);

	/* Process the data through the Complex Magnitude Module for
	calculating the magnitude at each bin */
	arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);

	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);
	printf("max value = %f\r\n", maxValue);
	
	if(testIndex !=  refIndex) {
		status = ARM_MATH_TEST_FAILURE;
	}

	/* ----------------------------------------------------------------------
	** Loop here if the signals fail the PASS check.
	** This denotes a test failure
	** ------------------------------------------------------------------- */

	if( status != ARM_MATH_SUCCESS) {
		printf("failed!\r\n");
		while(1);
	}
	printf("[");
	for(i = 0; i < TEST_LENGTH_SAMPLES/2; i++) {
		printf("%f.2, ", testOutput[i]);
	}
	printf("]");
	while(1);                             /* main function does not return */
}

/*********************************************************************************************************//**
  * @brief  Configure the NVIC vector table.
  * @retval None
  ***********************************************************************************************************/
void NVIC_Configuration(void) {
	NVIC_SetVectorTable(NVIC_VECTTABLE_FLASH, 0x0);     /* Set the Vector Table base location at 0x00000000   */
}

/*********************************************************************************************************//**
  * @brief  Configure the system clocks.
  * @retval None
  ***********************************************************************************************************/
void CKCU_Configuration(void) {
#if 1
	CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
	CKCUClock.Bit.AFIO       = 1;
	CKCU_PeripClockConfig(CKCUClock, ENABLE);
#endif
}

/*********************************************************************************************************//**
  * @brief  Configure the GPIO ports.
  * @retval None
  ***********************************************************************************************************/
void GPIO_Configuration(void) {
#if (RETARGET_PORT == RETARGET_USART0)
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_2 | AFIO_PIN_3, AFIO_FUN_USART_UART);
#endif

#if (RETARGET_PORT == RETARGET_USART1)
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_4 | AFIO_PIN_5, AFIO_FUN_USART_UART);
#endif

#if (RETARGET_PORT == RETARGET_UART0)
	AFIO_GPxConfig(GPIO_PC, AFIO_PIN_4 | AFIO_PIN_5, AFIO_FUN_USART_UART);
#endif

#if (RETARGET_PORT == RETARGET_UART1)
	AFIO_GPxConfig(GPIO_PC, AFIO_PIN_1 | AFIO_PIN_3, AFIO_FUN_USART_UART);
#endif
}

#if (HT32_LIB_DEBUG == 1)
/*********************************************************************************************************//**
  * @brief  Report both the error name of the source file and the source line number.
  * @param  filename: pointer to the source file name.
  * @param  uline: error line source number.
  * @retval None
  ***********************************************************************************************************/
void assert_error(u8* filename, u32 uline) {
  /*
     This function is called by IP library that the invalid parameters has been passed to the library API.
     Debug message can be added here.
     Example: printf("Parameter Error: file %s on line %d\r\n", filename, uline);
  */

  while (1) {
  }
}
#endif

/* Private functions ---------------------------------------------------------------------------------------*/


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
