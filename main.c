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
#define TEST_LENGTH_SAMPLES 512

/* Private function prototypes -----------------------------------------------------------------------------*/
void NVIC_Configuration(void);
void CKCU_Configuration(void);
void GPIO_Configuration(void);

void RUN_FFT(void);

/* Global variables ----------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
//extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
float32_t InputSignal[TEST_LENGTH_SAMPLES] = {
	0.089844,  0.000000, 0.359375,  0.000000,
	0.414063,  0.000000, 0.257813,  0.000000,
	-0.121094, 0.000000, -0.660156, 0.000000,
	0.621094,  0.000000, -0.269531, 0.000000,
	0.738281,  0.000000, -0.386719, 0.000000,
	0.414063,  0.000000, -0.843750, 0.000000,
	-0.074219, 0.000000, 0.699219,  0.000000,
	-0.464844, 0.000000, 0.464844,  0.000000,
	-0.476563, 0.000000, 0.714844,  0.000000,
	0.078125,  0.000000, -0.355469, 0.000000,
	-0.617188, 0.000000, -0.656250, 0.000000,
	-0.480469, 0.000000, -0.121094, 0.000000,
	0.425781,  0.000000, -0.812500, 0.000000,
	0.078125,  0.000000, -0.910156, 0.000000,
	0.210938,  0.000000, -0.566406, 0.000000,
	0.660156,  0.000000, -0.093750, 0.000000,
	-0.859375, 0.000000, 0.265625,  0.000000,
	-0.675781, 0.000000, 0.277344,  0.000000,
	-0.925781, 0.000000, -0.300781, 0.000000,
	0.140625,  0.000000, 0.375000,  0.000000,
	0.398438,  0.000000, 0.207031,  0.000000,
	-0.160156, 0.000000, -0.722656, 0.000000,
	0.527344,  0.000000, -0.378906, 0.000000,
	0.609375,  0.000000, -0.531250, 0.000000,
	0.265625,  0.000000, -0.984375, 0.000000,
	-0.222656, 0.000000, 0.566406,  0.000000,
	-0.585938, 0.000000, 0.343750,  0.000000,
	-0.593750, 0.000000, 0.628906,  0.000000,
	0.015625,  0.000000, -0.410156, 0.000000,
	-0.644531, 0.000000, -0.640625, 0.000000,
	-0.457031, 0.000000, -0.082031, 0.000000,
	0.511719,  0.000000, -0.730469, 0.000000,
	0.175781,  0.000000, -0.777344, 0.000000,
	0.355469,  0.000000, -0.433594, 0.000000,
	0.816406,  0.000000, 0.062500,  0.000000,
	-0.734375, 0.000000, 0.406250,  0.000000,
	-0.542969, 0.000000, 0.382813,  0.000000,
	-0.855469, 0.000000, -0.253906, 0.000000,
	0.179688,  0.000000, 0.406250,  0.000000,
	0.375000,  0.000000, 0.187500,  0.000000,
	-0.238281, 0.000000, -0.816406, 0.000000,
	0.429688,  0.000000, -0.476563, 0.000000,
	0.468750,  0.000000, -0.679688, 0.000000,
	0.097656,  0.000000, 0.867188,  0.000000,
	-0.378906, 0.000000, 0.421875,  0.000000,
	-0.722656, 0.000000, 0.226563,  0.000000,
	-0.703125, 0.000000, 0.539063,  0.000000,
	-0.035156, 0.000000, -0.437500, 0.000000,
	-0.632813, 0.000000, -0.636719, 0.000000,
	-0.429688, 0.000000, -0.011719, 0.000000,
	0.609375,  0.000000, -0.617188, 0.000000,
	0.308594,  0.000000, -0.640625, 0.000000,
	0.496094,  0.000000, -0.285156, 0.000000,
	0.953125,  0.000000, 0.203125,  0.000000,
	-0.593750, 0.000000, 0.542969,  0.000000,
	-0.414063, 0.000000, 0.496094,  0.000000,
	-0.753906, 0.000000, -0.191406, 0.000000,
	0.226563,  0.000000, 0.406250,  0.000000,
	0.375000,  0.000000, 0.144531,  0.000000,
	-0.300781, 0.000000, -0.886719, 0.000000,
	0.324219,  0.000000, -0.609375, 0.000000,
	0.335938,  0.000000, -0.828125, 0.000000,
	-0.042969, 0.000000, 0.707031,  0.000000,
	-0.535156, 0.000000, 0.269531,  0.000000,
	-0.867188, 0.000000, 0.113281,  0.000000,
	-0.796875, 0.000000, 0.468750,  0.000000,
	-0.097656, 0.000000, -0.476563, 0.000000,
	-0.656250, 0.000000, -0.621094, 0.000000,
	-0.375000, 0.000000, 0.039063,  0.000000,
	0.675781,  0.000000, -0.523438, 0.000000,
	0.425781,  0.000000, -0.511719, 0.000000,
	0.652344,  0.000000, -0.132813, 0.000000,
	-0.890625, 0.000000, 0.343750,  0.000000,
	-0.453125, 0.000000, 0.675781,  0.000000,
	-0.316406, 0.000000, 0.589844,  0.000000,
	-0.664063, 0.000000, -0.132813, 0.000000,
	0.234375,  0.000000, 0.382813,  0.000000,
	0.355469,  0.000000, 0.089844,  0.000000,
	-0.351563, 0.000000, -0.980469, 0.000000,
	0.222656,  0.000000, -0.734375, 0.000000,
	0.195313,  0.000000, -0.980469, 0.000000,
	-0.195313, 0.000000, 0.558594,  0.000000,
	-0.675781, 0.000000, 0.128906,  0.000000,
	-0.992188, 0.000000, 0.003906,  0.000000,
	-0.886719, 0.000000, 0.386719,  0.000000,
	-0.140625, 0.000000, -0.507813, 0.000000,
	-0.660156, 0.000000, -0.613281, 0.000000,
	-0.335938, 0.000000, 0.132813,  0.000000,
	0.765625,  0.000000, -0.406250, 0.000000,
	0.535156,  0.000000, -0.371094, 0.000000,
	0.792969,  0.000000, 0.027344,  0.000000,
	-0.742188, 0.000000, 0.484375,  0.000000,
	-0.312500, 0.000000, 0.792969,  0.000000,
	-0.191406, 0.000000, 0.699219,  0.000000,
	-0.578125, 0.000000, -0.078125, 0.000000,
	0.289063,  0.000000, 0.390625,  0.000000,
	0.324219,  0.000000, 0.050781,  0.000000,
	-0.410156, 0.000000, 0.933594,  0.000000,
	0.097656,  0.000000, -0.859375, 0.000000,
	0.046875,  0.000000, 0.882813,  0.000000,
	-0.351563, 0.000000, 0.421875,  0.000000,
	-0.824219, 0.000000, -0.011719, 0.000000,
	0.875000,  0.000000, -0.117188, 0.000000,
	-0.984375, 0.000000, 0.316406,  0.000000,
	-0.207031, 0.000000, -0.546875, 0.000000,
	-0.660156, 0.000000, -0.574219, 0.000000,
	-0.285156, 0.000000, 0.203125,  0.000000,
	0.875000,  0.000000, -0.308594, 0.000000,
	0.664063,  0.000000, -0.238281, 0.000000,
	0.953125,  0.000000, 0.167969,  0.000000,
	-0.578125, 0.000000, 0.640625,  0.000000,
	-0.175781, 0.000000, 0.933594,  0.000000,
	-0.074219, 0.000000, 0.769531,  0.000000,
	-0.531250, 0.000000, -0.023438, 0.000000,
	0.281250,  0.000000, 0.394531,  0.000000,
	0.292969,  0.000000, 0.011719,  0.000000,
	-0.472656, 0.000000, 0.859375,  0.000000,
	-0.003906, 0.000000, -0.992188, 0.000000,
	-0.093750, 0.000000, 0.730469,  0.000000,
	-0.511719, 0.000000, 0.277344,  0.000000,
	-0.964844, 0.000000, -0.152344, 0.000000,
	0.757813,  0.000000, -0.246094, 0.000000,
	0.917969,  0.000000, 0.253906,  0.000000,
	-0.261719, 0.000000, -0.570313, 0.000000,
	-0.664063, 0.000000, -0.542969, 0.000000,
	-0.242188, 0.000000, 0.257813,  0.000000,
	0.964844,  0.000000, -0.191406, 0.000000,
	0.808594,  0.000000, -0.089844, 0.000000
};
static float32_t OutputSignal[TEST_LENGTH_SAMPLES / 2];


/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = 256;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;

u32 i = 0;
arm_status status;
float32_t maxValue;

/* Global functions ----------------------------------------------------------------------------------------*/
/*********************************************************************************************************//**
  * @brief  Main program.
  * @retval None
  ***********************************************************************************************************/
int main(void) {
	NVIC_Configuration();               /* NVIC configuration                                                 */
	CKCU_Configuration();               /* System Related configuration                                       */
	GPIO_Configuration();               /* GPIO Related configuration                                         */
	RETARGET_Configuration();           /* Retarget Related configuration                                     */

	RUN_FFT();
	
	for(i = 0; i < TEST_LENGTH_SAMPLES / 2; i++) {
		if (i != 0 && i % 4 == 0) printf("\r\n");
		printf("%f ", OutputSignal[i]);
	}
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

void RUN_FFT(void) {
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len256, InputSignal, ifftFlag, doBitReverse);

	/* Process the data through the Complex Magnitude Module for
	calculating the magnitude at each bin */
	arm_cmplx_mag_f32(InputSignal, OutputSignal, fftSize);

	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(OutputSignal, fftSize, &maxValue, &testIndex);
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
