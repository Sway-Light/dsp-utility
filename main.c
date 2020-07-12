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
-0.688965,0.000000,-0.570801,0.000000,
-0.436035,0.000000,-0.292969,0.000000,
-0.139160,0.000000,0.015137,0.000000,
0.171387,0.000000,0.321777,0.000000,
0.461914,0.000000,0.589844,0.000000,
0.701172,0.000000,0.791504,0.000000,
0.862793,0.000000,0.907227,0.000000,
0.928223,0.000000,0.924805,0.000000,
0.892578,0.000000,0.833008,0.000000,
0.753906,0.000000,0.653809,0.000000,
0.533691,0.000000,0.401855,0.000000,
0.256836,0.000000,0.105469,0.000000,
-0.051758,0.000000,-0.206055,0.000000,
-0.357422,0.000000,-0.497070,0.000000,
-0.623047,0.000000,-0.733887,0.000000,
-0.825684,0.000000,-0.893066,0.000000,
-0.936523,0.000000,-0.956055,0.000000,
-0.948242,0.000000,-0.916504,0.000000,
-0.856445,0.000000,-0.776367,0.000000,
-0.673828,0.000000,-0.552734,0.000000,
-0.419922,0.000000,-0.271973,0.000000,
-0.120605,0.000000,0.035156,0.000000,
0.190430,0.000000,0.340332,0.000000,
0.478516,0.000000,0.603516,0.000000,
0.712891,0.000000,0.803223,0.000000,
0.867676,0.000000,0.912598,0.000000,
0.929199,0.000000,0.920898,0.000000,
0.885254,0.000000,0.826172,0.000000,
0.744141,0.000000,0.641602,0.000000,
0.519531,0.000000,0.386230,0.000000,
0.239258,0.000000,0.083984,0.000000,
-0.070313,0.000000,-0.225586,0.000000,
-0.373535,0.000000,-0.512207,0.000000,
-0.637695,0.000000,-0.743652,0.000000,
-0.835449,0.000000,-0.897949,0.000000,
-0.941406,0.000000,-0.954590,0.000000,
-0.943848,0.000000,-0.909668,0.000000,
-0.848633,0.000000,-0.763184,0.000000,
-0.662109,0.000000,-0.540039,0.000000,
-0.404297,0.000000,-0.255371,0.000000,
-0.101074,0.000000,0.054199,0.000000,
0.208496,0.000000,0.356934,0.000000,
0.495117,0.000000,0.618164,0.000000,
0.723633,0.000000,0.809570,0.000000,
0.875488,0.000000,0.915527,0.000000,
0.931152,0.000000,0.916992,0.000000,
0.879883,0.000000,0.816406,0.000000,
0.732910,0.000000,0.625488,0.000000,
0.503418,0.000000,0.368652,0.000000,
0.220703,0.000000,0.065430,0.000000,
-0.089844,0.000000,-0.244141,0.000000,
-0.390625,0.000000,-0.527344,0.000000,
-0.652832,0.000000,-0.759277,0.000000,
-0.844238,0.000000,-0.904297,0.000000,
-0.942871,0.000000,-0.956543,0.000000,
-0.940918,0.000000,-0.901855,0.000000,
-0.838867,0.000000,-0.751953,0.000000,
-0.645508,0.000000,-0.523438,0.000000,
-0.383789,0.000000,-0.236816,0.000000,
-0.082520,0.000000,0.073730,0.000000,
0.228027,0.000000,0.374023,0.000000,
0.511719,0.000000,0.632813,0.000000,
0.736328,0.000000,0.820801,0.000000,
0.881348,0.000000,0.919434,0.000000,
0.929199,0.000000,0.916504,0.000000,
0.874023,0.000000,0.805664,0.000000,
0.720215,0.000000,0.610840,0.000000,
0.487793,0.000000,0.350098,0.000000,
0.202148,0.000000,0.046875,0.000000,
-0.109375,0.000000,-0.263672,0.000000,
-0.409180,0.000000,-0.544434,0.000000,
-0.666504,0.000000,-0.770996,0.000000,
-0.852539,0.000000,-0.912109,0.000000,
-0.943848,0.000000,-0.957520,0.000000,
-0.938477,0.000000,-0.895996,0.000000,
-0.830078,0.000000,-0.742188,0.000000,
-0.633301,0.000000,-0.506836,0.000000,
-0.367188,0.000000,-0.218262,0.000000,
-0.063965,0.000000,0.092773,0.000000,
0.245605,0.000000,0.391602,0.000000,
0.527344,0.000000,0.644531,0.000000,
0.750000,0.000000,0.829590,0.000000,
0.887207,0.000000,0.920410,0.000000,
0.930664,0.000000,0.910645,0.000000,
0.864258,0.000000,0.797363,0.000000,
0.708496,0.000000,0.599121,0.000000,
0.471191,0.000000,0.334473,0.000000,
0.183105,0.000000,0.028320,0.000000,
-0.127930,0.000000,-0.282227,0.000000,
-0.425781,0.000000,-0.558594,0.000000,
-0.678223,0.000000,-0.781250,0.000000,
-0.859863,0.000000,-0.915527,0.000000,
-0.948242,0.000000,-0.954102,0.000000,
-0.936523,0.000000,-0.889648,0.000000,
-0.821289,0.000000,-0.727539,0.000000,
-0.617676,0.000000,-0.490723,0.000000,
-0.350098,0.000000,-0.197754,0.000000,
-0.044434,0.000000,0.110840,0.000000,
0.265137,0.000000,0.410156,0.000000,
0.541016,0.000000,0.658691,0.000000,
0.757324,0.000000,0.838379,0.000000,
0.893066,0.000000,0.924805,0.000000,
0.927734,0.000000,0.906250,0.000000,
0.859375,0.000000,0.790527,0.000000,
0.694824,0.000000,0.583496,0.000000,
0.456543,0.000000,0.314941,0.000000,
0.165527,0.000000,0.007813,0.000000,
-0.146484,0.000000,-0.299316,0.000000,
-0.442871,0.000000,-0.574707,0.000000,
-0.693359,0.000000,-0.791016,0.000000,
-0.867676,0.000000,-0.922363,0.000000,
-0.950684,0.000000,-0.954102,0.000000,
-0.932129,0.000000,-0.882324,0.000000,
-0.811035,0.000000,-0.716309,0.000000,
-0.603027,0.000000,-0.474609,0.000000,
-0.331543,0.000000,-0.179199,0.000000,
-0.023926,0.000000,0.130859,0.000000,
0.283203,0.000000,0.424805,0.000000,
0.558594,0.000000,0.673340,0.000000,
0.770996,0.000000,0.845215,0.000000,
0.899902,0.000000,0.926758,0.000000,
0.928223,0.000000,0.899414,0.000000,
0.852051,0.000000,0.776855,0.000000,
0.685059,0.000000,0.569824,0.000000,
0.437012,0.000000,0.296875,0.000000,
0.144531,0.000000,-0.010742,0.000000,
-0.166504,0.000000,-0.317871,0.000000,
-0.459961,0.000000,-0.592285,0.000000
};
static float32_t OutputSignal[TEST_LENGTH_SAMPLES / 2];


/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = TEST_LENGTH_SAMPLES / 2;
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

	printf("RUN_FFT\r\n");
	RUN_FFT();
	printf("finish\r\n");
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
