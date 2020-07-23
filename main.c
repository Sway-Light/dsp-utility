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
#include "ws2812.h"

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
#define TEST_LENGTH_SAMPLES 128

/* Private function prototypes -----------------------------------------------------------------------------*/
void NVIC_Configuration(void);
void CKCU_Configuration(void);
void GPIO_Configuration(void);
void GPTM1_Configuration(void);
void ADC_Configuration(void);
void TM_Configuration(void);
void wsUpdateMag(void);

void ADC_MainRoutine(void);
void RUN_FFT(void);

void delay(u32 d);

/* Global variables ----------------------------------------------------------------------------------------*/
s32 gADC_Result;
vu32 gADC_CycleEndOfConversion;
/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
bool sampleFlag = FALSE;
s32 InputSignal[TEST_LENGTH_SAMPLES];
float32_t fftData[TEST_LENGTH_SAMPLES];
static float32_t OutputSignal[TEST_LENGTH_SAMPLES / 2];

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = TEST_LENGTH_SAMPLES / 2;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

const u8 WS_LED[72] = {
	 7, 6, 5, 4, 3, 2, 1, 0,
	 8, 9,10,11,12,13,14,15,
	23,22,21,20,19,18,17,16,
	24,25,26,27,28,29,30,31,
	39,38,37,36,35,34,33,32,
	40,41,42,43,44,45,46,47,
	55,54,53,52,51,50,49,48,
	56,57,58,59,60,61,62,63,
	71,70,69,68,67,66,65,64};

u8 ws_white[3] = {255, 255, 255};
u16 i = 0;
s16 j = 0;

/* Global functions ----------------------------------------------------------------------------------------*/
/*********************************************************************************************************//**
  * @brief  Main program.
  * @retval None
  ***********************************************************************************************************/
int main(void) {
	NVIC_Configuration();               /* NVIC configuration                                                 */
	CKCU_Configuration();               /* System Related configuration                                       */
	GPIO_Configuration();               /* GPIO Related configuration                                         */
	GPTM1_Configuration();
	RETARGET_Configuration();           /* Retarget Related configuration                                     */

	wsInit();
	ADC_Configuration();

	TM_Configuration();

	ADC_Cmd(HT_ADC0, ENABLE);

	/* Enable TM which will trigger ADC start of conversion periodically                                      */
	TM_Cmd(HT_GPTM0, ENABLE);
	
	printf("\r\n");
	for(j = 0; j < 31; j++) printf("%5.2fk ", 0.6 + 0.63 * j);
	printf("\r\n");
	
	//wsBlinkAll(300);
	for(j = 0; j < WS_PIXEL; j++) {
		wsSetColor(WS_LED[j], ws_white, 0.1);
		delay(20000);
	}
	for(j = 0; j < WS_PIXEL; j++) {
		wsSetColor(WS_LED[j], ws_white, 0);
		delay(20000);
	}
	
	while(1) {                             /* main function does not return */
//		static u8 wsCount = 0;
		ADC_MainRoutine();
		if (sampleFlag) {
			for (j = 0; j < TEST_LENGTH_SAMPLES; j += 1) fftData[j] = ((float)InputSignal[j]) / 2048.0;
			RUN_FFT();
			printf("\r");
			for (j = 1; j < 32; j += 1) {
				printf("%6.0f ", OutputSignal[j]);
//				if(j%4 == 0) {
//					wsSetColor(j/4, ws_white, (float)(OutputSignal[j]/35));
//				}
			}
			wsUpdateMag();
			i = 0;
			sampleFlag = FALSE;
		}
	}
}

/*********************************************************************************************************//**
  * @brief  Configure the NVIC vector table.
  * @retval None
  ***********************************************************************************************************/
void NVIC_Configuration(void) {
	NVIC_SetVectorTable(NVIC_VECTTABLE_FLASH, 0x0);     /* Set the Vector Table base location at 0x00000000   */
	NVIC_EnableIRQ(GPTM1_IRQn);
}

/*********************************************************************************************************//**
  * @brief  Configure the system clocks.
  * @retval None
  ***********************************************************************************************************/
void CKCU_Configuration(void) {
#if 1
	CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
	CKCUClock.Bit.AFIO = 1;
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

/*********************************************************************************************************//**
  * @brief  ADC configuration.
  * @retval None
  ***********************************************************************************************************/
void ADC_Configuration(void) {
	{ /* Enable peripheral clock                                                                              */
		CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
		CKCUClock.Bit.AFIO = 1;
		CKCUClock.Bit.ADC0 = 1;
		CKCU_PeripClockConfig(CKCUClock, ENABLE);
	}

	/* Configure AFIO mode as ADC function                                                                    */
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_6, AFIO_MODE_2);

	{ /* ADC related settings                                                                                 */
		/* CK_ADC frequency is set to (CK_AHB / 64)                                                             */
		CKCU_SetADCnPrescaler(CKCU_ADCPRE_ADC0, CKCU_ADCPRE_DIV16);

		/* One Shot mode, sequence length = 3                                                                   */
		ADC_RegularGroupConfig(HT_ADC0, ONE_SHOT_MODE, 1, 0);

		/* ADC conversion time = (Sampling time + Latency) / CK_ADC = (1.5 + ADST + 12.5) / CK_ADC              */
		/* Set ADST = 36, sampling time = 1.5 + ADST                                                            */
		#if (LIBCFG_ADC_SAMPLE_TIME_BY_CH)
		// The sampling time is set by the last parameter of the function "ADC_RegularChannelConfig()".
		#else
		ADC_SamplingTimeConfig(HT_ADC0, 0);
		#endif

		/* Set ADC conversion sequence as channel n                                                             */
		ADC_RegularChannelConfig(HT_ADC0, ADC_CH_6, 0, 0);

		/* Set GPTM0 CH3O as ADC trigger source                                                                 */
		ADC_RegularTrigConfig(HT_ADC0, ADC_TRIG_GPTM0_CH3O);
	}

	/* Enable ADC single/cycle end of conversion interrupt                                                    */
	ADC_IntConfig(HT_ADC0, ADC_INT_SINGLE_EOC | ADC_INT_CYCLE_EOC, ENABLE);

	/* Enable the ADC interrupts                                                                              */
	NVIC_EnableIRQ(ADC0_IRQn);
}

/*********************************************************************************************************//**
  * @brief  TM configuration.
  * @retval None
  ***********************************************************************************************************/
void TM_Configuration(void) {
	{ /* Enable peripheral clock                                                                              */
		CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
		CKCUClock.Bit.GPTM0 = 1;
		CKCU_PeripClockConfig(CKCUClock, ENABLE);
	}

	{ /* Time base configuration                                                                              */
		TM_TimeBaseInitTypeDef TimeBaseInit;
		TimeBaseInit.Prescaler = (SystemCoreClock / 160000) - 1;
		TimeBaseInit.CounterReload = 4 - 1;
		TimeBaseInit.RepetitionCounter = 0;
		TimeBaseInit.CounterMode = TM_CNT_MODE_UP;
		TimeBaseInit.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
		TM_TimeBaseInit(HT_GPTM0, &TimeBaseInit);
	}

	{ /* Channel n output configuration                                                                       */
		TM_OutputInitTypeDef OutInit;
		OutInit.Channel = TM_CH_3;
		OutInit.OutputMode = TM_OM_PWM2;
		OutInit.Control = TM_CHCTL_ENABLE;
		OutInit.ControlN = TM_CHCTL_DISABLE;
		OutInit.Polarity = TM_CHP_NONINVERTED;
		OutInit.PolarityN = TM_CHP_NONINVERTED;
		OutInit.IdleState = MCTM_OIS_LOW;
		OutInit.IdleStateN = MCTM_OIS_HIGH;
		OutInit.Compare = 2 - 1;
		OutInit.AsymmetricCompare = 0;
		TM_OutputInit(HT_GPTM0, &OutInit);
	}

	TM_IntConfig(HT_GPTM0, TM_INT_CH3CC, ENABLE);
	NVIC_EnableIRQ(GPTM0_IRQn);
}

void GPTM1_Configuration(void) {
	TM_TimeBaseInitTypeDef TimeBaseInit;
	{ /* Enable peripheral clock                                                                              */
		CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
		CKCUClock.Bit.GPTM1 = 1;
		CKCU_PeripClockConfig(CKCUClock, ENABLE);
	}
	TM_TimeBaseStructInit(&TimeBaseInit);                       // Init GPTM1 time-base
	TimeBaseInit.CounterMode = TM_CNT_MODE_UP;                  // up count mode
	TimeBaseInit.CounterReload = 36000;                  // interrupt in every 500us
	TimeBaseInit.Prescaler = 5;
	TimeBaseInit.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;          // reload immediately
	TM_TimeBaseInit(HT_GPTM1, &TimeBaseInit);                   // write the parameters into GPTM1
	TM_ClearFlag(HT_GPTM1, TM_FLAG_UEV);                        // Clear Update Event Interrupt flag
	TM_IntConfig(HT_GPTM1, TM_INT_UEV, ENABLE);                 // interrupt by GPTM update
	TM_Cmd(HT_GPTM1, ENABLE);                                   // enable the counter 1
}

void ADC_MainRoutine(void) {
	if (gADC_CycleEndOfConversion) {
		if (i < TEST_LENGTH_SAMPLES) {
			InputSignal[i] = gADC_Result;
			InputSignal[i + 1] = 0;
			i += 2;
		} else {
			sampleFlag = TRUE;
		}
		gADC_CycleEndOfConversion = FALSE;
	}
}

void RUN_FFT(void) {
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len64, fftData, ifftFlag, doBitReverse);

	/* Process the data through the Complex Magnitude Module for
	calculating the magnitude at each bin */
	arm_cmplx_mag_f32(fftData, OutputSignal, fftSize);
}

void wsUpdateMag(void) {
	u8 i, j;
	u8 level;
	u8 color[9][3] = {
			{0, 255, 0},
			{128, 255, 0},
			{200, 255, 0},
			{255, 255, 0},
			{200, 255, 0},
			{255, 200, 0},
			{255, 128, 0},
			{255, 80, 0},
			{255, 0, 0}
		};
	for (i = 1; i < 16; i += 1) {
		if(i % 2 == 0) {
			if (OutputSignal[i] < 3) level = 1;
			else if(OutputSignal[i] < 5) level = 2;
			else if(OutputSignal[i] < 8) level = 3;
			else if(OutputSignal[i] < 11) level = 4;
			else if(OutputSignal[i] < 14) level = 5;
			else if(OutputSignal[i] < 17) level = 6;
			else if(OutputSignal[i] < 20) level = 7;
			else if(OutputSignal[i] < 23) level = 8;
			else level = 9;
			
			for (j = 0; j < 9; j++) {
				if (j < level) wsSetColor(WS_LED[(i / 2) + j * 8], color[j], 0.2);
				else wsSetColor(WS_LED[(i / 2) + j * 8], color[j], 0);
			}
		}
	}
}

void delay(u32 d) {
	while(d--);
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
