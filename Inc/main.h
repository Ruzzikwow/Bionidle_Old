/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DRDY_Pin GPIO_PIN_3
#define DRDY_GPIO_Port GPIOA
#define DRDY_EXTI_IRQn EXTI3_IRQn
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_4
#define LED_R_GPIO_Port GPIOB
#define LED_GR_Pin GPIO_PIN_5
#define LED_GR_GPIO_Port GPIOB
#define LED_Y_Pin GPIO_PIN_6
#define LED_Y_GPIO_Port GPIOB



/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define mode              0 //var of program
#define BLOOD							0x000A0000
#define	KOORTIK						0x00200000
#define	SPONGE						0x00300000
#define DRY								0x00400000
#define SHORT							0x00C00000
#define Delta							0x00050000
#define TIMER_PERIOD			100
#define EXT_TIM_PULSE 	  80
#define BUZZER_OFF				TIM10->CCR1 = 0
#define BUZZER_ON					TIM10->CCR1 = EXT_TIM_PULSE
#define DRDY_Pin GPIO_PIN_3
#define DRDY_GPIO_Port GPIOA
#define DRDY_EXTI_IRQn EXTI3_IRQn
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB



/* USER CODE BEGIN Private defines */
/* Command Definitions */
#define ADS1220_RESET 0x06                                                    //Send the RESET command (06h) to make sure the ADS1220 is properly reset after power-up
#define ADS1220_START 0x08                                         //Send the START/SYNC command (08h) to start converting in continuous conversion mode
//#define WREG  0x43
#define WREG  0x40
//#define RREG  0x23
#define RREG  0x20
#define DUMMY_BYTE   					0x00
#define ADS1220_CMD_RDATA    	0x10
#define ADS1220_CMD_RREG     	0x20
#define ADS1220_CMD_WREG     	0x40
#define ADS1220_CMD_SYNC    	0x08
#define ADS1220_CMD_SHUTDOWN  0x02
#define ADS1220_CMD_RESET    	0x06

/* ADS1220 Register Definitions */
#define ADS1220_0_REGISTER   		0x00
#define ADS1220_1_REGISTER     	0x01
#define ADS1220_2_REGISTER     	0x02
#define ADS1220_3_REGISTER    	0x03

#define ADS1220_MUX_0_1   	0x00
#define ADS1220_MUX_0_2   	0x10
#define ADS1220_MUX_0_3   	0x20
#define ADS1220_MUX_1_2   	0x30
#define ADS1220_MUX_1_3   	0x40
#define ADS1220_MUX_2_3   	0x50
#define ADS1220_MUX_1_0   	0x60
#define ADS1220_MUX_3_2   	0x70
#define ADS1220_MUX_0_G			0x80
#define ADS1220_MUX_1_G   	0x90
#define ADS1220_MUX_2_G   	0xa0
#define ADS1220_MUX_3_G   	0xb0
#define ADS1220_MUX_EX_VREF 0xc0
#define ADS1220_MUX_AVDD   	0xd0
#define ADS1220_MUX_DIV2   	0xe0

// Define GAIN
#define ADS1220_GAIN_1      0x00
#define ADS1220_GAIN_2      0x02
#define ADS1220_GAIN_4      0x04
#define ADS1220_GAIN_8      0x06
#define ADS1220_GAIN_16     0x08
#define ADS1220_GAIN_32     0x0a
#define ADS1220_GAIN_64     0x0c
#define ADS1220_GAIN_128    0x0e

// Define PGA_BYPASS
#define ADS1220_PGA_BYPASS 	0x01

/* ADS1220 Register 1 Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//                DR[2:0]            |      MODE[1:0]        |     CM    |     TS    |    BCS
//
// Define DR (data rate)
#define ADS1220_DR_20			0x00
#define ADS1220_DR_45			0x20
#define ADS1220_DR_90			0x40
#define ADS1220_DR_175		0x60
#define ADS1220_DR_330		0x80
#define ADS1220_DR_600		0xa0
#define ADS1220_DR_1000		0xc0

// Define MODE of Operation
#define ADS1220_MODE_NORMAL 0x00
#define ADS1220_MODE_DUTY		0x08
#define ADS1220_MODE_TURBO 	0x10
#define ADS1220_MODE_DCT		0x18

// Define CM (conversion mode)
#define ADS1220_CC			0x04

// Define TS (temperature sensor)
#define ADS1220_TEMP_SENSOR	0x02

// Define BCS (burnout current source)
#define ADS1220_BCS			0x01

/* ADS1220 Register 2 Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//         VREF[1:0]     |        50/60[1:0]     |    PSW    |             IDAC[2:0]
//
// Define VREF
#define ADS1220_VREF_INT		0x00
#define ADS1220_VREF_EX_DED	0x40
#define ADS1220_VREF_EX_AIN	0x80
#define ADS1220_VREF_SUPPLY	0xc0

// Define 50/60 (filter response)
#define ADS1220_REJECT_OFF	0x00
#define ADS1220_REJECT_BOTH	0x10
#define ADS1220_REJECT_50		0x20
#define ADS1220_REJECT_60		0x30

// Define PSW (low side power switch)
#define ADS1220_PSW_SW			0x08

// Define IDAC (IDAC current)
#define ADS1220_IDAC_OFF	0x00
#define ADS1220_IDAC_10		0x01
#define ADS1220_IDAC_50		0x02
#define ADS1220_IDAC_100	0x03
#define ADS1220_IDAC_250	0x04
#define ADS1220_IDAC_500	0x05
#define ADS1220_IDAC_1000	0x06
#define ADS1220_IDAC_2000	0x07

/* ADS1220 Register 3 Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0 
//--------------------------------------------------------------------------------------------
//               I1MUX[2:0]          |               I2MUX[2:0]          |   DRDYM   | RESERVED
//
// Define I1MUX (current routing)
#define ADS1220_IDAC1_OFF		0x00
#define ADS1220_IDAC1_AIN0	0x20
#define ADS1220_IDAC1_AIN1	0x40
#define ADS1220_IDAC1_AIN2	0x60
#define ADS1220_IDAC1_AIN3	0x80
#define ADS1220_IDAC1_REFP0	0xa0
#define ADS1220_IDAC1_REFN0	0xc0

// Define I2MUX (current routing)
#define ADS1220_IDAC2_OFF		0x00
#define ADS1220_IDAC2_AIN0	0x04
#define ADS1220_IDAC2_AIN1	0x08
#define ADS1220_IDAC2_AIN2	0x0c
#define ADS1220_IDAC2_AIN3	0x10
#define ADS1220_IDAC2_REFP0	0x14
#define ADS1220_IDAC2_REFN0	0x18

// define DRDYM (DOUT/DRDY behaviour)

#define ADS1220_DRDY_MODE	0x02

#define ADS1220_CS_LOW()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define ADS1220_CS_HIGH()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
