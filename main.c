/**
  ******************************************************************************
  * @file    app.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides all the Application firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/ 

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "main.h"
#include "usbd_cdc_vcp.h"
#include "stm32f4_discovery.h"
#include "lcd.h"
#include "private_structs.h"
#include <stdio.h>

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;



/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t PrescalerValue = 0;
//__IO uint16_t CCR1_Val = 29;//
//__IO uint16_t CCR2_Val = 31;//47
//__IO uint16_t CCR3_Val = 37;//
//__IO uint16_t CCR4_Val = 41;//

__IO uint16_t CCR1_Val = 29;//
__IO uint16_t CCR2_Val = 29;//47
__IO uint16_t CCR3_Val = 29;//
__IO uint16_t CCR4_Val = 29;//
int TIM3_Counter_Clock = 2000;//counter clock rate

uint16_t errorCode = 0;
uint16_t state = IDLE;
uint16_t preState = IDLE;

uint16_t capture;
compartment compartments[4];
extern char msg[17];
uint32_t command;



extern void reportResult(void);

char lcd_buffer[17];    // LCD display buffer
volatile uint32_t system_time_counter = 0;

/* Private function prototypes -----------------------------------------------*/
extern void dirSet(uint16_t desiredDir,compartment* com);
void TIM_Config(void);
void COM1_Config(void);

void EXTI_Config(void);

void init_Compartments(void);

void Delay2(__IO uint32_t nCount);
void clean(char* array,uint8_t size);
int main(void)
{
		
	uint8_t data_to_send2[17];
	capture = 0;

	//uint16_t i = 0;
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

  /* TIM Configuration */
  TIM_Config();	
	COM1_Config();
	EXTI_Config();
	lcd_init(LCD_DISP_ON);
  /* Compute the prescaler value */
////////////////  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / TIM3_Counter_Clock) - 1;

////////////////  /* Time base configuration */
////////////////  TIM_TimeBaseStructure.TIM_Period = 65535;
////////////////  TIM_TimeBaseStructure.TIM_Prescaler = 0;
////////////////  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
////////////////  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

////////////////  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

////////////////  /* Prescaler configuration */
////////////////  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

////////////////  /* Output Compare Timing Mode configuration: Channel1 */
////////////////  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
////////////////  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////////////////  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
////////////////  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

////////////////  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

////////////////  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

////////////////  /* Output Compare Timing Mode configuration: Channel2 */
////////////////  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
////////////////  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

////////////////  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

////////////////  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / TIM3_Counter_Clock) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
   
  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);	

  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);
  /* TIM Interrupts enable */

	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 |TIM_IT_CC4, ENABLE);

	
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
  
	if (SysTick_Config(SystemCoreClock / 1000))
  { 
    //Capture error
    while (1);
  }
	
	//LEDInit();
	
	// Add carriage return and line feed
	clean((char *)data_to_send2,30);
	clean(msg,30);

	while (1)
  {
		
		switch (state)
		{
			case IDLE:	
				VCP_DataRx (data_to_send2,30);
				Delay2(0x3FFFFF);
				init_Compartments();
//				lcd_clrscr();    //clears the LCD
//				lcd_gotoxy(0,0); 	//goes to bottom left
////				lcd_puts((char*) (msg));//prints the int			
//				sprintf(lcd_buffer,"%i",compartments[BIN4].dosage);
//				lcd_puts((char*) (lcd_buffer));
				if (command == 1){
					compartments[BIN1].state = CHEC;
					compartments[BIN2].state = CHEC;	
					compartments[BIN3].state = CHEC;
					compartments[BIN4].state = CHEC;	
					compartments[BIN1].preState = IDLE;	
					compartments[BIN2].preState = IDLE;	
					compartments[BIN3].preState = IDLE;	
					compartments[BIN4].preState = IDLE;						
					preState = IDLE;
					state = OPER;
					TIM_Cmd(TIM3, ENABLE);
//					Delay2(0x3FFFFF);
//					TIM_Cmd(TIM3, DISABLE);
//					clean(msg,17);
//					clean((char *)data_to_send2,17);
				}
				break;
				
			case OPER:
					lcd_clrscr();    //clears the LCD
					lcd_gotoxy(0,0); 	//goes to bottom left
					sprintf(lcd_buffer,"%i %i %i %i",compartments[BIN1].pillFall,compartments[BIN2].pillFall,compartments[BIN3].pillFall,compartments[BIN4].pillFall); //Divides the int by 2 so that it accuratly describes milliseconds		
//					sprintf(lcd_buffer,"%i %i",compartments[BIN4].pillFall,compartments[BIN4].dosage); //Divides the int by 2 so that it accuratly describes milliseconds		
					lcd_puts(lcd_buffer);//prints the int				
					//state = REPORT;
				//GPIO_WriteBit(GPIOB, GPIO_Pin_6,Bit_RESET);				
				break;
				
			
			case REPORT:
				TIM_Cmd(TIM3, DISABLE);
			
				errorCode = 0;
				command = 0;
				clean((char*)data_to_send2,30);			
				clean(msg,30);
				//report
				reportResult();
				STM_EVAL_LEDOn(LED6);		
				STM_EVAL_LEDOff(LED3);
				STM_EVAL_LEDOff(LED4);
				STM_EVAL_LEDOff(LED5);					
				//set states
				state = IDLE;
				preState = REPORT;
				break;
	
		}
		//prePillFall = pillFall;//
  }
} 



void EXTI_Config(void)
{
	/*
	PD0 - IR sensor for BIN1
	PD2 - IR sensor for BIN2
	
	PD4 - IR sensor for BIN3
	PD6 - IR sensor for BIN4
	

	*/
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the BUTTON Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_6;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	  /* Configure Button pin as input */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);

//    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
		
//		/* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
		
//		/* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource4);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
		
////		/* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);



}


void TIM_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);


  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
  /* Initialize Leds mounted on STM32F4-Discovery board */
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  /* Turn on LED4, LED3, LED5 and LED6 */
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED5);
  STM_EVAL_LEDOff(LED6);
}
void COM1_Config(void){//Enable the pins for the motor
	/*
	PB6 - TIM3 dir
	PB4 - TIM3 pulse
	
	PD7 - TIM2 dir
	PD5 - TIM2 pulse
	
	PD3 - TIM4 dir
	PD1 - TIM4 pulse
	
	PC12 - TIM5 dir
	PC10 - TIM5 pulse
	*/
	
	GPIO_InitTypeDef GPIO_InitStructure;

	compartments[BIN1].GPIOx_Dir = GPIOB;
	compartments[BIN1].GPIO_Pin_Dir = GPIO_Pin_6;
	compartments[BIN1].GPIOx_Pul = GPIOB;
	compartments[BIN1].GPIO_Pin_Pul = GPIO_Pin_4;
	
	compartments[BIN2].GPIOx_Dir = GPIOD;
	compartments[BIN2].GPIO_Pin_Dir = GPIO_Pin_7;
	compartments[BIN2].GPIOx_Pul = GPIOD;
	compartments[BIN2].GPIO_Pin_Pul = GPIO_Pin_5;
	
	compartments[BIN3].GPIOx_Dir = GPIOD;
	compartments[BIN3].GPIO_Pin_Dir = GPIO_Pin_3;
	compartments[BIN3].GPIOx_Pul = GPIOD;
	compartments[BIN3].GPIO_Pin_Pul = GPIO_Pin_1;	
	
	compartments[BIN4].GPIOx_Dir = GPIOC;
	compartments[BIN4].GPIO_Pin_Dir = GPIO_Pin_12;
	compartments[BIN4].GPIOx_Pul = GPIOC;
	compartments[BIN4].GPIO_Pin_Pul = GPIO_Pin_10;	
	
  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	/* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5 | GPIO_Pin_3 | GPIO_Pin_1;
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	

	/* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 

	GPIO_WriteBit(GPIOB, GPIO_Pin_6,Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_4,Bit_SET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_7,Bit_RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_5,Bit_RESET);
	
	
	GPIO_WriteBit(GPIOD, GPIO_Pin_3,Bit_RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_1,Bit_RESET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_10,Bit_RESET);

	
	
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED5);
  STM_EVAL_LEDOff(LED6);
}


void init_Compartments(){
	
	sscanf( msg, "%d %d %d %d %d", &command, &(compartments[BIN1].dosage), &(compartments[BIN2].dosage), &(compartments[BIN3].dosage),&(compartments[BIN4].dosage) );	
	compartments[BIN1].pillFall = 0;
	compartments[BIN1].stepCount = 0;
	compartments[BIN1].forFlag = 1;
	compartments[BIN1].iterator = 0;
	compartments[BIN1].iterator2 = 0;
	compartments[BIN1].dir = CW;
	//compartments[BIN1].dosage = dosageInt[BIN1];
	compartments[BIN1].preState = IDLE;
	compartments[BIN1].state = IDLE;
	
	compartments[BIN2].pillFall = 0;
	compartments[BIN2].stepCount = 0;
	compartments[BIN2].forFlag = 1;
	compartments[BIN2].iterator = 0;
	compartments[BIN2].iterator2 = 0;
	compartments[BIN2].dir = CW;
	//compartments[BIN2].dosage = dosageInt[BIN2];
	compartments[BIN2].preState = IDLE;
	compartments[BIN2].state = IDLE;
	
	compartments[BIN3].pillFall = 0;
	compartments[BIN3].stepCount = 0;
	compartments[BIN3].forFlag = 1;
	compartments[BIN3].iterator = 0;
	compartments[BIN3].iterator2 = 0;
	compartments[BIN3].dir = CW;
	//compartments[BIN3].dosage = dosageInt[BIN3];
	compartments[BIN3].preState = IDLE;
	compartments[BIN3].state = IDLE;
	
	compartments[BIN4].pillFall = 0;
	compartments[BIN4].stepCount = 0;
	compartments[BIN4].forFlag = 1;
	compartments[BIN4].iterator = 0;
	compartments[BIN4].iterator2 = 0;
	compartments[BIN4].dir = CW;
	//compartments[BIN4].dosage = dosageInt[BIN4];
	compartments[BIN4].preState = IDLE;
	compartments[BIN4].state = IDLE;

	GPIO_WriteBit(GPIOB, GPIO_Pin_6,Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_4,Bit_RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_7,Bit_RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_5,Bit_RESET);
	
	GPIO_WriteBit(GPIOC, GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOC, GPIO_Pin_10,Bit_RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_3,Bit_RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_1,Bit_RESET);
}


void Delay2(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void clean(char* array,uint8_t size){
	uint8_t i = 0;
	for (i = 0 ; i < size ; i ++){
		array[i] = 0;
	}
}
#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
