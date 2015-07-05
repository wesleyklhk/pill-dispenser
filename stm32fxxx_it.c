/**
  ******************************************************************************
  * @file    stm32fxxx_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
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
#include "stm32fxxx_it.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "stm32f4_discovery.h"

#include "usbd_cdc_core.h"
#include "main.h"
#include "lcd.h"


#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include <stdio.h>
#include "private_structs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


#define ONEREV 800
#define FULLSTEPFACTOR 1
#define HALFSTEPFACTOR 2
#define ONEDIR 1
#define NUMOFBINS 4
#define FALSE 0
#define TRUE 1

/* Private variables ---------------------------------------------------------*/
extern uint16_t capture;
extern char lcd_buffer[17];    // LCD display buffer 
extern __IO uint16_t CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern __IO uint16_t CCR3_Val;
extern __IO uint16_t CCR4_Val;

extern uint16_t errorCode;
extern uint16_t state;
extern uint16_t preState;


uint16_t oneRevCount = ONEREV * FULLSTEPFACTOR;
uint8_t timeOutMode = TRUE;
extern compartment compartments[4];
uint8_t singleDirMode = ONEDIR;

/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
void reportResult(void);
void dirSet(uint16_t desiredDir);
void TimerCoreLogic(uint8_t binNum);
void PillDropSensorLogic(compartment* current);
/******************************************************************************/
/*             Cortex-M Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	system_time_counter++;
}

/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_WKUP_IRQHandler(void)
{
	/*
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ; 
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
	*/
  EXTI_ClearITPendingBit(EXTI_Line18);
}
void TIM3_IRQHandler(void)
{
	
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

    /* LED4 toggling with frequency = 4.57 Hz */
		//STM_EVAL_LEDToggle(LED4);
		if (state == OPER){
			TimerCoreLogic(BIN1);
		}	

		
		capture = TIM_GetCapture1(TIM3);
		TIM_SetCompare1(TIM3, capture + CCR1_Val);
	}  else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		//STM_EVAL_LEDToggle(LED5);
		if (state == OPER){
			TimerCoreLogic(BIN2);
		}	
		capture = TIM_GetCapture2(TIM3);
    TIM_SetCompare2(TIM3, capture + CCR2_Val);
	} else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		//STM_EVAL_LEDToggle(LED5);
		if (state == OPER){
			TimerCoreLogic(BIN3);
		}	
		capture = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, capture + CCR3_Val);
	}else if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		//STM_EVAL_LEDToggle(LED5);
		if (state == OPER){
			TimerCoreLogic(BIN4);
		}	
		capture = TIM_GetCapture4(TIM3);
    TIM_SetCompare4(TIM3, capture + CCR4_Val);
	}


}


/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void EXTI0_IRQHandler(void){ //user button	
	compartment* current = &(compartments[BIN1]);		
	
	 if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {

		PillDropSensorLogic(current);
//		lcd_clrscr();    //clears the LCD
//		lcd_gotoxy(0,0); 	//goes to bottom left
//		sprintf(lcd_buffer,"%i",current->pillFall); //Divides the int by 2 so that it accuratly describes milliseconds		
//		lcd_puts((char*) (lcd_buffer));//prints the int				
//		STM_EVAL_LEDToggle(LED3);
//		STM_EVAL_LEDToggle(LED6);
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
	
}

void EXTI2_IRQHandler(void){ //user button	
	compartment* current = &(compartments[BIN2]);		
	
	 if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {

		PillDropSensorLogic(current);
//		lcd_clrscr();    //clears the LCD
//		lcd_gotoxy(0,0); 	//goes to bottom left
//		sprintf(lcd_buffer,"%i",current->pillFall); //Divides the int by 2 so that it accuratly describes milliseconds		
//		lcd_puts((char*) (lcd_buffer));//prints the int				
//		STM_EVAL_LEDToggle(LED3);
//		STM_EVAL_LEDToggle(LED6);
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
	
}

void EXTI4_IRQHandler(void){ //user button	
	compartment* current = &(compartments[BIN3]);		
	
	 if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
		STM_EVAL_LEDOn(LED5);
		PillDropSensorLogic(current);
//		lcd_clrscr();    //clears the LCD
//		lcd_gotoxy(0,0); 	//goes to bottom left
//		sprintf(lcd_buffer,"%i",current->pillFall); //Divides the int by 2 so that it accuratly describes milliseconds		
//		lcd_puts((char*) (lcd_buffer));//prints the int				
//		STM_EVAL_LEDToggle(LED3);
//		STM_EVAL_LEDToggle(LED6);
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line4);
  }
	
}

void EXTI9_5_IRQHandler(void){ //user button	
	compartment* current = &(compartments[BIN4]);		
	
	 if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
//		STM_EVAL_LEDToggle(LED5);
		PillDropSensorLogic(current);
//		lcd_clrscr();    //clears the LCD
//		lcd_gotoxy(0,0); 	//goes to bottom left
//		sprintf(lcd_buffer,"%i",current->pillFall); //Divides the int by 2 so that it accuratly describes milliseconds		
//		lcd_puts((char*) (lcd_buffer));//prints the int				
//		STM_EVAL_LEDToggle(LED3);
//		STM_EVAL_LEDToggle(LED6);
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line6);
  }
	
}
void reportResult(void){
					
		uint8_t data_to_send[20];
		uint8_t temp[4];
		uint8_t i = 0;
		errorCode = 0;
		for (i = 0 ; i < 20 ; i ++ ){
			data_to_send[i] = 0;
		}
		for (i = BIN1 ; i <= BIN4 ; i++){
			temp[i] = compartments[i].dosage - compartments[i].pillFall;
			errorCode = errorCode + temp[i];
		}
		if (errorCode == 0){
			snprintf((char*)data_to_send, sizeof data_to_send, "%i%i%i%i%i", 
				errorCode,compartments[BIN1].pillFall,compartments[BIN2].pillFall,compartments[BIN3].pillFall,
				compartments[BIN4].pillFall);
		}else{
			snprintf((char*)data_to_send, sizeof data_to_send, "%i%i%i%i%i", 
				1,temp[BIN1],temp[BIN2],temp[BIN3],temp[BIN4]);
		}
		VCP_DataTx (data_to_send,20);

	
	//EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}

void TimerCoreLogic(uint8_t binNum){
		uint8_t temp;
		//char buffer[17];
		uint8_t i = 0;
		compartment* current = &(compartments[binNum]);

		STM_EVAL_LEDToggle(LED3);
//		STM_EVAL_LEDToggle(LED5);
//		STM_EVAL_LEDToggle(LED6);
		switch (current->state)
		{
			case DISP:
//				if (binNum == BIN1){
//					STM_EVAL_LEDOn(LED5);
//				}else if (binNum == BIN2){
////					lcd_clrscr();    //clears the LCD
////					lcd_gotoxy(0,0); 	//goes to bottom left
////					sprintf(lcd_buffer,"%i",current->stepCount); //Divides the int by 2 so that it accuratly describes milliseconds		
////					lcd_puts((char*) (lcd_buffer));//prints the int					
//					STM_EVAL_LEDOn(LED4);			
//				}else if (binNum == BIN3){
//					STM_EVAL_LEDOn(LED5);			
//				}else if (binNum == BIN4){
//					STM_EVAL_LEDOn(LED6);			
//				}				
				
				//if (current->pillFall < current->dosage){//if not enough pill drops, keep dropping
					///full_step();
				if (singleDirMode == ONEDIR){
					GPIO_ToggleBits(current->GPIOx_Pul,current->GPIO_Pin_Pul);
					
					current->preState = current->state;
					current->state = CHEC;
					current->stepCount++;
				}	
				break;
			
			case CHEC:
//				if (binNum == BIN1){
//					STM_EVAL_LEDOff(LED5);
//				}else if (binNum == BIN2){
////					lcd_clrscr();    //clears the LCD
////					lcd_gotoxy(0,0); 	//goes to bottom left
////					sprintf(lcd_buffer,"%i",current->stepCount); //Divides the int by 2 so that it accuratly describes milliseconds		
////					lcd_puts((char*) (lcd_buffer));//prints the int									
//					STM_EVAL_LEDOff(LED4);			
//				}else if (binNum == BIN3){
//					STM_EVAL_LEDOff(LED5);			
//				}else if (binNum == BIN4){
//					STM_EVAL_LEDOff(LED6);			
//				}		
				current->stepCount++;
				if (current->pillFall < current->dosage){
					if ((current->stepCount < oneRevCount) || (timeOutMode == FALSE)){
						current->preState = current->state;
						current->state = DISP;	
					}else{
						current->preState = current->state;
						current->state = WAIT;
					}
				}else{
						current->preState = current->state;
						current->state = WAIT;
				}
				break;
			
//				if ((current->stepCount < oneRevCount) || (timeOutMode == FALSE)){
//					if (current->pillFall < current->dosage){
//						current->preState = current->state;
//						current->state = DISP;						
//					}else{
//						current->preState = current->state;
//						current->state = WAIT;	
//					}
//				}else{
//					current->preState = current->state;
//					current->state = WAIT;
//				}
//				break;
				
			case WAIT:
//				if (binNum == BIN1){
//					STM_EVAL_LEDToggle(LED3);
//					STM_EVAL_LEDOff(LED4);	
//					STM_EVAL_LEDOff(LED5);	
//				}else if (binNum == BIN2){
//					STM_EVAL_LEDToggle(LED6);			
//					STM_EVAL_LEDOff(LED4);	
//					STM_EVAL_LEDOff(LED5);	
//				}else if (binNum == BIN3){
//					STM_EVAL_LEDToggle(LED5);			
//				}else if (binNum == BIN4){
//					STM_EVAL_LEDToggle(LED6);			
//				}

				temp = 0;
				for (i = BIN1 ; i <= BIN4 ; i++){
					temp = (compartments[i].state == WAIT) + temp;
				}

				if (temp >= NUMOFBINS){
					for (i = BIN1 ; i <= BIN4 ; i++){
						compartments[i].preState = WAIT;
						compartments[i].state = REPORT;
					}
					preState = OPER;
					state = REPORT;
					STM_EVAL_LEDOff(LED6);
					STM_EVAL_LEDOff(LED4);	
					STM_EVAL_LEDToggle(LED5);					
					STM_EVAL_LEDOff(LED3);	
//					lcd_clrscr();    //clears the LCD
//					lcd_gotoxy(0,0); 	//goes to bottom left
//					lcd_puts("DONE");//prints the int						
					return;
				}			
				break;
				
			default:
				return;
				//STM_EVAL_LEDToggle(LED6);
				break;

		}	
}
void PillDropSensorLogic(compartment* current){
		current->pillFall++;
		current->stepCount = 0;
		switch (current->state){
			case DISP:
				current->state = CHEC;
			case CHEC:
				current->state = CHEC;
			default:
				break;
		}
//		switch (current->state)
//		{
//			case DISP:
//				current->pillFall++;
//				current->preState = state;
//				current->state = CHEC;
//				current->forFlag = 1; 
//				current->iterator2 = 0;
//				//dirSet(CW,current);
//				break;
//			default:

				
//				break;
//		}	
		//EXTI_ClearITPendingBit(EXTI_Line0);
}



void dirSet(uint16_t desiredDir){

}

/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

