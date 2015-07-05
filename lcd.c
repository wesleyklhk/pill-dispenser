#include "stm32f4xx.h"
#include "lcd.h"

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;

/**
 * Don't forget to uncomment the systick handler
 * if you want to be able to use this.
 */
/**
 static __IO uint32_t TimingDelay;

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}
*/

__inline __asm void delay100ns(){
	MOV 	R0, #4						; one instruction consumed (5.95ns) - 21d = 15h
loop4times								; takes (5.95ns)
	SUB 	R0, #1						; subtract from register (5.95ns)
	CMP		R0, #0						; compare if reached zero (5.95ns)
	BNE		loop4times				; conditional branch if result of comparison between R0 and #0 is 0
	BX		LR								; this statement is necessary to return to the calling function
}

/**
 * This function does a precise countdown using assembly.
 * The lcd_e_delay() function written for the Atmel at 4MHz
 * executes 2 instructions in a 250ns interval. We try to achieve
 * the same here using MiniDelay but since the Cortex runs at
 * 168MHz each instruction takes 5.95ns and therefore we need more
 * than just 2 instructions so we loop 5 times over 4 instructions
 * (plus two more instructions) in order to get a similar delay as the
 * Atmel - DELAY ESTIMATED AT: 511.7ns.
 */
__inline __asm void lcd_e_delay(){
	MOV 	R0, #21						; one instruction consumed (5.95ns) - 21d = 15h
loop21times								; takes (5.95ns)
	SUB 	R0, #1						; subtract from register (5.95ns)
	CMP		R0, #0						; compare if reached zero (5.95ns)
	BNE		loop21times				; conditional branch if result of comparison between R0 and #0 is 0
	BX		LR								; this statement is necessary to return to the calling function
}

/**
 * This function is meant to take a number of microseconds
 * and loop for that amount of time in order to create delay.
 * It assumes the processor runs at 168MHz. If the clock is
 * skewed the timing will be delayed accordingly.
 * 
 * Assuming one instruction takes 5.95ns then for one 1us
 * we need to execute 168 instructions. Since 'us' is in
 * microseconds then 1us = 23.8*R1 + 29.75 (the inner loop
 * takes 5.95*4*R1 = 23.8*R1, the outer loop takes 
 * (23.8*R1 + 29.75)*us ). Therefore assuming processor
 * runs at 168MHz we need R1 = 40. So when passing 1us to
 * this function the user will get an estimated delay of
 * 1.0115us (+1.15ns absolute error).
 */
__inline __asm void Delay(__IO uint32_t us /* in microseconds - volatile*/){ //us variable is thrown into R0 by default
loop_nus													; loop for n microseconds
	MOV		R1, #41				  					; the number of us we want to consume (load parameter into R1) - 41d =29h
loop_1us													; loop for 1 microsecond
	SUB 	R1, #1										; subtract from register (5.95ns)
	CMP		R1, #0										; compare if reached zero (5.95ns)
	BNE		loop_1us									; branch back if result of comparison between register and #0 is 0
	SUB		R0, #1										; subtract 1 from us parameter
  CMP		R0, #0										; compare if reached zero
	BNE		loop_nus									; branch back if the number of microseconds is not zero yet
	BX		LR												; this statement is necessary to return to the calling function
}

void lcd_init(uint8_t dispAttr){
	//__IO uint8_t outputdata; //holds whatever is on the port
	
	/* a few things will happen here so the 
	 * students won't need to do them:
	 *  - configuration of the "LCD port"
	 *    defined in the defines as PORT D 
	 *	- initializing the LCD
	 */
	
	/* we want to initialize delays that will enable
	 * to delay by microseconds
	 */
	//if (SysTick_Config(SystemCoreClock/1000000))
  //{ 
    /* Capture error */ 
  //  while (1);
  //}
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	
	/** configure the "LCD port" **/
	/* "LCD PORT" Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_PORT, ENABLE);
	/* Configure:
	 * pin 0,1,2 for output: (RS,RW,E)
	 * pin 4,5,6,7 for data output: DB11-DB14 in 4-bit mode
	 */
  //first set output...
	GPIO_InitStructure.GPIO_Pin = RS_PORT_Pin | RW_PORT_Pin |
															  E_PORT_Pin | 
																DATA_PORT_Pin_0 | DATA_PORT_Pin_1 |
															  DATA_PORT_Pin_2 | DATA_PORT_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(LCD_PORT, &GPIO_InitStructure);
	/** //...then input pin
	GPIO_InitStructure.GPIO_Pin = DB7_PORT_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(LCD_PORT, &GPIO_InitStructure); **/
	/** this ends the configuration of the "LCD PORT" **/
	
	//outputdata = (uint8_t)GPIO_ReadOutputData(LCD_PORT);
  //outputdata &= 0x05; //we're only interested in the lower bits (RS,RW,E)
	
	Delay(50000);
	
	/** initialize the LCD display by software instruction **/
	GPIO_Write(LCD_PORT, (1<<LCD_DATA0_PIN) | (1<<LCD_DATA1_PIN));
	//enable writing to LCD
	lcd_e_toggle();
	Delay(5000);
	lcd_e_toggle();
	Delay(5000);
	lcd_e_toggle();
	//lcd_waitbusy();
	Delay(160);
	
	//configure for 4-bit mode
	GPIO_Write(LCD_PORT, 1<<LCD_DATA1_PIN);
	lcd_e_toggle();
	Delay(160);
	
	
	
	lcd_command(LCD_FUNCTION_4BIT_2LINES);      /* function set: display lines  */
	lcd_command(LCD_DISP_OFF);              /* display off                  */
	lcd_clrscr();                           /* display clear                */
	lcd_command(LCD_MODE_DEFAULT);          /* set entry mode               */
	/** end initialize the LCD **/
	
	lcd_command(dispAttr);
}

uint8_t lcd_waitbusy(){
		__IO uint8_t c;
		//wait until the DB7 signal is gone
		while( (c=lcd_read(0)) & (1<<LCD_BUSY));
	
		Delay(16);
	
		//return address counter
		return lcd_read(0);
}

uint8_t lcd_read(uint8_t rs){
		__IO uint8_t data;
	
		if(rs){ //read data
			lcd_rs_high();
		}else{ //read busy flag
			lcd_rs_low();
		}
		lcd_rw_high(); // read mode
		
		//set data pins to input
		GPIO_InitStructure.GPIO_Pin = DATA_PORT_Pin_0 | DATA_PORT_Pin_1 |
																	DATA_PORT_Pin_2 | DATA_PORT_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(LCD_PORT, &GPIO_InitStructure);
		
		//read 2 nibbles
		lcd_e_high(); //enable
		lcd_e_delay();
		data = (uint8_t)GPIO_ReadInputData(LCD_PORT) << 4;
		lcd_e_low(); //disable
		
		lcd_e_delay();
		
		lcd_e_high(); //enable
		lcd_e_delay();
		data |= (uint8_t)GPIO_ReadInputData(LCD_PORT) & 0x0F;
		lcd_e_low(); //disable
		
		return data;
}

void lcd_write(uint8_t data, uint8_t rs){
		__IO uint8_t outputdata;
		if(rs){ //write data
			lcd_rs_high();
		}else{ //write instruction
			lcd_rs_low();
		}
		lcd_rw_low(); //set to write
		
		//set data pins to output
		GPIO_InitStructure.GPIO_Pin = DATA_PORT_Pin_0 | DATA_PORT_Pin_1 |
																	DATA_PORT_Pin_2 | DATA_PORT_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(LCD_PORT, &GPIO_InitStructure);
		
		/**
		 * Preserve the RS,RW,E bits in order to write out:
		 * our data goe son the 4 most significant bits with
		 * the high nibble output first.
		 */
		
		outputdata = (uint8_t)GPIO_ReadOutputData(LCD_PORT);
		outputdata &= 0x05; //we're only interested in the lower bits (RS,RW,E)
		  
		//output high nibble first
		GPIO_Write(LCD_PORT, outputdata | (data & 0xF0));
		//push data
		lcd_e_toggle();
		
		GPIO_Write(LCD_PORT, outputdata | ((data << 4) & 0xF0));
		//push data
		lcd_e_toggle(); 
		
		GPIO_Write(LCD_PORT, outputdata | 0xF0); //set pins high as inactive state
}

void lcd_command(uint8_t cmd)
{
    lcd_waitbusy();
    lcd_write(cmd,0);
		Delay(1000);
}

/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void lcd_clrscr(void)
{
    lcd_command(1<<LCD_CLR);
		//lcd_command(1<<LCD_HOME);
}

void lcd_e_high(){
		GPIO_WriteBit(LCD_PORT, E_PORT_Pin, Bit_SET);
}

void lcd_e_low(){
		GPIO_WriteBit(LCD_PORT, E_PORT_Pin, Bit_RESET);
}

void lcd_e_toggle(){
		lcd_e_low();
		lcd_e_delay();
		lcd_e_high();
		lcd_e_delay();
		lcd_e_low();
		Delay(100);
}

void lcd_rs_high(){
		GPIO_WriteBit(LCD_PORT, RS_PORT_Pin, Bit_SET);
}

void lcd_rs_low(){
		GPIO_WriteBit(LCD_PORT, RS_PORT_Pin, Bit_RESET);
}

void lcd_rw_high(){
		GPIO_WriteBit(LCD_PORT, RW_PORT_Pin, Bit_SET);
		//delay100ns();
}

void lcd_rw_low(){
		GPIO_WriteBit(LCD_PORT, RW_PORT_Pin, Bit_RESET);
		//delay100ns();
}

/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void lcd_gotoxy(uint8_t x, uint8_t y)
{
    if ( y==0 ) 
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
    else
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
}/* lcd_gotoxy */

int lcd_getxy(void)
{
    return lcd_waitbusy();
}

/*************************************************************************
Send data byte to LCD controller 
Input:   data to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_data(uint8_t data)
{
    lcd_waitbusy();
    lcd_write(data,1);
}

/*************************************************************************
Set cursor to home position
*************************************************************************/
void lcd_home(void)
{
    lcd_command(1<<LCD_HOME);
}

/*************************************************************************
Display character at current cursor position 
Input:    character to be displayed                                       
Returns:  none
*************************************************************************/
void lcd_putc(char c)
{
    __IO uint8_t pos;


    pos = lcd_waitbusy();   // read busy-flag and address counter
    if (c=='\n')
    {
        lcd_newline(pos);
    }
    else
    {
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
        }else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ){
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }
        lcd_waitbusy();
        lcd_write(c, 1);
    }

}/* lcd_putc */

/*************************************************************************
Move cursor to the start of next line or to the first line if the cursor 
is already on the last line.
*************************************************************************/
void lcd_newline(uint8_t pos)
{
    __IO uint8_t addressCounter;

    if ( pos < (LCD_START_LINE2) )
        addressCounter = LCD_START_LINE2;
    else
        addressCounter = LCD_START_LINE1;
    lcd_command((1<<LCD_DDRAM)+addressCounter);

}/* lcd_newline */

/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void lcd_puts(char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd_putc(c);
    }

}/* lcd_puts */
