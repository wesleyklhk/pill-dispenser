#ifndef PRISTR_H
#define PRISTR_H
#include <stdint.h>
#include <stm32f4xx.h>
typedef struct
{
	uint16_t preState;
	uint16_t state;
	uint32_t dosage;
	uint16_t pillFall;
	uint16_t stepCount;
	uint16_t forFlag;
	uint16_t iterator;
	uint16_t iterator2;
	uint16_t dir;
	GPIO_TypeDef* GPIOx_Dir; 
	uint16_t GPIO_Pin_Dir;
	GPIO_TypeDef* GPIOx_Pul; 
	uint16_t GPIO_Pin_Pul;
} compartment;

#define IDLE 0
#define OPER 1
#define CHEC 100
#define DISP 101
#define WAIT 102
#define REPORT 10000

#define DFOR 4 //every FOR steps it spin in the reverse direction for BAC steps
#define DBAC 1

#define HFOR 3 //every FOR steps it spin in the reverse direction for BAC steps
#define HBAC 1

#define CW 0
#define ACW 1

#define BIN1 0
#define BIN2 1
#define BIN3 2
#define BIN4 3


#endif
