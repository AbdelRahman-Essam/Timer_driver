/**
 * main.c
 */
#include "std_types.h"
#include "GPIO.h"
#include "Reg_map.h"
#include "INT.h"
#include "GPTM_init.h"
int x=0;
u8 LED=(1<<1);
#define LED_RED (1<<1)
#define LED_BLUE (1<<2)
#define LED_GREEN (1<<3)
void PORTF_ISR(void);

int main(void)
{
	GPIOQuickInit(PORTF,APB,RCGC,0b00001110,OUT,Drive_8mA,PAD_NPU_NPD);
    GPTMQuickInit(Timer0, cons, AB, RCGCT, PERIODIC, COUNT_UP, STALL_DISABLE, 0x00F42400, NON_CAPTURE_MATCH, TIMER0_Handler);
    GPIOQuickInit(PORTF,AHB,RCGC,0b00010000,IN,Drive_8mA,Pad_PU);
    INT_Enable(INT_PortF);
    GPIOIntQuickInit(PORTF,0b00010000,Edge,Falling);
    INT_SetISR(INT_PortF, PORTF_ISR);
    while(1);
	return 0;
}
void TIMER0_Handler(void)
{
    if((GPIORead(0x40025000, LED)&LED) == 0) GPIOWrite(0x40025000, LED, LED);
    else GPIOWrite(0x40025000, LED, 0);

    TIMER0_ICR_R |= (1<<0);
}
void PORTF_ISR(void)
{
	x++;
	LED=(1<<((x%3)+1));
}
