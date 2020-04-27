/**
 * main.c
 */
#include <GPIO.h>
int main(void)
{

    SYSCTL_RCC_R |= (1<<11);
    SYSCTL_RCC_R &= ~(1<<22);
    SYSCTL_RCC_R |= (1<<4);
    SYSCTL_RCC_R &= ~(1<<5);
    SYSCTL_RCC_R |= (1<<13);

    GPIOQuickInit(PORTF,AHB,RCGC,0b00000010,OUT,Drive_8mA,PAD_NPU_NPD);

    GPIOQuickInit(PORTF,AHB,RCGC,0b00010001,IN,Drive_8mA,Pad_PU);

    GPIOWrite(PORTF,0b00000010,0x0);

    REG EN0_INT = INT+EN0;
    SETBIT(*EN0_INT,30);


    REG Int_Mask = PORTF_AHB + GPIOIM ;
    CLRBIT(*Int_Mask,4);

    REG Int_Sense = PORTF_AHB + GPIOIS ;
    CLRBIT(*Int_Sense,4);

    REG GPIOIBE_Reg = PORTF_AHB + GPIOIBE ;
    CLRBIT(*GPIOIBE_Reg,4);

    REG Int_Event = PORTF_AHB + GPIOIEV ;
    SETBIT(*Int_Event,4);

    REG GPIORIS_Reg = PORTF_AHB + GPIORIS ;
    CLRBIT(*GPIORIS_Reg,4);

    SETBIT(*Int_Mask,4);



	return 0;
}
