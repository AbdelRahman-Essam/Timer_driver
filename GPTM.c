/*
 * Timer.c
 *
 *  Created on: Apr 25, 2020
 *      Author: Abdelrahman Essam
 */
#include "GPTM_init.h"
static u8 TIMER_NVICVectors[26] = {35,36,37,38,39,40,51,52,86,87,108,109,110,111,112,113,114,115,116,117,118,119,120,121};



void 	GPTMModuleSet(GPTM_t Timer,GPTM_Cons_t Cons)
{/*Important: Bits in this register should only be changed when the TA/BEN bit in the GPTMCTL register is cleared.*/
	REG reg= Timer;
	u8 reg_data=*reg;
	reg_data&=~(7);
	reg_data|=Cons;
	*reg=reg_data;
}
u8 		GPTMModuleGet(GPTM_t Timer,GPTM_Cons_t Cons)
{
	REG reg= Timer;
	u8 reg_data=*reg;
	reg_data&=7;
	return(reg_data);
}
u8	 	GPTMIntTrans(GPTM_t Timer)
{		u8 timer=0;
		if(Timer==Timer0)
			timer=0;
		else if(Timer==Timer1)
			timer=1;
		else if(Timer==Timer2)
			timer=2;
		else if(Timer==Timer3)
			timer=3;
		else if(Timer==Timer4)
			timer=4;
		else if(Timer==Timer5)
			timer=5;
		else if(Timer ==WTimer0)
			timer=6;
		else if(Timer ==WTimer1)
			timer=7;
		else if(Timer ==WTimer2)
			timer=8;
		else if(Timer ==WTimer3)
			timer=9;
		else if(Timer ==WTimer4)
			timer=10;
		else if(Timer ==WTimer5)
			timer=11;
		return (timer);
}
u8 		GPTMTimerTrans(GPTM_t Timer)
{ u8 timer=0;
	if(Timer==Timer0|Timer==WTimer0)
		timer=0;
	else if(Timer==Timer1|Timer==WTimer1)
		timer=1;
	else if(Timer==Timer2|Timer==WTimer2)
		timer=2;
	else if(Timer==Timer3|Timer==WTimer3)
		timer=3;
	else if(Timer==Timer4|Timer==WTimer4)
		timer=4;
	else if(Timer==Timer5|Timer==WTimer5)
		timer=5;
return (timer);
}
void 	GPTMClockSet(GPTM_t Timer,GPTM_ClockMode_t ClockMode)
{
	u8 shift=0;
    u8 timer=GPTMTimerTrans(Timer);
    CLRBIT(SYSCTL_RCGCTIMER_R,timer);
    CLRBIT(SYSCTL_SCGCTIMER_R,timer);
    CLRBIT(SYSCTL_DCGCTIMER_R,timer);
    if(Timer>Timer5)
    	shift=58;
    REG reg = SYSCTL + ClockMode+shift;
    u8 reg_data=*reg;
    SETBIT(reg_data,timer);
    *reg = reg_data;
}
void 	GPTMDirection(GPTM_t Timer,GPTM_MODULE_t Module,GPTM_DIRECTION direction)
{
		REG reg_MR =Timer +GPTMTAMR+Module;
		u32 data_mr =*reg_MR;
	    data_mr &= ~(1<<4);
	    data_mr |= (direction<<4);
	    *reg_MR = data_mr;
}
void 	GPTMClockRst(GPTM_t Timer)
{
    u8 timer=GPTMTimerTrans(Timer);
    CLRBIT(SYSCTL_RCGCTIMER_R,timer);
    CLRBIT(SYSCTL_SCGCTIMER_R,timer);
    CLRBIT(SYSCTL_DCGCTIMER_R,timer);
}
void 	GPTMModeSet(GPTM_t Timer,GPTM_Cons_t Cons, GPTM_MODULE_t Module, GPTM_MODE mode)
{/*Important: Bits in this register should only be changed when the TA/BEN bit in the GPTMCTL register is cleared.*/
	REG reg_MR =Timer+Module +GPTMTAMR;
	u32 data_mr =*reg_MR;
	if(mode<=2)
	{
	    data_mr &= ~(3);
		data_mr |=mode;
		CLRBIT(data_mr,3);
	}
	else
	{
		data_mr |=3;
		CLRBIT(data_mr,3);
		if(mode==INPUT_EDGE_COUNT)
			CLRBIT(data_mr,2);
		else if(mode==INPUT_EDGE_COUNT)
			SETBIT(data_mr,2);
		else if(mode==PWM)
			SETBIT(data_mr,3);
	}
	*reg_MR=data_mr;
}
void 	GPTMDisable(GPTM_t Timer,GPTM_Cons_t Cons, GPTM_MODULE_t Module)
{
	REG reg_ctrl=Timer+GPTMCTL;
	u32 data_ctrl=*reg_ctrl;
    data_ctrl &= ~(1<<2*Module);
    *reg_ctrl = data_ctrl;
}
void 	GPTMReady(GPTM_t Timer,GPTM_Cons_t Cons)
{
	u8 shift=0;
	u8 timer=GPTMTimerTrans(Timer);
	if(Timer>Timer5)
	    	shift=58;
	    REG reg = SYSCTL + PRTIMER +shift;
	    while(!GETBIT(*reg,timer))
	    	;
}
void 	GPTMStall(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t Module, GPTM_STALL stall)
{
	REG reg_ctrl=Timer+GPTMCTL;
	u32 data_ctrl=*reg_ctrl;
	   data_ctrl &= ~(1<<(2*Module+1));
	   data_ctrl |= (stall<<(2*Module+1));
	   *reg_ctrl = data_ctrl;
}
void 	TIMER_Init(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t module,GPTM_ClockMode_t ClockMode, GPTM_MODE mode, GPTM_DIRECTION direction, GPTM_STALL stall)
{
	GPTMClockSet(Timer, ClockMode);
	GPTMReady(Timer, Cons);
	GPTMDisable(Timer, Cons, module);
	GPTMModuleSet(Timer, Cons);
	GPTMModeSet(Timer, Cons, module, mode);
	GPTMDirection(Timer, Cons, direction);
	GPTMStall(Timer, Cons, module, stall);
}
void	TIMERSetIntISR(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t module, GPTM_INT interrupt_Type, void (*function) (void))
{
	u8 module_index=0;
	REG reg_im= Timer+GPTMIMR;
	u32 data_im=*reg_im;
	REG reg_MR =Timer+module +GPTMTAMR;
	u32 data_mr =*reg_MR;

	    if(interrupt_Type == NON_CAPTURE_MATCH) *reg_MR = *reg_MR | (1<<5);
	    else if(interrupt_Type ==CAPTURE_EVENT && (*reg_MR&(1<<3))) *reg_MR = *reg_MR | (1<<9);

	    if(module == 4 && interrupt_Type == 4) interrupt_Type = 3;
	       else if(module == 4 && interrupt_Type == 3) return;

	    module_index=TIMER_NVICVectors[GPTMIntTrans(Timer)*2+(u8)(module*0.25)];
	    INT_Init();
	    g_pfnRAMVectors[module_index]=function;
	    INT_Enable(module_index);
	    TIMER0_IMR_R |= (1<<0);
}
u8 FANCYTIMER_ONESHOT_SetTimeoutSeconds(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t module,GPTM_ClockMode_t ClockMode, GPTM_MODE mode, u32 msec, void (*function) (void))
{
    float finterval_load = (1.0/(float)(SYSCLK));
    finterval_load = (float)((float)msec/1000.0) / (float)finterval_load;
    u32 interval_load = (u32)finterval_load;

    if(interval_load > 0xFFFF)
    {
        if(Cons != 0){return 0;}
    }

    TIMER_Init(Timer, Cons, module, ClockMode,ONE_SHOT, COUNT_UP, STALL_ENABLE);

    TIMER_SetIntervalLoad(Timer, module, (u32)interval_load);
    TIMERSetIntISR(Timer, Cons, module, INT_TIMEOUT, function);
    return 1;
}
void TIMER_Start(GPTM_t Timer,GPTM_MODULE_t module)
{
    REG reg_ctl =(Timer + (GPTMCTL));
    u32 data_ctl = *reg_ctl;
    data_ctl |= (1<<2*module);
    *reg_ctl = data_ctl;
}
void TIMER_Stop(GPTM_t Timer,GPTM_MODULE_t module)
{
    REG reg_ctrl =Timer +GPTMCTL;
    u32 data_ctrl = *reg_ctrl;
    data_ctrl &= ~(1<<2*module);
    *reg_ctrl = data_ctrl;
}
void TIMER_EnableSnapshot(GPTM_t Timer,GPTM_MODULE_t module)
{
    REG reg_mr =(Timer + (GPTMTAMR) + module);
    *reg_mr = *reg_mr | (1<<7);
}
void TIMER_DisableSnapshot(GPTM_t Timer,GPTM_MODULE_t module)
{
    REG reg_mr =(Timer + (GPTMTAMR) + module);
    *reg_mr = *reg_mr & (~(1<<7));
}
//Set the interval load and prescaler values
void TIMER_SetIntervalLoad(GPTM_t Timer,GPTM_MODULE_t module, u32 value)
{
    REG reg_ilr = (Timer + GPTMTAILR + module);
    *reg_ilr = value;
}
void TIMER_SetPrescalar(GPTM_t Timer,GPTM_MODULE_t module, u16 value)
{
    REG reg_pr = Timer + GPTMTAPR + module;
    *reg_pr = value;
}
//Set the values of the match register and the prescaler match register
void TIMER_SetMatch(GPTM_t Timer,GPTM_MODULE_t module, u32 value)
{
    REG reg_matchr = Timer + (GPTMTAMATCHR) + module;
    *reg_matchr = value;
}
void TIMER_SetPrescalerMatch(GPTM_t Timer,GPTM_MODULE_t module, u16 value)
{

    REG reg_pmr =(Timer + (GPTMTAPMR) + module);
    *reg_pmr = value;
}

//It will return the snapshot of the timer and optionally Prescaler snapshot if it is used
u32 TIMER_GetSnapshot(GPTM_t Timer,GPTM_MODULE_t module)
{
    REG reg_mr =(Timer + (GPTMTAMR) + module);
    REG reg_r  =(Timer + (GPTMTAR) + module);
    REG reg_ps =(Timer + (GPTMTAPS) + module); //B is always the following register after A
    u32 snapshot = 0;

    //Prescaler is always a time extension HERE
    //MYSTERY: PRESCALER SNAPSHOT IS ZERO ALL THE TIME IN ALL MODES
    //But the function is working well as reg_r has the whole value with the value the prescaler snapshot should have
    snapshot = (*reg_r);
    snapshot = snapshot | ((*reg_ps)<<16);
    return snapshot;
}


void 	GPTMQuickInit(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t module,GPTM_ClockMode_t ClockMode, GPTM_MODE mode, GPTM_DIRECTION direction, GPTM_STALL stall,u32 Initial_Value, GPTM_INT interrupt_Type, void (*function) (void))
{
	u32 value=Initial_Value;
	TIMER_Init(Timer, Cons, module, ClockMode, mode, direction, stall);
	TIMERSetIntISR(Timer, Cons, module, interrupt_Type, function);
	TIMER_SetIntervalLoad(Timer, module, value);
	TIMER_Start(Timer, module);
}



//void TIMER0B_ISR(void){
//    u16 i = 1<<8;
//    u8 bit_position = 0;
//    REG reg_icr =(Timer0 +GPTMICR);
//    REG reg_mis =(Timer0 +GPTMMIS);
//
//    while(((*reg_mis) & i) == 0){
//        i = i << 1;
//        bit_position++;
//        if(i>(1<<11)) return;
//    }
//
//    //TIMER0B_interrupt_sources[bit_position]();
//    all_ptrs[1][bit_position]();
//
//    *(reg_icr) = (*(reg_icr)) | (1<<(bit_position+8));
//}
//
//void TIMER2A_ISR(void){
//    uint16_t i = 1<<0;
//    uint8_t bit_position = 0;
//    volatile uint32_t *reg_icr = (volatile uint32_t *)(TIM2_AB + (TIMERICR));
//    volatile uint32_t *reg_mis = (volatile uint32_t *)(TIM2_AB + (TIMERMIS));
//
//    while(((*reg_mis) & i) == 0){
//        i = i << 1;
//        bit_position++;
//        if(i>(1<<4)) return;
//    }
//
//    //TIMER0A_interrupt_sources[bit_position]();
//    all_ptrs[4][bit_position]();
//
//    *(reg_icr) = (*(reg_icr)) | (1<<bit_position);
//}
//
//void TIMER2B_ISR(void){
//    uint16_t i = 1<<8;
//    uint8_t bit_position = 0;
//    volatile uint32_t *reg_icr = (volatile uint32_t *)(TIM2_AB + (TIMERICR));
//    volatile uint32_t *reg_mis = (volatile uint32_t *)(TIM2_AB + (TIMERMIS));
//
//    while(((*reg_mis) & i) == 0){
//        i = i << 1;
//        bit_position++;
//        if(i>(1<<11)) return;
//    }
//
//    //TIMER0B_interrupt_sources[bit_position]();
//    all_ptrs[5][bit_position]();
//
//    *(reg_icr) = (*(reg_icr)) | (1<<(bit_position+8));
//}
//
//void TIMER5A_ISR(void){
//    uint16_t i = 1<<0;
//    uint8_t bit_position = 0;
//    volatile uint32_t *reg_icr = (volatile uint32_t *)(TIM5_AB + (TIMERICR));
//    volatile uint32_t *reg_mis = (volatile uint32_t *)(TIM5_AB + (TIMERMIS));
//
//    while(((*reg_mis) & i) == 0){
//        i = i << 1;
//        bit_position++;
//        if(i>(1<<4)) return;
//    }
//
//    //TIMER0A_interrupt_sources[bit_position]();
//    all_ptrs[10][bit_position]();
//
//    *(reg_icr) = (*(reg_icr)) | (1<<bit_position);
//}
//
//void TIMER5B_ISR(void){
//    uint16_t i = 1<<8;
//    uint8_t bit_position = 0;
//    volatile uint32_t *reg_icr = (volatile uint32_t *)(TIM5_AB + (TIMERICR));
//    volatile uint32_t *reg_mis = (volatile uint32_t *)(TIM5_AB + (TIMERMIS));
//
//    while(((*reg_mis) & i) == 0){
//        i = i << 1;
//        bit_position++;
//        if(i>(1<<11)) return;
//    }
//
//    //TIMER0B_interrupt_sources[bit_position]();
//    all_ptrs[11][bit_position]();
//
//    *(reg_icr) = (*(reg_icr)) | (1<<(bit_position+8));
//}















