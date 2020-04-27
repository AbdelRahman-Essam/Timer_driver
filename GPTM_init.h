/*
 * Timer.h
 *
 *  Created on: Apr 25, 2020
 *      Author: Abdelrahman Essam
 */
#ifndef GPTM_INIT_H_
#define GPTM_INIT_H_
#include "GPTM_config.h"
#include "utils.h"
#include "std_types.h"
#include "GPIO.h"
#include "INT.h"

void 	GPTMQuickInit(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t module,GPTM_ClockMode_t ClockMode, GPTM_MODE mode, GPTM_DIRECTION direction, GPTM_STALL stall,u32 value, GPTM_INT interrupt_Type, void (*function) (void));

void 	TIMER_Init(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t module,GPTM_ClockMode_t ClockMode, GPTM_MODE mode, GPTM_DIRECTION direction, GPTM_STALL stall);
void 	TIMERSetIntISR(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t module, GPTM_INT interrupt_Type, void (*function) (void));
u8 		FANCYTIMER_ONESHOT_SetTimeoutSeconds(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t Module,GPTM_ClockMode_t ClockMode, GPTM_MODE mode, u32 msec, void (*function) (void));
void TIMER_Start(GPTM_t Timer,GPTM_MODULE_t module);
void TIMER_Stop(GPTM_t Timer,GPTM_MODULE_t module);
void TIMER_EnableSnapshot(GPTM_t Timer,GPTM_MODULE_t module);
void TIMER_DisableSnapshot(GPTM_t Timer,GPTM_MODULE_t module);
void TIMER_SetIntervalLoad(GPTM_t Timer,GPTM_MODULE_t module, u32 value);
void TIMER_SetPrescalar(GPTM_t Timer,GPTM_MODULE_t module, u16 value);
void TIMER_SetMatch(GPTM_t Timer,GPTM_MODULE_t module, u32 value);
void TIMER_SetPrescalerMatch(GPTM_t Timer,GPTM_MODULE_t module, u16 value);
u32 TIMER_GetSnapshot(GPTM_t Timer,GPTM_MODULE_t module);






/************************Internal Functions*************************/
void 	GPTMModuleSet(GPTM_t Timer,GPTM_Cons_t Cons);
u8 		GPTMModuleGet(GPTM_t Timer,GPTM_Cons_t Cons);
void 	GPTMClockSet(GPTM_t Timer,GPTM_ClockMode_t ClockMode);
void 	GPTMClockRst(GPTM_t Timer);
void 	GPTMDirection(GPTM_t Timer,GPTM_MODULE_t Module,GPTM_DIRECTION direction);
u8 		GPTMTimerTrans(GPTM_t Timer);
void 	GPTMModeSet(GPTM_t Timer,GPTM_Cons_t Cons, GPTM_MODULE_t Module, GPTM_MODE Mode);
void 	GPTMDisable(GPTM_t Timer,GPTM_Cons_t Cons, GPTM_MODULE_t Module);
void 	GPTMReady(GPTM_t Timer,GPTM_Cons_t Cons);
void 	GPTMStall(GPTM_t Timer,GPTM_Cons_t Cons,GPTM_MODULE_t Module, GPTM_STALL stall);
u8	 	GPTMIntTrans(GPTM_t Timer);

#endif /* GPTM_INIT_H_ */
