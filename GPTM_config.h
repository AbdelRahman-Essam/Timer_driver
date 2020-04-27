/*
 * Timer_config.h
 *
 *  Created on: Apr 26, 2020
 *      Author: Abdelrahman Essam
 */

#ifndef GPTM_CONFIG_H_
#define GPTM_CONFIG_H_

#define SYSCLK	16000000

typedef enum{
	Timer0 =  0x40030000,Timer1 = 0x40031000,Timer2 = 0x40032000, Timer3 = 0x40033000, Timer4 =0x40034000, Timer5 = 0x40035000,
	WTimer0 =  0x40036000,WTimer1 = 0x40037000,WTimer2 = 0x4004C000, WTimer3 = 0x4004D000, WTimer4 =0x4004E000, WTimer5 = 0x4004F000
}GPTM_t;
typedef enum{cons=0,indv=4
}GPTM_Cons_t;;
typedef enum{A=0,B=4,AB=0
}GPTM_MODULE_t;
typedef enum{RCGCT=0x604 ,SCGCT=0x704 ,DCGCT=0x804
}GPTM_ClockMode_t;
typedef enum{
    ONE_SHOT = 1,
    PERIODIC = 2,
    INPUT_EDGE_COUNT = 6,
    INPUT_EDGE_TIME = 3,
    PWM = 4,
    RTC = 5
}GPTM_MODE;


typedef enum{
    COUNT_DOWN = 0,
    COUNT_UP = 1
}GPTM_DIRECTION;

typedef enum{
    STALL_DISABLE = 0,
    STALL_ENABLE = 1
}GPTM_STALL;

typedef enum{
    INT_TIMEOUT = 0,
    CAPTURE_MATCH = 1,
    CAPTURE_EVENT = 2,
    INT_RTC = 3,
    NON_CAPTURE_MATCH = 4
}GPTM_INT;

typedef enum{
    UPDATE_NEXT_CYCLE = 0,
    UPDATE_NEXT_TIMEOUT = 1
}GPTM_UPDATE;

typedef enum{
    REG_ILD, REG_PR, REG_MATCH
}GPTM_REGS;

#endif /* GPTM_CONFIG_H_ */
