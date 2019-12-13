/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                           STM3240G-EVAL
*                                         Evaluation Board
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : DC
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  <stdarg.h>
#include  <stdio.h>
#include  <math.h>
#include  <stm32f4xx_hal.h>

#include  <cpu.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <os_app_hooks.h>

#include  <app_cfg.h>
#include  <bsp.h>


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
            /* --------------- APPLICATION GLOBALS ---------------- */
static  OS_TCB       AppTaskStartTCB;
static  CPU_STK      AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
uint8_t k;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg);
void BSP_LEDrefresh(uint8_t k);

/*
*********************************************************************************************************
*                                                main()
*********************************************************************************************************
*/
static void Button_ISR_Handler(void)
{
    k = k + 1;
    BSP_LEDrefresh(k);
    BSP_ButtonClearIT();
}

void BSP_LEDrefresh(uint8_t k)
{
    uint32_t bsrr_mask = 0;
    if(k & DEF_BIT_00) bsrr_mask |= DEF_BIT_12; // BSP_GPIOD_LED4
    else bsrr_mask |= DEF_BIT_12 << 16;
    if(k & DEF_BIT_01) bsrr_mask |= DEF_BIT_13; // BSP_GPIOD_LED3
    else bsrr_mask |= DEF_BIT_13 << 16;
    if(k & DEF_BIT_02) bsrr_mask |= DEF_BIT_14; // BSP_GPIOD_LED5
    else bsrr_mask |= DEF_BIT_14 << 16;
    if(k & DEF_BIT_03) bsrr_mask |= DEF_BIT_15; // BSP_GPIOD_LED6
    else bsrr_mask |= DEF_BIT_15 << 16;

    GPIOD->BSRR = bsrr_mask;
}

int main(void)
{
    OS_ERR   err;

    HAL_Init();                                                 /* See Note 1.                                          */

    Mem_Init();                                                 /* Initialize Memory Managment Module                   */
    Math_Init();                                                /* Initialize Mathematical Module                       */

    BSP_IntDisAll();                                            /* Disable all Interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */
    App_OS_SetAllHooks();

    OSTaskCreate(&AppTaskStartTCB,                              /* Create the start task                                */
                  "App Task Start",
                  AppTaskStart,
                  0u,
                  APP_CFG_TASK_START_PRIO,
                 &AppTaskStartStk[0u],
                  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

    while (DEF_ON) {                                            /* Should Never Get Here.                               */
        ;
    }
}

/*
*********************************************************************************************************
*                                          STARTUP TASK
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
    OS_ERR      err;
    (void)p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

    BSP_IntVectSet(BSP_INT_ID_EXTI0, Button_ISR_Handler);
    BSP_IntEn(BSP_INT_ID_EXTI0);
    
    BSP_LED_Off(0u);
    BSP_SEGMENT_Off ();
    k=1;
    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
        // BSP_LED_Toggle(0u);
        // BSP_SEGMENT_On(k++,0xf2);
        if (k == 16) k=1; else k++;
        BSP_LEDrefresh(k);
        OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}

