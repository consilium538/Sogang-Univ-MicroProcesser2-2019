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

typedef union _dot_t
{
    uint64_t i;
    uint8_t v[8];
} dot_t;

static  OS_TCB       AppTaskStartTCB;
static  CPU_STK      AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB       DotMatRefreshTaskTCB;
static  CPU_STK      DotMatRefreshTaskStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_Q         KeyPressEvent_Q;
static  OS_Q         DotMatRefresh_Q;

uint32_t k;
dot_t dot_buf;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg);
static void DotMatRefreshTask(void *p_arg);

/*
*********************************************************************************************************
*                                                main()
*********************************************************************************************************
*/

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

    uint16_t key_state;

    // for test
    uint32_t framecount = 0;
    uint64_t test_frame[] = {
        0xf173371ff8ecce8f,
        0x783937dffbec9c1e,
        0x3c389bffffd91c3c,
        0x8fceecf81f3773f1
    };

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */
    
    BSP_LED_Off(0u);

    /* OSQCreate(
        (OS_Q *)&KeyPressEvent_Q,
        (CPU_CHAR *)"KeyPressEvent",
        (OS_MSG_QTY)32,
        (OS_ERR *)&err
    ); */

    OSQCreate(
        (OS_Q *)&DotMatRefresh_Q,
        (CPU_CHAR *)"DotMatRefresh",
        (OS_MSG_QTY)8,
        (OS_ERR *)&err
    );

    OSTaskCreate(&DotMatRefreshTaskTCB,
                  "DotMat Task",
                  DotMatRefreshTask,
                  0u,
                  5u,
                 &DotMatRefreshTaskStk[0u],
                  DotMatRefreshTaskStk[APP_CFG_TASK_START_STK_SIZE / 10u],
                  APP_CFG_TASK_START_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &err);

    framecount = 0;

    while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */

        key_state = BSP_KeyMat_read();
        dot_buf.i = test_frame[framecount % 4];
        framecount++;
        // BSP_DotMat_write(key_slice, k);
        OSQPost(
            (OS_Q *)&DotMatRefresh_Q,
            (void *)&dot_buf,
            (OS_MSG_SIZE) sizeof(dot_t),
            (OS_OPT) OS_OPT_POST_FIFO|OS_OPT_POST_NO_SCHED|OS_OPT_POST_ALL,
            (OS_ERR *) &err
        );
        
        OSTimeDlyHMSM(0u, 0u, 0u, 100u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}

/* static void KeyEventTask(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;

    uint16_t key_state;

    while(DEF_TRUE)
    {
        key_state = BSP_KeyMat_read();
    }
} */

static void DotMatRefreshTask(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;

    uint8_t      colcount = 0;
    dot_t        dot_state = {0};
    void        *p_DotQ;
    OS_MSG_SIZE  msg_size;
    CPU_TS       ts;

    while(DEF_TRUE)
    {
        p_DotQ = OSQPend(
            (OS_Q *)&DotMatRefresh_Q,
            (OS_TICK)0,
            (OS_OPT)(OS_OPT_PEND_BLOCKING),
            (OS_MSG_SIZE *)&msg_size,
            (CPU_TS *)&ts,
            (OS_ERR *)err
        );

        if(p_DotQ)
            dot_state.i = *(uint64_t *)p_DotQ;

        if(colcount == 7)
            colcount = 0;
        else
            colcount++;

        BSP_DotMat_write(dot_state.v[colcount], colcount);
        OSTimeDlyHMSM(0u, 0u, 0u, 1u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}
