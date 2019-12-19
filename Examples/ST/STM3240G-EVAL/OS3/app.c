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
*                                       DEFINE AND TYPEDEF
*********************************************************************************************************
*/

#define LP_TH 5

#define KEY_UP 3
#define KEY_LEFT 5
#define KEY_CLICK 6
#define KEY_RIGHT 7
#define KEY_DOWN 9
#define KEY_OMIT 13

typedef union _dot_t
{
    uint64_t i;
    uint8_t v[8];
} dot_t;

typedef struct _dot_state
{
    dot_t port;
    uint8_t x;
    uint8_t y;
    CPU_BOOLEAN show;
} dot_state_t;

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static OS_TCB AppTaskStartTCB;
static CPU_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB DotMatRefreshTaskTCB;
static CPU_STK DotMatRefreshTaskStk[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB KeyEventTaskTCB;
static CPU_STK KeyEventTaskStk[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB Level1TaskTCB;
static CPU_STK Level1TaskStk[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB Level2TaskTCB;
static CPU_STK Level2TaskStk[APP_CFG_TASK_START_STK_SIZE];

static OS_Q KeyPressEvent_Q;
static OS_Q DotMatRefresh_Q;

static OS_TMR blink_TMR;

dot_t dot_buf;
dot_state_t dot_state;
CPU_BOOLEAN blink;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void AppTaskStart(void  *p_arg);
static void DotMatRefreshTask(void *p_arg);
static void KeyEventTask(void *p_arg);
static void Level1Task(void *p_arg);
static void Level2Task(void *p_arg);

static void tmr1_callback(void);

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

static void tmr1_callback(void)
{
    blink = blink ? 0 : 1;
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

    OS_MSG_SIZE  msg_size;
    CPU_TS ts;
    uint32_t keyevent;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */
    
    BSP_LED_Off(0u);

    OSQCreate(
        (OS_Q *)&KeyPressEvent_Q,
        (CPU_CHAR *)"KeyPressEvent",
        (OS_MSG_QTY)32,
        (OS_ERR *)&err
    );

    OSQCreate(
        (OS_Q *)&DotMatRefresh_Q,
        (CPU_CHAR *)"DotMatRefresh",
        (OS_MSG_QTY)8,
        (OS_ERR *)&err
    );

    OSTaskCreate(
        &DotMatRefreshTaskTCB,
        "DotMat Task",
        DotMatRefreshTask,
        0u,
        12u,
        &DotMatRefreshTaskStk[0u],
        DotMatRefreshTaskStk[APP_CFG_TASK_START_STK_SIZE / 10u],
        APP_CFG_TASK_START_STK_SIZE,
        0u,
        0u,
        0u,
        (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
        &err
    );

    OSTaskCreate(
        &KeyEventTaskTCB,
        "KeyEvent Task",
        KeyEventTask,
        0u,
        13u,
        &KeyEventTaskStk[0u],
        KeyEventTaskStk[APP_CFG_TASK_START_STK_SIZE / 10u],
        APP_CFG_TASK_START_STK_SIZE,
        0u,
        0u,
        0u,
        (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
        &err
    );

    /* OSTaskCreate(
        &Level1TaskTCB,
        "KeyEvent Task",
        Level1Task,
        0u,
        14u,
        &Level1TaskStk[0u],
        Level1TaskStk[APP_CFG_TASK_START_STK_SIZE / 10u],
        APP_CFG_TASK_START_STK_SIZE,
        0u,
        0u,
        0u,
        (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
        &err
    ); */

    OSTaskCreate(
        &Level2TaskTCB,
        "KeyEvent Task",
        Level2Task,
        0u,
        14u,
        &Level2TaskStk[0u],
        Level2TaskStk[APP_CFG_TASK_START_STK_SIZE / 10u],
        APP_CFG_TASK_START_STK_SIZE,
        0u,
        0u,
        0u,
        (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
        &err
    );

    OSTmrCreate(
        (OS_TMR *)&blink_TMR,
        (CPU_CHAR *)"blink Timer",
        (OS_TICK) 5,
        (OS_TICK) 5,
        (OS_OPT) OS_OPT_TMR_PERIODIC,
        (OS_TMR_CALLBACK_PTR)  tmr1_callback,
        (void *)0,
        (OS_ERR *)&err
    );

    dot_state.port.i = 0;
    dot_state.x = 0;
    dot_state.y = 0;
    dot_state.show = DEF_FALSE;

    while (DEF_TRUE) {
        OSTimeDlyHMSM(0u, 0u, 0u, 100u,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
}

static void KeyEventTask(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;

    uint16_t key_state;
    uint32_t key_log[16] = {0};
    CPU_BOOLEAN key_priv[16] = {0};
    /* st, go, lock, boot, 3, 6, 9, esc, 2, 5, 8, 0, 1, 4, 7, ent */
    const uint8_t key_cross[] = {
        11, 10, 15, 12, 3, 6, 9, 14, 2, 5, 8, 0, 1, 4, 7, 13
    };
    uint32_t tmp;

    while(DEF_TRUE)
    {
        key_state = BSP_KeyMat_read();
        for(uint32_t i = 0; i < 16; i++)
        {
            key_log[i] = (key_log[i] << 1) | ((key_state&(1<<i))?1:0);
            if((__builtin_popcount(key_log[i]) > LP_TH) && !key_priv[i])
            {
                key_priv[i] = DEF_TRUE;
                OSQPost(
                    (OS_Q *)&KeyPressEvent_Q,
                    (void *)(key_cross[i]+1),
                    (OS_MSG_SIZE) sizeof(uint16_t),
                    (OS_OPT) OS_OPT_POST_FIFO|OS_OPT_POST_ALL,
                    (OS_ERR *) &err
                );
            }
            else if((__builtin_popcount(key_log[i]) < LP_TH) && key_priv[i]) key_priv[i] = DEF_FALSE;
            else
            {
                __NOP();
            }
        }
        tmp = __builtin_popcount(key_log[11]);
        
        OSTimeDlyHMSM(0u, 0u, 0u, 10u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}

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

static void Level1Task(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;

    OS_MSG_SIZE  msg_size;
    CPU_TS ts;
    uint32_t keyevent;

    const uint64_t test_frame[] = {
        0x182462524a462418, // 0
        0x7c10101010141810, // 1
        0x7e0418204042423c, // 2, up
        0x003c42201820423c, // 3
        0x1010107e12141810, // 4, left
        0x1e2020201e02023e, // 5, sel
        0x1c22223e0202221c, // 6, right
        0x202020202022223e, // 7
        0x1c22221c2222221c, // 8, down
        0x1c22203c2222221c, // 9
        0x0008183878381808, // go
        0x007e7e7e7e7e7e00, // st
        0x3c66c39999db5a18, // boot, show
        0x0002ffffc2c0c000, // ent
        0xe020f827390f3907, // esc
        0x7e7e7e7e7e242418  // lock
    };

    while (DEF_TRUE) {
        uint32_t keyevent = (uint32_t)OSQPend(
            (OS_Q *)&KeyPressEvent_Q,
            (OS_TICK)0,
            (OS_OPT)(OS_OPT_PEND_BLOCKING),
            (OS_MSG_SIZE *)&msg_size,
            (CPU_TS *)&ts,
            (OS_ERR *)err
        );
        if(keyevent)
        {
            dot_buf.i = test_frame[keyevent - 1];
            OSQPost(
                (OS_Q *)&DotMatRefresh_Q,
                (void *)&dot_buf,
                (OS_MSG_SIZE) sizeof(dot_t),
                (OS_OPT) OS_OPT_POST_FIFO|OS_OPT_POST_NO_SCHED|OS_OPT_POST_ALL,
                (OS_ERR *) &err
            );
            OSTimeDlyHMSM(0u, 0u, 0u, 500u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
            dot_buf.i = 0;
            OSQPost(
                (OS_Q *)&DotMatRefresh_Q,
                (void *)&dot_buf,
                (OS_MSG_SIZE) sizeof(dot_t),
                (OS_OPT) OS_OPT_POST_FIFO|OS_OPT_POST_NO_SCHED|OS_OPT_POST_ALL,
                (OS_ERR *) &err
            );
            OSTimeDlyHMSM(0u, 0u, 0u, 500u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
        }
        OSTimeDlyHMSM(0u, 0u, 0u, 10u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}

static void Level2Task(void *p_arg)
{
    OS_ERR err;
    (void)p_arg;

    OS_MSG_SIZE  msg_size;
    CPU_TS ts;
    uint32_t keyevent;

    OSTmrStart(&blink_TMR, &err);

    while (DEF_TRUE) {
        while(keyevent = (uint32_t)OSQPend(
            (OS_Q *)&KeyPressEvent_Q,
            (OS_TICK)0,
            (OS_OPT)(OS_OPT_PEND_BLOCKING),
            (OS_MSG_SIZE *)&msg_size,
            (CPU_TS *)&ts,
            (OS_ERR *)err
        ))
        {
            if(keyevent == KEY_OMIT) dot_state.show = !dot_state.show;
            else if(!dot_state.show)
            {
                switch(keyevent)
                {
                    case KEY_DOWN:
                        dot_state.y = (dot_state.y == 0) ? 0 : dot_state.y - 1;
                        break;
                    case KEY_UP:
                        dot_state.y = (dot_state.y == 7) ? 7 : dot_state.y + 1;
                        break;
                    case KEY_RIGHT:
                        dot_state.x = (dot_state.x == 7) ? 7 : dot_state.x + 1;
                        break;
                    case KEY_LEFT:
                        dot_state.x = (dot_state.x == 0) ? 0 : dot_state.x - 1;
                        break;
                    case KEY_CLICK:
                        dot_state.port.v[dot_state.x] ^= 1 << dot_state.y;
                        break;
                    default:
                        break;
                }
            }
            OSTmrStart(&blink_TMR, &err);
        }
        if(!dot_state.show)
        {
            dot_buf.i = dot_state.port.i;
            dot_buf.v[dot_state.x] ^= blink << dot_state.y;
        }
        else
        {
            dot_buf.i = blink ? dot_state.port.i : 0;
        }
        OSQPost(
            (OS_Q *)&DotMatRefresh_Q,
            (void *)&dot_buf,
            (OS_MSG_SIZE) sizeof(dot_t),
            (OS_OPT) OS_OPT_POST_FIFO|OS_OPT_POST_NO_SCHED|OS_OPT_POST_ALL,
            (OS_ERR *) &err
        );
        OSTimeDlyHMSM(0u, 0u, 0u, 10u,
                    OS_OPT_TIME_HMSM_STRICT,
                    &err);
    }
}
