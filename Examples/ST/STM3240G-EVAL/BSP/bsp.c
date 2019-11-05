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
*                                    MICRIUM BOARD SUPPORT PACKAGE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                         STM3240G-EVAL
*                                        Evaluation Board
*
* Filename      : bsp.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define   BSP_MODULE
#include  <bsp.h>
#include  <bsp_os.h>

#include  <stm32f4xx_hal.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  BSP_BIT_RCC_PLLCFGR_PLLM               25u
//#define  BSP_BIT_RCC_PLLCFGR_PLLM               8u
#define  BSP_BIT_RCC_PLLCFGR_PLLN              336u
#define  BSP_BIT_RCC_PLLCFGR_PLLP                2u
#define  BSP_BIT_RCC_PLLCFGR_PLLQ                7u


#define  BSP_GPIOD_LED4                        DEF_BIT_12
#define  BSP_GPIOD_LED3                        DEF_BIT_13
#define  BSP_GPIOD_LED5                        DEF_BIT_14
#define  BSP_GPIOD_LED6                        DEF_BIT_15

#define  BSP_GPIOC_SEGMENTA                     DEF_BIT_13
#define  BSP_GPIOC_SEGMENTB                     DEF_BIT_09
#define  BSP_GPIOE_SEGMENTC                     DEF_BIT_07
#define  BSP_GPIOA_SEGMENTD                     DEF_BIT_05
#define  BSP_GPIOA_SEGMENTE                     DEF_BIT_01
#define  BSP_GPIOD_SEGMENTF                     DEF_BIT_06
#define  BSP_GPIOE_SEGMENTG                     DEF_BIT_13
#define  BSP_GPIOC_SEGMENTH                     DEF_BIT_05
#define  BSP_GPIOC_SEGCOM1                      DEF_BIT_15
#define  BSP_GPIOD_SEGCOM2                      DEF_BIT_04
#define  BSP_GPIOD_SEGCOM3                      DEF_BIT_02
#define  BSP_GPIOE_SEGCOM4                      DEF_BIT_15

/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             REGISTERS
*********************************************************************************************************
*/

#define  BSP_REG_DEM_CR                       (*(CPU_REG32 *)0xE000EDFC)
#define  BSP_REG_DWT_CR                       (*(CPU_REG32 *)0xE0001000)
#define  BSP_REG_DWT_CYCCNT                   (*(CPU_REG32 *)0xE0001004)
#define  BSP_REG_DBGMCU_CR                    (*(CPU_REG32 *)0xE0042004)

/*
*********************************************************************************************************
*                                            REGISTER BITS
*********************************************************************************************************
*/

#define  BSP_DBGMCU_CR_TRACE_IOEN_MASK                   0x10
#define  BSP_DBGMCU_CR_TRACE_MODE_ASYNC                  0x00
#define  BSP_DBGMCU_CR_TRACE_MODE_SYNC_01                0x40
#define  BSP_DBGMCU_CR_TRACE_MODE_SYNC_02                0x80
#define  BSP_DBGMCU_CR_TRACE_MODE_SYNC_04                0xC0
#define  BSP_DBGMCU_CR_TRACE_MODE_MASK                   0xC0

#define  BSP_BIT_DEM_CR_TRCENA                    DEF_BIT_24

#define  BSP_BIT_DWT_CR_CYCCNTENA                 DEF_BIT_00

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  BSP_LED_Init        (void);
static  void  BSP_ButtonInit    (void);
static  void  BSP_SEGMENT_Init  (void);
/*
*********************************************************************************************************
*                                               BSP_Init()
*
* Description : Initialize the Board Support Package (BSP).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This function SHOULD be called before any other BSP function is called.
*
*               (2) CPU instruction / data tracing requires the use of the following pins :
*                   (a) (1) Aysynchronous     :  PB[3]
*                       (2) Synchronous 1-bit :  PE[3:2]
*                       (3) Synchronous 2-bit :  PE[4:2]
*                       (4) Synchronous 4-bit :  PE[6:2]
*
*                   (c) The application may wish to adjust the trace bus width depending on I/O
*                       requirements.
*               (3) The voltage scaling allows optimizing the power consumption when the device is
*                   clocked below the maximum system frequency, to update the voltage scaling value
*                   regarding system frequency refer to product datasheet.
*********************************************************************************************************
*/

void  BSP_Init (void)
{
    RCC_OscInitTypeDef  RCC_OscInitStruct;
    RCC_ClkInitTypeDef  RCC_ClkInitStruct;

    BSP_IntInit();

    HAL_RCC_DeInit();

    __HAL_RCC_PWR_CLK_ENABLE();                                 /* Enable Power Control clock.                          */
                                                                /* See Note 3.                                          */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

                                                                /* PLLCLK    = HSE * (PLLN / PLLM)      = 336MHz.       */
                                                                /* SYSCLK    = PLLCLK / PLLP            = 168MHz.       */
                                                                /* OTG_FSCLK = PLLCLK / PLLQ            =  48MHz.       */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = BSP_BIT_RCC_PLLCFGR_PLLM;
    RCC_OscInitStruct.PLL.PLLN       = BSP_BIT_RCC_PLLCFGR_PLLN;
    RCC_OscInitStruct.PLL.PLLP       = BSP_BIT_RCC_PLLCFGR_PLLP;
    RCC_OscInitStruct.PLL.PLLQ       = BSP_BIT_RCC_PLLCFGR_PLLQ;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);


    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;          /* HCLK    = AHBCLK  = PLL / AHBPRES(1) = 168MHz.       */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;           /* APB1CLK = AHBCLK  / APB1DIV(4)       = 42MHz (max).  */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;           /* APB2CLK = AHBCLK  / APB2DIV(2)       = 84MHz.        */
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

                                                                /* STM32F405x/407x/415x/417x Revision Z devices: ...... */
    if (HAL_GetREVID() == 0x1001)                               /* ....prefetch is supported                            */
    {
      __HAL_FLASH_PREFETCH_BUFFER_ENABLE();                     /* Enable the Flash prefetch                            */
    }

    BSP_LED_Init();                                             /* Init LEDs.                                           */
    BSP_ButtonInit();
    BSP_SEGMENT_Init();
    
#ifdef TRACE_EN                                                 /* See project / compiler preprocessor options.         */
    BSP_CPU_REG_DBGMCU_CR |=  BSP_DBGMCU_CR_TRACE_IOEN_MASK;    /* Enable tracing (see Note #2).                        */
    BSP_CPU_REG_DBGMCU_CR &= ~BSP_DBGMCU_CR_TRACE_MODE_MASK;    /* Clr trace mode sel bits.                             */
    BSP_CPU_REG_DBGMCU_CR |=  BSP_DBGMCU_CR_TRACE_MODE_SYNC_04; /* Cfg trace mode to synch 4-bit.                       */
#endif
}


/*
*********************************************************************************************************
*                                            BSP_CPU_ClkFreq()
*
* Description : Read CPU registers to determine the CPU clock frequency of the chip.
*
* Argument(s) : none.
*
* Return(s)   : The CPU clock frequency, in Hz.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT32U  BSP_CPU_ClkFreq (void)
{
    CPU_INT32U  hclk_freq;


    hclk_freq = HAL_RCC_GetHCLKFreq();
    return (hclk_freq);
}


/*
*********************************************************************************************************
*                                            HAL_InitTick()
*
* Description : This function has been overwritten from the STM32F4xx HAL libraries because Micrium's RTOS
*               has its own Systick initialization and because it is recomended to initialize the tick after
*               multi-tasking has started.
*
* Argument(s) : TickPriority          Tick interrupt priority.
*
* Return(s)   : HAL_OK.
*
* Caller(s)   : HAL_InitTick ()) is called automatically at the beginning of the program after reset by
*               HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
*
* Note(s)     : none.
*********************************************************************************************************
*/

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    HAL_NVIC_SetPriorityGrouping(0);

    if (OSRunning > 0u) {                                       /*Check if multi-tasking has started.                   */
        BSP_Tick_Init();
    }

    return (HAL_OK);
}


/*
*********************************************************************************************************
*                                            BSP_Tick_Init()
*
* Description : Initialize all the peripherals that required OS Tick services (OS initialized)
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/
void BSP_Tick_Init (void)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;

    cpu_clk_freq = BSP_CPU_ClkFreq();                           /* Determine SysTick reference freq.                    */

#if (OS_VERSION >= 30000u)
    cnts  = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;       /* Determine nbr SysTick increments.                    */
#else
    cnts  = cpu_clk_freq / (CPU_INT32U)OS_TICKS_PER_SEC;        /* Determine nbr SysTick increments.                    */
#endif

    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */
}


/*
*********************************************************************************************************
*                                           BSP_LED_Init()
*
* Description : Initialize any or all the LEDs on the board.
*
* Argument(s) : led     The ID of the LED to control:
*
*                       0    inialize ALL  LEDs
*                       3    inialize user LED3
*                       4    inialize user LED4
*                       5    inialize user LED5
*                       6    inialize user LED6
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static void  BSP_LED_Init()
{
    GPIO_InitTypeDef  gpio_init;


    BSP_PeriphEn(BSP_PERIPH_ID_GPIOD);                          /* Configure GPIOD for LED3-6       */

    gpio_init.Pin   = BSP_GPIOD_LED3 | BSP_GPIOD_LED4 | BSP_GPIOD_LED5 | BSP_GPIOD_LED6;
    gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull  = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_HIGH;

    HAL_GPIO_Init(GPIOD, &gpio_init);

}


/*
*********************************************************************************************************
*                                             BSP_LED_On()
*
* Description : Turn ON any or all the LEDs on the board.
*
* Argument(s) : led     The ID of the LED to control:
*
*                       0    turns ON ALL  LEDs
*                       3    turns ON user LED3
*                       4    turns ON user LED4
*                       5    turns ON user LED5
*                       6    turns ON user LED4
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LED_On (CPU_INT08U  led)
{
    switch (led) {
        case 0u:
             HAL_GPIO_WritePin(GPIOD, (BSP_GPIOD_LED3 | BSP_GPIOD_LED4 | BSP_GPIOD_LED5 | BSP_GPIOD_LED6), GPIO_PIN_SET);
             break;


        case 3u:
             HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_LED3, GPIO_PIN_SET);
             break;


        case 4u:
             HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_LED4, GPIO_PIN_SET);
             break;


        case 5u:
             HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_LED5, GPIO_PIN_SET);
             break;


        case 6u:
             HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_LED6, GPIO_PIN_SET);
             break;


        default:
             break;
    }
}


/*
*********************************************************************************************************
*                                              BSP_LED_Off()
*
* Description : Turn OFF any or all the LEDs on the board.
*
* Argument(s) : led     The ID of the LED to control:
*
*                       0    turns OFF ALL the LEDs
*                       3    turns OFF user LED3
*                       4    turns OFF user LED4
*                       5    turns OFF user LED5
*                       6    turns OFF user LED6
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LED_Off (CPU_INT08U led)
{
    switch (led) {
        case 0u:
             HAL_GPIO_WritePin(GPIOD, (BSP_GPIOD_LED3 | BSP_GPIOD_LED4 | BSP_GPIOD_LED5 | BSP_GPIOD_LED6 ), GPIO_PIN_RESET);
             break;


        case 3u:
             HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_LED3, GPIO_PIN_RESET);
             break;


        case 4u:
             HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_LED4, GPIO_PIN_RESET);
             break;


        case 5u:
             HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_LED5, GPIO_PIN_RESET);
             break;


        case 6u:
             HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_LED6, GPIO_PIN_RESET);
             break;


        default:
             break;
    }
}


/*
*********************************************************************************************************
*                                            BSP_LED_Toggle()
*
* Description : TOGGLE any or all the LEDs on the board.
*
* Argument(s) : led     The ID of the LED to control:
*
*                       0    TOGGLE ALL the LEDs
*                       3    TOGGLE user LED3
*                       4    TOGGLE user LED4
*                       5    TOGGLE user LED5
*                       6    TOGGLE user LED6
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LED_Toggle (CPU_INT08U  led)
{
    switch (led) {
        case 0u:
             HAL_GPIO_TogglePin(GPIOD,(BSP_GPIOD_LED3 | BSP_GPIOD_LED4 | BSP_GPIOD_LED5 | BSP_GPIOD_LED6));
             break;


        case 3u:
             HAL_GPIO_TogglePin(GPIOD,BSP_GPIOD_LED3);
             break;


        case 4u:
             HAL_GPIO_TogglePin(GPIOD, BSP_GPIOD_LED4);
             break;


        case 5u:
             HAL_GPIO_TogglePin(GPIOD, BSP_GPIOD_LED5);
             break;


        case 6u:
             HAL_GPIO_TogglePin(GPIOD, BSP_GPIOD_LED6);
             break;


        default:
             break;
    }
}

/*
*********************************************************************************************************
*                                           BSP_ButtonInit()
*
* Description : Initialize the STM32F4DISCOVERY's user button
*
* Argument(s) : none
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static void  BSP_ButtonInit()
{
    GPIO_InitTypeDef gpio_init;
      
    BSP_PeriphEn(BSP_PERIPH_ID_GPIOA);
    gpio_init.Pin = GPIO_PIN_0;
    gpio_init.Mode = GPIO_MODE_IT_RISING;
    gpio_init.Pull  = GPIO_PULLDOWN;
    
    HAL_GPIO_Init(GPIOA, &gpio_init);

}  

/*
*********************************************************************************************************
*                                           BSP_ButtonClearIT()
*
* Description : Clear Interrupt Pending due to the STM32F4DISCOVERY's user button
*
* Argument(s) : none
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_ButtonClearIT()
{
   __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
}  

/*
*********************************************************************************************************
*                                           BSP_SEGMENT_Init()
*
* Description : Initialize any or all the SEVEN SEGMENT LEDs Installed.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : 
  SEGMENTA    GPIOC_13
  SEGMENTB    GPIOC_9
  SEGMENTC    GPIOE_7
  SEGMENTD    GPIOA_5
  SEGMENTE    GPIOA_1
  SEGMENTF    GPIOD_6
  SEGMENTG    GPIOE_13
  SEGMENTH    GPIOC_5
  SEGCOM1     GPIOC_15
  SEGCOM2     GPIOD_4
  SEGCOM3     GPIOD_2
  SEGCOM4     GPIOE_15
*********************************************************************************************************
*/
static void BSP_SEGMENT_Init()
{
  GPIO_InitTypeDef      gpio_init;
  
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_HIGH;
  
  // Configure GPIOA
  BSP_PeriphEn(BSP_PERIPH_ID_GPIOA);
  gpio_init.Pin = BSP_GPIOA_SEGMENTD | BSP_GPIOA_SEGMENTE;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  
  // Configure GPIOC
  BSP_PeriphEn(BSP_PERIPH_ID_GPIOC);
  gpio_init.Pin = BSP_GPIOC_SEGMENTA | BSP_GPIOC_SEGMENTB | BSP_GPIOC_SEGMENTH | BSP_GPIOC_SEGCOM1;
  HAL_GPIO_Init(GPIOC, &gpio_init);
  
  // Configure GPIOD
  BSP_PeriphEn(BSP_PERIPH_ID_GPIOD);
  gpio_init.Pin = BSP_GPIOD_SEGMENTF | BSP_GPIOD_SEGCOM2 | BSP_GPIOD_SEGCOM3;
  HAL_GPIO_Init(GPIOD, &gpio_init);
  
  // Configure GPIOE
  BSP_PeriphEn(BSP_PERIPH_ID_GPIOE);
  gpio_init.Pin = BSP_GPIOE_SEGMENTC | BSP_GPIOE_SEGMENTG | BSP_GPIOE_SEGCOM4;
  HAL_GPIO_Init(GPIOE, &gpio_init);
}

/*
*********************************************************************************************************
*                                             BSP_SEGMET_On()
*
* Description : Turn ON any or all the LEDS in the SEGMENT specified by com
*
* Argument(s) : com     The Digit ID from 1 to 4 (left to right)
*               seg     Controls segment LEDS
*                       bit 7 - SEGMENTA, bit 6 - SEGMENTB, bit 5 - SEGMENTC, bit 4 - SEGMENTD
*                       bit 3 - SEGMENTE, bit 2 - SEGMENTF, bit 1 - SEGMENTG, bit 0 - SEGMENTH(DP
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_SEGMENT_On (CPU_INT08U  com, CPU_INT08U   seg)
{

  HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGCOM1, GPIO_PIN_SET);       
  HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGCOM2, GPIO_PIN_SET);        
  HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGCOM3, GPIO_PIN_SET);        
  HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGCOM4, GPIO_PIN_SET);   
  
  HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTA, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTB, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGMENTC, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, BSP_GPIOA_SEGMENTD, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, BSP_GPIOA_SEGMENTE, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGMENTF, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGMENTG, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTH, GPIO_PIN_RESET);
  
  
  if (seg & 0x80)   HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTA, GPIO_PIN_SET);
  if (seg & 0x40)   HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTB, GPIO_PIN_SET);  
  if (seg & 0x20)   HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGMENTC, GPIO_PIN_SET);
  if (seg & 0x10)   HAL_GPIO_WritePin(GPIOA, BSP_GPIOA_SEGMENTD, GPIO_PIN_SET);
  if (seg & 0x08)   HAL_GPIO_WritePin(GPIOA, BSP_GPIOA_SEGMENTE, GPIO_PIN_SET);
  if (seg & 0x04)   HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGMENTF, GPIO_PIN_SET);
  if (seg & 0x02)   HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGMENTG, GPIO_PIN_SET);
  if (seg & 0x01)   HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTH, GPIO_PIN_SET);
  
  switch (com) {
      case 1:
        HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGCOM1, GPIO_PIN_RESET);
        break;
      case 2:   
        HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGCOM2, GPIO_PIN_RESET);        
        break;
      case 3:   
        HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGCOM3, GPIO_PIN_RESET);        
        break;
      case 4:
        HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGCOM4, GPIO_PIN_RESET);
        break;
      default:
        break;
  }
}

/*
*********************************************************************************************************
*                                             BSP_SEGMET_Off()
*
* Description : Turn OFF All LEDs in the segment display
*
* Argument(s) : none.
* 
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_SEGMENT_Off ()
{

  HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGCOM1, GPIO_PIN_SET);       
  HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGCOM2, GPIO_PIN_SET);        
  HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGCOM3, GPIO_PIN_SET);        
  HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGCOM4, GPIO_PIN_SET);   
  
  HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTA, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTB, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGMENTC, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, BSP_GPIOA_SEGMENTD, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, BSP_GPIOA_SEGMENTE, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, BSP_GPIOD_SEGMENTF, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, BSP_GPIOE_SEGMENTG, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, BSP_GPIOC_SEGMENTH, GPIO_PIN_RESET);

}
