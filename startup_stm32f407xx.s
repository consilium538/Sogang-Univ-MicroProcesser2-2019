/**
  ******************************************************************************
  * @file      startup_stm32f407xx.s
  * @author    MCD Application Team
  * @brief     STM32F407xx Devices vector table for GCC based toolchains. 
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
    
  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

/* start address for the initialization values of the .data section. 
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */  
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called. 
 * @param  None
 * @retval : None
*/

    .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:  
  ldr   sp, =_estack     /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */  
  movs  r1, #0
  b  LoopCopyDataInit

CopyDataInit:
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds  r1, r1, #4
    
LoopCopyDataInit:
  ldr  r0, =_sdata
  ldr  r3, =_edata
  adds  r2, r0, r1
  cmp  r2, r3
  bcc  CopyDataInit
  ldr  r2, =_sbss
  b  LoopFillZerobss
/* Zero fill the bss segment. */  
FillZerobss:
  movs  r3, #0
  str  r3, [r2], #4
    
LoopFillZerobss:
  ldr  r3, = _ebss
  cmp  r2, r3
  bcc  FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit
  
/* Enable Floating Point Support at reset for FPU */
  /* Load address of CPACR register */
  ldr.w  r0, =0xE000ED88
  /* Read value at CPACR */
  ldr  r1, [r0]
  /* Set bits 20-23 to enable CP10 and CP11 coprocessors */
  orr  r1,  r1, #(0xF <<20)
  /* Write back the modified CPACR value */
  str  r1, [r0]
  /* Wait for store to complete */
  dsb
  
/* Disable automatic FP register content */
/* Disable lazy context switch */
  
  /* Load address to FPCCR register */
  ldr.w  R0, =0xE000EF34         
  ldr  R1, [R0]
  /* Clear the LSPEN and ASPEN bits */
  and  R1,  R1, #(0x3FFFFFFF)
  str  R1, [R0]
  /* Reset pipeline now the FPU is enabled */
  isb
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl  main
  bx  lr    
.size  Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an 
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None     
 * @retval None       
*/
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M3. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
* 
*******************************************************************************/
   .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors
    
    
g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  OS_CPU_PendSVHandler
  .word  OS_CPU_SysTickHandler
  
  /* External Interrupts */
  .word     BSP_IntHandlerWWDG                   /* Window WatchDog              */                                        
  .word     BSP_IntHandlerPVD                    /* PVD through EXTI Line detection */                        
  .word     BSP_IntHandlerTAMP_STAMP             /* Tamper and TimeStamps through the EXTI line */            
  .word     BSP_IntHandlerRTC_WKUP               /* RTC Wakeup through the EXTI line */                      
  .word     BSP_IntHandlerFLASH                  /* FLASH                        */                                          
  .word     BSP_IntHandlerRCC                    /* RCC                          */                                            
  .word     BSP_IntHandlerEXTI0                  /* EXTI Line0                   */                        
  .word     BSP_IntHandlerEXTI1                  /* EXTI Line1                   */                          
  .word     BSP_IntHandlerEXTI2                  /* EXTI Line2                   */                          
  .word     BSP_IntHandlerEXTI3                  /* EXTI Line3                   */                          
  .word     BSP_IntHandlerEXTI4                  /* EXTI Line4                   */                          
  .word     BSP_IntHandlerDMA1_CH0           /* DMA1 Stream 0                */                  
  .word     BSP_IntHandlerDMA1_CH1           /* DMA1 Stream 1                */                   
  .word     BSP_IntHandlerDMA1_CH2           /* DMA1 Stream 2                */                   
  .word     BSP_IntHandlerDMA1_CH3           /* DMA1 Stream 3                */                   
  .word     BSP_IntHandlerDMA1_CH4           /* DMA1 Stream 4                */                   
  .word     BSP_IntHandlerDMA1_CH5           /* DMA1 Stream 5                */                   
  .word     BSP_IntHandlerDMA1_CH6           /* DMA1 Stream 6                */                   
  .word     BSP_IntHandlerADC                    /* ADC1, ADC2 and ADC3s         */                   
  .word     BSP_IntHandlerCAN1_TX                /* CAN1 TX                      */                         
  .word     BSP_IntHandlerCAN1_RX0               /* CAN1 RX0                     */                          
  .word     BSP_IntHandlerCAN1_RX1               /* CAN1 RX1                     */                          
  .word     BSP_IntHandlerCAN1_SCE               /* CAN1 SCE                     */                          
  .word     BSP_IntHandlerEXTI9_5                /* External Line[9:5]s          */                          
  .word     BSP_IntHandlerTIM1_BRK_TIM9          /* TIM1 Break and TIM9          */         
  .word     BSP_IntHandlerTIM1_UP_TIM10          /* TIM1 Update and TIM10        */         
  .word     BSP_IntHandlerTIM1_TRG_COM_TIM11     /* TIM1 Trigger and Commutation and TIM11 */
  .word     BSP_IntHandlerTIM1_CC                /* TIM1 Capture Compare         */                          
  .word     BSP_IntHandlerTIM2                   /* TIM2                         */                   
  .word     BSP_IntHandlerTIM3                   /* TIM3                         */                   
  .word     BSP_IntHandlerTIM4                   /* TIM4                         */                   
  .word     BSP_IntHandlerI2C1_EV                /* I2C1 Event                   */                          
  .word     BSP_IntHandlerI2C1_ER                /* I2C1 Error                   */                          
  .word     BSP_IntHandlerI2C2_EV                /* I2C2 Event                   */                          
  .word     BSP_IntHandlerI2C2_ER                /* I2C2 Error                */                            
  .word     BSP_IntHandlerSPI1                   /* SPI1                         */                   
  .word     BSP_IntHandlerSPI2                   /* SPI2                         */                   
  .word     BSP_IntHandlerUSART1                 /* USART1                       */                   
  .word     BSP_IntHandlerUSART2                 /* USART2                       */                   
  .word     BSP_IntHandlerUSART3                 /* USART3                       */                   
  .word     BSP_IntHandlerEXTI15_10              /* External Line[15:10]s        */                          
  .word     BSP_IntHandlerRTCAlarm              /* RTC Alarm (A and B) through EXTI Line */                 
  .word     BSP_IntHandlerOTG_FS_WKUP            /* USB OTG FS Wakeup through EXTI line */                       
  .word     BSP_IntHandlerTIM8_BRK_TIM12         /* TIM8 Break and TIM12         */         
  .word     BSP_IntHandlerTIM8_UP_TIM13          /* TIM8 Update and TIM13        */         
  .word     BSP_IntHandlerTIM8_TRG_COM_TIM14     /* TIM8 Trigger and Commutation and TIM14 */
  .word     BSP_IntHandlerTIM8_CC                /* TIM8 Capture Compare         */                          
  .word     BSP_IntHandlerDMA1_STREAM7           /* DMA1 Stream7                 */                          
  .word     BSP_IntHandlerFSMC                   /* FSMC                         */                   
  .word     BSP_IntHandlerSDIO                   /* SDIO                         */                   
  .word     BSP_IntHandlerTIM5                   /* TIM5                         */                   
  .word     BSP_IntHandlerSPI3                   /* SPI3                         */                   
  .word     BSP_IntHandlerUSART4                  /* UART4                        */                   
  .word     BSP_IntHandlerUSART5                  /* UART5                        */                   
  .word     BSP_IntHandlerTIM6_DAC               /* TIM6 and DAC1&2 underrun errors */                   
  .word     BSP_IntHandlerTIM7                   /* TIM7                         */
  .word     BSP_IntHandlerDMA2_CH0           /* DMA2 Stream 0                */                   
  .word     BSP_IntHandlerDMA2_CH1           /* DMA2 Stream 1                */                   
  .word     BSP_IntHandlerDMA2_CH2           /* DMA2 Stream 2                */                   
  .word     BSP_IntHandlerDMA2_CH3           /* DMA2 Stream 3                */                   
  .word     BSP_IntHandlerDMA2_CH4           /* DMA2 Stream 4                */                   
  .word     BSP_IntHandlerETH                    /* Ethernet                     */                   
  .word     BSP_IntHandlerETHWakeup               /* Ethernet Wakeup through EXTI line */                     
  .word     BSP_IntHandlerCAN2_TX                /* CAN2 TX                      */                          
  .word     BSP_IntHandlerCAN2_RX0               /* CAN2 RX0                     */                          
  .word     BSP_IntHandlerCAN2_RX1               /* CAN2 RX1                     */                          
  .word     BSP_IntHandlerCAN2_SCE               /* CAN2 SCE                     */                          
  .word     BSP_IntHandlerOTG_FS                 /* USB OTG FS                   */                   
  .word     BSP_IntHandlerDMA2_CH5           /* DMA2 Stream 5                */                   
  .word     BSP_IntHandlerDMA2_CH6           /* DMA2 Stream 6                */                   
  .word     BSP_IntHandlerDMA2_CH7           /* DMA2 Stream 7                */                   
  .word     BSP_IntHandlerUSART6                 /* USART6                       */                    
  .word     BSP_IntHandlerI2C3_EV                /* I2C3 event                   */                          
  .word     BSP_IntHandlerI2C3_ER                /* I2C3 error                   */                          
  .word     BSP_IntHandlerOTG_HS_EP1_OUT         /* USB OTG HS End Point 1 Out   */                   
  .word     BSP_IntHandlerOTG_HS_EP1_IN          /* USB OTG HS End Point 1 In    */                   
  .word     BSP_IntHandlerOTG_HS_WKUP            /* USB OTG HS Wakeup through EXTI */                         
  .word     BSP_IntHandlerOTG_HS                 /* USB OTG HS                   */                   
  .word     BSP_IntHandlerDCMI                /* DCMI                         */                   
  .word     BSP_IntHandlerCRYP                /* CRYP crypto                  */                   
  .word     BSP_IntHandlerHASH_RNG            /* Hash and Rng                 */
  .word     BSP_IntHandlerFPU                 /* FPU                          */
                         
                         
/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler. 
* As they are weak aliases, any function with the same name will override 
* this definition.
* 
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler
  
   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler
  
   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler
  
   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler              
  
   .weak      WWDG_IRQHandler                   
   .thumb_set WWDG_IRQHandler,Default_Handler      
                  
   .weak      PVD_IRQHandler      
   .thumb_set PVD_IRQHandler,Default_Handler
               
   .weak      TAMP_STAMP_IRQHandler            
   .thumb_set TAMP_STAMP_IRQHandler,Default_Handler
            
   .weak      RTC_WKUP_IRQHandler                  
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler
            
   .weak      FLASH_IRQHandler         
   .thumb_set FLASH_IRQHandler,Default_Handler
                  
   .weak      RCC_IRQHandler      
   .thumb_set RCC_IRQHandler,Default_Handler
                  
   .weak      EXTI0_IRQHandler         
   .thumb_set EXTI0_IRQHandler,Default_Handler
                  
   .weak      EXTI1_IRQHandler         
   .thumb_set EXTI1_IRQHandler,Default_Handler
                     
   .weak      EXTI2_IRQHandler         
   .thumb_set EXTI2_IRQHandler,Default_Handler 
                 
   .weak      EXTI3_IRQHandler         
   .thumb_set EXTI3_IRQHandler,Default_Handler
                        
   .weak      EXTI4_IRQHandler         
   .thumb_set EXTI4_IRQHandler,Default_Handler
                  
   .weak      DMA1_Stream0_IRQHandler               
   .thumb_set DMA1_Stream0_IRQHandler,Default_Handler
         
   .weak      DMA1_Stream1_IRQHandler               
   .thumb_set DMA1_Stream1_IRQHandler,Default_Handler
                  
   .weak      DMA1_Stream2_IRQHandler               
   .thumb_set DMA1_Stream2_IRQHandler,Default_Handler
                  
   .weak      DMA1_Stream3_IRQHandler               
   .thumb_set DMA1_Stream3_IRQHandler,Default_Handler 
                 
   .weak      DMA1_Stream4_IRQHandler              
   .thumb_set DMA1_Stream4_IRQHandler,Default_Handler
                  
   .weak      DMA1_Stream5_IRQHandler               
   .thumb_set DMA1_Stream5_IRQHandler,Default_Handler
                  
   .weak      DMA1_Stream6_IRQHandler               
   .thumb_set DMA1_Stream6_IRQHandler,Default_Handler
                  
   .weak      ADC_IRQHandler      
   .thumb_set ADC_IRQHandler,Default_Handler
               
   .weak      CAN1_TX_IRQHandler   
   .thumb_set CAN1_TX_IRQHandler,Default_Handler
            
   .weak      CAN1_RX0_IRQHandler                  
   .thumb_set CAN1_RX0_IRQHandler,Default_Handler
                           
   .weak      CAN1_RX1_IRQHandler                  
   .thumb_set CAN1_RX1_IRQHandler,Default_Handler
            
   .weak      CAN1_SCE_IRQHandler                  
   .thumb_set CAN1_SCE_IRQHandler,Default_Handler
            
   .weak      EXTI9_5_IRQHandler   
   .thumb_set EXTI9_5_IRQHandler,Default_Handler
            
   .weak      TIM1_BRK_TIM9_IRQHandler            
   .thumb_set TIM1_BRK_TIM9_IRQHandler,Default_Handler
            
   .weak      TIM1_UP_TIM10_IRQHandler            
   .thumb_set TIM1_UP_TIM10_IRQHandler,Default_Handler
      
   .weak      TIM1_TRG_COM_TIM11_IRQHandler      
   .thumb_set TIM1_TRG_COM_TIM11_IRQHandler,Default_Handler
      
   .weak      TIM1_CC_IRQHandler   
   .thumb_set TIM1_CC_IRQHandler,Default_Handler
                  
   .weak      TIM2_IRQHandler            
   .thumb_set TIM2_IRQHandler,Default_Handler
                  
   .weak      TIM3_IRQHandler            
   .thumb_set TIM3_IRQHandler,Default_Handler
                  
   .weak      TIM4_IRQHandler            
   .thumb_set TIM4_IRQHandler,Default_Handler
                  
   .weak      I2C1_EV_IRQHandler   
   .thumb_set I2C1_EV_IRQHandler,Default_Handler
                     
   .weak      I2C1_ER_IRQHandler   
   .thumb_set I2C1_ER_IRQHandler,Default_Handler
                     
   .weak      I2C2_EV_IRQHandler   
   .thumb_set I2C2_EV_IRQHandler,Default_Handler
                  
   .weak      I2C2_ER_IRQHandler   
   .thumb_set I2C2_ER_IRQHandler,Default_Handler
                           
   .weak      SPI1_IRQHandler            
   .thumb_set SPI1_IRQHandler,Default_Handler
                        
   .weak      SPI2_IRQHandler            
   .thumb_set SPI2_IRQHandler,Default_Handler
                  
   .weak      USART1_IRQHandler      
   .thumb_set USART1_IRQHandler,Default_Handler
                     
   .weak      USART2_IRQHandler      
   .thumb_set USART2_IRQHandler,Default_Handler
                     
   .weak      USART3_IRQHandler      
   .thumb_set USART3_IRQHandler,Default_Handler
                  
   .weak      EXTI15_10_IRQHandler               
   .thumb_set EXTI15_10_IRQHandler,Default_Handler
               
   .weak      RTC_Alarm_IRQHandler               
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler
            
   .weak      OTG_FS_WKUP_IRQHandler         
   .thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler
            
   .weak      TIM8_BRK_TIM12_IRQHandler         
   .thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler
         
   .weak      TIM8_UP_TIM13_IRQHandler            
   .thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler
         
   .weak      TIM8_TRG_COM_TIM14_IRQHandler      
   .thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler
      
   .weak      TIM8_CC_IRQHandler   
   .thumb_set TIM8_CC_IRQHandler,Default_Handler
                  
   .weak      DMA1_Stream7_IRQHandler               
   .thumb_set DMA1_Stream7_IRQHandler,Default_Handler
                     
   .weak      FSMC_IRQHandler            
   .thumb_set FSMC_IRQHandler,Default_Handler
                     
   .weak      SDIO_IRQHandler            
   .thumb_set SDIO_IRQHandler,Default_Handler
                     
   .weak      TIM5_IRQHandler            
   .thumb_set TIM5_IRQHandler,Default_Handler
                     
   .weak      SPI3_IRQHandler            
   .thumb_set SPI3_IRQHandler,Default_Handler
                     
   .weak      UART4_IRQHandler         
   .thumb_set UART4_IRQHandler,Default_Handler
                  
   .weak      UART5_IRQHandler         
   .thumb_set UART5_IRQHandler,Default_Handler
                  
   .weak      TIM6_DAC_IRQHandler                  
   .thumb_set TIM6_DAC_IRQHandler,Default_Handler
               
   .weak      TIM7_IRQHandler            
   .thumb_set TIM7_IRQHandler,Default_Handler
         
   .weak      DMA2_Stream0_IRQHandler               
   .thumb_set DMA2_Stream0_IRQHandler,Default_Handler
               
   .weak      DMA2_Stream1_IRQHandler               
   .thumb_set DMA2_Stream1_IRQHandler,Default_Handler
                  
   .weak      DMA2_Stream2_IRQHandler               
   .thumb_set DMA2_Stream2_IRQHandler,Default_Handler
            
   .weak      DMA2_Stream3_IRQHandler               
   .thumb_set DMA2_Stream3_IRQHandler,Default_Handler
            
   .weak      DMA2_Stream4_IRQHandler               
   .thumb_set DMA2_Stream4_IRQHandler,Default_Handler
            
   .weak      ETH_IRQHandler      
   .thumb_set ETH_IRQHandler,Default_Handler
                  
   .weak      ETH_WKUP_IRQHandler                  
   .thumb_set ETH_WKUP_IRQHandler,Default_Handler
            
   .weak      CAN2_TX_IRQHandler   
   .thumb_set CAN2_TX_IRQHandler,Default_Handler
                           
   .weak      CAN2_RX0_IRQHandler                  
   .thumb_set CAN2_RX0_IRQHandler,Default_Handler
                           
   .weak      CAN2_RX1_IRQHandler                  
   .thumb_set CAN2_RX1_IRQHandler,Default_Handler
                           
   .weak      CAN2_SCE_IRQHandler                  
   .thumb_set CAN2_SCE_IRQHandler,Default_Handler
                           
   .weak      OTG_FS_IRQHandler      
   .thumb_set OTG_FS_IRQHandler,Default_Handler
                     
   .weak      DMA2_Stream5_IRQHandler               
   .thumb_set DMA2_Stream5_IRQHandler,Default_Handler
                  
   .weak      DMA2_Stream6_IRQHandler               
   .thumb_set DMA2_Stream6_IRQHandler,Default_Handler
                  
   .weak      DMA2_Stream7_IRQHandler               
   .thumb_set DMA2_Stream7_IRQHandler,Default_Handler
                  
   .weak      USART6_IRQHandler      
   .thumb_set USART6_IRQHandler,Default_Handler
                        
   .weak      I2C3_EV_IRQHandler   
   .thumb_set I2C3_EV_IRQHandler,Default_Handler
                        
   .weak      I2C3_ER_IRQHandler   
   .thumb_set I2C3_ER_IRQHandler,Default_Handler
                        
   .weak      OTG_HS_EP1_OUT_IRQHandler         
   .thumb_set OTG_HS_EP1_OUT_IRQHandler,Default_Handler
               
   .weak      OTG_HS_EP1_IN_IRQHandler            
   .thumb_set OTG_HS_EP1_IN_IRQHandler,Default_Handler
               
   .weak      OTG_HS_WKUP_IRQHandler         
   .thumb_set OTG_HS_WKUP_IRQHandler,Default_Handler
            
   .weak      OTG_HS_IRQHandler      
   .thumb_set OTG_HS_IRQHandler,Default_Handler
                  
   .weak      DCMI_IRQHandler            
   .thumb_set DCMI_IRQHandler,Default_Handler
                                   
   .weak      HASH_RNG_IRQHandler                  
   .thumb_set HASH_RNG_IRQHandler,Default_Handler   

   .weak      FPU_IRQHandler                  
   .thumb_set FPU_IRQHandler,Default_Handler  

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
