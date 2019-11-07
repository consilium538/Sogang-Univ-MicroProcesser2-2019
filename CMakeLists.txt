CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(micrium_test)

SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR ARM)
SET(UTIL_SEARCH_CMD which)

SET(TOOLCHAIN_PERFIX arm-none-eabi-)

EXECUTE_PROCESS(
    COMMAND ${UTIL_SEARCH_CMD} ${TOOLCHAIN_PERFIX}gcc
    OUTPUT_VARIABLE BINUTILS_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)

set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL ${ARM_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}size CACHE INTERNAL "size tool")

set(CMAKE_SYSROOT ${ARM_TOOLCHAIN_DIR}/../arm-none-eabi)
set(CMAKE_FIND_ROOT_PATH ${BINUTILS_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set( CPU_FLAGS "-mthumb -mcpu=cortex-m4 -march=armv7-m -mfloat-abi=fpu" )
set( COMPILER_FLAGS "-ffreestanding -ffunction-sections -fdata-sections -fsigned-char -fmessage-length=0 -fshort-enums")


#############################
#   directory definitions   #
#############################

set( DRIVER_DIR ${CMAKE_SOURCE_DIR}/Examples/ST/STM32CubeF4/Drivers )
set( CMSIS_DIR ${DRIVER_DIR}/CMSIS )
set( CMSIS_DEV_DIR ${CMSIS_DIR}/Device/ST/STM32F4xx )
set( HAL_DIR ${DRIVER_DIR}/STM32F4xx_HAL_Driver )
set( USER_DIR ${CMAKE_SOURCE_DIR}/Examples/ST/STM3240G-EVAL )
set( BSP_DIR ${USER_DIR}/BSP )
set( USER_OS_DIR ${USER_DIR}/OS3 )

set( CPU_DIR ${CMAKE_SOURCE_DIR}/Software/uC-CPU )
set( LIB_DIR ${CMAKE_SOURCE_DIR}/Software/uC-LIB )
set( OS_DIR ${CMAKE_SOURCE_DIR}/Software/uCOS-III )
# set( SERIAL_DIR ${CMAKE_SOURCE_DIR}/Software/uC-Serial ) # do not need this

###############################################################################

##########################
#   include diretories   #
##########################

set(INC_FOLDERS
    ${CMSIS_DIR}/Include
    ${CMSIS_DEV_DIR}/Include
    ${HAL_DIR}/Inc
    ${BSP_DIR}
    ${BSP_DIR}/OS/uCOS-III
    ${USER_OS_DIR}
    ${CPU_DIR}
    ${CPU_DIR}/ARM-Cortex-M4/GNU
    ${LIB_DIR}
    ${LIB_DIR}/Ports/ARM-Cortex-M4/GNU
    ${OS_DIR}/Source
    ${OS_DIR}/Ports/ARM-Cortex-M4/Generic/GNU
)

####################
#   source files   #
####################

set(HAL_SRCS
    ${HAL_DIR}/Src/stm32f4xx_hal_adc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_adc_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal.c
    ${HAL_DIR}/Src/stm32f4xx_hal_can.c
    ${HAL_DIR}/Src/stm32f4xx_hal_cec.c
    ${HAL_DIR}/Src/stm32f4xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_crc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_cryp.c
    ${HAL_DIR}/Src/stm32f4xx_hal_cryp_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dac.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dac_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dcmi.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dcmi_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dma2d.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dma.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dma_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_eth.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash_ramfunc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_fmpi2c.c
    ${HAL_DIR}/Src/stm32f4xx_hal_fmpi2c_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32f4xx_hal_hash.c
    ${HAL_DIR}/Src/stm32f4xx_hal_hash_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_hcd.c
    ${HAL_DIR}/Src/stm32f4xx_hal_i2c.c
    ${HAL_DIR}/Src/stm32f4xx_hal_i2c_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_i2s.c
    ${HAL_DIR}/Src/stm32f4xx_hal_i2s_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_irda.c
    ${HAL_DIR}/Src/stm32f4xx_hal_iwdg.c
    ${HAL_DIR}/Src/stm32f4xx_hal_ltdc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_msp_template.c
    ${HAL_DIR}/Src/stm32f4xx_hal_nand.c
    ${HAL_DIR}/Src/stm32f4xx_hal_nor.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pccard.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pcd.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pcd_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_qspi.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rng.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rtc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rtc_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_sai.c
    ${HAL_DIR}/Src/stm32f4xx_hal_sai_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_sd.c
    ${HAL_DIR}/Src/stm32f4xx_hal_sdram.c
    ${HAL_DIR}/Src/stm32f4xx_hal_smartcard.c
    ${HAL_DIR}/Src/stm32f4xx_hal_spdifrx.c
    ${HAL_DIR}/Src/stm32f4xx_hal_spi.c
    ${HAL_DIR}/Src/stm32f4xx_hal_sram.c
    ${HAL_DIR}/Src/stm32f4xx_hal_tim.c
    ${HAL_DIR}/Src/stm32f4xx_hal_tim_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_uart.c
    ${HAL_DIR}/Src/stm32f4xx_hal_usart.c
    ${HAL_DIR}/Src/stm32f4xx_hal_wwdg.c
    ${HAL_DIR}/Src/stm32f4xx_ll_fmc.c
    ${HAL_DIR}/Src/stm32f4xx_ll_fsmc.c
    ${HAL_DIR}/Src/stm32f4xx_ll_sdmmc.c
    ${HAL_DIR}/Src/stm32f4xx_ll_usb.c
)

set(BSP_SRCS
    ${BSP_DIR}/bsp.c
    ${BSP_DIR}/bsp_int.c
    ${BSP_DIR}/bsp_p.c
    ${BSP_DIR}/bsp_periph.c
    ${BSP_DIR}/cpu_bsp.c
    ${BSP_DIR}/serial_bsp_stm3240x.c
    ${BSP_DIR}/CMSIS/system_stm32f4xx.c
    ${BSP_DIR}/OS/uCOS-III/bsp_os.c
)

set(${USER_OS_SRCS}
    ${USER_OS_DIR}/app.c
    ${USER_OS_DIR}/app_org.c
    ${USER_OS_DIR}/app_serial.c
    ${USER_OS_DIR}/app_hooks.c
)

set(${CPU_SRCS}
    ${CPU_DIR}/cpu_core.c
    ${CPU_DIR}/ARM-Cortex-M4/GNU/cpu_c.c
    ${CPU_DIR}/ARM-Cortex-M4/GNU/cpu_s.s
)

set(${LIB_SRCS}
    ${LIB_DIR}/lib_ascii.c
    ${LIB_DIR}/lib_math.c
    ${LIB_DIR}/lib_mem.c
    ${LIB_DIR}/lib_str.c
    ${LIB_DIR}/Ports/ARM-Cortex-M4/GNU/lib_mem_a.s
)

set(${OS_SRCS}
    ${OS_DIR}/Source/os_cfg_app.c
    ${OS_DIR}/Source/os_core.c
    ${OS_DIR}/Source/os_dbg.c
    ${OS_DIR}/Source/os_flag.c
    ${OS_DIR}/Source/os_int.c
    ${OS_DIR}/Source/os_mem.c
    ${OS_DIR}/Source/os_msg.c
    ${OS_DIR}/Source/os_mutex.c
    ${OS_DIR}/Source/os_pend_multi.c
    ${OS_DIR}/Source/os_prio.c
    ${OS_DIR}/Source/os_q.c
    ${OS_DIR}/Source/os_sem.c
    ${OS_DIR}/Source/os_stat.c
    ${OS_DIR}/Source/os_task.c
    ${OS_DIR}/Source/os_tick.c
    ${OS_DIR}/Source/os_time.c
    ${OS_DIR}/Source/os_tmr.c
    ${OS_DIR}/Source/os_var.c
    ${OS_DIR}/Ports/ARM-Cortex-M4/Generic/GNU/os_cpu_c.c
    ${OS_DIR}/Ports/ARM-Cortex-M4/Generic/GNU/os_cpu_a.S
)

set(SRC_FILES
    ${HAL_SRCS}
    ${BSP_SRCS}
    ${USER_OS_SRCS}
    ${CPU_SRCS}
    ${LIB_SRCS}
    ${OS_SRCS}
)

###############################################################################

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_C_FLAGS ${CPU_FLAGS} ${COMPILER_FLAGS} )
INCLUDE_DIRECTORIES( ${INC_FOLDERS} )
ADD_EXECUTABLE( ${PROJECT_NAME}.elf ${SRC_FILES} )



