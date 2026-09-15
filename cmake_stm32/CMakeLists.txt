# CMake build system cho firmware moteus tren STM32G474 -> cau hinh compiler, include path, source list

cmake_minimum_required(VERSION 3.16)
project(moteus_firmware C CXX ASM)

# ===== NGON NGU & STANDARD =====
set(CMAKE_CXX_STANDARD 20)   # dung C++20
set(CMAKE_C_STANDARD 11)     # dung C11

# ===== PATH CONFIG =====
set(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")         # root project
set(MBED_DIR "${ROOT}/third_party/mbed-os")        # duong dan mbed-os

set(FW_DIR_NAME "fw_mpc")
set(FW_PATH "${ROOT}/${FW_DIR_NAME}")              # duong dan firmware

message(STATUS "Using firmware directory: ${FW_PATH}") # in ra log build

# ===== GLOBAL DEFINE =====
# define macro cho compiler
add_definitions(-D_GNU_SOURCE -DTARGET_STM32G4 -D__FPU_PRESENT=1 -D__CORTEX_M4=1 -DARM_MATH_CM4=1)

# ===== FORCE INCLUDE CONFIG HEADER =====
# ep moi file include mbed_config.h
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include \"${CMAKE_CURRENT_SOURCE_DIR}/mbed_config.h\"")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include \"${CMAKE_CURRENT_SOURCE_DIR}/mbed_config.h\"")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -include \"${CMAKE_CURRENT_SOURCE_DIR}/mbed_config.h\"")

# ===== INCLUDE DIRECTORIES =====
include_directories(SYSTEM
    ${ROOT}/third_party/boost              # boost
    ${MBED_DIR}                            # mbed root
    ${MBED_DIR}/cmsis                      # CMSIS
    ${MBED_DIR}/cmsis/TARGET_CORTEX_M
    ${MBED_DIR}/drivers                    # driver layer
    ${MBED_DIR}/hal                        # hardware abstraction layer
    ${MBED_DIR}/platform                   # platform layer
    ${MBED_DIR}/targets                    # target config
    ${MBED_DIR}/targets/TARGET_STM
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/device
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device
    ${ROOT}/third_party/mjlib              # mjlib
    ${FW_PATH}                             # firmware src
    ${ROOT}
    ${ROOT}/third_party
    ${ROOT}/third_party/eigen              # eigen math lib
)

# ===== MJLIB SOURCES =====
# cac module core cua mjlib (protocol, telemetry, config...)
set(MJLIB_SOURCES 
    ${ROOT}/third_party/mjlib/mjlib/base/assert.cc
    ${ROOT}/third_party/mjlib/mjlib/base/crc.cc
    ${ROOT}/third_party/mjlib/mjlib/base/error_code.cc
    ${ROOT}/third_party/mjlib/mjlib/base/time_conversions.cc
    ${ROOT}/third_party/mjlib/mjlib/micro/callback_table.cc
    ${ROOT}/third_party/mjlib/mjlib/micro/command_manager.cc
    ${ROOT}/third_party/mjlib/mjlib/micro/error.cc
    ${ROOT}/third_party/mjlib/mjlib/micro/error_code.cc
    ${ROOT}/third_party/mjlib/mjlib/micro/persistent_config.cc
    ${ROOT}/third_party/mjlib/mjlib/micro/pool_ptr.cc
    ${ROOT}/third_party/mjlib/mjlib/micro/telemetry_manager.cc
    ${ROOT}/third_party/mjlib/mjlib/multiplex/micro_error.cc
    ${ROOT}/third_party/mjlib/mjlib/multiplex/micro_server.cc
    ${ROOT}/third_party/mjlib/mjlib/multiplex/micro_stream_datagram.cc
)

# ===== MBED OS SOURCES =====
set(MBED_OS_SOURCES
    ${MBED_DIR}/cmsis/TARGET_CORTEX_M/mbed_tz_context.c

    # driver layer
    ${MBED_DIR}/drivers/AnalogIn.cpp
    ${MBED_DIR}/drivers/BusIn.cpp
    ${MBED_DIR}/drivers/BusInOut.cpp
    ${MBED_DIR}/drivers/BusOut.cpp
    ${MBED_DIR}/drivers/CAN.cpp
    ${MBED_DIR}/drivers/FlashIAP.cpp
    ${MBED_DIR}/drivers/I2C.cpp
    ${MBED_DIR}/drivers/I2CSlave.cpp
    ${MBED_DIR}/drivers/InterruptIn.cpp
    ${MBED_DIR}/drivers/InterruptManager.cpp
    ${MBED_DIR}/drivers/MbedCRC.cpp
    ${MBED_DIR}/drivers/RawSerial.cpp
    ${MBED_DIR}/drivers/ResetReason.cpp
    ${MBED_DIR}/drivers/SPI.cpp
    ${MBED_DIR}/drivers/SPISlave.cpp
    ${MBED_DIR}/drivers/Serial.cpp
    ${MBED_DIR}/drivers/SerialBase.cpp
    ${MBED_DIR}/drivers/TableCRC.cpp
    ${MBED_DIR}/drivers/Ticker.cpp
    ${MBED_DIR}/drivers/Timeout.cpp
    ${MBED_DIR}/drivers/Timer.cpp
    ${MBED_DIR}/drivers/TimerEvent.cpp
    ${MBED_DIR}/drivers/UARTSerial.cpp
    ${MBED_DIR}/drivers/Watchdog.cpp

    # HAL layer
    ${MBED_DIR}/hal/LowPowerTickerWrapper.cpp
    ${MBED_DIR}/hal/mbed_critical_section_api.c
    ${MBED_DIR}/hal/mbed_flash_api.c
    ${MBED_DIR}/hal/mbed_gpio.c
    ${MBED_DIR}/hal/mbed_itm_api.c
    ${MBED_DIR}/hal/mbed_lp_ticker_api.c
    ${MBED_DIR}/hal/mbed_lp_ticker_wrapper.cpp
    ${MBED_DIR}/hal/mbed_pinmap_common.c
    ${MBED_DIR}/hal/mbed_pinmap_default.c
    ${MBED_DIR}/hal/mbed_ticker_api.c
    ${MBED_DIR}/hal/mbed_us_ticker_api.c

    # platform layer (error, memory, boot...)
    ${MBED_DIR}/platform/CThunkBase.cpp
    ${MBED_DIR}/platform/CallChain.cpp
    ${MBED_DIR}/platform/TARGET_CORTEX_M/mbed_fault_handler.c
    ${MBED_DIR}/platform/internal/mbed_atomic_impl.c
    ${MBED_DIR}/platform/mbed_alloc_wrappers.cpp
    ${MBED_DIR}/platform/mbed_application.c
    ${MBED_DIR}/platform/mbed_assert.c
    ${MBED_DIR}/platform/mbed_board.c
    ${MBED_DIR}/platform/mbed_critical.c
    ${MBED_DIR}/platform/mbed_error.c
    ${MBED_DIR}/platform/mbed_error_hist.c
    ${MBED_DIR}/platform/mbed_interface.c
    ${MBED_DIR}/platform/mbed_mem_trace.cpp
    ${MBED_DIR}/platform/mbed_mktime.c
    ${MBED_DIR}/platform/mbed_mpu_mgmt.c
    ${MBED_DIR}/platform/mbed_poll.cpp
    ${MBED_DIR}/platform/mbed_retarget.cpp
    ${MBED_DIR}/platform/mbed_rtc_time.cpp
    ${MBED_DIR}/platform/mbed_sdk_boot.c
    ${MBED_DIR}/platform/mbed_semihost_api.c
    ${MBED_DIR}/platform/mbed_sleep_manager.c
    ${MBED_DIR}/platform/mbed_stats.c
    ${MBED_DIR}/platform/mbed_wait_api_no_rtos.c
    ${MBED_DIR}/platform/mbed_wait_api_rtos.cpp

    # target specific (STM32G4)
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE/PeripheralPins.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE/system_clock.c

    # HAL driver STM32
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/analogin_device.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/analogout_device.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_adc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_adc_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_comp.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_cordic.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_cortex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_crc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_crc_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_cryp.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_cryp_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_dac.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_dac_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_dma.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_dma_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_exti.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_fdcan.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_flash.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_flash_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_flash_ramfunc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_fmac.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_gpio.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_hrtim.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_i2c.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_i2c_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_i2s.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_irda.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_iwdg.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_lptim.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_msp_template.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_nand.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_nor.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_opamp.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_opamp_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_pcd.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_pcd_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_pwr.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_pwr_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_qspi.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_rcc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_rcc_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_rng.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_rtc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_rtc_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_sai.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_sai_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_smartcard.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_smartcard_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_smbus.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_spi.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_spi_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_sram.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_tim.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_tim_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_uart.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_uart_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_usart.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_usart_ex.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_hal_wwdg.c

    # low-level LL driver
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_adc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_comp.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_cordic.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_crc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_crs.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_dac.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_dma.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_exti.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_fmac.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_fmc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_gpio.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_hrtim.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_i2c.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_lptim.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_lpuart.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_opamp.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_pwr.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_rcc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_rng.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_rtc.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_spi.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_tim.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_ucpd.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_usart.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_usb.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/stm32g4xx_ll_utils.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/device/system_stm32g4xx.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/flash_api.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/gpio_irq_device.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/pwmout_device.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/serial_device.c
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/spi_api.c
    ${MBED_DIR}/targets/TARGET_STM/analogin_api.c
    ${MBED_DIR}/targets/TARGET_STM/analogout_api.c
    ${MBED_DIR}/targets/TARGET_STM/can_api.c
    ${MBED_DIR}/targets/TARGET_STM/gpio_api.c
    ${MBED_DIR}/targets/TARGET_STM/gpio_irq_api.c
    ${MBED_DIR}/targets/TARGET_STM/hal_tick_overrides.c
    ${MBED_DIR}/targets/TARGET_STM/i2c_api.c
    ${MBED_DIR}/targets/TARGET_STM/lp_ticker.c
    ${MBED_DIR}/targets/TARGET_STM/mbed_crc_api.c
    ${MBED_DIR}/targets/TARGET_STM/mbed_overrides.c
    ${MBED_DIR}/targets/TARGET_STM/pinmap.c
    ${MBED_DIR}/targets/TARGET_STM/port_api.c
    ${MBED_DIR}/targets/TARGET_STM/pwmout_api.c
    ${MBED_DIR}/targets/TARGET_STM/qspi_api.c
    ${MBED_DIR}/targets/TARGET_STM/reset_reason.c
    ${MBED_DIR}/targets/TARGET_STM/rtc_api.c
    ${MBED_DIR}/targets/TARGET_STM/serial_api.c
    ${MBED_DIR}/targets/TARGET_STM/sleep.c
    ${MBED_DIR}/targets/TARGET_STM/stm_spi_api.c
    ${MBED_DIR}/targets/TARGET_STM/trng_api.c
    ${MBED_DIR}/targets/TARGET_STM/us_ticker.c
    ${MBED_DIR}/targets/TARGET_STM/watchdog_api.c

    # startup file (entry point reset handler)
    ${MBED_DIR}/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/device/TOOLCHAIN_GCC_ARM/startup_stm32g474xx.S
)

set(MOTEUS_FW_SOURCES
    ${FW_PATH}/bldc_servo.cc            # control BLDC
    ${FW_PATH}/board_debug.cc
    ${FW_PATH}/drv8323.cc               # driver chip
    ${FW_PATH}/error.cc
    ${FW_PATH}/fdcan.cc                 # CAN bus
    ${FW_PATH}/firmware_info.cc
    ${FW_PATH}/foc.cc                   # FOC
    ${FW_PATH}/git_info.cc
    ${FW_PATH}/git_info_linkstamp.cc
    ${FW_PATH}/moteus.cc               
    ${FW_PATH}/moteus_assert.cc
    ${FW_PATH}/moteus_controller.cc
    ${FW_PATH}/moteus_hardfault_handler.cc
    ${FW_PATH}/moteus_hw.cc             # hardware init
    ${FW_PATH}/stm32g4_adc.cc
    ${FW_PATH}/stm32g4_async_uart.cc
    ${FW_PATH}/stm32_gpio_interrupt_in.cc
    ${FW_PATH}/stm32_serial.cc
    ${FW_PATH}/system_info.cc
    ${FW_PATH}/uuid.cc
    ${FW_PATH}/hardfault.s              # asm handler
)

# ===== STATIC LIB =====
add_library(mbed_os STATIC ${MBED_OS_SOURCES}) # build mbed os thanh static lib
add_library(mjlib STATIC ${MJLIB_SOURCES})     # build mjlib

# ===== EXECUTABLE & STUBS =====
set(STUB_FILE "${CMAKE_BINARY_DIR}/gcc13_syscall_stubs.c")
file(WRITE ${STUB_FILE} "
#include <stddef.h>
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
int _getentropy(void *buffer, size_t length) { return -1; }
")

add_executable(moteus.elf ${MOTEUS_FW_SOURCES} ${STUB_FILE})

# define rieng cho target
target_compile_definitions(moteus.elf PRIVATE STM32G474xx USE_HAL_DRIVER __MBED__)

# tat canh bao bien chua su dung
target_compile_options(moteus.elf PRIVATE -Wno-unused-variable)

# link library
target_link_libraries(moteus.elf PRIVATE mjlib mbed_os)

# ===== LINKER OPTIONS =====
target_link_options(moteus.elf PRIVATE 
    -L${CMAKE_CURRENT_SOURCE_DIR}             # search path
    -T${FW_PATH}/stm32g474.ld                 # linker script
    -Wl,--gc-sections                         # remove unused code
    -Wl,--wrap,main                           # wrap main
    -Wl,--wrap,_malloc_r
    -Wl,--wrap,_free_r
    -Wl,--wrap,_realloc_r
    -Wl,--wrap,_calloc_r
    -Wl,--wrap,atexit
    -Wl,-u,__wrap__malloc_r                   # force include symbol
    -Wl,-u,__wrap__free_r
    -Wl,-u,__wrap__realloc_r
    -Wl,-u,__wrap__calloc_r
    -Wl,-u,__wrap_main
    -Wl,-u,__wrap_atexit
)

# CONVERT .ELF -> .BIN
add_custom_command(TARGET moteus.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary moteus.elf moteus.bin
    COMMENT "Generating unified moteus.bin"
)