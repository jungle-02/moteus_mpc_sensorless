// File config tong hop cho mbed tren STM32G474RE -> dinh nghia macro

#ifndef MBED_CONFIG_H
#define MBED_CONFIG_H

// ===== ARM / TOOLCHAIN / CORE CONFIG =====
#define ARM_MATH_CM4 1     // enable CMSIS DSP cho Cortex-M4
#define BOOST_ALL_NO_LIB 1 // tat auto link cua Boost (tranh loi link)
#define CLOCK_SOURCE                                                           \
  USE_PLL_HSE_EXTC | USE_PLL_HSI // cau hinh clock: dung PLL tu HSE ngoai + HSI

// ===== DEVICE FEATURE ENABLE =====
// bat cac peripheral co san tren MCU
#define DEVICE_ANALOGIN 1
#define DEVICE_ANALOGOUT 1
#define DEVICE_FLASH 1
#define DEVICE_I2C 1
#define DEVICE_I2CSLAVE 1
#define DEVICE_I2C_ASYNCH 1 // I2C async mode
#define DEVICE_INTERRUPTIN 1
#define DEVICE_LPTICKER 1 // low power timer
#define DEVICE_MPU 1      // memory protection unit
#define DEVICE_PORTIN 1
#define DEVICE_PORTINOUT 1
#define DEVICE_PORTOUT 1
#define DEVICE_PWMOUT 1
#define DEVICE_RESET_REASON 1
#define DEVICE_RTC 1
#define DEVICE_SERIAL 1
#define DEVICE_SERIAL_FC 1 // serial flow control
#define DEVICE_SLEEP 1
#define DEVICE_SPI 1
#define DEVICE_SPISLAVE 1
#define DEVICE_SPI_ASYNCH 1
#define DEVICE_STDIO_MESSAGES 1
#define DEVICE_USTICKER 1 // microsecond ticker
#define DEVICE_WATCHDOG 1
#define LPTICKER_DELAY_TICKS 1 // do tre ticker

// ===== DRIVER / EVENT / PLATFORM CONFIG =====
#define MBED_CONF_DRIVERS_UART_SERIAL_RXBUF_SIZE 256 // buffer RX UART
#define MBED_CONF_DRIVERS_UART_SERIAL_TXBUF_SIZE 256 // buffer TX UART

// event queue config
#define MBED_CONF_EVENTS_SHARED_DISPATCH_FROM_APPLICATION 1
#define MBED_CONF_EVENTS_SHARED_EVENTSIZE 256
#define MBED_CONF_EVENTS_SHARED_HIGHPRIO_EVENTSIZE 256
#define MBED_CONF_EVENTS_SHARED_HIGHPRIO_STACKSIZE 1024
#define MBED_CONF_EVENTS_SHARED_STACKSIZE 1024

// platform config
#define MBED_CONF_PLATFORM_CTHUNK_COUNT_MAX 4
#define MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE 9600
#define MBED_CONF_PLATFORM_ERROR_DECODE_HTTP_URL_STR "%d"
#define MBED_CONF_PLATFORM_ERROR_HIST_SIZE 4
#define MBED_CONF_PLATFORM_MAX_ERROR_FILENAME_LEN 16
#define MBED_CONF_PLATFORM_STDIO_BAUD_RATE 9600
#define MBED_CONF_PLATFORM_STDIO_FLUSH_AT_EXIT 1

// ===== RTOS CONFIG =====
#define MBED_CONF_RTOS_IDLE_THREAD_STACK_SIZE 512
#define MBED_CONF_RTOS_MAIN_THREAD_STACK_SIZE 4096
#define MBED_CONF_RTOS_THREAD_STACK_SIZE 4096
#define MBED_CONF_RTOS_TIMER_THREAD_STACK_SIZE 768

// clock cho LPUART
#define MBED_CONF_TARGET_LPUART_CLOCK_SOURCE                                   \
  USE_LPUART_CLK_LSE | USE_LPUART_CLK_PCLK1

// ===== MEMORY / TARGET CONFIG =====
#define MEM_ALLOC malloc // wrapper malloc
#define MEM_FREE free    // wrapper free

// target architecture
#define TARGET_CORTEX 1
#define TARGET_CORTEX_M 1
#define TARGET_FAMILY_STM32 1
#define TARGET_FF_ARDUINO 1
#define TARGET_FF_MORPHO 1
#define TARGET_LIKE_CORTEX_M4 1
#define TARGET_LIKE_MBED 1
#define TARGET_M4 1

// specific board / MCU
#define TARGET_NUCLEO_G474RE 1
#define TARGET_RELEASE 1
#define TARGET_RTOS_M4_M7 1
#define TARGET_STM 1
#define TARGET_STM32G4 1
#define TARGET_STM32G474RE 1
#define TARGET_STM32G474xE 1
#define TARGET_Target 1

// toolchain
#define TOOLCHAIN_GCC 1
#define TOOLCHAIN_GCC_ARM 1

// SPI config
#define TRANSACTION_QUEUE_SIZE_SPI 2

// USB config
#define USBHOST_OTHER 1
#define USB_STM_HAL 1

// HAL / LL driver
#define USE_FULL_LL_DRIVER 1
#define USE_HAL_DRIVER 1
#define _RTE_ 1

// ===== CMSIS / CORE DEFINE =====
#define __CMSIS_RTOS 1
#define __CORTEX_M4 1
#define __FPU_PRESENT 1 // co FPU hardware
#define __MBED_CMSIS_RTOS_CM 1
#define __MBED__ 1

// ===== STANDARD MACRO FIX =====
// dam bao cac macro standard duoc define
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif
#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

// ===== FLASH LAYOUT =====
#define MBED_APP_START 0x8010000 // dia chi bat dau app trong flash
#define MBED_APP_SIZE 0x0070000  // kich thuoc app
#define SPI_FILL_CHAR 0xFF       // gia tri fill SPI

// ===== TIMER CONFIG =====
#define MBED_US_TIMER_TIM TIM15 // timer dung cho us ticker
#define MBED_US_TIMER_TIM_USCORE TIM15_
#define MBED_US_TIMER_USCORE_TIM _TIM15
#define TIM_MST_IRQ TIM1_BRK_TIM15_IRQn // interrupt cua timer
#define TIM_MST_BIT_WIDTH 16            // do rong counter

// tranh conflict giua newlib va system header
#define _UNISTD_H_
#define _SYS_UNISTD_H
#define __int64_t_defined 1

// ===== FUNCTION DECLARATION =====
#ifndef __ASSEMBLER__
#ifdef __cplusplus
extern "C" {
#endif
void wait_ns(unsigned int ns); // delay theo nanosecond
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLER__ */

#endif // MBED_CONFIG_H