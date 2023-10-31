#ifndef __RSERIAL_DEF_H__
#define __RSERIAL_DEF_H__

/** Include for rserial */
#if defined(STM32G474xx)

#elif defined(STM32F103xB) || defined(STM32F103xE)

#    include "usart.h"
#    include "stm32f1xx_hal.h"
#    include "stm32f1xx_hal_uart.h"

#elif defined(STM32F072xB) || defined(STM32F091xC)

#    include "stm32f0xx_hal.h"

#elif defined(STM32F407xx) || defined(STM32F429xx)

#    include "stm32f4xx_hal.h"
#    define HTIM htim2

#elif defined(STM32F765xx)

#    include "stm32f7xx_hal.h"

#endif

/** rserial struct definition for rserial */

#if defined(STM32G474xx) || defined(STM32F103xB) || defined(STM32F072xB) || defined(STM32F091xC) || \
    defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F765xx) || defined(STM32F103xE)

struct serial_port
{
    UART_HandleTypeDef uart;
    int                byte_timeout_us;
};

#endif

#if defined(RSERIAL_FOR_APPLE) || defined(RSERIAL_FOR_UNIX) || defined(RSERIAL_FOR_WINDOWS)

#    include "termios.h"
#    include "sys/time.h"
#    include <fcntl.h>
#    include <termios.h>
#    include <unistd.h>
#    include "string.h"
#    include <sys/ioctl.h>
#    include "errno.h"
#    include <sys/file.h>
#    include <stdbool.h>
#    include <sys/signal.h>

struct serial_port
{
    int            fd;
    bool           opened;
    bool           enable_interrupt;
    struct timeval byte_timeout;
    struct termios old_settings;
};

#endif

#endif  // __RSERIAL_DEF_H__