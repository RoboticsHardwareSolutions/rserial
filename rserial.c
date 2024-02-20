#include "rserial.h"
#include "rserial_def.h"
#include "string.h"

#if defined(STM32F072xB) || defined(STM32F091xC) || defined(STM32F103xB) || defined(STM32F407xx) || \
    defined(STM32F429xx) || defined(STM32F103xE) || defined(STM32F765xx) || defined(STM32G474xx)

USART_TypeDef* convert_uart_name(char* port_name)
{
    if (strcmp(port_name, "UART1") == 0)
    {
        return USART1;
    }
    else if (strcmp(port_name, "UART2") == 0)
    {
        return USART2;
    }
    else if (strcmp(port_name, "UART3") == 0)
    {
        return USART3;
    }
    else
    {
        return NULL;
    }
}

int check_baud(int baud)
{
    if (baud < 50 || baud > 230400)
    {
        return -1;
    }
    return 0;
}

int data_bit_convert(const char* mode, uint32_t* word_len)
{
    switch (mode[0])
    {
    case '8':
        *word_len = UART_WORDLENGTH_8B;
        break;
    case '9':
        *word_len = UART_WORDLENGTH_9B;
        break;
    default:
        return -1;
        break;
    }
    return 0;
}

int parity_convert(const char* mode, uint32_t* parity)
{
    switch (mode[1])
    {
    case 'N':
    case 'n':
        *parity = UART_PARITY_NONE;
        break;
    case 'E':
    case 'e':
        *parity = UART_PARITY_EVEN;
        break;
    case 'O':
    case 'o':
        *parity = UART_PARITY_ODD;
        break;
    default:
        return -1;
        break;
    }
    return 0;
}

int stop_bit_convert(const char* mode, uint32_t* stop)
{
    switch (mode[2])
    {
    case '1':
        *stop = UART_STOPBITS_1;
        break;
    case '2':
        *stop = UART_STOPBITS_2;
        break;
    default:
        return -1;
        break;
    }
    return 0;
}

int rserial_open(rserial* instance, char* port_name, int baud, char* mode, int flow_ctrl, int byte_timeout_us)
{
    uint32_t tmp;
    memset(&instance->uart, 0, sizeof(instance->uart));
    instance->byte_timeout_us        = byte_timeout_us;
    instance->uart.Init.Mode         = UART_MODE_TX_RX;
    instance->uart.Init.OverSampling = UART_OVERSAMPLING_16;
    instance->uart.Instance          = convert_uart_name(port_name);
    if (instance->uart.Instance == NULL)
    {
        return -1;
    }
    if (flow_ctrl == 1)
    {
        instance->uart.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    }
    else
    {
        instance->uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    }
    if (check_baud(baud) == 0)
    {
        instance->uart.Init.BaudRate = baud;
    }
    else
    {
        return -1;
    }
    if (data_bit_convert(mode, &tmp))
    {
        return -1;
    }
    else
    {
        instance->uart.Init.WordLength = tmp;
    }
    if (stop_bit_convert(mode, &tmp))
    {
        return -1;
    }
    else
    {
        instance->uart.Init.StopBits = tmp;
    }
    if (parity_convert(mode, &tmp))
    {
        return -1;
    }
    else
    {
        instance->uart.Init.Parity = tmp;
    }
    if (HAL_UART_Init(&instance->uart) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

int rserial_read(rserial* instance, uint8_t* data, size_t size, unsigned int timeout_us)
{
    if (HAL_UART_Receive(&instance->uart, data, size, timeout_us / 1000) != HAL_OK)
    {
        return -1;
    }
    return size;
}

int rserial_readline(rserial* instance, char* data, char eol, int timeout_us) {}

int rserial_write(rserial* instance, uint8_t* data, size_t size)
{
    if (HAL_UART_Transmit(&instance->uart, data, size, 5000) != HAL_OK)
    {
        return -1;
    }
    return size;
}

bool rserial_is_ok(rserial* instance)
{
    uint32_t err = HAL_UART_GetError(&instance->uart);

    if (err & HAL_UART_ERROR_PE)
    {
        return false;
    }
    if (err & HAL_UART_ERROR_NE)
    {
        return false;
    }
    if (err & HAL_UART_ERROR_FE)
    {
        return false;
    }
    if (err & HAL_UART_ERROR_ORE)
    {
        return false;
    }
    if (err & HAL_UART_ERROR_DMA)
    {
        return false;
    }
    return true;
}

int rserial_close(rserial* instance)
{
    if (HAL_UART_DeInit(&instance->uart) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

#endif

#if defined(RSERIAL_FOR_WINDOWS) || defined(RSERIAL_FOR_UNIX) || defined(RSERIAL_FOR_APPLE)

int serial_select(int fd, fd_set* rset, struct timeval* tv)
{
    int s_rc;
    while ((s_rc = select(fd + 1, rset, NULL, NULL, tv)) == -1)
    {
        if (errno == EINTR)
        {
            FD_ZERO(rset);
            FD_SET(fd, rset);
        }
        else
        {
            return -1;
        }
    }

    if (s_rc == 0)
    {
        return -1;
    }
    return s_rc;
}

int baud_convert(int baudrate)
{
    int baudr = -1;
    switch (baudrate)
    {
    case 50:
        baudr = B50;
        break;
    case 75:
        baudr = B75;
        break;
    case 110:
        baudr = B110;
        break;
    case 134:
        baudr = B134;
        break;
    case 150:
        baudr = B150;
        break;
    case 200:
        baudr = B200;
        break;
    case 300:
        baudr = B300;
        break;
    case 600:
        baudr = B600;
        break;
    case 1200:
        baudr = B1200;
        break;
    case 1800:
        baudr = B1800;
        break;
    case 2400:
        baudr = B2400;
        break;
    case 4800:
        baudr = B4800;
        break;
    case 9600:
        baudr = B9600;
        break;
    case 19200:
        baudr = B19200;
        break;
    case 38400:
        baudr = B38400;
        break;
    case 57600:
        baudr = B57600;
        break;
    case 115200:
        baudr = B115200;
        break;
    case 230400:
        baudr = B230400;
        break;
    default:
        break;
    }
    return baudr;
}

int data_bit_convert(const char* mode)
{
    int bits = -1;
    switch (mode[0])
    {
    case '8':
        bits = CS8;
        break;
    case '7':
        bits = CS7;
        break;
    case '6':
        bits = CS6;
        break;
    case '5':
        bits = CS5;
        break;
    default:
        break;
    }
    return bits;
}

int parity_convert(const char* mode, int* cpar, int* ipar)
{
    switch (mode[1])
    {
    case 'N':
    case 'n':
        *cpar = 0;
        *ipar = IGNPAR;
        break;
    case 'E':
    case 'e':
        *cpar = PARENB;
        *ipar = INPCK;
        break;
    case 'O':
    case 'o':
        *cpar = (PARENB | PARODD);
        *ipar = INPCK;
        break;
    default:
        return -1;
        break;
    }
    return 0;
}

int stop_bit_convert(const char* mode)
{
    int bstop = -1;

    switch (mode[2])
    {
    case '1':
        bstop = 0;
        break;
    case '2':
        bstop = CSTOPB;
        break;
    default:
        break;
    }
    return bstop;
}

int set_rts(int fd, int on)
{
    int flags;

    ioctl(fd, TIOCMGET, &flags);
    if (on)
    {
        flags |= TIOCM_RTS;
    }
    else
    {
        flags &= ~TIOCM_RTS;
    }
    ioctl(fd, TIOCMSET, &flags);
    return 1;
}

int rserial_enable_it(rserial* instance, void (*handler)(int))
{
    if (instance == NULL || handler == NULL)
    {
        return -1;
    }

    instance->enable_interrupt  = true;
    instance->interrupt_handler = handler;

    return 0;
}

int rserial_open(rserial*    instance,
                 const char* port_name,
                 int         baud,
                 const char* mode,
                 int         flow_ctrl,
                 int         byte_timeout_us)
{
    if (instance == NULL || port_name == NULL || instance->opened || byte_timeout_us < 0)
    {
        return -1;
    }
    instance->byte_timeout.tv_sec  = 0;
    instance->byte_timeout.tv_usec = byte_timeout_us;

    instance->fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (instance->fd == -1)
    {
        return -1;
    }

    if (instance->enable_interrupt)
    {
        struct sigaction saio;
        saio.sa_handler = instance->interrupt_handler;
        sigemptyset(&saio.sa_mask);
        saio.sa_flags = 0;
        sigaction(SIGIO, &saio, NULL);
        fcntl(instance->fd, F_SETOWN, getpid());
        fcntl(instance->fd, F_SETFL, FASYNC);
    }

    int bits, parity, stop_bits, cpar, ipar, baudrate;

    struct termios new_settings;
    memset(&new_settings, 0, sizeof(new_settings));

    if (flock(instance->fd, LOCK_EX | LOCK_NB) != 0)
    {
        close(instance->fd);
        return -1;
    }

    if (tcgetattr(instance->fd, &instance->old_settings) != 0)
    {
        close(instance->fd);
        flock(instance->fd, LOCK_UN);
        return -1;
    }

    if (mode != NULL)
    {
        if (strlen(mode) != 3)
        {
            close(instance->fd);
            flock(instance->fd, LOCK_UN);
            return -1;
        }
        if (9600 == B9600)
        {
            baudrate = baud;
        }
        else
        {
            baudrate = baud_convert(baud);
        }
        parity    = parity_convert(mode, &cpar, &ipar);
        bits      = data_bit_convert(mode);
        stop_bits = stop_bit_convert(mode);

        if (bits < 0 || parity < 0 || stop_bits < 0 || baudrate < 50)
        {
            close(instance->fd);
            flock(instance->fd, LOCK_UN);
            return -1;
        }
    }
    else
    {
        close(instance->fd);
        flock(instance->fd, LOCK_UN);
        return -1;
    }

    new_settings.c_cflag = (unsigned long) (bits | cpar | stop_bits | CLOCAL | CREAD);

    if (flow_ctrl)
    {
        new_settings.c_cflag |= CRTSCTS;
    }
    else
    {
        new_settings.c_cflag &= ~(unsigned long) CRTSCTS;
    }

    new_settings.c_iflag = (unsigned long) ipar;
    new_settings.c_iflag &= ~((unsigned long) ICANON | ECHO | ECHOE | ISIG);
    new_settings.c_oflag &= ~(unsigned long) OPOST;
    new_settings.c_cc[VMIN]  = 0;
    new_settings.c_cc[VTIME] = 0;

    cfsetospeed(&new_settings, (unsigned long) baudrate);
    cfsetispeed(&new_settings, (unsigned long) baudrate);

    if ((tcsetattr(instance->fd, TCSANOW, &new_settings)) != 0)
    {
        tcsetattr(instance->fd, TCSANOW, &instance->old_settings);
        close(instance->fd);
        flock(instance->fd, LOCK_UN);
        return -1;
    }

    //    FIXME not work on mac OS ! Maybe RTS and DTR not usable .....
    //    if (ioctl(instance->fd, TIOCMGET, &drt_rts) == -1) {
    //        tcsetattr(instance->fd, TCSANOW, &old_settings);
    //        close(instance->fd);
    //        flock(instance->fd, LOCK_UN);
    //        return -1;
    //    }
    //
    //    drt_rts |= TIOCM_DTR;
    //    drt_rts |= TIOCM_RTS;
    //
    //    if (ioctl(instance->fd, TIOCMSET, &drt_rts) == -1) {
    //        tcsetattr(instance->fd, TCSANOW, &old_settings);
    //        close(instance->fd);
    //        flock(instance->fd, LOCK_UN);
    //    }
    //        return -1;

    if (tcflush(instance->fd, TCIFLUSH) < 0)
    {
        tcsetattr(instance->fd, TCSANOW, &instance->old_settings);
        close(instance->fd);
        flock(instance->fd, LOCK_UN);
        return -1;
    }
    instance->opened = true;
    return 0;
}

int rserial_read(rserial* instance, uint8_t* data, size_t size, unsigned int timeout_us)
{
    if (instance == NULL || data == NULL || instance->opened == false || size == 0)
    {
        return -1;
    }

    int             rc;
    fd_set          selsect_set;
    struct timeval  tv;
    struct timeval* p_tv;
    int             length_to_read = (int) size;
    int             msg_length     = 0;
    FD_ZERO(&selsect_set);
    FD_SET(instance->fd, &selsect_set);
    tv.tv_sec  = 0;
    tv.tv_usec = (int) timeout_us;
    p_tv       = &tv;

    while (length_to_read != 0)
    {
        rc = serial_select(instance->fd, &selsect_set, p_tv);
        if (rc == -1)
        {
            return -1;
        }

        rc = (int) read(instance->fd, data + msg_length, (size_t) length_to_read);
        if (rc == 0 || rc == -1)
        {
            return -1;
        }

        msg_length += rc;
        length_to_read -= rc;

        if (length_to_read == 0)
        {
            break;
        }

        if (length_to_read > 0 && (instance->byte_timeout.tv_sec > 0 || instance->byte_timeout.tv_usec > 0))
        {
            tv.tv_sec  = instance->byte_timeout.tv_sec;
            tv.tv_usec = instance->byte_timeout.tv_usec;
            p_tv       = &tv;
        }
    }
    return msg_length;
}

int rserial_readline(rserial* instance, char* data, char eol, int timeout_us)
{
    if (instance == NULL || data == NULL || instance->opened == false)
    {
        return -1;
    }

    int             rc;
    fd_set          selsect_set;
    struct timeval  tv;
    struct timeval* p_tv;
    int             msg_length = 0;
    FD_ZERO(&selsect_set);
    FD_SET(instance->fd, &selsect_set);
    tv.tv_sec  = 0;
    tv.tv_usec = timeout_us;
    p_tv       = &tv;

    do
    {
        rc = serial_select(instance->fd, &selsect_set, p_tv);
        if (rc == -1)
        {
            return -1;
        }

        rc = (int) read(instance->fd, data + msg_length, 1);
        if (rc == 0 || rc == -1)
        {
            return -1;
        }

        msg_length += rc;

        if (data[msg_length - rc] == eol)
        {
            break;
        }

        if ((instance->byte_timeout.tv_sec > 0 || instance->byte_timeout.tv_usec > 0))
        {
            tv.tv_sec  = instance->byte_timeout.tv_sec;
            tv.tv_usec = instance->byte_timeout.tv_usec;
            p_tv       = &tv;
        }

    } while (1);

    return msg_length;
}

int rserial_read_stream(rserial* instance, uint8_t* data)
{
    if (instance == NULL || data == NULL || instance->opened == false)
    {
        return -1;
    }

    int rc;
    int bytes_available;

    rc = ioctl(instance->fd, FIONREAD, &bytes_available);
    if (rc == -1)
    {
        return -1;
    }

    if (bytes_available == 0)
    {
        return bytes_available;
    }

    rc = (int) read(instance->fd, data, (size_t) bytes_available);
    if (rc == 0 || rc == -1)
    {
        return -1;
    }

    return rc;
}

int rserial_write(rserial* instance, uint8_t* data, size_t size)
{
    if (data == NULL || instance->opened == false || instance == NULL || size == 0)
    {
        return -1;
    }

    int rc, err;
    int msg_length      = 0;
    int length_to_write = (int) size;

    do
    {
        set_rts(instance->fd, true);

        rc = (int) write(instance->fd, data + msg_length, size);
        if (rc == -1)
        {
            return -1;
        }

        msg_length += rc;
        length_to_write -= rc;

        if (length_to_write == 0)
        {
            break;
        }

        err = tcdrain(instance->fd);
        if (err == -1)
        {
            return -1;
        }
        set_rts(instance->fd, false);
    } while (1);

    return msg_length;
}

int rserial_close(rserial* instance)
{
    if (instance == NULL || instance->opened != true)
    {
        return -1;
    }
    if (instance->fd != 0)
    {
        tcdrain(instance->fd);
        tcsetattr(instance->fd, TCSANOW, &instance->old_settings);
        close(instance->fd);
        flock(instance->fd, LOCK_UN);
        instance->opened = false;
        return 0;
    }
    return -1;
}

#endif
