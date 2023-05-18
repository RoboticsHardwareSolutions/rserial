#include "rserial.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "string.h"
#include <sys/ioctl.h>
#include "errno.h"

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

int rserial_open(rserial* instance, char* port_name, int baud, char* mode, int flow_ctrl, int byte_timeout_us)
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
    tv.tv_usec = (int)timeout_us;
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

int rserial_write(rserial* instance, uint8_t* data, size_t size)
{
    if (data == NULL || instance->opened == false || instance == NULL || size == 0)
    {
        return -1;
    }
    int rc;
    int msg_length      = 0;
    int length_to_write = (int) size;
    do
    {
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
