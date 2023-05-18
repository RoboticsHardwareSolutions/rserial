#ifndef __RSERIAL_H__
#define __RSERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "termios.h"
#include "sys/time.h"

typedef struct
{
    int            fd;
    bool           opened;
    struct timeval byte_timeout;
    struct termios old_settings;
} rserial;

/**
 *
 * @param instance  - instance rserial
 * @param port_name - port name - example "/dev/ttyACM0"
 * @param baud - availiable bauds:#define 50 75 110 134 150 200 300 600 1200 1800 2400 4800 7200 9600 19200 38400 14400
 *                                         28800 57600 76800 115200 230400
 * @param mode example "8N1" or "7N2" or "8E1"
 * @param flow_ctrl - true for enable
 * @return 0 - if ok and -1 if error open
 */
int rserial_open(rserial* instance, char* port_name, int baud, char* mode, int flow_ctrl, int byte_timeout_us);

/**
 * @param instance
 * @param data
 * @param size
 * @return
 */
int rserial_read(rserial* instance, uint8_t* data, size_t size, unsigned int timeout_us);

int rserial_readline(rserial* instance, char* data, char eol, int timeout_us);

int rserial_write(rserial* instance, uint8_t* data, size_t size);

int rserial_close(rserial* instance);

#ifdef __cplusplus
}
#endif

#endif  //__RSERIAL_H__
