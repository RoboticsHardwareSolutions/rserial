#ifndef __RSERIAL_H__
#define __RSERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "rserial_def.h"

typedef struct serial_port rserial;

/**
 * @brief                   Open serial device with params.
 *
 * @param instance          Rserial instance
 * @param port_name         Port name, for example "/dev/ttyS0"
 * @param baud              Availiable bauds:#define 50 75 110 134 150 200 300 600 1200 1800 2400 4800 7200 9600 19200
 *                          38400 14400 28800 57600 76800 115200 230400
 * @param mode              Port mode, for example "8N1" or "7N2" or "8E1" or "8O1"
 *                          ( O - parity ODD, N - parity None, N - parity Even)
 * @param flow_ctrl         Flow control for port (1 - enable, 0 - disable)
 * @param byte_timeout_us   Timeout. Saved to instance. Nesessary to set not 0 value (f.e. 5000000)
 *                          for correct working of rserial_read()
 * @return int              Return err -1 or OK - 0
 */
int rserial_open(rserial* instance, char* port_name, int baud, char* mode, int flow_ctrl, int byte_timeout_us);

/**
 * @brief                   Read bytes from device up to a certain eol.
 *                          Blocking\non-blocking read, depending of timeout.
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, where to reed data
 * @param size              Size of expected data
 * @param timeout_us        Timeout. Blocking.
 * @return int              Return err -1, 0 - no available data, >0 - size of readed data from device
 */
int rserial_read(rserial* instance, uint8_t* data, size_t size, unsigned int timeout_us);

/**
 * @brief                   Read bytes from device with fixed size and timeout.
 *                          Blocking\non-blocking read, depending of timeout.
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, where to reed data
 * @param eol               Character for marking "end of line"
 * @param timeout_us        Timeout. Blocking.
 * @return int              Return err -1, 0 - no available data, >0 - size of readed data from device
 */
int rserial_readline(rserial* instance, char* data, char eol, int timeout_us);

/**
 * @brief                   Read bytes from device with non-blocking mode.
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, where to reed data
 * @return int              Return err -1, 0 - no available data, >0 - size of readed data from device
 */
int rserial_read_no_size(rserial* instance, uint8_t* data);

/**
 * @brief
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, which to send
 * @param size              Size of data
 * @return int              Return err -1, 0 - no available to write data, >0 - size of written data to device
 */
int rserial_write(rserial* instance, uint8_t* data, size_t size);

/**
 * @brief
 *
 * @param instance          Rserial instance
 * @return int              Return err -1 or OK - 0
 */
int rserial_close(rserial* instance);

/**
 * @brief                   Enables IT mode. Need to use before rserial_open().
 *
 * @param instance          Rserial instance
 * @return int              Return err -1 or OK - 0
 */
int rserial_enable_it(rserial* instance);

#ifdef __cplusplus
}
#endif

#endif  //__RSERIAL_H__
