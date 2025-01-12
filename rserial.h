#pragma once
#include "rserial_def.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"

typedef enum {
  FLOW_CTRL_NONE,
  FLOW_CTRL_RTS,
  FLOW_CTRL_CTS,
  FLOW_CTRL_RTSCTS,
  FLOW_CTRL_DE,

} flow_ctrl_t;

typedef struct serial_port rserial;

/**
 * @brief                   Open serial device with params.
 *
 * @param instance          Rserial instance
 * @param port_name         Port name, for example "/dev/ttyS0"
 * @param baud              Availiable bauds:#define 50 75 110 134 150 200 300
 * 600 1200 1800 2400 4800 7200 9600 19200 38400 14400 28800 57600 76800 115200
 * 230400
 * @param mode              Port mode, for example "8N1" or "7N2" or "8E1" or
 * "8O1" ( O - parity ODD, N - parity None, N - parity Even)
 * @param flow_ctrl         Flow control for port (1 - enable, 0 - disable)
 * @param byte_timeout_us   Timeout. Saved to instance. Nesessary to set not 0
 * value (f.e. 5000000) for correct working of rserial_read()
 * @return int              Return err -1 or OK - 0
 */
int rserial_open(rserial *instance, const char *port_name, int baud,
                 const char *mode, flow_ctrl_t flow_ctrl, int byte_timeout_us);

/**
 * @brief                   Read bytes from device up to a certain eol.
 *                          Blocking\non-blocking read, depending of timeout.
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, where to reed data
 * @param size              Size of expected data
 * @param timeout_us        Timeout. Blocking.
 * @return int              Return err -1, 0 - no available data, >0 - size of
 * readed data from device
 */
int rserial_read(rserial *instance, uint8_t *data, size_t size,
                 unsigned int timeout_us);

/**
 * @brief                   Read bytes from device with fixed size and timeout.
 *                          Blocking\non-blocking read, depending of timeout.
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, where to reed data
 * @param eol               Character for marking "end of line"
 * @param timeout_us        Timeout. Blocking.
 * @return int              Return err -1, 0 - no available data, >0 - size of
 * readed data from device
 */
int rserial_readline(rserial *instance, char *data, char eol, int timeout_us);

/**
 * @brief                   Read bytes from device with non-blocking mode.
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, where to reed data
 * @return int              Return err -1, 0 - no available data, >0 - size of
 * readed data from device
 */
int rserial_read_stream(rserial *instance, uint8_t *data);

/**
 * @brief
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, which to send
 * @param size              Size of data
 * @return int              Return err -1, 0 - no available to write data, >0 -
 * size of written data to device
 */
int rserial_write(rserial *instance, uint8_t *data, size_t size);

bool rserial_is_ok(rserial *instance);

int rserial_close(rserial *instance);

/**
 * @brief                   Enables IT mode. Need to define handler function and
 * use before rserial_open().
 *
 * @param instance          Rserial instance
 * @param handler           Interrupt handler function
 * @return int              Return err -1, 0 - no available data, >0 - size of
 * readed data from device
 */
int rserial_enable_it(rserial *instance, void (*handler)(int));

/** read and write with IT **/
#if defined(STM32F072xB) || defined(STM32F091xC) || defined(STM32F103xB) ||    \
    defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F103xE) ||    \
    defined(STM32F765xx) || defined(STM32G474xx)

/**
 * @brief
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, which to send
 * @param size              Size of data
 * @return int              Return err -1, 0 - no available to write data, >0 -
 * size of written data to device
 */
int rserial_write_it(rserial *instance, uint8_t *data, size_t size);

/**
 * @brief                   Read bytes from device up to a certain eol.
 *
 * @param instance          Rserial instance
 * @param data              Pointer to buffer, where to reed data
 * @param size              Size of expected data
 * @return int              Return err -1, 0 - no available data, >0 - size of
 * readed data from device
 */
int rserial_read_it(rserial *instance, uint8_t *data, size_t size);

#endif




