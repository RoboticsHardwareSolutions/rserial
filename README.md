# rserial

### MCU with Interrupt:

1. If you want to use IT, you must enable IRQ of your UART interface in MspInit function after initialization of pins for this interface.

```c
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    ...
    // initialize pins for USART6
    
    /* USER CODE BEGIN USART6_MspInit 1 */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    /* USER CODE END USART6_MspInit 1 */
}
```

2. After `rserial_open(&serial, ...)` to registrate necessary callbacks for this instance:
```c
{
    is_opened = rserial_open(&serial, port_name, 115200, "8N1", FLOW_CTRL_DE, 4000);

    HAL_UART_RegisterCallback(&serial.uart, HAL_UART_TX_COMPLETE_CB_ID, tx_cplt_callback);
    HAL_UART_RegisterCallback(&serial.uart, HAL_UART_RX_COMPLETE_CB_ID, rx_cplt_callback);
}
```
_Note:_ `tx_cplt_callback` and `rx_cplt_callback` must be defined.


3. For this instance must be defined IRQ Handler by you:

```c
void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&serial.uart);
}
```

4After that you can to use read and write function with suffix _it.
Example:
```c
rserial_write_it(&serial, buffer, sizeof(buffer));
```

### UNIX

Simple example how to use
```

volatile sig_atomic_t data_available;

void interrupt_handler(int data)
{
    // ...

    data_available = true;

    // ...
}

int main(int argc, char** argv)
{
    rserial serialport;
    int     err = 0, bytes_aval, err;
    char    buff[100];

    // When you choose to use IT, uncomment next
    // err = rserial_enable_it(&serialport, interrupt_handler);
    //  Open port with parametrs
    err = rserial_open(&serialport, "/dev/ttyS0", 115200, "8N1", 0, 10000);
    if (err == -1)
    {
        printf("Error opening port (%d): %s\n", errno, strerror(errno));
        return 0;
    }

    while (1)
    {
        // Read data from ttyX

        // Method 1
        // Read data with fixed size and timeout (Blocking mode).
        bytes_aval = rserial_read(&serialport, buff, 10, 100);
        if (bytes_aval > 0)
        {
            // Just see what received
            printf("%.*s", bytes_aval, buff);
        }
        else if (bytes_aval == -1)
        {
            printf("Error reading from port (%d): %s\n", errno, strerror(errno));
        }
        else
        {
            printf("No data available\n");
        }

        // Method 2
        // Read data w\o size (Non-blocking mode)
        bytes_aval = rserial_read_no_size(&serialport, buff);
        if (bytes_aval > 0)
        {
            // Just see what received
            printf("%.*s", bytes_aval, buff);
        }
        else if (bytes_aval == -1)
        {
            printf("Error reading from port (%d): %s\n", errno, strerror(errno));
        }
        else
        {
            printf("No data available\n");
        }

        // Method 3
        // Read data after sigaction fires
        if(data_available)
        {
            // ...
            read data from ttyX with any other methods
            // ...
            data_available = false;
        }

        //  Write data to ttyX
        err = rserial_write(&serialport, "Hello!", sizeof("Hello!"));
        if (err == -1)
        {
            printf("Error writing to port (%d): %s\n", errno, strerror(errno));
        }
    }
    // Just close port
    rserial_close(&serialport);
    return 1;
}
```



