# rserial

Simple example how to use
```
int main(int argc, char** argv)
{
    rserial serialport;
    int     err = 0, bytes_aval, err;
    char    buff[100];

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



