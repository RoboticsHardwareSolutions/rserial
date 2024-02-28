#include "unistd.h"
#include "screen.h"
#include "tty.h"
#include "stdbool.h"
#include "ctype.h"
#include "signal.h"
#include "rserial.h"
#include "stdio.h"

#define RECEIVE_BUFFER_SIZE 1000
#define RECEIVED_DATA_STRING_SIZE 12

volatile bool quit = false;

rserial       serial;
unsigned char receive_buffer[RECEIVE_BUFFER_SIZE];
unsigned long receive_len;

unsigned char pream[]   = {255, 255};
unsigned char id[]      = {1};
unsigned char len[]     = {8};
unsigned char payload[] = {67, 103, 81, 113, 65, 103, 103, 66};
unsigned char crc[]     = {179, 40};
unsigned char all[]     = {255, 255, 1, 8, 67, 103, 81, 113, 65, 103, 103, 66, 179, 40};

void exit_app()
{
    quit = true;
}

void view_input_data()
{
    println(" INPUT --------------------------------------------------");
    unsigned int print_line = receive_len / RECEIVED_DATA_STRING_SIZE;
    if (print_line == 0 && (receive_len % RECEIVED_DATA_STRING_SIZE) != 0)
    {
        print_line++;
    }
    unsigned int buf_ptr = 0;
    for (unsigned long i = 0; i < print_line; i++)
    {
        char line[RECEIVED_DATA_STRING_SIZE * 3];  // it means HEX symbol two characters and + " " between characters
        memset(line, 0, sizeof(line));
        unsigned int internal_line_prt = 0;
        for (unsigned int y = 0; y < RECEIVED_DATA_STRING_SIZE; y++)
        {
            sprintf(&line[internal_line_prt], "%02X ", receive_buffer[buf_ptr]);
            internal_line_prt = internal_line_prt + 3;
            buf_ptr++;
        }
        buf_ptr = buf_ptr + RECEIVED_DATA_STRING_SIZE;
        println("-> %.*s", sizeof(line), line);
    }
}

void init_stdin()
{
    tty_save_backup();
    tty_set_non_canonical_mode();
    tty_read_nonblock();
}

void deinit_stdin(void)
{
    tty_set_backup();
    tty_read_block();
    tty_flash();
}

void show_output_echo(void)
{
    println(" OUTPUT --------------------------------------------------");
}

void get_cmd_from_stdin(void)
{
    char sym;
    int  answer = read(STDIN_FILENO, &sym, 1);

    if (answer != -1 && answer != 0)
    {
        if (isalpha(sym))
        {
            if (isupper(sym))
                sym = tolower(sym);
        }

        switch (sym)
        {
        case '0':

            break;
        case '1':
            rserial_write(&serial, pream, sizeof(pream));
            break;
        case '2':
            rserial_write(&serial, id, sizeof(id));
            break;
        case '3':
            rserial_write(&serial, len, sizeof(len));
            break;
        case '4':
            rserial_write(&serial, payload, sizeof(payload));
            break;
        case '5':
            rserial_write(&serial, crc, sizeof(crc));
            break;
        case '6':
            rserial_write(&serial, all, sizeof(all));
            break;
        case '7':
            break;
        case '\x1b':
        case 0x03:
        case 0x18:
        case 0x1A:
            exit_app();
            break;
        default:
            break;
        }
    }
}

int main(void)
{
    if ((signal(SIGTERM, exit_app) == SIG_ERR) || (signal(SIGINT, exit_app) == SIG_ERR) ||
        (signal(SIGTSTP, exit_app) == SIG_ERR))
    {
        printf("%s error define signal handler \n", __func__);
        return 0;
    }
    if (rserial_open(&serial, "/dev/ttyS1", 115200, "8N1", 0, 100))
    {
        printf("cannot open serial\n");
        exit_app();
    }
    init_stdin();
    while (!quit)
    {
        // GET DATA
        int res = rserial_read(&serial, &receive_buffer[receive_len], RECEIVED_DATA_STRING_SIZE, 100000);
        if (res != -1)
        {
            receive_len = receive_len + res;
        }
        clean_screen();
        start_screen();
        view_input_data();
        get_cmd_from_stdin();
        show_output_echo();
        //  SEND DATA
        end_screen();
        usleep(10000);
    }
    deinit_stdin();
    clean_screen();
    rserial_close(&serial);
    return 0;
}