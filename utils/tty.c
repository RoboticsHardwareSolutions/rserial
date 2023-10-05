#include "tty.h"
#include "stdio.h"
#include <unistd.h>
#include "string.h"
#include <fcntl.h>
#include "termios.h"
#include "stdint.h"

static struct termios tty, backup_tty;

void tty_save_backup(void)
{
    tcgetattr(STDIN_FILENO, &backup_tty);
}

void tty_set_backup(void)
{
    uint8_t null_array[sizeof(struct termios)];
    memset(null_array, 0, sizeof(struct termios));

    if (memcmp(&backup_tty, null_array, sizeof(struct termios)) == 0)
    {
        printf("error tty zero value\r\n");
        return;
    }
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &backup_tty);
}

void tty_set_non_canonical_mode(void)
{
    uint8_t null_array[sizeof(struct termios)];
    memset(null_array, 0, sizeof(struct termios));

    if (memcmp(&backup_tty, null_array, sizeof(struct termios)) == 0)
    {
        printf("error mmust do buckup tty \r\n");
        return;
    }

    tcgetattr(STDIN_FILENO, &tty);
    tty.c_lflag &= ~(ICANON | ECHO | ISIG);
    tty.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &tty);
}

void tty_read_nonblock(void)
{
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void tty_read_block(void)
{
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

void tty_flash(void)
{
    tcflush(STDIN_FILENO, TCIFLUSH);
}
