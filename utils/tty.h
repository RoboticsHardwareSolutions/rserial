#ifndef __TTY_H__
#define __TTY_H__

void tty_save_backup(void);
void tty_set_backup(void);
void tty_set_non_canonical_mode(void);
void tty_read_nonblock(void);
void tty_read_block(void);
void tty_flash(void);

#endif
