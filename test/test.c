#include "rserial.h"
#include "runit/src/runit.h"

rserial serial1;
rserial serial2;

#define VIRTUAL_PORT1 "/dev/ttys001"
#define VIRTUAL_PORT2 "/dev/ttys002"

uint8_t example_of_data1[] = {0xAA, 0xBB, 0xCC};
char    example_of_data2[] = "Hello World\n";
char    example_of_data3[] = "Hi";
uint8_t read_data[100];

void test_open_close(void)
{
    runit_true(rserial_open(&serial1, VIRTUAL_PORT1, 115200, "8N1", false, 7000) == 0);
    runit_true(rserial_open(&serial2, VIRTUAL_PORT2, 115200, "8N1", false, 7000) == 0);
    runit_true(rserial_close(&serial1) == 0);
    runit_true(rserial_close(&serial2) == 0);
}

void test_read_write(void)
{
    runit_true(rserial_open(&serial1, VIRTUAL_PORT1, 115200, "8N1", false, 7000) == 0);
    runit_true(rserial_open(&serial2, VIRTUAL_PORT2, 115200, "8N1", false, 7000) == 0);

    /** serial1 send*/
    memset(read_data, 0, sizeof(read_data));
    runit_true(rserial_write(&serial1, example_of_data1, sizeof(example_of_data1)) == sizeof(example_of_data1));
    runit_true(rserial_read(&serial2, read_data, sizeof(example_of_data1), 1000) == sizeof(example_of_data1));
    runit_true(memcmp(example_of_data1, read_data, sizeof(example_of_data1)) == 0);

    /** serial2 send*/
    memset(read_data, 0, sizeof(read_data));
    runit_true(rserial_write(&serial2, example_of_data1, sizeof(example_of_data1)) == sizeof(example_of_data1));
    runit_true(rserial_read(&serial1, read_data, sizeof(example_of_data1), 1000) == sizeof(example_of_data1));
    runit_true(memcmp(example_of_data1, read_data, sizeof(example_of_data1)) == 0);

    runit_true(rserial_close(&serial1) == 0);
    runit_true(rserial_close(&serial2) == 0);
}

void test_read_write_with_timeout(void)
{
    runit_true(rserial_open(&serial1, VIRTUAL_PORT1, 115200, "8N1", false, 1000) == 0);
    runit_true(rserial_open(&serial2, VIRTUAL_PORT2, 115200, "8N1", false, 1000) == 0);

    /** serial1 send invalid*/
    memset(read_data, 0, sizeof(read_data));
    runit_true(rserial_write(&serial1, example_of_data1, sizeof(example_of_data1) - 1) == sizeof(example_of_data1) - 1);
    runit_true(rserial_read(&serial2, read_data, sizeof(example_of_data1), 1000) == -1);

    /** serial2 send valid*/
    memset(read_data, 0, sizeof(read_data));
    runit_true(rserial_write(&serial2, example_of_data1, sizeof(example_of_data1)) == sizeof(example_of_data1));
    runit_true(rserial_read(&serial1, read_data, sizeof(example_of_data1), 1000) == sizeof(example_of_data1));
    runit_true(memcmp(example_of_data1, read_data, sizeof(example_of_data1)) == 0);

    runit_true(rserial_close(&serial1) == 0);
    runit_true(rserial_close(&serial2) == 0);
}

void test_readline_write(void)
{
    runit_true(rserial_open(&serial1, VIRTUAL_PORT1, 115200, "8N1", false, 1000) == 0);
    runit_true(rserial_open(&serial2, VIRTUAL_PORT2, 115200, "8N1", false, 1000) == 0);

    /** serial2 send */
    memset(read_data, 0, sizeof(read_data));
    runit_true(rserial_write(&serial2, (uint8_t*) example_of_data2, sizeof(example_of_data2) - 1) ==
               sizeof(example_of_data2) - 1);
    runit_true(rserial_readline(&serial1, (char*) read_data, '\n', 1000) == sizeof(example_of_data2) - 1);
    runit_true(memcmp(example_of_data2, read_data, sizeof(example_of_data2)) == 0);

    memset(read_data, 0, sizeof(read_data));
    runit_true(rserial_write(&serial2, (uint8_t*) example_of_data3, sizeof(example_of_data3)) ==
               sizeof(example_of_data3));
    runit_true(rserial_readline(&serial1, (char*) read_data, '\0', 1000) == sizeof(example_of_data3));
    runit_true(memcmp(example_of_data3, read_data, sizeof(example_of_data3)) == 0);

    memset(read_data, 0, sizeof(read_data));
    runit_true(rserial_write(&serial2, (uint8_t*) example_of_data3, sizeof(example_of_data3) - 1) ==
               sizeof(example_of_data3) - 1);
    runit_true(rserial_readline(&serial1, (char*) read_data, '\0', 1000) == -1);
    runit_true(memcmp(example_of_data3, read_data, sizeof(example_of_data3)) == 0);

    runit_true(rserial_close(&serial1) == 0);
    runit_true(rserial_close(&serial2) == 0);
}

int main(void)
{
    printf("WARNING DEFINE NAME OF TWO SERIAL PORT FOR TEST\n");
    printf("For create two virtual serial enter :\n");
    printf("$ socat -d -d pty,raw,echo=0 pty,raw,echo=0\n");
    printf("You get next text :\n");
    printf("$ socat[17554] N PTY is %s\n", VIRTUAL_PORT1);
    printf("$ socat[17554] N PTY is %s\n", VIRTUAL_PORT2);
    printf("$ socat[17554] N starting data transfer loop with FDs [5,5] and [7,7]\n");
    printf("After you must define VIRTUAL_PORT1 & VIRTUAL_PORT2 -> %s & %s \n", VIRTUAL_PORT2, VIRTUAL_PORT1);
    printf("rserial start unit test\n");
    test_open_close();
    test_read_write();
    test_read_write_with_timeout();
    test_readline_write();
    runit_report();
    return runit_at_least_one_fail;
}
