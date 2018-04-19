#include "philosopher.h"
#include "libc.h"
#include "hilevel.h"

extern void put_i();
int statePH = 0;
int x = 0;

void philosopher_thinking(int me) {
    write(STDOUT_FILENO, "Philosopher ", 12);
    put_i(UART0, me);
    write(STDOUT_FILENO, " thinking", 9);
    PL011_putc(UART0, '\n', true);
}

void philosopher_eating(int me) {
    write(STDOUT_FILENO, "Philosopher ", 12);
    put_i(UART0, me);
    write(STDOUT_FILENO, " eating", 7);
    PL011_putc(UART0, '\n', true);
}

void philosopher_reads(int me) {
    write(STDOUT_FILENO, "Philosopher ", 12);
    put_i(UART0, me);
    write(STDOUT_FILENO, " read", 5);
    PL011_putc(UART0, x + '0', true);
    write(STDOUT_FILENO, " from channel ", 14);
    PL011_putc(UART0, (me-1) + '0', true);
    PL011_putc(UART0, '\n', true);
}

void main_philosopher(){
    int x = id();
    int me = (x-1);
     chanWrite(me-1, 2, me); // I WANNA EAT.
     //RUN FOREVER.
     while (1){
       switch (statePH) {
            case 0:
                statePH = chanRead(me-1, me);
                yield();
                break;
            case 1:
                philosopher_thinking(me);
                chanWrite(me-1, 2, me); // I WANNA EAT.
                statePH = 0;
                yield();
                break;
            case 2:
                philosopher_eating(me);
                chanWrite(me-1, 3, me); // I HAVE EATEN.
                statePH = 0;
                yield();
                break;
            default:
                statePH = 0;
                chanWrite(me-1, 2, me); // I STILL WANNA EAT.
                yield();
                break;
        }
     }

    exit(EXIT_SUCCESS);
}
