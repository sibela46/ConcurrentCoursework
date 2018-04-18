#include "philosopher.h"
#include "libc.h"
#include "hilevel.h"

extern void put_i();
int statePH = 0;

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

void main_philosopher(){
    int me = id();
    //RUN FOREVER.
    while (1){
       switch (statePH) {
           case 0:
               philosopher_thinking(me);
               int x = chanRead(me-1, 0, me);
               if (x == 0){
                   statePH = 0;
                   chanWrite(me-1, 1, 0, me); // I WANNA EAT.
               }
               else {
                   statePH = 1;
               }
               yield();
               break;
           case 1:
               philosopher_eating(me);
               chanWrite(me-1, 2, 0, me); // I HAVE EATEN.
               statePH = 0;
               yield();
               break;
           default:
               chanWrite(me-1, 1, 0, me); // I STILL WANNA EAT.
               yield();
               break;
       }
        yield();
    }

    exit(EXIT_SUCCESS);
}
