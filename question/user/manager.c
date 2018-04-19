#include "manager.h"
#include "libc.h"
#include "hilevel.h"

#define numberOfPhilosophers 16
phl_t  phl[numberOfPhilosophers];
int    forks[numberOfPhilosophers];

extern void main_philosopher();
extern void put_i();
int state = 0;
int eat  = 0;

void philosopher_eaten(int current) {
    write(STDOUT_FILENO, "Philosopher ", 12);
    put_i(UART0, current);
    write(STDOUT_FILENO, " has eaten", 10);
    PL011_putc(UART0, '\n', true);
}

void manager_reads(int x, int i) {
    write(STDOUT_FILENO, "Manager ", 8);
    write(STDOUT_FILENO, " read", 5);
    PL011_putc(UART0, eat + '0', true);
    write(STDOUT_FILENO, " from channel ", 14);
    PL011_putc(UART0, (i) + '0', true);
    PL011_putc(UART0, '\n', true);
}

void main_manager() {

    //SPAWN THE NEW PROCESSES.
    for(int i=0; i < numberOfPhilosophers; i++){
        int child = fork();
        if (child == 0){
           exec(&main_philosopher);
        }
        else {
            pipe(i, 0, 0, child-1); //-1 because console is 0, manager is 1
        }
    }

    //RUN FOREVER
    while (1) {
        for (int i=0; i < numberOfPhilosophers; i++){
            switch (state) {
                case 0:
                    state = chanRead(i, 0);
                    yield();
                    break;
                case 1:
                    chanWrite(i, 1, 0); // THINK.
                    state = 0;
                    yield();
                    break;
                case 2:
                    chanWrite(i, 2, 0); // EAT.
                    state = 0;
                    yield();
                    break;
                default:
                    chanWrite(i, 1, 0); // THINK.
                    state = 0;
                    yield();
                    break;
            }
        }
    }

    exit(EXIT_SUCCESS);
}
