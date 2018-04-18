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

void main_manager() {

    //INITIALISE PIPES.
    for (int i=0; i<numberOfPhilosophers; i++){
        pipe(i/*channelID*/, 0 /*block*/, 0 /*pid_a*/, i+1 /*pid_b*/);
        forks[i] = 1;
    }

    //SPAWN THE NEW PROCESSES.
    for(int i=0; i < numberOfPhilosophers; i++){
        int id = fork();
        if (id != 0){
           exec(&main_philosopher);
        }
    }

    //RUN FOREVER
    while (1) {
        for (int i=0; i < numberOfPhilosophers; i++){
            switch (state) {
                case 0:
                    eat = chanRead(i, 0, 0);
                    if (eat == 1){
                        state = 2; //EAT.
                    }
                    else {
                        state = 1; // THINK.
                    }
                    yield();
                    break;
                case 1:
                    chanWrite(i, 0, 0, 0); // THINK.
                    state = 0;
                    yield();
                    break;
                case 2:
                    chanWrite(i, 1, 0, 0); // EAT.
                    state = 0;
                    yield();
                    break;
                default:
                    chanWrite(i, 0, 0, 0); // THINK.
                    state = 0;
                    yield();
                    break;
            }
        }
    }

    exit(EXIT_SUCCESS);
}
