/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "P6.h"
#include "hilevel.h"
#include "libc.h"

extern void main_P7();

void main_P6() {
    int me = id();
    int child = fork();

    PL011_putc(UART0, child + '0', true);
    if (child == 0){
        exec(&main_P7);
    }

    pipe(0, 0, me, 2);
    while(1){
        int x = chanRead(0, me);
        if (x != 0){
            PL011_putc(UART0, x + '0', true);

        }
        chanWrite(0, 4, me);
        yield();
    }

  exit( EXIT_SUCCESS );
}
