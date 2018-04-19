/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "P7.h"
#include "hilevel.h"
#include "libc.h"

void main_P7() {
    int channelID = 1;
    int selfID    = 7;
    int block     = 0;
    int me = id();

    while (1) {
        chanWrite(0, 5, me);
        int x = chanRead(0, me);
//         if (x != 0) {
//             PL011_putc(UART0, x+ '0', true);
//         }
    }



  exit( EXIT_SUCCESS );
}
