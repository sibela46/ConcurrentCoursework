/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

/* Since we *know* there will be 2 processes, stemming from the 2 user
 * programs, we can
 *
 * - allocate a fixed-size process table (of PCBs), and then maintain
 *   an index into it for the currently executing process,
 * - employ a fixed-case of round-robin scheduling: no more processes
 *   can be created, and neither is able to terminate.
 */

#define totalprocesses 5


extern void     main_P1();
extern uint32_t tos_P1;
extern void     main_P2();
extern void     main_P3();
extern void     main_P4();
extern void     main_P5();

pcb_t pcb[ totalprocesses ];
int executing = 0;

uint32_t currentstackpos = ( uint32_t )( &tos_P1 );

// a) uses index to determine which process is currently executing,
// b) preserves the execution context
// of that process by invoking memcpy to copy ctx into the associated PCB,
// c) restores the execution context of the only other process by invoking memcpy to copy the
//    associated PCB into ctx,
// d) updates index to reflect the currently executing process.


void switchprocess(ctx_t* ctx, int nextprocess){
    PL011_putc( UART0, pcb[executing].status + '0', 1); //Printing integers
    PL011_putc( UART0, '\n', true);
    PL011_putc( UART0, executing + '0', 1); //Printing which process is executing
    memcpy( &pcb[ executing ].ctx, ctx, sizeof( ctx_t ) ); // preserve P_1
    if( pcb[ executing ].status != STATUS_TERMINATED){     // update   P_1 status
        pcb[ executing ].status = STATUS_READY;
    }
    memcpy( ctx, &pcb[ nextprocess ].ctx, sizeof( ctx_t ) ); // restore  P_2
    pcb[ nextprocess ].status = STATUS_EXECUTING;            // update   P_2 status
    executing = nextprocess; // update   index => P_2
    PL011_putc( UART0, '\n', true);

    return;
}

void updatemissed() {
    for(int p=0; p < totalprocesses; p++){
        if(pcb[ p ].status != STATUS_EXECUTING ){
            pcb[ p ].missed++;
        }else{
            pcb[ p ].missed = 0;
        }
    }
  return;
}

void age() {
    for(int p=0; p < totalprocesses; p++){
        if ( pcb[ p ].priority - pcb[ p ].missed > 0 ) {
            pcb[ p ].priority -= pcb[ p ].missed;
        }
    }
  return;
}

void roundrobinscheduler( ctx_t* ctx ) {
    int nextprocess = ((executing+1)%totalprocesses);

    if(pcb[ executing ].status != STATUS_TERMINATED){
        nextprocess = ((executing+2)%totalprocesses);
    }
    switchprocess(ctx, nextprocess);
  return;
}

void priorityscheduler( ctx_t* ctx ) {
    //check how many system calls, increment for each.
    //age every time the process isn't run by adding an attribute 'missed' - decrease priority by 5
    //0 = highest priority
        int nextprocess = 0;

        for(int p=0; p < totalprocesses; p++){
            if(pcb[ p ].status != STATUS_TERMINATED){
               if(pcb[ p ].priority < pcb[nextprocess].priority){
                  nextprocess = p;
               }
            }
        }
        //finding highest priority process aka: lowest number

        pcb[nextprocess].priority += 10;

        switchprocess(ctx, nextprocess);

        updatemissed();
        age();
  return;
}


//create an array with tos_P's stack pointers so when priority scheduling, you can index it.
uint32_t tospointers[totalprocesses];

uint32_t processpointers [ totalprocesses ] = {( uint32_t )( &main_P1 ), ( uint32_t )( &main_P2 ), ( uint32_t )( &main_P3 ), ( uint32_t )( &main_P4 ), ( uint32_t )( &main_P5 )};
//uint32_t tospointers [ totalprocesses ] = {( uint32_t )( &tos_P1 ), ( uint32_t )( &tos_P2 ), ( uint32_t )( &tos_P3 ), ( uint32_t )( &tos_P4 ), ( uint32_t )( &tos_P5 )};

uint32_t allocatestack(int i){
    uint32_t returnstackpos = currentstackpos;
    currentstackpos -= 0x00001000;
    tospointers[i] = returnstackpos;
    return returnstackpos;
}
void setmemory(int i, uint32_t processpointer, uint32_t tos){
      memset( &pcb[ i ], 0, sizeof( pcb_t ) );
      pcb[ i ].pid      = (i+1);
      pcb[ i ].status   = STATUS_READY;
      pcb[ i ].priority = (i+1); //same as id of process so process priorities are [1,2,3,4,5]
      pcb[ i ].ctx.cpsr = 0x50;
      pcb[ i ].ctx.pc   = processpointer;
      pcb[ i ].ctx.sp   = tos;
   return;
}

void hilevel_handler_rst( ctx_t* ctx              ) {
  /* Configure the mechanism for interrupt handling by
   *
   * - configuring timer st. it raises a (periodic) interrupt for each
   *   timer tick,
   * - configuring GIC st. the selected interrupts are forwarded to the
   *   processor via the IRQ interrupt signal, then
   * - enabling IRQ interrupts.
   */

  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor

  int_enable_irq();
  /* Initialise PCBs representing processes stemming from execution of
   * the two user programs.  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR
   *   mode, with IRQ interrupts enabled, and
   * - the PC and SP values matche the entry point and top of stack.
   */
  for(int i=0; i<totalprocesses; i++){
      setmemory(i, processpointers[i], allocatestack(i));
  }

  /* Once the PCBs are initialised, we (arbitrarily) select one to be
   * restored (i.e., executed) when the function then returns.
   */

  memcpy( ctx, &pcb[ 0 ].ctx, sizeof( ctx_t ) ); //&pcb[].ctx is the address in memory to that data
  pcb[ 0 ].status = STATUS_EXECUTING;
  executing = 0;

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {
  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    //roundrobinscheduler(ctx);
    priorityscheduler(ctx);
    //PL011_putc( UART0, 'T', true );
    TIMER0->Timer1IntClr = 0x01;

  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identified encoded as an immediate operand in the
   * instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call,
   * - write any return value back to preserved usr mode registers.
   */

  switch( id ) {
    case 0x00 : { // 0x00 => yield()
      //roundrobinscheduler(ctx);
      //systemcalled increments for every system call for priorityscheduler
        pcb[ executing ].priority++;
        priorityscheduler(ctx);
      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

      //systemcalled increments for every system call for priorityscheduler
      pcb[ executing ].priority++;
      ctx->gpr[ 0 ] = n;
      break;
    }

    case 0x04 : { // 0x04 => exit()
        pcb[ executing ].status = STATUS_TERMINATED;
        //roundrobinscheduler(ctx);
        priorityscheduler(ctx);
      break;
    }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}
