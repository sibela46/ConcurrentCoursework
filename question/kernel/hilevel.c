/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"
#include "libc.h"

pcb_t pcb[ 3 ]; int executing = 0; int processes = 3; int time = 0;

void scheduler( ctx_t* ctx ) {
//////////////////////////////////////////////////////////////////////////////////////////
///                                STAGE 1 a AND b                                     ///
//////////////////////////////////////////////////////////////////////////////////////////

  for (int i = 0; i < processes; i++){
      int nextProcess = (i+1)%processes;
      if (i == executing) {
        memcpy( &pcb[ i ].ctx, ctx, sizeof( ctx_t ) ); // preserve P_i
        pcb[ i ].status = STATUS_READY;                // update   P_i status
        if (pcb[ i+1 ].pr > pcb[ i ].pr){
            memcpy( ctx, &pcb[ nextProcess ].ctx, sizeof( ctx_t ) ); // restore  P_i+1
            pcb[ nextProcess ].status = STATUS_EXECUTING;  // update   P_i+1 status
            executing = nextProcess;                       // update   index => P_i+1
            break;
        }
        else {
            time++;
            memcpy( ctx, &pcb[ i ].ctx, sizeof( ctx_t ) ); // restore  P_i+1
            pcb[ i ].status = STATUS_EXECUTING;  // update   P_i+1 status
            executing = i;                       // update   index => P_i+1
            //if (time % 10 == 0){
               pcb[ i+1 ].pr = pcb[ i ].pr + 1;
            //}
            break;
        }
      }
  }

//////////////////////////////////////////////////////////////////////////////////////////

  return;
}

extern void     main_P3();
extern uint32_t tos_P3;
extern void     main_P4();
extern uint32_t tos_P4;
extern void     main_P5();
extern uint32_t tos_P5;

void hilevel_handler_rst(ctx_t* ctx) {

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

  for (int i = 0; i < processes; i ++){
      memset( &pcb[ i ], 0, sizeof( pcb_t ) );
      pcb[ i ].pid      = i+1;
      pcb[ i ].status   = STATUS_READY;
      pcb[ i ].ctx.cpsr = 0x50;
  }

    pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_P3 );
    pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_P3  );
    pcb[ 0 ].pr       = 7;  //priority of process 1

    pcb[ 1 ].ctx.pc   = ( uint32_t )( &main_P4 );
    pcb[ 1 ].ctx.sp   = ( uint32_t )( &tos_P4  );
    pcb[ 1 ].pr       = 3;  //priority of process 2

    pcb[ 2 ].ctx.pc   = ( uint32_t )( &main_P5 );
    pcb[ 2 ].ctx.sp   = ( uint32_t )( &tos_P5  );
    pcb[ 2 ].pr       = 5;  //priority of process 3

    /* Once the PCBs are initialised, we (arbitrarily) select one to be
     * restored (i.e., executed) when the function then returns.
     */

    memcpy( ctx, &pcb[ 0 ].ctx, sizeof( ctx_t ) );
    pcb[ 0 ].status = STATUS_EXECUTING;
    executing = 0;

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
      scheduler( ctx );
      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

	  GICC0->EOIR = id;
      ctx->gpr[ 0 ] = n;
      break;
    }

	case 0x02 : { // 0x10 => read(fd, x, n)
	  int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

 	  for( int i = 0; i < n; i++ ) {
      	x[i] = PL011_getc( UART0, true );
      }

	  write(1,x,n);
      break;
	}

  case 0x03 : {
    extern void     main_P6();
    extern uint32_t tos_P6;

    processes = processes + 1;
    pcb[ processes-1 ].pid = processes;
    pcb[ processes-1 ].status = STATUS_READY;
    pcb[ processes-1 ].ctx.cpsr = 0x50;
    pcb[ processes-1 ].ctx.pc   = ( uint32_t )( &main_P6 );
    pcb[ processes-1 ].ctx.sp   = ( uint32_t )( &tos_P6  );
    pcb[ processes-1 ].pr       = 2;  //priority of process 1
    break;
  }

  case 0x06 : {
    exit(1);
  }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }


  return;
}

void hilevel_handler_irq (ctx_t* ctx){
      // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    scheduler(ctx);
    TIMER0->Timer1IntClr = 0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}

void terminate(){
  exit(EXIT_SUCCESS);
}
