/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"
#include "libc.h"

#define pcbSize 1000

int processes = 4; pcb_t pcb[ pcbSize ]; int executing = 0; int time = 0;

void executeNext (pcb_t* current, pcb_t* next, ctx_t* ctx){
  memcpy( &current->ctx, ctx, sizeof( ctx_t ) ); // preserve P_i
  current->status = STATUS_READY;                // update   P_i status
  memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  P_1
  next->status = STATUS_EXECUTING;  // update   P_1 status
  executing = next->pid;
}

void fixscheduler( ctx_t* ctx ) {
  int nextProcess = (executing+1)%(processes+1);
  // PL011_putc(UART0, 'E', 1);
  // PL011_putc(UART0, executing+ '0', 1);
  // PL011_putc(UART0, 'N', 1);
  // PL011_putc(UART0, nextProcess + '0', 1);

  while (pcb[nextProcess].status == STATUS_TERMINATED){
    nextProcess = (nextProcess+1)%(processes+1);
  }

  executeNext(&pcb[executing], &pcb[nextProcess], ctx);
  //PL011_putc(UART0, 'Z', 1);
}


void priorityscheduler( ctx_t* ctx ) {
  for (int i = 0; i < processes; i++){
      int nextProcess = (i+1)%(processes);
      if (i == executing && pcb[i].status != STATUS_TERMINATED) {
        if (pcb[ nextProcess ].pr > pcb[ i ].pr){
            executeNext(&pcb[i], &pcb[nextProcess], ctx);
            break;
        }
        else {
            //time++;
            executeNext(&pcb[i], &pcb[i], ctx);
            //if (time % 3 == 0){
               pcb[ nextProcess ].pr = pcb[ i ].pr + 1;
            //}
            break;
        }
      }
  }
  return;
}

extern void     main_P3();
extern void     main_P4();
extern void     main_P5();
extern void     main_console();
extern uint32_t tos_console;
extern uint32_t tos_addedP;

void initialise_pcb (ctx_t* ctx) {

   for (int i = 1; i < processes+1; i ++){
       memset( &pcb[ i ], 0, sizeof( pcb_t ) );
       pcb[ i ].pid      = i;
       pcb[ i ].status   = STATUS_READY;
       pcb[ i ].ctx.cpsr = 0x50;
   }

     pcb[ 1 ].ctx.pc   = ( uint32_t )( &main_P3 );
    // pcb[ 1 ].ctx.sp   = ( uint32_t )( &tos_P3  );
    pcb[ 1 ].pr       = 3;  //priority of process 3

     pcb[ 2 ].ctx.pc   = ( uint32_t )( &main_P4 );
    // pcb[ 2 ].ctx.sp   = ( uint32_t )( &tos_P4  );
    pcb[ 2 ].pr       = 5;  //priority of process 4

     pcb[ 3 ].ctx.pc   = ( uint32_t )( &main_P5 );
    // pcb[ 3 ].ctx.sp   = ( uint32_t )( &tos_P5  );
    pcb[ 3 ].pr       = 7;  //priority of process 4

}

void add_process (ctx_t* ctx) {
    processes += 1;
    uint32_t sizeOfProcess = 0x00001000;
    memcpy( &pcb[ processes ], 0, sizeof( pcb_t ) );
    memcpy( &pcb[ processes ].ctx, ctx, sizeof( ctx_t ) );
    pcb[ processes ].pid      = processes;

   pcb[ processes ].tos = (uint32_t)( &tos_addedP - (processes*0x00001000));
   uint32_t offset = pcb[executing].tos - pcb[executing].ctx.sp;
   memcpy( (void*) pcb[ processes ].tos - sizeOfProcess, (void*) pcb[0].tos - sizeOfProcess, sizeOfProcess); //because memcpy opposite of stack
   pcb[ processes ].ctx.sp = pcb[ processes ].tos - offset;

    // uint32_t new_tos = ( uint32_t )( ((uint32_t)&tos_addedP - 16*0x00001000) + processes*0x00001000);
    // pcb[ processes ].tos      = new_tos;
    // uint32_t offset = pcb[0].tos - pcb[0].ctx.sp;
    // pcb[ processes ].ctx.sp   = new_tos - offset;
    //
    // memcpy( (void*) pcb[ processes ].tos - 0x00001000, (void*) pcb[0].tos - 0x00001000, 0x00001000); //because memcpy opposite of stack

}


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

    pcb[ 0 ].pid      = 0;
    pcb[ 0 ].status   = STATUS_READY;
    pcb[ 0 ].ctx.cpsr = 0x50;
    pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
    pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_console  );
    pcb[ 0 ].tos      = ( uint32_t )( &tos_console  );
    pcb[ 0 ].pr       = 4;  //priority of process 1

    initialise_pcb(ctx);

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
      //scheduler( ctx );
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
    add_process(ctx);
    break;
  }

  case 0x04: {
    pcb[executing].status = STATUS_TERMINATED;
    priorityscheduler(ctx);
  }

  case 0x05 : {
    //pcb[ executing ].ctx.cpsr = 0x50;
    uint32_t address = (uint32_t) ctx->gpr[0];
    memset((void*)(pcb[executing].tos - 0x00001000), 0, 0x00001000);
    ctx->sp = pcb[executing].tos;
    ctx->pc = (uint32_t) address;
    break;
  }

  case 0x06 : {
    pcb[executing].status = STATUS_TERMINATED;
    break;
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
    //scheduler(ctx);
    priorityscheduler(ctx);
    TIMER0->Timer1IntClr = 0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}
