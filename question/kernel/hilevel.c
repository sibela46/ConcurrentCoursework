/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

 /* Parts to uncomment for the different stages:
  *
  * 1. Stage1-a
  *    a)change processes to 2
  *    b)remove P5 from the userPrograms table.
  *    c)uncomment roundRobinScheduler and comment out priorityScheduler
  *    d)uncomment iniatialiseProcesses
  * 2. Stage1-b
  *    a)change processes to 3
  *    b)add P5 back to the table
  *    c)uncomment roundRobinScheduler and comment out priorityScheduler
  * 3. Stage2-a
  *    a)change processes to 0 as they will be dynamically created.
  *    b)leave only P3, P4 and P5
  *    c)comment out initialisation of processes
  * 4. Stage2-b
  *    a)type in execute philosophers
  *
  * They don't *precisely* match the standard C library, but are intended
  * to act as a limited model of similar concepts.
  */
#include "libc.h"
#include "hilevel.h"

#define numberOfPhilosophers 16
#define numberOfProcesses 1000
#define sizeOfProcess 0x00001000
pcb_t pcb[numberOfProcesses]; channel channels[numberOfPhilosophers]; int processes = 0; int executing = 0;

//Declare the user programs and stack pointer
extern void     main_P3();
extern void     main_P4();
extern void     main_P5();
extern void     main_P6();
extern void     main_P7();
extern void     main_manager();
extern void     main_philosopher();
extern uint32_t tos_p;

extern void     main_console();
extern uint32_t tos_console;

//Keep track of the current stack pointer for the processes
uint32_t currentTos = (uint32_t) &tos_p;
//Keep track of the three top of stack stack pointers for the programs
uint32_t tosPointers [ numberOfProcesses ];
//Initialise the three user programs
uint32_t userPrograms [ numberOfProcesses ] = {/*(uint32_t) (&main_P3), (uint32_t) (&main_P4)/*
,(uint32_t) (&main_P6), (uint32_t) (&main_P7), (uint32_t) (&main_P5)*/ (uint32_t) (&main_manager), (uint32_t) (&main_philosopher)};

uint32_t allocateStack(int i) {
  uint32_t tos = currentTos;
  currentTos -= sizeOfProcess;
  tosPointers[ i ] = tos;
  return tos;
}

void initialiseProcess(ctx_t* ctx, int i) {
  memset( &pcb[ i ], 0, sizeof( pcb_t ) );
  pcb[ i ].pid      = i;
  pcb[ i ].status   = STATUS_READY;
  pcb[ i ].ctx.cpsr = 0x50;
  pcb[ i ].ctx.pc   = ( uint32_t )( userPrograms[ i-1 ] );
  pcb[ i ].ctx.sp   = ( uint32_t )( allocateStack (i) );
  pcb[ i ].pr       = i+2;
}

void executeNext (ctx_t* ctx, int next){
  memcpy( &pcb[ executing].ctx, ctx, sizeof( ctx_t ) ); // preserve P_i
  if (pcb[ executing ].status != STATUS_TERMINATED){
    pcb[executing].status = STATUS_READY;               // update   P_i status
  }
  memcpy( ctx, &pcb[ next].ctx, sizeof( ctx_t ) );      // restore  P_i+1
  pcb[ next ].status = STATUS_EXECUTING;                // update   P_i+1 status
  executing = next;
}

void roundRobinScheduler (ctx_t* ctx) {

  int nextProcess = (executing+1)%(processes+1);

  if (nextProcess == 0){
    nextProcess += 1; //because console is always 0.
  }
  executeNext(ctx, nextProcess);

}

void decrementPriority() {
  for (int i=0; i<processes+1; i++){
    if (i == executing) {
      pcb[ i ].pr -= 1;
    }
  }
}

void incrementPriority() {
  for (int i=0; i<processes+1; i++){
    if (i != executing) {
      pcb[ i ].pr += 1;
    }
  }
}

int highestPriority(ctx_t* ctx){
  int highest = 0;
  int processId = 0;

  /*Loop through all the processes in the process table to find the
  one with maximum priority*/
  for (int i=0; i<processes+1; i++){
    if (pcb[i].pr > highest && pcb[i].status != STATUS_TERMINATED){
      highest = pcb[i].pr;
      processId = i;
    }
  }

  /*Decrement the priority of the current process as it ages
  while the other processes are waiting in the queue.*/
  decrementPriority();
  /*Increment the priorities of the processes in the ready queue
  while the current process is executing.*/
  incrementPriority();

  return processId;
}

void priorityScheduler (ctx_t* ctx) {

  //Find the process with highest priority.
  int highest = highestPriority(ctx);
  //Execute it.
  executeNext(ctx, highest);
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

//   for (int i=1; i < processes+1; i++){
//     initialiseProcess(ctx, i);
//   }

/* Initialise the console to be the first in the pcb table */
  memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );
  pcb[ 0 ].pid      = 0;
  pcb[ 0 ].status   = STATUS_READY;
  pcb[ 0 ].ctx.cpsr = 0x50;
  pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_console );
  pcb[ 0 ].pr       = 0;

  tosPointers[0] = (uint32_t)&tos_console;

/* Once the PCBs are initialised, we (arbitrarily) select one to be
* restored (i.e., executed) when the function then returns.
*/

  memcpy( ctx, &pcb[ 0 ].ctx, sizeof( ctx_t ) );
  pcb[ 0 ].status = STATUS_EXECUTING;

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {

  uint32_t id = GICC0->IAR;


  if( id == GIC_SOURCE_TIMER0 ) {
//     roundRobinScheduler(ctx);
    priorityScheduler(ctx);
    TIMER0->Timer1IntClr = 0x01;
  }

  GICC0->EOIR = id;

  return;
}

void put_s( PL011_t *stream, char inpstr[], int n){
    for( int i = 0; i < n; i++ ) {
        PL011_putc( stream, inpstr[i], true );
    }
}

void put_i(PL011_t *stream, int r){

    if (r < 10){
        PL011_putc(stream, r + '0', true);
    }
    else{
        PL011_putc(UART0, '1', true);
        PL011_putc(UART0, (r-10) + '0', true);
    }
}

void hilevel_handler_svc(ctx_t* ctx, uint32_t id) {
/* Based on the identified encoded as an immediate operand in the
* instruction,
*
* - read  the arguments from preserved usr mode registers,
* - perform whatever is appropriate for this system call,
* - write any return value back to preserved usr mode registers.
*/

  switch( id ) {
    case 0x00 : { // 0x00 => yield()
//       roundRobinScheduler(ctx);
      priorityScheduler(ctx);
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

    case 0x02 : { // 0x02 => read(fd, x, n)
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
      //Increment the number of processes;
      processes += 1;
      //Creates new memory
      memset(&pcb[processes], 0, sizeof(pcb_t));
      //Copy context
      memcpy(&pcb[processes].ctx, ctx, sizeof(ctx_t));
      //Update the id and the status of the newly created process
      pcb[processes].pid = processes;
      pcb[processes].status = pcb[executing].status;
      pcb[processes].ctx.cpsr = ctx->cpsr;
      //Allocate stack for the new process and make its stack pointer point to the top of the newly allocated stack.
      tosPointers[processes] = (uint32_t) allocateStack(processes);
      //Update stack pointer of new program so that it starts from where the parent finished.
      uint32_t offset = (uint32_t) tosPointers[executing] - ctx->sp;
      //Copy the content that was executed by the parent before forking.
      memcpy((void*)(tosPointers[processes] -  offset),(void*)(tosPointers[executing] - offset), offset);
      pcb[processes].ctx.sp = (uint32_t) tosPointers[processes] - offset;

      //Set the priority of the new process.
      pcb[processes].pr = (processes + 2);

      //return value of child is 0.
      pcb[processes].ctx.gpr[0] = 0;
      //return value of parent is child's pid;
      ctx->gpr[0] = processes;

      break;
    }

    case 0x04 : {
      pcb[executing].status = STATUS_TERMINATED;
      priorityScheduler(ctx);
      break;
    }

    case 0x05 : {
      void* execId = (void*) ctx->gpr[0];
      ctx->sp = tosPointers[executing];
      ctx->pc = (uint32_t) execId;
      break;
    }

    case 0x06 : { //0x04 => kill(pid, x)
      int currentID = ctx->gpr[0];
      pcb[currentID].status = STATUS_TERMINATED;
      break;
    }

    case 0x08 : { //0x08 => chwrite(fd, x, block, id)
      int   fd  = ( int   )( ctx->gpr[ 0 ] );
      int    x  = ( int   )( ctx->gpr[ 1 ] );
      int   id  = ( int   )( ctx->gpr[ 2 ] );

      if (channels[fd].lastWrote == id) {
//           write(STDOUT_FILENO, "Cannot write\n", 13);
          ctx->gpr[0] = -1;
      }
      else  {
          channels[fd].lastWrote = id;
          channels[fd].storage   =  x;
          ctx->gpr[0]            =  0;
      }

      break;
    }

    case 0x09 : { //0x09 => chread(fd, block)
      int   fd  = ( int   )( ctx->gpr[ 0 ] );
      int   id  = ( int   )( ctx->gpr[ 1 ] );
      int x = 0;

      if (channels[fd].lastWrote == id) {
//           write(STDOUT_FILENO, "Cannot read\n", 12);
          x = 0;
      }
      else {
          x = channels[fd].storage;
          channels[fd].storage = 0;
      }

      ctx->gpr[0] = x;
      break;
    }

    case 0x10 : { //0x10 => pipe(id, block, pid_a, pid_b)
      int     fd  = ( int )( ctx->gpr[ 0 ] );
      int  block  = ( int )( ctx->gpr[ 1 ] );
      int  pid_a  = ( int )( ctx->gpr[ 2 ] );
      int  pid_b  = ( int )( ctx->gpr[ 3 ] );

      channels[fd].pid_a     = pid_a;
      channels[fd].pid_b     = pid_b;
      channels[fd].storage   = 0;
      channels[fd].channelID = fd+1;
      channels[fd].lastWrote = -1;

      ctx->gpr[0] = fd;
      break;
    }

    case 0x11 : { //0x11 => open(fd, id)
      int fd  = ( int )( ctx->gpr[ 0 ] );
      int id  = ( int )( ctx->gpr[ 1 ] );
      if (channels[fd].pid_a == 0) {
          channels[fd].pid_a = id;
      }
      else {
          channels[fd].pid_b = id;
      }

      ctx->gpr[ 0 ] = fd;
      break;
    }

    case 0x12 : { //0x12 => close(fd, id)
      int fd  = ( int )( ctx->gpr[ 0 ] );
      int id  = ( int )( ctx->gpr[ 1 ] );

      if (channels[fd].pid_a == id){
         channels[fd].pid_a   = 0;
      }
      else {
         channels[fd].pid_b   = 0;
      }

      break;
    }

    case 0x13 : { //0x13 => id()

      ctx->gpr[0] = executing;
      break;
    }
    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

return;

}
