// os.h - interface to priority/blocking Real-Time Operating System
// Runs on TM4C123
// A priority/blocking real-time operating system

#ifndef __OS_H
#define __OS_H  1


// ******** OS_Init ************
// Initializes operating system, disable interrupts
// Initializes OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initializes OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void);

//******** OS_AddThreads ***************
// Adds eight main threads to the scheduler
// Inputs: function pointers to eight void/void main threads
//         priorites for each main thread (0 highest)
// Outputs: 1 if successful
// This function is only called once, after OS_Init and before OS_Launch
int OS_AddThreads(void(*thread0)(void), uint32_t p0,
                  void(*thread1)(void), uint32_t p1,
                  void(*thread2)(void), uint32_t p2,
                  void(*thread3)(void), uint32_t p3,
                  void(*thread4)(void), uint32_t p4,
                  void(*thread5)(void), uint32_t p5,
                  void(*thread6)(void), uint32_t p6,
                  void(*thread7)(void), uint32_t p7);


//******** OS_Launch ***************
// Starts the scheduler, enables interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice);

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void);

// ******** OS_Sleep ************
// places this thread into a sleep state
// input:  number of msec to sleep
// output: none
void OS_Sleep(uint32_t sleepTime);

// ******** OS_InitSemaphore ************
// Initializes counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value);

// ******** OS_Wait ************
// Decrements semaphore and block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt);

/// ******** OS_Signal ************
// Increments semaphore
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt);

// ******** OS_FIFO_Init ************
// Initializes FIFO.  The "put" and "get" indices initially
// are equal, which means that the FIFO is empty.  Also
// initializes semaphores to track properties of the FIFO.
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void);

// ******** OS_FIFO_Put ************
// Puts an element in the FIFO.
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data);

// ******** OS_FIFO_Get ************
// Get an element from the FIFO.
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void);

// ******** OS_PeriodTrigger0_Init ************
// Initializes periodic timer interrupt to signal
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest priority)
// Outputs: none
void OS_PeriodTrigger0_Init(int32_t *semaPt, uint32_t period);

// ******** OS_PeriodTrigger1_Init ************
// Initialize periodic timer interrupt to signal
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest  priority)
// Outputs: none
void OS_PeriodTrigger1_Init(int32_t *semaPt, uint32_t period);

// ******** OS_EdgeTrigger_Init ************
// Initializes button1, PD6, to signal on a falling edge interrupt
// Inputs:  semaphore to signal
//          priority
// Outputs: none
void OS_EdgeTrigger_Init(int32_t *semaPt, uint8_t priority);

// ******** OS_EdgeTrigger_Restart ************
// restarts button1 to signal on a falling edge interrupt
// rearm interrupt
// Inputs:  none
// Outputs: none
void OS_EdgeTrigger_Restart(void);

#endif
