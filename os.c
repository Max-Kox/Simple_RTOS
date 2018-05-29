// os.c - implementation of priority/blocking Real-Time Operating System
// Runs on TM4C123
// A priority/blocking real-time operating system

#include <stdint.h>
#include "os.h"
#include "CortexM.h"
#include "BSP.h"
#include "tm4c123gh6pm.h"

void Sleep_Counter_decrement(void);
// function definitions in osasm.s
void StartOS(void);

#define NUMTHREADS  8        // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread
struct tcb
{
	int32_t *sp;				// pointer to stack (valid for threads not running
	struct tcb *next;		// linked-list pointer
	int32_t *blockPt;		// nonzero if blocked on this semaphore, and points to blocking thread
	int32_t sleep;			// nonzero if this thread is sleeping
	int8_t priority;		// 0 is the highest, 254 - is the lowest
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];


// ******** OS_Init ************
// Initializes operating system, disable interrupts
// Initializes OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initializes OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void)
{
	DisableInterrupts();
	BSP_Clock_InitFastest();						// sets processor clock to fastest speed
	BSP_PeriodicTask_Init(&Sleep_Counter_decrement, 1000, 2);		// decrements sleep counters of sleeping threads
}

void SetInitialStack(int i)
{
	tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
	Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
	Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
	Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
	Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
	Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
	Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
	Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
	Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
	Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
	Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
	Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
	Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
	Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
	Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
	Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

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
                  void(*thread7)(void), uint32_t p7)
{
	int32_t status;
	status = StartCritical();
	for(int i=0; i<8; i++)
	{
		tcbs[i].next = &tcbs[(i+1)%8];
		tcbs[i].blockPt = 0;
		tcbs[i].sleep = 0;
	}
	SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(thread0); // PC
	SetInitialStack(1); Stacks[1][STACKSIZE-2] = (int32_t)(thread1); // PC
	SetInitialStack(2); Stacks[2][STACKSIZE-2] = (int32_t)(thread2); // PC
	SetInitialStack(3); Stacks[3][STACKSIZE-2] = (int32_t)(thread3); // PC
	SetInitialStack(4); Stacks[4][STACKSIZE-2] = (int32_t)(thread4); // PC
	SetInitialStack(5); Stacks[5][STACKSIZE-2] = (int32_t)(thread5); // PC
	SetInitialStack(6); Stacks[6][STACKSIZE-2] = (int32_t)(thread6); // PC
	SetInitialStack(7); Stacks[7][STACKSIZE-2] = (int32_t)(thread7); // PC
	tcbs[0].priority = p0;		//setting the priority of the task0
	tcbs[1].priority = p1;		//setting the priority of the task1
	tcbs[2].priority = p2;		//setting the priority of the task2
	tcbs[3].priority = p3;		//setting the priority of the task3
	tcbs[4].priority = p4;		//setting the priority of the task4
	tcbs[5].priority = p5;		//setting the priority of the task5
	tcbs[6].priority = p6;		//setting the priority of the task6
	tcbs[7].priority = p7;		//setting the priority of the task7
	RunPt = &tcbs[0];					// thread 0 will run first
	EndCritical(status);
	return 1;               // successful
}

//******** OS_Launch ***************
// Starts the scheduler, enables interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice)
{
	STCTRL = 0;                  // disable SysTick during setup
	STCURRENT = 0;               // any write to current clears it
	SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
	STRELOAD = theTimeSlice - 1; // reload value
	STCTRL = 0x00000007;         // enable, core clock and interrupt arm
	StartOS();                   // start on the first task
}
// runs every timeslice
void Scheduler(void)       // every time slice
{
	tcbType *pt;
	tcbType *theOne_pt;
	uint8_t max_priority;
	pt = RunPt;
	max_priority = 255;
	do
	{
		pt = pt->next;
		if( (pt->blockPt == 0) && (pt->sleep == 0) && (pt->priority < max_priority) )
		{
			theOne_pt = pt;
			max_priority = pt->priority;
		}
	}while(pt != RunPt);
	RunPt = theOne_pt;
}

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void)
{
	STCURRENT = 0;        // any write to current clears it
	INTCTRL = 0x04000000; // trigger SysTick
// next thread gets a full time slice
}

// ******** OS_Sleep ************
// places this thread into a sleep state
// input:  number of msec to sleep
// output: none
void OS_Sleep(uint32_t sleepTime)
{
	RunPt->sleep = sleepTime;
	OS_Suspend();
}

void Sleep_Counter_decrement(void)
{
	for(int i=0; i<8; i++)
	{
		if(tcbs[i].sleep)
		{
			tcbs[i].sleep--;
		}
	}
}

// ******** OS_InitSemaphore ************
// Initializes counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value)
{
	DisableInterrupts();
	*semaPt = value;
	EnableInterrupts();
}

// ******** OS_Wait ************
// Decrements semaphore and block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt)
{
	DisableInterrupts();
	(*semaPt)--;
	if((*semaPt) < 0)
	{
		RunPt->blockPt = semaPt;
		EnableInterrupts();
		OS_Suspend();
	}
	EnableInterrupts();
}

// ******** OS_Signal ************
// Increments semaphore
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt)
{
	tcbType *pt;
	DisableInterrupts();
	(*semaPt)++;
	if((*semaPt) <= 0)
	{
		pt = RunPt->next;
		while(pt->blockPt != semaPt)
		{
			pt = pt->next;
		}
		pt->blockPt = 0;
	}
	EnableInterrupts();
}

#define FSIZE 10    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initializes FIFO.  The "put" and "get" indices initially
// are equal, which means that the FIFO is empty.  Also
// initializes semaphores to track properties of the FIFO.
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void)
{
	PutI = GetI = 0;
	OS_InitSemaphore(&CurrentSize, 0);
	LostData = 0 ;
}

// ******** OS_FIFO_Put ************
// Puts an element in the FIFO.
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data)
{
	if(CurrentSize == FSIZE)	// FIFO is full
	{
		LostData++;
		return -1;
	}
	else
	{
		Fifo[PutI] = data;
		PutI = (PutI + 1) % FSIZE;
		OS_Signal(&CurrentSize);
		return 0;				// success
	}
}

// ******** OS_FIFO_Get ************
// Get an element from the FIFO.
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void)
{
	uint32_t data;
	OS_Wait(&CurrentSize);
	data = Fifo[GetI];
	GetI = (GetI + 1) % FSIZE;
	return data;
}
// *****periodic events****************
int32_t *PeriodicSemaphore0;
uint32_t Period0; // time between signals
int32_t *PeriodicSemaphore1;
uint32_t Period1; // time between signals
void RealTimeEvents(void)
{
	int flag=0;
	static int32_t realCount = -10; // lets all the threads execute once
	realCount++;
	if(realCount >= 0)
	{
		if((realCount%Period0)==0)
		{
			OS_Signal(PeriodicSemaphore0);
			flag = 1;
		}
		if((realCount%Period1)==0)
		{
			OS_Signal(PeriodicSemaphore1);
			flag=1;
		}
		if(flag)
		{
			OS_Suspend();
		}
	}
}
// ******** OS_PeriodTrigger0_Init ************
// Initializes periodic timer interrupt to signal
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest priority)
// Outputs: none
void OS_PeriodTrigger0_Init(int32_t *semaPt, uint32_t period)
{
	PeriodicSemaphore0 = semaPt;
	Period0 = period;
	BSP_PeriodicTask_InitC(&RealTimeEvents,1000,0);
}
// ******** OS_PeriodTrigger1_Init ************
// Initialize periodic timer interrupt to signal
// Inputs:  semaphore to signal
//          period in ms
// priority level at 0 (highest  priority)
// Outputs: none
void OS_PeriodTrigger1_Init(int32_t *semaPt, uint32_t period)
{
	PeriodicSemaphore1 = semaPt;
	Period1 = period;
	BSP_PeriodicTask_InitC(&RealTimeEvents,1000,0);
}

//****edge-triggered event************
int32_t *edgeSemaphorePt;
// ******** OS_EdgeTrigger_Init ************
// Initializes button1, PD6, to signal on a falling edge interrupt
// Inputs:  semaphore to signal
//          priority
// Outputs: none
void OS_EdgeTrigger_Init(int32_t *semaPt, uint8_t priority)
{
	SYSCTL_RCGCGPIO_R |= 0x08;				// 1) activates clock for Port D
	edgeSemaphorePt = semaPt;					// allows time for clock to stabilize
																		// 2) no need to unlock PD6
	GPIO_PORTD_AMSEL_R &= ~0x40;			// 3) disables analog on PD6
	GPIO_PORTD_PCTL_R &= ~0x0F000000;	// 4) configures PD6 as GPIO
	GPIO_PORTD_DIR_R &= ~0x40;				// 5) makes PD6 input
	GPIO_PORTD_AFSEL_R &= ~0x40;			// 6) disables alt funct on PD6
	GPIO_PORTD_PUR_R &= ~0x40;				// disables pull-up on PD6
	GPIO_PORTD_DEN_R |= 0x40;					// 7) enables digital I/O on PD6
	GPIO_PORTD_IS_R &= ~0x40;					// (d) PD6 is edge-sensitive
	GPIO_PORTD_IBE_R &= ~0x40;    		//     PD6 is not both edges
	GPIO_PORTD_IEV_R &= ~0x40;      	//   	 PD6 falling edge event 
	GPIO_PORTD_ICR_R = 0x40;					// (e) clears flags
	GPIO_PORTD_IM_R |= 0x40;					// (f) arms interrupt on PD6
	NVIC_PRI0_R = (NVIC_PRI0_R&0x00FFFFFF)|(((uint32_t)priority)<<29);	// priority on Port D edge trigger is NVIC_PRI0_R	31 � 29
	NVIC_EN0_R |= 0x00000008;					// enable is bit 3 in NVIC_EN0_R
}

// ******** OS_EdgeTrigger_Restart ************
// restarts button1 to signal on a falling edge interrupt
// rearm interrupt
// Inputs:  none
// Outputs: none
void OS_EdgeTrigger_Restart(void)
{
	GPIO_PORTD_ICR_R = 0x40;			// clears flag6
	GPIO_PORTD_IM_R |= 0x40;			// rearms interrupt 3 in NVIC
}
void GPIOPortD_Handler(void)
{
	GPIO_PORTD_ICR_R = 0x40;			// step 1 acknowledges by clearing flag
	OS_Signal(edgeSemaphorePt);		// step 2 signals semaphore (no need to run scheduler)
	GPIO_PORTD_IM_R &= ~0x40;			// step 3 disarms interrupt to prevent bouncing to create multiple signals
}

