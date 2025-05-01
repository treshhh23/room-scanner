#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "interrupts.h"

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

void Timer3_Init(void){

	uint32_t period =  	512000;						// 32-bit value in 1us increments (128000 for motor)
	
	// Step 1: Activate timer
	SYSCTL_RCGCTIMER_R = 0x08;						// (Step 1)Activate timer 
	SysTick_Wait10ms(1);							// Wait for the timer module to turn on
	
	
	// Step 2: Arm and Configure Timer Module
	TIMER3_CTL_R = 0x0;							// (Step 2-1) Disable Timer3 during setup (Timer stops counting)
	TIMER3_CFG_R = 0x0;								// (Step 2-2) Configure for 32-bit timer mode   
	TIMER3_TAMR_R = 0x2;							// (Step 2-3) Configure for periodic mode   
	TIMER3_TAPR_R = 0x0;							// (Step 2-4) Set prescale value to 0; i.e. Timer3 works with Maximum Freq = bus clock freq (22MHz)  
	TIMER3_TAILR_R = (period*22)-1; 	// (Step 2-5) Reload value (we multiply the period by 22 to match the units of 1 us)  
	TIMER3_ICR_R = 0x1;				// (Step 2-6) Acknowledge the timeout interrupt (Clear timeout flag of Timer3)
	TIMER3_IMR_R = 0x1;						// (Step 2-7) Arm timeout interrupt   
	
	
	// Step 3: Enable Interrupt at Processor side
	NVIC_EN1_R = 0x00000008; 									// Enable IRQ 35 in NVIC 
	NVIC_PRI8_R = 0x40000000;							// Set Interrupt Priority to 2 
														
	// Step 4: Enable the Timer to start counting
	//TIMER3_CTL_R = 0x1;							Enable Timer3
} 

void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//? Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//??Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor on PJ1
}


// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){

		GPIO_PORTJ_IS_R = 0;    						// (Step 1) PJ1 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;    						//     			PJ1 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0;  						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x03;       					// 					Clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x03;       					// 					Arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;       					// (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000;							// (Step 4) Set interrupt priority to 5															// (Step 3) Enable Global Interrupt. lets go!
}

void interrupts_Init(void){
	PortJ_Init();
	Timer3_Init();
	PortJ_Interrupt_Init();
	EnableInt();	
}

