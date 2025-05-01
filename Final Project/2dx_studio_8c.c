/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include "interrupts.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MOTOR_DELAY							44000

#define MAXRETRIES              5           // number of receive attempts before giving up

int rotate = 0;
int dataTog = 0;
int status = 0;
int PWM = 0; // Set to 1 for PWM demo
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;
		GPIO_PORTG_DIR_R |= 0x02;// make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x03;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x03;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x03;                                     // disable analog functionality on PN
	//GPIO_PORTG_PUR_R |= 0x01;

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
		
}


void RotateCW() {
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait(MOTOR_DELAY);
}

void RotateCCW() {
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait(MOTOR_DELAY);
}

void PortH_Init() { //Pins 0-3: Output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}	
	GPIO_PORTH_DIR_R |= 0x0F;        							
  GPIO_PORTH_AFSEL_R &= ~0x0F;     								
  GPIO_PORTH_DEN_R |= 0x0F;        								
  GPIO_PORTH_AMSEL_R &= ~0x0F;
	return;
}

void GPIOJ_IRQHandler(void){
    
    // Check for an interrupt on PJ0 (bit 0)
    if(GPIO_PORTJ_MIS_R & 0x01){
        // Toggle 'data'
        if(dataTog == 0){
						GPIO_PORTF_DATA_R |= 0b00010000;
            dataTog = 1;
        } else {
						GPIO_PORTF_DATA_R &= 0b11101111;
            dataTog = 0;
        }
        // Clear the PJ0 interrupt flag
        GPIO_PORTJ_ICR_R = 0x01;
    }
    
    // Check for an interrupt on PJ1 (bit 1)
    if(GPIO_PORTJ_MIS_R & 0x02){
        // Toggle 'rotate'
        if(rotate == 0){
            rotate = 1;
						FlashAllLEDs();
        } else {
            rotate = 0;
        }
        // Clear the PJ1 interrupt flag
        GPIO_PORTJ_ICR_R = 0x02;
    }
}

	void TIMER3A_IRQHandler(void){ 
	FlashAllLEDs();
	SysTick_Wait10ms(10);
	TIMER3_ICR_R = 0x1;	
	// Execute user task -- we simply flash LED
}
//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint16_t j;
	uint16_t i;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortG_Init();
	
	VL53L1X_XSHUT();
	SysTick_Wait10ms(20);
	
	interrupts_Init();
	PortH_Init();
		 	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);
	


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// 1 Wait for device booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
    status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
    //status = VL53L1X_SetTimingBudgetInMs(dev, 200); /*  in ms possible values [20, 50, 100, 200, 500] */
		//status = VL53L1X_SetInterMeasurementInMs(dev, 400); /* in ms, IM must be > = TB */
		status = VL53L1X_StartRanging(dev);
		
 while(1) {
        // If the button signaled a rotation start (rotate==1)
        if(rotate == 1) {
            uint32_t totalSteps = 0;  // Count how many CW steps are executed
            
            // Rotate clockwise until either the desired full rotation or button press (rotate==0) occurs
            for(j = 0; j < 64; j++) {
                for(i = 0; i < 8; i++) {
									  RotateCW();
                    // Check if button was pressed during rotation (flag cleared to 0)
                    if(rotate == 0) {
                        break;
                    }
                    totalSteps++;  // count this CW step
                }
							
                // Break out of outer loop if stop signal received
                if(rotate == 0) {
                    break;
                }
                
                // Sensor reading loop after each block of steps
                if(dataTog){
								GPIO_PORTN_DATA_R ^= 0b00000001; 
								while(dataReady == 0) {
                    status = VL53L1X_CheckForDataReady(dev, &dataReady);
                    VL53L1_WaitMs(dev, 5);
                }
                dataReady = 0;
                
                // Read sensor data (if needed)
                status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
                status = VL53L1X_GetDistance(dev, &Distance);
                status = VL53L1X_GetSignalRate(dev, &SignalRate);
                status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
                status = VL53L1X_GetSpadNb(dev, &SpadNum);
								GPIO_PORTN_DATA_R ^= 0b00000001;
								
                status = VL53L1X_ClearInterrupt(dev);
                
                sprintf(printf_buffer, "%u, %u, %u, %u, %u\r\n", 
                        RangeStatus, Distance, SignalRate, AmbientRate, SpadNum);
								FlashLED1(1);
                UART_printf(printf_buffer);
                SysTick_Wait10ms(50);
            }
					}
					
            
            // Whether rotation completed fully or was interrupted, rotate back to home.
            for(i = 0; i < totalSteps; i++) {
                RotateCCW();
            }
            
            // Reset the flag after completing the reverse rotation.
            rotate = 0;
        } else if (PWM){
					// 10 Hz
					GPIO_PORTF_DATA_R |= 0b00000001;
					GPIO_PORTG_DATA_R |= 0b00000010;
					SysTick_Wait(1100000);
					GPIO_PORTG_DATA_R &= ~0b00000010;
					GPIO_PORTF_DATA_R &= ~0b00000001;
					SysTick_Wait(1100000);	
				} else {
					WaitForInt();
				}
    }

}

