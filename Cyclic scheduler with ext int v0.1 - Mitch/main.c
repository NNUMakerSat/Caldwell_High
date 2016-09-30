#include <msp430.h>
#include <stdlib.h>
#include <stdbool.h>
#include "driverlib.h"
#include <stdint.h>
#include "SPI_5969.h"
#define BUTTON 0x08    // was 0x02   0x08 = P1.3
#define RAD_TO_HUB_PIN 0x01
#define RAD_TO_HUB 0x01
#define SLAVE 0x01
#define hand_Shake 0xCC
bool hand_ShakePass = false;
uint8_t t = 0x00;
uint8_t i = 0x00;
uint8_t ii = 0x00;
uint8_t counter = 0x00;
uint8_t SPI_SEND;
uint8_t test_counter;
uint8_t test_array[10];
uint8_t SPI_send_flg;

/*
 * main.c
 *
 * This program sets up an interrupt on input P1.3 and waits for it to happen.
 * When the interrupt activates, the ISR sets a flag register "t" and disables
 * interrupts.  In the main, when the flag register is active, it polls
 * input pin 1.3 and waits until it switches state before incrementing a count
 * and re-enabling interrupts.
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    //******************************************************************************
    //
    // Below is the P1.3 interrupt configuration section.
    //
    //******************************************************************************

    PM5CTL0 &= ~LOCKLPM5;   // Disable the GPIO power-on default high-impedance mode ABSOLUTELY REQUIRED
    P1DIR = 0x01;			// Set Port 1 to input (now P1.0 output) was 0x00
    P1REN |= 0x08;			// was 0x02
    P1SEL0 = 0x00;			// Set as GPIO and not secondary function
    P1SEL1 = 0x00;

    P2DIR = BIT4;

    P1IE |= BUTTON;         // P1.3 interrupt enabled
    P1IFG &= ~BUTTON;       // P1.3 IFG cleared
    P1IE |= BUTTON;         // P1.3 interrupt enabled (added 9/3/2016)

    init_SPI(RAD_TO_HUB_PIN, SLAVE);			// sets up SPI (Rad Science Board to Hub pin setup, SLAVE for slave mode)

// Handshake
//    while (!hand_ShakePass){
    P3OUT |= BIT5;
//    write_uint8_SPI (hand_Shake, RAD_TO_HUB);	// SPI handshake with Hub
    P3OUT &= ~BIT5;
//    if (UCB0RXBUF == 0xBB){hand_ShakePass = true;}
//    }



    //******************************************************************************
    //
    // Below is the TIMER1_A3 interrupt configuration section.
    //
    //******************************************************************************

    PMM_unlockLPM5();

    //Start timer in continuous mode sourced by ACLK
    Timer_A_clearTimerInterrupt(TIMER_A1_BASE);

    Timer_A_initContinuousModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
   // param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;       //sets timer clk divider
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = false;
    Timer_A_initContinuousMode(TIMER_A1_BASE, &param);

    Timer_A_startCounter(TIMER_A1_BASE,
                         TIMER_A_CONTINUOUS_MODE
                         );

    //******************************************************************************
    //
    // Below is the P1.3 external interrupt monitoring section.
    //
    //******************************************************************************
    __enable_interrupt();   // intrinsic function
while (1) {
	if (SPI_send_flg){
		P3OUT |= BIT5;
		while (ii < 10){
			write_uint8_SPI (test_array[ii], RAD_TO_HUB);	// SPI send to Hub
			ii++;
		}
		P3OUT &= ~BIT5;
	}
	while ( t==1 ) {
		if (P1IN &= ~BIT3) {  //If P1.3 goes low  //was BIT3
		t = 0;    				//clears interrupt indicator
		counter++;
	    P1IE |= BUTTON;         // P1.3 interrupt enabled
	    P1IFG &= ~BUTTON;       // P1.3 IFG cleared
							}
					}
			}
}

//******************************************************************************
//
//This is the P1.3 external interrupt vector service routine.
//
//******************************************************************************

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
//	P1IE &= ~BUTTON;         // P1.3 interrupt disabled
	P1IE &= ~BIT3;         // P1.3 interrupt disabled
	t = 1;  //indicator to tell us received active interrupt
	return;
}

//******************************************************************************
//
//This is the TIMER1_A3 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A1_VECTOR)))
#endif
void TIMER1_A1_ISR(void)
{
    //Any access, read or write, of the TAIV register automatically resets the
    //highest "pending" interrupt flag
    switch(__even_in_range(TA1IV,14))
    {
    case  0: break;                              //No interrupt
    case  2: break;                              //CCR1 not used
    case  4: break;                              //CCR2 not used
    case  6: break;                              //CCR3 not used
    case  8: break;                              //CCR4 not used
    case 10: break;                              //CCR5 not used
    case 12: break;                              //CCR6 not used
    case 14:
        //Toggle P1.0                    // overflow
 //       GPIO_toggleOutputOnPin(
 //           GPIO_PORT_P1,
 //           GPIO_PIN0
 //           );



//    		write_uint8_SPI (SPI_SEND, RAD_TO_HUB);	// SPI send to Hub
    		test_array[i] = counter;
    		i++;
    		P2OUT ^= BIT4;

    		counter = 0x00;					// reset counter
//    		if (i >= 2){SPI_send_flg = 1;P1IE &= ~BIT3;}
//    	if (test_counter >= 10) {
//   		write_uint8_SPI (0xFF, RAD_TO_HUB);	// SPI send to Hub
//    		test_counter = 0;
//    		P3OUT &= ~BIT5;
//    	}

        break;
    default: break;
    }
}
