/*
 * File:   test_code.c
 * Author: David Cheng
 *
 * Created on December 28, 2022, 6:37 PM
 */
#pragma config FOSC = INTIO67
#pragma config BORV = 30 //brownout voltage thresh
#pragma config PWRT = ON //power-up timer - delays start of execution 64ms
#pragma config WDTEN = OFF //watchdog
#pragma config PBADEN = OFF //a/d conv - could be on for this program, but not needed
#pragma config HFOFST = OFF, MCLRE = OFF
//#include "uart_layer.h"
//#include <pic18f45k20.h>
#include <stdio.h>
#include <xc.h>
//#include <stdint.h>

//uint8_t data = 0; // reference for data recieved 
//bool got_data_bool = false; // tool used to tell if we recieved data 
//uint8_t print_buffer[256] = {0}; // buffer for whatever we want to print out in the UART
int ADvalue[7];
float measured_volt; // where the measured voltage gets saved to
// ADC setup


void MSDelay(unsigned char);
void read_analog(void);

//int VoltPin2;
void main(void) {
    TRISA = 0b11110111; // set AD inputs and outputs on RA4; 11110111 in binary
    TRISC = 0x00; // set PORTC as an output 
    ADCON1 = 0x00; //bits 4 and 5 (+) and (-) refs. 00 is Vdd & Vss
    ADCON2 = 0b10101100; //Right justified, Sample time: 12 TAD, Approximation clock: FOSC/2; 10101100 in binary
    read_analog(); // calls subroutine 
	
	// LED blink test
       if (measured_volt > 0){
            PORTC = 0x55; 
            MSDelay(250); 
            PORTC = 0xAA;
            MSDelay(250);
            
        }
    }
         
// Read ADC         
void read_analog(void){
		
		ADCON0=0x00;
		ADCON0.bits(0) = 1 // enables ADC 
			
		for(int i = 0, i < 8, i++){
			// float measured_volt = 0;
			ADCON0.GO = 1;
			while(ADCON0.GO = 1){}
			measured_volt = ((256 * ADRESH) + ADRESL); // voltage read math
       			measured_volt = (measured_volt/1024) * 3.3;
			ADvalue[i] = measured_volt; 
			ADCON0 = ADCON0+4; // increments AD input AN0-AN6
		}
	ADCON0.bits(0) = 0 // disables ADC
}
    
void MSDelay(unsigned char mili) // delay function
{
    unsigned char i, j;
    for(i=0; i<mili; i++){
        for(j=0; j<100; j++){};
	}
}