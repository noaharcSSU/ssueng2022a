//MainProjectProgramV3p1

//Next tasks:
//Get EEProm, Timers and interrupts working - outlines & subfunctions untested
//Power Control needs fixing
//Flashes RB5 that discharges Supercap 2

#pragma config FOSC = INTIO67   //internal oscillator block, port function on RA6 and RA7
#pragma config BORV = 27, PWRT = ON //brown-out-Reset volt = 2.7V
#pragma config WDTEN = OFF  //watchdog time disabled
#pragma config PBADEN = OFF //PORT A/D disabled
#pragma config HFOFST = OFF, MCLRE = OFF // fast start-up and MCLR pin disabled
//#include <pic18f45k20.h>
#include <stdio.h>
#include <xc.h>

#define ldata PORTD 	 //lcd data pins on PORTD
#define rs PORTBbits.RB0 //rs os lcd on PORTB0
#define en PORTBbits.RB1	//en of lcd on PORTB1
#define LCDdelay 1

#define red RC4   //debug LED
#define blue RC6	//debug LED
#define green RC5   //debug LED


#define VDD 3.058   //using chip's VDD as reference for A/D converters

#define Q1SolarRun RA4
#define Q2BuckRun RA6
#define Q3BattCharge RA7

#define SCapDrainTime 1
#define SC1Drain_Q10 RB4
#define SC2Drain_Q20 RB5
#define SC3Drain_Q30 RC0
#define SC4Drain_Q40 RC1

#define sigfigs 4
#define numADvalues 9
#define loadThresh 5
#define max_charge_volt 4
// EEPROM----------------------
//#include <stdint.h>
//#include <stdbool.h>
//#include "conbits.h"
//#include "uart_layer.h"

uint8_t data = 0;
//------------------------------
#define delayConst 0x02
long ADreadValue [numADvalues] = {1234,5678,9101,110725}; //[0] is solar in Volt. [1,2,3,4] are supercap nodes,
//{1234,5678,9876,5432,1111,2222,3333,4444}
int ReadoutDigits[numADvalues]; //keep track of number of digits sprintf returns
float SupercapVolt[4]={7,17,27,37};
double max_cap_delta = 1.325; //cap voltage deviation from average of 4 supercaps
char V_string[sigfigs][4]; //groups of 4 numbers on screen

void MSDelay(int);
void CalcSupercapVolts(void);
void PowerControl(void);

void lcd_init (void);
void lcdcmd(unsigned char);
void lcddata (unsigned char);
void pushToDisplay (void);
void setup (void); //I don't know why it won't run setup if you don't treat
//it like some second-rate subroutine
void getADvalues(void);
//void Eeprom_read(void);
//void Eeprom_write(void);

void __interrupt (high_priority) Read_ADCs_ISR(void){
    
	if (INTCONbits.TMR0IF == 1)
	MSDelay (100);  
	INTCONbits.TMR0IF = 0; // clears timer 0 interrupt bit
	unsigned char x;
	for (x=0; x<10; x++)   //loop255 times
	{
    	PORTBbits.RB4 = 1;  //setRB1 - pin
    	MSDelay(10);
    	PORTBbits.RB4 = 0; //clear RB1 - pin
    	MSDelay(10);
   	 
	}
	PORTBbits.RB4 = 0;
}
//ISR for blinking LED with button push

void __interrupt(low_priority) Blink_LED_ISR(void)   

{
	MSDelay(100);
	INTCON3bits.INT2IF = 0; //clearing INT2IF flag
	unsigned char z;
	for (z=0; z<10; z++)   //loop255 times
	{
    	PORTBbits.RB3 = 1;  //setRB1 - pin
    	MSDelay(100);
    	PORTBbits.RB3 = 0; //clear RB1 - pin
    	MSDelay(10);
      	 
	}
	PORTBbits.RB3 = 0;
}


void setup (){
	ANSEL = 0xFF; // page 128 129
	ANSELH = 0b00000010; //bit[1] is A/D input AN9
    OSCCON = 0b00000011; ////page28.       bits[6,5,4] are clock divider  
    LATB = 0x00;
    LATC = 0x00;
    LATD = 0x00;  	//setting ports B and D as outputs
    LATA = 0b00101111;
     				   //A to D setup in the actual subroutine
 	TRISCbits.TRISC4 = 0;       	//input from button for INT2
 	TRISBbits.TRISB1 = 1;
 	TRISBbits.TRISB4 = 0;
 	TRISBbits.TRISB3 = 0;
	 
 	//Interrupt Configuration
 	RCONbits.IPEN = 1; 	//enable priority
 	INTCONbits.GIEH = 1;   //Global high priority enable
 	INTCONbits.GIEL = 1;   //Global peripheral enable
 	INTCONbits.TMR0IE = 1; //Enables timer 0 interrupt
 	INTCON2bits.TMR0IP = 1;	//sets timer 0 interrupt as high priority
 	T0CONbits.T08BIT = 0;  	// sets timer 0 to 8 bit
 	T0CONbits.T0CS = 0;    	//timer 0 uses internal clock
 	T0CONbits.PSA = 0;     	//timer 0 uses prescaler
 	T0CONbits.T0PS2 = 1;   	//sets bit 2 for prescaler selection
 	T0CONbits.T0PS1 = 0;   	//sets bit 1 for prescaler selection
 	T0CONbits.T0PS0 = 0;   	//sets bit 0 for prescaler selection
 	INTCONbits.INT0IE =1;
 	INTCON3bits.INT2IE = 1;	//enable INT2
 	INTCON3bits.INT2IP = 0;	//define INT2 as low priority
 	//INTCON2bits.INTEDG2 = 1;   //trigger INT2 on rising edge
 	PORTBbits.RB2 = 0;     	//clear RB2
	 
 	T0CONbits.TMR0ON = 1;
	 
}
	//PORTAbits.Q1SolarRun = 0;//debug
 	//PORTAbits.Q2BuckRun = 0;

void main (void) {
	setup();
	while(1){
    	/*
    	PORTA=0x00;
    	PORTB=0x00;
    	PORTC=0x00;
    	PORTD=0x00;
    	PORTCbits.RC4 = 1;
    	MSDelay(50);
    	PORTCbits.RC4 = 0;
    	MSDelay(50);
    	VDD
    	*/
    	PORTCbits.green = 1;//debug
    	getADvalues();
    	PORTCbits.green = 0;//debug
    	CalcSupercapVolts();
    	PowerControl();
/*
    	PORTCbits.blue = 1;//debug
    	//MSDelay(10);
    	OSCCON = 0b01100011;

    	for(int i=0; i<4; i++){
 		  ReadoutDigits[i] = sprintf (V_string[i], "%ld", ADreadValue[i+1]);
    	}
    	PORTCbits.blue = 0;//debug
*/ 	 
    	lcd_init();
    	pushToDisplay();
    	MSDelay(1000);
    	OSCCON = 0b01000011;
    	PORTCbits.red = 1;//debug
    	CalcSupercapVolts();
    	for(int i=0; i<4; i++){//converting variables to text for display
     	ReadoutDigits[i] = sprintf (V_string[i], "%f", SupercapVolt[i]);
     	//+ReadoutDigits[i] = 6;
    	}
    	PORTCbits.red = 0;//debug
    	//use %g for floats or maybe longs
    	OSCCON = 0b00010011;
    	MSDelay(100);

    	lcd_init();
    	pushToDisplay();

    	MSDelay(1000);
	}
}

void getADvalues(){
	//#define VDD 3.058   //used as scaling factor in routines that use AD values

//Nick's hardware:    
	ADCON0 = 0x01; //power on AD converter .... page 255
	//Noah's hardware AN0 on pin 2 not used, reserved for supercap current:
	//ADCON0 = 0x05; //power on AD converter .... page 255
    //bits [2,3,4,5] point to AD input, Ax
	MSDelay(5);
	ADCON1 = 0x00; //bits 4 and 5 only, points to reference
	ADCON2 = 0b10111111; //bit 7 left justified
               		 //bits [3,4,5] TAD 000 is fast ACQ, 111 is slow ACQ
               		 //bits [0,1,2] ADC clock
	for(int i = 0; i < numADvalues; i++){
  	  ADCON0bits.GO = 1;  //start AD conversion
  	  while(ADCON0bits.GO ==1) {}; //wait for conversion to finish
  	  ADreadValue [i] = (256 * ADRESH) + ADRESL;
  	  ADCON0 = ADCON0 + 4;
    	/*
        if(A/D converter inputs not consecutive){
     	   add extra 4 to ADCON0 to advance to next AD input
     	   to read correct AD port
        }*/
	}
	ADCON0 = 0x00; //turn off A/D conv to save power
}

void CalcSupercapVolts(void){
	float divFactor[4]={57.31,32.54,21.18,15.75};
	SupercapVolt[0] = ((float)ADreadValue[1])/divFactor[0];
	SupercapVolt[1] = ((float)ADreadValue[2])/divFactor[1] - SupercapVolt[0];//
	SupercapVolt[2] = (float)ADreadValue[3]/divFactor[2]-SupercapVolt[1]-SupercapVolt[0];
	SupercapVolt[3] = (float)ADreadValue[4]/divFactor[3]-SupercapVolt[2]-SupercapVolt[1]-SupercapVolt[0];
}

void PowerControl(void){
 /*
#define red RC4   //debug LED
#define blue RC6	//debug LED
#define green RC5   //debug LED

#define Q1SolarRun RA4
#define Q2BuckRun RA6
#define Q3BattCharge RA7

#define SCapDrainTime 1
#define SC1Drain_Q10 RB4
#define SC2Drain_Q20 RB5
#define SC3Drain_Q30 RC0
#define SC4Drain_Q40 RC1
 */

	double solar_volt = ADreadValue[0];


    double SupercapSum = ADreadValue[4]*20 ;
    double SupercapVoltAvg = SupercapSum / 4;


/* 	 if(SupercapVolt[0] - SupercapVoltAvg > max_cap_delta){
     	   PORTBbits.SC1Drain_Q10 = 1;
     	   MSDelay(10);
     	   PORTBbits.SC1Drain_Q10 = 0;
        }
	SupercapVolt[1] = (ADreadValue[2]*10) - (ADreadValue[1]*5);
      if(SupercapVolt[1] - SupercapVoltAvg > max_cap_delta){
     	   PORTBbits.SC2Drain_Q20 = 1;
     	   MSDelay(10);
     	   PORTBbits.SC2Drain_Q20 = 0;
        }
	SupercapVolt[2] = (ADreadValue[3]*15) - (ADreadValue[2]*10);
      if(SupercapVolt[2] - SupercapVoltAvg > max_cap_delta){
     	   PORTCbits.SC3Drain_Q30 = 1;
     	   MSDelay(10);
     	   PORTCbits.SC3Drain_Q30 = 0;
        }
	SupercapVolt[3] = ADreadValue[4]*20 - ADreadValue[3]*15;
      if(SupercapVolt[3] - SupercapVoltAvg > max_cap_delta){
     	   PORTCbits.SC4Drain_Q40 = 1;
     	   MSDelay(10);
     	   PORTCbits.SC4Drain_Q40 = 0;
        }
	// Q1 trigger condition
	if(solar_volt > 10
        	& SupercapVolt[0] < 5
        	& SupercapVolt[1] < 5
        	& SupercapVolt[2] < 5
        	& SupercapVolt[3] < 5
        	& solar_volt > SupercapSum
        	){
    	PORTAbits.Q1SolarRun = 1;
	}
	else{
    	PORTAbits.Q1SolarRun = 0;
	}
	// Q2 trigger condition
	if((SupercapVolt[0] > 5
        	& SupercapVolt[1] > 5
        	& SupercapVolt[2] > 5
        	& SupercapVolt[3] > 5)
        	||
        	(SupercapVolt[0] < 5
        	& SupercapVolt[1] < 5
        	& SupercapVolt[2] < 5
        	& SupercapVolt[3] < 5
        	& ADreadValue[7] > loadThresh)
        	){
            	PORTAbits.Q2BuckRun = 1;
    	}
	else{
    	PORTAbits.Q2BuckRun = 0;
	}
	// Q3 trigger condition
	if(PORTAbits.RA6 == 1 & ADreadValue[7] < max_charge_volt){
    	PORTAbits.RA7 = 1;
	} else{
    	PORTAbits.RA7 = 0;
	}
*/    
}

// EEPROM Read/Write
uint8_t Eeprom_read(uint8_t addr){
	EEADR = addr;
	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.RD = 1;
	while(EECON1bits.RD);
	return EEDATA;
}

void Eeprom_write(uint8_t addr,uint8_t data){
	EEADR = addr;
	EEDATA = data;
	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	INTCONbits.GIEH = 0;
    
	EECON2 = 0x55;
	EECON2 = 0xAA;
    
	EECON1bits.WR = 1;
	while(EECON1bits.WR);
	EECON1bits.WREN = 0;
	INTCONbits.GIEH = 1;
}

void pushToDisplay (void){
	//PORTCbits.blue = 1;//debug
	//lcd_init();
	lcdcmd(0x01);   	//clear LCD
//////Line 1
	lcdcmd (0x80);  //ready on line 1
//first number
	for (int i = 0; i < ReadoutDigits[0]; i++){
  	  lcddata(V_string[0][i]);
   	}
	//lcddata('V');
    lcddata(' ');
//second number
	for (int j = 0; j < ReadoutDigits[1]; j++){
    	lcddata(V_string[1][j]);
    	}
	//lcddata('V');
//////Line 2    
	lcdcmd (0xC0);  //ready on line 1
	for (int k = 0; k < ReadoutDigits[2]; k++){
    	lcddata(V_string[2][k]);
    	}
	//lcddata('V');
    lcddata(' ');
	for (int m = 0; m < ReadoutDigits[3]; m++){
    	lcddata(V_string[3][m]);
    	}
	//lcddata('V');
}


void lcdcmd(unsigned char value)    	//command
{
	ldata = value; 	 //put received argument on PORTD
	rs = 0;
	en = 1;
	MSDelay(LCDdelay);
	en = 0;
	MSDelay(LCDdelay);
}

void lcddata (unsigned char value)   	//data
{
    
	ldata = value;
	rs = 1;
	en = 1;
	MSDelay(LCDdelay);
	en = 0;
	MSDelay(LCDdelay);
}

void lcd_init (void)
{
	//PORTCbits.red = 1;//debug
	en = 0; 		//initialize, clear enable
	lcdcmd(0x38);   	//initialize LCD 2 lines, 5x7 matrix
	lcdcmd(0x01);   	//display on cursor
	lcdcmd(0x00);   	//display on cursor
	lcdcmd(0x06); 	 //increments cursor after each byt
	lcdcmd(0x0F);   	//line 1 position 6
	//PORTCbits.red = 0;//debug  
}

 void MSDelay(int mili)
 {
	 int i, j;
	 for (i=0; i <mili; i++)
     	for(j=0; j<delayConst; j++);    	 
 }



