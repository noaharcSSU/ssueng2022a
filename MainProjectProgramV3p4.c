//MainProjectProgramV3p4.c - Batt & Sol Volts, timer on display

//Revamped Power Control, added logic to turn Q1 Q2 and Q3 on and off

//Next tasks
//EEProm subfunctions not implemented

#pragma config FOSC = INTIO67   //internal oscillator block, port function on RA6 and RA7
#pragma config BORV = 27, PWRT = ON //brown-out-Reset volt = 2.7V
#pragma config WDTEN = OFF  //watchdog time disabled
#pragma config PBADEN = ON //don't disable. Set to on so pin 36 can be solar input volt sense
#pragma config HFOFST = OFF, MCLRE = OFF // fast start-up and MCLR pin disabled
//#include <pic18f45k20.h>
#include <stdio.h>
#include <xc.h>

#define ldata PORTD      //lcd data pins on PORTD
#define rs PORTBbits.RB0 //rs os lcd on PORTB0
#define en PORTBbits.RB1    //en of lcd on PORTB1
#define LCDdelay 3

#define red PORTCbits.RC4   //debug LED
#define blue PORTCbits.RC6    //debug LED
#define green PORTCbits.RC5   //debug LED

#define VDD 3.058   //using chip's VDD as reference for A/D converters

#define Q1SolarRun PORTAbits.RA4
#define Q2BuckRun PORTAbits.RA6
#define Q3BattCharge PORTAbits.RA7

#define SCapDrainTime 300
#define SC1Drain_Q10 PORTCbits.RC7
#define SC2Drain_Q20 PORTBbits.RB4
#define SC3Drain_Q30 PORTCbits.RC0
#define SC4Drain_Q40 PORTCbits.RC1

#define Batt_max_volt 2.68
#define Batt_rebulk 2.22

#define sigfigs 4
#define numADvalues 10 //AN0 thru AN9, not all used in this implementation
#define loadThresh 5

#define max_supercap_volt 4.8
#define min_supercap_volt 2.22
#define max_cap_delta 1.125

// EEPROM----------------------
//#include <stdint.h>
//#include <stdbool.h>
//#include "conbits.h"
//#include "uart_layer.h"

uint8_t data = 0;
//------------------------------
#define delayConst 0x05
long ADreadValue [numADvalues]; //[0] is solar in Volt. [1,2,3,4] are supercap nodes,
//{1234,5678,9876,5432,1111,2222,3333,4444}
float solar_volt;
float solar_before; //see if solar voltage fell, turn off Q1 and Q2
float SupercapSum;
float SupercapVoltAvg;
float Batt_volt;
float Batt_current;
unsigned char BattChargeMode = 0;
float Load_current;
int ReadoutDigits[numADvalues]; //keep track of number of digits sprintf returns
float SupercapVolt[4];
//double max_cap_delta = 1.325; //cap voltage deviation from average of 4 supercaps
char V_string[6][sigfigs]; //groups of 6 numbers on screen

void MSDelay(int);
void CalcSupercapVolts(void);
void CalcVoltsAndAmps(void);
void PowerControl(void);

void lcd_init (void);
void lcdcmd(unsigned char);
void lcddata (unsigned char);
void pushToDisplay (unsigned char);
//void WakeUp (void);
void setup (void); //I don't know why it won't run setup if you don't treat
//it like some second-rate subroutine
void getADvalues(void);
//void Eeprom_read(void);
//void Eeprom_write(void);
//void __interrupt() WakeUp(void);

/*Every time I enable this it just stays in this loop, I think
void __interrupt (high_priority) Read_ADCs_ISR(void)
{
    if (INTCONbits.TMR0IF == 1)
    //MSDelay (100);  
    INTCONbits.TMR0IF = 0; // clears timer 0 interrupt bit

    //for (int x=0; x<3; x++)   //loop255 times
    //{
   	 if(TMR0L%5 == 0){blue = 1;}  //setRB1 - pin
   	 MSDelay(10);
   	 blue = 0; //clear RB1 - pin
   	 MSDelay(10);
    	if(TMR0L%3 == 0){blue = 1;}
	MSDelay(100);
	blue = 0;

	return;
    
}
*/
void setup (){
    ANSEL = 0xFF; // page 128 129
    ANSELH = 0b00001111; //bit[1] is A/D input AN9
	OSCCON = 0b00010011; ////page28.   	bits[6,5,4] are clock divider  
	TRISA = 0b00101111; //[0-3]ADin, [4]Q1SolarRun, [5]ADin, [6,7]Q2Buck & Q3Batt
	TRISB = 0b00001100; //[0,1] LCD control. [2]Button, [3]unused, [4,5]Q10 & Q20
	TRISC = 0x00;
	TRISD = 0x00; 	 //setting ports B and D as outputs
	TRISE = TRISE & 0b11111111;

	LATA = 0x00;
	LATB = 0x00;
	LATC = 0x00;
	LATD = 0x00; 	 //setting ports B and D as outputs
	LATE = 0x00;
	//LATA = 0b00101111;
	 TRISCbits.TRISC4 = 0;  		 //input from button for INT2

	 TRISBbits.TRISB3 = 0; //AN9 on pin 36 - was going to use for solar in, but used AN0
    
	 //Interrupt Configuration
	PEIE = 1;
	 RCONbits.IPEN = 1;     //enable priority
	 INTCONbits.GIEH = 1;   //Global high priority enable
	 INTCONbits.GIEL = 1;   //Global peripheral enable
	 INTCONbits.TMR0IE = 1; //Enables timer 0 interrupt
	 INTCON2bits.TMR0IP = 1;    //sets timer 0 interrupt as high priority
	 T0CONbits.T08BIT = 0; 	 // sets timer 0 to 8 bit
	 T0CONbits.T0CS = 0;   	 //timer 0 uses internal clock
	 T0CONbits.PSA = 1;		 //timer 0 uses prescaler
	 T0CONbits.T0PS2 = 1;  	 //sets bit 2 for prescaler selection
	 T0CONbits.T0PS1 = 1;  	 //sets bit 1 for prescaler selection
	 T0CONbits.T0PS0 = 1;  	 //sets bit 0 for prescaler selection
	 INTCONbits.INT0IE =1;
	 INTCON3bits.INT2IE = 1;    //enable INT2
	 INTCON3bits.INT2IP = 0;    //define INT2 as low priority
	 //INTCON2bits.INTEDG2 = 1;   //trigger INT2 on rising edge


	 T0CONbits.TMR0ON = 1;
    
}

void main (void) {
    setup();
    while(1){
   	 green = 1;//debug
    	T0CONbits.TMR0ON = 1;
   	 getADvalues();
    	T0CONbits.TMR0ON = 0;
   	 green = 0;//debug
   	 CalcSupercapVolts();
   	 PowerControl();

    	OSCCON = 0b01100011;
   	 //red = 1;//debug
   	 CalcSupercapVolts();
   	 for(int i=0; i<4; i++){//converting variables to text for display
        	ReadoutDigits[i] = sprintf (V_string[i], "%1.2f", SupercapVolt[i]);
        	//ReadoutDigits[i] = 4; //Dont Need. Use proper %x.y as appropriate instead.
   	 }
//	lcddata((char)BattChargeMode);
    	ReadoutDigits[4] = sprintf (V_string[4], "%x", TMR0L);
    	//OSCCON = 0b00010011;
    	OSCCON = 0b00010011;
    	//red = 0;//debug
    	MSDelay(1000);
   	 lcd_init();
   	 pushToDisplay(2);
    	MSDelay(1);
    	//OSCCON = 0b00010011;

    	OSCCON = 0b01100011;
    	//blue = 1;//debug
   	 CalcVoltsAndAmps();
    	ReadoutDigits[0] = sprintf (V_string[0], "%1.2f", SupercapVoltAvg);
    	ReadoutDigits[1] = sprintf (V_string[1], "%1.2f", solar_volt);
    	ReadoutDigits[2] = sprintf (V_string[2], "%1.2f", SupercapSum);
    	ReadoutDigits[3] = sprintf (V_string[3], "%1.2f", Batt_volt);
    	OSCCON = 0b00010011;
   	 
   	 //blue = 0;//debug
    	//TMR0H = 0x20;
    	//TMR0L = 0x11;
    	MSDelay(1000);
    	//long TMR0count = 256*TMR0H + TMR0L;
    	ReadoutDigits[4] = sprintf (V_string[4], "%x", TMR0H);
    	//T0CONbits.TMR0ON = 0;
    	//TMR0H = 0;
    	//TMR0L = 0;
    	//T0CONbits.TMR0ON = 1;
   	 lcd_init();
   	 
   	 pushToDisplay(3);
    	blue=0;
    	red=0;
//   	 __asm__ ("sleep");
    }
}

void getADvalues(){
    //#define VDD 3.058   //used as scaling factor in routines that use AD values
    ADCON0 = 0x01; //power on AD converter .... page 255
    MSDelay(1);
    ADCON1 = 0x00; //bits 4 and 5 only, points to reference
    ADCON2 = 0b10111111; //bit 7 left justified
          			  //bits [3,4,5] TAD 000 is fast ACQ, 111 is slow ACQ
          			  //bits [0,1,2] ADC clock
    for(int i = 0; i <= numADvalues; i++){
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

void CalcSupercapVolts(void){   //calibration for resistive dividers
    float divFactorC[4]={57.31, //node 4, 20V nom
                     	32.32, //node 3, 15V nom etc... ^
                     	21.02, //node 2, 10V nom cap C0 + C1
                     	15.6}; //node 1, 5V nom cap C0 only
    SupercapVolt[0] = ((float)ADreadValue[1])/divFactorC[0];
    SupercapVolt[1] = ((float)ADreadValue[2])/divFactorC[1] - SupercapVolt[0];//
    SupercapVolt[2] = (float)ADreadValue[3]/divFactorC[2]-SupercapVolt[1]-SupercapVolt[0];
    SupercapVolt[3] = (float)ADreadValue[4]/divFactorC[3]-SupercapVolt[2]-SupercapVolt[1]-SupercapVolt[0];
}
void CalcVoltsAndAmps(void){	//calibration for resistive dividers
	float divFactorA[10]={  10.64, //solar volt
                        	15.62, //supercap sum, ie node 4
                        	4,  // supercap volt avg: (int)number of supercaps in string
                        	1,  //not implemented
                        	1,  //not implemented
                        	1,  //not implemented
                        	331.65, //battery volt WARNING 3V max using this resistor divider!!
                        	1,
                        	1,
                        	1};
	solar_volt = 0.7+(((float)ADreadValue[0])/divFactorA[0]);
	SupercapSum = (float)ADreadValue[4]/divFactorA[1];
	SupercapVoltAvg = SupercapSum/divFactorA[2];
	Load_current = (float)ADreadValue[5]/divFactorA[5];
	Batt_volt = (float)ADreadValue[6]/divFactorA[6];
}


void PowerControl(void){
	//------------Battery Charge Mode
if((    	solar_volt > SupercapSum + 5 || SupercapSum > min_supercap_volt + 2)
    	&&  Batt_volt < Batt_rebulk)
	{BattChargeMode = 1;}

if(     	Batt_volt > Batt_max_volt
    	||  SupercapVoltAvg < min_supercap_volt + 1)
	{BattChargeMode = 0;}
    
	//------------Q1 Solar run------------
if( 	(solar_volt - 0.7 > SupercapSum + 0.1)//considering the input diode, + headroom
    	&& SupercapSum < (max_supercap_volt *4) //check the total cap bank voltage
    	&& (   SupercapVolt[0] < max_supercap_volt //check each
        	&& SupercapVolt[1] < max_supercap_volt //individual cap also
        	&& SupercapVolt[2] < max_supercap_volt //to see if it's over volt
        	&& SupercapVolt[3] < max_supercap_volt //make sure none of
        	&& SupercapVoltAvg < max_supercap_volt + 0.2 //them get overcharged
        	)
 	)
	{Q1SolarRun = 1;
	if(solar_volt - SupercapSum > 6) //if supercaps are low compared to solar
    	{
    	MSDelay(50);                	//only give a pulse
    	Q1SolarRun = 0;
    	red=1;
    	//then turn off again
    	}
	}
	if(solar_volt - SupercapSum > 4) //if supercaps are low compared to solar
    	{
    	blue=1;
    	MSDelay(500);                	//only give a pulse
    	Q1SolarRun = 0;             	//then turn off again
   	 
    	}

if(solar_volt+0.25 < solar_before //see if solar voltage has dropped more than 1/4 volt
   || solar_volt < 8.02
   ||(  	SupercapVolt[0] > max_supercap_volt
    	||  SupercapVolt[1] > max_supercap_volt
    	||  SupercapVolt[2] > max_supercap_volt
    	||  SupercapVolt[3] > max_supercap_volt
    	||  SupercapVoltAvg > max_supercap_volt + 0.5
	)
  )
  {Q1SolarRun = 0;}

	//------------Q2 Buck run------------
if ( (  	Q1SolarRun == 1 && SupercapVoltAvg > min_supercap_volt)
    	||  SupercapVoltAvg + 0.5 > max_supercap_volt
    	||  (SupercapVoltAvg > min_supercap_volt && Load_current > loadThresh)
   )  
   {Q2BuckRun = 1;}
if (	SupercapVoltAvg + 0.5 < max_supercap_volt
    	&& Load_current < loadThresh
    	&& BattChargeMode == 0
   	 
	)
	{MSDelay(1000);Q2BuckRun = 0;}  
if (	SupercapVoltAvg < min_supercap_volt
    	&& Load_current < loadThresh
	)
	{MSDelay(1000);Q2BuckRun = 0;}    


	//------------Q3 Battery Charge run------------
if (BattChargeMode == 1 && SupercapVoltAvg > min_supercap_volt + 1.5){
   	 
    	Q3BattCharge = 1;
    	MSDelay(100);
    	}
Q3BattCharge = 0;

//---------------Supercap Balancers-------------------
if(SupercapVolt[0] >= max_supercap_volt){
	SC1Drain_Q10 = 1;
	MSDelay(SCapDrainTime);
	SC1Drain_Q10  = 0;
}
if(SupercapVolt[1] >= max_supercap_volt){
	SC2Drain_Q20 = 1;
 	MSDelay(SCapDrainTime);
	SC2Drain_Q20  = 0;
}
if(SupercapVolt[2] >= max_supercap_volt){
	SC3Drain_Q30 = 1;
 	MSDelay(SCapDrainTime);
	SC3Drain_Q30  = 0;
}
if(SupercapVolt[3] >= max_supercap_volt){
	SC4Drain_Q40 = 1;
 	MSDelay(SCapDrainTime);
	SC4Drain_Q40  = 0;
    
}
solar_before = solar_volt;
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

void pushToDisplay (unsigned char Values){
    //PORTCbits.blue = 1;//debug
    //lcd_init();
    lcdcmd(0x01);  	 //clear LCD
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
    lcddata(' ');
	lcddata('!');
if(BattChargeMode == 0){lcddata('0');}
if(BattChargeMode == 1){lcddata('1');}
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
	lcddata(' ');
    
    for (int n = 0; n < ReadoutDigits[4]; n++){
   	 lcddata(V_string[4][n]);
   	 }
}


void lcdcmd(unsigned char value)   	 //command
{
    ldata = value;      //put received argument on PORTD
    rs = 0;
    en = 1;
    MSDelay(LCDdelay);
    en = 0;
    MSDelay(LCDdelay);
}

void lcddata (unsigned char value)  	 //data
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
    en = 0;    	 //initialize, clear enable
    lcdcmd(0x38);  	 //initialize LCD 2 lines, 5x7 matrix
    lcdcmd(0x01);  	 //display on cursor
    //lcdcmd(0x00);  	 //display on cursor
    lcdcmd(0x06);      //increments cursor after each byt
    lcdcmd(0x0F);  	 //line 1 position 6
    //PORTCbits.red = 0;//debug  
}

 void MSDelay(int mili)
 {
     int i, j;
     for (i=0; i <mili; i++)
		 for(j=0; j<delayConst; j++);   	 
 }






