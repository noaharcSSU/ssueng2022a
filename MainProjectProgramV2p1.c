//SmoothPower Main Project V2
//Good display engine
//
//(need to add A/D routines back in)
#pragma config FOSC = INTIO67   //internal oscillator block, port function on RA6 and RA7
#pragma config BORV = 27, PWRT = ON //brown-out-Reset Voltage = 2.7V
#pragma config WDTEN = OFF  //watchdog time disabled
#pragma config PBADEN = OFF //PORT A/D disabled
#pragma config HFOFST = OFF, MCLRE = OFF // fast start-up and MCLR pin disabled
//#include <pic18f45k20.h>
#include <stdio.h>
#include <xc.h>

#define ldata PORTD  	//lcd data pins on PORTD
#define rs PORTBbits.RB0 //rs os lcd on PORTB0
#define en PORTBbits.RB1	//en of lcd on PORTB1
#define red RC4   //debug LED
#define blue RC6	//debug LED
#define green RC5   //debug LED
#define sigfigs 5 //the decimal point even counts as a sigfig, so 5 is really 4 actual sigfigs

double Voltage [9]; //[0] is solar in Volt. [1,2,3,4] are supercap nodes,
double Voltage1 [4] = {2.537, 3.321, 2.166, 2.039}; //debug constants to display
double Voltage2 [4] = {77717, 888, 2166, 2039}; //debug constants to display
double max_cap_delta = 0.325; //cap voltage deviation from average of 4 supercaps
char V_string[sigfigs][6];

void MSDelay(int);
void Calc_volt(void);
void lcd_init (void);
void lcdcmd(unsigned char);
void lcddata (unsigned char);
void pushToDisplay (void);
void setup (void);


void setup (){
	OSCCON = 0b01000011;
        	////654     	bits[6,5,4] are clock divider
	TRISB = 0x00;
	TRISC = 0x00;
	TRISD = 0x00;  	//setting ports B and D as outputs
	TRISA = 0x03;
	ADCON0 = 0x01;
	ADCON1 = 0x00;
	ADCON2 = 0xA8;    
	lcd_init();
	PORTCbits.red = 1;//debug
	MSDelay(20);
	PORTCbits.red = 0;//debug
	MSDelay(20);
	PORTCbits.green = 1;//debug
	MSDelay(20);
	PORTCbits.green = 0;//debug    
	MSDelay(20);
	PORTCbits.blue = 1;//debug
	MSDelay(20);
	PORTCbits.blue = 0;//debug
}

void main (void) {

	while(1){
	setup();
	//PORTCbits.red = 1;//debug
	void Calc_volt(void);
	//PORTCbits.red = 0;//debug
	for (int i=0; i<sigfigs; i++){
    	sprintf (V_string[i], "%g", Voltage1[i] );   	//convert digits of C ‘long’ datatype number into ASCII characters for display 
	}
	pushToDisplay();
	for (int j=0; j<sigfigs; j++){
    	sprintf (V_string[j], "%g", Voltage2[j] );   	 
	}
	pushToDisplay();
	PORTCbits.blue = 0;//debug
	PORTCbits.green = 1;//debug
	MSDelay(2500);
	PORTCbits.green = 0;//debug
	}
}



void lcdcmd(unsigned char value)    	//command
{
	//PORTCbits.red = 1;//debug
	ldata = value;  	//put received argument on PORTD
	rs = 0;
	en = 1;
	MSDelay(20);
	en = 0;
	MSDelay(50);
	//PORTCbits.red = 0;//debug
}

void lcddata (unsigned char value)   	//data
{
    
	ldata = value;
	rs = 1;
	en = 1;
	MSDelay(20);
	//PORTCbits.red = 1;//debug
	en = 0;
	MSDelay(50);
	//PORTCbits.red = 0;//debug
}

void lcd_init (void)
{
	en = 0;     	//initialize, clear enable
	MSDelay(25);
	lcdcmd(0x38);   	//initialize LCD 2 lines, 5x7 matrix
	//lcdcmd(0x28)  	//initialize LCD with 2 rows enabled
	MSDelay(25);
	lcdcmd(0x01);   	//display on cursor
	lcdcmd(0x06);  	//increments cursor after each byt
	MSDelay(25);
	lcdcmd(0x0F);   	//line 1 position 6
	MSDelay(10);
}

 void MSDelay(int mili)
 {
 	int i, j;
 	for (i=0; i <mili; i++)
     	for(j=0; j <5; j++);        	 
 }

void pushToDisplay (void){
	PORTCbits.red = 0;//debug
	//PORTCbits.blue = 1;//debug
	lcd_init();
	lcdcmd(0x01);   	//clear LCD
	lcdcmd (0x80);
	for (int j = 0; j < sigfigs; j++){
    	lcddata(V_string[0][j]);
    	}
	lcddata('V');
	lcddata(' ');
	for (int j = 0; j < sigfigs; j++){
    	lcddata(V_string[1][j]);
    	}
	lcddata('V');
	lcdcmd (0xC0);
	for (int i = 0; i < sigfigs; i++){
    	lcddata(V_string[2][i]);
    	}
	lcddata('V');
	lcddata(' ');
	for (int i = 0; i < sigfigs; i++){
    	lcddata(V_string[3][i]);
    	}
	lcddata('V');
}

// average voltage of the supercaps
void Calc_volt(void){
	double sum = Voltage[0] + Voltage[1] + Voltage[2] + Voltage[3] ;
	double volt_avg = sum / 4;
  	if(volt_avg - Voltage[0] > max_cap_delta){
        	PORTCbits.RC0 = 1;
        	MSDelay(500);
        	PORTCbits.RC0 = 0;
        	MSDelay(500);
    	}
  	/*else{
        	PORTCbits.RC7 = 1;
        	MSDelay(500);
        	PORTCbits.RC7 = 0;
        	MSDelay(500);
   	}*/
  	if(volt_avg - Voltage[1] > max_cap_delta){
        	PORTCbits.RC1 = 1;
        	MSDelay(500);
        	PORTCbits.RC1 = 0;
        	MSDelay(500);
    	}
  	/*else{
        	PORTCbits.RC6 = 1;
        	MSDelay(500);
        	PORTCbits.RC6 = 0;
        	MSDelay(500);
    	}*/
  	if(volt_avg - Voltage[2] > max_cap_delta){
        	PORTCbits.RC2 = 1;
        	MSDelay(500);
        	PORTCbits.RC2 = 0;
        	MSDelay(500);
    	}
  	/*else{
        	PORTCbits.RC5 = 1;
        	MSDelay(500);
        	PORTCbits.RC5 = 0;
        	MSDelay(500);
    	}*/
  	if(volt_avg - Voltage[3] > max_cap_delta){
        	PORTCbits.RC3 = 1;
        	MSDelay(500);
        	PORTCbits.RC3 = 0;
        	MSDelay(500);
    	}
  	/*else{
        	PORTCbits.RC4 = 1;
        	MSDelay(500);
        	PORTCbits.RC4 = 0;
        	MSDelay(500);
    	}*/
}
