//MainProjectProgramV2p7- Display 4 floats in fixed position

//Next tasks 
//Adding timer for wakeup
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
#define en PORTBbits.RB1    //en of lcd on PORTB1
#define red RC4   //debug LED
#define blue RC6    //debug LED
#define green RC5   //debug LED
#define sigfigs 4
#define SCapDrainTime 1
#define numADvalues 5

unsigned char delayConst = 1;
float Voltage [numADvalues]; //[0] is solar in Volt. [1,2,3,4] are supercap nodes,
unsigned char VoltageDigits[numADvalues] = {5,5,5,5}; //keep track of number of digits sprintf returns
//double Voltage2 [4] = {1.0111, 2.0222, 3.0333, 4.0444}; //debug constants to display
double max_cap_delta = 0.325; //cap voltage deviation from average of 4 supercaps
char V_string[sigfigs][6];

void MSDelay(int);
void PowerControl(void);
void PowerControlAlt(void);
void lcd_init (void);
void lcdcmd(unsigned char);
void lcddata (unsigned char);
void pushToDisplay (void);
void setup (void); //I don't know why it won't run setup if you don't treat
//it like some second-rate subroutine
void getADvalues(void);


void setup (){
	ANSEL = 0xFF; // page 128 129
	ANSELH = 0b00000010; //bit[1] is A/D input AN9
    OSCCON = 0b00000011; ////page28.       bits[6,5,4] are clock divider  
    TRISB = 0x00;
    TRISC = 0x00;
    TRISD = 0x00;  	//setting ports B and D as outputs
    TRISA = 0b00101111;
     				   //A to D setup in the actual subroutine
    lcd_init();
}

void main (void) {
	PORTCbits.red= 1;//debug
   
    PORTCbits.red= 0;//debug

    while(1){

    PORTCbits.green = 1;//debug
	getADvalues();
	PORTCbits.green = 0;//debug
    //Get one set of values for display
    OSCCON = 0b01100011; ///go fast for sprintf 	bits[6,5,4] are clock divider  
    for (int i=0; i<numADvalues; i++){//converting variables to text for display
      //VoltageDigits[i] =
	sprintf (V_string[i], "%f", Voltage[i] );
    }
    OSCCON = 0b00010011; ///go slow again to save power, plenty fast for other tasks
	setup();
	pushToDisplay();
	//MSDelay(5);
    
    /*
    //Prepare another set of values for display
    PORTCbits.blue = 1;//debug    
    for (int j=0; j<sigfigs; j++){
        sprintf (V_string[j], "%g", Voltage2[j] );     
    }
	PORTCbits.blue = 0;//debug
    pushToDisplay();
	*/
    PowerControl();
    }
    //start timer, set desired wakeup parameters
    //sleep
}

void lcdcmd(unsigned char value) 	   //command
{
    //PORTCbits.red = 1;//debug
    ldata = value;  	//put received argument on PORTD
    rs = 0;
    en = 1;
    //MSDelay(2);
    en = 0;
    //MSDelay(5);
    //PORTCbits.red = 0;//debug
}

void getADvalues(){
    ADCON0 = 0x01; //power on AD converter .... page 255
	//bits [2,3,4,5] point to AD input, Ax
    MSDelay(5);
    ADCON1 = 0x00; //bits 4 and 5 only, points to reference
    ADCON2 = 0b10111111; //bit 7 left justified
                    	//bits [3,4,5] TAD 000 is fast ACQ, 111 is slow ACQ
                    	//bits [0,1,2] ADC clock
    for(int i = 0; i < numADvalues; i++){
   	 ADCON0bits.GO = 1;  //start AD conversion
   	 while(ADCON0bits.GO ==1) {}; //wait for conversion to finish
   	 Voltage [i] = 3.0*((256 * ADRESH) + ADRESL)/1024;
   	 ADCON0 = ADCON0 + 4;
        /*
  	  if(A/D converter inputs not consecutive){
  		  add extra 4 to ADCON0 to advance to next AD input
  		  to read correct AD port
  	  }*/
    }
    ADCON0 = 0x00; //turn off A/D conv to save power
}

void lcddata (unsigned char value)       //data
{
    
    ldata = value;
    rs = 1;
    en = 1;
    MSDelay(1);
    en = 0;
    MSDelay(1);
}

void lcd_init (void)
{
    PORTCbits.blue = 1;//debug
    en = 0;  	   //initialize, clear enable
    MSDelay(2);
    lcdcmd(0x38);       //initialize LCD 2 lines, 5x7 matrix
    MSDelay(2);
    lcdcmd(0x01);       //display on cursor
    lcdcmd(0x06);  	//increments cursor after each byt
    MSDelay(2);
    lcdcmd(0x0F);       //line 1 position 6
    MSDelay(1);
    PORTCbits.blue = 0;//debug
}

 void MSDelay(int mili)
 {
 	int i, j;
 	for (i=0; i <mili; i++)
  	   for(j=0; j<delayConst; j++); 		 
 }

void pushToDisplay (void){
    //PORTCbits.blue = 1;//debug
    //lcd_init();
    lcdcmd(0x01);       //clear LCD
//////Line 1
    lcdcmd (0x80);  //ready on line 1
//first number
    for (int i = 0; i < VoltageDigits[0]; i++){
   	 lcddata(V_string[0][i]);
	   }
	lcddata(' ');
    //lcddata('V');
//second number
    for (int j = 0; j < VoltageDigits[1]; j++){
        lcddata(V_string[1][j]);
        }
    //lcddata('V');
//////Line 2    
    lcdcmd (0xC0);  //ready on line 1
    for (int k = 0; k < VoltageDigits[2]; k++){
        lcddata(V_string[2][k]);
        }
    //lcddata('V');
	lcddata(' ');
    for (int m = 0; m < VoltageDigits[3]; m++){
        lcddata(V_string[3][m]);
        }
    //lcddata('V');
}

// average voltage of the supercaps
void PowerControl(void){
    double SupercapVolt[4]; // variable array for voltage nodes
	double SupercapSum = Voltage[4]*20 ;
	double SupercapVoltAvg = SupercapSum / 4;
    SupercapVolt[0] = Voltage[1]*5;
  	if(SupercapVolt[0] - SupercapVoltAvg > max_cap_delta){
  		  PORTCbits.RC0 = 1;
  		  MSDelay(500);
  		  PORTCbits.RC0 = 0;
  	  }
  	else{
  		  PORTCbits.RC7 = 1;
  		  MSDelay(500);
  		  PORTCbits.RC7 = 0;
  		  MSDelay(500);
       }
    SupercapVolt[1] = (Voltage[2]*10) - (Voltage[1]*5);
  	if(SupercapVolt[1] - SupercapVoltAvg > max_cap_delta){
  		  PORTCbits.RC1 = 1;
  		  MSDelay(500);
  		  PORTCbits.RC1 = 0;

  	  }
  	else{
  		  PORTCbits.RC6 = 1;
  		  MSDelay(500);
  		  PORTCbits.RC6 = 0;
  		  MSDelay(500);
  	  }
  	 
    SupercapVolt[2] = (Voltage[3]*15) - (Voltage[2]*10);
  	if(SupercapVolt[2] - SupercapVoltAvg > max_cap_delta){
  		  PORTCbits.RC2 = 1;
  		  MSDelay(500);
  		  PORTCbits.RC2 = 0;

  	  }
  	else{
  		  PORTCbits.RC5 = 1;
  		  MSDelay(500);
  		  PORTCbits.RC5 = 0;
  		  MSDelay(500);
  	  }
    SupercapVolt[3] = Voltage[4]*20 - Voltage[3]*15;
  	if(SupercapVolt[3] - SupercapVoltAvg > max_cap_delta){
  		  PORTCbits.RC3 = 1;
  		  MSDelay(500);
  		  PORTCbits.RC3 = 0;

  	  }
  	else{
  		  PORTCbits.RC4 = 1;
  		  MSDelay(500);
  		  PORTCbits.RC4 = 0;
  		  MSDelay(500);
  	  }
}



