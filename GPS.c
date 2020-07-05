/*
Program : To Interface GPS MAX2769 with  Aurum board through SPI communication 

Date : 08 May 2018
*/

#include <p18f4550.h>

#define TRIS_SDI TRISBbits.TRISB0       // DIRECTION define SDI input
#define TRIS_SCK TRISBbits.TRISB1      // DIRECTION define clock pin as output
#define TRIS_SDO TRISCbits.TRISC7      //DIRECTION SD0 as output
#define CS_bar   PORTAbits.RA2
//#define PGM   PORTAbits.RA3

signed char WriteSPI( unsigned char data_out );

extern void _startup (void);
//-------------------------------------------------------------------------------------------------
/*The following lines of code perform interrupt vector relocation to work with the USB bootloader. 
These must be used with every application program to run as a USB application.*/

#pragma code _RESET_INTERRUPT_VECTOR = 0x1000

void _reset (void)
{
	_asm goto _startup _endasm
}

#pragma code
#pragma code _HIGH_INTERRUPT_VECTOR = 0x1008
void high_ISR (void)
{
}

#pragma code
#pragma code _LOW_INTERRUPT_VECTOR = 0x1018
void low_ISR (void)
{
}
#pragma code
/*End of interrupt vector relocation*/
//------------------------------------------------------------------------------

signed char WriteSPI( unsigned char data_out )  //Function to be used for writing data 
{
    SSPBUF = data_out;              // write byte to SSPBUF register
    while( !PIR1bits.SSPIF );       // wait until bus cycle complete
    PIR1bits.SSPIF = 0;             // Clear interrupt flag
    return ( 0 );                   // if WCOL bit is not set return non-negative#
}

void myMsDelay(unsigned int time)
{
	unsigned int i, j;
	for (i = 0; i < time; i++)
		for (j = 0; j < 71; j++);/*Calibrated is not correct*/
}

void main()
{
    int i=0;
	unsigned char REG[10][4];       //REG[j] is a 32 bit word that includes the data(28 bits) and
                                    //the address(4 bits) of the register.
//	PGM=1;

//------------------------To set the registor values---------------------------
	//Configuration 1
	REG[0][3] = 0b10100010;
	REG[0][2] = 0b10010111;  //Turn off both LNA
	REG[0][1] = 0b00011011;  //Shut-down Antenna bias
	REG[0][0] = 0b00110000;
	
	//Configuration 2
	REG[1][3] = 0b00000101;  
	REG[1][2] = 0b01010001;  //Gain set from Serial Interface
	REG[1][1] = 0b00101010;  
	REG[1][0] = 0b10000001;

	//Configuration 3
	REG[2][3] = 0b11111110;  //GAIN set to 62 dB
	REG[2][2] = 0b11111110;  
	REG[2][1] = 0b00011101;  
	REG[2][0] = 0b11000010;
	
	//PLL Configuration
	REG[3][3] = 0b10011010;  //Enable Clock Buffer
	REG[3][2] = 0b11000000;
	REG[3][1] = 0b00000000;  
	REG[3][0] = 0b10000011;  //Enable Integer PLL
	
	//PLL Integer Division Ratio
//	REG[4][3] = 0x01;// fractional pll
//	REG[4][2] = 0b00111000;
//	REG[4][1] = 0x00;
//	REG[4][0] = 0x84;

	REG[4][3] = 0b10011001; //integer pll
	REG[4][2] = 0b00101000;
	REG[4][1] = 0b00111110;
	REG[4][0] = 0b10000100;

	//PLL Division Ratio
	REG[5][3] = 0xD4;
	REG[5][2] = 0xFD;
	REG[5][1] = 0xF7;
	REG[5][0] = 0x05;
	
	//Reserved
	REG[6][3] = 0x80;
	REG[6][2] = 0x00;
	REG[6][1] = 0x00;
	REG[6][0] = 0x06;

	//Clock Fractional Division Ratio.
	REG[7][3] = 0x10;
	REG[7][2] = 0x06;
	REG[7][1] = 0x1B;
	REG[7][0] = 0x27;
	
	//Test Mode 1
	REG[8][3] = 0x1E;
	REG[8][2] = 0x0F;
	REG[8][1] = 0x40;
	REG[8][0] = 0x18;
	
	//Test Mode 2
	REG[9][3] = 0x28;
	REG[9][2] = 0xC0;
	REG[9][1] = 0x40;
	REG[9][0] = 0x29;
    
    ////-----------SPI Configuration-----------////
    SSPSTAT &= 0x3F;            // initial state
    SSPCON1 = 0x00;             // initial state
    SSPCON1 |= 0x00;            // select serial mode       (0b00000010 - SPI Master mode, clock = Fosc/64,last 3 bits =000 makes clock=Fosc/4)
    //SSPSTAT |= 0x80;          // select data input sample phase
    SSPSTATbits.CKE = 1;        // data transmitted on rising edge
    SSPCON1bits.CKP = 0;        // clock idle state low for (0,0) SPI Mode 
    TRIS_SDI = 1;               // define SDI input
    TRIS_SDO = 0;               //SD0 as output 
    TRIS_SCK  = 0;              // define clock pin as output  
    TRISAbits.TRISA2=0;         //Chip Select CS pin set as output pin
    SSPCON1 |= 0x20;            // enable synchronous serial port,  0b00100000  Enable serial port and configures SCK, SDO, SDI 
    
	//To program the registers of GPS Chip.
	while(1){
        for(i=0;i<10;i++)
		{
			CS_bar=0;                           //setting CS_bar low to mark the beginning of communication
        	WriteSPI(REG[i][3]);               //send first byte of data
			WriteSPI(REG[i][2]);
			WriteSPI(REG[i][1]);
			WriteSPI(REG[i][0]);
        	CS_bar=1;
		   // myMsDelay(5);                           
  		}                                 //setting CS_bar high to mark the end of communication                        
    
	}
	while(1);  
}
