#include <p18f4550.h>
#include <spi.h>
//#include <stdint.h>
#include "LTC6946.h"
//#define LED PORTBbits.RB3;

#pragma config PBADEN=OFF

extern void _startup (void);

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

void WAIT_TO_TX(void)
{
	while (!(SSPSTAT & 0x01))
	{
		;
	}
}

void myMsDelay(unsigned int time)
{
	unsigned int i, j;
	for (i = 0; i < time; i++)
		for (j = 0; j < 71; j++);
}																								//not calibrated

char PLLSPI_write_byte(uint8_t REGISTOR, uint8_t data)
{
	// returns 1 if success else 0
	int ii;            
																								//	PORTAbits.RA5=0;
	PORTBbits.RB2=0;																			//  PORTBbits.RB2 is CS_bar
	// Send Adress
	if (WriteSPI(REGISTOR<<1) ==-1)
		return 0;
			
	/*Wait for transmission to finish */
	WAIT_TO_TX();

	/* Read the received value */
	ii = SSPBUF;

	/* Send the Data */
	if(WriteSPI(data) ==-1)
		return 0;
			
	/*Wait for transmission to finish */
	WAIT_TO_TX();
			

	/* Read the received value */
	ii = SSPBUF;

	PORTBbits.RB2=1;

	return 1;
}

char PLLSPI_read_byte(uint8_t REGISTOR, uint8_t* RX)
{
	// returns 1 if success else 0
	uint8_t ii;            
	                                                                                                       //PORTAbits.RA5=0;
	PORTBbits.RB2=0;
	/* Send the Address */
	if (WriteSPI((REGISTOR<<1) | (0x01)) ==-1)
		return 0;
			
	/*Wait for transmission to finish */
	WAIT_TO_TX();

	/* Read the received value */
	ii = SSPBUF;

	/* Send the Data */
	if (WriteSPI(0xff) ==-1)
		return 0;
			
	/*Wait for transmission to finish */
	WAIT_TO_TX();
			

	/* Read the received value */
	ii = SSPBUF;

	PORTBbits.RB2=1;

	*RX = ii;
	return 1;
}

uint8_t pow(uint8_t i)
{
	if(i == 0)
		return 1;
	else
		return 2*pow(i - 1);
}

void read_status_bit(uint8_t _bit)
{
	uint8_t status;
	status = 0;
	PLLSPI_read_byte(0, &status);
	if (status & pow(_bit) == 0)
		PORTAbits.RA3 = 1;
	else
		PORTAbits.RA4 = 1;
}

uint8_t REG[12];                //!< Register values to be written or read from
char ErrorSPI=0;
uint8_t O_DIV;
uint8_t filt;
uint8_t LKWIN;
uint8_t B_DIV;
uint8_t BST;
uint8_t rfo;
uint8_t i_cp;
uint8_t lkcnt;
uint8_t R_DIV[2];
int i;
uint8_t READ_BACK[12];
uint8_t SPI_write_status = 1;
uint8_t status;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void main(void)
{	
	// Make all of read back and reg = 0
	for (i = 0; i < 12; i++) 
		{
		REG[i] = 0x00;	
		READ_BACK[i] = 0x00;
		}

//PORTA initialisation............................
	PORTA=0x00; ;
	LATA=0x00; ;
	ADCON1=0x0F; 
	TRISA=0x00; ;
    CMCON=0x07;
	CVRCON=0X00;

//PORTB initialisation............................
	PORTB=0x00;
	LATB=0x00;
	// as PBADEN=0 from config settings;
	ADCON1=0x0F; //0x0E changed to 0x0f
	TRISB=0b11100001;

//PORTC initialisation............................
	PORTC=0x00;
	LATC=0x00;
	TRISC=0x00;

//PORTD initialisation............................
	PORTD=0x00;
	LATD=0x00;
	ADCON1=0x0F; //0xFF changed to 0x0f
	TRISD=0x00;

	PORTA = 0x00;


	PORTAbits.RA5 = 1;
	PORTAbits.RA3 = 1;
	
	ErrorSPI=3;
	while(ErrorSPI)
		{
		myMsDelay(20);
		PORTAbits.RA4 = 0;
		PORTAbits.RA3 = 1;
		myMsDelay(20);
		PORTAbits.RA4 = 1;
		PORTAbits.RA3 = 0;
		ErrorSPI--;
		}

	OpenSPI(SPI_FOSC_4, MODE_00,SMPMID);
	SSPBUF=0;

    // Define STAT = LOCK
    REG[1] = 0x04;

    // Power Down REFO and Mute output during caliberation
    REG[2] = LTC6946_PDREFO | LTC6946_MTCAL;
	REG[2] = LTC6946_MTCAL;

    // f_PFD < 2.4MHz
    B_DIV = LTC6946_BD_0;

    // Output Frequency Range set to 2.24GHz to 3.74GHz
    O_DIV = LTC6946_O_DIV_1;

    // Set f_STEP to 500kHz
    R_DIV[0] = 0x7B;
    R_DIV[1] = 0;

    REG[3] = B_DIV | R_DIV[1];
    REG[4] = R_DIV[0];

	
    REG[5] = 0x84;
    REG[6] = 0x38;

    // Required settings to allow proper loop locking
    REG[7] = LTC6946_ALCCAL | LTC6946_ALCULOK | LTC6946_LKEN | LTC6946_CAL; 

    // V_REF > 2.0V P-P
    BST = LTC6946_BST_0;

    // f_REF < 20MHz
    filt = LTC6946_FILT_3;

    // RF Output = 3dBm Differential
    rfo = LTC6946_RFO_3;

    REG[8] = BST | filt | O_DIV | rfo;

    // charge pump current 
    i_cp = LTC6946_CP_7;
    // f_PFD < 550kHz;
    LKWIN = LTC6946_LKWIN_3;
    // Lock count = 512
    lkcnt = LTC6946_LKCNT_2;
    REG[9] = i_cp | LKWIN | lkcnt;

    // Sets charge pump
    REG[10] = LTC6946_CPCHI | LTC6946_CPCLO;
	
	for (i = 1; i < 11; i++) 
	{
		PLLSPI_write_byte(i, REG[i]);
		PLLSPI_write_byte(i, REG[i]);
		PORTAbits.RA4 = 0;
		PORTAbits.RA3 = 0;
		myMsDelay(20);
		READ_BACK[i] = 0x00;
		PLLSPI_read_byte(i, READ_BACK + i);
		if (READ_BACK[i] == REG[i]) {
			PORTAbits.RA4 = 1;
		}
		else
			PORTAbits.RA3 = 1;
		myMsDelay(20);
	}
	PLLSPI_read_byte(i, READ_BACK + i);
	PLLSPI_read_byte(0, READ_BACK + 0);

	while(1);
}