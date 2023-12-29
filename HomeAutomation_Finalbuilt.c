#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

#define  _XTAL_FREQ 8000000 //1 Mhz internal clock by default
//#define F_CPU 8000000/64
//#define Baud_value (((float)(F_CPU)/(float)9600)-1)

// PIC18F45K20 Configuration Bit Settings
// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO7   // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 18        // Brown Out Reset Voltage bits (VBOR set to 1.8 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config HFOFST = OFF      // HFINTOSC Fast Start-up (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))


/** RB4, RB3, RB7 --> Input Switches ***/
//#define BUZ PORTBbits.RB6
#define LED PORTBbits.RB4
#define REL PORTBbits.RB1

#define lcd PORTD
unsigned char rs,en,rw;
unsigned int d;

void lcd_cmd(unsigned char z)
{
  rs=0x00;//for command
  rw=0x00;
  en=0x08;//en high
  lcd=((z)&0xf0)|en|rs;
  __delay_ms(10);//Delay10TCYx(10);
  en=0x00;//en low
  lcd=((z)&0xf0)|en|rs;
  en=0x08;//en high
  lcd=((z<<4)&0xf0)|en|rs;
  __delay_ms(10);
  //Delay10TCYx(10);
  en=0x00;//en low
  lcd=((z<<4)&0xf0)|en|rs;
}
void lcd_data(unsigned char z)
{
     rs=0x04 ;//for data
     rw=0x00;
     en=0x08;//en high
     lcd=((z)&0xf0)|en|rs;
     __delay_ms(10);
     //Delay10TCYx(10);
     en=0x00;//en low
     lcd=((z)&0xf0)|en|rs;
     en=0x08;//en high
     lcd=((z<<4)&0xf0)|en|rs;
     __delay_ms(10);
     //Delay10TCYx(10);
     en=0x00;//en low
     lcd=((z<<4)&0xf0)|en|rs;
}

void lcd_string(char *dat)
	{
        unsigned int i = 0;
        while(*dat != '\0')
		{
            lcd_data(*dat++);	// Call lcddata function to send characters one by one from "data" array
		}
	}
void init_lcd()
{
     lcd_cmd(0x02); //4bit mode
     lcd_cmd(0x06); //entry mode
     lcd_cmd(0x0c); //display on cr off
     lcd_cmd(0x28); //4 bit
}

void init_usart()
{
   //float temp;
    //TRISC6=0;                       /*Make Tx pin as output*/
    //TRISC7=1;                       /*Make Rx pin as input*/
    //temp= 12;     
    //SPBRG=(int)temp;                /*baud rate=9600, SPBRG = (F_CPU /(64*9600))-1*/
    //TXSTA=0x20;                     /*Transmit Enable(TX) enable*/ 
    //RCSTA=0x90;                     /*Receive Enable(RX) enable and serial port enable */
    TRISC6=0;                       /*Make Tx pin as output*/
    TRISC7=1;                       /*Make Rx pin as input*/ 
    SPBRG = 103;         /* Serial Port Baud Rate Generator for 9600    */  
    TXSTA = 0X24;         /* TXEN=1, SYNC=0, BRGH=1                      */
    RCSTA = 0X90;         /* Reception Enable (SPEN=1,CREN=1)            */
    
}

/******************TRANSMIT FUNCTION*****************************************/ 
void usart_tx(char out)
{        
        //while(TXIF==0);            /*wait for transmit interrupt flag*/
        //TXREG=out;                 /*transmit data via TXREG register*/ 
    TXREG = out;          /* Load the character to be transmitted        */ 
    while(!TRMT);         /* Wait here till transmission is complete     */
    
}
// USART STRING //
void usart_string(char *dat)
{        
    unsigned int i = 0;
    while(*dat != '\0')
	{
        usart_tx(*dat++);	// Call lcddata function to send characters one by one from "data" array
    }
}
/*******************RECEIVE FUNCTION*****************************************/
char usart_rx()
{
    while(RCIF==0);       //wait for receive interrupt flag
    if(RCSTAbits.OERR)
    {           
        CREN = 0;
        NOP();
        CREN=1;
    }
    return(RCREG);   //receive data is stored in RCREG register and return 
}

void main(void) 
{
    char rec;
    //Data Direction Registers
    OSCCON=0x72;		/*Set internal Osc. frequency to 8 MHz*/

    TRISB = 0b00000001; //configure PORT B, 0 = OUTPUT, 1 = INPUT
    TRISA = 0x00;
    TRISD = 0x00; // Configure Port D as output port
    
    init_lcd();
    
    init_usart();
    
    usart_string("SYSTEM STARTED");
    usart_string("\n");
    
    //TITLE OF THE PROJECT
    lcd_cmd(0x80); //Print on the First Line
    lcd_string("HOME  AUTOMATION");
    lcd_cmd(0xC0); //Print on the Second Line
    lcd_string("-----SYSTEM-----");
    
    __delay_ms(4000);
    lcd_cmd(0x01); //Clear the Display
    lcd_string(" LIGHTS ARE OFF ");
    LED = 0;
    REL = 0;
    while(1)
    {
        rec = usart_rx(); // recive data from app
        
        if(rec == 'A')
        {
            LED = 1;
            __delay_ms(50);
            REL = 1;
            rec = 0; // reset value
            lcd_cmd(0x80); //Print on the First Line
            lcd_string("  LIGHT IS ON   ");
        }
        
        else if(rec == 'B')
        {
           LED = 0;
           __delay_ms(50);
           REL = 0;
           rec = 0;
           lcd_cmd(0x80); //Print on the First Line
           lcd_string("  LIGHT IS OFF  ");
        }
        
        else if(rec == 'C' || rec == 'D' || rec == 'E' || rec == 'F')
        {
           LED = 0;
           __delay_ms(50);
           REL = 0;
           rec = 0;
           lcd_cmd(0x80); //Print on the First Line
           lcd_string("  WRONG INPUT  ");
        }

    }
    return;
}