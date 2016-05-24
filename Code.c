// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"
#include <math.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define PI 3.14159257
#define N 256.0

char rxbuf[81],ch;
int cnt=0;
char field[20];
int fpos[20];
int i=0,field_no;
float freq,amp,integer,duty,dutyy;
char *signal;
float x,V_out,Vout;
int u,v;
uint32_t phase=0,d_phase;
int n;
uint16_t lut[256],lutval,r;
int min,sample,lt;
float zero_slow,zero_fast,cal_slow,cal_fast,slope_slow,slope_fast,vol_slow,vol_fast,newvol_slow,newvol_fast;
float zero_slow_o,zero_fast_o,cal_slow_o,cal_fast_o,slope_slow_o,slope_fast_o,vol_slow_o,vol_fast_o,newvol_slow_o,newvol_fast_o;
float Vavg,offset=1;
float freq1,freq2,f;
float volt,newvolt,voldb;
int wait=0;
char ftoa[15];
int count=0,sqr=0;
float fc,sqval,cap;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
void waitPbPress()
{
	while(PUSH_BUTTON);
}
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
void initEEPROM()
{

	SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;
	__asm(" NOP");
	__asm(" NOP");
	__asm(" NOP");
	__asm(" NOP");
	__asm(" NOP");
	__asm(" NOP");

	while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);

	while( (EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY) || (EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY) );


			SYSCTL_SREEPROM_R = SYSCTL_SREEPROM_R0;	//Setting the last bit = 1, to reset the EEPROM.
			__asm(" NOP");
			__asm(" NOP");
			__asm(" NOP");
			SYSCTL_SREEPROM_R = 0X00;				//Clearing the last bit = 0, to again enable the EEPROM.
			__asm(" NOP");
			__asm(" NOP");
			__asm(" NOP");
			//while( (EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY) || (EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY) );
			//while( !(SYSCTL_PREEPROM_R & SYSCTL_PREEPROM_R0) ); 	//Wait untill the EEPROM moule is not ready.
						__asm(" NOP");
			__asm(" NOP");
			__asm(" NOP");
			__asm(" NOP");
			__asm(" NOP");
			__asm(" NOP");
			while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);			//Wait while EEPROM is busy.
			while( (EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY) || (EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY) );

}
uint32_t getdataEEPROM(uint32_t location)
{

	EEPROM_EEBLOCK_R = 0x00;										//Selecting the EEPROM Block 0.
	EEPROM_EEOFFSET_R = location;										//Selecting the first word to write.
																	//Reading the data in EEPROM data register.
	while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
	return(EEPROM_EERDWR_R);
}

void writeEEPROM(float value,uint32_t location)
{
	EEPROM_EEBLOCK_R = 0x00;										//Selecting the EEPROM Block 0.
	EEPROM_EEOFFSET_R = location;										//Selecting the first word to write.
	EEPROM_EERDWR_R = value;									//Reading the data in EEPROM data register.
	while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);

}
// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,D,E & F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure SSI0 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;           // turn-on SSI0 clocking
    GPIO_PORTA_DIR_R |= 0x2C;                        // make bits 2,3 and 5 outputs
    GPIO_PORTA_DR2R_R |= 0x2C;                       // set drive strength to 2mA
   	GPIO_PORTA_AFSEL_R |= 0x2C;                      // select alternative functions for MOSI, SCLK pins
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA5_SSI0TX | GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA3_SSI0FSS ; // map alt fns to SSI2
    GPIO_PORTA_DEN_R |= 0x2C;                        // enable digital operation on TX, CLK pins

    // Configure the SSI0 as a SPI master, mode 0, 16bit operation, 1 MHz bit rate
    SSI0_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI0_CR1_R = 0;                                  // select master mode
    SSI0_CC_R = 0;                                   // select system clock as the clock source
    SSI0_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI0_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode(SPH=0, SPO=0), 16-bit
    SSI0_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

    // Configure AN0 as an analog input
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    GPIO_PORTE_AFSEL_R |= 0x01;                      // select alternative functions for AN3 (PE0)
    GPIO_PORTE_DEN_R &= ~0x01;                       // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= 0x01;                      // turn on analog operation on pin PE0
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 3;                               // set first sample to AN3
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    SYSCTL_RCGCADC_R |= 2;                           // turn on ADC module 1 clocking
    GPIO_PORTD_AFSEL_R |= 0x02;                      // select alternative functions for AN6 (PD1)
    GPIO_PORTD_DEN_R &= ~0x02;                       // turn off digital operation on pin PD1
    GPIO_PORTD_AMSEL_R |= 0x02;                      // turn on analog operation on pin PD1
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 6;                               // set first sample to AN6
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    //TIMER1_TAILR_R = 680;                      	 	 // set load value to 800 for 20 MHz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);            // turn-on interrupt 37 (TIMER1A)
    NVIC_PRI5_R|= 0;

    // Configure Timer 2 as the time base
    	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;       // turn-on timer
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
        TIMER2_TAILR_R = 400000;                      	// set load value to 400000 for 100 Hz interrupt rate
        TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        NVIC_EN0_R |= 1 << (INT_TIMER2A-16);            // turn-on interrupt 37 (TIMER1A)
        NVIC_PRI5_R|=0xE0000000;
  }

  int16_t readAdc0Ss3()
  {
      ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
      while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
      return ADC0_SSFIFO3_R;                           // get single result from the FIFO
  }

 int16_t readAdc1Ss3()
  {
      ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
      while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
      return ADC1_SSFIFO3_R;                           // get single result from the FIFO
  }

// Blocking function that writes a serial character when the UART buffer is not full
void sendcommand(uint16_t command)
{
	SSI0_DR_R = command;               // write command
	while (SSI0_SR_R & SSI_SR_BSY);	  // wait for transmission to stop
}
void putsUart0(char* str);

/*void Timer1Isr()
{
	RED_LED = 1;
	phase=phase+d_phase;
	r=phase>>24;
	lutval=lut[r]+12288;
	sendcommand(lutval);
	TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
	RED_LED = 0;
}*/

void Timer1Isr()
{

	if(signal=="Square signal")
	{
			if(sqr==0)
			{
			sqval=(2048+amp*385*offset)+12288;
			sendcommand(sqval);
			sqr=1;
			TIMER1_TAILR_R = (40000000*dutyy)/freq;
			}
			else
			{
				sqval=(2048-amp*385*offset)+12288;
				sendcommand(sqval);
				sqr=0;
				TIMER1_TAILR_R = (40000000*duty)/freq;
			}

	}
	else
	{
	RED_LED = 1;
	phase=phase+d_phase;
	r=phase>>24;
	lutval=lut[r]+12288;
	sendcommand(lutval);
	RED_LED = 0;
	}
	TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag

}

void Timer2Isr()
{
	//if(freq1<freq2)
	if(count<30)

				{
						if(wait!=0)
						wait--;

						else if(wait==0)
					{
						freq1+=f;
						d_phase=(freq1/29500)*4294967296;
						phase=phase+d_phase;
						wait=(5/freq1)*500;
						count++;

						putsUart0("\r\n");
						putsUart0("FREQUENCY: ");
						sprintf(ftoa,"%3.2f",freq1);
						putsUart0(ftoa);

						vol_fast=readAdc1Ss3();
						newvol_fast=slope_fast*vol_fast;
						putsUart0("                         VOLTAGEf(db): ");

						voldb=20*log10(newvol_fast/amp);
						sprintf(ftoa,"%3.2f",voldb);
						putsUart0(ftoa);
						putsUart0("\r\n");

						if(voldb<=-3.00 && voldb >=-3.10)
							{
								fc=freq1;
							}
					}

				}

	TIMER2_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag

}

void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	int i;
    for (i = 0; i < strlen(str); i++)
	putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
void convert()
{
   for(cnt=0; rxbuf[cnt] != '\0'; cnt++)
   	    {
     			if(isalnum(rxbuf[cnt]))
     	        {
     	           if(isalpha(rxbuf[cnt]));
     	           rxbuf[cnt]=toupper(rxbuf[cnt]);
     	        }
     	        else if(rxbuf[cnt]=='.' || rxbuf[cnt]=='-')
     	        {

     	        }
     	        else
     	        rxbuf[cnt]=' ';

        }

    putsUart0( "\r\n Converted string: ");
    putsUart0(rxbuf);


	   for(cnt=0; rxbuf[cnt] != '\0'; cnt++)
	     {
	         if((cnt==0 && isalpha(rxbuf[0]))||(rxbuf[cnt-1]==' ' && isalpha(rxbuf[cnt])))
	         {field[i]='A';
	         fpos[i]=cnt;
	         i++;
	         }

	         else if((cnt==0 && isdigit(rxbuf[0]))||(rxbuf[cnt-1]==' ' && (isdigit(rxbuf[cnt])||rxbuf[cnt]=='-')))
	         {field[i]='N';
	         fpos[i]=cnt;
	         i++;
	         }
	     }
	   	 field_no= i;
	     //putsUart0( "\r\n field type:");
	     //putsUart0(field);

	     //putsUart0( "\r\n string location:");
	     //putsUart0(&rxbuf[fpos[1]]);

	   for(cnt=0; rxbuf[cnt] != '\0'; cnt++)
	     	    {
	     	         if(rxbuf[cnt]==' ' )
	     	         {
	     	        	 rxbuf[cnt]='\0';
	     	         }
	     	    }
	   //putsUart0( "\r\n string at a position :");
	   //putsUart0(&rxbuf[fpos[1]]);
}


bool iscommand(char* cmd,uint8_t args)
{
	bool TRUE= strcmp(cmd, &rxbuf[fpos[0]])==0;
	TRUE= TRUE && (field_no-1==args);
	return TRUE;
}

bool iscommand2(char* cmd, char* cmd2, uint8_t args)
{
	bool TRUE= strcmp(cmd, &rxbuf[fpos[0]])==0 && strcmp(cmd2, &rxbuf[fpos[1]])==0;
	TRUE= TRUE && (field_no==args) && field[0]=='A'&& field[1]=='A';
	return TRUE;

}
bool isnumber(pos)
{
	bool TRUE= field[pos]=='N';
	return TRUE;
}

float getnum(uint8_t pos)
{
	if (isnumber(pos))
	{
	integer=atof(&rxbuf[fpos[pos]]);
	return integer;
	}
	else
	putsUart0("\r\n Invalid freq/amp ");
	return 0;
}

void boolean()
{
	if (iscommand("RESET",0))
	{
		putsUart0("\r\n Reset CPU");
		  __asm("    .global _c_int00\n"
		        "    b.w     _c_int00");
	}

	/*else if (iscommand("SQUARE",3))
	{	square=1;
		modulation="Square signal";
		putsUart0("\r\n");
		putsUart0(modulation);
		freq=getnum(1);
		amp=getnum(2);
		duty=getnum(3);
		for(n=0;n<=255;n++)
		{	if(n<=(256*(duty/100)-1))
			lut[n]=2048-amp*385*cal;
			if(n>(256*(duty/100)-1))
			lut[n]=2048+amp*385*cal;
		}
		d_phase=(freq/29350)*4294967296;
		TIMER1_TAILR_R = 680;
		TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

	}*/

	else if (iscommand("SQUARE",3))
		{	sqr=1;
			signal="Square signal";
			putsUart0("\r\n");
			putsUart0(signal);
			freq=getnum(1);
			amp=getnum(2);
			duty=getnum(3);
			duty=duty/100;
			dutyy=1-duty;
			TIMER1_TAILR_R = (40000000*duty)/freq;
			TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

		}

	else if (iscommand("SINE",2))
	{
		signal="SINE signal";
		putsUart0("\r\n");
		putsUart0(signal);
		freq=getnum(1);
		amp=getnum(2);
		for(n=0;n<=255;n++)
		{
			lut[n]=2048+(amp*390*offset)*sin((n/N)*2*PI);
		}
		TIMER1_TAILR_R = 680;
		d_phase=(freq/29350)*4294967296;
	   TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer


			/*for(u=0;u<5;u++)
			{
					for(v=0;v<=255;v++)
			//while(1)
					{
						d_phase=(freq/58800)*4294967296;
						phase=phase+d_phase;
						r=phase>>24;
						lutval=lut[r]+12288;
						sendcommand(lutval);

					}

			}*/

		}

	else if (iscommand("SWEEP",2))
		{	//wait=0;
			count=0;
			signal="SWEEP SIGNAL";
			putsUart0("\r\n");
			putsUart0(signal);
			freq1=getnum(1);
			if(freq1==1)
			{
				wait=(3/freq1)*100;
			}
			else
			wait=(5/freq1)*100;
			freq2=getnum(2);
			f=(freq2-freq1)/30;
			TIMER1_TAILR_R = 680;
		   TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

		}
	else if (iscommand("SAWTOOTH",2))
		{
			signal="Sawtooth signal";
			putsUart0("\r\n");
			putsUart0(signal);
			freq=getnum(1);
			amp=getnum(2);

			min=amp*830*offset;
			sample=(amp*830*offset)/256;
			lut[0]=min;
			for(n=1;n<=255;n++)
			{
				min=min-sample; 		// 4096/256=16
				lut[n]=min;
			}

			/*min=amp*830*cal;
			sample=(amp*830*cal)/256;
			for(n=0;n<=255;n++)
			{
				lt=min-sample*n; 		// 4096/256=16
				lut[n]=lt;
			}*/
			TIMER1_TAILR_R = 680;

			d_phase=(freq/29350)*4294967296;
			 TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
		}

	else if (iscommand("DC",1))
		{
			signal="DC command";
			putsUart0("\r\n");
			putsUart0(signal);
			amp=getnum(1);
			if(amp>5)
			amp=5;
			else if(amp<-5)
			amp=-5;
			V_out=(amp - 5.2)/(-5.1);
			x=(4096*V_out)/2.048;
			Vout= 12288 + x;
			sendcommand(Vout);
		}

	else if (iscommand2("ZERO","INPUT",2))
			{
			putsUart0("ZEROING INPUT......");
				putsUart0("\r\n");
				zero_slow=readAdc0Ss3();
				zero_fast=readAdc1Ss3();
			}

	else if (iscommand2("CALIBRATE","INPUT",2))
			{
				putsUart0("\r\n CALIBRATING INPUT......");

				cal_slow=readAdc0Ss3();
				cal_fast=readAdc1Ss3();

				slope_slow=(4-0)/(cal_slow-zero_slow);
				slope_fast=(4-0)/(cal_fast-zero_fast);


			}

	else if (iscommand("VOLTAGE",0))
			{
				signal="New Voltage: ";
				putsUart0("\r\n");
				putsUart0(signal);

				vol_slow=readAdc0Ss3();
				vol_fast=readAdc1Ss3();

				newvol_slow=slope_slow*vol_slow;
				newvol_fast=slope_fast*vol_fast;

				sprintf(ftoa,"%3.2f",newvol_slow);
				putsUart0("\r\n slow (in) : ");
				putsUart0(ftoa);

				sprintf(ftoa,"%3.2f",newvol_fast);
				putsUart0("\r\n fast (in): ");
				putsUart0(ftoa);
			}
	else if (iscommand2("ZERO","OUTPUT",2))
				{
					putsUart0("\r\n ZEROING OUTPUT......");

					zero_slow_o=readAdc0Ss3();
					zero_fast_o=readAdc1Ss3();
				}

		else if (iscommand2("CALIBRATE","OUTPUT",2))
				{

					putsUart0("\r\nCALIBRATING OUTPUT......");

					cal_slow_o=readAdc0Ss3();
					cal_fast_o=readAdc1Ss3();

					slope_slow_o=(4-0)/(cal_slow_o-zero_slow_o);
					slope_fast_o=(4-0)/(cal_fast_o-zero_fast_o);

					vol_slow_o=readAdc0Ss3();
					vol_fast_o=readAdc1Ss3();

					newvol_slow_o=slope_slow_o*vol_slow_o;
					newvol_fast_o=slope_fast_o*vol_fast_o;

					Vavg=(newvol_slow_o+newvol_fast_o)/2;
					offset=amp/Vavg;


				}


		else if (iscommand("SAVE",0))
			{
				putsUart0("\r\n Saving in EEPROM.....");

				slope_slow=slope_slow*100000;
				slope_fast=slope_fast*100000;
				offset=offset*100000;
				writeEEPROM(slope_fast,0x01);
				writeEEPROM(slope_slow,0x02);
				writeEEPROM(offset,0x03);
			}


		else if (iscommand("GETDATA",0))
				{
					putsUart0("\r\n Fetching from EEPROM.....");
					slope_fast=getdataEEPROM(0x01);
					slope_fast=slope_fast/100000;
					slope_slow=getdataEEPROM(0x02);
					slope_slow=slope_slow/100000;
					offset=getdataEEPROM(0x03);
					offset=offset/100000;
				}
		else if (iscommand("LCR",0))
				{
					/*putsUart0("\r\n Cut-off frequency: ");
					sprintf(ftoa,"%3.2f",fc);
					putsUart0(ftoa);*/
					float r=1000;
					cap=1/(2*PI*fc*r);
					putsUart0("\r\n Capacitance: ");
					sprintf(ftoa,"%3.9f",cap);
					putsUart0(ftoa);

			}
	else
	putsUart0("\r\n Invalid signal Command ");
}

int main(void)
{
	// Initialize hardware
	initHw();
	initEEPROM();

	   GREEN_LED ^= 1;
	   waitMicrosecond(500000);
	   GREEN_LED ^= 1;



	putsUart0( "\n\rEnter the string :");
    while(1)
    {

    	ch = getcUart0();

    	if(ch=='\r'||ch=='\n')
    	{ if(cnt==0)
    		{putsUart0("\r\n Invalid Command ");
    		putsUart0("\r\n\r\n Enter the string: ");
    		}

    	else
    	  {	rxbuf[cnt]='\0';
    	  	//putsUart0( "\r\n You entered: ");
    	  	//putsUart0(rxbuf);
    	  	convert();
    	  	boolean();
    	  	i=cnt=0;
    	  	memset(field,0,20);
    	  	memset(fpos,0,20);
    	  	memset(rxbuf,0,81);
    	  	putsUart0( "\n\r\n\r Enter new string :");
    		//RED_LED = 0;

      	  }
       	}
    	else if(ch=='\b')
    	  	{
    			if (cnt>0)
    			cnt--;

    	  	}

    	else if(ch>=32)
    	rxbuf[cnt++]=ch;
    	{
    		if(strlen(rxbuf)==80)
    		{	rxbuf[cnt]='\0';
    			putsUart0( "\r\n You entered: ");
    			putsUart0(rxbuf);
    			convert();
    		}
    	}

    }

}


