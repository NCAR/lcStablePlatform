#include "LPC21xx.h"
#include <STDIO.H>
//#include <STRING.H>

#define MAX_IM      0x3F0
#define SPI_OK      0 // transfer ended No Errors
#define SPI_BUSY    1 // transfer busy
#define SPI_ERROR   2 // SPI error
#define PITCH_LABEL 0xD4
#define ROLL_LABEL  0xD5
#define SSM         0x60000000

#define TOP 1
#define BOTTOM 0
#define PLATFORM  TOP   // Set to TOP or BOTTOM to select appropriate coeffs

#if (PLATFORM == BOTTOM) // Update 6/19/2018
#define PITCH_POLARITY  1.0   // Set pitch equation polarity (+1.0) for bottom platform, float
#define PITCH_GAIN      370.0 // Gain of pitch compensation, float
#define PITCH_OFFSET    7000  // Pitch compensartion offset, int
#define PITCH_MAXLIM    8850  // Maximum Servo PWM for pitch motor, int, ~PITCH_OFFSET+5*PITCH_GAIN
#define PITCH_MINLIM    5150  // Minimum Servo PWM for pitch motor, int, ~PITCH_OFFSET-5*PITCH_GAIN
#define ROLL_POLARITY   1.0   // Set roll equation polarity (+1) for bottom platform, float
#define ROLL_GAIN       120.0 // Gain of roll compensation, float
#define ROLL_OFFSET     6920  // Roll compensartion offset, int
#define ROLL_MAXLIM     8360  // Maximum Servo PWM for roll motor, int, ~ROLL_OFFSET+12*ROLL_GAIN
#define ROLL_MINLIM     5480  // Minimum Servo PWM for roll motor, int, ~ROLL_OFFSET-12*ROLL_GAIN

#elif (PLATFORM == TOP) // Updated 6/19/2018
#define PITCH_POLARITY  -1.0  // Set pitch equation polarity (-1.0) for top platform, float
#define PITCH_GAIN      308.0 // Gain of pitch compensation, float
#define PITCH_OFFSET    7000  // Pitch compensartion offset, int
#define PITCH_MAXLIM    8540  // Maximum Servo PWM for pitch motor, int, ~PITCH_OFFSET+5*PITCH_GAIN
#define PITCH_MINLIM    5460  // Minimum Servo PWM for pitch motor, int, ~PITCH_OFFSET-5*PITCH_GAIN
#define ROLL_POLARITY   1.0   // Set roll equation polarity (+1) for top platform, float
#define ROLL_GAIN       173.0 // Gain of roll compensation, float
#define ROLL_OFFSET     6670  // Roll compensartion offset, int
#define ROLL_MAXLIM     8746  // Maximum Servo PWM for roll motor, int, ~ROLL_OFFSET+12*ROLL_GAIN
#define ROLL_MINLIM     4594  // Minimum Servo PWM for roll motor, int, ~ROLL_OFFSET-12*ROLL_GAIN

#else
#endif

static char Buf[100]; // A2D data buffer

long  data;
unsigned char  f_10ms, f_500us, f_50ms, f_100ms;
const    char  ascii[] = "0123456789ABCDEF";
short motor1_pos, motor2_pos, pitch, roll, ovcr1, ovcr2;
float attitude;
float pitch_attitude;
float roll_attitude;
unsigned short data1, data2, data3, data4;

void Uart0_Init(void);
void ADC_Init(void);
void T1_Init(void);
void PWM_Init(void);
void SPI_Init(void);
void PrintByte(unsigned char b);
void PrintString(const char *s);
unsigned short read_adc(unsigned char);
//unsigned short SPI0_Read(void);
__irq void T1_Isr(void);

 int main (void)
{
  VICIntEnClr = 0xFFFFFFFF; // disable all interrupts!
  VPBDIV = 1;
  PINSEL0 = 0x000A1505;
  PINSEL1 = 0x15400000; 
  IO1DIR =  0x0C000000;  
  IO0DIR =  0x00080751;  //Set outputs
  IO0CLR =  0x00080751;  //Clear all outputs.
//  while(1) {
//		IO0SET = 0x00080000;		 //toggle debug 
//    IO0CLR = 0x00080000;
//  }
  PWMMR4 = 7500;  
  PWMMR6 = 7500;  
  f_500us = 0;
  f_10ms = 0;
  f_50ms = 0;
  f_100ms = 0;
  pitch = PITCH_OFFSET;
  roll = ROLL_OFFSET;  
   
  PWM_Init(); // PWM Timer Initialization
  Uart0_Init();
  ADC_Init(); // ADC Initialization (internal A/D)
  T1_Init(); // 10 msec tick
  SPI_Init(); // initialize SPI 
  
  while (1) // Loop forever
  {
		IO0SET = 0x00080000;		 //toggle debug
    IO0CLR = 0x00080000;
/*
    ovcr1 = read_adc(1);  
    ovcr2 = read_adc(4);
    motor1_pos = read_adc(2);
    motor2_pos = read_adc(8);

		if ( ovcr1 > MAX_IM) {   // Check motor overcurrent
		  VICIntEnClr = 0xFFFFFFFF;    //disable all interrupts
      PWMMR4 = 0;
      PWMMR6 = 0;
      PWMLER = 0x7F; // enable PWM4-PWM6 match latch (reload
      PrintString("\r\nMotor Current Overload\r\n");
      while (1); // wait for a RESET
		}
*/
   if (f_500us == 1) {		 //T1 isr every 500_u
	   f_500us = 0;
     
      while ((IO0PIN & 0x02000000) == 0) { //FIFO not empty, rflag P0.25
        data = 0x00000000;      

        IOCLR0 = 0x00000400;     // /CS = 0, P0.10

        S0SPDR = 0x08;           //Read next ARINC fifo word (32 bits)
        while ((S0SPSR & 0x80) == 0); //busy

      	S0SPDR = 0x0;
        while ((S0SPSR & 0x80) == 0); //busy
        data1 = S0SPDR;
        data = (data << 8) | data1; 

      	S0SPDR = 0x0;
        while ((S0SPSR & 0x80) == 0); //busy
        data2 = S0SPDR;
        data = (data << 8) | data2; 

      	S0SPDR = 0x0;
        while ((S0SPSR & 0x80) == 0); //busy
        data3 = S0SPDR;
        data = (data << 8) | data3; 

      	S0SPDR = 0x0;
        while ((S0SPSR & 0x80) == 0); //busy
        data4 = S0SPDR;
        data = (data << 8) | data4; 

        IOSET0 = 0x00000400;

        attitude = (float)((data<<3>>13)) * .0006866455078125;
        switch (data & 0xFF) {
        case PITCH_LABEL:
          pitch_attitude = PITCH_POLARITY * attitude;
          if((data & SSM) != SSM) break;
          if (pitch_attitude >= 0.0) {
            pitch =  (short)(pitch_attitude * PITCH_GAIN + PITCH_OFFSET);
            if (pitch > PITCH_MAXLIM) pitch = PITCH_MAXLIM;
          } 
          else {
            pitch =  (short)(pitch_attitude * PITCH_GAIN + PITCH_OFFSET);
            if (pitch < PITCH_MINLIM) pitch = PITCH_MINLIM;
          }
          PWMMR4 = pitch;
          PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
      	  IO0SET = 0x00080000;		 //toggle debug
          IO0CLR = 0x00080000;
        case ROLL_LABEL:
          roll_attitude = ROLL_POLARITY * attitude;
          if((data & SSM) != SSM) break;
          if (roll_attitude >= 0.0) {
            roll =  (short)(roll_attitude * ROLL_GAIN + ROLL_OFFSET);
            if (roll > ROLL_MAXLIM) roll = ROLL_MAXLIM;
          }
          else {
            roll =  (short)(roll_attitude * ROLL_GAIN + ROLL_OFFSET);
            if (roll < ROLL_MINLIM) roll = ROLL_MINLIM;
          }
          PWMMR6 = roll;
          PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
          IO0SET = 0x00080000;		 //toggle debug
          IO0CLR = 0x00080000;
        default:
          break;
        }
      } 

      f_10ms += 1;
      f_50ms += 1;																				 
      f_100ms += 1;
    }

    if (f_10ms == 20) {       // every 10 mseconds (100Hz)
      f_10ms = 0;
    }

    if (f_50ms == 100) {			// 20 Hz TE cooler control
	    f_50ms = 0;      
    }

    if (f_100ms == 200) {				// 100 mseconds
      f_100ms = 0;
      roll = -roll;
      sprintf(Buf,"PITCH %08x %f %i\r\n", data, pitch_attitude, pitch);
      PrintString(Buf);
      sprintf(Buf,"ROLL  %08x %f %i\r\n", data, roll_attitude, roll);
      PrintString(Buf);
//    sprintf(Buf,"%08x\r\n",data);
//    PrintString(Buf);
/*
      if(PWMMR4 == 7794) PWMMR4 = 7200;
      else PWMMR4 = 7794;
      if(PWMMR6 == 7794) PWMMR6 = 7200;
      else PWMMR6 = 7794;
      PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
*/
    }
  }
}    

void T1_Init(void)
{
  VICVectAddr2 = (unsigned int) &T1_Isr;
  VICVectCntl2 = 0x25; //  enabled
  VICIntEnable |= 0x20; // Channel#5 is the Timer 1
  T1MR0 = 30000; // = 0.5 msec / 16,67 nsec
  T1MCR = 3; // Interrupt on Match0, reset timer on match
// Pclk = 60 MHz, timer count = 16,67 nsec
  T1TC  = 0; // reset Timer counter
  T1TCR = 1; // enable Timer
}

void ADC_Init(void)
{
  ADCR  = 0x00200F00; // initialise internal ADC, select AIN 0-3 F=clkdiv
}
unsigned short read_adc(unsigned char channel)
{
  ADCR &= 0xFFFFFFF0;
  ADCR |=  channel;
  ADCR |= 0x01000000; // start conversion now 
  while((ADDR & 0x80000000) == 0);   //wait for conversion done
  return (ADDR & 0x0000FFFF) >> 6;
}

void PWM_Init(void)                           
{

  PWMPR  = 12; // prescaler to 60, timer runs at 60 MHz / 12 = 5 MHz
  PWMPC  = 0; // prescale counter to 0
  PWMTC  = 0; // reset timer to 0
  PWMMR0 = 15000; // -> PMW base frequency = 5 MHz / 15000 = 333.3 Hz
  PWMMR4 = pitch; // Motor 1 pulse, 1 count is 0.1 deg
  PWMMR6 = roll; // Motor 2 pulse
  PWMMCR = 0x00000002; // 
  PWMPCR = 0x5000; // enable PWM4 PWM6 outputs
  PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
  PWMTCR = 0x09; // enable PWM mode and start timer
}

void SPI_Init(void)
{
  unsigned char i;

  S0SPCR   = 0x20;               /* 0010 1000     Initialize SPI hardware:
                                    |||| ||||
                                    |||| | -----> reserved
                                    ||||  ------> SPI clock phase select
                                    ||| --------> SPI clock polarity = low when idle
                                    || ---------> SPI master mode
                                    | ----------> SPI data order = msb first
                                     -----------> SPI interrupt disabled        */
  S0SPCCR = 60; // SCK = 1 MHz, cclk = 60 MHz, counter > 8 and even
 
  IOCLR0 = 0x00000400;     // SSEL = 1, P0.10
  S0SPDR = 0x01;           //ARINC Reset cmd.
  while ((S0SPSR & 0x80) == 0); //busy
  IOSET0 = 0x00000400;
  IOCLR0 = 0x00000400;     // SSEL = 1, P0.10
  S0SPDR = 0x10;           //ARINC control register write
  while ((S0SPSR & 0x80) == 0); //busy
  S0SPDR = 0x00;           //ARINC control register values 0x08
  while ((S0SPSR & 0x80) == 0); //busy
  S0SPDR = 0x24;           //ARINC control register values 0x26
  while ((S0SPSR & 0x80) == 0); //busy
  IOSET0 = 0x00000400;
//  IOCLR0 = 0x00000400;     // SSEL = 1, P0.10
//  S0SPDR = 0x0A;
//  while ((S0SPSR & 0x80) == 0);
//  status = S0SPDR;
//  IOSET0 = 0x00000400;

  IOCLR0 = 0x00000400;     // SSEL = 1, P0.10
  S0SPDR = 0x06;           //ARINC label set.
  while ((S0SPSR & 0x80) == 0); //busy
  for (i = 0; i < 5; i++) {
    S0SPDR = 0x00;         //ARINC labels not selected
    while ((S0SPSR & 0x80) == 0); //busy
  }
  S0SPDR = 0x30;           // ARINC pitch and roll lables selected WAS 0X30
  while ((S0SPSR & 0x80) == 0); //busy
  for (i=0 ; i<26; i++) {
    S0SPDR = 0x00;         //ARINC labels not selected
    while ((S0SPSR & 0x80) == 0); //busy
  }

  IOSET0 = 0x00000400;

}

void Uart0_Init(void)
{

  U0LCR = 0x83;                              /* 8 bits, no Parity, 1 Stop bit*/
  U0DLL = 0x21;                              /* 115.2Kbaud for 60MHz PCLK Clock         */
  U0DLM = 0x00;
  U0FCR = 0x07;
  U0LCR = 0x03;                              /* DLAB = 0                     */
}

//***************************************************************************
//* Program DAC for detector Peltier cooler
//*************************************************************************

static void ua_outchar(char c)
{
  U0THR = c;
  while(!(U0LSR & 0x40));
}

void PrintByte(unsigned char b)
{
  ua_outchar(ascii[b >> 4]);
  ua_outchar(ascii[b & 0x0f]);
}

void PrintString(const char *s)
{
  while(*s)
  {
//    if(*s == '\n') ua_outchar('\r');
    ua_outchar(*s);
    s++;
  }
}

__irq void T1_Isr(void)  // Timer 1 ISR every 0.5 msec
{
  f_500us = 1; // toggles every 0.5 mseconds

  T1IR = 0x01; // reset interrupt flag
  VICVectAddr = 0; // reset VIC
}

//static unsigned char ADC_Read(void)
//{
//  unsigned int i;
//  ADCR = 0x00200302; // Init ADC (Pclk = 12MHz) and select channel AD0.1
//  ADCR |= 0x01000000; // Start A/D Conversion
//  do
//  {
//    i = ADDR; // Read A/D Data Register
//  } while ((i & 0x80000000) == 0); // Wait for end of A/D Conversion
//  return (i >> 8) & 0x00FF; // bit 8:15 is 8-bit AD value
//}



