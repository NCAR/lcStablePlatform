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
#define PLATFORM  BOTTOM   // Set to TOP or BOTTOM to select appropriate coeffs

#define PITCH_LOOPIN_MIN     -5  // Operational Range, min, int
#define PITCH_LOOPIN_MAX      5  // Operational Range, max, int
#define PITCH_LOOPERRK_MIN -100  // Loop Error saturation, min, int
#define PITCH_LOOPERRK_MAX  100  // Loop Error saturation, max, int

#define ROLL_LOOPIN_MIN     -10  // Operational Range, min, int
#define ROLL_LOOPIN_MAX      10  // Operational Range, max, int
#define ROLL_LOOPERRK_MIN  -200  // Loop Error saturation, min, int
#define ROLL_LOOPERRK_MAX   200  // Loop Error saturation, max, int

#define LOOP_K             32.0  // Loop Gain, float
#define LOOP_TC           24000  // Loop Time Dominant Pole Time Constant, 2*tau/T [s/s]
#define LOOP_TCP          24001  // 1 + LOOP_TC
#define LOOP_TCN         -23999  // 1 - LOOP_TC
//#define LOOP_TCP          30001  // 1 + LOOP_TC
//#define LOOP_TCN         -29999  // 1 - LOOP_TC
#define PWM_QUANT             3  // PWM Code Quantization to 2^PWM_QUANT, int
#define FB_AVE               50  // Feedback Averaging, # Samples
#define GAIN_ADJ         1.0313  // Gain factor to compensate for low loop gain, = (32+1)/32

#if (PLATFORM == BOTTOM) // Update 6/19/2018

#define PITCH_POLARITY      1.0  // Set pitch equation polarity (+1.0) for bottom platform, float
#define PITCH_PWM_MIN      5150  // PWM code min limit, int
#define PITCH_PWM_MAX      8850  // PWM code max limit, int
#define PITCH_PWM_GAIN   8*46.0  // PWM to Actuator position angle gain, float
#define PITCH_PWM_OFFSET   7000  // PWM to Actuator position angle offset (est.), int
#define PITCH_FB_GAIN    0.0308  // ADC output to attitude angle feedback gain, float
#define PITCH_FB_OFFSET   347.0  // ADC output to attitude angle feedback offset, float

#define ROLL_POLARITY       1.0  // Set roll equation polarity (+1.0) for bottom platform, float
#define ROLL_PWM_MIN       5480  // PWM code min limit, int
#define ROLL_PWM_MAX       8360  // PWM code max limit, int
#define ROLL_PWM_GAIN    4*30.0  // PWM to Actuator position angle gain (est.), float
#define ROLL_PWM_OFFSET    6920  // PWM to Actuator position angle offset (est.), int
#define ROLL_FB_GAIN     0.0541  // ADC output to attitude angle feedback gain, float
#define ROLL_FB_OFFSET    376.0  // ADC output to attitude angle feedback offset, float

#elif (PLATFORM == TOP) // Updated 6/19/2018

#define PITCH_POLARITY     -1.0  // Set pitch equation polarity (+1.0) for bottom platform, float
#define PITCH_PWM_MIN      5460  // PWM code min limit, int
#define PITCH_PWM_MAX      8540  // PWM code max limit, int
#define PITCH_PWM_GAIN   8*39.0  // PWM to Actuator position angle gain, float
#define PITCH_PWM_OFFSET   7000  // PWM to Actuator position angle offset (est.), int
#define PITCH_FB_GAIN         ?  // ADC output to attitude angle feedback gain, float
#define PITCH_FB_OFFSET       ?  // ADC output to attitude angle feedback offset, float

#define ROLL_POLARITY       1.0  // Set roll equation polarity (+1.0) for bottom platform, float
#define ROLL_PWM_MIN       4590  // PWM code min limit, int
#define ROLL_PWM_MAX       8750  // PWM code max limit, int
#define ROLL_PWM_GAIN    4*43.0  // PWM to Actuator position angle gain (est.), float
#define ROLL_PWM_OFFSET    6670  // PWM to Actuator position angle offset (est.), int
#define ROLL_FB_GAIN          ?  // ADC output to attitude angle feedback gain, float
#define ROLL_FB_OFFSET        ?  // ADC output to attitude angle feedback offset, float

#else
#endif



static char Buf[100]; // A2D data buffer

long  data;
unsigned char  f_10ms, f_500us, f_50ms;
unsigned int   f_1000ms;
const    char  ascii[] = "0123456789ABCDEF";
short motor1_pos, motor2_pos, ovcr1, ovcr2;

float attitude;

float pitch_attitude;
float pitch_loopInput;
float pitch_loopErrK;
float pitch_loopErrK_d;
float pitch_loopErrK_filt;
float pitch_loopErrK_filt_d;
short pitch_pwm;
unsigned short pitch_encoder;
float pitch_attitudePos;
float pitch_posFilter;
unsigned short pitch_decCntr;
float pitch_loopFB;

float roll_attitude;
float roll_loopInput;
float roll_loopErrK;
float roll_loopErrK_d;
float roll_loopErrK_filt;
float roll_loopErrK_filt_d;
short roll_pwm;
unsigned short roll_encoder;
float roll_attitudePos;
float roll_posFilter;
unsigned short roll_decCntr;
float roll_loopFB;

unsigned short data1, data2, data3, data4;
unsigned int i;

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
  f_1000ms = 0;
	attitude = 0.0;
	pitch_attitude  = 0;
	roll_attitude   = 0;
	pitch_loopInput = 0;
	roll_loopInput  = 0;
	pitch_loopErrK  = 0;
	roll_loopErrK   = 0;
	pitch_loopErrK_d = 0;
	roll_loopErrK_d  = 0;
	pitch_loopErrK_filt = 0;
	roll_loopErrK_filt  = 0;
	pitch_loopErrK_filt_d = 0;
	roll_loopErrK_filt_d  = 0;
  pitch_pwm = PITCH_PWM_OFFSET;
  roll_pwm  = ROLL_PWM_OFFSET; 
  pitch_encoder = 0x180;  // Initialization offsets
  roll_encoder =  0x162;	
	pitch_attitudePos = PITCH_FB_OFFSET;
	roll_attitudePos  = ROLL_FB_OFFSET;
	pitch_posFilter = 0;
	roll_posFilter = 0;
	pitch_decCntr = 0;
	roll_decCntr = 0;
	pitch_loopFB = 0;
	roll_loopFB  = 0;
	

   
  PWM_Init(); // PWM Timer Initialization
  Uart0_Init();
  ADC_Init(); // ADC Initialization (internal A/D)
  T1_Init(); // 10 msec tick
  SPI_Init(); // initialize SPI 
  
  while (1) // Loop forever
  {
		IO0SET = 0x00080000;		 //toggle debug
    IO0CLR = 0x00080000;

    //ovcr1 = read_adc(1);  
    //ovcr2 = read_adc(4);
    //motor2_pos = read_adc(8);
    //motor1_pos = read_adc(2);
    pitch_encoder = read_adc(8);
    roll_encoder  = read_adc(2);

		//if ( ovcr1 > MAX_IM) {   // Check motor overcurrent
		//  VICIntEnClr = 0xFFFFFFFF;    //disable all interrupts
    //  PWMMR4 = 0;
    //  PWMMR6 = 0;
    //  PWMLER = 0x7F; // enable PWM4-PWM6 match latch (reload
    //  PrintString("\r\nMotor Current Overload\r\n");
    //  while (1); // wait for a RESET
		//}

   if (f_500us == 1) {		 //T1 isr every 500_u
	   f_500us = 0;
     
//		sprintf(Buf,".");
//		PrintString(Buf);

		// Calculate Loop Error and saturate
		pitch_loopErrK = LOOP_K*(pitch_loopInput - pitch_loopFB);
		if (pitch_loopErrK > PITCH_LOOPERRK_MAX) pitch_loopErrK = PITCH_LOOPERRK_MAX;
		if (pitch_loopErrK < PITCH_LOOPERRK_MIN) pitch_loopErrK = PITCH_LOOPERRK_MIN;

		// Filter with dominant pole
		pitch_loopErrK_filt = (pitch_loopErrK + pitch_loopErrK_d - pitch_loopErrK_filt_d * LOOP_TCN) / LOOP_TCP;
		pitch_loopErrK_d = pitch_loopErrK;            // delay a sample period
		pitch_loopErrK_filt_d = pitch_loopErrK_filt;
		
		// Convert Loop Error to PWM conversion factors, quantize, and limit
		pitch_pwm = (short) (pitch_loopErrK_filt * PITCH_PWM_GAIN + PITCH_PWM_OFFSET);
		if (pitch_pwm > PITCH_PWM_MAX) pitch_pwm = (short) PITCH_PWM_MAX;
		if (pitch_pwm < PITCH_PWM_MIN) pitch_pwm = (short) PITCH_PWM_MIN;
		pitch_pwm = (pitch_pwm >> PWM_QUANT) << PWM_QUANT; // Truncate bits
		
		// Convert Encoder ADC Output (10-bit) to attitude position
		pitch_attitudePos = ((float) pitch_encoder - PITCH_FB_OFFSET) * PITCH_FB_GAIN;

		// Boxcar filter and decimate the attitude position
		pitch_posFilter = pitch_posFilter + pitch_attitudePos; // Accumulate
		pitch_decCntr = pitch_decCntr + 1;
		if (pitch_decCntr >= FB_AVE) {
			pitch_loopFB = pitch_posFilter / (float) FB_AVE; // Update the feedback value periodically per FB_AVE
			pitch_posFilter = 0.0;
			pitch_decCntr = 0;
		}

		PWMMR4 = pitch_pwm;
		PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
		IO0SET = 0x00080000;		 //toggle debug
		IO0CLR = 0x00080000;

		// Calculate Loop Error and saturate
		roll_loopErrK = LOOP_K*(roll_loopInput - roll_loopFB);
		if (roll_loopErrK > ROLL_LOOPERRK_MAX) roll_loopErrK = ROLL_LOOPERRK_MAX;
		if (roll_loopErrK < ROLL_LOOPERRK_MIN) roll_loopErrK = ROLL_LOOPERRK_MIN;

		// Filter with dominant pole
		roll_loopErrK_filt = (roll_loopErrK + roll_loopErrK_d - roll_loopErrK_filt_d * LOOP_TCN) / LOOP_TCP;
		roll_loopErrK_d = roll_loopErrK;            // delay a sample period
		roll_loopErrK_filt_d = roll_loopErrK_filt;
		
		// Convert Loop Error to PWM conversion factors, quantize, and limit
		roll_pwm = (short) (roll_loopErrK_filt * ROLL_PWM_GAIN + ROLL_PWM_OFFSET);
		if (roll_pwm > ROLL_PWM_MAX) roll_pwm = (short) ROLL_PWM_MAX;
		if (roll_pwm < ROLL_PWM_MIN) roll_pwm = (short) ROLL_PWM_MIN;
		roll_pwm = (roll_pwm >> PWM_QUANT) << PWM_QUANT; // Truncate bits
		
		// Convert Encoder ADC Output (10-bit) to attitude position
		roll_attitudePos = ((float) roll_encoder - ROLL_FB_OFFSET) * ROLL_FB_GAIN;

		// Boxcar filter and decimate the attitude position
		roll_posFilter = roll_posFilter + roll_attitudePos; // Accumulate
		roll_decCntr = roll_decCntr + 1;
		if (roll_decCntr >= FB_AVE) {
			roll_loopFB = roll_posFilter / (float) FB_AVE; // Update the feedback value periodically per FB_AVE
			roll_posFilter = 0.0;
			roll_decCntr = 0;
		}

		PWMMR6 = roll_pwm;
		PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
		IO0SET = 0x00080000;		 //toggle debug
		IO0CLR = 0x00080000;

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
          if((data & SSM) != SSM) break;

					pitch_attitude = PITCH_POLARITY * attitude;  // Set polarity
				
					// Apply gain adjustment and Saturate loop input
					pitch_loopInput = GAIN_ADJ * pitch_attitude;
          if (pitch_loopInput > PITCH_LOOPIN_MAX) pitch_loopInput = PITCH_LOOPIN_MAX;
					if (pitch_loopInput < PITCH_LOOPIN_MIN) pitch_loopInput = PITCH_LOOPIN_MIN;
					
				/*					
          if (pitch_attitude >= 0.0) {
            pitch_pwm =  (short)(pitch_attitude * PITCH_PWM_GAIN + PITCH_PWM_OFFSET);
            if (pitch_pwm > PITCH_PWM_MAX) pitch_pwm = PITCH_PWM_MAX;
          } 
          else {
            pitch_pwm =  (short)(pitch_attitude * PITCH_PWM_GAIN + PITCH_PWM_OFFSET);
            if (pitch_pwm < PITCH_PWM_MIN) pitch_pwm = PITCH_PWM_MIN;
          }
          PWMMR4 = pitch_pwm;
          PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
      	  IO0SET = 0x00080000;		 //toggle debug
          IO0CLR = 0x00080000;
				*/
        case ROLL_LABEL:
          if((data & SSM) != SSM) break;
          roll_attitude = ROLL_POLARITY * attitude;				

					// Apply gain adjustment and Saturate loop input
  				roll_loopInput = GAIN_ADJ * roll_attitude;
          if      (roll_loopInput > ROLL_LOOPIN_MAX) roll_loopInput = ROLL_LOOPIN_MAX;
					else if (roll_loopInput < ROLL_LOOPIN_MIN) roll_loopInput = ROLL_LOOPIN_MIN;
/*
					if (roll_attitude >= 0.0) {
            roll_pwm =  (short)(roll_attitude * ROLL_PWM_GAIN + ROLL_PWM_OFFSET);
            if (roll_pwm > ROLL_PWM_MAX) roll_pwm = ROLL_PWM_MAX;
          }
          else {
            roll_pwm =  (short)(roll_attitude * ROLL_PWM_GAIN + ROLL_PWM_OFFSET);
            if (roll_pwm < ROLL_PWM_MIN) roll_pwm = ROLL_PWM_MIN;
          }
          PWMMR6 = roll_pwm;
          PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
          IO0SET = 0x00080000;		 //toggle debug
          IO0CLR = 0x00080000;
*/
        default:
          break;
        }
      } 

      f_10ms += 1;
      f_50ms += 1;																				 
      f_1000ms += 1;
    }

    if (f_10ms == 20) {       // every 10 mseconds (100Hz)
      f_10ms = 0;
    }

    if (f_50ms == 100) {			// 20 Hz TE cooler control
	    f_50ms = 0;      
    }

    if (f_1000ms == 2000) {				// 1000 mseconds
      f_1000ms = 0;
//      roll_pwm = -roll_pwm;
//      sprintf(Buf,"0x%08x P:%.3f,%i R:%.3f,%i\r\n", data, pitch_attitude, pitch_pwm, roll_attitude, roll_pwm);
//      PrintString(Buf);
//      sprintf(Buf,"Err:%.3f,Erf:%.3f,Enc:%d,FB:%.2f\r\n", pitch_loopErrK, pitch_loopErrK_filt, pitch_encoder, pitch_loopFB);
//      PrintString(Buf);
//      sprintf(Buf,"Err:%.3f,Erf:%.3f,Enc:%d,FB:%.2f\r\n", roll_loopErrK, roll_loopErrK_filt, roll_encoder, roll_loopFB);
//      PrintString(Buf);
      sprintf(Buf,"Pin:%.2f,Ppwm:%d,Penc:%d,Pfb:%.2f,Rin:%.2f,Rpwm:%d,Renc:%d,Rfb:%.2f\r\n",  
			    pitch_attitude, pitch_pwm, pitch_encoder, pitch_loopFB, roll_attitude, roll_pwm, roll_encoder, roll_loopFB); // 89-char
      PrintString(Buf);
//      sprintf(Buf,"P(M2):0x%08x, R(M1):0x%08x\r\n", pitch_encoder, roll_encoder);
//      PrintString(Buf);
//      sprintf(Buf,"ROLL  %08x %f %i\r\n", data, roll_attitude, roll);
//      PrintString(Buf);
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
  ADCR  = 0x00200F00; // initialise ADC: ADC operational, no burst, select AIN 0-3 F=clkdiv
}
unsigned short read_adc(unsigned char channel)
{
  ADCR &= 0x00FFFFF0;                // Clear any previous conversion and channel
  ADCR |=  channel;                  // Set the channel
  ADCR |= 0x01000000;                // start conversion
  while((ADDR & 0x80000000) == 0);   //wait for conversion done
  return (ADDR & 0x0000FFFF) >> 6;
}

void PWM_Init(void)                           
{

  PWMPR  = 12; // prescaler to 60, timer runs at 60 MHz / 12 = 5 MHz
  PWMPC  = 0; // prescale counter to 0
  PWMTC  = 0; // reset timer to 0
  PWMMR0 = 15000; // -> PMW base frequency = 5 MHz / 15000 = 333.3 Hz (3ms)
  PWMMR4 = pitch_pwm; // Motor 1 pulse, 1 count is 0.1 deg
  PWMMR6 = roll_pwm; // Motor 2 pulse
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



