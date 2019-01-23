#include "LPC21xx.h"
#include <STDIO.H>
#include <STRING.H>

#define FW_REV 20190122

/* Platform Selection */
#define TOP 1
#define BOTTOM 0
#define PLATFORM  BOTTOM   // MUST SET: TOP or BOTTOM to select appropriate coeffs

/* Platform Coefficients */
#if (PLATFORM == BOTTOM) // Updated 1/22/2019

#define PITCH_POLARITY          1.0  // Set pitch equation polarity (+1.0) for bottom platform, float
#define PITCH_BI_TRANSLATION   -2.9  // Translates Pitch from INS ref frame to platform base ref frame
                                 //   Negative correction polarity corrects positive aircraft pitch
                                 //   Platform mounting estimated -2.9deg relative to airframe
#define PITCH_PWM_MIN      5150  // PWM code min limit, int
#define PITCH_PWM_MAX      8850  // PWM code max limit, int
#define PITCH_PWM_GAIN    345.3  // PWM to Actuator position angle gain, float
#define PITCH_PWM_OFFSET   6888  // PWM to Actuator position angle offset, flat to platform (est.), int
#define PITCH_FB_GAIN    0.0311  // ADC output to attitude angle feedback gain, float
#define PITCH_FB_OFFSET   338.0  // ADC output to attitude angle feedback offset, flat to platform, float

#define ROLL_BI_TRANSLATION 0.0  // Translates Pitch from INS ref frame to platform base ref frame
#define ROLL_PWM_MIN       5480  // PWM code min limit, int
#define ROLL_PWM_MAX       8360  // PWM code max limit, int
#define ROLL_PWM_GAIN     111.1  // PWM to Actuator position angle gain (est.), float
#define ROLL_PWM_OFFSET    6840  // PWM to Actuator position angle offset, flat to platform (est.), int
#define ROLL_FB_GAIN     0.0555  // ADC output to attitude angle feedback gain, float
#define ROLL_FB_OFFSET    367.9  // ADC output to attitude angle feedback offset, flat to platform, float

#elif (PLATFORM == TOP) // Updated 1/22/2018

#define PITCH_POLARITY       -1.0  // Set pitch equation polarity (+1.0) for bottom platform, float
#define PITCH_BI_TRANSLATION -2.9  // Translates Pitch from INS ref frame to platform base ref frame
                                 //   Negative to correct positive aircraft pitch
                                 //   Platform mounting estimated -2.9deg relative to airframe
#define PITCH_PWM_MIN      5460  // PWM code min limit, int
#define PITCH_PWM_MAX      8540  // PWM code max limit, int
#define PITCH_PWM_GAIN    306.1  // PWM to Actuator position angle gain, float
#define PITCH_PWM_OFFSET   7000  // PWM to Actuator position angle offset, flat to platform (est.), int
#define PITCH_FB_GAIN     0.035  // ADC output to attitude angle feedback gain, float
#define PITCH_FB_OFFSET   302.0  // ADC output to attitude angle feedback offset, flat to platform, float

#define ROLL_BI_TRANSLATION 0.0  // Translates Roll from INS ref frame to platform base ref frame
#define ROLL_PWM_MIN       4590  // PWM code min limit, int
#define ROLL_PWM_MAX       8750  // PWM code max limit, int
#define ROLL_PWM_GAIN     177.1  // PWM to Actuator position angle gain (est.), float
#define ROLL_PWM_OFFSET    6790  // PWM to Actuator position angle offset, flat to platform (est.), int
#define ROLL_FB_GAIN       0.18  // ADC output to attitude angle feedback gain, float
#define ROLL_FB_OFFSET    116.0  // ADC output to attitude angle feedback offset, flat to platform, float

#else
#endif

/* Input Limits */
#define PITCH_IN_MIN   PITCH_IN_BASE_MIN - PITCH_BI_TRANSLATION
#define PITCH_IN_MAX   PITCH_IN_BASE_MAX - PITCH_BI_TRANSLATION
#define ROLL_IN_MIN    ROLL_IN_BASE_MIN  - ROLL_BI_TRANSLATION
#define ROLL_IN_MAX    ROLL_IN_BASE_MAX  - ROLL_BI_TRANSLATION

/* Control Loop Dynamics and Limits */
#define PITCH_IN_BASE_MIN    -5  // Operational Range relative to base, min, int
#define PITCH_IN_BASE_MAX     5  // Operational Range relative to base, max, int
#define PITCH_LOOPERRK_MIN -100  // Loop Error saturation, min, int
#define PITCH_LOOPERRK_MAX  100  // Loop Error saturation, max, int

#define ROLL_IN_BASE_MIN    -10  // Operational Range relative to base, min, int
#define ROLL_IN_BASE_MAX     10  // Operational Range relative to base, max, int
#define ROLL_LOOPERRK_MIN  -200  // Loop Error saturation, min, int
#define ROLL_LOOPERRK_MAX   200  // Loop Error saturation, max, int

#define LOOP_K             32.0  // Loop Gain, float
#define LOOP_TC           24000  // Loop Time Dominant Pole Time Constant, 2*tau/T [s/s]
#define LOOP_TCP          24001  // 1 + LOOP_TC
#define LOOP_TCN         -23999  // 1 - LOOP_TC
#define PWM_QUANT             3  // PWM Code Quantization to 2^PWM_QUANT, int
#define FB_AVE               50  // Feedback Averaging, # Samples
#define GAIN_ADJ         1.0313  // Gain factor to compensate for low control loop gain, = (32+1)/32

#define MAX_IM            0x300  // 10-bit ADC output corresponding to Maximum ave. motor current 
                                 //   ~(I_motor * 0.3ohms * 50 - 0.7V) * 0x3FF / 3.3V)
                                 //   This value must be 0x300 or below (corresponding to 2.48V) due to sense diode drop
                                 //   Empirical observation shows this value less than 0x200, (1.64V, I_motor~150mA)

/* Operational Modes */
#define INPUT_MODE_ARINC     0  // Input Data Modes
#define INPUT_MODE_MANUAL    1
#define INPUT_MODE_CAL       2
#define INPUT_MODE_TP        3

#define LOOP_MODE_CLOSED     0  // Control Loop Modes
#define LOOP_MODE_OPEN       1

#define REFERENCE_MODE_INS   0  // Reference frame modes
#define REFERENCE_MODE_BASE  1

#define OUTPUT_MODE_STREAM   0  // Send Data on output interface
#define OUTPUT_MODE_SILENT   1  // Do not sent data
#define OUTPUT_MODE_EXAMPLE  2  // Send Example Data

/* Calibration */
#define CAL_SEQ_LEN 24
#define CAL_SEQ_INTERVAL 10  // seconds between sequence positions

/* Serial Input Commands */
#define COMMAND_BUF_MAX  20  // Command Buffer max length
#define COMMAND_STATUS_NOT_COMPLETE 0
#define COMMAND_STATUS_COMPLETE     1

/* SPI and I2C Interface */
#define SPI_OK      0 // transfer ended No Errors
#define SPI_BUSY    1 // transfer busy
#define SPI_ERROR   2 // SPI error
#define PITCH_LABEL 0xD4
#define ROLL_LABEL  0xD5
#define SSM         0x60000000


void execute_control_loop_iteration(void);
void read_arinc_update_loop_input(void);
void test_pattern_update_loop_input(void);
void calibration_update_loop_input(void);
void send_output_data(void);
void send_config(void);
void check_motor_overcurrent(void);
float get_reference_frame_pitch_translation(short ref_mode);
float get_reference_frame_roll_translation(short ref_mode);
void toggle_test_output(void);
void init_system(void);
void Uart0_Init(void);
void ADC_Init(void);
void T1_Init(void);
void PWM_Init(void);
void SPI_Init(void);
void PrintByte(unsigned char b);
void PrintString(const char *s);
int atoi(char * str);
static void process_inchar(void);
static void process_command(void);
static void ua_outchar(char c);
unsigned short read_adc(unsigned char);
__irq void T1_Isr(void);
__irq void UART_RX_Isr(void);


static char Buf[100];                  // A2D data buffer
static char cmd_buf[COMMAND_BUF_MAX] = {0,0,0,0,0,0,0,0,0,0}; // UART Receive command buffer
static int  cmd_buf_inext = 0;         // Next available cmd_buf index
static short command_status = COMMAND_STATUS_NOT_COMPLETE;

long  data;
unsigned char  f_10ms, f_500us, f_50ms;
unsigned int   f_1000ms;
const    char  ascii[] = "0123456789ABCDEF";
short pitch_ovcr;
short roll_ovcr;

float attitude;

static short input_data_mode = INPUT_MODE_ARINC;
static short output_data_mode = OUTPUT_MODE_STREAM;
static short loop_mode = LOOP_MODE_CLOSED;
static short reference_mode = REFERENCE_MODE_INS;

// Note: test pattern pitch and roll angles do not include mounting corrections
//   test pattern is meant to exercise the platform relative to its base
const float tp_pitch_angles[5] = {0.0,1.0,2.0,-2.0,-1.0};
const float tp_roll_angles[5]  = {0.0,2.0,4.0,-4.0,-2.0};
unsigned short tp_pitch_index = 0;
unsigned short tp_roll_index  = 0;
unsigned int tp_counter = 0;

// Calibration sequence positions
//   Test 5 position, approach from both sides, cross (t) or Criss-cross (x) pattern
//   Even indexes are approach position, odd are measurement positions
//   pitch angles are offset to correspond to true leveling relative to ground
const float cal_roll_angles_t[CAL_SEQ_LEN]  = {0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,
                                             -7.0,-6.0,-5.0,-6.0, -1.0,0.0,1.0,0.0, 5.0,6.0,7.0,6.0};
const float cal_pitch_angles_t[CAL_SEQ_LEN] = {-4.0,-3.0,-2.0,-3.0, -1.0,0.0, 1.0,0.0, 2.0,3.0,4.0,3.0, 
                                              0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0};

const float cal_roll_angles_x[CAL_SEQ_LEN]  = {-7.0,-6.0,-5.0,-6.0, -1.0,0.0,1.0,0.0, 5.0,6.0,7.0,6.0,
                                               -7.0,-6.0,-5.0,-6.0, -1.0,0.0,1.0,0.0, 5.0,6.0,7.0,6.0};
const float cal_pitch_angles_x[CAL_SEQ_LEN] = {-4.0,-3.0,-2.0,-3.0, -1.0,0.0, 1.0,0.0, 2.0,3.0,4.0,3.0, 
                                               -2.0,-3.0,-4.0,-3.0,  1.0,0.0,-1.0,0.0, 4.0,3.0,2.0,3.0};

int   cal_sequence_counter = 0; // Counter to adjust time between positions
short cal_sequence_index = 0;   // Advances the position of the cal sequence

float pitch_attitude;
float pitch_base;
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
float roll_base;
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

int main (void)
{
    init_system();
    send_config();
  
    while (1) // Loop forever
    {
        process_inchar();
        if (command_status == COMMAND_STATUS_COMPLETE)
        {
            process_command();
            command_status = COMMAND_STATUS_NOT_COMPLETE;
        }

        toggle_test_output();

        pitch_encoder = read_adc(8);
        pitch_ovcr = read_adc(4);

        roll_encoder  = read_adc(2);
        roll_ovcr = read_adc(1);  
        
        check_motor_overcurrent();

        if (f_500us == 1) //T1 isr every 500_u 
        {		 
            // Update input to control Loop base on mode
            if (INPUT_MODE_ARINC == input_data_mode)
            {
                read_arinc_update_loop_input();
            }
            else if (INPUT_MODE_TP == input_data_mode)
            {
                test_pattern_update_loop_input();
            }
            else if (INPUT_MODE_MANUAL == input_data_mode)
            {
                // Do nothing. In manual mode, loop input is updated through serial commands
            }
            else if (INPUT_MODE_CAL == input_data_mode)
            {
                calibration_update_loop_input();
            }
            else 
            {}

            // Update Control Loop
            execute_control_loop_iteration();

            // Increment Time Counters
            f_500us = 0;
            f_10ms += 1;
            f_50ms += 1;																				 
            f_1000ms += 1;
        }

        if (f_10ms == 20)         // every 10 mseconds (100Hz)
        {
          f_10ms = 0;
        }

        if (f_50ms == 100)        // 20 Hz TE cooler control
        {
            f_50ms = 0;      
        }

        if (f_1000ms == 2000)      // 1000 mseconds
        {
            send_output_data();
            f_1000ms = 0;
        }
    } // end while loop

    return 0; // Never reached
}    

void read_arinc_update_loop_input(void)
{
    while ((IO0PIN & 0x02000000) == 0) //FIFO not empty, rflag P0.25
    { 
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

        attitude = (float)((data<<3>>13)) * 0.0006866455078125;

        switch (data & 0xFF) 
        {
            case PITCH_LABEL:
                if((data & SSM) != SSM) break;

                // Verify that attitude input does not exceed limits
                pitch_attitude = attitude;
                if (pitch_attitude > PITCH_IN_MAX) pitch_attitude = PITCH_IN_MAX;
                if (pitch_attitude < PITCH_IN_MIN) pitch_attitude = PITCH_IN_MIN;
            break;
                
            case ROLL_LABEL:
                if((data & SSM) != SSM) break;

                // Verify that attitude input does not exceed limits
                roll_attitude = attitude;
                if (roll_attitude > ROLL_IN_MAX) roll_attitude = ROLL_IN_MAX;
                if (roll_attitude < ROLL_IN_MIN) roll_attitude = ROLL_IN_MIN;
            break;

            default:
            break;
        }
    } 
    return;
}

void test_pattern_update_loop_input(void)
{
    tp_counter = (tp_counter + 1) % (2000 * CAL_SEQ_INTERVAL); // 1 sec = 2000 counts (0.5ms step)
    
    if (tp_counter == 1)
    {
        tp_pitch_index = (tp_pitch_index + 1) % 5;
        if (tp_pitch_index == 0)
        {
            tp_roll_index = (tp_roll_index + 1) % 5;
        }
    }

    // Note: test pattern mode is always referenced to base.
    //   pitch and roll angles do not include mounting corrections
    //   test pattern is meant to exercise the platform relative to its base
    pitch_attitude = tp_pitch_angles[tp_pitch_index];
    roll_attitude  = tp_roll_angles[tp_roll_index];

    return;
}

void calibration_update_loop_input(void)
{
    cal_sequence_counter++;
    
    // Even sequence indexes are approach angles, need only wait 1sec
    // Odd sequence indexes are measurement angles, wait 5 sec
    if ((cal_sequence_counter >= 2000) && (0 == (cal_sequence_index % 2)))
    {
        cal_sequence_index++;
        cal_sequence_counter = 0;
    }
    else if (cal_sequence_counter >= 10000)
    {
        send_output_data();
        cal_sequence_index++;        
        cal_sequence_counter = 0;
    }

    if (cal_sequence_index < CAL_SEQ_LEN)
    {
        pitch_attitude = cal_pitch_angles_t[cal_sequence_index];
        roll_attitude  = cal_roll_angles_t[cal_sequence_index];
    }
    else
    {
        pitch_attitude = 0.0;
        roll_attitude = 0.0;
        cal_sequence_index = 0;
        cal_sequence_counter = 0;
        input_data_mode = INPUT_MODE_MANUAL;
        PrintString("!CAL DONE, now IMM\r\n");
    }
}

void execute_control_loop_iteration(void)
{
    pitch_base = PITCH_POLARITY * (pitch_attitude + get_reference_frame_pitch_translation(reference_mode));
    roll_base = roll_attitude + get_reference_frame_roll_translation(reference_mode);

    if (LOOP_MODE_CLOSED == loop_mode)
    {
        // Apply gain adjustment and Saturate loop input
        pitch_loopInput =  GAIN_ADJ * pitch_base;
        if (pitch_loopInput > PITCH_IN_BASE_MAX) pitch_loopInput = PITCH_IN_BASE_MAX;
        if (pitch_loopInput < PITCH_IN_BASE_MIN) pitch_loopInput = PITCH_IN_BASE_MIN;                                

        roll_loopInput = GAIN_ADJ * roll_base;
        if (roll_loopInput > ROLL_IN_BASE_MAX) roll_loopInput = ROLL_IN_BASE_MAX;
        if (roll_loopInput < ROLL_IN_BASE_MIN) roll_loopInput = ROLL_IN_BASE_MIN;

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
        if (pitch_decCntr >= FB_AVE)
        {
            pitch_loopFB = pitch_posFilter / (float) FB_AVE; // Update the feedback value periodically per FB_AVE
            pitch_posFilter = 0.0;
            pitch_decCntr = 0;
        }

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
        if (roll_decCntr >= FB_AVE)
        {
            roll_loopFB = roll_posFilter / (float) FB_AVE; // Update the feedback value periodically per FB_AVE
            roll_posFilter = 0.0;
            roll_decCntr = 0;
        }   
    }
    else if (LOOP_MODE_OPEN == loop_mode)
    {
        pitch_pwm = (short)(pitch_base * PITCH_PWM_GAIN + PITCH_PWM_OFFSET);
        if (pitch_pwm > PITCH_PWM_MAX) pitch_pwm = PITCH_PWM_MAX;
        if (pitch_pwm < PITCH_PWM_MIN) pitch_pwm = PITCH_PWM_MIN;

        roll_pwm =  (short)(roll_base * ROLL_PWM_GAIN + ROLL_PWM_OFFSET);
        if (roll_pwm > ROLL_PWM_MAX) roll_pwm = ROLL_PWM_MAX;
        if (roll_pwm < ROLL_PWM_MIN) roll_pwm = ROLL_PWM_MIN;
    }
    else
    {}

    PWMMR4 = pitch_pwm;
    PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
    IO0SET = 0x00080000;		 //toggle debug
    IO0CLR = 0x00080000;

    PWMMR6 = roll_pwm;
    PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
    IO0SET = 0x00080000;		 //toggle debug
    IO0CLR = 0x00080000; 

    return;
}

void send_output_data(void)
{
    if (OUTPUT_MODE_STREAM == output_data_mode)
    {
        sprintf(Buf,"#%.2f,%d,%d,%.2f,%d,%.2f,%d,%d,%.2f,%d\r\n",  
                    pitch_attitude, pitch_pwm, pitch_encoder, pitch_loopFB, pitch_ovcr,
                    roll_attitude,  roll_pwm,  roll_encoder,  roll_loopFB,  roll_ovcr);
        PrintString(Buf);

        // sprintf(Buf,"Pin:%.2f,Ppwm:%d,Penc:%d,Pfb:%.2f,Rin:%.2f,Rpwm:%d,Renc:%d,Rfb:%.2f\r\n",  
        //             pitch_attitude, pitch_pwm, pitch_encoder, pitch_loopFB, roll_attitude, roll_pwm, roll_encoder, roll_loopFB); // 89-char
        // PrintString(Buf);

        // sprintf(Buf,"0x%08x P:%.3f,%i R:%.3f,%i\r\n", data, pitch_attitude, pitch_pwm, roll_attitude, roll_pwm);
        // PrintString(Buf);
        // sprintf(Buf,"Err:%.3f,Erf:%.3f,Enc:%d,FB:%.2f\r\n", pitch_loopErrK, pitch_loopErrK_filt, pitch_encoder, pitch_loopFB);
        // PrintString(Buf);
        // sprintf(Buf,"Err:%.3f,Erf:%.3f,Enc:%d,FB:%.2f\r\n", roll_loopErrK, roll_loopErrK_filt, roll_encoder, roll_loopFB);
        // PrintString(Buf);
        // sprintf(Buf,"P(M2):0x%08x, R(M1):0x%08x\r\n", pitch_encoder, roll_encoder);
        // PrintString(Buf);
        // sprintf(Buf,"ROLL  %08x %f %i\r\n", data, roll_attitude, roll);
        // PrintString(Buf);
        // sprintf(Buf,"%08x\r\n",data);
        // PrintString(Buf);
    }
    else if (OUTPUT_MODE_EXAMPLE == output_data_mode)
    {
        sprintf(Buf,"#9.99,9999,999,9.99,999,-9.99,9999,999,-9.99,999\r\n");
        PrintString(Buf);
    }
    else if (OUTPUT_MODE_SILENT == output_data_mode)
    {}
    else
    {}
    return;    
}

float get_reference_frame_pitch_translation(short ref_mode)
{
    float translation = 0.0;
    if (REFERENCE_MODE_INS == ref_mode)
    {
        translation = PITCH_BI_TRANSLATION;
    }
    else if (REFERENCE_MODE_BASE == ref_mode)
    {
        translation = 0.0;
    }
    else
    {
        translation = 0.0;
    }
    return translation;
}

float get_reference_frame_roll_translation(short ref_mode)
{
    float translation = 0.0;
    if (REFERENCE_MODE_INS == ref_mode)
    {
        translation = ROLL_BI_TRANSLATION;
    }
    else if (REFERENCE_MODE_BASE == ref_mode)
    {
        translation = 0.0;
    }
    else
    {
        translation = 0.0;
    }
    return translation;
}

void check_motor_overcurrent(void)
{
    if ((roll_ovcr > MAX_IM) || (pitch_ovcr > MAX_IM)){   // Check motor overcurrent
      VICIntEnClr = 0xFFFFFFFF;    //disable all interrupts
      PWMMR4 = 0;
      PWMMR6 = 0;
      PWMLER = 0x7F; // enable PWM4-PWM6 match latch (reload
      PrintString("\r\nMotor Current Overload\r\n");
      while (1); // wait for a RESET
    }
    return;
}

void toggle_test_output(void)
{
    IO0SET = 0x00080000;		 //toggle debug
    IO0CLR = 0x00080000;
    return;
}

void init_system(void)
{
    VICIntEnClr = 0xFFFFFFFF; // disable all interrupts!
    VPBDIV = 1;
    PINSEL0 = 0x000A1505;
    PINSEL1 = 0x15400000; 
    IO1DIR =  0x0C000000;  
    IO0DIR =  0x00080751;  //Set outputs
    IO0CLR =  0x00080751;  //Clear all outputs.
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
    T1_Init();  // 0.5 msec tick
    SPI_Init(); // initialize SPI 
    return;
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
    return;
}

void ADC_Init(void)
{
    ADCR = 0x00200F00; // initialize ADC: ADC operational, no burst, select AIN 0-3 F=clkdiv
    return;
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
    PWMMR6 = roll_pwm;  // Motor 2 pulse
    PWMMCR = 0x00000002; // 
    PWMPCR = 0x5000; // enable PWM4 PWM6 outputs
    PWMLER = 0x7F; // enable PWM0 - PWM6 match latch (reload)
    PWMTCR = 0x09; // enable PWM mode and start timer
    return;
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

    for (i = 0; i < 5; i++)
    {
        S0SPDR = 0x00;         //ARINC labels not selected
        while ((S0SPSR & 0x80) == 0); //busy
    }
    
    S0SPDR = 0x30;           // ARINC pitch and roll lables selected WAS 0X30
    while ((S0SPSR & 0x80) == 0); //busy

    for (i=0 ; i<26; i++)
    {
        S0SPDR = 0x00;         //ARINC labels not selected
        while ((S0SPSR & 0x80) == 0); //busy
    }

    IOSET0 = 0x00000400;
    return;
}

void Uart0_Init(void)
{
    U0LCR = 0x83; // 8 bits, no Parity, 1 Stop bit
    U0DLL = 0x21; // 115.2Kbaud for 60MHz PCLK Clock
    U0DLM = 0x00;
    U0FCR = 0x07; // FIFO Enabled
    U0LCR = 0x03; // DLAB = 0
    return;
}

static void ua_outchar(char c)
{
    U0THR = c;
    while(!(U0LSR & 0x40));
    return;
}

/* process_inchar()
   Check for UART input characters and add them to the buffer
   Characters only accumulated into command if start with '#'
   Command finished (command_status set) when '\r' or '\n' received.
 */
static void process_inchar(void)
{
    char input_char = 0x00;

    if ((U0LSR & 0x01) && (command_status == COMMAND_STATUS_NOT_COMPLETE)) //If character is in RX buffer
    {
        input_char = U0RBR;
        switch (input_char) {
            case ('\n'):
                cmd_buf[cmd_buf_inext] = 0x00; // NULL String termination

                if (cmd_buf_inext != 0)
                {
                    command_status = COMMAND_STATUS_COMPLETE;
                }
                cmd_buf_inext = 0;
            break;

            case ('\r'):
                cmd_buf[cmd_buf_inext] = 0x00;

                if (cmd_buf_inext != 0) 
                {
                    command_status = COMMAND_STATUS_COMPLETE;
                }
                cmd_buf_inext = 0;
            break;

            case ('#'):
                PrintByte('#');
                cmd_buf[0] = '#';
                cmd_buf_inext = 1;
            break;

            default:
                cmd_buf[cmd_buf_inext] = input_char;
                cmd_buf_inext++;
                if (cmd_buf_inext >= COMMAND_BUF_MAX)
                {                
                    cmd_buf_inext = 0;
                }
            break;
        }
    }
    return;
}
/* end process_inchar() */

/* process_command()
   When a command is complete, process the command string stored in cmd_buf
   The command is a string (NULL terminated), so string functions used
 */
static char cmd_delim[] = ",";

static void process_command(void)
{
    char * cmd_command = strtok(cmd_buf, cmd_delim);
    char * cmd_arg     = strtok(NULL,    cmd_delim);
    
    if (0 == strcmp("#NOR", cmd_command)) // Normal Input/Output Mode
    {
        PrintString("!NOR Received\r\n");
        input_data_mode = INPUT_MODE_ARINC;
        output_data_mode = OUTPUT_MODE_STREAM;
    }
    else if (0 == strcmp("#CAL", cmd_command)) // Calibration Sequence
    {    
        PrintString("!CAL Received\r\n");
        cal_sequence_index = 0;
        cal_sequence_counter = 0;
        input_data_mode = INPUT_MODE_CAL;
        output_data_mode = OUTPUT_MODE_SILENT;
    }
    else if (0 == strcmp("#OMN", cmd_command)) // Output Mode: Normal, Streaming
    {
        PrintString("!OMN Received\r\n");
        output_data_mode = OUTPUT_MODE_STREAM;
    }
    else if (0 == strcmp("#OMS", cmd_command)) // Output Mode: Silent
    {
        PrintString("!OMS Received\r\n");
        output_data_mode = OUTPUT_MODE_SILENT;
    }
    else if (0 == strcmp("#OMX", cmd_command)) // Output Mode: Example, Streaming
    {
        PrintString("!OMX Received\r\n");
        output_data_mode = OUTPUT_MODE_EXAMPLE;
    }
    else if (0 == strcmp("#OMQ", cmd_command)) // Output Mode: Query, Single
    {
        PrintString("!OMQ Received\r\n");
        output_data_mode = OUTPUT_MODE_STREAM;
        send_output_data();
        output_data_mode = OUTPUT_MODE_SILENT;
    }
    else if (0 == strcmp("#LMO", cmd_command)) // Loop Mode: Open-loop
    {    
        PrintString("!LMO Received\r\n");
        loop_mode = LOOP_MODE_OPEN;
    }
    else if (0 == strcmp("#LMC", cmd_command)) // Loop Mode: closed-loop
    {
        PrintString("!LMC Received\r\n");
        loop_mode = LOOP_MODE_CLOSED;
    }
    else if (0 == strcmp("#CFG", cmd_command)) // Output Configuration
    {
        PrintString("!CFG Received\r\n");
        send_config();
    }
    else if (0 == strcmp("#WPM", cmd_command)) // Write PROM Enable
    {
        PrintString("!WPM Received\r\n");
        //TODO: Implement Write PROM Enable
    }
    else if (0 == strcmp("#IMA", cmd_command)) // Input Mode: ARINC
    {    
        PrintString("!IMA Received\r\n");
        input_data_mode = INPUT_MODE_ARINC;
        output_data_mode = OUTPUT_MODE_STREAM;
    }
    else if (0 == strcmp("#IMT", cmd_command)) // Input Mode: Test Pattern
    {    
        PrintString("!IMT Received\r\n");
        reference_mode = REFERENCE_MODE_BASE;
        input_data_mode = INPUT_MODE_TP;
    }
    else if (0 == strcmp("#IMM", cmd_command)) // Input Mode: Manual
    {    
        PrintString("!IMM Received\r\n");
        input_data_mode = INPUT_MODE_MANUAL;        
    }
    else if (0 == strcmp("#MZ", cmd_command)) // Manual Zero Position
    {    
        PrintString("!MZ Received\r\n");
        if (INPUT_MODE_MANUAL == input_data_mode)
        {
            pitch_attitude = 0;
            roll_attitude = 0;
        }
        else
        {
            PrintString("!ERROR: Must Enabled Manual Mode\r\n");
        }
    }
    else if ((0 == strcmp("#MP", cmd_command)) && (NULL != cmd_arg)) // Manual Pitch Change
    {
        int temp_arg = 0;
        PrintString("!MP Received\r\n");
        if (INPUT_MODE_MANUAL == input_data_mode)
        {
            temp_arg = atoi(cmd_arg);
            if ((temp_arg > PITCH_IN_MAX)) temp_arg = 0;
            if ((temp_arg < PITCH_IN_MIN)) temp_arg = 0;

            pitch_attitude = temp_arg;       
        }
        else
        {
            PrintString("!ERROR: Must Enabled Manual Mode\r\n");
        }
    }
    else if ((0 == strcmp("#MR", cmd_command)) && (NULL != cmd_arg)) // Manual Roll Change
    {
        int temp_arg = 0;
        PrintString("!MR Received\r\n");
        if (INPUT_MODE_MANUAL == input_data_mode)
        {
            temp_arg = atoi(cmd_arg);
            if ((temp_arg > ROLL_IN_MAX)) temp_arg = 0;
            if ((temp_arg < ROLL_IN_MIN)) temp_arg = 0;

            roll_attitude = temp_arg;
        }
        else
        {
            PrintString("!ERROR: Must Enabled Manual Mode\r\n");
        }
    }
    else if (0 == strcmp("#RMI", cmd_command))  // Reference Mode: INS
    {    
        PrintString("!RMI Received\r\n");
        reference_mode = REFERENCE_MODE_INS;
    }
    else if (0 == strcmp("#RMB", cmd_command)) // Reference Mode: Base
    {    
        PrintString("!RMB Received\r\n");
        reference_mode = REFERENCE_MODE_BASE;
    }
    else
    {}
    return;
}
/* end process_command() */

void send_config(void)
{
    sprintf(Buf,"!Firmware Revision: %d\r\n",FW_REV);  
    PrintString(Buf);
    sprintf(Buf,"!Platform: %d\r\n",PLATFORM);  
    PrintString(Buf);

    sprintf(Buf,"!Data Mode: %d\r\n",input_data_mode);  
    PrintString(Buf);
    sprintf(Buf,"!Output Mode: %d\r\n",output_data_mode);  
    PrintString(Buf);
    sprintf(Buf,"!Loop Mode: %d\r\n",loop_mode);  
    PrintString(Buf);

    sprintf(Buf,"!Loop K Gain: %f\r\n",LOOP_K);
    PrintString(Buf);
    sprintf(Buf,"!Loop TC: %d\r\n",LOOP_TC);
    PrintString(Buf);
    sprintf(Buf,"!PWM Quant: %d\r\n",PWM_QUANT);
    PrintString(Buf);
    sprintf(Buf,"!FB Averages: %d\r\n",FB_AVE);
    PrintString(Buf);
    sprintf(Buf,"!Gain Adjust: %f\r\n",GAIN_ADJ);
    PrintString(Buf);

    sprintf(Buf,"!Pitch Translation: %f\r\n",PITCH_BI_TRANSLATION);
    PrintString(Buf);
    sprintf(Buf,"!Pitch PWM Min: %d\r\n",PITCH_PWM_MIN);
    PrintString(Buf);
    sprintf(Buf,"!Pitch PWM Max: %d\r\n",PITCH_PWM_MAX);
    PrintString(Buf);
    sprintf(Buf,"!Pitch PWM Gain: %f\r\n",PITCH_PWM_GAIN);
    PrintString(Buf);
    sprintf(Buf,"!Pitch PWM Offset: %d\r\n",PITCH_PWM_OFFSET);
    PrintString(Buf);
    sprintf(Buf,"!Pitch FB Gain: %f\r\n",PITCH_FB_GAIN);
    PrintString(Buf);
    sprintf(Buf,"!Pitch FB Offset: %f\r\n",PITCH_FB_OFFSET);
    PrintString(Buf);

    sprintf(Buf,"!Roll Translation: %f\r\n",ROLL_BI_TRANSLATION);
    PrintString(Buf);
    sprintf(Buf,"!Roll PWM Min: %d\r\n",ROLL_PWM_MIN);
    PrintString(Buf);
    sprintf(Buf,"!Roll PWM Max: %d\r\n",ROLL_PWM_MAX);
    PrintString(Buf);
    sprintf(Buf,"!Roll PWM Gain: %f\r\n",ROLL_PWM_GAIN);
    PrintString(Buf);
    sprintf(Buf,"!Roll PWM Offset: %d\r\n",ROLL_PWM_OFFSET);
    PrintString(Buf);
    sprintf(Buf,"!Roll FB Gain: %f\r\n",ROLL_FB_GAIN);
    PrintString(Buf);
    sprintf(Buf,"!Roll FB Offset: %f\r\n",ROLL_FB_OFFSET);
    PrintString(Buf);
    return;
}

void PrintByte(unsigned char b)
{
    ua_outchar(b & 0x0f);
    return;
}

void PrintString(const char *s)
{
    while(*s)
    {
        //    if(*s == '\n') ua_outchar('\r');
        ua_outchar(*s);
        s++;
    }
    return;
}


/* atoi()
   Custom Implementation for basic signed integers
 */
int atoi(char * str)
{
    int len = strlen(str);
    int value = 0;
    int k;

    if (0 == len)
    {
        return 0;
    }

    for (k=0; k < len; k++) //Check for floating point
    {
        if ((str[0] == '.') || (str[0] == ',')) return 0;
    }
    
    if (str[0] != '-')
    {
        value += (str[0] - '0');
    }

    for (k=1; k < len; k++)
    {
        value = 10 * value;
        value += (str[k] - '0');
    }

    if (str[0] == '-')
    {
        value = -1 * value;
    }

    return value;
}

__irq void T1_Isr(void)  // Timer 1 ISR every 0.5 msec
{
    f_500us = 1; // toggles every 0.5 mseconds

    T1IR = 0x01; // reset interrupt flag
    VICVectAddr = 0; // reset VIC
    return;
}
