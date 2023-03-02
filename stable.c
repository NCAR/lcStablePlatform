#include "LPC21xx.h"
#include <STDLIB.H>
#include <STDIO.H>
#include <STRING.H>
#include <MATH.H>

#define FW_REV 20230301
//#define DEBUG  // Uncomment to add in debug functionality

/* Platform Selection */
#define TOP 0
#define BOTTOM 1

// Platform type is no longer controlled by #define, rather, is now included in saved memory structure
//   PLATFORM value here merely sets the starter coefficients for an unprogrammed memory.
//   TOP is used because it has more restrictive PWM limits
#define PLATFORM  TOP   // MUST SET: TOP or BOTTOM to select appropriate coeffs

/* Platform Coefficients */
#define PITCH_BI_TRANSLATION 0.0 // Translates Pitch from INS ref frame to platform base ref frame
                                 //   Negative to correct positive aircraft pitch
                                 //   Platform mounting estimated -2.9deg relative to airframe
#define PITCH_PWM_MIN      5000  // PWM code min limit, int
#define PITCH_PWM_MAX      9000  // PWM code max limit, int
#define PITCH_PWM_GAIN    300.0  // PWM to Actuator position angle gain, float
#define PITCH_PWM_OFFSET   7000  // PWM to Actuator position angle offset, flat to platform (est.), int
#define PITCH_FB_GAIN     0.030  // ADC output to attitude angle feedback gain, float
#define PITCH_FB_OFFSET   300.0  // ADC output to attitude angle feedback offset, flat to platform, float

#define ROLL_BI_TRANSLATION 0.0  // Translates Roll from INS ref frame to platform base ref frame
#define ROLL_PWM_MIN       5000  // PWM code min limit, int
#define ROLL_PWM_MAX       9000  // PWM code max limit, int
#define ROLL_PWM_GAIN     100.0  // PWM to Actuator position angle gain (est.), float
#define ROLL_PWM_OFFSET    7000  // PWM to Actuator position angle offset, flat to platform (est.), int
#define ROLL_FB_GAIN       0.10  // ADC output to attitude angle feedback gain, float
#define ROLL_FB_OFFSET    200.0  // ADC output to attitude angle feedback offset, flat to platform, float
/*
#if (PLATFORM == BOTTOM) // Updated 1/22/2019

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
*/

/* Control Loop Dynamics and Limits */
#define PITCH_IN_BASE_MIN    -5.0  // Operational Range relative to base, min, float
#define PITCH_IN_BASE_MAX     5.0  // Operational Range relative to base, max, float
#define PITCH_LOOPERRK_MIN -100.0  // Loop Error saturation, min, float
#define PITCH_LOOPERRK_MAX  100.0  // Loop Error saturation, max, float

#define ROLL_IN_BASE_MIN    -10.0  // Operational Range relative to base, min, float
#define ROLL_IN_BASE_MAX     10.0  // Operational Range relative to base, max, float
#define ROLL_LOOPERRK_MIN  -200.0  // Loop Error saturation, min, float
#define ROLL_LOOPERRK_MAX   200.0  // Loop Error saturation, max, float

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

/* Input Attitude Limits */
// Attitude input limits moved to variable. Depends on programmabe pitch_bi_translation
//#define PITCH_IN_MIN   (PITCH_IN_BASE_MIN - PITCH_BI_TRANSLATION)
//#define PITCH_IN_MAX   (PITCH_IN_BASE_MAX - PITCH_BI_TRANSLATION)
//#define ROLL_IN_MIN    (ROLL_IN_BASE_MIN  - ROLL_BI_TRANSLATION)
//#define ROLL_IN_MAX    (ROLL_IN_BASE_MAX  - ROLL_BI_TRANSLATION)

/* Operational Modes Options */
#define INPUT_MODE_ARINC     0  // Input Data Modes
#define INPUT_MODE_MANUAL    1
#define INPUT_MODE_CAL       2
#define INPUT_MODE_TP        3

#define OUTPUT_MODE_STREAM   0  // Send Data on output interface
#define OUTPUT_MODE_SILENT   1  // Do not sent data
#define OUTPUT_MODE_EXAMPLE  2  // Send Example Data

#define LOOP_MODE_OPEN     0  // Control Loop Modes
#define LOOP_MODE_CLOSED   1

#define REFERENCE_MODE_BASE 1  // Reference frame modes
#define REFERENCE_MODE_INS  0

#define WRITE_PROM_MODE_ENABLED   0
#define WRITE_PROM_MODE_DISABLED  1

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

/* In-Application Programming Flash Access */
// Note: use of IAP operations requires non-access of base 32Bytes of RAM
//  Project options modified to start RAM access at 0x40001000 instead of 0x40000000
#define IAP_LOCATION 0x7ffffff1
#define CONFIG_SAVE_SECTOR 0x0E 
#define CONFIG_SAVE_ADDR   0x00038200 // 256kB Part, Sector 14: 0x00038000 - 0x00039FFF


// coeff_set_t, structure of control loop coefficients
//typedef struct coeff_set_struct
typedef struct platform_config_struct
{
    float pitch_bi_translation;
    unsigned int pitch_pwm_min;
    unsigned int pitch_pwm_max;
    float pitch_pwm_gain;
    unsigned int pitch_pwm_offset;
    float pitch_fb_gain;
    float pitch_fb_offset;
       
    float roll_bi_translation;
    unsigned int roll_pwm_min;
    unsigned int roll_pwm_max;
    float roll_pwm_gain;
    unsigned int roll_pwm_offset;
    float roll_fb_gain;
    float roll_fb_offset; 
    
    float pitch_in_min;
    float pitch_in_max;
    float roll_in_min;
    float roll_in_max;

    unsigned short platform_type;   // 0=TOP, 1=BOTTOM
} plat_config_t;

/* Function Prototypes */
void execute_control_loop_iteration(void);
void read_arinc_update_loop_input(void);
void test_pattern_update_loop_input(void);
void calibration_update_loop_input(void);
void send_output_data(void);
void send_const(void);
void send_coeffs(plat_config_t * p_config);
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
static void ua_outchar(char c);
void PrintString(const char *s);
static void process_inchar(void);
static void process_command(void);
unsigned short read_adc(unsigned char);
int save_configurations(plat_config_t * p_config);
int load_configurations(plat_config_t * p_config);
void read_coefficients(void);
void update_configuration(plat_config_t * p_config, unsigned int config_index, float value);
__irq void T1_Isr(void);

/* Global Variables */
static char Buf[100];                  // A2D data buffer
static char cmd_buf[COMMAND_BUF_MAX] = {0,0,0,0,0,0,0,0,0,0}; // UART Receive command buffer
static int  cmd_buf_inext = 0;         // Next available cmd_buf index
static short command_status = COMMAND_STATUS_NOT_COMPLETE;
static char cmd_delim[] = ",";


long  data;
unsigned short data1, data2, data3, data4;
unsigned int i;
unsigned char  f_500us;
unsigned char  f_100ms;
unsigned int   f_1000ms;
const    char  ascii[] = "0123456789ABCDEF";
short pitch_ovcr;
short roll_ovcr;
float attitude;

// Operational Modes (Global)
static short input_data_mode  = INPUT_MODE_ARINC;
static short output_data_mode = OUTPUT_MODE_STREAM;
static short reference_mode   = REFERENCE_MODE_INS;
static short write_prom_mode  = WRITE_PROM_MODE_DISABLED;
static short loop_mode        = LOOP_MODE_OPEN;

// Test Pattern position sequence  (Global)
// Note: test pattern pitch and roll angles do not include mounting corrections
//   test pattern is meant to exercise the platform relative to its base
const float tp_pitch_angles[5] = {0.0,1.0,2.0,-2.0,-1.0};
const float tp_roll_angles[5]  = {0.0,2.0,4.0,-4.0,-2.0};
unsigned short tp_pitch_index = 0;
unsigned short tp_roll_index  = 0;
unsigned int tp_counter = 0;

// Calibration sequence positions  (Global)
//   Test 5 position, approach from both sides, cross (t) or Criss-cross (x) pattern
//   Even indexes are approach position, odd are measurement positions
//   pitch angles are offset to correspond to true leveling relative to ground
//   X-pattern removed due to reduce program memory size.
const float cal_roll_angles_t[CAL_SEQ_LEN]  = {0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,
                                             -7.0,-6.0,-5.0,-6.0, -1.0,0.0,1.0,0.0, 5.0,6.0,7.0,6.0};
const float cal_pitch_angles_t[CAL_SEQ_LEN] = {-4.0,-3.0,-2.0,-3.0, -1.0,0.0, 1.0,0.0, 2.0,3.0,4.0,3.0, 
                                              0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0};
//const float cal_roll_angles_x[CAL_SEQ_LEN]  = {-7.0,-6.0,-5.0,-6.0, -1.0,0.0,1.0,0.0, 5.0,6.0,7.0,6.0,
//                                               -7.0,-6.0,-5.0,-6.0, -1.0,0.0,1.0,0.0, 5.0,6.0,7.0,6.0};
//const float cal_pitch_angles_x[CAL_SEQ_LEN] = {-4.0,-3.0,-2.0,-3.0, -1.0,0.0, 1.0,0.0, 2.0,3.0,4.0,3.0, 
//                                               -2.0,-3.0,-4.0,-3.0,  1.0,0.0,-1.0,0.0, 4.0,3.0,2.0,3.0};
int   cal_sequence_counter = 0; // Counter to adjust time between positions
short cal_sequence_index = 0;   // Advances the position of the cal sequence

/* Working configuration set (Global) */
plat_config_t g_config;

/* Control loop state variables (Global) */
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

/* In-Application Programming (IAP) Entry Function */
typedef void (*IAP)(unsigned long [],unsigned long []);
IAP IAP_entry = (IAP) IAP_LOCATION;

/* ------------------------------------------------------------
   main()
   - Accept attitude position inputs and adjust motor PWM controls accordingly to correct
   - Calculation of motor control loop state parameters
   - Serial Interface for configuration and data output
   - Manage control loop coefficients, working and in memory
   - ADC monitoring of motor encoder voltage outputs and motor over-current conditions
   - Timer Interrupts trigger periodic functions
   - Loop forever
   -------------------------------------------------------------- 
*/
int main (void)
{
    int rtn = 0;

    init_system();
    PrintString("\n\n!NCAR Stabilized Platform\n");
    send_const();

    /* Init Coeff Structure with Default Coefficients */
    g_config.pitch_bi_translation = (float)        PITCH_BI_TRANSLATION;
    g_config.pitch_pwm_min        = (unsigned int) PITCH_PWM_MIN;
    g_config.pitch_pwm_max        = (unsigned int) PITCH_PWM_MAX;
    g_config.pitch_pwm_gain       = (float)        PITCH_PWM_GAIN;
    g_config.pitch_pwm_offset     = (unsigned int) PITCH_PWM_OFFSET;
    g_config.pitch_fb_gain        = (float)        PITCH_FB_GAIN;
    g_config.pitch_fb_offset      = (float)        PITCH_FB_OFFSET;
    g_config.roll_bi_translation  = (float)        ROLL_BI_TRANSLATION;
    g_config.roll_pwm_min         = (unsigned int) ROLL_PWM_MIN;
    g_config.roll_pwm_max         = (unsigned int) ROLL_PWM_MAX;
    g_config.roll_pwm_gain        = (float)        ROLL_PWM_GAIN;
    g_config.roll_pwm_offset      = (unsigned int) ROLL_PWM_OFFSET;
    g_config.roll_fb_gain         = (float)        ROLL_FB_GAIN;
    g_config.roll_fb_offset       = (float)        ROLL_FB_OFFSET;
    g_config.pitch_in_min         = (float)        (PITCH_IN_BASE_MIN - PITCH_BI_TRANSLATION);
    g_config.pitch_in_max         = (float)        (PITCH_IN_BASE_MAX - PITCH_BI_TRANSLATION);
    g_config.roll_in_min          = (float)        (ROLL_IN_BASE_MIN  - ROLL_BI_TRANSLATION);
    g_config.roll_in_max          = (float)        (ROLL_IN_BASE_MAX  - PITCH_BI_TRANSLATION);
    g_config.platform_type        = (unsigned short) PLATFORM;

    PrintString("!--- Default Coeffs ---\n");
    send_coeffs(&g_config);

    /* Load Coefficients from Memory, If Possible */
    PrintString("\n!Loading Coeffs from Memory\n");
    rtn = load_configurations(&g_config);
    if (0 == rtn) 
    {
        PrintString("!Load Successful\n");
    }
    else   // If unable to load from memory, then print error enter silent output mode.
    {
        PrintString("\n!--- Working Coeffs ---\n");
        send_coeffs(&g_config);
        PrintString("\n!Load ERRORS, check configs\n");
        output_data_mode = OUTPUT_MODE_SILENT; 
    }
  
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

        if (f_500us == 1) //T1 isr every 500us 
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
            {} // DO NOTHING

            // Update Control Loop
            execute_control_loop_iteration();

            // Increment Time Counters
            f_500us = 0;
            f_100ms += 1;
            f_1000ms += 1;
        }
        
        if (f_100ms == 200)      // 100 mseconds
        {
            send_output_data();
            f_100ms = 0;
        }
        if (f_1000ms == 2000)      // 1000 mseconds
        {
            f_1000ms = 0;
        }
    } // end while loop

    return 0; // Never reached
}    
// end main() ------------------------------------------------------------

/* ------------------------------------------------------------
   read_arinc_update_loop_input()
   - Read in ARINC attitude data from the HI-3588 SPI Interface
   - Extract pitch or roll value from ARINC word
   - Apply limits to attitude values 
   -------------------------------------------------------------- 
*/
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
                if (pitch_attitude > g_config.pitch_in_max) pitch_attitude = g_config.pitch_in_max;
                if (pitch_attitude < g_config.pitch_in_min) pitch_attitude = g_config.pitch_in_min;
            break;
                
            case ROLL_LABEL:
                if((data & SSM) != SSM) break;

                // Verify that attitude input does not exceed limits
                roll_attitude = attitude;
                if (roll_attitude > g_config.roll_in_max) roll_attitude = g_config.roll_in_max;
                if (roll_attitude < g_config.roll_in_min) roll_attitude = g_config.roll_in_min;
            break;

            default:
            break;
        }
    } 
    return;
}
// end read_arinc_update_loop_input() ------------------------------------------------------------

/* ------------------------------------------------------------
   test_pattern_update_loop_input()
   - Update Test Pattern input attitude values periodically
   - Cycle through pitch positions for each roll position 
   -------------------------------------------------------------- 
*/
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
// end test_pattern_update_loop_input() ------------------------------------------------------------

/* ------------------------------------------------------------
   calibration_update_loop_input()
   - Update Calibration input attitude values
   -------------------------------------------------------------- 
*/
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
        PrintString("!CAL DONE, now IMM\n");
    }
}
// end calibration_update_loop_input() ------------------------------------------------------------

/* ------------------------------------------------------------
   execute_control_loop_iteration()
   - Calculate control loop state parameters and update motor PWM values
   - Calculations different for closed-loop mode or open-loop mode
   - Closed-loop is 1st order loop with gain compensation
     - LOOP_TC Dominant pole
     - Secondary pole is motor response time
   - PWM setting is coarsely quantized to prevent limit cycles  
   - Motor Encoder values averaged over FB_AVE samples
   -------------------------------------------------------------- 
*/
void execute_control_loop_iteration(void)
{
    if (g_config.platform_type == TOP) // Pitch base polarity depends on platform as TOP or BOTTOM
    {
        pitch_base = -1.0 * (pitch_attitude + get_reference_frame_pitch_translation(reference_mode));
    }
    else
    {
        pitch_base = (pitch_attitude + get_reference_frame_pitch_translation(reference_mode));
    }
    roll_base = roll_attitude + get_reference_frame_roll_translation(reference_mode);

    // Convert Encoder ADC Output (10-bit) to attitude position
    pitch_attitudePos = ((float) pitch_encoder - g_config.pitch_fb_offset) * g_config.pitch_fb_gain;

    // Boxcar filter and decimate the attitude position
    pitch_posFilter = pitch_posFilter + pitch_attitudePos; // Accumulate
    pitch_decCntr = pitch_decCntr + 1;
    if (pitch_decCntr >= FB_AVE)
    {
        pitch_loopFB = pitch_posFilter / (float) FB_AVE; // Update the feedback value periodically per FB_AVE
        pitch_posFilter = 0.0;
        pitch_decCntr = 0;
    }

    // Convert Encoder ADC Output (10-bit) to attitude position
    roll_attitudePos = ((float) roll_encoder - g_config.roll_fb_offset) * g_config.roll_fb_gain;

    // Boxcar filter and decimate the attitude position
    roll_posFilter = roll_posFilter + roll_attitudePos; // Accumulate
    roll_decCntr = roll_decCntr + 1;
    if (roll_decCntr >= FB_AVE)
    {
        roll_loopFB = roll_posFilter / (float) FB_AVE; // Update the feedback value periodically per FB_AVE
        roll_posFilter = 0.0;
        roll_decCntr = 0;
    }


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
        pitch_pwm = (short) (pitch_loopErrK_filt * g_config.pitch_pwm_gain + g_config.pitch_pwm_offset);
        if (pitch_pwm > g_config.pitch_pwm_max) pitch_pwm = (short) g_config.pitch_pwm_max;
        if (pitch_pwm < g_config.pitch_pwm_min) pitch_pwm = (short) g_config.pitch_pwm_min;
        pitch_pwm = (pitch_pwm >> PWM_QUANT) << PWM_QUANT; // Truncate bits

        // Calculate Loop Error and saturate
        roll_loopErrK = LOOP_K*(roll_loopInput - roll_loopFB);
        if (roll_loopErrK > ROLL_LOOPERRK_MAX) roll_loopErrK = ROLL_LOOPERRK_MAX;
        if (roll_loopErrK < ROLL_LOOPERRK_MIN) roll_loopErrK = ROLL_LOOPERRK_MIN;

        // Filter with dominant pole
        roll_loopErrK_filt = (roll_loopErrK + roll_loopErrK_d - roll_loopErrK_filt_d * LOOP_TCN) / LOOP_TCP;
        roll_loopErrK_d = roll_loopErrK;            // delay a sample period
        roll_loopErrK_filt_d = roll_loopErrK_filt;

        // Convert Loop Error to PWM conversion factors, quantize, and limit
        roll_pwm = (short) (roll_loopErrK_filt * g_config.roll_pwm_gain + g_config.roll_pwm_offset);
        if (roll_pwm > g_config.roll_pwm_max) roll_pwm = (short) g_config.roll_pwm_max;
        if (roll_pwm < g_config.roll_pwm_min) roll_pwm = (short) g_config.roll_pwm_min;
        roll_pwm = (roll_pwm >> PWM_QUANT) << PWM_QUANT; // Truncate bits

    }
    else if (LOOP_MODE_OPEN == loop_mode)
    {
        pitch_pwm = (short)(pitch_base * g_config.pitch_pwm_gain + g_config.pitch_pwm_offset);
        if (pitch_pwm > g_config.pitch_pwm_max) pitch_pwm = g_config.pitch_pwm_max;
        if (pitch_pwm < g_config.pitch_pwm_min) pitch_pwm = g_config.pitch_pwm_min;

        roll_pwm =  (short)(roll_base * g_config.roll_pwm_gain + g_config.roll_pwm_offset);
        if (roll_pwm > g_config.roll_pwm_max) roll_pwm = g_config.roll_pwm_max;
        if (roll_pwm < g_config.roll_pwm_min) roll_pwm = g_config.roll_pwm_min;
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
// end execute_control_loop_iteration() ------------------------------------------------------------

/* ------------------------------------------------------------
   send_output_data()
   - Output the sample data on serial interface, comma delimited
   - Different outputs for different output modes
   -------------------------------------------------------------- 
*/
void send_output_data(void)
{
    if (OUTPUT_MODE_STREAM == output_data_mode)
    {
        sprintf(Buf,"#%.2f,%d,%d,%.2f,%d,%.2f,%d,%d,%.2f,%d\n",  
                    pitch_attitude, pitch_pwm, pitch_encoder, pitch_loopFB, pitch_ovcr,
                    roll_attitude,  roll_pwm,  roll_encoder,  roll_loopFB,  roll_ovcr);
        PrintString(Buf);

#ifdef DEBUG
        sprintf(Buf,"Pin:%.2f,Ppwm:%d,Penc:%d,Pfb:%.2f,Rin:%.2f,Rpwm:%d,Renc:%d,Rfb:%.2f\n",  
                    pitch_attitude, pitch_pwm, pitch_encoder, pitch_loopFB, roll_attitude, roll_pwm, roll_encoder, roll_loopFB); // 89-char
        PrintString(Buf);
        sprintf(Buf,"Err:%.3f,Erf:%.3f,Enc:%d,FB:%.2f\n", pitch_loopErrK, pitch_loopErrK_filt, pitch_encoder, pitch_loopFB);
        PrintString(Buf);
        sprintf(Buf,"Err:%.3f,Erf:%.3f,Enc:%d,FB:%.2f\n", roll_loopErrK, roll_loopErrK_filt, roll_encoder, roll_loopFB);
        PrintString(Buf);
#endif

    }
    else if (OUTPUT_MODE_EXAMPLE == output_data_mode)
    {
        sprintf(Buf,"#9.99,9999,999,9.99,999,-9.99,9999,999,-9.99,999\n");
        PrintString(Buf);
    }
    else if (OUTPUT_MODE_SILENT == output_data_mode)
    {} // DO NOTHING
    else
    {} // DO NOTHING
    return;    
}
// end send_output_data() ------------------------------------------------------------

/* ------------------------------------------------------------
   get_reference_frame_pitch_translation()
   - Get the pitch translation coefficient from the base reference frame to 
     the active reference frame, depends on the reference mode
   -------------------------------------------------------------- 
*/
float get_reference_frame_pitch_translation(short ref_mode)
{
    float translation = 0.0;
    if (REFERENCE_MODE_INS == ref_mode)
    {
        translation = g_config.pitch_bi_translation;
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
// end get_reference_frame_pitch_translation() ---------------------------------------

/* ------------------------------------------------------------
   get_reference_frame_roll_translation()
   - Get the roll translation coefficient from the base reference frame to 
     the active reference frame, depends on the reference mode
   -------------------------------------------------------------- 
*/
float get_reference_frame_roll_translation(short ref_mode)
{
    float translation = 0.0;
    if (REFERENCE_MODE_INS == ref_mode)
    {
        translation = g_config.roll_bi_translation;
    }
    return translation;
}
// end get_reference_frame_roll_translation() ---------------------------------------

/* ------------------------------------------------------------
   check_motor_overcurrent()
   -------------------------------------------------------------- 
*/
void check_motor_overcurrent(void)
{
    if ((roll_ovcr > MAX_IM) || (pitch_ovcr > MAX_IM)){   // Check motor overcurrent
      VICIntEnClr = 0xFFFFFFFF;    //disable all interrupts
      PWMMR4 = 0;
      PWMMR6 = 0;
      PWMLER = 0x7F; // enable PWM4-PWM6 match latch (reload
      PrintString("\nWARNING: Motor Overcurrent\n");
      while (1); // wait for a RESET
    }
    return;
}
// end check_motor_overcurrent() ---------------------------------------

/* ------------------------------------------------------------
   toggle_test_output()
   -------------------------------------------------------------- 
*/
void toggle_test_output(void)
{
    IO0SET = 0x00080000;		 //toggle debug
    IO0CLR = 0x00080000;
    return;
}
// end toggle_test_output() ---------------------------------------

/* ------------------------------------------------------------
   process_inchar()
   - Check for UART input characters and add them to the buffer
   - Characters only accumulated into command if start with '#'
   - Command finished (command_status set) when '\r' or '\n' received.
   -------------------------------------------------------------- 
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
// end process_inchar() ---------------------------------------

/* -------------------------------------------------------------- 
   process_command()
   - When a command is complete, process the command string stored in cmd_buf
   - The command is a string (NULL terminated), so string functions used
   -------------------------------------------------------------- 
 */
static void process_command(void)
{
    char * cmd_command = strtok(cmd_buf, cmd_delim);
    char * cmd_arg1     = strtok(NULL,   cmd_delim);
    char * cmd_arg2     = strtok(NULL,   cmd_delim);
    int rtn = -1;
    
    if (0 == strcmp("#NOR", cmd_command)) // Normal Input/Output Mode
    {
        PrintString("!NOR Rcvd\n");
        input_data_mode = INPUT_MODE_ARINC;
        output_data_mode = OUTPUT_MODE_STREAM;
    }
    else if (0 == strcmp("#CAL", cmd_command)) // Calibration Sequence
    {    
        PrintString("!CAL Rcvd\n");
        cal_sequence_index = 0;
        cal_sequence_counter = 0;
        input_data_mode = INPUT_MODE_CAL;
        output_data_mode = OUTPUT_MODE_SILENT;
    }
    else if (0 == strcmp("#OMN", cmd_command)) // Output Mode: Normal, Streaming
    {
        PrintString("!OMN Rcvd\n");
        output_data_mode = OUTPUT_MODE_STREAM;
    }
    else if (0 == strcmp("#OMS", cmd_command)) // Output Mode: Silent
    {
        PrintString("!OMS Rcvd\n");
        output_data_mode = OUTPUT_MODE_SILENT;
    }
    else if (0 == strcmp("#OMX", cmd_command)) // Output Mode: Example, Streaming
    {
        PrintString("!OMX Rcvd\n");
        output_data_mode = OUTPUT_MODE_EXAMPLE;
    }
    else if (0 == strcmp("#OMQ", cmd_command)) // Output Mode: Query, Single
    {
        PrintString("!OMQ Rcvd\n");
        output_data_mode = OUTPUT_MODE_STREAM;
        send_output_data();
        output_data_mode = OUTPUT_MODE_SILENT;
    }
    else if (0 == strcmp("#LMO", cmd_command)) // Loop Mode: Open-loop
    {    
        PrintString("!LMO Rcvd\n");
        loop_mode = LOOP_MODE_OPEN;
    }
    else if (0 == strcmp("#LMC", cmd_command)) // Loop Mode: closed-loop
    {
        PrintString("!LMC Rcvd\n");
        loop_mode = LOOP_MODE_CLOSED;
    }
    else if (0 == strcmp("#CFG", cmd_command)) // Output Configuration
    {
        PrintString("!CFG Rcvd\n");
        send_const();
        PrintString("!--- Working Coeffs ---\n");
        send_coeffs(&g_config);
    }
    else if (0 == strcmp("#RPC", cmd_command)) // Output Configuration Currently Stored in PROM
    {
        PrintString("!RPC Rcvd\n");
        read_coefficients();
    }
    else if (0 == strcmp("#WPE", cmd_command)) // Enable Writing to PROM 
    {
        PrintString("!WPE Rcvd\n");
        write_prom_mode = WRITE_PROM_MODE_ENABLED;
    }
    else if (0 == strcmp("#WPC", cmd_command)) // Write Coefficients to PROM 
    {
        PrintString("!WPC Rcvd\n");
        if (write_prom_mode == WRITE_PROM_MODE_ENABLED)
        {
            rtn = save_configurations(&g_config);
            if (0 == rtn)
            {
                PrintString("!WPC Success\n");
                read_coefficients();
            }
            else
            {
                PrintString("!ERROR: not saved\n");
            }
        }
        else
        {
            PrintString("!ERROR: set WPE\n");
        }
    }
    else if (0 == strcmp("#LPC", cmd_command)) // Load Coefficients from PROM 
    {
        PrintString("!LPC Rcvd\n");
        rtn = load_configurations(&g_config);
        if (0 == rtn)
        {
            PrintString("\n!-- Loaded Working Coeffs --\n");
            send_coeffs(&g_config);
        }
        else
        {
            send_coeffs(&g_config);
            PrintString("\n!Load ERRORS, check configs\n");
            output_data_mode = OUTPUT_MODE_SILENT; 
        }
    }
    else if ((0 == strcmp("#UC", cmd_command)) && (NULL != cmd_arg1) && (NULL != cmd_arg2)) // Update Coefficient 
    {
        int   temp_arg1 = 0;
        float temp_arg2 = 0.0;
        PrintString("!UC Rcvd\n");

        temp_arg1 = atoi(cmd_arg1);
        temp_arg2 = atof(cmd_arg2);
        update_configuration(&g_config, temp_arg1, temp_arg2);
        sprintf(Buf,"!Updated Coeff %d=%f\n",temp_arg1,temp_arg2);
        PrintString(Buf);
        PrintString("\n!--- Working Coeffs ---\n");
        send_coeffs(&g_config);
    }
    else if (0 == strcmp("#IMA", cmd_command)) // Input Mode: ARINC
    {    
        PrintString("!IMA Rcvd\n");
        input_data_mode = INPUT_MODE_ARINC;
        output_data_mode = OUTPUT_MODE_STREAM;
    }
    else if (0 == strcmp("#IMT", cmd_command)) // Input Mode: Test Pattern
    {    
        PrintString("!IMT Rcvd\n");
        reference_mode = REFERENCE_MODE_BASE;
        input_data_mode = INPUT_MODE_TP;
    }
    else if (0 == strcmp("#IMM", cmd_command)) // Input Mode: Manual
    {    
        PrintString("!IMM Rcvd\n");
        input_data_mode = INPUT_MODE_MANUAL;        
    }
    else if (0 == strcmp("#MZ", cmd_command)) // Manual Zero Position
    {    
        PrintString("!MZ Rcvd\n");
        if (INPUT_MODE_MANUAL == input_data_mode)
        {
            pitch_attitude = 0;
            roll_attitude = 0;
        }
        else
        {
            PrintString("!ERROR: set IMM\n");
        }
    }
    else if ((0 == strcmp("#MP", cmd_command)) && (NULL != cmd_arg1)) // Manual Pitch Change
    {
        int temp_arg = 0;
        PrintString("!MP Rcvd\n");
        if ((INPUT_MODE_MANUAL == input_data_mode) && (REFERENCE_MODE_INS == reference_mode))
        {
            temp_arg = atoi(cmd_arg1);
            if ((temp_arg > g_config.pitch_in_max)) temp_arg = g_config.pitch_in_max;
            if ((temp_arg < g_config.pitch_in_min)) temp_arg = g_config.pitch_in_min;
            pitch_attitude = temp_arg;       
		}
		else if	((INPUT_MODE_MANUAL == input_data_mode) && (REFERENCE_MODE_BASE == reference_mode))
		{
            temp_arg = atoi(cmd_arg1);
            if ((temp_arg > PITCH_IN_BASE_MAX)) temp_arg = PITCH_IN_BASE_MAX;
            if ((temp_arg < PITCH_IN_BASE_MIN)) temp_arg = PITCH_IN_BASE_MIN;
            pitch_attitude = temp_arg;       
        }
        else
        {
            PrintString("!ERROR: set IMM\n");
        }
    }
    else if ((0 == strcmp("#MR", cmd_command)) && (NULL != cmd_arg1)) // Manual Roll Change
    {
        int temp_arg = 0;
        PrintString("!MR Rcvd\n");
        if ((INPUT_MODE_MANUAL == input_data_mode) && (REFERENCE_MODE_INS == reference_mode))
        {
            temp_arg = atoi(cmd_arg1);
            if ((temp_arg > g_config.roll_in_max)) temp_arg = g_config.roll_in_max;
            if ((temp_arg < g_config.roll_in_min)) temp_arg = g_config.roll_in_min;
            roll_attitude = temp_arg;
        }
		else if	((INPUT_MODE_MANUAL == input_data_mode) && (REFERENCE_MODE_BASE == reference_mode))
		{
            temp_arg = atoi(cmd_arg1);
            if ((temp_arg > ROLL_IN_BASE_MAX)) temp_arg = ROLL_IN_BASE_MAX;
            if ((temp_arg < ROLL_IN_BASE_MIN)) temp_arg = ROLL_IN_BASE_MIN;
            roll_attitude = temp_arg;
        }
        else
        {
            PrintString("!ERROR: set IMM\n");
        }
    }
    else if (0 == strcmp("#RMI", cmd_command))  // Reference Mode: INS
    {    
        PrintString("!RMI Rcvd\n");
        reference_mode = REFERENCE_MODE_INS;
    }
    else if (0 == strcmp("#RMB", cmd_command)) // Reference Mode: Base
    {    
        PrintString("!RMB Rcvd\n");
        reference_mode = REFERENCE_MODE_BASE;
    }
    else
    {}
    return;
}
/* end process_command() */

/* -------------------------------------------------------------- 
   send_const()
   - Output constant variables on the serial interface
   - includes firmware rev., platform name, static control loop params
   -------------------------------------------------------------- 
 */
 void send_const(void)
{
    sprintf(Buf,"!FW Rev: %d\n",FW_REV);  
    PrintString(Buf);
    if (TOP == g_config.platform_type)
        PrintString("!Platform: TOP\n");
    else if (BOTTOM == g_config.platform_type)
        PrintString("!Platform: BOTTOM\n");
    else
        PrintString("!Platform: UNKNOWN\n");

    sprintf(Buf,"!Input Mode: %d\n!Output Mode: %d\n!Ref Mode: %d\n",
               input_data_mode,output_data_mode,reference_mode);  
    PrintString(Buf);

    sprintf(Buf,"!Loop Mode: %d\n!Loop K Gain: %f\n!Loop TC: %d\n",loop_mode,LOOP_K,LOOP_TC);  
    PrintString(Buf);
    sprintf(Buf,"!PWM Quant: %d\n",PWM_QUANT);
    PrintString(Buf);
    sprintf(Buf,"!FB Averages: %d\n",FB_AVE);
    PrintString(Buf);
    sprintf(Buf,"!Gain Adj: %f\n",GAIN_ADJ);
    PrintString(Buf);

    return;
}
// end send_const() ---------------------------------------

/* -------------------------------------------------------------- 
   send_const()
   param p_config: pointer to a coefficient set
   - Output control loop configurable variables on the serial interface
   -------------------------------------------------------------- 
 */
 void send_coeffs(plat_config_t * p_config)
{
    sprintf(Buf,"!Pitch Translation: %f\n",p_config->pitch_bi_translation);
    PrintString(Buf);
    sprintf(Buf,"!Pitch PWM Min: %d\n",p_config->pitch_pwm_min);
    PrintString(Buf);
    sprintf(Buf,"!Pitch PWM Max: %d\n",p_config->pitch_pwm_max);
    PrintString(Buf);
    sprintf(Buf,"!Pitch PWM Gain: %f\n",p_config->pitch_pwm_gain);
    PrintString(Buf);
    sprintf(Buf,"!Pitch PWM Offset: %d\n",p_config->pitch_pwm_offset);
    PrintString(Buf);
    sprintf(Buf,"!Pitch FB Gain: %f\n",p_config->pitch_fb_gain);
    PrintString(Buf);
    sprintf(Buf,"!Pitch FB Offset: %f\n",p_config->pitch_fb_offset);
    PrintString(Buf);

    sprintf(Buf,"!Roll Translation: %f\n",p_config->roll_bi_translation);
    PrintString(Buf);
    sprintf(Buf,"!Roll PWM Min: %d\n",p_config->roll_pwm_min);
    PrintString(Buf);
    sprintf(Buf,"!Roll PWM Max: %d\n",p_config->roll_pwm_max);
    PrintString(Buf);
    sprintf(Buf,"!Roll PWM Gain: %f\n",p_config->roll_pwm_gain);
    PrintString(Buf);
    sprintf(Buf,"!Roll PWM Offset: %d\n",p_config->roll_pwm_offset);
    PrintString(Buf);
    sprintf(Buf,"!Roll FB Gain: %f\n",p_config->roll_fb_gain);
    PrintString(Buf);
    sprintf(Buf,"!Roll FB Offset: %f\n",p_config->roll_fb_offset);
    PrintString(Buf);
    return;
}
// end send_coeffs() ---------------------------------------

/* -------------------------------------------------------------- 
   update_configuration()
   - Update a coefficient of a set, given the coefficient index and desired value
   - Changing _bi_translation also requires updating _in_min and _in_max
   -------------------------------------------------------------- 
 */
 void update_configuration(plat_config_t * p_config, unsigned int config_index, float value)
{
    switch (config_index)
    {
        case (0):
            p_config->pitch_bi_translation = value;
            p_config->pitch_in_min = (float) (PITCH_IN_BASE_MIN - p_config->pitch_bi_translation);
            p_config->pitch_in_max = (float) (PITCH_IN_BASE_MAX - p_config->pitch_bi_translation);
        break;

        case (1):
            p_config->pitch_pwm_min = (unsigned int) floor(fabsf(value));
        break;

        case (2):
            p_config->pitch_pwm_max = (unsigned int) floor(fabsf(value));
        break;

        case (3):
            p_config->pitch_pwm_gain = (float) fabsf(value);
        break;

        case (4):
            p_config->pitch_pwm_offset = (unsigned int) floor(fabsf(value));
        break;

        case (5):
            p_config->pitch_fb_gain = (float) fabsf(value);
        break;

        case (6):
            p_config->pitch_fb_offset = (float) fabsf(value);
        break;

        case (7):
            p_config->roll_bi_translation = value;
            p_config->roll_in_min = (float) (ROLL_IN_BASE_MIN - p_config->roll_bi_translation);
            p_config->roll_in_max = (float) (ROLL_IN_BASE_MAX - p_config->roll_bi_translation);
        break;

        case (8):
            p_config->roll_pwm_min = (unsigned int) floor(fabsf(value));
        break;

        case (9):
            p_config->roll_pwm_max = (unsigned int) floor(fabsf(value));
        break;

        case (10):
            p_config->roll_pwm_gain = (float) fabsf(value);
        break;

        case (11):
            p_config->roll_pwm_offset = (unsigned int) floor(fabsf(value));
        break;

        case (12):
            p_config->roll_fb_gain = (float) fabsf(value);
        break;

        case (13):
            p_config->roll_fb_offset = (float) fabsf(value);
        break;

        case (14):
            p_config->platform_type = (unsigned short) floor(fabsf(value));
        break;

        default:
        break;
    }
    return;
}
// end update_configuration() ---------------------------------------

/* -------------------------------------------------------------- 
   save_configurations()
   - Utilizes in-application programming (IAP) to update PROM memory
   - See LP2129 User's Manual for details of IAP commands
   - IAP PROM write must target a particular sector. For LP2129 (256kB size),
       sector CONFIG_SAVE_SECTOR is used to avoid program memory and boot sectors
   - PROM blocks must be written in 512 Byte Blocks. Copy coeff set over to a
       512Byte temp block to perform IAP memory write.
   - Interrupts must be disabled during IAP access
   - Lowest 32Bytes of RAM must be reserved for IAP functions. Project compiler
       should be configured to start RAM at >0x40000020 to avoid lowest memory
   - The sector should be erased before writing to avoid strange results when
       making multiple successive IAP write commands
   --------------------------------------------------------------
 */
 int save_configurations(plat_config_t * p_config)
{
    volatile unsigned long VICIntEnable_state;
    unsigned long command[5];        // Worst case command size is 5
    unsigned long result[2] = {1,1}; // Set to INVALID_COMMAND status as default
    unsigned char mem_temp[512];     // Temporary RAM memory for writing to PROM with correct size

    /* Transfer coefficient set to temp 512Byte block */
    memcpy((void *) &(mem_temp[0]), (void *) p_config, sizeof(*p_config));

    /* Disable Interrupts */
    VICIntEnable_state = VICIntEnable; // save VIC enable state
    VICIntEnClr = 0xFFFFFFFF;          // disable all interrupts

    /* Prep Flash for write (IAP) */
    command[0] = 50; // Prep sector for write command
    command[1] = CONFIG_SAVE_SECTOR; // Start Sector Number
    command[2] = CONFIG_SAVE_SECTOR; // End Sector Number
        // command[3:4] ignored in this case
    IAP_entry(command, result);
    if (result[0] != 0) {
        PrintString("!ERROR PrepMem Failed\n");
    }

    /* Erase Sector (IAP) */
    // Note: Writing to Memory without erasing first produces strange results for successive writes
    command[0] = 52;                // Erase Sector command
    command[1] = CONFIG_SAVE_SECTOR; // Start Sector Number
    command[2] = CONFIG_SAVE_SECTOR; // End Sector Number
    command[3] = 60000;             // System clock frequency in kHz, 60MHz (Assuming 12MHz XO and 5x PLL multiplier)
    IAP_entry(command, result);    
    if (result[0] != 0) {
        PrintString("!ERROR Erase Failed\n");
    }

    /* Blank Check Sector (IAP) */
    command[0] = 53;                // Erase Sector command
    command[1] = CONFIG_SAVE_SECTOR; // Start Sector Number
    command[2] = CONFIG_SAVE_SECTOR; // End Sector Number
    IAP_entry(command, result);
    if (result[0] != 0) {
        PrintString("!ERROR Check Failed\n");
    }

    /* Prep Flash for write (IAP) */
    command[0] = 50; // Prep sector for write command
    command[1] = CONFIG_SAVE_SECTOR; // Start Sector Number
    command[2] = CONFIG_SAVE_SECTOR; // End Sector Number
        // command[3:4] ignored in this case
    IAP_entry(command, result);
    if (result[0] != 0) {
        PrintString("!ERROR PrepMem Failed\n");
    }
    
    /* Write Flash (IAP) */
    command[0] = 51;                              // Copy RAM to Flash command
    command[1] = (unsigned long) CONFIG_SAVE_ADDR; // Destimation Flash Address
    command[2] = (unsigned long) &(mem_temp[0]);  // Source RAM Address
    command[3] = sizeof(mem_temp);                // Number of Bytes to write
    command[4] = 60000;                           // System clock frequency in kHz, 60MHz (Assuming 12MHz XO and 5x PLL multiplier)
    IAP_entry(command, result);    
    if (result[0] != 0) {
        PrintString("!ERROR Write Failed\n");
    }
    
    /* Enable Interrupts */
    VICIntEnable = VICIntEnable_state ; // Restore VIC Interrupts States
        
    return result[0];  // return the IAP write result, should be =0 for CMD_SUCCESS
}
// end save_configurations() ---------------------------------------

/* -------------------------------------------------------------- 
   load_configurations()
   - Recall coefficient set from PROM memory and load into the working set
   - Check the values in memory. If a value deviates too far from its default value
       then consider data corrupt and do not load. Meant to avoid potential damage to hardware
   --------------------------------------------------------------
 */
 int load_configurations(plat_config_t * p_config)
{
    plat_config_t config_temp;
    int rtn = 0;

    // Get the full coeff set from FLASH memory
    memcpy((void *) &config_temp, (void *) CONFIG_SAVE_ADDR, sizeof(config_temp));

    read_coefficients();

    // Check the deviation in stored memory relative to expected values
    //   If deviation is too great, then do not load coefficients
    //   Must check to prevent unpredictable behavior in the event of corrupted coefficients
    if ((config_temp.pitch_bi_translation < -4.0) || (config_temp.pitch_bi_translation > 0.0)) {
        PrintString("!Bad PITCH_BI_TRANSLATION\n");		
        config_temp.pitch_bi_translation = 0.0;
        rtn = -1;
    }
    if ((config_temp.pitch_pwm_min < PITCH_PWM_MIN - 1000) || (config_temp.pitch_pwm_min > PITCH_PWM_MIN + 1000)) {
        PrintString("!Bad PITCH_PWM_MIN\n");
        config_temp.pitch_pwm_min = PITCH_PWM_MIN;
        rtn = -1;
    }
    if ((config_temp.pitch_pwm_max < PITCH_PWM_MAX - 1000) || (config_temp.pitch_pwm_max > PITCH_PWM_MAX + 1000)) {
        PrintString("!Bad PITCH_PWM_MAX\n");
        config_temp.pitch_pwm_max = PITCH_PWM_MAX;
        rtn = -1;
    }
    if ((config_temp.pitch_pwm_gain < PITCH_PWM_GAIN/3.0) || (config_temp.pitch_pwm_gain > PITCH_PWM_GAIN*3.0)) {
        PrintString("!Bad PITCH_PWM_GAIN\n");
        config_temp.pitch_pwm_gain = PITCH_PWM_GAIN;
        rtn = -1;
    }
    if ((config_temp.pitch_pwm_offset < PITCH_PWM_OFFSET - 500) || (config_temp.pitch_pwm_offset > PITCH_PWM_OFFSET + 500)) {
        PrintString("!Bad PITCH_PWM_OFFSET\n");
        config_temp.pitch_pwm_offset = PITCH_PWM_OFFSET;
        rtn = -1;
    }
    if ((config_temp.pitch_fb_gain < PITCH_FB_GAIN/3.0) || (config_temp.pitch_fb_gain > PITCH_FB_GAIN*3.0)) {
        PrintString("!Bad PITCH_FB_GAIN\n");
        config_temp.pitch_fb_gain = PITCH_FB_GAIN;
        rtn = -1;
    }
    if ((config_temp.pitch_fb_offset < 50) || (config_temp.pitch_fb_offset > 500)) {
        PrintString("!Bad PITCH_FB_OFFSET\n");
        config_temp.pitch_fb_offset = 200;
        rtn = -1;
    }
    if ((config_temp.roll_bi_translation < -2.0) || (config_temp.roll_bi_translation > 2.0)) {
        PrintString("!Bad ROLL_BI_TRANSLATION\n");
        config_temp.roll_bi_translation = 0.0;
        rtn = -1;
    }
    if ((config_temp.roll_pwm_min < ROLL_PWM_MIN - 1000) || (config_temp.roll_pwm_min > ROLL_PWM_MIN + 1000)) {
        PrintString("!Bad ROLL_PWM_MIN\n");
        config_temp.roll_pwm_min = ROLL_PWM_MIN;
        rtn = -1;
    }
    if ((config_temp.roll_pwm_max < ROLL_PWM_MAX - 1000) || (config_temp.roll_pwm_max > ROLL_PWM_MAX + 1000)) {
        PrintString("!Bad ROLL_PWM_MAX\n");
        config_temp.roll_pwm_max = ROLL_PWM_MAX;
        rtn = -1;
    }
    if ((config_temp.roll_pwm_gain < ROLL_PWM_GAIN/3.0) || (config_temp.roll_pwm_gain > ROLL_PWM_GAIN*3.0)) {
        PrintString("!Bad ROLL_PWM_GAIN\n");
        config_temp.roll_pwm_gain = ROLL_PWM_GAIN;
        rtn = -1;
    }
    if ((config_temp.roll_pwm_offset < ROLL_PWM_OFFSET - 500) || (config_temp.roll_pwm_offset > ROLL_PWM_OFFSET + 500)) {
        PrintString("!Bad ROLL_PWM_OFFSET\n");
        config_temp.roll_pwm_offset = ROLL_PWM_OFFSET;
        rtn = -1;
    }
    if ((config_temp.roll_fb_gain < ROLL_FB_GAIN/4.0) || (config_temp.roll_fb_gain > ROLL_FB_GAIN*4.0)) {
        PrintString("!Bad ROLL_FB_GAIN\n");
        config_temp.roll_fb_gain = ROLL_FB_GAIN;
        rtn = -1;
    }
    if ((config_temp.roll_fb_offset < 50) || (config_temp.roll_fb_offset > 500)) {
        PrintString("!Bad ROLL_FB_OFFSET\n");
        config_temp.roll_fb_offset = 200;
        rtn = -1;
    }
    if ((config_temp.pitch_in_min < PITCH_IN_BASE_MIN - config_temp.pitch_bi_translation - 1.0) || 
        (config_temp.pitch_in_min > PITCH_IN_BASE_MIN - config_temp.pitch_bi_translation + 1.0)) 
    {
        PrintString("!Bad PITCH_IN_MIN\n");
        sprintf(Buf,"!%f %f\n",config_temp.pitch_in_min, PITCH_IN_BASE_MIN - config_temp.pitch_bi_translation);  
        PrintString(Buf);
        config_temp.pitch_in_min = (float) (PITCH_IN_BASE_MIN - PITCH_BI_TRANSLATION);
        rtn = -1;
    }
    if ((config_temp.pitch_in_max < PITCH_IN_BASE_MAX - config_temp.pitch_bi_translation - 1.0) || 
        (config_temp.pitch_in_max > PITCH_IN_BASE_MAX - config_temp.pitch_bi_translation + 1.0)) 
    {
        PrintString("!Bad PITCH_IN_MAX\n");
        config_temp.pitch_in_max = (float) (PITCH_IN_BASE_MAX - PITCH_BI_TRANSLATION);
        rtn = -1;
    }
    if ((config_temp.roll_in_min < ROLL_IN_BASE_MIN - config_temp.roll_bi_translation - 1.0) || 
        (config_temp.roll_in_min > ROLL_IN_BASE_MIN - config_temp.roll_bi_translation + 1.0)) 
    {
        PrintString("!Bad ROLL_IN_MIN\n");
        config_temp.roll_in_min = (float) (ROLL_IN_BASE_MIN - ROLL_BI_TRANSLATION);
        rtn = -1;
    }
    if ((config_temp.roll_in_max < ROLL_IN_BASE_MAX - config_temp.roll_bi_translation - 1.0) || 
        (config_temp.roll_in_max > ROLL_IN_BASE_MAX - config_temp.roll_bi_translation + 1.0)) 
    {
        PrintString("!Bad ROLL_IN_MAX\n");
        config_temp.roll_in_max = (float) (ROLL_IN_BASE_MAX - ROLL_BI_TRANSLATION);
        rtn = -1;
    }
    if ((config_temp.platform_type != TOP) && (config_temp.platform_type != BOTTOM)) {
        PrintString("!Bad PLATFORM_TYPE\n");
        config_temp.platform_type = TOP;
        rtn = -1;
    }

    // Copy temporary coeff set over to working set
    memcpy((void *) p_config, (void *) &config_temp, sizeof(config_temp));

    return rtn;
}
// end load_configurations() ---------------------------------------


/* ------------------------------------------------------------
   read_coefficients()
   - Read coefficients from PROM memory and display
   -------------------------------------------------------------- 
*/
void read_coefficients(void)
{
    plat_config_t config_temp;
    
    // Get the full coeff set from FLASH memory
    memcpy((void *) &config_temp, (void *) CONFIG_SAVE_ADDR, sizeof(config_temp));

    PrintString("\n!-- Configs in Memory --\n");
    send_coeffs(&config_temp);

    return;
}
// end read_coefficients() ---------------------------------------

/* ------------------------------------------------------------
   init_system()
   -------------------------------------------------------------- 
*/
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
    f_100ms = 0;
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
// end init_system() ---------------------------------------

/* ------------------------------------------------------------
   T1_Init()
   - Timer initialization for periodic function triggers
   -------------------------------------------------------------- 
*/
void T1_Init(void)
{
    VICVectAddr2 = (unsigned int) &T1_Isr;
    VICVectCntl2 = 0x25;    // Interrupt enabled
    VICIntEnable |= 0x20;   // Channel#5 is the Timer 1
    T1MR0 = 30000;          // = 0.5 msec / 16,67 nsec
    T1MCR = 3;              // Interrupt on Match0, reset timer on match
                            // Pclk = 60 MHz, timer count = 16,67 nsec
    T1TC  = 0;              // reset Timer counter
    T1TCR = 1;              // enable Timer
    return;
}
// end T1_Init() ---------------------------------------

/* ------------------------------------------------------------
   ADC_Init()
   -------------------------------------------------------------- 
*/
void ADC_Init(void)
{
    ADCR = 0x00200F00; // initialize ADC: ADC operational, no burst, select AIN 0-3 F=clkdiv
    return;
}
// end ADC_Init() ---------------------------------------

/* ------------------------------------------------------------
   read_adc()
   - Read from ADC channel, blocking
   -------------------------------------------------------------- 
*/
unsigned short read_adc(unsigned char channel)
{
    ADCR &= 0x00FFFFF0;                // Clear any previous conversion and channel
    ADCR |=  channel;                  // Set the channel
    ADCR |= 0x01000000;                // start conversion
    while((ADDR & 0x80000000) == 0);   // wait for conversion done
    return (ADDR & 0x0000FFFF) >> 6;
}
// end read_adc() ---------------------------------------

/* ------------------------------------------------------------
   PWM_Init()
   -------------------------------------------------------------- 
*/
void PWM_Init(void)                           
{
    PWMPR  = 12;         // prescaler to 60, timer runs at 60 MHz / 12 = 5 MHz
    PWMPC  = 0;          // prescale counter to 0
    PWMTC  = 0;          // reset timer to 0
    PWMMR0 = 15000;      // -> PMW base frequency = 5 MHz / 15000 = 333.3 Hz (3ms)
    PWMMR4 = pitch_pwm;  // Motor 1 pulse, 1 count is 0.1 deg
    PWMMR6 = roll_pwm;   // Motor 2 pulse
    PWMMCR = 0x00000002; // 
    PWMPCR = 0x5000;     // enable PWM4 PWM6 outputs
    PWMLER = 0x7F;       // enable PWM0 - PWM6 match latch (reload)
    PWMTCR = 0x09;       // enable PWM mode and start timer
    return;
}
// end PWM_Init() ---------------------------------------

/* ------------------------------------------------------------
   SPI_Init()
   - Setup SPI interface to ARINC Chip. Operates at 1MHz
   -------------------------------------------------------------- 
*/
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
// end SPI_Init() ---------------------------------------

/* ------------------------------------------------------------
   Uart0_Init()
   -------------------------------------------------------------- 
*/
void Uart0_Init(void)
{
    U0LCR = 0x83; // 8 bits, no Parity, 1 Stop bit
    U0DLL = 0x21; // 115.2Kbaud for 60MHz PCLK Clock
    U0DLM = 0x00;
    U0FCR = 0x07; // FIFO Enabled
    U0LCR = 0x03; // DLAB = 0
    return;
}
// end Uart0_Init() ---------------------------------------

/* ------------------------------------------------------------
   ua_outchar()
   - Output a character on the serial interface, blocking
   -------------------------------------------------------------- 
*/
static void ua_outchar(char c)
{
    U0THR = c;
    while(!(U0LSR & 0x40));
    return;
}
// end ua_outchar() ---------------------------------------

/* ------------------------------------------------------------
   PrintString()
   - Output an array of chars on serial interface
   -------------------------------------------------------------- 
*/
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
// end PrintString() ---------------------------------------


/* ------------------------------------------------------------
   T1_Isr()
   - Interrupt Service Routine for Timer 1
   - Triggered every 0.5ms
   -------------------------------------------------------------- 
*/
__irq void T1_Isr(void)
{
    f_500us = 1;     // asserts every 0.5 mseconds
    T1IR = 0x01;     // reset interrupt flag
    VICVectAddr = 0; // reset VIC
    return;
}
// end T1_Isr() ---------------------------------------
