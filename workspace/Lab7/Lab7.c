//#############################################################################
// FILE:   Lab7.c
//
// TITLE:  Segbot Balancing
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);

//had to also define interrupts for the ADC, this will allow an interrupt to run once the ADC is done
__interrupt void ADCA_ISR(void);

//these variables are counting the interrupt functions so that Tera Term will print
uint32_t ADCAInterruptCount = 0;
uint32_t SPIBInterruptCount = 0;

//variables for ADCA
int16_t adca2result = 0;
int16_t adca3result = 0;
float volt_adca2 = 0.0;
float volt_adca3 = 0.0;

//Predefinition of the setupSpib and eQEPS functions
void setupSpib(void);
void init_eQEPs(void);

//Predefinition of Encoders
float readEncLeft(void);
float readEncRight(void);

//Encoder Wheel Variables
float leftwheel = 0;
float rightwheel = 0;
float leftwheelft = 0;
float rightwheelft = 0;
float uleft = 5.0;
float uright = 5.0;

// Needed global Variables for Kalman filter
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -0.615; //we adjusted this so that it could better find the balance point, accounting for battery weight
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheel = 0;
float RightWheel = 0;
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};

// Kalman Filter variables
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

//Variables in SWI
float K1 = -60;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;
float thd_left_k = 0;
float thd_right_k = 0;
float thd_left_k1 = 0;
float thd_right_k1 = 0;
float LeftWheel_k1 = 0;
float RightWheel_k1 = 0;
float gyrorate_dot = 0;
float gyrorate_dot_k1 = 0;
float gyro_value_k1 = 0;
float ubal = 0;

//Exercise 4 Turn Variables
float WheelDif = 0;
float WheelDif_k1 = 0;
float vel_WheelDif = 0;
float vel_WheelDif_k1 = 0;
float turnref = 0;
float errorDif = 0;
float errorDif_k1 = 0;
float turn = 0;
float IK_turn = 0;
float IK_turn_k1 = 0;
float turn_controleffort = 0;
float turn_saturation_limit = 0;
float FwdBackOffset = 0;
float ut_Right = 0;
float ut_Left = 0;
float Kp = 3.0;
float Ki = 20.0;
float Kd = 0.08;
float turnrate = 0;
float turnrate_k1 = 0;
float turnref_k1 = 0;

//define functions for later use so that the main function can reference them
void setEPWM2A(float);
void setEPWM2B(float);

//global variables we defined for the robot motors
float saturation_limit = 10.0; //motor will only go from -10 to 10

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

//PWM variables
int16_t pwm1 = 0;
int16_t pwm2 = 0;

//voltage variables
float volt_spivalue2 = 0;
float volt_spivalue3 = 0;

//interrupt initializations
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;

//Exercise 4 initializations from lab 5
int16_t dummy= 0;
int16_t accelXraw = 0;
int16_t accelYraw = 0;
int16_t accelZraw = 0;
float accelXreading = 0;
float accelYreading = 0;
float accelZreading = 0;
int16_t gyroXraw = 0;
int16_t gyroYraw = 0;
int16_t gyroZraw = 0;
float gyroXreading = 0;
float gyroYreading = 0;
float gyroZreading = 0;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    //this sets the ADCA interrupt to PieVect 1.1
    PieVectTable.ADCA1_INT = &ADCA_ISR;

    //this is needed to have the interrupt be called correctly
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 20000); //set this to 20000 so the interrupt calls every 20 uS
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000); //set to 4000 to call every 4 ms
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    //EPwm2 will control the motors through an H bridge
    EALLOW;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; //this sets the counter to count upwards
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; //this sets the counter to free run, so it won't stop
    EPwm2Regs.TBCTL.bit.PHSEN = 0; //does not load the time base counter from the time base phase register
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; //clkdiv set to 0 will not scale the bits, if this is changed it will change the carrier frequency
    EPwm2Regs.TBCTR = 0; //starts timer at zero
    EPwm2Regs.TBPRD = 2500;//2500; we want the period to be 20 KHz (50 microseconds), where the counting frequency is 50 MHz
    //so by using 1/50 MHz * TBRRD = 0.00005, it is possible to set the correct TBPRD
    EPwm2Regs.CMPA.bit.CMPA = 0; //the duty cycle will start at 0, and can be manually controlled the motor
    EPwm2Regs.CMPB.bit.CMPB = 0; //because there are two motors you have to define two CMP
    //by starting the duty cycle at 0, the motors will start at stationary
    EPwm2Regs.AQCTLA.bit.CAU = 1; //this will clear when TBCTR = CMPA
    EPwm2Regs.AQCTLA.bit.ZRO = 2; //this will set the pin when TBCTR is zero
    EPwm2Regs.AQCTLB.bit.CBU = 1; //this will clear when TBCTR = CMPB
    EPwm2Regs.AQCTLB.bit.ZRO = 2; //this will set the pin when TBCTR is zero
    EPwm2Regs.TBPHS.bit.TBPHS = 0; //this will set the pin when TBCTR is zero
    EDIS;

    EALLOW;
    //This EPwm controls the timing of the ADC
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (?pulse? is the same as ?trigger?)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1 ms sample. Input clock is 50MHz. sampling at 10000Hz
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings

    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag

    //ADCA, defined for use with the joy stick
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be A0, uses channel 2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be A1, uses channel 3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //although SOC0 must be triggered first, it does not necessarily have to be the first ADC channel

    //The PinMux changes the pins, such that the original output pin will become an EPwm pin
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //right motor
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //left motor

    //This disables the pull up resistor
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    EDIS;

    init_eQEPs(); //Calling functions in main()

    setupSpib(); //Call this function in main() somewhere after the DINT; line of code.

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6;  //SPIB

    // Enable TINT0 in the PIE: Group 1 interrupt 1, used with ADCA1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable SPIB in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA, "Joystick 1 Volt: %f Joystick 2 Volt: %f Accel Z: %f Gyro X: %f leftwheel: %f rightwheel: %f\r\n", volt_adca2, volt_adca3, accelZreading, gyroXreading, leftwheel, rightwheel);
            //prints our values to Tera Term to check them
            UARTPrint = 0;
        }
    }
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    thd_left_k = 0.6*thd_left_k1+100*LeftWheel-100*LeftWheel_k1; //uses past states of theta dot and left wheel to find the the angular velocity of left wheel
    thd_right_k = 0.6*thd_right_k1+100*RightWheel-100*RightWheel_k1; //this is the given transfer function (100z-100)/(z-0.6)
    gyrorate_dot = 0.6*gyrorate_dot_k1+100*gyro_value-100*gyro_value_k1; //uses past state of gyro rate dot and gyro value to find gryo rate dot

    ubal = -K1*tilt_value-K2*gyro_value-K3*(thd_left_k+thd_right_k)/2-K4*gyrorate_dot; //then ubal is the K value matrix multiplied by the tilt and gyro matrix

    //Start of the code for Excercise 4, turn control
    WheelDif = LeftWheel-RightWheel; //the difference between the wheels
    vel_WheelDif = 0.333*vel_WheelDif_k1 +166.667*WheelDif -166.667*WheelDif_k1;//using a different trasnfer function we can find the velocity of the wheel difference
    turnref = turnref_k1 + 0.004*((turnrate+turnrate_k1)/2); //this is the integral of turnrate, which then gives the turn ref
    errorDif = turnref-WheelDif; //defining an error between the turn and the wheel
    IK_turn = IK_turn_k1 + 0.004*((errorDif+errorDif_k1)/2); //this is the integral of error, which is the other part of the control
    turn = Kp*errorDif+Ki*IK_turn-Kd*vel_WheelDif; //PID control using the error, integral of error, and velocity

    if (fabs(turn) > 3) //if turn is greater than 3, it will stop IK from increasing any higher. This will minimize the integral windup
    {
        IK_turn = IK_turn_k1;
    }

    if(turn >= 4) //the saturation limit will be the maximum, so if input exceeds the limit it will return the saturation limit
    {
        turn = 4;
    }
    else if(turn <= -4) //the same is true for the bottom of the saturation limit
    {
        turn = -4;
    }
    //the integral windup then minimizes the wheels from spinning rapidly to account for too much on the integral
    //then the saturation also makes sure that the wheels won't turn too fast and cause the segbot to tip over

    ut_Right = ubal/2.0 -turn+FwdBackOffset; //ut_right and ut_left account now for the balance, the turn, and the offset
    ut_Left = ubal/2.0 +turn+FwdBackOffset;

    setEPWM2B(-ut_Left); //these two functions then set the EPWM signal to be equal to the myeffort function, which will then use the scaling function below
    setEPWM2A(ut_Right);

    //all the definitions of past states to be used in the tranfer functions and integrals
    thd_left_k1 = thd_left_k;
    thd_right_k1 = thd_right_k;
    gyrorate_dot_k1 = gyrorate_dot;
    LeftWheel_k1 = LeftWheel;
    RightWheel_k1 = RightWheel;
    gyro_value_k1 = gyro_value;
    WheelDif_k1 = WheelDif;
    vel_WheelDif_k1 = vel_WheelDif;
    IK_turn_k1 = IK_turn;
    errorDif_k1 = errorDif;
    turnrate_k1 = turnrate;
    turnref_k1 = turnref;

    numSWIcalls++;
    DINT;
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    /* Excercise 3 and 4 left over from Lab 5. Included for reference in final project
    if(updown == 1) //the global variable is initialized at 1 so the function will start here
    {
        pwm1 = pwm1 + 10;
        pwm2 = pwm2 + 10;

        if(pwm1 >= 3000)
        {
            updown = 0; //this will create the other if statement to become true
        }
    }

    else if(updown == 0)
    {
        pwm1 = pwm1 - 10;
        pwm2 = pwm2 - 10;

        if(pwm1 <= 0)
        {
            updown = 1; //this will now bring it back to the first if statement and repeat the cycle
        }
    }*/

    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    /*
    leftwheel = readEncLeft(); //takes the encoder readings
    rightwheel = readEncRight();
    leftwheelft = leftwheel/5.02; //by pushing the robot along a foot, we can figure out how to change the encoder in radians to feet
    rightwheelft = rightwheel/5.02;

    PosLeft_K = leftwheelft; //this is redudant but mad more sense for us to read to create velocity
    PosRight_K = rightwheelft;
    VelLeft_K = (PosLeft_K-PosLeft_K_1)/0.004; //velocity calls the previous state and then takes the difference to create delta(position)/delta(time)
    VelRight_K = (PosRight_K-PosRight_K_1)/0.004;

    phi = (RWH/WR)*(rightwheel-leftwheel); //given formula for bearing
    theta_avg = 0.5*(rightwheel+leftwheel); //given formula for theta
    AngVelLeft_K = (leftwheel - leftwheel_K_1)/0.004; // similiar to VelLeft_K, where it takes the difference in angular position and divides by time for angular velocity
    AngVelRight_K = (rightwheel - rightwheel_K_1)/0.004;
    theta_dot_avg = 0.5*(AngVelLeft_K + AngVelRight_K); //by taking the sum and dividing we can get the average
    Xr_dot = RWH*theta_dot_avg*(cos(phi)); //this is the robot x velocity, found by using theta and phi
    Yr_dot = RWH*theta_dot_avg*(sin(phi));
    Xr = Xr_1+ 0.004*((Xr_dot + Xr_dot_1)/2); //then by integrating using trapezoid rule we can find the x position
    Yr = Yr_1 + 0.004*((Yr_dot + Yr_dot_1)/2);

    eturn = turn + (VelLeft_K - VelRight_K); //the difference between wheel speed causes a turn

    ek_left = Vref - VelLeft_K - kpturn*eturn; //this is the error term with the turn variable added. This is a part of the control
    ek_right = Vref - VelRight_K + kpturn*eturn;

    IK_left = IK_left_1 + 0.004*((ek_left+ek_left_1)/2); //this is the integral of error, which is the other part of the control
    IK_right = IK_right_1 + 0.004*((ek_right+ek_right_1)/2);

    uleft = (kp*ek_left)+(ki*IK_left); //the PI control, where Kp and Ki can be changed to provide a better control system for the robot
    uright = (kp*ek_right)+(ki*IK_right);

    if (fabs(uleft) > 10) //if uleft control effort is greater than 10, it will stop IK from increasing any higher. This will minimize the integral windup
    {
        IK_left = IK_left_1;
    }

    if (fabs(uright) > 10)
    {
        IK_right = IK_right_1;
    }

    PosLeft_K_1 = PosLeft_K; //all of this saves past states so that a new value can be made and the compared to the past state
    PosRight_K_1 = PosRight_K; //it is important to set it K_1 = K, otherwise the states would never change
    leftwheel_K_1 = leftwheel;
    rightwheel_K_1 = rightwheel;
    ek_left_1 = ek_left;
    ek_right_1 = ek_right;
    IK_left_1 = IK_left;
    IK_right_1 = IK_right;
    Xr_1 = Xr;
    Yr_1 = Yr;
    Xr_dot_1 = Xr_dot;
    Yr_dot_1 = Yr_dot;

    CpuTimer1.InterruptCount++;
     */
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    //commented this out so that we can see the blue light for the first 4 seconds of the robot starting up
    // Blink LaunchPad Blue LED
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    /*
    if ((CpuTimer2.InterruptCount % 5) == 0) {
        UARTPrint = 1;
     */
}

__interrupt void ADCA_ISR(void)
{
    //all of the ADC functions that were used in past labs
    adca2result = AdcaResultRegs.ADCRESULT0; //using SOC0 to look at ADCAIN2
    adca3result = AdcaResultRegs.ADCRESULT1; //using SOC1 to look at ADCAIN3

    volt_adca2 = adca2result*(3.0/4096.0); // Here covert ADCIND0 to volts
    volt_adca3 = adca3result*(3.0/4096.0); // Here covert ADCIND0 to volts

    //Beginning of Excercise 4
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00));
    SpibRegs.SPITXBUF = 0; //transmits accel x
    SpibRegs.SPITXBUF = 0; //transmits accel y
    SpibRegs.SPITXBUF = 0; //transmits accel z
    SpibRegs.SPITXBUF = 0; //transmits temp
    SpibRegs.SPITXBUF = 0; //transmits gryo x
    SpibRegs.SPITXBUF = 0; //transmits gryo y
    SpibRegs.SPITXBUF = 0; //transmits gyro z

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
void init_eQEPs(void)
{
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;

    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EDIS;
}

float readEncLeft(void)
{
    //left wheel encoder data from lab 6
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*((-2*PI)/12000)); //this takes the encoder data and turns it into radians
}

float readEncRight(void)
{
    //right wheel encoder data from lab 6
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*((2*PI)/12000)); //this takes the encoder data and turns it into radians
}

__interrupt void SPIB_isr(void)
{
    //reads the MPU data. The dummy value allows us to skip the temp reading and pull only accel and gyro
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummy = SpibRegs.SPIRXBUF;
    accelXraw = SpibRegs.SPIRXBUF;
    accelYraw = SpibRegs.SPIRXBUF;
    accelZraw = SpibRegs.SPIRXBUF;
    dummy = SpibRegs.SPIRXBUF;
    gyroXraw = SpibRegs.SPIRXBUF;
    gyroYraw = SpibRegs.SPIRXBUF;
    gyroZraw = SpibRegs.SPIRXBUF;

    //converts the raw data from MPU to usable numbers
    accelXreading = accelXraw*4.0/32767.0;
    accelYreading = accelYraw*4.0/32767.0;
    accelZreading = accelZraw*4.0/32767.0;

    gyroXreading = gyroXraw*250.0/32767.0;
    gyroYreading = gyroYraw*250.0/32767.0;
    gyroZreading = gyroZraw*250.0/32767.0;

    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    //All the following code was given to us. It works through the initilizations at robot start up and the Kalman Filter
    if(calibration_state == 0)
    {
        calibration_count++;
        if (calibration_count == 2000)
        {
            calibration_state = 1;
            calibration_count = 0;
        }
    }

    else if(calibration_state == 1)
    {
        accelx_offset+=accelXreading;
        accely_offset+=accelYreading;
        accelz_offset+=accelZreading;
        gyrox_offset+=gyroXreading;
        gyroy_offset+=gyroYreading;
        gyroz_offset+=gyroZreading;
        calibration_count++;
        if (calibration_count == 2000)
        {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    }

    else if(calibration_state == 2)
    {
        accelXreading -=(accelx_offset);
        accelYreading -=(accely_offset);
        accelZreading -=(accelz_offset-accelzBalancePoint);
        gyroXreading -= gyrox_offset;
        gyroYreading -= gyroy_offset;
        gyroZreading -= gyroz_offset;

        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyroXreading*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;

        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;

        // Update Step
        z = -accelZreading; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;

        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3)  // should never be greater than 3
        {
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }

    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

void setupSpib(void) //this whole function is used to define the SPIB
{
    int16_t temp = 0; //can use this a dummy variable

    //Step 1 copies initiliazations and brings them into the setup
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF 0x calls hexidecimal
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK?s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don?t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL =0x10; //Interrupt Level to 16 words or more received into FIFO causes
    //interrupt. This is just the initial setting for the register. Will be changed below

    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    //sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    //if we want to call individual registers then we need to pull slave select low and high every time
    SpibRegs.SPITXBUF = 0x1300; //this calls register 13 to 00
    SpibRegs.SPITXBUF = 0x0000; //then because the registers will move upwards, this is register 14 and 15 both going to 0
    SpibRegs.SPITXBUF = 0x0000; //then because the registers will move upwards, this is register 16 and 17 both going to 0
    SpibRegs.SPITXBUF = 0x0013; //then because the registers will move upwards, this is register 18 and 19 both going to 0
    SpibRegs.SPITXBUF = 0x0200; //then because the registers will move upwards, this is register 1A and 1B going to 0 and 13
    SpibRegs.SPITXBUF = 0x0806; //then because the registers will move upwards, this is register 1C and 1D going to 08 and 06
    SpibRegs.SPITXBUF = 0x0000; //then because the registers will move upwards, this is register 1E and 1F both going to 0
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7); // wait for the correct number of 16 bit values to be received into the RX FIFO
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High to end transmission
    //read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    //Simpler to start another address
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    SpibRegs.SPITXBUF = 0x2300; //this calls register 13 to 00
    SpibRegs.SPITXBUF = 0x408C; //then because the registers will move upwards, this is register 14 and 15 both going to 0
    SpibRegs.SPITXBUF = 0x0288; //then because the registers will move upwards, this is register 16 and 17 both going to 0
    SpibRegs.SPITXBUF = 0x0C0A;
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4); // wait for the correct number of 16 bit values to be received into the RX FIFO
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    //read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = 0x2A81; // Write to address 0x2A the value 0x81
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1); // wait for one byte to be received
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    //to get better control could also tweak these values to better zero the MPU
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EC); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x001E); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x000B); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x002C); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0019); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0042); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
void setEPWM2A(float controleffort) //copied from lab 3, this sets the control effort for the EPWM so that it doesn't saturate
{

    if(controleffort >= saturation_limit) //the saturation limit will be the maximum, so if input exceeds the limit it will return the saturation limit
    {
        controleffort = 10.0;
    }
    else if(controleffort <= (-saturation_limit)) //the same is true for the bottom of the saturation limit
    {
        controleffort = -10.0;
    }
    EPwm2Regs.CMPA.bit.CMPA = (125.0*controleffort)+1250.0; //this scales the duty cycle so that -10 to 10 corresponds to 0 from 2500
    //2500 is defined by TBPRD
}

void setEPWM2B(float controleffort) //copied from lab 3, this sets the control effort for the EPWM so that it doesn't saturate
{

    if(controleffort>saturation_limit) //the saturation limit will be the maximum, so if input exceeds the limit it will return the saturation limit
    {
        controleffort = 10.0;
    }
    else if(controleffort<(-saturation_limit)) //the same is true for the bottom of the saturation limit
    {
        controleffort = -10.0;
    }
    EPwm2Regs.CMPB.bit.CMPB = (125.0*controleffort)+1250.0; //this scales the duty cycle so that -10 to 10 corresponds to 0 from 2500
    //2500 is defined by TBPRD
}
