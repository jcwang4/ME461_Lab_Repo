//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
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

//define functions for later use so that the main function can reference them
void setEPWM2A(float);
void setEPWM2B(float);
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

//global variables we defined for the robot motors, servo motors, and the song length
int16_t updown = 1;
float saturation_limit = 10.0; //motor will only go from -10 to 10
float angle_max = 90.0; //servo can only go from -90 to 90 degrees
float myeffort = 0.0;
float servoeffort = 0.0;
int16_t updown_motor = 1;
int16_t updown_servo = 1;
uint16_t length = 0;

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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 375000); //we chose the Star Spangled Banner for our song so we had to slow down the song speed
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000); //this was lowered so that the LED would dim and brighten quickly by using CMPA

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200); //uncommented out for exercise 2
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

//-----------------------------------------------------------------------
//EPwm12 controls the LED's
    EPwm12Regs.TBCTL.bit.CTRMODE = 0; //this sets the counter to count upwards
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 2; //this sets the counter to free run, so it won't stop
    EPwm12Regs.TBCTL.bit.PHSEN = 0; // does not load the time base counter from the time base phase register
    EPwm12Regs.TBCTL.bit.CLKDIV = 0; //clkdiv set to 0 will not scale the bits, if this is changed it will change the carrier frequency
                                     //clkdiv by 5 will make it so that the frequency will be too low and the LED will hurt our eyes

    EPwm12Regs.TBCTR = 0; //starts timer at zero

    EPwm12Regs.TBPRD = 2500; //we want the period to be 20 KHz (50 microseconds), where the counting frequency is 50 MHz
                            //so by using 1/50 MHz * TBRRD = 0.00005, it is possible to set the correct TBPRD

    EPwm12Regs.CMPA.bit.CMPA = 0; //the duty cycle will start at 0, and can be manually controlled to set the brightness of the LED
                                  //the duty cycle of the PWM controls the brightness of the light

    EPwm12Regs.AQCTLA.bit.CAU = 1; //this will clear when TBCTR = CMPA
    EPwm12Regs.AQCTLA.bit.ZRO = 2; //this will set the pin when TBCTR is zero

    EPwm12Regs.TBPHS.bit.TBPHS = 0;//sets the phase to zero

//----------------------------------------
//EPwm2 will control the motors through an H bridge
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

//--------------------------------------
//EPwm8 will control the servo
    EPwm8Regs.TBCTL.bit.CTRMODE = 0; //this sets the counter to count upwards
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2; //this sets the counter to free run, so it won't stop
    EPwm8Regs.TBCTL.bit.PHSEN = 0; //does not load the time base counter from the time base phase register
    EPwm8Regs.TBCTL.bit.CLKDIV = 4; //the carrier frequency is changed to 50 Hz, but this number would be too high (above 16 bit)
                                    //therefore the bit scaling needs to be changed so that the 50 Hz is within TBPRD range

    EPwm8Regs.TBCTR = 0; //starts timer at zero

    EPwm8Regs.TBPRD = 62500; //By using the CLKDIV, it is now possible to use 50 Hz frequency because the EPWM8 is counting at
                             //50 MHz divided by 16

    EPwm8Regs.CMPA.bit.CMPA = 5000; //this starts the duty cycle at 8%, which is equivalent to 0 degrees on servo
    EPwm8Regs.CMPB.bit.CMPB = 5000; //this starts the duty cycle at 8%, which is equivalent to 0 degrees on servo
                                    //62500 * 0.08 = 5000

    EPwm8Regs.AQCTLA.bit.CAU = 1; //this will clear when TBCTR = CMPA
    EPwm8Regs.AQCTLA.bit.ZRO = 2; //this will set the pin when TBCTR is zero
    EPwm8Regs.AQCTLB.bit.CBU = 1; //this will clear when TBCTR = CMPB
    EPwm8Regs.AQCTLB.bit.ZRO = 2; //this will set the pin when TBCTR is zero


    EPwm8Regs.TBPHS.bit.TBPHS = 0; //this will set the pin when TBCTR is zero

//----------------------------------------
//EPwm9 will control the buzzer
    EPwm9Regs.TBCTL.bit.CTRMODE = 0; //this sets the counter to count upwards
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2; //this sets the counter to free run, so it won't stop
    EPwm9Regs.TBCTL.bit.PHSEN = 0; //does not load the time base counter from the time base phase register
    EPwm9Regs.TBCTL.bit.CLKDIV = 1;

    EPwm9Regs.TBCTR = 0; //starts timer at zero

    EPwm9Regs.TBPRD = 0;

   // EPwm9Regs.CMPA.bit.CMPA = 0; this gets rid of the intilization

    EPwm9Regs.AQCTLA.bit.CAU = 0; //does nothing when TBCTR = CMPA
    EPwm9Regs.AQCTLA.bit.ZRO = 3; //this will toggle the output, low will go high and vice versa

    EPwm9Regs.TBPHS.bit.TBPHS = 0; //this will set the pin when TBCTR is zero
//-------------------------------------
//The PinMux changes the pins, such that the original output pin will become an EPwm pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5); //LED 1
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //right motor
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //left motor
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); //servo 1
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1); //servo 2
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); //buzzer

//This disables the pull up resistor
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
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

    numSWIcalls++;

    DINT;
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%250) == 0) {
        //displayLEDletter(LEDdisplaynum); this is commented out so the dimming can actually be seen
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
    if (length < SONG_LENGTH)
    {
        EPwm9Regs.TBPRD = songarray[length++]; //this will increment the song upwards so that every time the TBPRD is reached the notes will proceed
    }
    else
    {
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0); //set PinMux to buzzer
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1; //then this will clear the bit so that nothing is else is played when the end of the song is reached
    }

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    CpuTimer2.InterruptCount++;

    //Code we have used in last couple of labs, this will create a print every time the counter reachs 50
    if ((CpuTimer2.InterruptCount % 50) == 0)
    {
        UARTPrint = 1;
    }

//Excercise 1 --------------------------------------------------
    if(updown == 1) //the global variable is initialized at 1 so the function will start here
    {
        EPwm12Regs.CMPA.bit.CMPA = EPwm12Regs.CMPA.bit.CMPA + 1; //this counts CMPA upwards, increasing brightness
        if(EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD) //then when CMPA is equal to TBPRD, it will then trigger updown = 0
        {
            updown = 0; //this will create the other if statement to become true
        }
    }

    if(updown == 0)
    {
        EPwm12Regs.CMPA.bit.CMPA = EPwm12Regs.CMPA.bit.CMPA - 1; //this counts CMPA downwards, decreasing brightness
        if(EPwm12Regs.CMPA.bit.CMPA == 0)
        {
            updown = 1; //this will now bring it back to the first if statement and repeat the cycle
        }
    }

// Exercise 2 ------------------------------------------------------------------
    if (myeffort >= 10 ) //if myeffort is greater than 10, then updown will be set to 0
    {
        updown_motor = 0;
    }

    else if (myeffort <= -10) //when myeffort is less than -10, it will be set to 1
    {
        updown_motor = 1;
    }

    if (updown_motor == 1) //this calls the updown = 1 case, which will increase the motor speed
    {
        myeffort += 0.001;
    }

    else //this calls the updown = 0 case, which will decrease the motor speed
    {
        myeffort -= 0.001;
    }

    setEPWM2A(myeffort); //these two functions then set the EPWM signal to be equal to the myeffort function, which will then use the scaling function below
    setEPWM2B(myeffort);

 //Exercise 3 ---------------------------------------------------------------------
    if (servoeffort >= 90 ) //if servoeffort is greater than 90, then updown_servo will be set to 0
       {
           updown_servo = 0;
       }
       else if (servoeffort <= -90) //when servoeffort is less than -10, it will be set to 1
       {
           updown_servo = 1;
       }

       if (updown_servo == 1)
       {
           servoeffort += 0.1; //increments servo upwards
       }
       else
       {
           servoeffort -= 0.1; //increments servo downwards
       }

       setEPWM8A_RCServo(servoeffort); //these two functions then set the EPWM signal to be equal to the servoeffort function, which will then use the scaling function below
       setEPWM8B_RCServo(servoeffort);

       if ((CpuTimer2.InterruptCount % 50) == 0)
       {
           UARTPrint = 1;
       }
}
//Exercise 2 Void Functions ---------------------------------------------------------
void setEPWM2A(float controleffort)
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

void setEPWM2B(float controleffort)
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

//Exercise 3 Void Functions ---------------------------------------------------------
void setEPWM8A_RCServo(float angle)
{

    if(angle >= angle_max) //the saturation limit will be the maximum, so if input exceeds the limit it will return the saturation limit
    {
        angle = 90.0;
    }
    else if(angle <= (-angle_max)) //the same is true for the bottom of the saturation limit
    {
        angle = -90.0;
    }
    EPwm8Regs.CMPA.bit.CMPA = (27.778*angle)+5000; //this scales the duty cycle so that -90 to 90 corresponds to 2500 from 7500
                                                   //5000 is defined by TBPRD
}

void setEPWM8B_RCServo(float angle)
{

    if(angle >= angle_max) //the saturation limit will be the maximum, so if input exceeds the limit it will return the saturation limit
    {
        angle = 90.0;
    }
    else if(angle <= (-angle_max)) //the same is true for the bottom of the saturation limit
    {
        angle = -90.0;
    }
    EPwm8Regs.CMPB.bit.CMPB = (27.778*angle)+5000; //this scales the duty cycle so that -90 to 90 corresponds to 2500 from 7500
                                                   //5000 is defined by TBPRD
}
