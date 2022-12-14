//#############################################################################
// FILE:   f28027_main.c
//
// TITLE:  Lab Starter
//#############################################################################

//
// Included Files
//
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "DSP28x_Project.h"
#include "f28027Serial.h"
#include "MadgwickAHRS.h"



#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
//
// Function Prototypes
//
__interrupt void adc_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SPI_RXint(void);
__interrupt void SWI_isr(void);

void setupSpia(void);        //for mpu9250
//void serialRXA(serial_t *s, char data);


int32_t numSWIcalls = 0;
uint16_t temp=0;
uint16_t XG_OFFSET_H        = 0x1300;
uint16_t XG_OFFSET_L        = 0x1400;
uint16_t YG_OFFSET_H        = 0x1500;
uint16_t YG_OFFSET_L        = 0x1600;
uint16_t ZG_OFFSET_H        = 0x1700;
uint16_t ZG_OFFSET_L        = 0x1800;

uint16_t SMPLRT_DIV         = 0x1900;

uint16_t CONFIG             = 0x1A00;
uint16_t GYRO_CONFIG        = 0x1B00;
uint16_t ACCEL_CONFIG       = 0x1C00;
uint16_t ACCEL_CONFIG_2     = 0x1D00;

uint16_t LP_ACCEL_ODR       = 0x1E00;
uint16_t WOM_THR            = 0x1F00;
uint16_t FIFO_EN            = 0x2300;

uint16_t I2C_MST_CTRL       = 0x2400;
uint16_t I2C_SLV0_ADDR      = 0x2500;
uint16_t I2C_SLV0_REG       = 0x2600;
uint16_t I2C_SLV0_CTRL      = 0x2700;
uint16_t I2C_SLV1_ADDR      = 0x2800;
uint16_t I2C_SLV1_REG       = 0x2900;
uint16_t I2C_SLV1_CTRL      = 0x2A00;

uint16_t INT_ENABLE         = 0x3800;
uint16_t INT_STATUS         = 0x3A00;

uint16_t I2C_SLV1_DO        = 0x6400;
uint16_t I2C_MST_DELAY_CTRL = 0x6700;

uint16_t USER_CTRL          = 0x6A00;
uint16_t PWR_MGMT_1         = 0x6B00;
uint16_t WHO_AM_I           = 0x7500;


//uint16_t XA_OFFSET_H        = 0x7700;
//uint16_t XA_OFFSET_L        = 0x7800;
//uint16_t YA_OFFSET_H        = 0x7A00;
//uint16_t YA_OFFSET_L        = 0x7B00;


// Offsets quad #1
//uint16_t XA_OFFSET_H        = 0x13;
//uint16_t XA_OFFSET_L        = 0x12;
//uint16_t YA_OFFSET_H        = 0xE6;
//uint16_t YA_OFFSET_L        = 0x18;


// Offsets quad #2
uint16_t XA_OFFSET_H        = 0x13;
uint16_t XA_OFFSET_L        = 0x12;
uint16_t YA_OFFSET_H        = 0xE6;
uint16_t YA_OFFSET_L        = 0x18;


uint16_t ZA_OFFSET_H        = 0x7D00;
uint16_t ZA_OFFSET_L        = 0x7E00;
uint16_t UARTPrint = 0;
int16_t Timer0Count = 0;
int16_t ADC0raw = 0;
int16_t ADC2raw = 0;
uint32_t ADCcount = 0;
int16_t readdata[4] = {0,0,0,0};
int16_t AccelXraw = 0;
int16_t AccelYraw = 0;
int16_t AccelZraw = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;
int16_t gyroXraw = 0;
int16_t gyroYraw = 0;
int16_t gyroZraw = 0;
float gyrox  = 0;
float gyroy  = 0;
float gyroz  = 0;
float print_accelx = 0;
float print_accely = 0;
float print_accelz = 0;
float print_gyrox  = 0;
float print_gyroy  = 0;
float print_gyroz  = 0;

int16_t calibration_state = 0;
int32_t calibration_count = 0;
float accelx_offset=0;
float accely_offset=0;
float accelz_offset=0;
float gyrox_offset=0;
float gyroy_offset=0;
float gyroz_offset=0;
uint16_t SPIenc_state = 0;
int16_t SPI_Interrupt_Finished = 1;

float compass_angle = 0.0;

float max_mx = 66;
float min_mx = -26;
float max_my = 94 ;
float min_my = 2;
float max_mz = -3;
float min_mz = -117;

float mx_offset = 0;// (53 + -92)/2.0;//(53 + -92)/2.0; //(max_mx + min_mx)/2.0
float my_offset = 0;// (53 + -92)/2.0;//(73 + -86)/2.0; // (max_my + min_my)/2.0
float mz_offset = 0;//(53 + -92)/2.0; //(273 + 125)/2.0; // (max_mz + min_mz)/2.0

float mx = 0;
float my = 0;
float mz = 0;

// Variables related to Madgwick algorithm
float Quaternions[4];
float DotQuaternions[4];
float mdg_beta = 0.1;

extern float q0;
extern float q1;
extern float q2;
extern float q3;

float R11 = 0;
float R21 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;

float mpu_roll = 0;
float mpu_pitch = 0;
float roll_tilt = 0;
float pitch_tilt = 0;
float yaw_tilt = 0;

float pitch_tiltrate = 0;
float roll_tiltrate = 0;
float yaw_tiltrate = 0;

int A1 = 0; //Top left propeller
int A2 = 0; //Bottom left propeller
int B1 = 0; //Top right propeller
int B2 = 0; // Bottom right propeller

float T_s = 0.005;
float integralThetaError = 0;
float integralThetaError_1 = 0;
float integralPhiError = 0;
float integralPhiError_1 = 0;

float tau_x = 0;
float tau_y = 0;
float tau_z = 0;
float F_z = 2300;


float Kp_y = 150;
float Ki_y = 0;
float Kd_y = 6.0;
float K_dd_y = 0.15;

float Kp_x = 150;
float Ki_x = 0.0;
float Kd_x = 6.0;
float K_dd_x = 0.15;

float Kp_z = 250;
float Ki_z = 0;
float Kd_z = 5.0;



//float thetaError = 0;
//float thetaError_1 = 0;
float w_x = 0;
float w_y = 0;
float w_z = 0;

float theta_est = 0;
float theta_est_1 = 0;
float theta_k = 0;
float theta_k_1 = 0;

float phi_est = 0;
float phi_est_1 = 0;
float phi_k = 0;
float phi_k_1 = 0;
float psi_est = 0;
float psi_est_1 = 0;
float psi_k = 0;
float psi_k_1 = 0;

float w_y_dot = 0;
float w_y_dot_1 = 0;
float w_y_1 = 0;

float w_x_dot = 0;
float w_x_dot_1 = 0;
float w_x_1 = 0;


//
// Main
//
void main(void)

{
    //
    // WARNING: Always ensure you call memcpy before running any functions from
    // RAM InitSysCtrl includes a call to a RAM based function and without a 
    // call to memcpy first, the processor will go "into the weeds"
    //
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the f2802x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the f2802x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in f2802x_DefaultIsr.c.
    // This function is found in f2802x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;            // This is needed to write to EALLOW protected registers
    PieVectTable.ADCINT1 = &adc_isr;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.TINT1 = &cpu_timer1_isr;
    PieVectTable.TINT2 = &cpu_timer2_isr;
    PieVectTable.SCIRXINTA = &RXAINT_recv_ready;
    PieVectTable.SCITXINTA = &TXAINT_data_sent;
    PieVectTable.SPIRXINTA = &SPI_RXint;
    PieVectTable.rsvd12_8 = &SWI_isr;
    EDIS;      // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize the Device Peripheral. This function can be
    //         found in f2802x_CpuTimers.c
    //
    InitCpuTimers();        // For this example, only initialize the Cpu Timers


    //
    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 60MHz CPU Freq, 1 second Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 60, 100000);
    ConfigCpuTimer(&CpuTimer1, 60, 1000000);
    ConfigCpuTimer(&CpuTimer2, 60, 1000000);

    //
    // To ensure precise timing, use write-only instructions to write to the 
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in f2802x_CpuTimers.h), the
    // below settings must also be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4001; //write-only instruction to set TSS bit = 0
    CpuTimer1Regs.TCR.all = 0x4001; //write-only instruction to set TSS bit = 0
    CpuTimer2Regs.TCR.all = 0x4001; //write-only instruction to set TSS bit = 0

    //
    // User specific code, enable interrupts
    //
    init_serial(&SerialA,115200);

    InitSpiGpio(); // Just Setup SPI pins
    setupSpia();
    InitAdc();
    InitAdcAio();

    //
    // Configure ADC
    EALLOW;
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;    // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;    // Disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 1;
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0;
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 2;
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0xB;  //EPWM4A
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 0xB;
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 40;
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 40;
    EDIS;


    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 2; // divide by 4 to get 0.005 ms
    EPwm4Regs.TBPRD = 37500;  // 0.00125  Input clock is 30MHz.  60Mhz/2
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A

    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A

    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    EDIS;

    EPwm1Regs.TBCTL.bit.CTRMODE = 0;      //set epwm2 to upcount mode
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;  // Divide by 16
    EPwm1Regs.TBPRD = 3000; //set epwm2 counter  50Hz
    EPwm1Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareB
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
    EPwm1Regs.CMPA.half.CMPA = 0;
    EPwm1Regs.CMPB = 0;

    EPwm2Regs.TBCTL.bit.CTRMODE = 0;      //set epwm2 to upcount mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;  // Divide by 16
    EPwm2Regs.TBPRD = 3000; //set epwm2 counter  50Hz
    EPwm2Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareB
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
    EPwm2Regs.CMPA.half.CMPA = 0;
    EPwm2Regs.CMPB = 0;

        //
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:
    //
    IER |= M_INT1;
    IER |= M_INT12;  // for SWI
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT9;  // SCIA
    IER |= M_INT6; // spia


    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 ADCA1
    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER12.bit.INTx8  = 1;

    EALLOW;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;   // Configure GPIO5
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
    EDIS;

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    //    SpiaRegs.SPICCR.bit.SPICHAR      = 0xF;  // Set to transmit 16 bits
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE
    //
    // Enable global Interrupts and higher priority real-time debug events
    //
    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA,"A0=%d A2=%d\r\n",ADC0raw,ADC2raw);  //Compiled printf minimal so only %d %x %u %c and maybe %s
//                        serial_printf(&SerialA,"Ax=%d,Ay=%d,Az=%d,Gx=%d,Gy=%d,Gz=%d \r\n",AccelXraw,AccelYraw,AccelZraw,gyroXraw,gyroYraw,gyroZraw);
            serial_printf(&SerialA,"Ax=%f,Ay=%f,Az=%f\r\n",accelx,accely,accelz);

            //           serial_printf(&SerialA,"r : %.2f, r_dot: %.2f, p : %.2f, p_dot : %.2f, y : %.2f, y_dot : %.2f \r\n",roll_tilt, roll_tiltrate, pitch_tilt, pitch_tiltrate, yaw_tilt, yaw_tiltrate);
//            serial_printf(&SerialA,"p : %.2f, p_dot : %.2f tau_y : %.2f, A1 : %.3f, A2 : %.3f \r\n",pitch_tilt, pitch_tiltrate, tau_y, A1, A2);
//            serial_printf(&SerialA,"accely : %.2f, accel_x : %.2f theta_est theta_k : %.2f w_y : %.2f : %.2f A1: %d A2: %d \r\n",accely, accelx, theta_est, theta_k, w_y, A1, A2);
//            serial_printf(&SerialA,"theta_est: %.2f phi_est: %.2f w_y: %.2f w_x: %.2f \r\n",theta_est, phi_est, w_y, w_x);


//             serial_printf(&SerialA,"r: %.2f rr: %.2f p: %.2f pr: %.2f y: %.2f yr: %.2f \r\n", roll_tilt,roll_tiltrate, pitch_tilt, pitch_tiltrate, yaw_tilt, yaw_tiltrate );

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
    GpioDataRegs.GPASET.bit.GPIO5 = 1;
    accelx = AccelXraw*4.0/32767.0;
    accely = AccelYraw*4.0/32767.0;
    accelz = AccelZraw*4.0/32767.0;

    gyrox  = gyroXraw*250.0/32767.0;
    gyroy  = gyroYraw*250.0/32767.0;
    gyroz  = gyroZraw*250.0/32767.0;


    mx = mx - mx_offset;   //Add in the offset from calibration
    my = my - my_offset;   //Add in the offset from calibration
    mz = mz - mz_offset; //Add in the offset from calibration

//    Use arctan to get compass angle if quadrotor flat
//    if(my < 0) {
//        compass_angle = 270.0-(((float)atan((mx/my)))*180.0/PI);
//    }
//    else if(my > 0) {
//        compass_angle = 90.0-(((float)atan((mx/my)))*180.0/PI);
//    }
//    else if((my == 0) && (mx < 0)) {
//        compass_angle = 180.0;
//    }
//    if((my == 0) && (mx > 0)) {
//        compass_angle = 0.0;
//    }

//    Use arctan to get compass angle if quadrotor flat
       if(my < 0) {
           compass_angle = 4.7124-(float)atan(mx/my);
       }
       else if(my > 0) {
           compass_angle = 1.571-(float)atan(mx/my);
       }
       else if((my == 0) && (mx < 0)) {
           compass_angle = PI;
       }
       if((my == 0) && (mx > 0)) {
           compass_angle = 0.0;
       }
    SPIenc_state = 99;  // no state

    if(calibration_state == 0){
        calibration_count++;

        if (calibration_count == 400) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){

        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;

        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;


        calibration_count++;
        if (calibration_count == 400) {
            calibration_state = 2;
            accelx_offset/=400.0;
            accely_offset/=400.0;
            accelz_offset/=400.0;
            gyrox_offset/=400.0;
            gyroy_offset/=400.0;
            gyroz_offset/=400.0;
            calibration_count = 0;
        }

    } else if(calibration_state == 2){
        accelx = accelx - (accelx_offset);
        accely = accely - (accely_offset);
        accelz = accelz - (accelz_offset)+1;
        print_accelx = accelx;
        print_accely = accely;
        print_accelz = accelz;
        gyrox  = gyrox - gyrox_offset;
        gyroy  = gyroy - gyroy_offset;
        gyroz  = gyroz - gyroz_offset;
        print_gyrox = gyrox;
        print_gyroy = gyroy;
        print_gyroz = gyroz;
//        mpu_roll = atan2(accely, accelz);
//        mpu_pitch = atan2(-accelx, sqrt(accely*accely + accelz*accelz) );

//        This is the Working Madgwick Algorithm -Ramya
        MadgwickAHRSupdateIMU(Quaternions, DotQuaternions, mdg_beta,
                              (PI/180.0)*gyrox,
                              (PI/180.0)*gyroy,
                              (PI/180.0)*gyroz,
                              accelx, accely, accelz);

        q0 = Quaternions[0];
        q1 = Quaternions[1];
        q2 = Quaternions[2];
        q3 = Quaternions[3];

        R11 = 2.0*q0*q0 -1 + 2.0*q1*q1;
        R21 = 2.0*(q1*q2 - q0*q3);
        R31 = 2.0*(q1*q3 + q0*q2);
        R32 = 2.0*(q2*q3 - q0*q1);
        R33 = 2.0*q0*q0 -1 + 2.0*q3*q3;


        //            Added - signs to angles
        roll_tilt =-1* atan2( R32, R33 );
        pitch_tilt = -1 * -atan( R31 / sqrt(1-R31*R31) );
//        yaw_tilt =-1 *  atan2( R21, R11 );

        //Measurements from gyroscope
        pitch_tiltrate = (gyroy*PI)/180.0; //z-axis: (gyroz*PI)/180.0; //y-axis: (gyroy*PI)/180.0; //x-axis: (gyrox*PI)/180.0; // rad/s
        roll_tiltrate = (gyrox*PI)/180.0; //z-axis: (gyroz*PI)/180.0; //y-axis: (gyroy*PI)/180.0; //x-axis: (gyrox*PI)/180.0; // rad/s
        yaw_tiltrate = (gyroz*PI)/180.0; //z-axis: (gyroz*PI)/180.0; //y-axis: (gyroy*PI)/180.0; //x-axis: (gyrox*PI)/180.0; // rad/s


        theta_est = -pitch_tilt;
        phi_est = -roll_tilt;
        psi_est = yaw_tilt;

        w_y = -pitch_tiltrate;
        w_x = -roll_tiltrate;
        w_z = yaw_tiltrate;

        w_y_dot = 0.6* w_y_dot_1 + 80 * w_y - 80*w_y_1;
        w_x_dot = 0.6* w_x_dot_1 + 80 * w_x - 80*w_x_1;

//        FT = 50;
//            f1_x = FT * sin(pitch_tilt);
////            f1_y = -FT *
//
////                    Motor commands A1, B1, A2, B2
//
//
//
//            k_F = 0;
//            k_M = 0;
////            length rotor to CoM (m)
//            l = 0.0916





//
//        w_y = -pitch_tiltrate;
//        w_x = - roll_tiltrate;
        w_z = yaw_tiltrate;
//
////        thetaError = -pitch_tilt;
//
//        theta_k = theta_k_1 + w_y * T_s;
//        phi_k = phi_k_1 + w_x * T_s;
        psi_k = psi_k_1 + w_z * T_s;
//
////        When accelx = 1, theta = 1.57 (pi/2)
////        theta_est = 0.9*(0.9*theta_k + 0.1* theta_k_1)+ 0.157 * accelx;
////        phi_est = 0.9*(0.9*phi_k + 0.1* phi_k_1)+ 0.157 * accely;
        psi_est = 0.9*(0.9*psi_k + 0.1* psi_k_1) + compass_angle*0.1;
//
//        theta_est = 0.999 * theta_k + .001 * accelx;

//        if((theta_est > PI / 2) || (theta_est < - PI / 2)){
//            integralThetaError = integralThetaError_1;
//        }
//        else{
//            integralThetaError = integralThetaError_1 + (theta_est + theta_est_1) / 2;
//        }

//        if((phi_est > PI / 2) || (phi_est < - PI / 2)){
//            integralPhiError = integralPhiError_1;
//        }
//        else{
//            integralPhiError = integralPhiError_1 + (phi_est + phi_est_1) / 2;
//        }



        integralPhiError = integralPhiError_1 + (phi_est + phi_est_1) / 2;

        tau_x = -Kp_x * (phi_est) -Kd_x * (w_x) - K_dd_x * w_x_dot - Ki_x * integralPhiError;
        tau_y = -Kp_y * (theta_est) -Kd_y * (w_y) - K_dd_y * w_y_dot - Ki_y * integralThetaError;
        tau_z = -Kp_z * (psi_est) -Kd_z * (w_z);


        A1 = tau_y*65  + tau_z *20 + F_z;
        A2 = -tau_y*65 + F_z + tau_z *20;
        B1 = tau_x * 65 + F_z - tau_z *20;
        B2 = -tau_x * 65 + F_z - tau_z *20;




//        A1 = tau_z *40;
//        A2 = tau_z *40;
//        B1 = -tau_z *40;
//        B2 = -tau_z *40;


        if(A1 > 3000){
            A1 = 3000;
        }
        else if(A1 <0){
            A1 = 0;
        }
        if(B1 > 3000){
            B1 = 3000;
        }
        else if(B1 <0){
            B1 = 0;
        }
        if(A2 > 3000){
            A2 = 3000;
        }
        else if(A2 <0){
            A2 = 0;
        }
        if(A2 > 3000){
            A2 = 3000;
        }
        if(B2 <0){
            B2 = 0;
        }
        else if(B2 > 3000){
            B2 = 3000;
        }

//        A1 = 0;
//        A2 = 0;
//        B1 = 0;
//        B2 = 0;
//        EPwm1Regs.CMPA.half.CMPA = A1;
//        EPwm1Regs.CMPB = B1;
//        EPwm2Regs.CMPA.half.CMPA = A2;
//        EPwm2Regs.CMPB = B2;

        EPwm1Regs.CMPA.half.CMPA = A1;
        EPwm2Regs.CMPB = A2;

        EPwm1Regs.CMPB = B1;
        EPwm2Regs.CMPA.half.CMPA = B2;


//        EPwm1Regs.CMPA.half.CMPA = 0;
//        EPwm1Regs.CMPB = 0;
//        EPwm2Regs.CMPA.half.CMPA = 0;
//        EPwm2Regs.CMPB = 0;

//        thetaError_1 = thetaError;
        theta_est_1 = theta_est;
        theta_k_1 = theta_k;
        phi_est_1 = phi_est;
        phi_k_1 = phi_k;
        psi_est_1 = psi_est;
        psi_k_1 = psi_k;

        w_y_dot_1 = w_y_dot;
        w_y_1 = w_y;
        w_x_dot_1 = w_x_dot;
        w_x_1 = w_x;

        integralPhiError_1 = integralPhiError;
        integralThetaError_1 = integralThetaError;

//        EPwm1Regs.CMPA.half.CMPA = A1;
//        EPwm1Regs.CMPB = B1;
//        EPwm2Regs.CMPA.half.CMPA = A2;
//        EPwm2Regs.CMPB = B2;
    }

    numSWIcalls++;

    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
    DINT;

}

__interrupt void adc_isr(void)
{

//    GpioDataRegs.GPASET.bit.GPIO5 = 1;
    ADC0raw = AdcResult.ADCRESULT0;
    ADC2raw = AdcResult.ADCRESULT1;

    ADCcount++;
    if ((ADCcount%50)==0) {
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    if (SPI_Interrupt_Finished == 1) {
        SPI_Interrupt_Finished = 0;
        SpiaRegs.SPIFFRX.bit.RXFFIL = 4;

        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

        SpiaRegs.SPITXBUF = ((0x8000)|(0x3A00));
        SpiaRegs.SPITXBUF = 0;
        SpiaRegs.SPITXBUF = 0;
        SpiaRegs.SPITXBUF = 0;
        SPIenc_state = 4;
    }



    //
    // Clear ADCINT1 flag reinitialize for next SOC
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    //
    // Acknowledge interrupt to PIE
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}




//
// cpu_timer0_isr -
//
__interrupt void cpu_timer0_isr(void)
{
    Timer0Count++;



    if (Timer0Count > 32000) { // Rolls over at 32767 so catch before that point
        Timer0Count = 0;
    }
   // GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    if((Timer0Count % 3) == 0){
    UARTPrint = 1;
    }
    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// cpu_timer1_isr -
//
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    //
    // The CPU acknowledges the interrupt
    //
    EDIS;
}

//
// cpu_timer2_isr -
//
__interrupt void cpu_timer2_isr(void)
{
    EALLOW;
    CpuTimer2.InterruptCount++;

    //
    // The CPU acknowledges the interrupt.
    //
    EDIS;
}

//// This function is called each time a char is recieved over UARTA.
//void serialRXA(serial_t *s, char data) {
//    numRXA ++;
//
//}

void SPI_RXint(void) {

    uint16_t i;

    switch (SPIenc_state) {
    case 4:
        GpioDataRegs.GPASET.bit.GPIO19 = 1;                 //Pull CS high as done R/W
        for (i=0; i<4; i++) {
            readdata[i] = SpiaRegs.SPIRXBUF; // readdata[0] is garbage
        }
        AccelXraw = readdata[1];
        AccelYraw = readdata[2];
        AccelZraw = readdata[3];

        SpiaRegs.SPIFFRX.bit.RXFFIL = 4;
        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

        SpiaRegs.SPITXBUF = ((0x8000)|(0x4200));
        SpiaRegs.SPITXBUF = 0;
        SpiaRegs.SPITXBUF = 0;
        SpiaRegs.SPITXBUF = 0;
        SPIenc_state = 5;
        break;
    case 5:
        GpioDataRegs.GPASET.bit.GPIO19 = 1;                 //Pull CS high as done R/W
        for (i=0; i<4; i++) {
            readdata[i] = SpiaRegs.SPIRXBUF; // readdata[0] is garbage
        }
        gyroXraw = readdata[1];
        gyroYraw = readdata[2];
        gyroZraw = readdata[3];

        SpiaRegs.SPIFFRX.bit.RXFFIL = 4;
        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

        SpiaRegs.SPITXBUF = ((0x8000)|(0x4A00));
        SpiaRegs.SPITXBUF = 0;
        SpiaRegs.SPITXBUF = 0;
        SpiaRegs.SPITXBUF = 0;

        SPIenc_state = 6;

        break;
    case 6:
        GpioDataRegs.GPASET.bit.GPIO19 = 1;                 //Pull CS high as done R/W
        for (i=0; i<4; i++) {
            readdata[i] = SpiaRegs.SPIRXBUF; // readdata[0] is garbage
        }
        mx = (float)readdata[1];
        my = (float)readdata[2];
        mz = (float)readdata[3];

        SPI_Interrupt_Finished = 1;
        PieCtrlRegs.PIEIFR12.bit.INTx8 = 1;
        break;

    }

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=0){
        temp = SpiaRegs.SPIRXBUF;
    }

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE

}



//
// End of File
//
void setupSpia(void)        //for mpu9250
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (SIMO)
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;   // Configure GPIO16 as SIMO

    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (SOMI)
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;   // Configure GPIO17 as SOMI

    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;    // Enable pull-up on GPIO18 (CLK)
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;   // Configure GPIO18 as CLK

    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;

    EDIS;

    EALLOW;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    SpiaRegs.SPICCR.bit.SPISWRESET   = 0;    // Put SPI in Reset

    SpiaRegs.SPICTL.bit.CLK_PHASE    = 1;
    SpiaRegs.SPICCR.bit.CLKPOLARITY  = 0;

    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;    // SPI master
    SpiaRegs.SPICCR.bit.SPICHAR      = 0xF;  // Set to transmit 16 bits
    SpiaRegs.SPICTL.bit.TALK         = 1;    // Enables transmission for the 4-pin option
    SpiaRegs.SPIPRI.bit.FREE         = 1;    // Free run, continue SPI operation
    SpiaRegs.SPICTL.bit.SPIINTENA    = 0;    // Disables the SPI interrupt

    SpiaRegs.SPIBRR = LSPCLK_HZ/1000000L - 1;

    SpiaRegs.SPISTS.all              = 0x0000;

    SpiaRegs.SPIFFTX.bit.SPIRST      = 1;    // SPI FIFO can resume transmit or receive.
    SpiaRegs.SPIFFTX.bit.SPIFFENA    = 1;    // SPI FIFO enhancements are enabled
    SpiaRegs.SPIFFTX.bit.TXFIFO      = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR  = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag

    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR  = 1;    // Write 1 to clear SPIFFRX[RXFFOVF]
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR  = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag
    SpiaRegs.SPIFFRX.bit.RXFFIENA    = 1;    // RX FIFO interrupt based on RXFFIL match

    SpiaRegs.SPIFFCT.bit.TXDLY       = 0; // The next word in the TX FIFO buffer is transferred to SPITXBUF immediately upon completion of transmission of the previous word.

    SpiaRegs.SPICCR.bit.SPISWRESET   = 1;    // Pull the SPI out of reset

    SpiaRegs.SPIFFTX.bit.TXFIFO      = 1;    // Release transmit FIFO from reset.
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpiaRegs.SPICTL.bit.SPIINTENA    = 0;    // Disable the SPI interrupt.
    SpiaRegs.SPIFFRX.bit.RXFFIL      = 4;    // A RX FIFO interrupt request is generated when there are 1 or more words in the RX buffer.

    //-----------------------------------------------------------------------------------------------------------------

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (0x1300         | 0x00); // 0x1300 (0x0000)
    SpiaRegs.SPITXBUF = (0x0000         | 0x00); // 0x1400 (0x0000),       0x1500 (0x0000)
    SpiaRegs.SPITXBUF = (0x0000         | 0x00); // 0x1600 (0x0000),       0x1700 (0x0000)
    SpiaRegs.SPITXBUF = (0x0000         | 0x13); // 0x1800 (0x0000),       0x1900 (0x0013)

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (0x1A00         | 0x02); //0x1A00 (0x0002),

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (0x1B00         | 0x00); // 0x1B00 (0x0000), 250 dps
    SpiaRegs.SPITXBUF = (0x0800         | 0x06); // 0x1C00 (0x0008), +-4g, 0x1D00 (0x0006), 5Hz
    SpiaRegs.SPITXBUF = (0x0000         | 0x00); // 0x1E00 (0x0000),       0x1F00 (0x0000)

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=3);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (FIFO_EN        | 0x0000); // 0x2300 (0x0000)
    SpiaRegs.SPITXBUF = (0x4000         | 0x008C); // 0x2400 (0x0040),       0x2500 (0x008C)
    SpiaRegs.SPITXBUF = (0x0100         | 0x00D8); // 0x2600 (0x0001),       0x2700 (0x00D8)
    SpiaRegs.SPITXBUF = (0x0C00         | 0x000A); // 0x2800 (0x000C),       0x2900 (0x000A)
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (I2C_SLV1_CTRL | 0x0081);  // 0x2A00 (0x0081)
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (INT_ENABLE | 0x0001);  // 0x3800
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (INT_STATUS | 0x0001);  // 0x3A00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (I2C_SLV1_DO | 0x0001);  // 0x6400
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (I2C_MST_DELAY_CTRL | 0x0003); // 0x6700
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (USER_CTRL | 0x0020);  // 0x6A00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (PWR_MGMT_1 | 0x0001); // 0x6B00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (WHO_AM_I | 0x0071);  // 0x7500
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

//    QUAD #1 accelerometer offsets
//    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
//    SpiaRegs.SPITXBUF = (XA_OFFSET_H | 0x13); // 0x7700
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
//    GpioDataRegs.GPASET.bit.GPIO19 = 1;
//    temp = SpiaRegs.SPIRXBUF;
//
//    DELAY_US(10);
//
//    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
//    SpiaRegs.SPITXBUF = (XA_OFFSET_L | 0x22); // 0x7800
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
//    GpioDataRegs.GPASET.bit.GPIO19 = 1;
//    temp = SpiaRegs.SPIRXBUF;
//
//    DELAY_US(10);
//
//    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
//    SpiaRegs.SPITXBUF = (YA_OFFSET_H | 0xE6); // 0x7A00
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
//    GpioDataRegs.GPASET.bit.GPIO19 = 1;
//    temp = SpiaRegs.SPIRXBUF;
//
//    DELAY_US(10);
//
//    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
//    SpiaRegs.SPITXBUF = (YA_OFFSET_L | 0x18); // 0x7B00
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
//    GpioDataRegs.GPASET.bit.GPIO19 = 1;
//    temp = SpiaRegs.SPIRXBUF;
//
//    DELAY_US(10);

//    QUAD #2 accelerometer offsets
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
        SpiaRegs.SPITXBUF = (XA_OFFSET_H | 0x13); // 0x7700
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPASET.bit.GPIO19 = 1;
        temp = SpiaRegs.SPIRXBUF;

        DELAY_US(10);

        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
        SpiaRegs.SPITXBUF = (XA_OFFSET_L | 0x22); // 0x7800
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPASET.bit.GPIO19 = 1;
        temp = SpiaRegs.SPIRXBUF;

        DELAY_US(10);

        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
        SpiaRegs.SPITXBUF = (YA_OFFSET_H | 0xB1); // 0x7A00
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPASET.bit.GPIO19 = 1;
        temp = SpiaRegs.SPIRXBUF;

        DELAY_US(10);

        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
        SpiaRegs.SPITXBUF = (YA_OFFSET_L | 0xE0); // 0x7B00
        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPASET.bit.GPIO19 = 1;
        temp = SpiaRegs.SPIRXBUF;

        DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (ZA_OFFSET_H | 0x0021); // 0x7D00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (ZA_OFFSET_L | 0x0050); // 0x7E00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=0){
        temp = SpiaRegs.SPIRXBUF;
    }

    DELAY_US(50);

    SpiaRegs.SPICTL.bit.SPIINTENA = 0;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;  //Enable PIE 6.1 interrupt
}
