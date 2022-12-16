/* SERIAL.C: This code is designed to act as a low-level serial driver for
    higher-level programming.  Ideally, one could simply call init_serial()
    to initialize the serial port, then use serial_send("data", 4) to send
    an array of data (8-bit unsigned character strings).

    WRITTEN BY : Paul Miller <pamiller@uiuc.edu>
    $Id: serial.c,v 1.4 2003/08/08 16:08:56 paul Exp $
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <buffer.h>
#include <f28027Serial.h>
#include <F2802x_Sci.h>

//extern float roll_tiltrate;
//extern float pitch_tiltrate;
extern float Kp_y;

serial_t SerialA;
extern uint16_t UARTPrint;
char RXAdata = 0;
uint32_t numRXA = 0;
int16_t ESP8266whichcommand = 0;
int16_t ESP8266insidecommands = 0;
char sendto8266[99];
char send8266Command[64];
int16_t send8266CommandSize = 0;
int16_t sendingto8266 = 0;
int16_t sendtoSize = 0;


float value1 = 0;
float value2 = 0;
float value3 = 0;
float value4 = 0;
float value5 = 0;
float value6 = 0;

float myfloat1=0;

int16_t collect = 0;
char gusarray[300];
int16_t g = 0;
char debug_array[200];
int16_t debugi = 0;
char sendback[10];
char past4[4] = {'\0','\0','\0','\0'};

extern uint16_t UARTPrint;

uint16_t init_serial(serial_t *s, uint32_t baud)
{
    volatile struct SCI_REGS *sci;
    uint32_t clk;

    if (s == &SerialA) {
        sci = &SciaRegs;
        s->sci = sci;

        InitSciGpio();

    } else {
        return 1;
    }

    init_buffer(&s->TX);

    /* init for standard baud,8N1 comm */
    sci->SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    sci->SCICCR.all = 0x0;
    sci->SCICTL1.all = 0x0;
    sci->SCICTL2.all = 0x0;
    sci->SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    sci->SCILBAUD = clk & 0xFF;
    sci->SCIHBAUD = (clk >> 8) & 0xFF;

    sci->SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    sci->SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    sci->SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    sci->SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    sci->SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    sci->SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    sci->SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    sci->SCIFFTX.bit.TXFIFOXRESET = 0;
    sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    sci->SCIFFTX.bit.TXFIFOXRESET = 1;

    sci->SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    sci->SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    sci->SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.TXWAKE = 0;
    sci->SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    sci->SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    sci->SCICTL1.bit.SWRESET = 1;       // re-enable SCI

    /* enable PIE interrupts */
    if (s == &SerialA) {
        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
        IER |= (M_INT9);
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP9);

    } 

    return 0;
}

void uninit_serial(serial_t *s)
{
    volatile struct SCI_REGS *sci = s->sci;

    /* disable PIE interrupts */
    if (s == &SerialA) {
        PieCtrlRegs.PIEIER9.bit.INTx1 = 0;
        PieCtrlRegs.PIEIER9.bit.INTx2 = 0;
        IER &= ~M_INT9;
    }

    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.RXENA = 0;         // disable SCI receiver
    sci->SCICTL1.bit.TXENA = 0;         // disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/

uint16_t serial_send(serial_t *s, char *data, uint16_t len)
{
    uint16_t i = 0;
    if (len && s->TX.size < BUF_SIZE) {
        for (i = 0; i < len; i++) {
            if (buf_write_1(&s->TX, data[i] & 0x00FF) != 0) break;
        }
        s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
        s->sci->SCIFFTX.bit.TXFFIENA = 1;       // TX: enable fifo interrupt
    }
    return i;
}


/***************************************************************************
 * TXxINT_DATA_SENT()
 *
 * Executed when transmission is ready for additional data.  These functions
 * read the next char of data and put it in the TXBUF register for transfer.
 ***************************************************************************/
#ifdef _FLASH
#pragma CODE_SECTION(TXAINT_data_sent, "ramfuncs");
#endif
__interrupt void TXAINT_data_sent(void)
{
    char data;
    if (buf_read_1(&SerialA.TX,0,&data) == 0) {
        while ( (buf_read_1(&SerialA.TX,0,&data) == 0)
                && (SerialA.sci->SCIFFTX.bit.TXFFST != 0x4) ) {
            buf_remove(&SerialA.TX, 1);
            SerialA.sci->SCITXBUF = data;
        }
    } else {
        SerialA.sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    }
    SerialA.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}




//float pinkx = 0;
//float pinky = 0;
//float bluex = 0;
//float bluey = 0;
//
////for serial com with pi
//typedef union {
//    uint16_t parts[2];
//    float value;
//} float_int;
//
//uint16_t com_state = 0;
//char send_sci[40]; //send the float
//float_int to_send[10];
//uint16_t received_count = 0;
//uint16_t opti_count = 0;
//float_int received_pi[10];
//char temp_MSB = 0;
//
//

int32_t daninISR = 0;
#ifdef _FLASH
#pragma CODE_SECTION(RXAINT_recv_ready, "ramfuncs");
#endif
__interrupt void RXAINT_recv_ready(void)
{
    RXAdata = SciaRegs.SCIRXBUF.all;
    daninISR++;
    /* SCI PE or FE error */
    if (RXAdata & 0xC000) {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
    } else {
        RXAdata = RXAdata & 0x00FF;
        //if (debugi < 100) {
            debug_array[debugi%200] = RXAdata;
            debugi++;
        //}


        if (ESP8266insidecommands == 1) {
            if (ESP8266whichcommand == 1) {
                past4[0] = RXAdata;
                if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                    ESP8266whichcommand = 2;
                    serial_send(&SerialA,"AT+CWMODE=3\r\n",strlen("AT+CWMODE=3\r\n"));
                    past4[0] = '\0';
                    past4[1] = '\0';
                    past4[2] = '\0';
                    past4[3] = '\0';
                }
                past4[3] = past4[2];
                past4[2] = past4[1];
                past4[1] = past4[0];
            } else if (ESP8266whichcommand == 2) {
                past4[0] = RXAdata;
                if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                    ESP8266whichcommand = 3;

                    serial_send(&SerialA,"AT+CWJAP=\"MECHNIGHT\",\"f33dback5\"\r\n",strlen("AT+CWJAP=\"MECHNIGHT\",\"f33dback5\"\r\n"));
                    past4[0] = '\0';
                    past4[1] = '\0';
                    past4[2] = '\0';
                    past4[3] = '\0';
                }
                past4[3] = past4[2];
                past4[2] = past4[1];
                past4[1] = past4[0];
            } else if (ESP8266whichcommand == 3) {
                past4[0] = RXAdata;
                if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                    ESP8266whichcommand = 4;
                    serial_send(&SerialA, "AT+CIPSTA=\"192.168.1.56\"\r\n", strlen("AT+CIPSTA=\"192.168.1.56\"\r\n")); //IP address to type into labview
                    past4[0] = '\0';
                    past4[1] = '\0';
                    past4[2] = '\0';
                    past4[3] = '\0';
                }
                past4[3] = past4[2];
                past4[2] = past4[1];
                past4[1] = past4[0];
            } else if (ESP8266whichcommand == 4) {
                past4[0] = RXAdata;
                if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                    ESP8266whichcommand = 5;
                    serial_send(&SerialA,"AT+CIPMUX=1\r\n", strlen("AT+CIPMUX=1\r\n"));
                    past4[0] = '\0';
                    past4[1] = '\0';
                    past4[2] = '\0';
                    past4[3] = '\0';
                }
                past4[3] = past4[2];
                past4[2] = past4[1];
                past4[1] = past4[0];
            } else if (ESP8266whichcommand == 5) {
                past4[0] = RXAdata;
                if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                    ESP8266whichcommand = 6;
                    serial_send(&SerialA,"AT+CIPSERVER=1,1336\r\n", strlen("AT+CIPSERVER=1,1336\r\n")); //1336 is the port to  type into Labview
                    past4[0] = '\0';
                    past4[1] = '\0';
                    past4[2] = '\0';
                    past4[3] = '\0';
                }
                past4[3] = past4[2];
                past4[2] = past4[1];
                past4[1] = past4[0];
            } else if (ESP8266whichcommand == 6) {
                past4[0] = RXAdata;
                if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                    ESP8266whichcommand = 12;  // should never get in 12
                    ESP8266insidecommands = 0;
                    past4[0] = '\0';
                    past4[1] = '\0';
                    past4[2] = '\0';
                    past4[3] = '\0';
                }
                past4[3] = past4[2];
                past4[2] = past4[1];
                past4[1] = past4[0];

            } else if (ESP8266whichcommand == 12) {
                sendback[0] = ' ';
                sendback[1] = 'D';
                sendback[2] = 'a';
                sendback[3] = 'n';
                serial_send(&SerialA,sendback,4);
            }

            sendback[0] = RXAdata;
            serial_send(&SerialA,sendback,1);
        } else {//ESP8266insidecommands == 0
            // for now just echo char
            if (sendingto8266 == 1) {
                if (RXAdata == '>') {
                    serial_send(&SerialA,sendto8266,sendtoSize);
                    sendingto8266 = 0;
                }
            } else if (collect==0){
                if (RXAdata == '*'){ //Reading from teraterm, looking for * symbol to start collecting into gusarray, maybe have  different symbols for different notes?
                    collect = 1;
                    g = 0;
                }
            } else if (collect==1){ //If collect == 1, this is what we want to mainly edit
                if (RXAdata =='a') {
                    sendingto8266 = 1;
                    sendtoSize = sprintf(sendto8266,"%.5f\r\n",Kp_y);
                    if (sendtoSize <= 9) {
                        send8266CommandSize = strlen("AT+CIPSEND=0,0\r\n");  // second zero is place holder for a number 1 to 9
                    } else {
                        if (sendtoSize > 99) {
                            sendtoSize = 99;
                            sendto8266[97] = '\r';
                            sendto8266[98] = '\n';
                        }
                        send8266CommandSize = strlen("AT+CIPSEND=0,00\r\n"); // second zero zero is place holder for a number 10 to 99
                    }
                    sprintf(send8266Command,"AT+CIPSEND=0,%d\r\n",sendtoSize);
                    serial_send(&SerialA,send8266Command,send8266CommandSize); //AT+CIPSEND is send command on chip. 0,3: Not sure what 0 is for (id?) but the 3 corresponds to length being sent.

                    collect = 0;
                    g=0;
                } else if (RXAdata =='\n'){ //End of array, should collect 1 or 2 floats from the * statement and turn it into myfloat variables, depending on labview
                    gusarray[g]='\0';
                    sscanf(gusarray,"%f,%f,%f,%f,%f,%f",&value1,&value2,&value3,&value4,&value5,&value6);
                    //sscanf(gusarray,"%f",&myfloat1);

                    Kp_y = myfloat1;
                    UARTPrint = 1;

                    collect = 0;
                    g=0;
                } else {
                    gusarray[g]=RXAdata;
                    g++;
                    if (g>=150){
                        g = 0;
                    }
                }
            }
//            if (debugi < 100) {
//                debug_array[debugi] = RXAdata;
//                debugi++;
//            }
            sendback[0] = RXAdata;
            //serial_send(&SerialA,sendback,1);
        }
        numRXA++;
        //        RXAdata = RXAdata & 0x00FF;
        //
        //        if (com_state == 0) { //wait for data from pi
        //            if (RXAdata == '*'){
        //                com_state = 1;//pre echo state
        //            } else if (RXAdata == '!'){
        //                com_state = 2;//pre sending state
        //            } else {
        //                com_state = 0;
        //            }
        //        } else if (com_state == 1) {
        //            if (RXAdata == '*') {
        //                com_state = 10;//echo state
        //            } else {
        //                com_state = 0;
        //            }
        //        } else if (com_state == 10) { //echo state
        //            if (received_count % 2 == 0) {
        //                temp_MSB = RXAdata;
        //            } else {
        //                received_pi[received_count/4].parts[(received_count%4)/2] = ((RXAdata << 8) | temp_MSB) & 0xFFFF;
        //            }
        //            received_count += 1;
        //            if (received_count == 16){
        //                com_state = 0;
        //                received_count = 0;
        //                pinkx = received_pi[0].value;
        //                pinky = received_pi[1].value;
        //                bluex = received_pi[2].value;
        //                bluey = received_pi[3].value;
        //            }
        //        } else if (com_state == 2) {
        //            if (RXAdata == '!') {
        //                int i = 0;
        ////                for (i = 0; i < 2; i++) {
        ////                    to_send[i].value = 0.1234+0.1*i;
        ////                }
        //                to_send[0].value = pitch_tiltrate;
        //                to_send[1].value = roll_tiltrate;
        //
        //                for (i = 0; i < 2; i++) {
        //                    send_sci[4*i] = to_send[i].parts[0] & 0xFF;
        //                    send_sci[4*i+1] = (to_send[i].parts[0]>>8) & 0xFF;
        //                    send_sci[4*i+2] = to_send[i].parts[1] & 0xFF;
        //                    send_sci[4*i+3] = (to_send[i].parts[1]>>8) & 0xFF;
        //                }
        //                serial_send(&SerialA, send_sci, 8);
        //                com_state = 0;
        //            } else {
        //                com_state = 0;
        //            }


    }


    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


/***************************************************************************
 * SERIAL_PRINTF()
 *
 * Simple printf command to print out a serial port
 ***************************************************************************/
uint16_t serial_printf(serial_t *s, char *fmt, ...)
{
    va_list ap;
    char buf[BUF_SIZE];

    va_start(ap,fmt);
    vsprintf(buf,fmt,ap);
    va_end(ap);

    return serial_send(s,buf,strlen(buf));
}

