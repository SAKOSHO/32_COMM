/* 
 * File:   main.c
 * Author: ²XØãÄ
 *
 * Created on 2021/11/07, 11/10
 * Discription@À±3ÌÊMnÌvOV2
 * ÒWð
 * 2021/11/01FÅ
 * 
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "spi.h"
#include "usart.h"
#include "MCP2515.h"
#include "COMM.h"

// CONFIG1
#pragma config FOSC  = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE  = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP    = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD   = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO  = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP   = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V   // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT   = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ      20000000
#define _CAN_BAUDRATE   2


/*--Prtotype--*/
void CONFIG();                                                                  //úÝè
void timer_init();                                                             //^C}[ÌúÝè
char BtoD(char data);                                                           //2ið10iÉÏ·
void interrupt timer();                                                        //^C}[ÌèÝ

/*--Grobal Variables--*/
volatile char *rx_data;
volatile char *fram_data;
volatile char usart_data;                                                       //usartf[^
volatile char rx_status;
volatile char rx_int;
volatile char data[8] = {2, 2, 2, 2, 2, 2, 2, 2};
volatile char mode = _ChargeMode;
volatile char size;
volatile int cnt = 0;
volatile int cnt1 = 0;
volatile int mode_cnt = 0;


void main(void)
{
    int rate1 = 10;
    int rate2 = 10;                                                              //[g²®
    
    CONFIG();                                                                   //úÝè
    __delay_ms(100);
    
    T1CONbits.TMR1ON = 1;                                                       //^C}[Jn

    while(1)
    {
        /* óM */
        rx_int = Read(_CANINTF);                                                //èÝtO
        
        if((rx_int & _Flagbit0) == 0b00000001)
        {   
            rx_data = Read_RX_ID(_F_RXB0SIDH, 13);
            Write(_CANINTF, 0b00000000);
            
            if(rx_data[3] == SIDH_MODE)                                         //[hîñ©ðmF
            {
                if((rx_data[5] & _ChargeMode) == 0b00000001)                    //[d[hÉÚs
                {
                    /*
                    RB5 = 0;
                    RB6 = 0;
                    RB7 = 1;
                    */
                    mode = _ChargeMode;
                }
            
                if((rx_data[5] & _COMMMode) == 0b00000010)                      //_EN[hÉÚs
                {
                    /*
                    RB5 = 0;
                    RB6 = 1;
                    RB7 = 0;
                    */
                    mode = _COMMMode;
                }
            
                if((rx_data[5] & _StanbyMode) == 0b00000011)                    //Ò@[hÉÚs
                {
                    /*
                    RB5 = 1;
                    RB6 = 0; 
                    RB7 = 0;
                    */
                    mode = _StanbyMode;
                }
            
                if((rx_data[5] & _MissionMode) == 0b00000100)                   //~bV[hÉÚs
                {
                    /*
                    RB5 = 0;
                    RB6 = 0; 
                    RB7 = 0;
                    */
                    mode = _MissionMode;
                }
            }
        }
        
        /* d¹ü©ç20bÜÅÍIðµ½[h */
        if(mode_cnt <= 2000)
        {
            /* 1Hz */
            if(cnt >= rate1)
            {
                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_R, 0, 0);                   //[hv
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv
                
                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);   //[h«ÝIDÌÝè
                Load_TX_Data(_F_TXB0D0, 1, &mode);                                  //[hñ
                RTS0(_CAN_BAUDRATE);                                                //Mv
                
                cnt = 0;
            }
        }
        
        /* d¹ü©ç20bãÉÊM[hÉÚs */
        /* _EN */
        if(mode_cnt > 2000 && mode_cnt <= 2500)
        {
            /* 100Hz */
            if(cnt >= rate2)
            {
                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00000111); //~bVf[^ÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00001010); //Á¬xxÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00001011); //Á¬xyÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00001100); //Á¬xzÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00001101); //p¬xxÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00001110); //p¬xyÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00001111); //p¬xzÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00010000); //§äÊf[^ÌÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00010001); //obe[Ìd³lÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00010010); //obe[Ìd¬lÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00010011); //Ìd¬lÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00010100); //p¨èZTÌd¬lÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00010101); //p¨§äAN`G[^Ìd¬lÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00010110); //ÊM@íÌd¬lÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00010111); //~bV@íÌd¬lÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00011000); //obe[Ì·xÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00011001); //Ì·xÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00011010); //p¨ZTÌ·xÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00011011); //p¨§äAN`G[^Ì·xÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00011100); //ÊM@Ì·xÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                Write(_TXB0DLC , 0b00000001);                                       //bZ[WTCY1byte
                Load_TX_ID(_F_TXB0SIDH, SIDH_READ, SIDL_R, 0b00000000, 0b00011101); //~bV@íÌ·xÇÝoµ
                Load_TX_Data(_F_TXB0D0, 1, 0);                                      //ÇÝoµ¾©çCóf[^
                RTS0(_CAN_BAUDRATE);                                                //Mv

                cnt = 0;
            }
        }
            
        /* AbvNR}hðGS©çusartÊMÅó¯æé */
        if(mode_cnt > 2500 && mode_cnt <= 3000)
        {
            /* óMf[^ú»*/
            RCIF = 0;
            RCREG = 0;
            
            usart_puts("C");                                                    //GSÉ_EN®¹Êm

            while(!RCIF)
            {
            }
            RCIF = 0;
            
            usart_data = RCREG;

            /* R}h²ÆÌCSSÉM·ébZ[W */
            switch(usart_data)
            {
                case '3':
                    mode = _StanbyMode;

                    Write(_TXB0DLC, 0b00000001);                                            //bZ[WTCY1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, 0, 0);           //[h«ÝIDÌÝè
                    Load_TX_Data(_F_TXB0D0, 1, &mode);                                      //[h«Ý
                    RTS0(_CAN_BAUDRATE);                                                    //Mv
                    break;
                    
                case '4':
                    mode = _MissionMode;

                    Write(_TXB0DLC, 0b00000001);                                            //bZ[WTCY1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, 0, 0);       //[h«ÝIDÌÝè
                    Load_TX_Data(_F_TXB0D0, 1, &mode);                                      //[h«Ý
                    RTS0(_CAN_BAUDRATE);                                                    //Mv
                    break;
                default:
                    mode = mode;
                    
                    Write(_TXB0DLC, 0b00000001);                                            //bZ[WTCY1byte
                    Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, 0, 0);       //[h«ÝIDÌÝè
                    Load_TX_Data(_F_TXB0D0, 1, &mode);                                      //[h«Ý
                    RTS0(_CAN_BAUDRATE);                                                    //Mv
                    break;
            }
            
            RCREG = 0;
            
            while(1)
            {
                if(mode_cnt > 3000) break;
            }
        }
            
        if(mode_cnt > 3000) mode_cnt = 0;
    }
}

void CONFIG()
{
    OSCCON = 0b01101000;
    ANSEL  = 0b00000000;
    ANSELH = 0b00000000;
    TRISB  = 0b00000000;
    TRISC  = 0b00000000;
    PORTB  = 0b00000000;
    PORTC  = 0b00000000;
    
    INTCON = 0b11000000;                                                        //èÝÝè
    
    usart_init();
    spi_init();
    timer_init();
    __delay_ms(100);
    
    MCP2515_init(_CAN_BAUDRATE);                                                //Æè ¦¸C®ìµÄ¢é2Éµ½DðÍÜ¾
    Write(_TXB0DLC , 0b00001000);                                               //bZ[WTCY8byte
    Write(_RXM0SIDH, 0b11111111);                                               //}XNÝèãÊSêv
    Write(_RXM0SIDL, 0b11111111);                                               //}XNÝèºÊSêv
    Write(_RXM0EID8, 0b11111111);                                               //}XNÝèg£ãÊSêv
    Write(_RXM0EID0, 0b11110000);                                               //}XNÝèg£ºÊãÊ4bitêv
    Write(_RXF0SIDH, 0b00000000);                                               //óMtB^ãÊrbgÌÝè
    Write(_RXF0SIDL, 0b00001000);                                               //óMtB^ºÊrbgÌÝè
    Write(_RXF0EID8, 0b00000000);                                               //óMtB^g£ãÊrbgÌÝè
    Write(_RXF0EID0, Sub_Filt);                                                 //óMtB^g£ºÊrbgÌÝè
    MCP2515_Open(0);                                                            //Æè ¦¸C0Éµ½DðÍÜ¾
}

void timer_init()
{
    T1CON = 0b00000000;
    TMR1H = 0b00111100;                                                         //500000ÅI[o[t[³¹é
    TMR1L = 0b10110000;
    PIE1bits.TMR1IE = 1;
    PIR1bits.TMR1IF = 0;
}



char BtoD(char data)
{
    char  binary;
    char decimal = 0;
    char bas = 1;
            
    binary = data & 0b00001111;
    
    while(binary>0)
    {
        decimal = decimal + (binary % 10) * bas;
        binary = binary / 10;
        bas = bas * 2;
    }
    
    return decimal;
}

void interrupt timer()
{
    PIR1bits.TMR1IF = 0;
    TMR1H = 0b00111100;                                                         //500000ÅI[o[t[
    TMR1L = 0b10110000;
    
    cnt++;
    cnt1++;
    mode_cnt++;
}