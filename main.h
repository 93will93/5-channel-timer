/*
 * File:   main.h
 * Author: William
 *
 * Created on 28 January, 2015, 8:23 AM
 *
 * IBC-1 V1.00 Code
 *
 */

#ifndef MAIN_H
#define    MAIN_H


// PIC16F1709 Configuration Bit Settings

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = SWON      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
//#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WDTPS = 2048    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up (HFINTOSC output and ready status are delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = INTMCLR  // MCLR Pin Enable bit (MCLR pin disabled, RE3 input pin enabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

//============================================================================//

#define FLAG_TICK       0b0000000000000001
#define _XTAL_FREQ      16000000

//Define ADC values to return

#define ARMED                   1
#define DISARMED                0

#define FIRING                  2
#define TEST                    0

#define ADC_VOLT_EL             0
#define ADC_VOLT_CF             1
#define ADC_VOLT_ZC             2
#define ADC_VOLT_FO             3
#define ADC_VOLT_KS             4

#define VOLT_EL_HIGH            390 //For Rel = 2k
#define VOLT_CF_HIGH            275 //NEED TO CALCULATE
//#define VOLT_ZC_THRESHOLD       10 //Calculate
#define VOLT_ZC_THRESHOLD       1 //Calculate
#define VOLT_MAINS_THRESHOLD    800

#define IBC_SN                  0x01
#define ISC_SN_BROADCAST_ADD    0x00
#define ISC_DEFAULT_SN          0x3FFE
#define ISC_NULL_SN             0x3FFF

#define CLEAR_UPDATE            0x00
#define DEFAULT_UPDATE          0x01
#define ERROR_UPDATE            0x02

#define CMD_SEND_DEFAULT        0b00000000
#define CMD_FORCE_DEFAULT       0b01000000
#define CMD_SEND_DC             0b00000001
#define CMD_FORCE_DC            0b01000001
#define CMD_SEND_AC             0b00000010
#define CMD_FORCE_AC            0b01000010
#define CMD_SEND_BLAST_VALUE    0b00000011
#define CMD_FORCE_BLAST_VALUE   0b01000011
#define CMD_SEND_TEMPERATURE    0b00000100
#define CMD_OPEN_RELAY          0b00000110
#define CMD_CLOSE_RELAY         0b00000101
#define CMD_GET_SN              0b00000111
#define CMD_SN_LIST_CHANGED     0b00001000
#define CMD_BLAST_COMMAND       0b00100101
#define PING_ISC                0b00101001
#define ARM_ISC                 0b00110001
#define DISARM_ISC              0b00110000
#define CMD_CABLE_FAULT         0b00100110
#define CMD_ISC_NEW_SN          0b00001001
#define CMD_NULL                0b11111111

#define CMD_SEND_DEFAULT_B        0b10000000
#define CMD_FORCE_DEFAULT_B       0b11000000
#define CMD_SEND_DC_B             0b10000001
#define CMD_FORCE_DC_B            0b11000001
#define CMD_SEND_AC_B             0b10000010
#define CMD_FORCE_AC_B            0b11000010
#define CMD_SEND_BLAST_VALUE_B    0b10000011
#define CMD_FORCE_BLAST_VALUE_B   0b11000011
#define CMD_SEND_TEMPERATURE_B    0b10000100
#define CMD_OPEN_RELAY_B          0b10000110
#define CMD_CLOSE_RELAY_B         0b10000101
#define CMD_GET_SN_B              0b10000111
#define CMD_SN_LIST_CHANGED_B     0b10001000
#define CMD_BLAST_COMMAND_B       0b10100101
#define PING_ISC_B                0b10101001
#define ARM_ISC_B                 0b10110001
#define DISARM_ISC_B              0b10110000
#define CMD_CABLE_FAULT_B         0b10100110
#define CMD_ISC_NEW_SN_B          0b10001001

#define CMD_PI_SN_ISCS          0b00000001
#define CMD_PI_SN_IB651         0b00000010
#define CMD_PI_DEFAULT_DATA     0b00000011
#define CMD_PI_FORCE_DEFAULT    0b01000011
#define CMD_PI_DC_DATA          0b00000100
#define CMD_PI_FORCE_DC         0b01000100
#define CMD_PI_AC_DATA          0b00000101
#define CMD_PI_FORCE_AC         0b01000101
#define CMD_PI_BLAST_VALUE      0b00000110
#define CMD_PI_FORCE_BLAST_VAL  0b01000110
#define CMD_PI_IBC_DEFAULT      0b00001000
#define CMD_PI_IBC_ERRORS       0b00001100
#define CMD_PI_ISC_PARENT       0b00001111
#define CMD_PI_BLAST_PERMISSION 0b10010000
#define CMD_PI_BLAST_DENIED     0b10010111
#define CMD_PI_BLAST_ACK        0b10100101
#define CMD_PI_PING_ISC         0b00101001
#define CMD_PI_ARM_ISC          0b10110001
#define CMD_PI_DISARM           0b10110000

#define CMD_PI_DEFAULT_DATA_B   0b10000011 //COMPLETE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define CMD_PI_PING_ISC_B       0b10101001


#define FLAG_UART_TX_ACTIVE         1

#define PLM_RECEIVE_BUFFER_LENGTH   71
#define PLM_TRANSMIT_BUFFER_LENGTH  71

#define PRESSED                     1
#define DEPRESSED                   0

#define ON                          1
#define OFF                         0

#define INPUT                       1
#define OUTPUT                      0




// INTERRUPTS
#define GLOBAL_INTERRUPTS                              INTCONbits.GIE

#define EXTERNAL_INTERRUPT_RB0_ENABLE                  INTCONbits.INT0IE
#define EXTERNAL_INTERRUPT_RB1_ENABLE                  INTCON3bits.INT1IE
#define EXTERNAL_INTERRUPT_RB2_ENABLE                  INTCON3bits.INT2IE

#define EXTERNAL_INTERRUPT_RB0_FLAG                    INTCONbits.INT0IF
#define EXTERNAL_INTERRUPT_RB1_FLAG                    INTCON3bits.INT1IF
#define EXTERNAL_INTERRUPT_RB2_FLAG                    INTCON3bits.INT2IF

#define RISING_EDGE_SELECT_PIN_INTERRUPT0              INTCON2bits.INTEDG0
#define RISING_EDGE_SELECT_PIN_INTERRUPT1              INTCON2bits.INTEDG1
#define RISING_EDGE_SELECT_PIN_INTERRUPT2              INTCON2bits.INTEDG2

#define INTERRUPT_PORTB_ONCHANGE_ENABLE                INTCONbits.RBIE
#define INTERRUPT_PORTB_ONCHANGE_FLAG                  INTCONbits.RBIF
#define EXTERNAL_INTERUPT_ONCHANGE_PORTRB7_ENABLE      IOCBbits.IOCB7
#define EXTERNAL_INTERUPT_ONCHANGE_PORTRB4_ENABLE      IOCBbits.IOCB4

// sensors 1-3 are INTn interrupts
#define CHANNEL_SENSOR1_PORT PORTBbits.RB0
#define CHANNEL_SENSOR1_FLAG INTCONbits.INT0IF

#define CHANNEL_SENSOR2_PORT PORTBbits.RB1
#define CHANNEL_SENSOR2_FLAG INTCON3bits.INT1F

#define CHANNEL_SENSOR3_PORT PORTBbits.RB2
#define CHANNEL_SENSOR3_FLAG INTCON3bits.INT2IF

// Sensors 4 and 5 are PORTB on Change interrupts
#define CHANNEL_SENSOR4_PORT PORTBbits.RB5
#define CHANNEL_SENSOR5_PORT PORTBbits.RB4

// =============================================================================

// CHANNEL INDICATORS
#define CHANNEL1_INDICATOR_PORT PORTAbits.RA0
#define CHANNEL1_INDICATOR_TRIS TRISAbits.RA0
#define CHANNEL1_INDICATOR_LAT  LATAbits.LATA0

#define CHANNEL2_INDICATOR_PORT PORTAbits.RA1
#define CHANNEL2_INDICATOR_TRIS TRISAbits.RA1
#define CHANNEL2_INDICATOR_LAT LATAbits.LATA1

#define CHANNEL3_INDICATOR_PORT PORTAbits.RA2
#define CHANNEL3_INDICATOR_TRIS TRISAbits.RA2
#define CHANNEL3_INDICATOR_LAT LATAbits.LATA2

#define CHANNEL4_INDICATOR_PORT PORTAbits.RA3
#define CHANNEL4_INDICATOR_TRIS TRISAbits.RA3
#define CHANNEL4_INDICATOR_LAT LATAbits.LATA3

#define CHANNEL5_INDICATOR_PORT PORTAbits.RA4
#define CHANNEL5_INDICATOR_TRIS TRISAbits.RA4
#define CHANNEL5_INDICATOR_LAT LATAbits.LATA4

//#define CHANNEL6_INDICATOR_PORT PORTAbits.RA7
//#define CHANNEL6_INDICATOR_TRIS PORTAbits.RA7
//#define CHANNEL6_INDICATOR_LAT LATAbits.LATA7
//==============================================================================
// Blasting Fiber Sensors

// Solid State Machine States
enum Flag { init = 0,
            connectedToPC = 1,
            monitoring = 2
          };

enum Channel{ noChannel = 0,
              channel1 = 1,
              channel2 = 2,
              channel3 = 3,
              channel4 = 4,
              channel5 = 5
            };
#endif //MAIN_H
