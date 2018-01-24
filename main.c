/*
 * File:   main.c
 * Author: User
 *
 * Created on 24 January 2018, 10:10 AM
 */


#include <xc.h>
#include "main.h"



enum Channel CHANNEL = noChannel;

void initialize(void);
void startLEDSequence(void);
void setUpInterrupts(void);
void recordBlast(void);
void interrupt isr()
{
   //NOP
    if(CHANNEL_SENSOR1_FLAG == ON){
        CHANNEL = channel1;
    }
    if(CHANNEL_SENSOR2_FLAG == ON) {
        CHANNEL = channel2;
    }
    if(CHANNEL_SENSOR3_FLAG == ON) {
        CHANNEL = channel3;
    }
    if(INTERRUPT_PORTB_ONCHANGE_FLAG == ON && CHANNEL_SENSOR4_PORT == ON) {
        CHANNEL = channel4;
    }
    if(INTERRUPT_PORTB_ONCHANGE_FLAG == ON && CHANNEL_SENSOR5_PORT == ON) {
        CHANNEL = channel5;
    }
    
}



void main(void) {
    enum Flag STATE = init;

    while(1){
        switch(STATE){
            case init:
                initialize();
                break;
            case connectedToPC:
                break;
            case monitoring:
                break;
            default:
                STATE = init;
                break;
        }
    }

    return;
}

void initialize(void){

    setUpInterrupts();
    ANSELA = 0; // SETTING ALL PORT A PINS AS DIGITAL
    ANSELB = 0;

    // SETTING CHANNEL PINS TO OUTPUT
    CHANNEL1_INDICATOR_TRIS = OFF;
    CHANNEL2_INDICATOR_TRIS = OFF;
    CHANNEL3_INDICATOR_TRIS = OFF;
    CHANNEL4_INDICATOR_TRIS = OFF;
    CHANNEL5_INDICATOR_TRIS = OFF;
   // CHANNEL6_INDICATOR_TRIS = OFF;

    CHANNEL1_INDICATOR_PORT = OFF;
    CHANNEL2_INDICATOR_PORT = OFF;
    CHANNEL3_INDICATOR_PORT = OFF;
    CHANNEL4_INDICATOR_PORT = OFF;
    CHANNEL5_INDICATOR_PORT = OFF;
   // CHANNEL6_INDICATOR_PORT = OFF;

    startLEDSequence();
}

void startLEDSequence(void){
    CHANNEL1_INDICATOR_PORT = ON;
    __delay_ms(10);
    CHANNEL1_INDICATOR_PORT = OFF;

    CHANNEL2_INDICATOR_PORT = ON;
    __delay_ms(10);
    CHANNEL2_INDICATOR_PORT = OFF;

    CHANNEL3_INDICATOR_PORT = ON;
    __delay_ms(10);
    CHANNEL3_INDICATOR_PORT = OFF;

    CHANNEL4_INDICATOR_PORT = ON;
    __delay_ms(10);
    CHANNEL4_INDICATOR_PORT = OFF;

    CHANNEL5_INDICATOR_PORT = ON;
    __delay_ms(10);
    CHANNEL5_INDICATOR_PORT = OFF;

//    CHANNEL6_INDICATOR_PORT = ON;
//    __delay_ms(10);
//    CHANNEL6_INDICATOR_PORT = OFF;

}

void setUpInterrupts(void){
    GLOBAL_INTERRUPTS = ON;
    EXTERNAL_INTERRUPT_RB0_ENABLE = ON;
    EXTERNAL_INTERRUPT_RB1_ENABLE = ON;
    EXTERNAL_INTERRUPT_RB2_ENABLE = ON;
    RISING_EDGE_SELECT_PIN_INTERRUPT0 = ON;
    INTERRUPTS_ENABLE = ON;
}

void recordBlast(){
     switch(CHANNEL){
            case channel1:
                //store time
                CHANNEL1_INDICATOR_PORT = ON;
                break;
            case channel2:
                //store time
                CHANNEL2_INDICATOR_PORT = ON;
                break;
            case channel3:
                //store time
                CHANNEL3_INDICATOR_PORT = ON;
                break;
            case channel4:
                //store time
                CHANNEL4_INDICATOR_PORT = ON;
                break;
            case channel5:
                //store time
                CHANNEL5_INDICATOR_PORT = ON;
                break;
            case noChannel:
                //store time
              
                break;
            default:
                CHANNEL = noChannel;
                break;
        }
    
}