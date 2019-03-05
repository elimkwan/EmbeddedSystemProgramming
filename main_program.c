/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

// Author:      Mohd A. Zainol
// Date:        1 Oct 2018
// Chip:        MSP432P401R LaunchPad Development Kit (MSP-EXP432P401R) for TI-RSLK
// File:        main_program.c
// Function:    The main function of our code in FreeRTOS

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* TI includes */
#include "gpio.h"

/* ARM Cortex */
#include <stdint.h>
#include "msp.h"
#include "SysTick.h"
#include "inc/CortexM.h"

#include "inc/songFile.h"
#include "inc/dcMotor.h"
#include "inc/bumpSwitch.h"
#include "inc/outputLED.h"
#include "inc/Systick.h"

//bit-banded addresses positive logic of input switch S1 and S2
#define SW1IN ((*((volatile uint8_t *)(0x42098004)))^1)
#define SW2IN ((*((volatile uint8_t *)(0x42098010)))^1)

//Global Variable
/*
 * VARIABLE TO CHANGE HERE:
 * SET to 0 IF INTERRUPT
 * SET to 1 IF POLLING
 */
uint8_t pol = 1; //0 is interrupt; 1 is polling
/*
 *
 */

uint8_t bumpSwitch_status; // global variable showing the bump swtich statuses
uint8_t mode = 0;

//Interrupt specific global Variable
uint8_t stop = 0;
SemaphoreHandle_t xBSemaphore;

//Polling specific global variable
uint8_t flag=0;

void main_program( void );
static void Switch_Init(void);
static void taskMasterThread( void *pvParameters );
static void taskPlaySong(void *pvParameters);
static void prvConfigureClocks( void );

//interrupt specific
static void outputLED_response_ISR(unsigned char bumpSwitch_status);
static void Port4_Init(void);
static void taskMain(void *pvParameters);
static void taskInterrupt(void *pvParameters);
static void BumpEdgeTrigger_Init(void);
static void taskReadInputSwitchInterrupt(void *pvParameters);

//polling specific
static void taskReadInputSwitch(void *pvParameters);
static void taskDisplayOutputLED(void *pvParameters);
static void taskBumpSwitch(void *pvParameters);
static void taskMainPolling(void *pvParameters);

//Shared
xTaskHandle taskHandle_BlinkRedLED;
xTaskHandle taskHandle_PlaySong;

//interrupt specific
xTaskHandle taskHandle_Main;
xTaskHandle taskHandle_Interrupt;
xTaskHandle taskHandle_InputSwitchInterrupt;

//polling specific
xTaskHandle taskHandle_BumpSwitch;
xTaskHandle taskHandle_MainPolling;
xTaskHandle taskHandle_InputSwitch;
xTaskHandle taskHandle_DisplayOutputLED;

void main_program( void )
{
    if (pol == 0){//INTERRUPT
            // initialise the clock configuration
            prvConfigureClocks();

            //initialise the switch and bump switches + interrupt;
            Switch_Init();
            Port4_Init(); //interrupt specific
            BumpEdgeTrigger_Init();//interrupt specific

            //initialise systick timer
            SysTick_Init();

            //crate a semaphore
            xBSemaphore = xSemaphoreCreateBinary(); //interrupt specific

            //Master Task will run first with priority 2
            xTaskCreate(taskMasterThread, "taskT", 128, NULL, 2, &taskHandle_BlinkRedLED);
            //Below tasks will all have the same priority therefore will run concurrently[Round-Robin algorithm]
            xTaskCreate(taskMain, "taskM", 128, NULL, 1, &taskHandle_Main);
            xTaskCreate(taskPlaySong, "taskS", 128, NULL, 1, &taskHandle_PlaySong);
            xTaskCreate(taskInterrupt, "taskInterrupt", 128, NULL, 1, &taskHandle_Interrupt);
            xTaskCreate(taskReadInputSwitchInterrupt, "taskQ", 128, NULL, 1, &taskHandle_InputSwitchInterrupt);

            vTaskStartScheduler();

            /* INFO: If everything is fine, the scheduler will now be running,
            and the following line will never be reached.  If the following line
            does execute, then there was insufficient FreeRTOS heap memory
            available for the idle and/or timer tasks to be created. See the
            memory management section on the FreeRTOS web site for more details. */
            for( ;; );
    }else if (pol == 1){ //POLLING
            // initialise the clock configuration
            prvConfigureClocks();
            Switch_Init();
            SysTick_Init();

            //Master Task will run first with priority 2
            xTaskCreate(taskMasterThread, "taskT", 128, NULL, 2, &taskHandle_BlinkRedLED);
            //Below tasks will all have the same priority therefore will run concurrently [Round-Robin algorithm]
            xTaskCreate(taskBumpSwitch, "taskB", 128, NULL, 1, &taskHandle_BumpSwitch);
            xTaskCreate(taskPlaySong, "taskS", 128, NULL, 1, &taskHandle_PlaySong);
            xTaskCreate(taskReadInputSwitch, "taskR", 128, NULL, 1, &taskHandle_InputSwitch);
            xTaskCreate(taskMainPolling, "taskM", 128, NULL, 1, &taskHandle_MainPolling);
            xTaskCreate(taskDisplayOutputLED, "taskD", 128, NULL, 1, &taskHandle_DisplayOutputLED);

            vTaskStartScheduler();

            /* INFO: If everything is fine, the scheduler will now be running,
            and the following line will never be reached.  If the following line
            does execute, then there was insufficient FreeRTOS heap memory
            available for the idle and/or timer tasks to be created. See the
            memory management section on the FreeRTOS web site for more details. */
            for( ;; );
    }

}
/*-----------------------------------------------------------------*/
/*------------------- FreeRTOS configuration ----------------------*/
/*-------------   DO NOT MODIFY ANYTHING BELOW HERE   -------------*/
/*-----------------------------------------------------------------*/
// The configuration clock to be used for the board
static void prvConfigureClocks( void )
{
    // Set Flash wait state for high clock frequency
    FlashCtl_setWaitState( FLASH_BANK0, 2 );
    FlashCtl_setWaitState( FLASH_BANK1, 2 );

    // This clock configuration uses maximum frequency.
    // Maximum frequency also needs more voltage.

    // From the datasheet: For AM_LDO_VCORE1 and AM_DCDC_VCORE1 modes,
    // the maximum CPU operating frequency is 48 MHz
    // and maximum input clock frequency for peripherals is 24 MHz.
    PCM_setCoreVoltageLevel( PCM_VCORE1 );
    CS_setDCOCenteredFrequency( CS_DCO_FREQUENCY_48 );
    CS_initClockSignal( CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
}

// The sleep processing for MSP432 board
void vPreSleepProcessing( uint32_t ulExpectedIdleTime ){}

#if( configCREATE_SIMPLE_TICKLESS_DEMO == 1 )
    void vApplicationTickHook( void )
    {
        /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */
        /* Only the full demo uses the tick hook so there is no code is
        executed here. */
    }
#endif
/*-----------------------------------------------------------------*/
/*-------------   DO NOT MODIFY ANYTHING ABOVE HERE   -------------*/
/*--------------------------- END ---------------------------------*/
/*-----------------------------------------------------------------*/

//-------------------------- Shared functions -----------------------
static void Switch_Init(void){
    // negative logic built-in Button 1 connected to P1.1
    // negative logic built-in Button 2 connected to P1.4
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;      // configure P1.4 and P1.1 as GPIO
    P1->DIR &= ~0x12;       // make P1.4 and P1.1 in
    P1->REN |= 0x12;        // enable pull resistors on P1.4 and P1.1
    P1->OUT |= 0x12;        // P1.4 and P1.1 are pull-up
}

static void taskPlaySong(void *pvParameters){
    //Plays the song
    init_song_pwm();
    for( ;; ){
        play_song();
    }
}

static void taskMasterThread( void *pvParameters )
{
        // This task got priority 2
        int i;
        int pressed = 0;
        ColorLED_Init();

        RedLED_Init();
        REDLED = 1; //Switch on the LED

        pressed = SW1IN | SW2IN;
        while(!pressed){                  // Wait for SW2 switch
            for (i=0; i<1000000; i++);  // Wait here waiting for command

            if (pol == 0){
                REDLED = 1; // Interrupt means solid RED
            }else{
                REDLED = !REDLED; // Polling means blinking LED
            }
            pressed = SW1IN | SW2IN;
            if (SW1IN){
                mode = 1;
            }
            else if (SW2IN){
                mode = 2;
            }
        }

        REDLED = !REDLED; //i.e. REDLED = 0;
        vTaskDelete(NULL); // delete its task
}
//-------------------------- END Shared functions -----------------------

//-------------------------- Interrupt functions -----------------------
void BumpEdgeTrigger_Init(void){
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;      // configure as GPIO
    P4->DIR &= ~0xED;       // make in
    P4->REN |= 0xED;        // enable pull resistors
    P4->OUT |= 0xED;        // pull-up
    P4->IES |= 0xED;        // falling edge event
    P4->IFG &= ~0xED;       // clear flag
    P4->IE |= 0xED;         // arm the interrupt
    // priority 2 on port4
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00D00000; //Numerical Priority 6
    // enable interrupt 38 in NVIC on port4
    NVIC->ISER[1] = 0x00000040;
}

void Port4_Init(void){
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;      // configure as GPIO
    P4->DIR &= ~0xED;       // set as input
    P4->REN |= 0xED;        // enable pull resistors
    P4->OUT |= 0xED;        // set xx are pull-up
    //see if it works?
    P4->IES |= 0xED;      // falling edge event
}

void PORT4_IRQHandler(void){
    // Interrupt Vector of Port4
    // Shared Variable
    bumpSwitch_status = P4->IV;      // 2*(n+1) where n is highest priority

    BaseType_t xHigher;
    xHigher = pdFALSE;

    P4->IFG &= ~0xED; // clear flag

    //the interrupt handler will give a semaphore
    xSemaphoreGiveFromISR(xBSemaphore, &xHigher);
    /*
     * More info: The give semaphore function will enable a task to run for once
     * This makes sure the Interrupt Service Routine is of a minimal time
     *
     * The task thats receives the semaphore is taskInterrupt();
     */
}

void outputLED_response_ISR(unsigned char bumpSwitch_status){
    //Since the value received form Interrupt is different from polling,
    //another function have to be made
    int i;
    switch(bumpSwitch_status){
      case 0x02: // Bump switch 1
          Port2_Output2(SKYBLUE);
        break;
      case 0x06:// Bump switch 2
          Port2_Output2(RED);
        break;
      case 0x08: // Bump switch 3
          Port2_Output2(PINK);
        break;
      case 0x0C: // Bump switch 4
          Port2_Output2(YELLOW);
        break;
      case 0x0E: // Bump switch 5
          Port2_Output2(GREEN);
        break;
      case 0x10: // Bump switch 6
          Port2_Output2(BLUE);
        break;
      case 0xED: // neither switch pressed
          Port2_Output2(COLOUROFF);
        break;
      default:
          Port2_Output2(WHITE);
    }
    for (i=0; i<100000; i++);
    Port2_Output2(COLOUROFF);
}

static void taskInterrupt(void *pvParamters){
    //Initialise the interrupt
    EnableInterrupts();       // Clear the I bit

    for ( ;; ){
        /*
         * More info: In here, the task will be waiting at
         * xSemaphoreTake(). It gets unblocked when the
         * semaphore is given from the PORT4_IRQHandler()
         * defined above
         */
        xSemaphoreTake(xBSemaphore, portMAX_DELAY);
        dcMotor_Stop(1);
        vTaskSuspend(taskHandle_Main);
        vTaskSuspend(taskHandle_PlaySong);
        //Make sure the motor is stopped

        //change colours - value stored in the global variable
        outputLED_response_ISR(bumpSwitch_status);

        //look at which mode we are on
        if (mode == 1){
            dcMotor_Stop(1);
            stop = 1;
        }
        else if (mode == 2){
            dcMotor_response_interrupt(bumpSwitch_status);
            vTaskResume(taskHandle_PlaySong);
        }

        vTaskResume(taskHandle_Main);
    }
}

static void taskMain(void *pvParameters){
    dcMotor_Init();
    int j = 0;
    for ( ;; ){
        if (mode == 1){
            // predefined route, stop when pressed
            if (!stop){
                j = 0;
                while (stop == 0 && j < 3){
                    dcMotor_Forward(500, 100);
                    j++;
                }
                j = 0;
                while (stop == 0 && j < 1){
                    dcMotor_Left(500, 100);
                    j++;
                }
            }
            dcMotor_Stop(1);
        }
        else if (mode == 2){
            //do something else
            dcMotor_Forward(500, 1);
        }

    }
}

static void taskReadInputSwitchInterrupt(void *pvParamters){
    for ( ;; ){
        if (SW1IN){
                mode = 1;
                stop = 0;
                vTaskResume(taskHandle_PlaySong);
            }
            else if (SW2IN){
                mode = 2;
            }
    }
}
//-------------------------- END Interrupt functions --------------------------



//-------------------------- Polling functions -----------------------
static void taskReadInputSwitch( void *pvParameters ){
    // This function act as a switch press once to stop playing,
    // press second time to resume
    char i_SW1=0;
    int i;

    for( ;; )
    {
        if (SW1IN == 1) { // 1 means being pressed
            mode=1;
            flag=0;
            i_SW1 ^= 1;                 // toggle the variable i_SW1
            for (i=0; i<1000000; i++);  // this waiting loop is used
            // to prevent the switch bounce.
            vTaskResume(taskHandle_PlaySong);
        }
        else if (SW2IN == 1){
            mode=2;
            flag=0;
            for (i=0; i<1000000; i++);  // this waiting loop is used
            // to prevent the switch bounce.
            vTaskResume(taskHandle_PlaySong);
        }

    }
}

static void taskBumpSwitch(void *pvParameters){
    BumpSwitch_Init();
    for( ;; ){
        bumpSwitch_status = Bump_Read_Input();
    }
}

static void taskDisplayOutputLED(void *pvParameters){
    for( ;; ){
        outputLED_response(bumpSwitch_status);
    }
}

static void taskMainPolling(void *pvParameters){

    dcMotor_Init();
    for ( ;; ){
        if (mode==1){
            flag=dcMotor_stop_flag(bumpSwitch_status);

            while (flag==1){
                 dcMotor_Stop(1);
                 vTaskSuspend(taskHandle_PlaySong);
            }
        }
        else if (mode==2){
            flag=0;
            dcMotor_response(bumpSwitch_status);
            vTaskResume(taskHandle_PlaySong);
       }
    }
}
//-------------------------- Polling functions -----------------------

