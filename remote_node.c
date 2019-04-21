/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//I have refferred the TI resource explorer functions and https://www.digikey.com/eewiki/display/microcontroller/I2C+Communication+with+the+TI+Tiva+TM4C123GXL for I2C communication

/*
 *  ======== empty.c ========
 */
/* Board Header file */
#include "Board.h"


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <types.h>
#include <time.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdbool.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include <ti/sysbios/knl/semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include "FreeRTOS.h"
#include "queue.h"


/*****************************
* Global variables
* shared mem
*****************************
*****************************/
#define TASKSTACKSIZE   1024
#define STR_SIZE 200
#define LOGGER_PERIOD 20
#define THRESHOLD_PERIOD 1000
#define LED_PERIOD  1000
#define SENSOR_PERIOD 1000
#define GAS_PERIOD 1000
#define HUMIDITY_PERIOD 1000
#define TEMP_SLAVE_ADDR (0x48)
#define TEMP_REG_ADDR   (0x00)
#define UART_PERIOD 1
#define QUEUE_SIZE 100

//semaphore
Semaphore_Struct sem_log_struct,sem_read_struct;
Semaphore_Handle sem_log,sem_read;
Semaphore_Params sem_log_params,sem_read_params;
uint8_t log_type;
Task_Struct uart_struct;
Task_Struct gas_struct;
Task_Struct sensor_struct;
Task_Struct logger_struct;
Task_Struct threshold_struct;
Char logger_stack[TASKSTACKSIZE];
Char sensor_stack[TASKSTACKSIZE];
Char uart_stack[TASKSTACKSIZE];
Char gas_stack[TASKSTACKSIZE];
Char threshold_stack[TASKSTACKSIZE];
uint8_t* logfile;
int8_t flag;
uint8_t* msg;
int32_t current_temperature,current_humidity,current_gas,condition;
Bool fans[5]={0,0,0,0,0};
int32_t temperature_threshold[5]={20,25,30,35,50};
int32_t humidity_threshold[5]={20,40,60,80,90};
int32_t gas_threshold[5]={20,40,60,80,90};
UART_Handle uart;
UART_Params uartParams;
QueueHandle_t log_queue;

uint8_t* error_msg[]={"The Log Queue is full, Data Lost","Log type not found"};

typedef enum
{
    QUEUE_FULL=0,
    ERROR_LOGTYPE=1,
}error_t;

typedef enum
{
    LOG_LED=0,
    LOG_TEMPERATURE=1,
    LOG_HUMIDITY=2,
    LOG_GAS=3,
    LOG_THRESHOLD=4,
    LOG_FAN=5,
    LOG_ERROR=6,
}logtype_t;

typedef struct
{
    logtype_t log_id;
    int32_t data;
    int32_t time_now;
}queue_data_t;

typedef enum
{
    LOG_DATA='A',
    GET_TEMPERATURE,
    GET_HUMIDITY,
    GET_GAS,
    GET_THRESHOLD,
    GET_FAN,
    CHANGE_MODE,
    CHANGE_TEMPERATURE_THRESHOLD,
    CHANGE_HUMIDITY_THRESHOLD,
    CHANGE_GAS_THRESHOLD,
    BUZZER_ON,
    BUZZER_OFF,
    FORCE_CHANGE_FANS,
}uart_command_t;

typedef struct
{
    uart_command_t command_id;
    int32_t data;
}uart_data_t;

void exit_handler(void)
{
    condition=0;
}

void uart_init(void)
{
   /* Create a UART with data processing off. */
   UART_Params_init(&uartParams);
   uartParams.writeDataMode = UART_DATA_BINARY;
   uartParams.readDataMode = UART_DATA_BINARY;
   uartParams.readReturnMode = UART_RETURN_FULL;
   uartParams.readEcho = UART_ECHO_OFF;
   uartParams.baudRate = 9600;
   uart = UART_open(Board_UART0, &uartParams);
   if (uart == NULL) {
       System_abort("Error opening the UART");
   }
}

void queue_adder(queue_data_t* data_send)
{
    if(Semaphore_getCount(sem_read) < QUEUE_SIZE-1)
    {
        GPIO_write(Board_LED0,0);
        xQueueSend(log_queue,( void * )data_send,( TickType_t ) 10 );
        Semaphore_post(sem_read);
    }
    else if(Semaphore_getCount(sem_read) == QUEUE_SIZE-1)
    {
        data_send->data=0;
        data_send->log_id=LOG_ERROR;
        GPIO_write(Board_LED0,1);
        xQueueSend(log_queue,( void * )data_send,( TickType_t ) 10 );
        Semaphore_post(sem_read);
    }
}

void Fan_update(int8_t value)
{
    queue_data_t data_send;
    uint8_t i=0;
    uint32_t time_now =  Clock_getTicks();
    //
    for(i=0;i<5;i++)
    {
        fans[i]=i<=value?1:0;
    }
    data_send.time_now =  Clock_getTicks();
    data_send.data=value;
    data_send.log_id=LOG_FAN;
    queue_adder(&data_send);
}

void uartFxn(UArg arg0, UArg arg1)
{
    uint8_t command;
    while(condition)
    {
        UART_read(uart, &command, 1);
        switch(command)
        {
            case LOG_DATA:
            {
                Semaphore_post(sem_log);
                break;
            }

            case GET_TEMPERATURE:
            {

                break;
            }

            case GET_HUMIDITY:
            {

                break;
            }

            case GET_GAS:
            {

                break;
            }

            case GET_THRESHOLD:
            {

                break;
            }

            case GET_FAN:
            {

                break;
            }

            case CHANGE_MODE:
            {

                break;
            }

            case CHANGE_TEMPERATURE_THRESHOLD:
            {

                break;
            }

            case CHANGE_HUMIDITY_THRESHOLD:
            {

                break;
            }

            case CHANGE_GAS_THRESHOLD:
            {

                break;
            }

            case BUZZER_ON:
            {

                break;
            }

            case BUZZER_OFF:
            {

                break;
            }

            case FORCE_CHANGE_FANS:
            {

                break;
            }

            default:
            {
                break;
            }
        }
    }
}

void loggerFxn(UArg arg0, UArg arg1)
{
    queue_data_t received_data;
    while (condition)
    {
         Task_sleep((unsigned int)arg0);
         //message queue receive
         Semaphore_pend(sem_log, BIOS_WAIT_FOREVER);
         Semaphore_pend(sem_read, BIOS_WAIT_FOREVER);
         xQueueReceive(log_queue, &(received_data), ( TickType_t ) 10 );
         switch(received_data.log_id)
         {
            case LOG_LED:
            {
                sprintf(msg,"time: %d sec %d msec\tLOG_LED\tled toggle count = %d\n\r",received_data.time_now/1000,received_data.time_now%1000,received_data.data);
                break;
            }

            case LOG_TEMPERATURE:
            {
                sprintf(msg,"time: %d sec %d msec\tLOG_TEMPERATURE\tTemperature: %dC, %dF, %dK\n\r",received_data.time_now/1000,received_data.time_now%1000,received_data.data,((received_data.data*9)/5)+32,received_data.data+273);
                break;
            }

            case LOG_HUMIDITY:
            {
                sprintf(msg,"time: %d sec %d msec\tLOG_HUMIDITY\tHumidity: %d%%\n\r",received_data.time_now/1000,received_data.time_now%1000,received_data.data);
                break;
            }

            case LOG_GAS:
            {
                sprintf(msg,"time: %d sec %d msec\tLOG_GAS\t\tGas Quality: %d%%\n\r",received_data.time_now/1000,received_data.time_now%1000,received_data.data);
                break;
            }

            case LOG_FAN:
            {
                sprintf(msg,"time: %d sec %d msec\tLOG_FAN\tFans ON: %d\n\r",received_data.time_now/1000,received_data.time_now%1000,received_data.data);
                break;
            }

            case LOG_THRESHOLD:
            {
                sprintf(msg,"time: %d sec %d msec\tLOG_THRESHOLD\tTHRESHOLD CHECKED\n\r",received_data.time_now/1000,received_data.time_now%1000);
                break;
            }

            case LOG_ERROR:
            {
                sprintf(msg,"time: %d sec %d msec\tLOG_ERROR\t%s\n\r",received_data.time_now/1000,received_data.time_now%1000,error_msg[received_data.data]);
                break;
            }

            default:
            {
                sprintf(msg,"time: %d sec %d msec\tLOG_ERROR\t%s data:%d\n\r",received_data.time_now/1000,received_data.time_now%1000,error_msg[ERROR_LOGTYPE],received_data.data);
                break;
            }
         }
         if(Semaphore_getCount(sem_read))
         {
             Semaphore_post(sem_log);
         }
         UART_write(uart,msg,strlen(msg));
         //Semaphore_post(sem_log);
    }
    free(msg);
}

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    //Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    //Enable and initialize the I2C0 master module.  Use the system clock for
    //the I2C0 module.  The last parameter sets the I2C data transfer rate.
    //If false the data rate is set to 100kbps and if true the data rate will
    //be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void ledtoggleFxn(UArg arg0, UArg arg1)
{
    static int32_t count=0;
    queue_data_t data_send;
    uint32_t time_now =  Clock_getTicks();
    while (condition)
    {
        Task_sleep((unsigned int)arg0);
        GPIO_toggle(Board_LED1);
        GPIO_toggle(Board_LED0);
        data_send.time_now =  Clock_getTicks();
        data_send.data=++count;
        data_send.log_id=LOG_LED;
        queue_adder(&data_send);
    }
}

void gasFxn(UArg arg0, UArg arg1)
{
    queue_data_t data_send;
    while (condition)
    {
       Task_sleep((unsigned int)arg0);
       current_gas=rand()%100;
       data_send.data = current_gas;
       data_send.time_now =  Clock_getTicks();
       data_send.log_id=LOG_GAS;
       queue_adder(&data_send);
    }
}

void thresholdFxn(UArg arg0, UArg arg1)
{
    uint8_t i=0;
    static uint8_t fans_on=0;
    queue_data_t data_send;
    while (condition)
    {
        Task_sleep((unsigned int)arg0);
        //
        for(i=0;i<5;i++)
        {
            if((current_temperature<temperature_threshold[i])&&(current_humidity<humidity_threshold[i])&&(current_gas<gas_threshold[i]))
            {
                break;
            }
        }
        if(i!=fans_on)
        {
            fans_on=i;
            Fan_update(fans_on);
        }
        data_send.time_now =  Clock_getTicks();
        data_send.data=rand()%5;
        data_send.log_id=LOG_THRESHOLD;
        queue_adder(&data_send);
    }
}

void sensorFxn(UArg arg0, UArg arg1)
{
    queue_data_t data_send;
    int8_t rx_MSB=0,rx_LSB=0;
    while (condition)
    {
        Task_sleep((unsigned int)arg0);
        //temperature start
        I2CMasterSlaveAddrSet(I2C0_BASE, TEMP_SLAVE_ADDR, false);

        //specify register to be read
        I2CMasterDataPut(I2C0_BASE, TEMP_REG_ADDR);

        //send control byte and register address byte to slave device
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(I2C0_BASE));

        //specify that we are going to read from slave device
        I2CMasterSlaveAddrSet(I2C0_BASE, TEMP_SLAVE_ADDR, true);

        //send control byte and read from the register we
        //specified
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(I2C0_BASE));

        //return data pulled from the specified register
        rx_MSB=I2CMasterDataGet(I2C0_BASE);

        //send control byte and read from the register we
       //specified
       I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
       //wait for MCU to finish transaction
       while(I2CMasterBusy(I2C0_BASE));
       rx_LSB=I2CMasterDataGet(I2C0_BASE);
       current_temperature=(((rx_MSB << 8) | rx_LSB) >> 4)/16.0;
       data_send.data = current_temperature;
       data_send.time_now =  Clock_getTicks();
       data_send.log_id=LOG_TEMPERATURE;
       flag=LOG_TEMPERATURE;
       queue_adder(&data_send);
       //temperature_end
       //humidity start
       current_humidity=rand()%100;
       data_send.data = current_humidity;
       data_send.time_now =  Clock_getTicks();
       data_send.log_id=LOG_HUMIDITY;
       queue_adder(&data_send);
       //humidity end
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params uart_task,logger_task,sensor_task,gas_task,threshold_task;
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initUART();
    InitI2C0();A
    condition = 1;
    log_queue = xQueueCreate(QUEUE_SIZE, sizeof(queue_data_t));
    msg = (uint8_t*)malloc(STR_SIZE);

    //semaphore
    Semaphore_Params_init(&sem_log_params);
    Semaphore_construct(&sem_log_struct, 0, &sem_log_params);
    sem_log = Semaphore_handle(&sem_log_struct);
    Semaphore_Params_init(&sem_read_params);
    Semaphore_construct(&sem_read_struct, 0, &sem_read_params);
    sem_read = Semaphore_handle(&sem_read_struct);

    /* Construct logger Task  thread */
    Task_Params_init(&logger_task);
    logger_task.arg0 = LOGGER_PERIOD;
    logger_task.stackSize = TASKSTACKSIZE;
    logger_task.stack = &logger_stack;
    Task_construct(&logger_struct, (Task_FuncPtr)loggerFxn, &logger_task, NULL);

    /* Construct uart Task  thread */
    Task_Params_init(&uart_task);
    uart_task.arg0 = UART_PERIOD;
    uart_task.stackSize = TASKSTACKSIZE;
    uart_task.stack = &uart_stack;
    Task_construct(&uart_struct, (Task_FuncPtr)uartFxn, &uart_task, NULL);

    /* Construct threshold  thread */
    Task_Params_init(&threshold_task);
    threshold_task.arg0 = THRESHOLD_PERIOD;
    threshold_task.stackSize = TASKSTACKSIZE;
    threshold_task.stack = &threshold_stack;
    Task_construct(&threshold_struct, (Task_FuncPtr)thresholdFxn, &threshold_task, NULL);

    /* Construct gas Task  thread */
    Task_Params_init(&gas_task);
    gas_task.arg0 = GAS_PERIOD;
    gas_task.stackSize = TASKSTACKSIZE;
    gas_task.stack = &gas_stack;
    Task_construct(&gas_struct, (Task_FuncPtr)gasFxn, &gas_task, NULL);

    /* Construct sensor  thread */
    Task_Params_init(&sensor_task);
    sensor_task.arg0 = SENSOR_PERIOD;
    sensor_task.stackSize = TASKSTACKSIZE;
    sensor_task.stack = &sensor_stack;
    Task_construct(&sensor_struct, (Task_FuncPtr)sensorFxn, &sensor_task, NULL);

     /* Turn on user LED */

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();
    uart_init();
    /* Start BIOS */
    BIOS_start();
    return (0);
}
