/* FreeRTOS 8.2 Tiva Demo
 *
 * main.c
 *
 * Andy Kobyljanec
 *
 * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
 * EK-TM4C1294XL.  TivaWare driverlib sourcecode is included.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "main.h"
#include "math.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "drivers/pinout.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "drivers/pinout.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "utils/uartstdio.h"

#define MQ_SIZE         (10)            // Size of the message queue
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
#define AUTOMATIC_MODE 1
#define MANUAL_MODE 0
#define SI7021_SLAVE_ADDRESS (0x40)
#define TEMP_ADDRESS (0xE3)
#define HUMIDITY_ADDRESS (0xE5)
#define HUMIDITY_ERROR 0
#define TEMPERATURE_ERROR 0
#define GAS_ERROR 3
#define NORMAL 0
#define DEGRADED 1
#define FAILURE 2

QueueHandle_t log_queue;
SemaphoreHandle_t sem_log,sem_read,sem_uart,sem_uart_comm;
static volatile uint32_t log_counter=0;
uint8_t log_type;
uint8_t* logfile;
uint8_t msg[STR_SIZE];
uint8_t fans_on=0;
bool buzzer=0;
uint8_t led_status=0;
int32_t current_temperature,current_gas,condition;
double current_humidity;
bool fans[5]={0,0,0,0,0};
int32_t temperature_threshold[5]={20,25,30,35,50};
int32_t humidity_threshold[5]={20,40,60,80,90};
int32_t gas_threshold[5]={20,40,60,80,90};
volatile uint32_t g_ui32Counter = 0;
volatile bool sensor_check=1;
uint8_t* error_msg[]={"The Log Queue is full, Data Lost","Log type not found"};
uint8_t* mode_msg[]={"Manual","Automatic"};
bool remote_mode = AUTOMATIC_MODE;
uint8_t* failure_msg[]={"Normal","Degraded","Failure"};
uint8_t failure_index=0;
bool sensor_status=1;
bool gas_status=1;
uint32_t g_ui32SysClock;

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
    LOG_COMMAND=5,
    LOG_FAN=6,
    LOG_ERROR=7,
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
    GET_TEMPERATURE='B',
    GET_HUMIDITY='C',
    GET_GAS='D',
    GET_THRESHOLD='E',
    GET_FAN='F',
    CHANGE_MODE='G',
    CHANGE_TEMPERATURE_THRESHOLD='H',
    CHANGE_HUMIDITY_THRESHOLD='I',
    CHANGE_GAS_THRESHOLD='J',
    BUZZER_ON='K',
    BUZZER_OFF='L',
    FORCE_CHANGE_FANS='M',
    GET_BUZZER='N',
    RETRY_BIST='O',
}uart_command_t;

typedef struct
{
    uart_command_t command_id;
    int32_t data;
    int32_t time_now;
}uart_data_t;


void exit_handler(void)
{
    condition=0;
}

int32_t uart_send(uint8_t* ptr, uint32_t size)
{
    int32_t i=0;
    while(*(ptr+i)!=0)
    {
        UARTCharPut(UART7_BASE, *(ptr+i));
        i++;
        if(i==size)
        {
            break;
        }
    }
    return i;
}

int32_t uart_receive(uint8_t* ptr, uint32_t size)
{
    int32_t i=0;
    for(i=0;i<size;i++)
    {
        *(ptr+i)=UARTCharGet(UART7_BASE);
    }
    return i;
}

void queue_adder(queue_data_t* data_send)
{
    data_send->time_now =  xTaskGetTickCount();
    if(log_counter==0)
    {
        xSemaphoreGive(sem_read);
    }
    if(log_counter < QUEUE_SIZE-1)
    {
        led_status&=0xFE;
        LEDWrite(0x0F,led_status);
        xQueueSend(log_queue,( void * )data_send,(TickType_t)10);
        log_counter++;
    }
    else if(log_counter == QUEUE_SIZE-1)
    {
        data_send->data=0;
        data_send->log_id=LOG_ERROR;
        led_status|=0x01;
        LEDWrite(0x0F,led_status);
        xQueueSend(log_queue,( void * )data_send,(TickType_t)10);
        log_counter++;
    }
    else
    {
        LEDWrite(0x0F, 0x01);
    }
}

void buzzer_control(void)
{
    if(buzzer)
    {
        //buzzer on
    }
    else
    {
        //buzzer off
    }
}

void Fan_update(int8_t value)
{
    queue_data_t data_send;
    uint8_t i=0;
    for(i=0;i<5;i++)
    {
        fans[i]=i<=value?1:0;
    }
    buzzer=fans[4];
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, fans[0]);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, fans[1]);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, fans[2]);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, fans[3]);
    if(remote_mode)
    {
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4,buzzer);
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5,buzzer);
    }
    data_send.data=value;
    data_send.log_id=LOG_FAN;
    queue_adder(&data_send);
}

void i2c_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    I2CMasterInitExpClk(I2C0_BASE, g_ui32SysClock, false);
}

void gpio_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_5);
}

void UARTIntHandler(void )
{
    uart_data_t send_data, received_data;
    queue_data_t data_log;
    uint32_t status = UARTIntStatus(UART7_BASE, true);
    UARTIntClear(UART7_BASE, status);
    uint8_t command;
    send_data.time_now=xTaskGetTickCount();
    command=UARTCharGet(UART7_BASE);
    data_log.log_id=LOG_COMMAND;
    data_log.data=command;
    queue_adder(&data_log);
    send_data.command_id=(uart_command_t)command;
    send_data.time_now=xTaskGetTickCount();
    switch(command)
    {
        case LOG_DATA:
        {
            xSemaphoreGive(sem_log);
            break;
        }

        case GET_TEMPERATURE:
        {
            send_data.data=current_temperature;
            uart_send((void*)&send_data,sizeof(uart_data_t));
            break;
        }

        case GET_HUMIDITY:
        {
            send_data.data=current_humidity;
           uart_send((void*)&send_data,sizeof(uart_data_t));
            break;
        }

        case GET_GAS:
        {
            send_data.data=current_gas;
           uart_send((void*)&send_data,sizeof(uart_data_t));
            break;
        }

        case GET_THRESHOLD:
        {
            UARTprintf("The temperature threshold is %d", temperature_threshold);
            UARTprintf("The humidity threshold is %d", humidity_threshold);
            UARTprintf("The gas threshold is %d", gas_threshold);
            uart_send((void*)&temperature_threshold,sizeof(int32_t)*5);
            uart_send((void*)&humidity_threshold,sizeof(int32_t)*5);
            uart_send((void*)&gas_threshold,sizeof(int32_t)*5);
            break;
        }

        case GET_FAN:
        {
            send_data.data=fans_on;
            uart_send((void*)&send_data,sizeof(uart_data_t));
            break;
        }

        case GET_BUZZER:
        {
            send_data.data=buzzer;
            uart_send((void*)&send_data,sizeof(uart_data_t));
            break;
        }

        case CHANGE_MODE:
        {
            if(remote_mode)
            {
               remote_mode=MANUAL_MODE;
            }
            else
            {
               remote_mode=AUTOMATIC_MODE;
            }
            break;
        }

        case CHANGE_TEMPERATURE_THRESHOLD:
        {
            uart_receive((void*)&temperature_threshold,sizeof(int32_t)*5);
            break;
        }

        case CHANGE_HUMIDITY_THRESHOLD:
        {
            uart_receive((void*)&humidity_threshold,sizeof(int32_t)*5);
            break;
        }

        case CHANGE_GAS_THRESHOLD:
        {
            uart_receive((void*)&gas_threshold,sizeof(int32_t)*5);
            break;
        }


        case BUZZER_ON:
        {
            remote_mode=MANUAL_MODE;
            buzzer=0;
            GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4,buzzer);
            GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5,buzzer);
            break;
        }

        case BUZZER_OFF:
        {
            remote_mode=MANUAL_MODE;
            buzzer=1;
            GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4,buzzer);
            GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5,buzzer);
            break;
        }

        case FORCE_CHANGE_FANS:
        {
            if(received_data.data<6)
            {
                remote_mode=MANUAL_MODE;
                fans_on=received_data.data;
                Fan_update(fans_on);
            }
            break;
        }

        default:
        {
            break;
        }
    }
}

void uart_init(void)
{
    //    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //    GPIOPinConfigure(GPIO_PA0_U0RX);
    //    GPIOPinConfigure(GPIO_PA1_U0TX);
    //    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    //    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    //    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTConfigSetExpClk(UART7_BASE, 12e7, 9600,
                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART7);
    UARTIntEnable(UART7_BASE,UART_INT_RX | UART_INT_RT);
    //UARTClockSourceSet(UART7_BASE, UART_CLOCK_PIOSC);
}

void UARTFxn(void* ptr)
{
    uint8_t command;
    uart_data_t received_data,send_data;
    queue_data_t data_log;
    send_data.time_now=xTaskGetTickCount();
    while(condition)
    {
        command=UARTCharGet(UART0_BASE);
        data_log.log_id=LOG_COMMAND;
        data_log.data=command;
        queue_adder(&data_log);
        send_data.command_id=(uart_command_t)command;
        send_data.time_now=xTaskGetTickCount();
        switch(command)
        {
            case LOG_DATA:
            {
                xSemaphoreGive(sem_log);
                break;
            }

            case GET_TEMPERATURE:
            {
                send_data.data=current_temperature;
                uart_send((void*)&send_data,sizeof(uart_data_t));
                break;
            }

            case GET_HUMIDITY:
            {
                send_data.data=current_humidity;
                uart_send((void*)&send_data,sizeof(uart_data_t));
                break;
            }

            case GET_GAS:
            {
                send_data.data=current_gas;
                uart_send((void*)&send_data,sizeof(uart_data_t));
                break;
            }

            case GET_THRESHOLD:
            {
                UARTprintf("The temperature threshold is %d", temperature_threshold);
                UARTprintf("The humidity threshold is %d", humidity_threshold);
                UARTprintf("The gas threshold is %d", gas_threshold);
                uart_send((void*)&temperature_threshold,sizeof(int32_t)*5);
                uart_send((void*)&humidity_threshold,sizeof(int32_t)*5);
                uart_send((void*)&gas_threshold,sizeof(int32_t)*5);
                break;
            }

            case GET_FAN:
            {
                send_data.data=fans_on;
                uart_send((void*)&send_data,sizeof(uart_data_t));
                break;
            }

            case GET_BUZZER:
            {
                send_data.data=buzzer;
                uart_send((void*)&send_data,sizeof(uart_data_t));
                break;
            }

            case CHANGE_MODE:
            {
                if(remote_mode)
                {
                   remote_mode=MANUAL_MODE;
                }
                else
                {
                   remote_mode=AUTOMATIC_MODE;
                }
                break;
            }

            case CHANGE_TEMPERATURE_THRESHOLD:
            {
                uart_receive((void*)&temperature_threshold,sizeof(int32_t)*5);
                break;
            }

            case CHANGE_HUMIDITY_THRESHOLD:
            {
                uart_receive((void*)&humidity_threshold,sizeof(int32_t)*5);
                break;
            }

            case CHANGE_GAS_THRESHOLD:
            {
                uart_receive((void*)&gas_threshold,sizeof(int32_t)*5);
                break;
            }

            case BUZZER_ON:
            {
                remote_mode=MANUAL_MODE;
                buzzer=0;
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4,buzzer);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5,buzzer);
                break;
            }

            case BUZZER_OFF:
            {
                remote_mode=MANUAL_MODE;
                buzzer=1;
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4,buzzer);
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5,buzzer);
                break;
            }

            case FORCE_CHANGE_FANS:
            {
                if(received_data.data<6)
                {
                    remote_mode=MANUAL_MODE;
                    fans_on=received_data.data;
                    Fan_update(fans_on);
                }
                break;
            }

            default:
            {
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void loggerFxn(void* ptr)
{
    uint8_t time_str[50];
    uint8_t timeslice;
    queue_data_t received_data;
    while (condition)
    {
         //message queue receive

         xSemaphoreTake(sem_log, portMAX_DELAY);
         xSemaphoreTake(sem_read, portMAX_DELAY);
         while(log_counter<1);
         if(log_counter>0)
         {
             if(xQueueReceive(log_queue, &(received_data), portMAX_DELAY)==0)
             {
                 continue;
             }
         }
         log_counter--;
         sprintf(time_str,"time: %d sec %d msec\t%s\t%s\t",received_data.time_now/1000,received_data.time_now%1000,mode_msg[remote_mode],failure_msg[failure_index]);
         timeslice=strlen(time_str);
         memcpy(msg,time_str,timeslice);
         switch(received_data.log_id)
         {
            case LOG_LED:
            {
                sprintf(msg+timeslice,"LOG_LED\tled toggle count = %d\n\r",received_data.data);
                break;
            }

            case LOG_TEMPERATURE:
            {
                sprintf(msg+timeslice,"LOG_TEMPERATURE\tTemperature: %dC, %dF, %dK\n\r",received_data.data,((received_data.data*9)/5)+32,received_data.data+273);
                break;
            }

            case LOG_COMMAND:
            {
                sprintf(msg+timeslice,"LOG_COMMAND \tCommand received: %c\n\r",received_data.data);
                break;
            }

            case LOG_HUMIDITY:
            {
                sprintf(msg+timeslice,"LOG_HUMIDITY\tHumidity: %d\n\r",received_data.data);
                break;
            }

            case LOG_GAS:
            {
                sprintf(msg+timeslice,"LOG_GAS\t\tThe CO value is %d ppm\n\r",received_data.data);
                break;
            }

            case LOG_FAN:
            {
                sprintf(msg+timeslice,"LOG_FAN\tFans ON: %d\n\r",received_data.data);
                break;
            }

            case LOG_THRESHOLD:
            {
                sprintf(msg+timeslice,"LOG_THRESHOLD\tTHRESHOLD CHECKED\n\r");
                break;
            }

            case LOG_ERROR:
            {
                sprintf(msg+timeslice,"LOG_ERROR\t%s\n\r",error_msg[received_data.data]);
                break;
            }

            default:
            {
                sprintf(msg+timeslice,"LOG_ERROR\t%s data:%d\n\r",error_msg[ERROR_LOGTYPE],received_data.data);
                break;
            }
         }
         if(log_counter)
         {
             xSemaphoreGive(sem_log);
             xSemaphoreGive(sem_read);
         }
         else
         {
             xSemaphoreTake(sem_uart,portMAX_DELAY);
             UARTprintf(msg);
             //uart_send(msg,strlen(msg));
             xSemaphoreGive(sem_uart);
             sprintf(msg+timeslice,"LOG_END\n\r");
         }
         xSemaphoreTake(sem_uart,portMAX_DELAY);
         UARTprintf(msg);
         //uart_send(msg,strlen(msg));
         xSemaphoreGive(sem_uart);
    }
    vTaskDelete(NULL);
}

void gasFxn(void* ptr)
{
    queue_data_t data_send;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1));
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_2);
    ADCSequenceConfigure(ADC1_BASE,3,ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceStepConfigure(ADC1_BASE,3,0,ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE,3);
    while (condition)
    {
       vTaskDelay(GAS_PERIOD);
       ADCProcessorTrigger(ADC1_BASE,3);
       while(!ADCIntStatus(ADC1_BASE,3,false));
       ADCIntClear(ADC1_BASE,3);
       uint32_t data_op[1];
       ADCSequenceDataGet(ADC1_BASE,3,data_op);
       current_gas = (((data_op[0]*3.3)/4095)*1.0698);
       current_gas = 3.027*(pow(2.718,current_gas));
       data_send.data = current_gas;
       data_send.log_id=LOG_GAS;
       queue_adder(&data_send);
       if(current_gas==GAS_ERROR)
       {
          gas_status=0;
          led_status^=0x04;
       }
       else
       {
           gas_status=1;
           led_status&=0xFB;
       }
       LEDWrite(0x0F,led_status);
    }
    vTaskDelete(NULL);
}

void thresholdFxn(void* ptr)
{
    uint8_t i=0;
    queue_data_t data_send;
    while (condition)
    {
        vTaskDelay(THRESHOLD_PERIOD);
        for(i=0;i<5;i++)
        {
            if((current_temperature<temperature_threshold[i])&&(current_humidity<humidity_threshold[i])&&(current_gas<gas_threshold[i]))
            {
                break;
            }
        }
        data_send.data=rand()%5;
        data_send.log_id=LOG_THRESHOLD;
        queue_adder(&data_send);
        if(gas_status&&sensor_status&&sensor_check)
        {
            failure_index=NORMAL;
            if((i!=fans_on)&&remote_mode)
            {
                fans_on=i;
                Fan_update(fans_on);
            }
            led_status&=0xF7;
        }
        else if(gas_status||(sensor_status&&sensor_check))
        {
            failure_index=DEGRADED;
            if(fans_on<2)
            {
                fans_on=2;
                Fan_update(fans_on);
            }
            led_status^=0x08;
        }
        else
        {
            failure_index=FAILURE;
            if(fans_on<5)
            {
                fans_on=5;
                Fan_update(fans_on);
            }
            led_status|=0x08;
        }
        if(!sensor_check)
        {
               sensor_status=0;
               led_status^=0x02;
        }
        else
        {
               sensor_status=1;
               led_status&=0xFD;
        }
        sensor_check=0;
        LEDWrite(0x0F,led_status);
    }
    vTaskDelete(NULL);
}

void sensorFxn(void* ptr)
{
    uint16_t data_op[2];
    queue_data_t data_send;
    while (condition)
    {
       vTaskDelay(SENSOR_PERIOD);
       I2CMasterSlaveAddrSet(I2C0_BASE, SI7021_SLAVE_ADDRESS, false);
       I2CMasterDataPut(I2C0_BASE,TEMP_ADDRESS);
       I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
       while(I2CMasterBusy(I2C0_BASE));
       I2CMasterSlaveAddrSet(I2C0_BASE,SI7021_SLAVE_ADDRESS,true);
       I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
       while(I2CMasterBusy(I2C0_BASE));
       data_op[0]=I2CMasterDataGet(I2C0_BASE);
       I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
       while(I2CMasterBusy(I2C0_BASE));
       data_op[1]=I2CMasterDataGet(I2C0_BASE);
       current_temperature=((((data_op[0]<<8|data_op[1])*175.72)/65536)-46.85);//replace this
       data_send.data=current_temperature;
       data_send.log_id=LOG_TEMPERATURE;
       queue_adder(&data_send);
       //humidity
       I2CMasterSlaveAddrSet(I2C0_BASE, SI7021_SLAVE_ADDRESS, false);
       I2CMasterDataPut(I2C0_BASE, HUMIDITY_ADDRESS);
       I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
       while(I2CMasterBusy(I2C0_BASE));
       I2CMasterSlaveAddrSet(I2C0_BASE,SI7021_SLAVE_ADDRESS,true);
       I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
       while(I2CMasterBusy(I2C0_BASE));
       data_op[0]=I2CMasterDataGet(I2C0_BASE);
       I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
       while(I2CMasterBusy(I2C0_BASE));
       data_op[1]=I2CMasterDataGet(I2C0_BASE);
       current_humidity=(((data_op[0]<<8|data_op[1])*125)/65536)-6;
       data_send.data=current_humidity;
       data_send.log_id=LOG_HUMIDITY;
       queue_adder(&data_send);
       sensor_check=1;
       if((current_humidity==HUMIDITY_ERROR)&&(current_temperature==TEMPERATURE_ERROR))
       {
           sensor_status=0;
           led_status^=0x02;
       }
       else
       {
           sensor_status=1;
           led_status&=0xFD;
       }
       LEDWrite(0x0F,led_status);
    }
    vTaskDelete(NULL);
}

// Main function
int main(void)
{
    // Initialize system clock to 120 MHz
    g_ui32SysClock = MAP_SysCtlClockFreqSet(( SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 12e7);
    g_ui32Counter = 0;
    // Set up the UART which is connected to the virtual COM port
    condition=1;
    i2c_init();
    //uart_init();
    UARTStdioConfig(0, 115200, 12e7);
    gpio_init();
    SysTickPeriodSet(12e4);
    SysTickIntEnable();
    SysTickEnable();
    log_queue=xQueueCreate(QUEUE_SIZE,sizeof(queue_data_t));
    sem_read = xSemaphoreCreateBinary();
    sem_log = xSemaphoreCreateBinary();
    sem_uart = xSemaphoreCreateBinary();
    sem_uart_comm = xSemaphoreCreateBinary();
    xSemaphoreGive(sem_uart);
    // Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);
    // Create demo tasks
    xTaskCreate(sensorFxn, (const portCHAR *)"sensor", TASKSTACKSIZE, NULL, 1, NULL);
    xTaskCreate(thresholdFxn, (const portCHAR *)"threshold", TASKSTACKSIZE, NULL, 1, NULL);
    xTaskCreate(loggerFxn, (const portCHAR *)"logger", TASKSTACKSIZE, NULL, 1, NULL);
    xTaskCreate(gasFxn, (const portCHAR *)"gas", TASKSTACKSIZE, NULL, 1, NULL);
    xTaskCreate(UARTFxn, (const portCHAR *)"uart", TASKSTACKSIZE, NULL, 1, NULL);
    //IntMasterEnable();
    vTaskStartScheduler();
    return 0;
}

/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
