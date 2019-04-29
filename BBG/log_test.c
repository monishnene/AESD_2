/******************************************
* project.c
* Main task
* Author: Monish Nene and Sanika Dongre
* Date created: 04/18/19
*******************************************/

/*******************************************
* Includes
*******************************************/
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <syscall.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

//reference http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html

/*******************************************
* Macros
*******************************************/

#define PERIOD 10
#define DEBUG 1
#ifndef DEBUG
#define printf(fmt, ...) (0)
#endif
#define BAUDRATE B115200    
#define UART4 "/dev/ttyO4"	 
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
#define STR_SIZE 200
#define logger_port 8000

/*****************************
* Global variables
* shared mem and semaphores
*****************************/
uint8_t* logfile;
uint8_t* str;
sem_t* sem_logfile;
sem_t* sem_logger;
sem_t* sem_uart;
int32_t	shm_light,shm_temp,n,i,condition;
time_t present_time;
struct tm *time_and_date;
int32_t socket_desc;
struct sockaddr_in sock_struct_client,sock_struct_server;
static uint8_t* logtype[]={"LOG_INFO","LOG_DATA","LOG_ERROR"};
static uint8_t logger_ready_id[]="check if logger is ready";
static uint8_t logfile_sem_id[]="sem_logfile";
static uint8_t uart_id[]="uart communication";
pthread_t thread_logger;
int fd_uart4,c, res;
struct termios term_uart;
char buffer_in[255];
char buffer_out[]="\nHello World!\nUart is working ;-)\n";

volatile int STOP=FALSE; 

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

static volatile uint32_t log_counter=0;
uint8_t log_type;
uint8_t* logfile;
uint8_t* msg;
uint8_t fans_on=0;
bool buzzer=0;
bool remote_mode = AUTOMATIC_MODE;
int32_t current_temperature,current_gas,condition;
double current_humidity;
bool fans[5]={0,0,0,0,0};
int32_t temperature_threshold[5]={20,25,30,35,50};
int32_t humidity_threshold[5]={20,40,60,80,90};
int32_t gas_threshold[5]={20,40,60,80,90};
volatile uint32_t g_ui32Counter = 0;
uint8_t* error_msg[]={"The Log Queue is full, Data Lost","Log type not found"};
uint32_t g_ui32SysClock;
uint32_t sys_clock;
char uart_val;

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

uart_data_t send_data, received_data;
/*****************************
* logger function to log data
******************************/
void* logger(void* ptr)
{	
	uint32_t size=0,counter=0;
	FILE* fptr;
	uart_data_t command_sent;
	command_sent.command_id = LOG_DATA;
	uint8_t* msg = (uint8_t*)malloc(STR_SIZE);
	printf("logger started\n");
	while(condition)
	{
		sem_wait(sem_logger);
		sem_wait(sem_logfile);
		fptr=fopen(logfile,"a");		
		//uart_send
		counter=0;
		while(1)
		{
			size=read(fd_uart4,(void*)msg,sizeof(STR_SIZE));
			n=fwrite(msg,1,size,fptr);
			counter++;
			if(!strncmp(msg+size-9,"LOG_END\n\r",9))
			{
				printf("Is it stuck in loop? loop:%d size:%d\n",counter,size);
				break;
			} 				
		}
		fclose(fptr);
		sem_post(sem_logfile);
		//receive data from UART		
		sem_post(sem_uart);
	}
	free(msg);
	pthread_exit(ptr);
}

/*****************************
* log file setup function
********************************/
void logfile_setup(void)
{
	FILE* fptr=fopen(logfile,"r");
	if(fptr==NULL)
	{
		return;
	}
	uint8_t* new_filename=malloc(STR_SIZE);
	uint32_t counter=1;
	while(fptr!=NULL)
	{
		fclose(fptr);
		sprintf(new_filename,"backup_%d_%s",counter++,logfile); //to create backup files
		fptr=fopen(new_filename,"r");	
	}
	rename(logfile,new_filename);
	return;
}

static void log_time(int sig, siginfo_t *si, void *uc)
{
	uint8_t input ='A';
	sem_wait(sem_uart);
	write(fd_uart4,(void*)&input,1);
	sem_post(sem_logger);
	//heartbeat();
}


int32_t timer_init(void)
{
	int32_t error=0;	
	timer_t timerid;
	struct sigevent signal_event;
	struct itimerspec timer_data;
	struct sigaction signal_action;
	printf("Timer Init\n");
	// Timer init
	signal_action.sa_flags = SA_SIGINFO;
	signal_action.sa_sigaction = log_time;//Function to be executed
	sigemptyset(&signal_action.sa_mask);
	error=sigaction(SIGRTMIN, &signal_action, NULL);
    	signal_event.sigev_notify = SIGEV_SIGNAL;
    	signal_event.sigev_signo = SIGRTMIN;
    	signal_event.sigev_value.sival_ptr = &timerid;
    	error=timer_create(CLOCK_REALTIME, &signal_event, &timerid);
	/* Start the timer */
    	timer_data.it_value.tv_sec = PERIOD;
    	timer_data.it_value.tv_nsec = 0;
   	timer_data.it_interval.tv_sec = timer_data.it_value.tv_sec;
    	timer_data.it_interval.tv_nsec = timer_data.it_value.tv_nsec;
	error=timer_settime(timerid, 0, &timer_data, NULL);
	return error;
}

void system_end(int sig)
{
	condition=0;
}

void uart_init(void)
{
	fd_uart4 = open(UART4, O_RDWR,O_SYNC, O_NOCTTY); 
	if (fd_uart4 <0)
	{
		printf("uart4 not found\n");
	}
	if(!isatty(fd_uart4))
	{
		perror("uart4_fd pointing error\n");
	}
	printf("uart4 found\n");
	fcntl(fd_uart4,F_SETFL,0);
}

void termios_init(void)
{
	tcgetattr(fd_uart4,&term_uart); /* save current serial port settings */

	/* 
	BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	CRTSCTS : output hardware flow control (only used if the cable has
	all necessary lines. See sect. 7 of Serial-HOWTO)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters
	*/
	term_uart.c_iflag=0;
	term_uart.c_oflag &= ~OPOST;
	term_uart.c_lflag &= ~(ECHO | ECHONL | ICANON | ECHOK | ECHOE | ISIG);
	term_uart.c_cflag = (BAUDRATE| CS8 | CLOCAL | CREAD);
	term_uart.c_cc[VMIN] = 1;
	term_uart.c_cc[VTIME] = 0;
	tcflush(fd_uart4, TCIFLUSH);
	tcsetattr(fd_uart4,TCSANOW,&term_uart);
}

/*int uart_write(char data_write[])
{
	write(fd_uart0,data_write,strlen(data_write));
	return 0;
}

char uart_read(void)
{
	char data_read[255];
	read(fd_uart0,data_read,255); 
    	buffer_out[res]=0;             /* set end of string, so we can printf */
    	/*printf(":%s:%d\n",data_read, res);
	return data_read;
}*/

int32_t main(int32_t argc, uint8_t **argv)
{
	uint8_t data_read;
	uart_init();
	termios_init();
	condition = 1;
	if(fork())
	{
		while(condition)
		{
			read(fd_uart4,&data_read,1);
			putchar(data_read);
		}
	}
	else
	{
		while(condition)
		{
			data_read=getchar();	
			write(fd_uart4,&data_read,1);
		}	
	}
}
