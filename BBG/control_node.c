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
#define UART0 "/dev/ttyO0"
#define BAUDRATE B38400    
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
int fd_uart0,fd_uart4,c, res;
struct termios oldtio0,oldtio4,newtio0,newtio4;
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

QueueHandle_t log_queue;
SemaphoreHandle_t sem_log,sem_read;
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
	uint32_t size=0;
	FILE* fptr;
	uart_data_t command_sent;
	command_sent.command_id = LOG_DATA;
	uint8_t* msg = (uint8_t*)malloc(STR_SIZE);
	printf("logger started\n");
	while(condition)
	{
		sem_wait(sem_logger);
		sem_wait(sem_uart);
		//uart_send
		//receive data from UART		
		sem_post(sem_uart);
		sem_wait(sem_logfile);
		fptr=fopen(logfile,"a");
		printf("Log command sent\n");		
		//n=fwrite(msg,1,size,fptr);
		fclose(fptr);
		sem_post(sem_logfile);
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
	/* 
	  Open modem device for reading and writing and not as controlling tty
	  because we don't want to get killed if linenoise sends CTRL-C.
	*/
	 fd_uart0 = open(UART0, O_RDWR | O_NOCTTY ); 
	 if (fd_uart0 <0)
	{
		printf("uart0 not found\n");
	}
	else
	{
		printf("uart0 found\n");
	}
	fd_uart4 = open(UART4, O_RDWR | O_NOCTTY ); 
 	if (fd_uart4 <0)
	{
		printf("uart4 not found\n");
	}
	else
	{
		printf("uart4 found\n");
	}
	tcgetattr(fd_uart0,&oldtio0); /* save current serial port settings */
	tcgetattr(fd_uart4,&oldtio4); /* save current serial port settings */
	/* 
	  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	  CRTSCTS : output hardware flow control (only used if the cable has
		    all necessary lines. See sect. 7 of Serial-HOWTO)
	  CS8     : 8n1 (8bit,no parity,1 stopbit)
	  CLOCAL  : local connection, no modem contol
	  CREAD   : enable receiving characters
	*/
	 newtio0.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
	 newtio4.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
	/*
	  IGNPAR  : ignore bytes with parity errors
	  ICRNL   : map CR to NL (otherwise a CR input on the other computer
		    will not terminate input)
	  otherwise make device raw (no other input processing)
	*/
	 newtio0.c_iflag = IGNPAR | ICRNL;
	 newtio4.c_iflag = IGNPAR | ICRNL; 
	/*
	 Raw output.
	*/
	 newtio0.c_oflag = 0;
	 newtio4.c_oflag = 0;
	 
	/*
	  ICANON  : enable canonical input
	  disable all echo functionality, and don't send signals to calling program
	*/
	 newtio0.c_lflag = ICANON;
	 newtio4.c_lflag = ICANON;
	 
	/* 
	  initialize all control characters 
	  default values can be found in /usr/include/termios.h, and are given
	  in the comments, but we don't need them here
	*/

	 newtio0.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
	 newtio0.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
	 newtio0.c_cc[VERASE]   = 0;     /* del */
	 newtio0.c_cc[VKILL]    = 0;     /* @ */
	 newtio0.c_cc[VEOF]     = 4;     /* Ctrl-d */
	 newtio0.c_cc[VTIME]    = 0;     /* inter-character timer unused */
	 newtio0.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
	 newtio0.c_cc[VSWTC]    = 0;     /* '\0' */
	 newtio0.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
	 newtio0.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	 newtio0.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	 newtio0.c_cc[VEOL]     = 0;     /* '\0' */
	 newtio0.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	 newtio0.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	 newtio0.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	 newtio0.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	 newtio0.c_cc[VEOL2]    = 0;     /* '\0' */
		
	 newtio4.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
	 newtio4.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
	 newtio4.c_cc[VERASE]   = 0;     /* del */
	 newtio4.c_cc[VKILL]    = 0;     /* @ */
	 newtio4.c_cc[VEOF]     = 4;     /* Ctrl-d */
	 newtio4.c_cc[VTIME]    = 0;     /* inter-character timer unused */
	 newtio4.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
	 newtio4.c_cc[VSWTC]    = 0;     /* '\0' */
	 newtio4.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
	 newtio4.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	 newtio4.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	 newtio4.c_cc[VEOL]     = 0;     /* '\0' */
	 newtio4.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	 newtio4.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	 newtio4.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	 newtio4.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	 newtio4.c_cc[VEOL2]    = 0;     /* '\0' */

	/* 
	  now clean the modem line and activate the settings for the port
	*/
	 tcflush(fd_uart0, TCIFLUSH);
	 tcsetattr(fd_uart0,TCSANOW,&newtio0);
	 tcflush(fd_uart4, TCIFLUSH);
	 tcsetattr(fd_uart4,TCSANOW,&newtio4);
}

int uart_write(char data_write[])
{
	res = write(fd_uart0,data_write,strlen(data_write));
	return 0;
}

char uart_read(void)
{
	char data_read[255];
	res = read(fd_uart0,data_read,255); 
    	buffer_out[res]=0;             /* set end of string, so we can printf */
    	printf(":%s:%d\n",data_read, res);
	return data_read;
}

int32_t main(int32_t argc, uint8_t **argv)
{
	uint8_t input=0;
	int32_t error=0;	
	pid_t process_id = getpid();	
	if(argc<2)
	{
		printf("%s <logfilename>\n",argv[0]);	 //log file name as command line argument
		return 0;
	}	
	condition=1;
	sem_logger=sem_open(logger_ready_id, O_CREAT, 0644,0);	
	sem_logfile=sem_open(logfile_sem_id, O_CREAT, 0644,1);	
	sem_uart=sem_open(uart_id, O_CREAT, 0644,1);
	logfile=argv[1];
	logfile_setup();
	error = pthread_create(&thread_logger,NULL,logger,NULL);
	//Signal Handling
	//signal(SIGUSR1,kill_logger);
	//signal(SIGUSR2,kill_server);
	timer_init();		
	//pthread join
	if(!fork())
	{
		error=pthread_join(thread_logger,NULL);
	}
	if(process_id!=getpid())
	{
		kill(getpid(),SIGINT);
	}	
	signal(SIGINT,system_end);
	printf("\n \
			LOG_DATA='A'\n \
			GET_TEMPERATURE='B'\n \
			GET_HUMIDITY='C'\n \
			GET_GAS='D'\n \
			GET_THRESHOLD='E'\n \
			GET_FAN='F'\n \
			CHANGE_MODE='G'\n \
			CHANGE_TEMPERATURE_THRESHOLD='H'\n \
			CHANGE_HUMIDITY_THRESHOLD='I'\n \
			CHANGE_GAS_THRESHOLD='J'\n \
			BUZZER_ON='K'\n \
			BUZZER_OFF='L'\n \
			FORCE_CHANGE_FANS='M'\n \
			GET_BUZZER='N'\n \
			RETRY_BIST='O'\n \
			EXIT CONTROL NODE='X'\n \
			DISPLAY_COMMNANDS='?'\n");
	while(condition)
	{
		if(input!='\n')
		{
			printf("\nEnter new command:\n");
		}				
		scanf("%c",&input);
		send_data.command_id = input;
		res = write(fd_uart0,send_data,strlen(send_data));
		switch(input)
		{
			case LOG_DATA:
			{
				sem_post(sem_logger);
				break;
			}

			case GET_TEMPERATURE:
			{
																		        				res = read(fd_uart0,received_data,sizeof(received_data)); 				
				//send_data.data=current_temperature;
				//uart_write(
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;	
			}

			case GET_HUMIDITY:
			{
				res= read(fd_uart0,received_data,sizeof(received_data));
				//send_data.data=current_humidity;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;
			}

			case GET_GAS:
			{
				res=read(fd_uart0,received_data,sizeof(received_data));

				//send_data.data=current_gas;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;
			}

			case GET_THRESHOLD:
			{
				res=read(fd_uart0,received_data,sizeof(received_data));
				//UART_write(uart, &temperature_threshold, sizeof(temperature_threshold));
				//UART_write(uart, &humidity_threshold, sizeof(humidity_threshold));
				//UART_write(uart, &gas_threshold, sizeof(gas_threshold));
				break;
			}

			case GET_FAN:
			{
				res=read(fd_uart0,received_data,sizeof(received_data));
				//send_data.data=fans_on;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;
			}

			case GET_BUZZER:
			{
				res=read(fd_uart0,received_data,sizeof(received_data));
				//send_data.data=buzzer;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;
			}

			case CHANGE_MODE:
			{
				if(received_data.data<2)
				{
				    remote_mode=received_data.data;
				}
				break;
			}

			case CHANGE_TEMPERATURE_THRESHOLD:
			{
				scanf("%d", &temperature_threshold[0]);
				scanf("%d", &temperature_threshold[1]);
				scanf("%d", &temperature_threshold[2]);
				scanf("%d", &temperature_threshold[3]);
				scanf("%d", &temperature_threshold[4]);
				//UART_read(uart, &temperature_threshold, sizeof(temperature_threshold));
				break;
			}

			case CHANGE_HUMIDITY_THRESHOLD:
			{
				scanf("%d", &humidity_threshold[0]);
				scanf("%d", &humidity_threshold[1]);
				scanf("%d", &humidity_threshold[2]);
				scanf("%d", &humidity_threshold[3]);
				scanf("%d", &humidity_threshold[4]);
				//UART_write(uart, &humidity_threshold, sizeof(temperature_threshold));
				break;
			}

			case CHANGE_GAS_THRESHOLD:
			{
				scanf("%d", &gas_threshold[0]);
				scanf("%d", &gas_threshold[1]);
				scanf("%d", &gas_threshold[2]);
				scanf("%d", &gas_threshold[3]);
				scanf("%d", &gas_threshold[4]);
				//UART_write(uart, &gas_threshold, sizeof(temperature_threshold));
				break;
			}

			case BUZZER_ON:
			{
				remote_mode=MANUAL_MODE;
				buzzer=1;
				buzzer_control();
				break;
			}

			case BUZZER_OFF:
			{
				remote_mode=MANUAL_MODE;
				buzzer=0;
				buzzer_control();
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

			case RETRY_BIST:
			{
				break;
			}
			
			case 'X':
			{
				condition=0;
				break;
			}
	
			case '\n':
			{
				break;
			}
		
			case '?':
			{
				printf("\n \
			LOG_DATA='A'\n \
			GET_TEMPERATURE='B'\n \
			GET_HUMIDITY='C'\n \
			GET_GAS='D'\n \
			GET_THRESHOLD='E'\n \
			GET_FAN='F'\n \
			CHANGE_MODE='G'\n \
			CHANGE_TEMPERATURE_THRESHOLD='H'\n \
			CHANGE_HUMIDITY_THRESHOLD='I'\n \
			CHANGE_GAS_THRESHOLD='J'\n \
			BUZZER_ON='K'\n \
			BUZZER_OFF='L'\n \
			FORCE_CHANGE_FANS='M'\n \
			GET_BUZZER='N'\n \
			RETRY_BIST='O'\n \
			DISPLAY_COMMNANDS='?'\n");
				break;		
			}

			default:
			{
				printf("Invalid Input\n");				
				break;
			}
		}
	}
	sem_unlink(uart_id);
	sem_unlink(logfile_sem_id);
	sem_unlink(logger_ready_id);
}
