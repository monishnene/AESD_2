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

/*******************************************
* Macros
*******************************************/

#define PERIOD 10
#define DEBUG 1
#ifndef DEBUG
#define printf(fmt, ...) (0)
#endif

/*****************************
* Global variables
* shared mem and semaphores
*****************************/
#define STR_SIZE 200
#define logger_port 8000
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
		switch(input)
		{
			case LOG_DATA:
			{
				sem_post(sem_logger);
				break;
			}

			case GET_TEMPERATURE:
			{
				//send_data.data=current_temperature;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;	
			}

			case GET_HUMIDITY:
			{
				//send_data.data=current_humidity;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;
			}

			case GET_GAS:
			{
				//send_data.data=current_gas;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;
			}

			case GET_THRESHOLD:
			{
				//UART_write(uart, &temperature_threshold, sizeof(temperature_threshold));
				//UART_write(uart, &humidity_threshold, sizeof(humidity_threshold));
				//UART_write(uart, &gas_threshold, sizeof(gas_threshold));
				break;
			}

			case GET_FAN:
			{
				//send_data.data=fans_on;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;
			}

			case GET_BUZZER:
			{
				//send_data.data=buzzer;
				//UART_write(uart, &send_data, sizeof(uart_data_t));
				break;
			}

			case CHANGE_MODE:
			{
				//if(received_data.data<2)
				{
				//    remote_mode=received_data.data;
				}
				break;
			}

			case CHANGE_TEMPERATURE_THRESHOLD:
			{
				//UART_read(uart, &temperature_threshold, sizeof(temperature_threshold));
				break;
			}

			case CHANGE_HUMIDITY_THRESHOLD:
			{
				//UART_write(uart, &humidity_threshold, sizeof(temperature_threshold));
				break;
			}

			case CHANGE_GAS_THRESHOLD:
			{
				//UART_write(uart, &gas_threshold, sizeof(temperature_threshold));
				break;
			}

			case BUZZER_ON:
			{
				//remote_mode=MANUAL_MODE;
				//buzzer=1;
				//buzzer_control();
				break;
			}

			case BUZZER_OFF:
			{
				//remote_mode=MANUAL_MODE;
				//buzzer=0;
				//buzzer_control();
				break;
			}

			case FORCE_CHANGE_FANS:
			{
				//if(received_data.data<6)
				{
					//remote_mode=MANUAL_MODE;
					//fans_on=received_data.data;
					//Fan_update(fans_on);
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
