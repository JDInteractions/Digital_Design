/* Digital_Design.c
 *
 * Created: 04-06-2021 13:21:47
 * Author : Jan & Lars
 */ 


#include "main.h"



char UARTBuffer[datasize] = {0};
unsigned int Len = 0;
char uart_type = 0;
unsigned int checksum_val = 0;
char data[datasize] = {0};


unsigned int S_Rate, S_rate_max = 0;
char SW = 0;
char BTN = 0;
char reset = 0;
char stop = SPI_START;
char telecommand[GEN_PKG+PADDING_SIZE]={0};
char bodeBuffer[BODE_PKG+PADDING_SIZE]={0};	
char spi_package[4]={0x55,0,0,0};
enum states {sync1, wait, sync2, Length, Length2, Type, ReadData, Check1, Check2};
enum tilstande {scope, set_sample, set_gen, BodePlot};
enum parameter {shape_s,amplitude_s,freq_s};
char param = shape_s;
char state = sync1;
char tilstand = scope;

char checksum_flag = 0;		//bruges til debug
//Holds latest adc sample -> read on adc interrupt

//Service Routine variables
volatile char adc_flag = 0;
volatile char uart_tx_flag = 1;
volatile char flag_uart_rx = 0;
volatile unsigned int uart_cnt_rx = 0;
 
//Double buffer 
unsigned int bufferCounter = 0;
char sampleBuffer[3][SAMPLE_BUF] = {{0},{0}};
int adc_user = 0;
int uart_user = 1;		//TODO char??
char checksum_flag=0;


unsigned int recordLength = 0;	


int main(void){ 
    
	setup();
	
    
    while (1){
	
		//Main program switch case. Default/starting state is "scope",
		//where adc_flag and uart-recieve flag is continuously polled.
		//The "handletype()"-function returns the next state based on uart-reception. 
		switch(tilstand){
		
			//Grundtilstand. Tjek for uart-flag. skift tilstand baseret p� uart-type. 
			case scope:
		
 				if(adc_flag){
					uart_tx_flag = 1;
	 				transmitADCSample(&sampleBuffer[uart_user][0], SCOPE_TYPE, recordLength);
					uart_tx_flag = 0;
					 adc_flag = 0;
				 
				}
				if(flag_uart_rx==1){
					flag_uart_rx=0;
					tilstand = handle_type(uart_type);
				}
				break;
		
			//SEND button pressed. S_Rate and recordLenght is updated. 
			//Samplerate  is compensated for record lengths < 47. 
			//Returns default state - "scope". 
			case set_sample:
				S_Rate = (data[5]<<8) | data[6];
				RL = (data[7]<<8) | data[8];
				setSampleRate(S_Rate);
				recordLength = RL;
				if(recordLength < MIN_RECORD_LENGTH){
					S_rate_max = sampleRate_comp(recordLength);
					if(S_Rate > S_rate_max){
						setSampleRate(S_rate_max);
					}
				}
				tilstand = scope;		
				break;
		
			//Button-press from generator tab registered. "handle_generator"-function is called. 
			case set_gen:
				handle_generator();
				tilstand = scope;
				break;
		
		
			//"Start" recieved from LabView. The SPI-package is updated with an increasing byte value from 0-255. 
			//The package is transmitted with each increment and at least one ADC-sample is completed
			//The samples are gathered and transmitted via UART to LabView at the end of the 0-255 loop. 
			case BodePlot:
				spi_package[1] = SPI_FREQ;
				for(int i = 0; i<=255;i++){//adjust frequency 
					spi_package[2]= sampleBuffer[adc_user][bufferCounter];
					spi_package[3]=calcSPIchecksum(spi_package,SPI_DATA_SIZE);
					if(!DEVEL){
						transmit_Spi_pkg(spi_package,SPI_DATA_SIZE);	
					}
				
					//Wait to make sure ADC sample is taken at target frequency TODO
					_delay_ms(10);

					bodeBuffer[i+HEADER_SIZE] = sampleBuffer[adc_user][HEADER_SIZE+bufferCounter];
				}
				transmitUARTPackage(bodeBuffer, BODE_TYPE, 255);
				tilstand = scope;
				break;
		}	
	}
}




// ================================================
// Functions
// ================================================
void setup(){
		
	//UART
	init_uart_interrupt1(UBBR_D);

	//Timers
	init_timer1();
	
	//SPI
	init_spi_master();
	
	//ADC
 	init_adc(1);
 	startADCSampling(ADC_CHANNEL);

	//SPI
	init_spi_master();

	//Interrupt
	sei();
	
	//Set initial record length and sample rate
	recordLength = 500;
	setSampleRate(10000);
	
	//Reset LabView generator window
	resetLabview();
	
	//OLED-display (debugging)
	if(DEVEL){
		_i2c_address = 0X78;
		I2C_Init();
		InitializeDisplay();
		print_fonts();
		clear_display();
	}
}

//Funktion som returnerer tilstande p� baggrund af den l�ste Type modtaget i telemetry. 
enum tilstande handle_type(char input){
	if(uart_type==0x01){
		return set_gen;
	}
	if(uart_type==0x02){
		return set_sample;
	}
	if(uart_type==0x03){
		return BodePlot;
	}
	else return scope;
}


void debug_print(char input, int value){
	char temp[100]={0};
	sprintf(temp,"%u",input);
	sendStrXY(temp,value,13);
}


//Funktion, som skelner mellem tastetryk i generator-fanen.
//BTN-byte og SW-byte gemmes i hver sin variabel. 
void handle_generator(){
	BTN = data[5];
	SW = data[6];

//Tjek v�rdien af BTN	
	switch(BTN)
	{

		//ENTER: konstru�r en SPI-datapakke med det tilsvarende dataindhold.
		//Ligeledes opdateres telecommand-pakken.
		case ENTER: 
			if (param == shape_s){
				telecommand[1+HEADER_SIZE] = SW;
				transmitUARTPackage(telecommand,GENERATOR_TYPE,4);
				spi_package[1]=SPI_SHAPE;
				spi_package[2]=SW;	
				spi_package[3]=calcCheckSum(spi_package,SPI_DATA_SIZE-1);
				if(!DEVEL){
					transmit_Spi_pkg(spi_package,SPI_DATA_SIZE);
				}
			}
			
			else if (param == amplitude_s){
				telecommand[2+HEADER_SIZE] = SW;
				transmitUARTPackage(telecommand,GENERATOR_TYPE,4);
				spi_package[1]=SPI_AMP;
				spi_package[2]=SW;
				spi_package[3]=calcCheckSum(spi_package,SPI_DATA_SIZE-1);
				if(!DEVEL){
					transmit_Spi_pkg(spi_package,SPI_DATA_SIZE);
				}
			}
			
			else if (param == freq_s){
				telecommand[3+HEADER_SIZE] = SW;
				transmitUARTPackage(telecommand,GENERATOR_TYPE,4);	
				spi_package[1]=SPI_FREQ;
				spi_package[2]=SW;
				spi_package[3]=calcCheckSum(spi_package,SPI_DATA_SIZE-1);
				if(!DEVEL){
					transmit_Spi_pkg(spi_package,SPI_DATA_SIZE);
				}
				
			}
		break;

		//SELECT: Tilstandsloop, som gemmer v�rdien af den nuv�rende valgte parameter (amplitude, frekvens eller shape). 
		//Opdater telecommandpakken med den tilsvarende v�rdi. 
		case SELECT:
			switch(param){
				case shape_s:
				telecommand[HEADER_SIZE]=1;
				transmitUARTPackage(telecommand,GENERATOR_TYPE,4);
				param = amplitude_s;
				break;
				
				case amplitude_s:
				telecommand[HEADER_SIZE]=2;
				transmitUARTPackage(telecommand,GENERATOR_TYPE,4);
				param = freq_s;
				break;
				
				case freq_s:
				telecommand[HEADER_SIZE] = 0;
				transmitUARTPackage(telecommand,GENERATOR_TYPE,4);
				param = shape_s;
				break;
			}
			break;

		//Run/Stop: Toggle stop-char mellem de to start/stop v�rdier. Opdat�r SPI-pakken med tilh�rende v�rdi.		
		case START_STOP:
			TOGGLEBIT(stop,0);
			if(CHKBIT(stop,0)) CLRBIT(ADCSRA,ADEN);
			else SETBIT(ADCSRA,ADEN);
			spi_package[1] = stop;
			spi_package[2] = 0;
			spi_package[3]=calcCheckSum(spi_package,SPI_DATA_SIZE-1);
			transmit_Spi_pkg(spi_package,SPI_DATA_SIZE);
			break;

		//RESET: Toggle reset-byte og opdater dette i spi-package. 		
		case RESET:
			spi_package[1] = RESET_SPI;
			spi_package[2] = 0;
			spi_package[3]=calcCheckSum(spi_package,SPI_DATA_SIZE-1);
			transmit_Spi_pkg(spi_package,SPI_DATA_SIZE);
			resetLabview();
			break;
			
	}
}



//Tilstandsmaskine, som genneml�ber datapakkens bestandele. 
void evaluate_recieve(){
	switch(state){
		
		//Tjek om f�rste karakter er 0x55 og skift tilstand hvis sand. 
		case sync1:
			if(UARTBuffer[uart_cnt_rx] == 0x55){
			data[uart_cnt_rx]=UARTBuffer[uart_cnt_rx];
			uart_cnt_rx++;
			state = sync2;
			}
			break;
		
		//Tjek om anden karakter er 0xAA og skift tilstand hvis sand. Ellers skift til tilstand Sync 1 igen.
		case sync2:
			if(UARTBuffer[uart_cnt_rx]==0xAA){
				data[uart_cnt_rx]=UARTBuffer[uart_cnt_rx];
				uart_cnt_rx++;
				state = Length;
				}
			else{
				state = sync1;
				uart_cnt_rx=0;
			}
			break;
		
		
		//L�s l�ngden af den modtagne pakke (byte1)
		case Length:
			data[uart_cnt_rx]=UARTBuffer[uart_cnt_rx];
			Len = (UARTBuffer[uart_cnt_rx++]<<8);
			state = Length2;
			break;
		
		//L�s l�ngden af den modtagne pakke (byte2)
		case Length2:
		data[uart_cnt_rx]=UARTBuffer[uart_cnt_rx];
		Len = Len + (UARTBuffer[uart_cnt_rx++]);
		state = Type;			
		break;
		
		//L�s type-byten og gem den i en char. 
		case Type:
		data[uart_cnt_rx]=UARTBuffer[uart_cnt_rx];
		uart_type = UARTBuffer[uart_cnt_rx++];
		state = ReadData;
		break;
		
		//L�s data, hvis der findes databytes i pakken og gem det i data[]  IF ELSE
		case ReadData:
				if(uart_cnt_rx < Len-2){
				data[uart_cnt_rx]=UARTBuffer[uart_cnt_rx];
				uart_cnt_rx++;
			break;
			}
				
		//Hvis hele datapakken er l�st og gemt skiftes tilstand. 	
			else{ //(uart_cnt_rx==(compare))
				state = Check1;
				
			}
			
		//L�s f�rste checksum-byte
		case Check1:
			checksum_val = (UARTBuffer[uart_cnt_rx++]<<8);
			state = Check2;
			break;
		
		//L�s anden checksum-byte og kontroller om den nye int checksum_val == 0x000
		case Check2:
			checksum_val = checksum_val | (UARTBuffer[uart_cnt_rx]);
			if(checksum_val==calcCheckSum(data,Len-2)){
				uart_cnt_rx=0;
				checksum_flag=0;
				Len=0;
				state = sync1;
				}	
			else{
				checksum_flag=1;
				uart_cnt_rx=0;
				state = sync1;
				Len=0;
			}	
			break;
	}		
}



// ================================================
// ADC
// ================================================

//Calculate and set compare match value for ADC Auto Trigger Source based on target ADC sample rate value.
void setSampleRate(unsigned int sampleRate){
	unsigned char compareValue = (F_CPU/(2*sampleRate))/ADC_TRIG_SRC_PS-1;
	OCR1A = compareValue;
	OCR1B = compareValue;
}



// ================================================
// Serial
// ================================================

void transmitUARTPackage(char * data, unsigned char type, unsigned int dataSize){
		
		//Construct package		
		data[0] = 0x55;
		data[1] = 0xAA;
		data[2] = (dataSize+PADDING_SIZE) >> 8;
		data[3] = (dataSize+PADDING_SIZE);
		data[4] = type;
		
		unsigned int checksum = calcCheckSum(data, dataSize+HEADER_SIZE);
		data[HEADER_SIZE+dataSize] = checksum >> 8;
		data[HEADER_SIZE+dataSize+1] = checksum & 0xFF;
		
		transmitStrUSART(data,dataSize+PADDING_SIZE);
		
}

void transmitADCSample(char * data, unsigned char type, unsigned int dataSize){
	
	//Construct package
	sampleBuffer[uart_user][0] = 0x55;
	sampleBuffer[uart_user][1] = 0xAA;
	sampleBuffer[uart_user][2] = (dataSize+PADDING_SIZE) >> 8;
	sampleBuffer[uart_user][3] = (dataSize+PADDING_SIZE);
	sampleBuffer[uart_user][4] = type;
	
	unsigned int checksum = calcCheckSum(data, dataSize+HEADER_SIZE);
	sampleBuffer[uart_user][HEADER_SIZE+dataSize] = checksum >> 8;
	sampleBuffer[uart_user][HEADER_SIZE+dataSize+1] = checksum & 0xFF;

	transmitStrUSART(data,dataSize+PADDING_SIZE);
	
}

void resetLabview(){
	param = shape_s;
	state = sync1;
	memset(telecommand,0,11);
	transmitUARTPackage(telecommand,GENERATOR_TYPE,4);
}



// ================================================
// Service Routines
// ================================================

//Service routine for ADC sample ready
ISR(ADC_vect){
	sampleBuffer[adc_user][HEADER_SIZE+bufferCounter++] = ADCH;
	
	if(bufferCounter >= recordLength){
		adc_flag = 1;
		if(uart_tx_flag==0){
			adc_user = !adc_user;
			uart_user = !uart_user;
		}
		bufferCounter = 0;
	}
	
	//Overflow handling
	if(bufferCounter > SAMPLE_BUF){
		bufferCounter = 0;
	}
}


//Service routine for Timer1 Compare B
ISR (TIMER1_COMPB_vect) {
}


//Service routine for UART receive vector
ISR(USART1_RX_vect){
	UARTBuffer[uart_cnt_rx] = UDR1;
	flag_uart_rx = 1;
	evaluate_recieve();
}



// ================================================
// Utils
// ================================================

//Calculate checksum 
//Handles ZERO16 and LRC8 checksums
//Returns 16-bit checksum
unsigned int calcCheckSum(char * data, unsigned int pkgSize){
	
	//ZERO 16 checksum
	if(CKSUM_TYPE == 0){
		return 0x0000;
	}
	//LRC8 checksum
	else if(CKSUM_TYPE==1){
		unsigned char checkSum = 0;
		for(int i = 0; i < pkgSize; i++){
			checkSum ^= data[i];
		}
		return 0x00 | checkSum;
	}
}

//Calculates resulting samlerate based on record length. 
//Used to compensate for UART baud bottleneck
unsigned int sampleRate_comp(unsigned int record_length){
	unsigned long dividend = BAUD_EFFECT*record_length;
	unsigned int samplerate = dividend/(record_length+PADDING_SIZE);
	return samplerate;
}

//Prints a char to OLED display
void debug_print_char(char input){
	if(DEVEL){
		char temp[100] = {0};
		sprintf(temp,"%u",input);
		sendStrXY(temp, 0,0);
	}
}

//Prints an int to OLED display
void debug_print_int(int input){
	if(DEVEL){
		char temp[100] = {0};
		sprintf(temp,"%u",input);
		sendStrXY(temp, 4,8);
	}
}
