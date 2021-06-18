/* Digital_Design.c
 *
 * Created: 04-06-2021 13:21:47
 * Author : Jan Dyrholm Madsen s205072 & Lars Stoltenberg Grove s205035
 */ 


#include "main.h"

//Interrupt flags and counters
volatile char flag_uart_rx = 0;
volatile unsigned int uart_cnt_rx = 0;
volatile char transmitcompleteflag = 0;
volatile char adc_flag = 0;

//UART
char telemetryPkg[datasize] = {0};				//UART RX
char telecommandPkg[GEN_PKG+PADDING_SIZE]={0};	//UART TX
	
//SPI
char SPIBufferTx[4]={0x55,0,0,0};
	
	
unsigned int Len = 0;
char uart_type = 0;
unsigned int checksum_val = 0;

char data[datasize] = {0};
unsigned int RL = 0;
unsigned int S_Rate, S_rate_max = 0;
char SW = 0;
char BTN = 0;
char reset = 0;
char stop = SPI_STOP;

char bodeBuffer[BODE_PKG+PADDING_SIZE]={0};	

	
	
	
enum tilstande {scope, set_sample, set_gen, BodePlot};
char tilstand = scope;
enum states {sync1, wait, sync2, Length, Length2, Type, ReadData, Check1, Check2};
char state = sync1;
enum parameter {shape_s,amplitude_s,freq_s};
char param = shape_s;



//ADC sampling (double buffer)
char sampleBuffer[2][SAMPLE_BUF] = {{0},{0}};
unsigned int recordLength = 0;	
unsigned int sampleCounter = 0;
int adc_user = 0;
int uart_user = 1;		//TODO char??



int main(void){ 
    
	setup();
	//setSampleRate(10000);
    
    while (1){
	
	//Main tilstandsmaskine
	//Reagerer p� uart-receive-flag. Scope er begyndelsestilstanden og herfra kaldes funktionen Handle_type.
	//Dermed skiftes der tilstand baseret p� den modtagne uart-type. 
	switch(tilstand){
		
		//Grundtilstand. Tjek for uart-flag. skift tilstand baseret p� uart-type. 
		case scope:
		
 			if(adc_flag){
				transmitcompleteflag = 1;
	 			transmitADCSample(&sampleBuffer[uart_user][0], SCOPE_TYPE, recordLength);
				transmitcompleteflag = 0;
				 adc_flag = 0;
				 
			}
			if(flag_uart_rx==1){
				flag_uart_rx=0;
				tilstand = handle_type(uart_type);
			}
			break;
		
		//"Send" er modtaget. Opdat�r S_rate og RL.
		case set_sample:
			S_Rate = ((unsigned int)data[5]<<8)|(unsigned int)data[6];
			RL = ((unsigned int)data[7]<<8)|(unsigned int)data[8];
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
		
		//Knaptryk fra "Generator" modtaget. Funktionen handle_generator behandler tastetryk. 
		case set_gen:
			handle_generator();
			tilstand = scope;
			break;
		
		//"Start" er modtaget. Spi-pakken-opdateres og der loopes med increments af 1Hz.
		case BodePlot:
			SPIBufferTx[1] = SPI_FREQ;
			for(int i = 0; i<=255;i++){//adjust frequency 
				SPIBufferTx[2]= i;
				SPIBufferTx[3]=calcCheckSum(SPIBufferTx,SPI_DATA_SIZE-1);
				if(!DEVEL){
					transmit_Spi_pkg(SPIBufferTx,SPI_DATA_SIZE);	
				}
				
				//Wait to make sure ADC sample is taken at target frequency TODO
				_delay_ms(10);

				bodeBuffer[i+HEADER_SIZE] = sampleBuffer[adc_user][HEADER_SIZE+sampleCounter];
			}
			transmitUARTPackage(bodeBuffer, BODE_TYPE, 255);
			tilstand = scope;
			break;
	}
		
		_delay_ms(10);
	
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

	//Interrupt
	sei();
	
	//Set initial record length and sample rate
	recordLength = 500;
	setSampleRate(10000);
		
	//Reset LabView generator window
	resetLabview();
	
	//OLED-display
	_i2c_address = 0X78;
	I2C_Init();
	InitializeDisplay();
	print_fonts();
	clear_display();
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
				telecommandPkg[1+HEADER_SIZE] = SW;
				transmitUARTPackage(telecommandPkg,GENERATOR_TYPE,4);
				SPIBufferTx[1]=SPI_SHAPE;
				SPIBufferTx[2]=SW;	
				SPIBufferTx[3]=calcCheckSum(SPIBufferTx,SPI_DATA_SIZE-1);
				if(!DEVEL){
					transmit_Spi_pkg(SPIBufferTx,SPI_DATA_SIZE);
				}
			}
			
			else if (param == amplitude_s){
				telecommandPkg[2+HEADER_SIZE] = SW;
				transmitUARTPackage(telecommandPkg,GENERATOR_TYPE,4);
				SPIBufferTx[1]=SPI_AMP;
				SPIBufferTx[2]=SW;
				SPIBufferTx[3]=calcCheckSum(SPIBufferTx,SPI_DATA_SIZE-1);
				if(!DEVEL){
					transmit_Spi_pkg(SPIBufferTx,SPI_DATA_SIZE);
				}
			}
			
			else if (param == freq_s){
				telecommandPkg[3+HEADER_SIZE] = SW;
				transmitUARTPackage(telecommandPkg,GENERATOR_TYPE,4);	
				SPIBufferTx[1]=SPI_FREQ;
				SPIBufferTx[2]=SW;
				SPIBufferTx[3]=calcCheckSum(SPIBufferTx,SPI_DATA_SIZE-1);
				if(!DEVEL){
					transmit_Spi_pkg(SPIBufferTx,SPI_DATA_SIZE);
				}
				
			}
		break;

//SELECT: Tilstandsloop, som gemmer v�rdien af den nuv�rende valgte parameter (amplitude, frekvens eller shape). 
//Opdater telecommandpakken med den tilsvarende v�rdi. 
		case SELECT:
			switch(param){
				case shape_s:
				telecommandPkg[HEADER_SIZE]=1;
				transmitUARTPackage(telecommandPkg,GENERATOR_TYPE,4);
				param = amplitude_s;
				break;
				
				case amplitude_s:
				telecommandPkg[HEADER_SIZE]=2;
				transmitUARTPackage(telecommandPkg,GENERATOR_TYPE,4);
				param = freq_s;
				break;
				
				case freq_s:
				telecommandPkg[HEADER_SIZE] = 0;
				transmitUARTPackage(telecommandPkg,GENERATOR_TYPE,4);
				param = shape_s;
				break;
			}
			break;

//Run/Stop: Toggle stop-char mellem de to start/stop v�rdier. Opdat�r SPI-pakken med tilh�rende v�rdi.		
		case START_STOP:
			TOGGLEBIT(stop,0);
			if(CHKBIT(stop,0)) CLRBIT(ADCSRA,ADEN);
			else SETBIT(ADCSRA,ADEN);
			SPIBufferTx[1] = stop;
			SPIBufferTx[2] = 0;
			SPIBufferTx[3]=calcCheckSum(SPIBufferTx,SPI_DATA_SIZE-1);
			if(!DEVEL){
				transmit_Spi_pkg(SPIBufferTx,SPI_DATA_SIZE);
			}
			break;

//RESET: Toggle reset-byte og opdater dette i spi-package. 		
		case RESET:
			SPIBufferTx[1] = RESET_SPI;
			SPIBufferTx[2] = 0;
			SPIBufferTx[3]=calcCheckSum(SPIBufferTx,SPI_DATA_SIZE-1);
			if(!DEVEL){
				transmit_Spi_pkg(SPIBufferTx,SPI_DATA_SIZE);
			}
			resetLabview();
			break;
			
	}
}



//Tilstandsmaskine, som genneml�ber datapakkens bestandele. 
void readTelemetry(){
	
	switch(state){
		
		//Tjek om f�rste karakter er 0x55 og skift tilstand hvis sand. 
		case sync1:
			if(telemetryPkg[uart_cnt_rx] == 0x55){
			data[uart_cnt_rx]=telemetryPkg[uart_cnt_rx];
			uart_cnt_rx++;
			state = sync2;
			}
			break;
		
		//Tjek om anden karakter er 0xAA og skift tilstand hvis sand. Ellers skift til tilstand Sync 1 igen.
		case sync2:
			if(telemetryPkg[uart_cnt_rx]==0xAA){
				data[uart_cnt_rx]=telemetryPkg[uart_cnt_rx];
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
			data[uart_cnt_rx]=telemetryPkg[uart_cnt_rx];
			Len = (telemetryPkg[uart_cnt_rx++]<<8);
			state = Length2;
			break;
		
		//L�s l�ngden af den modtagne pakke (byte2)
		case Length2:
		data[uart_cnt_rx]=telemetryPkg[uart_cnt_rx];
		Len = Len + (telemetryPkg[uart_cnt_rx++]);
		state = Type;			
		break;
		
		//L�s type-byten og gem den i en char. 
		case Type:
		data[uart_cnt_rx]=telemetryPkg[uart_cnt_rx];
		uart_type = telemetryPkg[uart_cnt_rx++];
		state = ReadData;
		break;
		
		//L�s data, hvis der findes databytes i pakken og gem det i data[]  IF ELSE
		case ReadData:
				if(uart_cnt_rx < Len-2){
				data[uart_cnt_rx]=telemetryPkg[uart_cnt_rx];
				uart_cnt_rx++;
			break;
			}
				
		//Hvis hele datapakken er l�st og gemt skiftes tilstand. 	
			else{ //(uart_cnt_rx==(compare))
				state = Check1;
				
			}
			
		//L�s f�rste checksum-byte
		case Check1:
		checksum_val = (telemetryPkg[uart_cnt_rx++]<<8);
		state = Check2;
		break;
		
		//L�s anden checksum-byte og kontroller om den nye int checksum_val == 0x000
		case Check2:
		checksum_val = checksum_val | (telemetryPkg[uart_cnt_rx]);
		if(checksum_val==calcCheckSum(data,Len-2)){
			uart_cnt_rx=0;
			Len=0;
			state = sync1;
			}	
		else{
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
	unsigned int compareValue = (F_CPU/(2*sampleRate))/ADC_TRIG_SRC_PS-1;
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
			
		for(int i = 0; i < dataSize+PADDING_SIZE; i++){
			putCharUSART(data[i]);
		}
		
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
	
	for(int i = 0; i < dataSize+PADDING_SIZE; i++){
		putCharUSART(sampleBuffer[uart_user][i]);
	}
}



// ================================================
// Service Routines
// ================================================

//Service routine for ADC sample ready
ISR(ADC_vect){
	sampleBuffer[adc_user][HEADER_SIZE+sampleCounter++] = ADCH;
	
	if(sampleCounter >= recordLength){
		adc_flag = 1;
		if(transmitcompleteflag==0){
			adc_user = !adc_user;
			uart_user = !uart_user;
		}
		sampleCounter = 0;
	}
	
	//Overflow
	if(sampleCounter > SAMPLE_BUF){
		sampleCounter = 0;
	}
}

//Service routine for UART receive vector
ISR(USART1_RX_vect){
	telemetryPkg[uart_cnt_rx] = UDR1;
	flag_uart_rx = 1;
	readTelemetry();
}

//Service routine for Timer1 Compare B
ISR (TIMER1_COMPB_vect) {
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
	//LRC8 checksumcalcCheckSum
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


//Reset LabView generator window to initial state
void resetLabview(){
	param = shape_s;
	state = sync1;
	memset(telecommandPkg,0,11);
	transmitUARTPackage(telecommandPkg,GENERATOR_TYPE,4);
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


void debug_print(char input, int value){
	char temp[100]={0};
	sprintf(temp,"%u",input);
	sendStrXY(temp,value,13);
}

