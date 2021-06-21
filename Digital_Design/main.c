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
char telemetryPkg[TELEMETRY_SIZE] = {0};				//UART RX
char telecommandPkg[GEN_PKG+PADDING_SIZE]={0};	//UART TX
	
//SPI
char SPIBufferTx[SPI_SIZE]={0x55,0,0,0};
char stop = SPI_STOP;

//BODE PLOT	
char bodeBuffer[BODE_PKG+PADDING_SIZE]={0};

unsigned int Len = 0;

unsigned int checksum_val = 0;
	
//State machines
enum tilstande {scope, set_sample, set_gen, BodePlot};
char tilstand = scope;
enum states {sync1, wait, sync2, Length, Length2, Type, ReadData, Check1, Check2};
char state = sync1;
enum parameter {shape_s,amplitude_s,freq_s};
char param = shape_s;
char uart_type = 0;

//ADC sampling (double buffer)
char sampleBuffer[2][SAMPLE_BUF] = {{0},{0}};
unsigned int recordLength = 0;	
unsigned int sampleCounter = 0;
int adc_user = 0;
int uart_user = 1;



int main(void){ 
    
	setup();
    SETBIT(DDRH, PH4); //TODO
	SETBIT(DDRH, PH3);
	
    while (1){
	
	//Main tilstandsmaskine
	//Reagerer på uart-receive-flag. "Scope" er begyndelsestilstanden og herfra kaldes funktionen Handle_type.
	//Dermed skiftes der tilstand baseret på den modtagne uart-type. 
	switch(tilstand){
		unsigned int S_Rate, S_rate_max = 0;
		//Grundtilstand. Tjek for uart-flag. skift tilstand baseret på uart-type. 
		case scope:
		
 			if(adc_flag){
				//transmitcompleteflag = 1; //TODO
	 			transmitADCSample(&sampleBuffer[uart_user][0], SCOPE_TYPE, recordLength);
				//transmitcompleteflag = 0;
				 adc_flag = 0;
				 
			}
			if(flag_uart_rx==1){
				flag_uart_rx=0;
				tilstand = handle_type(uart_type);
			}
			break;
		
		//"Send" er modtaget. Opdater S_rate og RL.
		case set_sample:
			S_Rate = ((unsigned int)telemetryPkg[5]<<8)|(unsigned int)telemetryPkg[6];
			recordLength = ((unsigned int)telemetryPkg[7]<<8)|(unsigned int)telemetryPkg[8];
			setSampleRate(S_Rate);
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
				
				//Wait to make sure ADC sample is taken at target frequency 
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


//Returns next state based on received telemetry type
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



//Handles and decodes button press in generator tab
void handle_generator(){
	char BTN = 0;
	char SW = 0;
	BTN = telemetryPkg[5];
	SW = telemetryPkg[6];

	//Get type of button press
	switch(BTN)
	{

		//ENTER: Construct SPI-datapackage with SW value.
		//Updates telecommand package to update Laview generator window.
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

		//SELECT: State loop, saves value of current parameter(amplitud, freq, shape)
		//Updates telecommand package to update Laview generator window.
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

		//Toggles stop-char between start/stop. Also updates SPI-package with start/stop value	
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
	
		//Toggle reset-byte and update SPI-package
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



//Decodes telemetry package 
void readTelemetry(){
	
	switch(state){
		
		//Check if first character is 0x55 and set next state if true
		case sync1:
			if(telemetryPkg[uart_cnt_rx] == 0x55){
			uart_cnt_rx++;
			state = sync2;
			}
			break;
		
		//Check if second character is 0xAA, set next state if true. Else revert back to state sync1
		case sync2:
			if(telemetryPkg[uart_cnt_rx]==0xAA){
				uart_cnt_rx++;
				state = Length;
				}
			else{
				state = sync1;
				uart_cnt_rx=0;
			}
			break;
		
		
		//Read first length byte
		case Length:
			Len = (telemetryPkg[uart_cnt_rx++]<<8);
			state = Length2;
			break;
		
		//Read second length byte
		case Length2:
			Len = Len + (telemetryPkg[uart_cnt_rx++]);
			state = Type;			
			break;
		
		//Get type of package
		case Type:
			uart_type = telemetryPkg[uart_cnt_rx++];
			state = ReadData;
			break;
		
		//Read data content of telemetry package
		case ReadData:
			if(uart_cnt_rx < Len-2){
				uart_cnt_rx++;
			break;
			}
			//Go to next state if all data is read
			else{
				state = Check1;
			}
			
		//Read first checksum byte
		case Check1:
			checksum_val = (telemetryPkg[uart_cnt_rx++]<<8);
			state = Check2;
			break;
		
		//Read second checksum byte and evaluate checksum
		case Check2:
			checksum_val = checksum_val | (telemetryPkg[uart_cnt_rx]);
			if(checksum_val==calcCheckSum(telemetryPkg,Len-2)){
				//Checksum OK
			}
			else{
				//Invalid checksum
			}
			uart_cnt_rx=0;
			state = sync1;
			Len=0;
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

//Transmit UART package
//Appends length, type and checksum. Data is already in buffer upon function call.
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

//Transmit ADC sample over UART. 
//Note, only handles 2 dimensional ADC sample buffer
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
	//Save new sample to sample buffer
	sampleBuffer[adc_user][HEADER_SIZE+sampleCounter++] = ADCH;
	
	//Enable adc flag if full record length is read
	if(sampleCounter >= recordLength){
		adc_flag = 1;
		//if(transmitcompleteflag==0){
		//Swap double buffer indexes 	
		adc_user = !adc_user;
		uart_user = !uart_user;
		//}
		//Reset buffer counter
		sampleCounter = 0;
	}
}

//Service routine for UART receive vector
ISR(USART1_RX_vect){
	telemetryPkg[uart_cnt_rx] = UDR1;
	flag_uart_rx = 1;
	readTelemetry();
}

//Service routine for Timer1 Compare B. Needed for Auto Trigger Source
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


//Calculates resulting samplerate based on record length.
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

