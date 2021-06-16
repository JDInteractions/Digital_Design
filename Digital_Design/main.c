/* Digital_Design.c
 *
 * Created: 04-06-2021 13:21:47
 * Author : Jan & Lars
 */ 


#include "main.h"

 



volatile char flag_uart_rx = 0;
volatile unsigned int uart_cnt_rx = 0;
char UARTBuffer[datasize] = {0};
unsigned int Len = 0;
char uart_type = 0;
unsigned int checksum_val = 0;
char checksum_flag = 0;
char rec_complete = 0;
char data[datasize] = {0};
char OLED_buffer[20]={0};
unsigned int RL = 0;
unsigned int S_Rate, S_rate_max = 0;
char SW = 0;
char BTN = 0;
unsigned int compare = 0;
char stop,reset = 0;
char telecommand[GEN_PKG+PADDING_SIZE]={0};
char bodeBuffer[BODE_PKG+PADDING_SIZE]={0};	
char spi_package[2]={0};
enum states {sync1, wait, sync2, Length, Length2, Type, ReadData, CS1, CS2};
enum tilstande {scope, set_sample, set_gen, BodePlot};
enum parameter {shape_s,amplitude_s,freq_s};
char param = shape_s;
char state = sync1;
char tilstand = scope;
char test = 0xff;
char test2  = 0xfe;
char test3 =0xfd;

 
volatile char uart_tx_flag = 0;
volatile int uart_cnt_tx = 1;

unsigned int dataSizeTX = 0;
volatile int uart_cnt = 0;	

//Holds latest adc sample -> read on adc interrupt
volatile char adc_flag = 0; 
unsigned int bufferCounter = 0;
char sampleBuffer[2][SAMPLE_BUF] = {{0},{0}};
int adc_user = 0;
int uart_user = 1;		//TODO char??

//unsigned int sampleRateTarget = 1000;
unsigned int recordLength = 500;	
unsigned int nextRecordLenght = 0;

int main(void){ 
    
	setup();
	setSampleRate(10000);
    
    while (1){
		
	//Main tilstandsmaskine
	//Reagerer p� uart-receive-flag. Scope er begyndelsestilstanden og herfra kaldes funktionen Handle_type.
	//Dermed skiftes der tilstand baseret p� den modtagne uart-type. 
	switch(tilstand){
		
		//Grundtilstand. Tjek for uart-flag. skift tilstand baseret p� uart-type. 
		case scope:
		
 			if(adc_flag){
	 			transmitADCSample(&sampleBuffer[uart_user][0], SCOPE_TYPE, recordLength);
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
			if(recordLength < 47){
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
			spi_package[0] = 7;
			for(int i = 0; i<=255;i++){//adjust frequency 
				spi_package[1]= sampleBuffer[adc_user][bufferCounter];
				
				//send SPI package
				
				//Wait to make sure ADC sample is taken at target frequency TODO
				_delay_ms(10);

				bodeBuffer[i+HEADER_SIZE] = sampleBuffer[adc_user][HEADER_SIZE+bufferCounter];
			}
			transmitUARTPackage(bodeBuffer, BODE_TYPE, 255);
			tilstand = scope;
			break;
	}
		
		
		
		for(int i=5;i<15;i++){
			OLED_buffer[i]=data[i]+0x30;
		}		
		sendStrXY(OLED_buffer,4,5);
		debug_print(uart_type,5);
		debug_print(rec_complete,6);
		debug_print(checksum_flag,7);
		//sendStrXY("Max_rate:",4,0);
 		sendStrXY("Type:",5,0);
 		sendStrXY("Rec_comp:",6,0);
 		sendStrXY("Checksum_f:",7,0);

		
		
	
	}
}

// ================================================
// Service Routines
// ================================================

//Service routine for ADC sample ready
ISR(ADC_vect){
	sampleBuffer[adc_user][HEADER_SIZE+bufferCounter++] = ADCH;
	
	if(bufferCounter >= recordLength){
		adc_flag = 1;
		
		adc_user = !adc_user;
		uart_user = !uart_user;
			
		bufferCounter = 0;
	}
	
	//Overflow
	//if(bufferCounter[adc_user][0] > SAMPLE_BUF){
	//bufferCounter[adc_user][0] = 0;
	//}
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
// Functions
// ================================================
void setup(){
		
	//UART
	init_uart_interrupt1(UBBR_D);

	//Timers
	init_timer1();
	
	//ADC
	init_adc(1);
	startADCSampling(ADC_CHANNEL);

	//Interrupt
	sei();
	
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
				spi_package[0]=4;
				spi_package[1]=SW;	
			}
			
			else if (param == amplitude_s){
				telecommand[2+HEADER_SIZE] = SW;
				transmitUARTPackage(telecommand,GENERATOR_TYPE,4);
				spi_package[0]=5;
				spi_package[1]=SW;
			}
			
			else if (param == freq_s){
				telecommand[3+HEADER_SIZE] = SW;
				transmitUARTPackage(telecommand,GENERATOR_TYPE,4);	
				spi_package[0]=7;
				spi_package[1]=SW;
				
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
			spi_package[0] = stop;
			spi_package[1] = 0;
			//send stop-byte on SPI
			break;

//RESET: Toggle reset-byte og opdater dette i spi-package. 		
		case RESET:
			
			spi_package[0] = RESET_SPI;
			spi_package[1] = 0;
			resetLabview();
			//send reset_byte + 0data SPI;
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
				state = Length;
				uart_cnt_rx++;
				}
			else{
				state = sync1;
				uart_cnt_rx++;
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
		compare = Len-2;
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
		//debug_print_char(test);
		if(Len>7){
			if(uart_cnt_rx < (compare)){
				data[uart_cnt_rx]=UARTBuffer[uart_cnt_rx];
				uart_cnt_rx++;
			}
				
		//Hvis hele datapakken er l�st og gemt skiftes tilstand. 	
			if(uart_cnt_rx==(compare)){
				state = CS1;
				uart_cnt_rx++;
			}
		}
		else state = CS1;
		//debug_print_char(test2);
		break;
		
		//L�s f�rste checksum-byte
		case CS1:
		checksum_val = (UARTBuffer[uart_cnt_rx++]<<8);
		state = CS2;
		break;
		
		//L�s anden checksum-byte og kontroller om den nye int checksum_val == 0x000
		case CS2:
		
		checksum_val = checksum_val | (UARTBuffer[uart_cnt_rx++]);
		if(checksum_val==calcCheckSum(data,compare)){
			rec_complete=1;	
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
// Utils
// ================================================

int calcCheckSum(char * data, unsigned int dataSize){
	
	if(CKSUM_TYPE == 0){
		return 0x0000;
	}
	else if(CKSUM_TYPE==1){
		char checkSum = data[0];
		for(int i = 1; i < dataSize+HEADER_SIZE; i++){
			checkSum ^= data[i];
		}
		return 0x00 | checkSum;
	}
}


void debug_print_char(char input){
	if(DEVEL){
		char temp[100] = {0};
		sprintf(temp,"%u",input);
		sendStrXY(temp, 0,0);
	}
}

void debug_print_int(int input){
	if(DEVEL){
		char temp[100] = {0};
		sprintf(temp,"%u",input);
		sendStrXY(temp, 0,0);
	}
}	

unsigned int sampleRate_comp(unsigned int input){
	unsigned long dividend = BAUD_EFFECT*input;
	unsigned int samplerate = dividend/(input+PADDING_SIZE);
	return samplerate;
}

// ================================================
// ADC
// ================================================


//Calculate and set compare match value for ADC Auto Trigger Source based on target ADC sample rate value.
void setSampleRate(unsigned int sampleRate){
	int compareValue = (F_CPU/(2*sampleRate))/ADC_TRIG_SRC_PS-1;
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
		
		int checksum = calcCheckSum(data, dataSize);
		data[HEADER_SIZE+dataSize] = checksum << 8;
		data[HEADER_SIZE+dataSize+1] = checksum;
			
		//UDR1 = UARToutputBuffer[uart_cnt_tx++];
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
	
	int checksum = calcCheckSum(data, dataSize);
	sampleBuffer[uart_user][HEADER_SIZE+dataSize] = checksum << 8;
	sampleBuffer[uart_user][HEADER_SIZE+dataSize+1] = checksum;
	
	//UDR1 = UARToutputBuffer[uart_cnt_tx++];
	for(int i = 0; i < recordLength+PADDING_SIZE; i++){
		putCharUSART(sampleBuffer[uart_user][i]);
	}
	
}

void resetLabview(){
	param = shape_s;
	state = sync1;
	memset(telecommand,0,GEN_PKG);
	transmitUARTPackage(telecommand,GENERATOR_TYPE,4);
}