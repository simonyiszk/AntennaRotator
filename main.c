/****************************/
/** Antenna rotator control**/
/**                        **/
/** 2019,HA5KFU radio club **/
/****************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define _CMD(x) if(!memcmp(ln, x, sizeof(x)-1))

/****************************/
/** Low-level HW functions **/
/****************************/

void init_uart(){
	UBRRH = 0;	
	UBRRL = 51;
	UCSRB = (1<<RXEN)|(1<<TXEN);
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
}

void adc_init(){
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
}

uint16_t adc_read(uint8_t ch){
	ch &= 7;
	ADMUX = (ADMUX & 0xF8) | ch;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return (ADC);
}

void uart_readln(char* buf, size_t maxlen){
	for(size_t i=0;i<maxlen;i++){
		while(! (UCSRA & (1<<RXC)));
		if((buf[i]=UDR)=='\r'){
			buf[i]=0;
			return;
		}
	}
}

void uart_print(const char* str){
	for(size_t i=0;str[i];i++){
		while(!(UCSRA & (1<<UDRE)));
		UDR=str[i];
	}
}

/****************************/
/** Rotator ctl. functions **/
/****************************/

typedef enum {
	RIGHT=2, LEFT, DOWN, UP
} Direction;

typedef enum {
	AZ, EL
} Axis;

void start_rotate(Direction d){
	 PORTC |= (1<<d);
}

void stop_rotate(Axis a){
	if(a==EL)
		PORTC &= ~((1<<UP)|(1<<DOWN));
	else
		PORTC &= ~((1<<LEFT)|(1<<RIGHT));
}

static inline Direction get_dir(Axis a, int16_t sgn){
	switch(a){
	case AZ:
		if(sgn>0)	return RIGHT;
		else		return LEFT;
	default:
		if(sgn>0)	return UP;
		else		return DOWN;
	}
}

//Calibrate ADC to 1000 at max_deg degrees
static inline uint16_t max_deg(Axis a){
	return a==AZ?450:180;
}

double get_axis_deg(Axis a){
	return adc_read(a)*max_deg(a)/1000.0;
}

void set_axis_deg(Axis a, double deg){
	int16_t target=deg*1000/max_deg(a);
	int16_t sgn=target-adc_read(a);
	start_rotate(get_dir(a, sgn));
	//waiting for the sign to change
	while((target-adc_read(a))*sgn>0);
	stop_rotate(a);
}

/****************************/
/** Main loop: UART listen **/
/****************************/

enum _PROTO{
	EASYCOMM2, YAESU,
	DEBUG=-1
} proto=EASYCOMM2;

void comm_Debug(const char* ln){
	_CMD("adc"){
		char tmp[50];
		snprintf(tmp, sizeof(tmp), "%d\r\n", adc_read(ln[3]-'0'));
		uart_print(tmp);
	}else
	_CMD("ld"){
		char ld_id=ln[2]-'0';
		switch(ln[3]){
		case '?':
			uart_print(PORTC & (1<<2<<ld_id)?"0":"1");
			break;
		case '.':
			PORTC ^= (1<<2<<ld_id);
			break;
		case '-':
			PORTC &= ~(1<<2<<ld_id);
			break;
		case '+':
			PORTC |= (1<<2<<ld_id);
			break;
		}
		
	}else
	_CMD("ctl"){
		int16_t deg=atoi(&ln[3]);
		set_axis_deg(EL, deg);
	}
}

void comm_Easy(const char* ln){
	const char* remain;
	//Rotator commands
	_CMD("MR"){
		start_rotate(RIGHT);
		remain=&ln[2];
	}else
	_CMD("ML"){
		start_rotate(LEFT);
		remain=&ln[2];
	}else
	_CMD("MD"){
		start_rotate(DOWN);
		remain=&ln[2];
	}else
	_CMD("MU"){
		start_rotate(UP);
		remain=&ln[2];
	}else
	_CMD("SA"){
		stop_rotate(AZ);
		remain=&ln[2];
	}else
	_CMD("SE"){
		stop_rotate(EL);
		remain=&ln[2];
	}else
	//HACK: positional query
	_CMD("AZ EL"){
		char tmp[50], sAZ[5]={'0'}, sEL[6]={'0'};
		snprintf(tmp, sizeof(tmp), "AZ%s EL%s\r\n", dtostrf(get_axis_deg(AZ), 3, 1, sAZ), dtostrf(get_axis_deg(EL), 3, 1, sEL));
		uart_print(tmp);
		remain=&ln[5];
	}else
	//axis homing
	_CMD("AZ"){
		set_axis_deg(AZ, strtod(&ln[2], (char**)&remain));
	}else
	_CMD("EL"){
		set_axis_deg(EL, strtod(&ln[2], (char**)&remain));
	}else
	//raw I/O
//	_CMD("OP"){
//		//TODO
//		remain=&ln[2];
//	}else
	_CMD("IP"){
		uart_print(ln);
		char ld_id=ln[2]-'0';
		uart_print(PORTC & (1<<2<<ld_id)?",0":",1");
		remain=&ln[3];
	}else
	_CMD("AN"){
		uart_print(ln);
		char tmp[50];
		snprintf(tmp, sizeof(tmp), ",%d\r\n", adc_read(ln[3]-'0'));
		uart_print(tmp);
		remain=&ln[3];
	}else
	//version
	_CMD("VE"){
		uart_print("VE2");
		ln+=2;
	}
	else{
		uart_print("ALerror\r\n");
		return;
	}
	
	//more cmd's in buffer
	if(remain[0]!=0)
		comm_Easy(&remain[1]);
}

void comm_Yaesu(const char* ln){
	char lf=0;
	
	//Rotator commands
	_CMD("R"){
		start_rotate(RIGHT);
	}else
	_CMD("L"){
		start_rotate(LEFT);
	}else
	_CMD("D"){
		start_rotate(DOWN);
	}else
	_CMD("U"){
		start_rotate(UP);
	}else
	_CMD("A"){
		stop_rotate(AZ);
	}else
	_CMD("E"){
		stop_rotate(EL);
	}else
	_CMD("S"){
		stop_rotate(AZ);
		stop_rotate(EL);
	}else
	//Status commands
	_CMD("C"){
		char tmp[8];
		snprintf(tmp, 8, "AZ=%03d", (int)get_axis_deg(AZ));
		uart_print(tmp);
		lf=1;
	}else
	_CMD("B"){
		char tmp[8];
		snprintf(tmp, 8, "EL=%03d", (int)get_axis_deg(EL));
		uart_print(tmp);
		lf=1;
	}else
	_CMD("C2"){
		char tmp[20];
		snprintf(tmp, 8, "AZ=%03d EL=%03d", (int)get_axis_deg(AZ), (int)get_axis_deg(EL));
		uart_print(tmp);
		lf=1;
	}else
	//Axis homing
	_CMD("M"){
		set_axis_deg(AZ, atoi(&ln[1]));
	}else
	_CMD("W"){
		set_axis_deg(AZ, atoi(&ln[1]));
		set_axis_deg(EL, atoi(&ln[5]));
	}else
		uart_print("?>");
	
	uart_print("\r");
	if(lf) uart_print("\n");
}

inline static void proto_print(){
	switch(proto){
	case DEBUG:
		uart_print("Protocol *: Debug\r\n");
		break;
	case EASYCOMM2:
		uart_print("Protocol E: Easycomm2\r\n");
		break;
	case YAESU:
		uart_print("Protocol Y: YAESU GS232B\r\n");
		break;
	}
}

void proto_switch(const char* ln){
	switch(ln[1]){
	case 'E':
		proto=EASYCOMM2;
		proto_print();
		break;
	case 'Y':
		proto=YAESU;
		/*fall-through*/
	case '?':
		proto_print();
		break;
	default:
		proto=DEBUG;
		proto_print();
	}
}

int main(void){
	char ln[50]={0};
	init_uart();
	DDRC |= (1<<2)|(1<<3)|(1<<4)|(1<<5);
	//PORTC = 0xff;
	adc_init();
	while(1){
		uart_readln(ln, sizeof(ln));
		if(ln[0]=='+') proto_switch(ln);
		else switch(proto){
		case EASYCOMM2:
			comm_Easy(ln);
			break;
		case YAESU:
			comm_Yaesu(ln);
			break;
		case DEBUG:
			comm_Debug(ln);
			break;
		}
		_delay_ms(100);
	}
	
	return 0;
}
