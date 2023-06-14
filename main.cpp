/****************************/
/** Antenna rotator control**/
/**                        **/
/** 2019,HA5KFU radio club **/
/** 2023,HA8KDA            **/
/****************************/

//Yaesu protocol

#include <avr/io.h>
#include <avr/interrupt.h>
#include <array>
#include <cctype>
#include <string_view>
#include <avr/wdt.h>

namespace{
	inline void uartputs(const std::string_view& str){
		for(char c: str){
			while(!(UCSRA & (1<<UDRE)));
			UDR=c;
		}
	}

	inline void uartputint3(uint16_t i){
		i%=1000;
		while(!(UCSRA & (1<<UDRE)));
		UDR=i/100+'0';
		i%=100;
		while(!(UCSRA & (1<<UDRE)));
		UDR=i/10+'0';
		i%=10;
		while(!(UCSRA & (1<<UDRE)));
		UDR=i+'0';
	}
	
	enum button: uint8_t{
		right=1<<2, left=1<<3, down=1<<4, up=1<<5
	};
	
	constexpr const uint16_t max = 510; //TODO calibrate??
	
	inline uint16_t getas(){
		ADMUX = (ADMUX & 0xF8) | 0;
		ADCSRA |= (1<<ADSC);
		while(ADCSRA & (1<<ADSC));
		uint32_t ad = ADC;
		
		return ad*450/max;
	}

	constexpr const int8_t azimuth_calibration = 13;

	inline uint16_t getel(){
		ADMUX = (ADMUX & 0xF8) | 1 ;
		ADCSRA |= (1<<ADSC);
		while(ADCSRA & (1<<ADSC));
		uint32_t ad = ADC;
		
		return ad*180/max;
	}
	
	inline void press(button b){
		PORTC= PORTC | b;
	}
	
	inline void release(button b){
		PORTC= PORTC & (~b);
	}
}

volatile struct {
	struct {
		bool turn;
		bool clockwise;
	} rotation;
	struct {
		bool turn;
		bool up;
	} elevation;
	bool elevation_value;
	bool azimuth_value;
	bool position_value;
	struct {
		uint16_t goal;
		bool issued;
	} rotate;
	struct {
		uint16_t goal;
		bool issued;
	} elevate;
	bool send_ack;
	bool send_unkown;
} state;

ISR(USART_RXC_vect){
	static std::array<char, 16> buffer [[gnu::section(".noinit")]]; //TODO 9 byte enough??
	static uint8_t index = 0;

	const char in = UDR;

	buffer[index++]=in;
	index%=buffer.size();

	if(in == '\r' || in == '\n'){
		if(index == 1) return;

		state.send_ack=true;
		
		switch(buffer[0]){
			case 'u':
			case 'U':
				state.elevation.turn=true;
				state.elevation.up=true;
				state.elevate.issued=false;
				break;
			case 'd':
			case 'D':
				state.elevation.turn=true;
				state.elevation.up=false;
				state.elevate.issued=false;
				break;
			case 'e':
			case 'E':
				state.elevation.turn=false;
				state.elevate.issued=false;
			case 'r':
			case 'R':
				state.rotation.turn=true;
				state.rotation.clockwise=true;
				state.rotate.issued=false;
				break;
			case 'l':
			case 'L':
				state.rotation.turn=true;
				state.rotation.clockwise=false;
				state.rotate.issued=false;
				break;
			case 'a':
			case 'A':
				state.rotation.turn=false;
				state.rotate.issued=false;
				break;
			case 's':
			case 'S':
				state.elevation.turn=false;
				state.rotation.turn=false;
				state.rotate.issued=false;
				state.elevate.issued=false;
				break;
			case 'c':
			case 'C':
				if (buffer[1]=='2')
					state.position_value=true;
				else
					state.azimuth_value=true;
				break;
			case 'b':
			case 'B':
				state.elevation_value=true;
				break;
			case 'm':
			case 'M':
				if ( std::isdigit(buffer[1]) && std::isdigit(buffer[2]) && std::isdigit(buffer[3]) ){
					for(int i=1;i<=3;i++)
						buffer[i]-='0';
					state.rotate.goal=(static_cast<uint16_t>(buffer[1])*100) + (buffer[2]*10) + buffer[3];
					state.rotate.issued=true;
					state.rotation.turn=false;
				}
				break;
			case 'w':
			case 'W':
				if ( 
					std::isdigit(buffer[1]) && std::isdigit(buffer[2]) && std::isdigit(buffer[3])
					&&
					std::isdigit(buffer[5]) && std::isdigit(buffer[6]) && std::isdigit(buffer[7])
				){
					for(int i=1;i<=7;i++)
						buffer[i]-='0';
					state.rotate.goal=(static_cast<uint16_t>(buffer[1])*100) + (buffer[2]*10) + buffer[3];
					state.rotate.issued=true;
					state.rotation.turn=false;
					state.elevate.goal=(static_cast<uint16_t>(buffer[5])*100) + (buffer[6]*10) + buffer[7];
					state.elevate.issued=true;
					state.elevation.turn=false;
				}
				break;
			default:
				state.send_ack=false;
				state.send_unkown=true;
				break;

		}
		index=0;
	}
}

void realmain [[noreturn]] (){
	UBRRH = 0;	
	UBRRL = 51;
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
	
	DDRC |= (1<<2)|(1<<3)|(1<<4)|(1<<5);
	
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);

	sei();

	WDTCR = (1<<WDCE) | (1<<WDP2) | (1<<WDP1);

	while(1){
		wdt_reset();

		uint16_t mechanical_azimuth = getas();
		uint16_t geographical_azimuth = mechanical_azimuth + azimuth_calibration;
		uint16_t elevation = getel();

		//Should it move right?
		if 
		( 
			(state.rotation.turn && state.rotation.clockwise) ||
			(state.rotate.issued && (geographical_azimuth < state.rotate.goal))
		)
			{
				press(button::right);
				release(button::left);

				if(mechanical_azimuth>440){ // 'Safty first' -- Anatoly Dyatlov
					state.rotation.turn=false;
					state.rotate.issued=false;
				}
			}
		else if //Should it move left?
		( 
			(state.rotation.turn && !state.rotation.clockwise) ||
			(state.rotate.issued && (geographical_azimuth > state.rotate.goal))
		)
			{
				press(button::left);
				release(button::right);

				if(mechanical_azimuth<10){ // 'Safty first' -- Anatoly Dyatlov
					state.rotation.turn=false;
					state.rotate.issued=false;
				}
			}
		else 
			{
				release(button::left);
				release(button::right);
				state.rotate.issued=false;
				//release left and right
			}
			
		//Should it move up?
		if 
		( 
			(state.elevation.turn && state.elevation.up) ||
			(state.elevate.issued && (elevation < state.elevate.goal))
		)
			{
				release(button::down);
				press(button::up);

				if(elevation>170){ // 'Safty first' -- Anatoly Dyatlov
					state.elevate.issued=false;
					state.elevation.turn=false;
				}
			}
		else if //Should it move down?
		( 
			(state.elevation.turn && !state.elevation.up) ||
			(state.elevate.issued && (elevation > state.elevate.goal))
		)
			{
				//check
				press(button::down);
				release(button::up);

				if(elevation<10){ // 'Safty first' -- Anatoly Dyatlov
					state.elevate.issued=false;
					state.elevation.turn=false;
				}
			}
		else 
			{
				release(button::up);
				release(button::down);
				state.elevate.issued = false;
			}
			
		if (state.send_unkown) {uartputs("? >"); state.send_unkown=false;}
		if (state.position_value){
			state.position_value = false;
			uartputs("+0");
			uartputint3(geographical_azimuth);
			uartputs("+0");
			uartputint3(elevation);
			uartputs("\n");
		}
		if (state.azimuth_value){
			state.azimuth_value = false;
			uartputs("+0");
			uartputint3(geographical_azimuth);
			uartputs("\n");
		}
		if (state.elevation_value){
			state.elevation_value = false;
			uartputs("+0");
			uartputint3(elevation);
			uartputs("\n");
		}
		if (state.send_ack) {uartputs("\r"); state.send_ack=false;}
	}
}

int main(){
	realmain();
	return 0;
}
