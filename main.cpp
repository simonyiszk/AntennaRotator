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
#include <eeprom_var.hpp>

#define F_CPU static_cast<unsigned long>(8e6)
#include <util/delay.h>
#include "filter.hpp"

#ifndef macro____commit
#warning "No version specified"
#define macro____commit_f "Unknown version"
#endif

namespace permanent_config
{
	namespace other_memory_space{
		uint16_t azimuth_null [[gnu::section(".eeprom")]] = 0;
		uint16_t elevation_null [[gnu::section(".eeprom")]] = 0;
		uint16_t azimuth_max [[gnu::section(".eeprom")]] = 544;
		uint16_t elevation_max [[gnu::section(".eeprom")]] = 580;
		uint8_t azimuth_error [[gnu::section(".eeprom")]] = 13;
		uint8_t elevation_error [[gnu::section(".eeprom")]] = 0;
	}
	
	namespace azimuth{
		avr::eeprom_ref adc_at_max {&other_memory_space::azimuth_max}; //ADC value at maximum value
		avr::eeprom_ref error {&other_memory_space::azimuth_error}; //error in degrees from the ideal 0 deg
		avr::eeprom_ref null {&other_memory_space::azimuth_null}; //ADC offset at mechanical 0 deg
	}	
	
	namespace elevation{
		avr::eeprom_ref adc_at_max {&other_memory_space::elevation_max};
		avr::eeprom_ref error {&other_memory_space::elevation_error};	
		avr::eeprom_ref null {&other_memory_space::elevation_null};
	}
}

namespace{
	inline void uartputc(const char c){
		while(!(UCSRA & (1<<UDRE)));
		UDR=c;
	}
	
	inline void uartputs(const std::string_view& str){
		for(char c: str)
			uartputc(c);
	}

	inline void uartputint3(uint16_t i){
		i%=1000;
		uartputc(i/100+'0');
		i%=100;
		uartputc(i/10+'0');
		i%=10;
		uartputc(i+'0');
	}
	
	inline void uartputint6(uint16_t i){
		i%=1000000;
		uartputc(i/100000+'0');
		i%=100000;
		uartputc(i/10000+'0');
		i%=10000;
		uartputc(i/1000+'0');
		i%=1000;
		uartputc(i/100+'0');
		i%=100;
		uartputc(i/10+'0');
		i%=10;
		uartputc(i+'0');
	}
	
	enum button: uint8_t{
		right=1<<2, left=1<<3, down=1<<4, up=1<<5
	};
		
	enum class ADChannel : uint8_t{
		asimuth=0,
		elevation=1,
		bandgap=0x0e,
		zero=0x0f
	};
	
	inline uint16_t getadch(ADChannel ch){
		ADMUX = static_cast<uint8_t>(ch);
		ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADSC);
		while(ADCSRA & (1<<ADSC));
		return ADC;
	}
	
	namespace mechanical_limits{
		constexpr uint16_t asimuth_max_degree = 450;
		constexpr uint16_t elevation_max_degree = 180;
	}
	
	inline uint16_t getas(){
		static filter::moving_average<16, uint16_t, 0x03ff> fir;
		int16_t meas = getadch(ADChannel::asimuth) - permanent_config::azimuth::null;
		fir << (meas > 0 ? meas : 0);
		return (static_cast<uint32_t>(fir.get())*mechanical_limits::asimuth_max_degree/permanent_config::azimuth::adc_at_max) + permanent_config::azimuth::error;
	}

	inline uint16_t getel(){
		static filter::moving_average<16, uint16_t, 0x03ff> fir;
		int16_t meas = getadch(ADChannel::elevation) - permanent_config::elevation::null;
		fir << (meas > 0 ? meas : 0);
		return (static_cast<uint32_t>(fir.get())*mechanical_limits::elevation_max_degree/permanent_config::elevation::adc_at_max) + permanent_config::elevation::error;
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
		bool up;
		bool down;
	} rotate;
	struct {
		uint16_t goal;
		bool issued;
		bool up;
		bool down;
	} elevate;
	bool send_debug;
	bool send_help;
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
				break;
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
			case 'o': //WARNING Not GS232A compilent
			case 'O': //Stores the null-degree AD value in the eeprom used to correct the readings
				state.send_debug=true;
				eeprom_busy_wait(); //The non-interrupt part could use the EEPROM
				if(buffer[1] == '2'){
					permanent_config::elevation::null.set(getadch(ADChannel::elevation));
				} else {
					permanent_config::azimuth::null.set(getadch(ADChannel::asimuth));
				}
				while(1); //Reset, so no volatile access is needed
				break;
			case 'f': //WARNING Not GS232A compilent
			case 'F': //Stores the full-scale AD value in the eeprom used to correct the readings
				state.send_debug=true;
				eeprom_busy_wait(); //The non-interrupt part could use the EEPROM
				if(buffer[1] == '2'){
					permanent_config::elevation::adc_at_max.set(getadch(ADChannel::elevation));
				} else {
					permanent_config::azimuth::adc_at_max.set(getadch(ADChannel::asimuth));
				}
				while(1); //Reset, so no volatile access is needed
				break;
			case 'q': //WARNING Not GS232A compilent
			case 'Q': //Stores the pointing error of the installation in the azimuth axis
				state.send_debug=true;
				if(std::isdigit(buffer[1]) && std::isdigit(buffer[2]) && std::isdigit(buffer[3])){
					for(int i=1;i<=7;i++)
						buffer[i]-='0';
					eeprom_busy_wait(); //The non-interrupt part could use the EEPROM
					permanent_config::azimuth::error.set((static_cast<uint16_t>(buffer[1])*100) + (buffer[2]*10) + buffer[3]);
					while(1);
				} else {
					state.send_unkown=true;
				}
				break;
			case 'g': //WARNING Not GS232A compilent
			case 'G': //Stores the pointing error of the installation in the elevation axis
				state.send_debug=true;
				if(std::isdigit(buffer[1]) && std::isdigit(buffer[2]) && std::isdigit(buffer[3])){
					for(int i=1;i<=7;i++)
						buffer[i]-='0';
					eeprom_busy_wait(); //The non-interrupt part could use the EEPROM
					permanent_config::elevation::error.set((static_cast<uint16_t>(buffer[1])*100) + (buffer[2]*10) + buffer[3]);
					while(1);
				} else {
					state.send_unkown=true;
				}
				break;
			case 'i': //WARNING Not GS232A compilent
			case 'I': //Debug print
				state.position_value=true;
				state.send_debug=true;
				break;
			case 'h': //WARNING Not GS232A compilent
			case 'H':
				state.send_help=true;
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
	
	DDRC = (1<<2)|(1<<3)|(1<<4)|(1<<5);
	
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);

	sei();

	wdt_enable(WDTO_2S);

	while(1){
		wdt_reset();

		uint16_t azimuth_pos = getas();
		uint16_t elevation_pos = getel();

		//Should it move right?
		if 
		( 
			(state.rotation.turn && state.rotation.clockwise) ||
			( (state.rotate.issued || state.rotate.up) && (azimuth_pos < state.rotate.goal))
		)
			{
				state.rotate.issued=false;
				state.rotate.up=true;
				release(button::left);
				press(button::right);
			}
		else if //Should it move left?
		( 
			(state.rotation.turn && !state.rotation.clockwise) ||
			(( state.rotate.issued || state.rotate.down) && (azimuth_pos > state.rotate.goal))
		)
			{
				state.rotate.issued=false;
				state.rotate.down=true;
				release(button::right);
				press(button::left);
			}
		else 
			{
				release(button::left);
				release(button::right);
				state.rotate.issued=false;
				state.rotate.up=false;
				state.rotate.down=false;
				//release left and right
			}
			
		//Should it move up?
		if 
		( 
			(state.elevation.turn && state.elevation.up) ||
			((state.elevate.issued || state.elevate.up) && (elevation_pos < state.elevate.goal))
		)
			{
				state.elevate.issued=false;
				state.elevate.up=true;
				release(button::down);
				press(button::up);
			}
		else if //Should it move down?
		( 
			(state.elevation.turn && !state.elevation.up) ||
			( (state.elevate.issued || state.elevate.down) && (elevation_pos > state.elevate.goal))
		)
			{
				state.elevate.issued=false;
				state.elevate.down=true;
				release(button::up);
				press(button::down);
			}
		else 
			{
				release(button::up);
				release(button::down);
				state.elevate.issued = false;
				state.elevate.up=false;
				state.elevate.down=false;
			}
		
		if (state.send_unkown) {uartputs("? >"); state.send_unkown=false;}
		if (state.position_value){
			state.position_value = false;
			uartputs("+0");
			uartputint3(azimuth_pos);
			uartputs("+0");
			uartputint3(elevation_pos);
			uartputc('\n');
		}
		if (state.azimuth_value){
			state.azimuth_value = false;
			uartputs("+0");
			uartputint3(azimuth_pos);
			uartputc('\n');
		}
		if (state.elevation_value){
			state.elevation_value = false;
			uartputs("+0");
			uartputint3(elevation_pos);
			uartputc('\n');
		}
		if (state.send_debug){
			state.send_debug=false;
			uartputs("\rADC: ");
			uartputint6(getadch(ADChannel::bandgap));
			uartputc(' ');
			uartputint6(getadch(ADChannel::zero));
			uartputs("\n\rAzimuth: ");
			uartputint3(permanent_config::azimuth::error);
			uartputc(' ');
			uartputint6(getadch(ADChannel::asimuth));
			uartputc(' ');
			uartputint3(permanent_config::azimuth::null);
			uartputc(' ');
			uartputint6(permanent_config::azimuth::adc_at_max);
			uartputs("\n\rElevation: ");
			uartputint3(permanent_config::elevation::error);
			uartputc(' ');
			uartputint6(getadch(ADChannel::elevation));
			uartputc(' ');
			uartputint3(permanent_config::elevation::null);
			uartputc(' ');
			uartputint6(permanent_config::elevation::adc_at_max);
			uartputc('\n');
		}
		if (state.send_help){
			state.send_help=false;
			uartputs("AntennaRotator controller HA5KFU HA7CSK HA8KDA -- https://github.com/simonyiszk/AntennaRotator\n\r" macro____commit "\n\n\n");
		}
		if (state.send_ack) {uartputc('\r'); state.send_ack=false;}
	}
}

int main(){
	realmain();
	return 0;
}
