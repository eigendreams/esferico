#ifndef Servo_h
#define Servo_h

#include <inttypes.h>
#include <ServoTimers.h>

#define MIN_PULSE_WIDTH     	544     		// the shortest pulse sent to a servo in us
#define MAX_PULSE_WIDTH     	2400     		// the longest pulse sent to a servo in us
#define DEFAULT_PULSE_WIDTH 	1500     		// default pulse width when servo is attached
#define REFRESH_INTERVAL    	20000     		// minimum time to refresh servos in microseconds
#define SERVOS_PER_TIMER   		12     			// the maximum number of servos controlled by one timer

// Even though each timer handles up to SERVOS_PER_TIMER servos, an extra servo channel is used for
// storing the "remainder" ticks value to complete the REFRESH_INTERVAL time
#define MAX_SERVOS   			(_Nbr_16timers * (SERVOS_PER_TIMER + 1))
#define INVALID_SERVO         	255     		// flag indicating an invalid servo index

typedef struct {
	uint8_t nbr :6;             				// a pin number from 0 to 38
	uint8_t isActive :1; 						// true if this channel is enabled, pin not pulsed if false
} ServoPin_t;

typedef struct {
	ServoPin_t Pin;
	volatile unsigned int ticks;
} servo_t;

class Servo {
public:
	Servo();
	uint8_t attach(int pin); 					// attach the given pin to the next free channel, sets pinMode, returns channel number (INVALID_SERVO if failure)
	uint8_t attach(int pin, int min, int max); 	// as above but also sets min and max values for writes.
	void detach();
	void write(int value); 						// if value is < MIN_PULSE_WIDTH its treated as an angle, otherwise as pulse width in microseconds
	void writeMicroseconds(int value); 			// write pulse width in microseconds
	int read(); 								// returns current pulse width as an angle between 0 and 180 degrees
	int readMicroseconds(); 					// returns current pulse width in microseconds for this servo
	bool attached(); 							// return true if this servo is attached, otherwise false
private:
	uint8_t servoIndex; 						// index into the channel data for this servo
	uint16_t min :12; 							// minimum in microseconds (unlike the due!), packed to 4*3 bits
	uint16_t max :12; 							// maximum in microseconds (unlike the due!), packed to 4*3 bits
	void setRefresh(void); 						// Stellaris specific, refreshes the "remainder" ticks value
};

#endif
