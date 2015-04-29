#include <Arduino.h>
#include <Servo.h>

// Stellaris specific includes
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

// The TRIM_DURATION needs adjusting, maybe with an scope
#define usToTicks(_us)    (((ROM_SysCtlClockGet() / 1000000) * _us) / 32)               				// converts microseconds to tick
#define ticksToUs(_ticks) (((unsigned)_ticks * 32)/ (ROM_SysCtlClockGet() / 1000000)) 					// converts from ticks back to microseconds
#define TRIM_DURATION     2          	                     											// compensation us to trim adjust for digitalWrite delays

static servo_t servos[MAX_SERVOS]; 																		// static array of servo structures
uint8_t ServoCount = 0;                  																// the total number of attached servos
static volatile uint8_t Channel[_Nbr_16timers]; 														// counter for the servo being pulsed for each timer

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)((_servo_nbr) / (SERVOS_PER_TIMER + 1))) 	// returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) ((_servo_nbr) % (SERVOS_PER_TIMER + 1))     					// returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer) * (SERVOS_PER_TIMER + 1) + (_channel))     				// macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])            						// macro to access servo class by timer and channel

void Servo_Handler(timer16_Sequence_t timer, unsigned long timer_base, unsigned long timer_timer, unsigned long timer_trigger);
#if defined (_useTimer1)
void HANDLER_FOR_TIMER1(void) {
	Servo_Handler(_timer1, TIM1_BASE, TIM1_TIMER, TIM1_TRIGGER);
}
#endif
#if defined (_useTimer2)
void HANDLER_FOR_TIMER2(void) {
	Servo_Handler(_timer2, TIM2_BASE, TIM2_TIMER, TIM2_TRIGGER);
}
#endif
#if defined (_useTimer3)
void HANDLER_FOR_TIMER3(void) {
	Servo_Handler(_timer3, TIM3_BASE, TIM3_TIMER, TIM3_TRIGGER);
}
#endif

void Servo_Handler(timer16_Sequence_t timer, unsigned long timer_base, unsigned long timer_timer, unsigned long timer_trig) {
	// Clear the timer interrupt.
	ROM_TimerIntClear(timer_base, timer_trig);
	// Load the timer with the next pulse width count value in ticks, checking state
	ROM_TimerLoadSet(timer_base, timer_timer, SERVO(timer, Channel[timer]).Pin.isActive ? SERVO(timer, Channel[timer]).ticks : usToTicks(DEFAULT_PULSE_WIDTH));
	// End the servo pulse set previously (if any)
	if (Channel[timer] > 0 && SERVO_INDEX(timer, Channel[timer] - 1) < ServoCount)
		digitalWrite(SERVO(timer, Channel[timer] - 1).Pin.nbr, LOW);
	// Set the current servo pin HIGH
	if (Channel[timer] < SERVOS_PER_TIMER) {
		if (SERVO_INDEX(timer, Channel[timer]) < ServoCount && SERVO(timer, Channel[timer]).Pin.isActive)
			digitalWrite(SERVO(timer, Channel[timer]).Pin.nbr, HIGH);
		Channel[timer]++; // Advance to next servo for processing next time
	} else {
		Channel[timer] = 0; // Start all over again
	}
}

static void _initISR(unsigned long timer_base, unsigned long timer_cfg,
		unsigned long timer_timer, unsigned long timer_int,
		unsigned long timer_trig, unsigned long timer_periph,
		void (*handlerFunc)(void)) {
	// Reset global time (maybe not needed)
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	// Enable TIMER
	ROM_SysCtlPeripheralEnable(timer_periph);
	// Enable processor interrupts (maybe not needed)
	ROM_IntMasterEnable();
	// Configure the TIMER with a divide by 32
	ROM_TimerConfigure(timer_base, timer_cfg);
	ROM_TimerPrescaleSet(timer_base, timer_timer, 31);
	// Initially load the timer with REFRESH_INTERVAL
	ROM_TimerLoadSet(timer_base, timer_timer, usToTicks(REFRESH_INTERVAL));
	// Setup the interrupt for the timeout.
	TimerIntRegister(timer_base, timer_timer, handlerFunc);
	// THE FOLLOWING MUST BE DONE IN REVERSE TO HALT THE INTERRUPT
	ROM_IntEnable(timer_int);
	ROM_TimerIntEnable(timer_base, timer_trig);
	ROM_TimerEnable(timer_base, timer_timer);
}

static void initISR(int timer) {
#if defined (_useTimer1)
	if (timer == _timer1)
		_initISR(TIM1_BASE, TIM1_CFG, TIM1_TIMER, TIM1_INTERRUPT, TIM1_TRIGGER, TIM1_PERIPH, HANDLER_FOR_TIMER1);
#endif
#if defined (_useTimer2)
	if (timer == _timer2)
		_initISR(TIM2_BASE, TIM2_CFG, TIM2_TIMER, TIM2_INTERRUPT, TIM2_TRIGGER, TIM2_PERIPH, HANDLER_FOR_TIMER2);
#endif
#if defined (_useTimer3)
	if (timer == _timer3)
		_initISR(TIM3_BASE, TIM3_CFG, TIM3_TIMER, TIM3_INTERRUPT, TIM3_TRIGGER, TIM3_PERIPH, HANDLER_FOR_TIMER3);
#endif
}

static void _finISR(unsigned long timer_base, unsigned long timer_timer, unsigned long timer_int, unsigned long timer_trig) {
	ROM_TimerDisable(timer_base, timer_timer);
	ROM_TimerIntDisable(timer_base, timer_trig);
	ROM_IntDisable(timer_int);
}

static void finISR(int timer) {
#if defined (_useTimer1)
	_finISR(TIM1_BASE, TIM1_TIMER, TIM1_INTERRUPT, TIM1_TRIGGER);
#endif
#if defined (_useTimer2)
	_finISR(TIM2_BASE, TIM2_TIMER, TIM2_INTERRUPT, TIM2_TRIGGER);
#endif
#if defined (_useTimer3)
	_finISR(TIM3_BASE, TIM3_TIMER, TIM3_INTERRUPT, TIM3_TRIGGER);
#endif
}

static boolean isTimerActive(int timer) {
	// returns true if any servo is active on this timer
	for (uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) {
		if (SERVO(timer, channel).Pin.isActive == true)
			return true;
	}
	return false;
}

Servo::Servo() {
	if (ServoCount < MAX_SERVOS) {
		this->servoIndex = ServoCount;
		ServoCount++;
		// the last channel of each timer is reserved for the "leftover" time
		SERVO_INDEX_TO_CHANNEL(ServoCount + 1) == 0 ? ServoCount++ : false;
		servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);
		setRefresh();
	} else {
		this->servoIndex = INVALID_SERVO;
	}
}

uint8_t Servo::attach(int pin) {
	return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max) {
	if (this->servoIndex < MAX_SERVOS) {
		pinMode(pin, OUTPUT);
		servos[this->servoIndex].Pin.nbr = pin;
		// resolution of min/max is 1 uS, NOT 4 us as in the Due
		this->min = min;
		this->max = max;
		// initialize the timer if it has not already been initialized
		if (isTimerActive(SERVO_INDEX_TO_TIMER(servoIndex)) == false)
			initISR(SERVO_INDEX_TO_TIMER(servoIndex));
		// this must be set AFTER the check for isTimerActive
		servos[this->servoIndex].Pin.isActive = true;
		setRefresh();
	}
	return this->servoIndex;
}

void Servo::detach() {
	if (this->servoIndex < MAX_SERVOS) {
		servos[this->servoIndex].Pin.isActive = false;
		if (isTimerActive(SERVO_INDEX_TO_TIMER(servoIndex)) == false)
			finISR(SERVO_INDEX_TO_TIMER(servoIndex));
		setRefresh();
	}
}

void Servo::write(int value) {
	// treat values less than MIN_PULSE_WIDTH as angles in degrees (valid values in microseconds are handled as microseconds)
	if (value < MIN_PULSE_WIDTH) {
		value = constrain(value, 0, 180);
		value = map(value, 0, 180, this->min, this->max);
	}
	writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value) {
	if (this->servoIndex < MAX_SERVOS) {
		value = constrain(value, this->min, this->max);
		// compensating for interrupt overhead and convert to ticks
		value = value - TRIM_DURATION;
		value = usToTicks(value);
		servos[this->servoIndex].ticks = value;
		setRefresh();
	}
}

int Servo::read() {
	return map(readMicroseconds() + 1, this->min, this->max, 0, 180);
}

int Servo::readMicroseconds() {
	if (this->servoIndex < MAX_SERVOS)
		return (ticksToUs(servos[this->servoIndex].ticks) + TRIM_DURATION);
	else
		return 0;
}

bool Servo::attached() {
	if (this->servoIndex < MAX_SERVOS)
		return servos[this->servoIndex].Pin.isActive;
	else
		return false;
}

void Servo::setRefresh(void) {
	if (this->servoIndex < MAX_SERVOS) {
		unsigned int us_sum_servos = 0;
		for (uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) {
			// if a servo is not active, it may be because it was never used, or it is simply
			// detached, since attaching also initializes the interrupt, and detaching is almost
			// never used, we may act as if an inactive pin means there was no servo, and mask
			// with the DEFAULT_PULSE_WIDTH
			if (SERVO(SERVO_INDEX_TO_TIMER(this->servoIndex), channel).Pin.isActive)
				us_sum_servos += ticksToUs(SERVO(SERVO_INDEX_TO_TIMER(this->servoIndex), channel).ticks);
			else
				us_sum_servos += DEFAULT_PULSE_WIDTH;
		}
		int us_leftover = REFRESH_INTERVAL - us_sum_servos;
		// ensure a wait of at least 4 us to avoid missing the interrupt
		us_leftover < 4 ? us_leftover = 4 : false;
		SERVO(SERVO_INDEX_TO_TIMER(this->servoIndex), SERVOS_PER_TIMER).ticks = usToTicks(us_leftover);
	}
}
