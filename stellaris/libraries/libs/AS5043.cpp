#include "Arduino.h"

#include "wiring_private.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "AS5043.h"

#define SPI_CLOCK_DIV2 		2
#define SPI_CLOCK_DIV4 		4
#define SPI_CLOCK_DIV8 		8
#define SPI_CLOCK_DIV16 	16
#define SPI_CLOCK_DIV32 	32
#define SPI_CLOCK_DIV64 	64
#define SPI_CLOCK_DIV128 	128

#define SPI_MODE0 			0x00
#define SPI_MODE1 			0x80
#define SPI_MODE2 			0x40
#define SPI_MODE3 			0xC0

// Pins PD0, PD1, PD2, PD3 for SSI, for convenience
#define DEFAULT_SSI 		3

// CLK, SS, RX, TX
static const uint8_t g_SSIPinMap[4][4] = { { PA_2, PA_3, PA_4, PA_5 }, { PF_2,
		PF_3, PF_0, PF_1 }, { PB_4, PB_5, PB_6, PB_7 },
		{ PD_0, PD_1, PD_2, PD_3 }, };

static const unsigned long g_ulSSIBase[4] = {
SSI0_BASE, SSI1_BASE, SSI2_BASE, SSI3_BASE };

static const unsigned long g_ulSSIPeriph[4] = {
SYSCTL_PERIPH_SSI0, SYSCTL_PERIPH_SSI1,
SYSCTL_PERIPH_SSI2, SYSCTL_PERIPH_SSI3 };

static const unsigned long g_ulSSIConfig[4][4] = { { GPIO_PA2_SSI0CLK,
		GPIO_PA3_SSI0FSS, GPIO_PA4_SSI0RX,
		GPIO_PA5_SSI0TX }, { GPIO_PF2_SSI1CLK, GPIO_PF3_SSI1FSS,
GPIO_PF0_SSI1RX, GPIO_PF1_SSI1TX }, { GPIO_PB4_SSI2CLK,
GPIO_PB5_SSI2FSS, GPIO_PB6_SSI2RX, GPIO_PB7_SSI2TX }, {
GPIO_PD0_SSI3CLK, GPIO_PD1_SSI3FSS, GPIO_PD2_SSI3RX,
GPIO_PD3_SSI3TX }, };

static const unsigned long g_ulSSIPort[4] = {
GPIO_PORTA_BASE, GPIO_PORTF_BASE, GPIO_PORTB_BASE, GPIO_PORTD_BASE };

static const unsigned long g_ulSSIPins[4] = {
GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 };

// *****************************************************************************

AS5043Class::AS5043Class(void) {
	SSIModule = DEFAULT_SSI;
	isSSI = true;
	this->pinclk = g_SSIPinMap[SSIModule][0];
	pincsn = 0;
	pindo = g_SSIPinMap[SSIModule][2];
	pinprog = g_SSIPinMap[SSIModule][3];
	maxdevices = MAX_AS5043_DEVICES;
}

AS5043Class::AS5043Class(uint8_t module) {
	SSIModule = module;
	isSSI = true;
	this->pinclk = g_SSIPinMap[SSIModule][0];
	pincsn = g_SSIPinMap[SSIModule][1];;
	pindo = g_SSIPinMap[SSIModule][2];
	pinprog = g_SSIPinMap[SSIModule][3];
	maxdevices = MAX_AS5043_DEVICES;
}

AS5043Class::AS5043Class(uint8_t pinCLK, uint8_t pinDO, uint8_t pinProg) {
	SSIModule = 255;
	isSSI = false;
	this->pinclk = pinCLK;
	pincsn = 0;
	pindo = pinDO;
	pinprog = pinProg;
	maxdevices = MAX_AS5043_DEVICES;
}

AS5043Class::AS5043Class(uint8_t pinCLK, uint8_t pinDO, uint8_t pinProg, uint8_t pinCSn) {
	SSIModule = 255;
	isSSI = false;
	this->pinclk = pinCLK;
	pincsn = pinCSn;
	pindo = pinDO;
	pinprog = pinProg;
	maxdevices = MAX_AS5043_DEVICES;
}

// *****************************************************************************

void AS5043Class::setPinCSn(uint8_t pinCSn) {
	pincsn = pinCSn;
	pinMode(pincsn, OUTPUT);
	digitalWrite(pincsn, HIGH);
}

void AS5043Class::begin() {
	this->begin(pincsn);
}

void AS5043Class::begin(uint8_t pinCSn) {

	if (pincsn > 0)
		this->setPinCSn(pinCSn);

	if (isSSI)
		this->initSSI();
	else {
		pinMode(this->pinclk, OUTPUT);
		pinMode(pindo, INPUT);
		pinMode(pinprog, OUTPUT);
		digitalWrite(this->pinclk, LOW);
		digitalWrite(pinprog, LOW);
	}
}

void AS5043Class::end() {
	digitalWrite(pincsn, HIGH);
	if (isSSI)
		ROM_SSIDisable(g_ulSSIBase[SSIModule]);
}

unsigned int AS5043Class::read() {
	if (isSSI)
		this->readSSI();
	else
		this->readBang();
	return sensorsData[0];
}

unsigned int AS5043Class::get() {
	return this->get(0);
}

unsigned int AS5043Class::get(uint8_t index) {
	return sensorsData[index];
}

unsigned int AS5043Class::getAngle() {
	return this->getAngle(0);
}

unsigned int AS5043Class::getAngle(uint8_t index) {
	unsigned int _singleData = sensorsData[index];
	return (_singleData >> 6);
}

unsigned int AS5043Class::getStatus() {
	return this->getStatus(0);
}

unsigned int AS5043Class::getStatus(uint8_t index) {
	unsigned int _singleData = sensorsData[index];
	return (_singleData & B00111111);
}

unsigned int AS5043Class::singleAlign() {
	this->end();
	this->setPins();

	// Entering alignment mode
	digitalWrite(pincsn, LOW);
	delayMicroseconds(4);
	digitalWrite(pincsn, HIGH);
	delayMicroseconds(4);
	digitalWrite(pinprog, HIGH);
	delayMicroseconds(4);
	digitalWrite(pincsn, LOW);
	delayMicroseconds(4);
	digitalWrite(pinprog, LOW);

	// Reading a single sensor data by software
	unsigned int data = 0;

	digitalWrite(this->pinclk, LOW);
	delayMicroseconds(1);
	for (uint8_t k = 0; k < 16; k++) {
		digitalWrite(this->pinclk, HIGH);
		delayMicroseconds(1);
		data = (data << 1) | (digitalRead(pindo) ? 0x0001 : 0x0000);
		digitalWrite(this->pinclk, LOW);
		delayMicroseconds(1);
	}
	this->closeComm();
	this->begin();

	return (data >> 6);
}

void AS5043Class::softProg(unsigned int data) {
	this->end();
	this->setPins();

	// Entering the programming mode
	digitalWrite(this->pinclk, LOW);
	delayMicroseconds(4);
	digitalWrite(pincsn, LOW);
	delayMicroseconds(4);
	digitalWrite(pinprog, HIGH);
	delayMicroseconds(4);
	digitalWrite(pincsn, HIGH);
	delayMicroseconds(4);

	// Start data write
	for (uint8_t k = 0; k < 16; k++)
		this->pushBit(((data & (0x0001 << (15 - k))) >> (15 - k)));

	// Finish the programming mode
	digitalWrite(pinprog, LOW);
	delayMicroseconds(4);
	digitalWrite(pincsn, LOW);
	delayMicroseconds(4);

	this->closeComm();
	this->begin();
}

bool AS5043Class::parityCheck(unsigned int data) {
	unsigned int parity = 0;
	unsigned int parity_bit = data & 0x0001;
	// We start with 1 because the bit 0 is the parity bit
	for (uint8_t k = 1; k < 16; k++)
		parity ^= ((data & (0x0001 << k)) >> k);
	return (parity_bit == parity);
}

// *****************************************************************************

void AS5043Class::initSSI() {
	// Ensure clock sync
	ROM_SysCtlClockSet(
	SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	// Enable the peripheral but disable SSI use, to allow for configuration
	ROM_SysCtlPeripheralEnable(g_ulSSIPeriph[SSIModule]);
	ROM_SSIDisable(g_ulSSIBase[SSIModule]);
	// ssPin is specified elsewhere
	ROM_GPIOPinConfigure(g_ulSSIConfig[SSIModule][0]);
	ROM_GPIOPinConfigure(g_ulSSIConfig[SSIModule][2]);
	ROM_GPIOPinConfigure(g_ulSSIConfig[SSIModule][3]);
	// Link module with pins
	ROM_GPIOPinTypeSSI(g_ulSSIPort[SSIModule], g_ulSSIPins[SSIModule]);
	// Ensure programming mode starts off (sending data is done without the module anyway)
	pinMode(pinprog, OUTPUT);
	digitalWrite(pinprog, LOW);
	// Configure the SSI module as master, mode 2, 1 MHz and 8 bit reads
	ROM_SSIClockSourceSet(g_ulSSIBase[SSIModule], SSI_CLOCK_SYSTEM);
	ROM_SSIConfigSetExpClk(g_ulSSIBase[SSIModule], ROM_SysCtlClockGet(),
	SSI_FRF_MOTO_MODE_2, SSI_MODE_MASTER, 1000000UL, 8);
	// Ready for use...
	ROM_SSIEnable(g_ulSSIBase[SSIModule]);
	// Clear out any data in the RX FIFO
	unsigned long initialData = 0;
	while (ROM_SSIDataGetNonBlocking(g_ulSSIBase[SSIModule], &initialData))
		;
}
//	this->setSSPin(g_SSIPinMap[SSIModule][1]);

void AS5043Class::readSSI() {
	uint8_t base_bits = maxdevices * 16 + maxdevices;
	uint8_t required_reads = base_bits / 8 + base_bits % 8 == 0 ? 0 : 1;
	unsigned long chainData = 0;
	uint8_t index = 0;
	digitalWrite(pincsn, LOW);
	for (uint8_t k = 0; k < required_reads; k++) {
		// Extract data
		chainData = (chainData << 8) | (this->atomSSI() & 0x00FF);
		// Put data in container by singles
		if ((k + 1) * 8 >= (index + 1) * 17) {
			uint8_t shift_val = 7 - index % 8;
			sensorsData[index++] = (chainData >> shift_val) & 0xFFFF;
		}
	}
	digitalWrite(pincsn, HIGH);
}

void AS5043Class::readBang() {
	uint8_t base_bits = maxdevices * 16 + maxdevices - 1;
	unsigned long chainData = 0;
	uint8_t index = 0;
	digitalWrite(pincsn, LOW);
	delayMicroseconds(1);
	digitalWrite(this->pinclk, LOW);
	delayMicroseconds(1);
	for (uint8_t k = 0; k < base_bits; k++) {
		digitalWrite(this->pinclk, HIGH);
		delayMicroseconds(1);
		chainData = (chainData << 1) | (digitalRead(pindo) ? 0x0001 : 0x0000);
		// Put data in container by singles
		if (k + 1 >= (index + 1) * 16 + index)
			sensorsData[index++] = chainData & 0xFFFF;
		digitalWrite(this->pinclk, LOW);
		delayMicroseconds(1);
	}
	this->closeComm();
}

void AS5043Class::pushBit(bool bit) {
	digitalWrite(pinprog, bit);
	delayMicroseconds(1);
	digitalWrite(this->pinclk, HIGH);
	delayMicroseconds(2);
	digitalWrite(this->pinclk, LOW);
	delayMicroseconds(1);
}

void AS5043Class::closeComm() {
	// Ensure pincsn and this->pinclk are left HIGH and pinprog LOW
	digitalWrite(pinprog, LOW);
	delayMicroseconds(1);
	digitalWrite(pincsn, HIGH);
	delayMicroseconds(1);
	digitalWrite(this->pinclk, HIGH);
}

void AS5043Class::setPins() {
	pinMode(pincsn, OUTPUT);
	pinMode(this->pinclk, OUTPUT);
	pinMode(pindo, INPUT);
	pinMode(pinprog, OUTPUT);
}

unsigned long AS5043Class::atomSSI() {
	// Write a null sequence and wait
	ROM_SSIDataPut(g_ulSSIBase[SSIModule], 0x00);
	while (ROM_SSIBusy(g_ulSSIBase[SSIModule]))
		;
	// Read value
	unsigned long _rxData = 0;
	ROM_SSIDataGet(g_ulSSIBase[SSIModule], &_rxData);
	return _rxData;
}
