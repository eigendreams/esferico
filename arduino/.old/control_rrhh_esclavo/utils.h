#include "Arduino.h"
#include "TimedAction.h"

#define TMAX 75
#define TMIN 0

double Input1, Input2;
int estado = 1;

unsigned long cuenta1, cuenta2, m1, m2;

void timed2()
{
	estado = 1;
}

TimedAction timedAction2 = TimedAction(150, timed2);

void timed1()
{
	timedAction2.setInterval(int(m2));
	timedAction2.reset();
	digitalWrite(6, HIGH);
	estado = 3;
}

TimedAction timedAction1 = TimedAction(150, timed1);

void suma1()
{
	cuenta1 += 1;
}

void suma2()
{
	cuenta2 += 1;
}

void vel()
{
	m1 = cuenta1;
	cuenta1 = 0;

	m2 = cuenta2;
	cuenta2 = 0;

	Input1 = map(m1, TMIN, TMAX, 0, 255);
	Input2 = map(m2, TMIN, TMAX, 0, 255);
}
