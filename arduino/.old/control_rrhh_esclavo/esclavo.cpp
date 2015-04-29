#include <Arduino.h>
#include "TimedAction.h"
#include <MsTimer2.h>

#include "utils.h"

void setup()
{
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	attachInterrupt(0, suma1, CHANGE);
	attachInterrupt(1, suma2, CHANGE);
	MsTimer2::set(300, vel);
	MsTimer2::start();
}

void loop()
{   
	if( estado == 1 )
	{
		digitalWrite(5, HIGH);
		digitalWrite(6, LOW);

		delay(10);
		estado = 2;
		timedAction1.setInterval(int(m1));
		timedAction1.reset();
		digitalWrite(5, LOW);
	}
	if( estado == 2 )
	{
		timedAction1.check();
	}
	if( estado == 3 )
	{
		timedAction2.check();
	}

	//timedActionEnviar.check();

	/*digitalWrite( 5, HIGH );
  digitalWrite( 6, LOW );

  delay(10);

  digitalWrite( 5, LOW );

  delay(50);

  digitalWrite( 6, HIGH );

  delay(70);*/
}
