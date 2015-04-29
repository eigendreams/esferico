volatile long risetime = 0;
volatile long falltime = 0;

int done = 0;

void setup()
{
  Serial.begin(57600 * 16);
  // put your setup code here, to run once:
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  attachInterrupt(11, catchrise, RISING);
  attachInterrupt(12, catchfall, FALLING);
  
}

void catchrise() {
  done = 0;
  risetime = micros();
}

void catchfall() {
  done = 1;
 falltime = micros(); 
}

void loop()
{
  if (done) {
    //Serial.println(falltime);
    //Serial.println(risetime);
    Serial.println(falltime - risetime);
    //Serial.println("a");
    done = 0;
  }
  // put your main code here, to run repeatedly:
  
}
