#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

enum State{ OPEN, CLOSED, OPENING, CLOSING, ERRORSTATE };
enum controlType {CW, CCW, STOP, COAST};



//Analog Input Pins
int outCounerPin = 1;
int inCounterPin = 2;
int lightPin = 0;

//Digital Input Pins
int tempPin = 7;
int resetPin = 6;
int doorClosedPin = 4;
int doorOpenPin = 5;

//Analog Output Pins
int motorSpeedPin = 3;

//Digital Output Pins
int errorPin = 10;
int motorControlPin1 = 8;
int motorControlPin2 = 9;

//State variables
int temperature = 0;
int lightLevel = 0;
bool doorIsOpen = false;
bool doorIsClosed = false;
bool resetIsPushed = false;
bool sunny = false;
bool warm = false;
bool allInside = true;
State state;
long timeStart;
DHT_Unified dht(tempPin, DHT22);

void checkLight()
{
  lightLevel = analogRead(lightPin);
  if ( analogRead(lightPin) > 150 )
    sunny = true;
  else
    sunny = false;
}

void checkTemp()
{
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
    temperature = 0;
  else 
  {
    temperature = event.temperature;
    if (event.temperature > 40)
      warm = true;
    else
      warm = false;
  }
    
}

void checkSwitches()
{
  doorIsOpen = digitalRead(doorOpenPin);
  doorIsClosed = digitalRead(doorClosedPin);
}

void checkReset()
{
  resetIsPushed = digitalRead(resetPin);
}

void checkSensors()
{
  checkLight();
  checkTemp();
  checkSwitches();
}

/*
bool warm()
{
  return temperature > 34;
}

bool sunny()
{
  return lightLevel > 50;
}
*/

bool timerPast( long threshold )
{
  return millis() > threshold;
}

void runMotor ( controlType control) {

  switch (control) {
    case CW:
      // set the Left Motor CW
      analogWrite(motorSpeedPin, 255);
      digitalWrite(motorControlPin1, HIGH);   // sets the Left Motor CW
      digitalWrite(motorControlPin2, LOW);
      break;
    case CCW:
      // set the Left Motor CCW
      analogWrite(motorSpeedPin, 255);
      digitalWrite(motorControlPin1, LOW);   // sets the Left Motor CCW
      digitalWrite(motorControlPin2, HIGH);
      break;
    case STOP:
      // set the Left Motor stop
      analogWrite(motorSpeedPin, 255);        // set Left Motor enable high for braking
      digitalWrite(motorControlPin1, LOW);
      digitalWrite(motorControlPin2, LOW);
      break;
    case COAST:
      // set Left Motor Speed to 0 for coast.  Control pins are don't care
      analogWrite(motorSpeedPin, 0);
      
      break;
    default:
      // Should never get here
      break;
  }
}

void setup() 
{
  Serial.begin(9600);
  state = CLOSING;

  analogWrite(motorSpeedPin, 255);

  
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

}

void loop() 
{
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  timeStart = millis();
  while ( state == CLOSED )
  {
    Serial.print("CLOSED    ");
    Serial.print(temperature);
    Serial.print("    ");
    Serial.println(lightLevel);
    checkSensors();
    if ( timerPast(3000) && warm && sunny )
    {
      state = OPENING;
    }
  }

  timeStart = millis();
  while ( state == OPENING )
  {
    Serial.println("OPENING");
    checkSensors();
    if ( !doorIsOpen )
    {
      runMotor(CCW);
    }
    else
    {
      runMotor( STOP );
      state = OPEN;
    }
    /*
    if ( timerPast(15000) )
    {
      runMotor( STOP );
      state = ERRORSTATE;
    }
    */
  }
  
  while ( state == OPEN ) 
  {
    Serial.println("OPEN");
    checkSensors();
    if ( (allInside && (!sunny || !warm)) )
    {
      state = CLOSING;
    }
  }

  timeStart = millis();
  while ( state == CLOSING )
  {
    Serial.println("CLOSING");
    checkSensors();
    if ( !doorIsClosed )
    {
      runMotor(CW);
    }
    else
    {
      runMotor(STOP);
      state = CLOSED;
    }
    /*
    if ( timerPast(30000) )
    {
      runMotor(STOP);
      state = ERRORSTATE;
    }
    */
  }

  while ( state == ERRORSTATE )
  {
    Serial.println("ERRORSTATE");
    digitalWrite( errorPin, HIGH );
    checkReset();
    if ( resetIsPushed )
    {
      digitalWrite( errorPin, LOW );
      state == OPENING;
    }
  }
}
