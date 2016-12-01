
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
int doorClosedPin = 5;
int doorOpenPin = 4;

//Analog Output Pins
int motorSpeedPin = 3;

//Digital Output Pins
int errorPin = 10;
int motorControlPin1 = 8;
int motorControlPin2 = 9;

//State variables
int temperature = 0;
int lightLevel = 0;
bool doorIsOpen = true;
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
  doorIsOpen = !digitalRead(doorOpenPin);
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
      Serial.print(" CW");
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

  pinMode(motorSpeedPin,OUTPUT);
  pinMode(motorControlPin1,OUTPUT);
  pinMode(motorControlPin2,OUTPUT);
  
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  doorIsOpen = false;
  
  while ( !doorIsOpen )
  {
    Serial.println("running Motor");
    runMotor(CW);
    delay(500);
    checkSensors();
  }
  

}

void loop() 
{
  delay(250);

  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  timeStart = millis();
  while ( state == CLOSED )
  {
    delay(250);
    Serial.print("CLOSED    ");
    Serial.print(temperature);
    Serial.print("    ");
    Serial.println(lightLevel);
    checkSensors();
    Serial.println(doorIsOpen);
    if ( timerPast(3000) && warm && sunny )
    {
      state = OPENING;
    }
  }

  timeStart = millis();
  while ( state == OPENING )
  {
    delay(250);
    Serial.println("OPENING");
    checkSensors();
    Serial.println(doorIsOpen);
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
    delay(250);
    Serial.println("OPEN");
    checkSensors();
    Serial.println(doorIsOpen);
    if ( (allInside && (!sunny || !warm)) )
    {
      state = CLOSING;
    }
  }

  timeStart = millis();
  while ( state == CLOSING )
  {
    delay(250);
    Serial.println("CLOSING");
    checkSensors();
    Serial.println(doorIsOpen);
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
    delay(250);
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

