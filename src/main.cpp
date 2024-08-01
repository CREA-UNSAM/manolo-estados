#include <Arduino.h>

// DEFINES
#define LOGIC 1
#define MOTORS_MAX_PWM_VALUE 150
#define MOTORS_MIN_PWM_VALUE 0
#define MAXOUTPUT 150


// PIN DEFINITIONS
const int PIN_LED = 8;           //D8 | Digital 8 | GPIO 14 
const int PIN_BUTTON = 9;        //D9 | Digital 9 | GPIO 15

// MOTOR RIGHT
const int PIN_MOTOR_R_PWM = 5;   //D5 | Digital 5 | GPIO 11
const int PIN_MOTOR_R_1 = 7;     //D6 | Digital 6 | GPIO 12
const int PIN_MOTOR_R_2 = 6;     //D7 | Digital 7 | GPIO 13

// MOTOR LEFT
const int PIN_MOTOR_L_PWM = 3;    //D3 | Digital 3 | GPIO 5
const int PIN_MOTOR_L_1 = 4;      //D2 | Digital 2 | GPIO 4
const int PIN_MOTOR_L_2 = 2;      //D4 | Digital 4 | GPIO 6

// SENSOR PINS
const int PIN_SENSOR_0 = 11;
const int PIN_SENSOR_1 = A0;
const int PIN_SENSOR_2 = A1;
const int PIN_SENSOR_3 = A2;
const int PIN_SENSOR_4 = A3;
const int PIN_SENSOR_5 = A4;
const int PIN_SENSOR_6 = A5;
const int PIN_SENSOR_7 = 12;

const int CANT_ANALOG_SENSORS = 6;
const int CANT_DIGITAL_SENSORS = 2;

const int CANT_ALL_SENSORS = CANT_ANALOG_SENSORS + CANT_DIGITAL_SENSORS;

const int PINS_ANALOG_SENSORS[CANT_ANALOG_SENSORS] = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5, PIN_SENSOR_6};
const int PINS_DIGITAL_SENSORS[CANT_DIGITAL_SENSORS] = {PIN_SENSOR_0, PIN_SENSOR_7};

const float SPEED_MULTIPLIER_1 = 1 / 1.5;
const float SPEED_MULTIPLIER_2 = 1 / 2;
const float SPEED_MULTIPLIER_3 = 1 / 3;
const float SPEED_MULTIPLIER_4 = 1 / 4;

const unsigned int blinkDelay = 500;
const unsigned int runDelay = 200;

int analogSensorValues[CANT_ANALOG_SENSORS];
int sensorValues[CANT_ALL_SENSORS];

int motorspeedR;
int motorspeedL;

bool state = false;
bool blink = true;
bool button_state;

unsigned long blinkTimer;
unsigned long runTimer;


void setup() {
  Serial.begin(9600);
  Serial.println("STARTING SERIAL");
  for(int i{0}; i < 5; i++)
    Serial.println("==========================================");

  // Initialize the LED pin as an output
  pinMode(PIN_LED, OUTPUT);

  // Initialize the button pin as an input
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Initialize 6 analog inputs for sensors
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    pinMode(PINS_ANALOG_SENSORS[i], INPUT);
  }

  // Initialize 2 digital inputs for sensors
  for (int i = 0; i < CANT_DIGITAL_SENSORS; i++) {
    pinMode(PINS_DIGITAL_SENSORS[i], INPUT);
  }

  // Initialize the 3 outputs for each motor
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_1, OUTPUT);
  pinMode(PIN_MOTOR_L_2, OUTPUT);

  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_1, OUTPUT);
  pinMode(PIN_MOTOR_R_2, OUTPUT);

  blinkTimer = millis();
  runTimer = millis();
  button_state = digitalRead(PIN_BUTTON);
}

void printSerialData()
{
  Serial.print(" | ");

  // Monitorización por serial
  Serial.print("SA: ");
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    Serial.print(analogSensorValues[i]);
    Serial.print(" ");
  }
  Serial.print(" | ");
  for (int i = 0; i < CANT_ALL_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }

  Serial.print(" | Motor Speed L: ");
  Serial.print(motorspeedL);
  Serial.print(" | Motor Speed R: ");
  Serial.println(motorspeedR);
}

void handleMotorSpeed()
{
  // Lógica de control del seguidor de línea
  digitalWrite(PIN_MOTOR_L_1, LOW);   
  digitalWrite(PIN_MOTOR_L_2, HIGH);  
  digitalWrite(PIN_MOTOR_R_1, LOW);   
  digitalWrite(PIN_MOTOR_R_2, HIGH);  

  if(sensorValues[0] == 1)
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE * SPEED_MULTIPLIER_4);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE);
  }
  else if(sensorValues[7] == 1)
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE * SPEED_MULTIPLIER_4);
  }
  else if(sensorValues[1] == 1)
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE * SPEED_MULTIPLIER_3);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE);
  }
  else if(sensorValues[6] == 1)
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE * SPEED_MULTIPLIER_3);
  }
  else if(sensorValues[2] == 1)
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE * SPEED_MULTIPLIER_2);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE);
  }
  else if(sensorValues[5] == 1)
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE * SPEED_MULTIPLIER_2);
  }
  else if(sensorValues[3] == 1)
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE * SPEED_MULTIPLIER_1);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE);
  }
  else if(sensorValues[4] == 1)
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE * SPEED_MULTIPLIER_1);
  }
  else
  {
    analogWrite(PIN_MOTOR_L_PWM, MOTORS_MAX_PWM_VALUE);
    analogWrite(PIN_MOTOR_R_PWM, MOTORS_MAX_PWM_VALUE);
  }
}

void readSensorData()
{
  // Lectura de valores de sensores
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++)
    analogSensorValues[i] = analogRead(PINS_ANALOG_SENSORS[i]);

  sensorValues[0] = !digitalRead(PINS_DIGITAL_SENSORS[0]);

  //----analog to digital conversion
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    sensorValues[i + 1] = analogSensorValues[i] > 512 ? 0 : 1;
    }

  sensorValues[CANT_ALL_SENSORS - 1] = !digitalRead(PINS_DIGITAL_SENSORS[1]);
}

void run(unsigned long currentTime)
{
  if((currentTime - runTimer) >= runDelay)
  {
    readSensorData();
  
    handleMotorSpeed();

    printSerialData();

    runTimer = currentTime;
  }
}

void stop(unsigned int currentTime)
{
  if((currentTime - blinkTimer) >= blinkDelay)
  {
    analogWrite(PIN_MOTOR_L_PWM, 0);
    analogWrite(PIN_MOTOR_R_PWM, 0);
    digitalWrite(PIN_LED, blink);
    blink = !blink;
    blinkTimer = currentTime;
  }
}

void loop() {
  unsigned long currentTime = millis();
  if(state)
  {
    run(currentTime);
  }
  else
  {
    stop(currentTime);
  }

  if(!button_state && digitalRead(PIN_BUTTON))
  {
    state = !state;
  }

  button_state = digitalRead(PIN_BUTTON);
}