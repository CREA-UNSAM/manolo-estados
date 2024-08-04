
// DEFINES
#define LOGIC 0
#define DEBUG 1                       
#define MOTORS_MAX_PWM_VALUE 255
#define MOTORS_MIN_PWM_VALUE 35

const int baseSpeed = 160;

// PIN DEFINITIONS
const int PIN_LED = 8;           //D8 | Digital 8 | GPIO 14 
const int PIN_BUTTON = 9;        //D9 | Digital 9 | GPIO 15

// MOTOR RIGHT
const int PIN_MOTOR_R_PWM = 5;   //D5 | Digital 5 | GPIO 11
const int PIN_MOTOR_R_1 = 6;     //D6 | Digital 6 | GPIO 12
const int PIN_MOTOR_R_2 = 7;     //D7 | Digital 7 | GPIO 13

// MOTOR LEFT
const int PIN_MOTOR_L_PWM = 3;    //D3 | Digital 3 | GPIO 5
const int PIN_MOTOR_L_1 = 2;      //D2 | Digital 2 | GPIO 4
const int PIN_MOTOR_L_2 = 4;      //D4 | Digital 4 | GPIO 6

// SENSOR PINS
const int PIN_SENSOR_0 = 11;
const int PIN_SENSOR_1 = A5;
const int PIN_SENSOR_2 = A4;
const int PIN_SENSOR_3 = A3;
const int PIN_SENSOR_4 = A2;
const int PIN_SENSOR_5 = A1;
const int PIN_SENSOR_6 = A0;
const int PIN_SENSOR_7 = 12;

const int CANT_ANALOG_SENSORS = 6;
const int CANT_DIGITAL_SENSORS = 2;

const int CANT_ALL_SENSORS = CANT_ANALOG_SENSORS + CANT_DIGITAL_SENSORS;

const int PINS_ANALOG_SENSORS[CANT_ANALOG_SENSORS] = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5, PIN_SENSOR_6};
const int PINS_DIGITAL_SENSORS[CANT_DIGITAL_SENSORS] = {PIN_SENSOR_0, PIN_SENSOR_7};

//const float SPEED_MULTIPLIER_0[2] = {135, 135};
//const float SPEED_MULTIPLIER_1[2] = {100, 125};
//const float SPEED_MULTIPLIER_1_5[2] = {80, 120};
//const float SPEED_MULTIPLIER_2[2] = {50, 110};
//const float SPEED_MULTIPLIER_2_5[2] = {-60, 120};
//const float SPEED_MULTIPLIER_3[2] = {-65, 120};
//const float SPEED_MULTIPLIER_4[2] = {-70, 140};

// float SPEED_MULTIPLIER_0[2] = {140, 140};
// const float SPEED_MULTIPLIER_1[2] = {90, 130};
// const float SPEED_MULTIPLIER_1_5[2] = {80, 140};
// const float SPEED_MULTIPLIER_2[2] = {70, 140};
// const float SPEED_MULTIPLIER_2_5[2] = {-70, 140};
// const float SPEED_MULTIPLIER_3[2] = {-80, 140};
// const float SPEED_MULTIPLIER_4[2] = {-100, 140};

float SPEED_MULTIPLIER_0[2] = {145, 145};
const float SPEED_MULTIPLIER_1[2] = {80, 130};
const float SPEED_MULTIPLIER_1_5[2] = {70, 140};
const float SPEED_MULTIPLIER_2[2] = {60, 140};
const float SPEED_MULTIPLIER_2_5[2] = {-70, 140};
const float SPEED_MULTIPLIER_3[2] = {-80, 140};
const float SPEED_MULTIPLIER_4[2] = {-100, 140};

// const float SPEED_MULTIPLIER_0[2] = {220, 220};
// const float SPEED_MULTIPLIER_1[2] = {120, 180};
// const float SPEED_MULTIPLIER_1_5[2] = {80, 180};
// const float SPEED_MULTIPLIER_2[2] = {60, 170};
// const float SPEED_MULTIPLIER_2_5[2] = {0, 160};
// const float SPEED_MULTIPLIER_3[2] = {-60, 180};
// const float SPEED_MULTIPLIER_4[2] = {-80, 150};

const unsigned int blinkDelay = 490;
const unsigned int runBlinkDelayHigh = 500;
const unsigned int runBlinkDelayLow = 500;
const unsigned int runDelay = 0;
const unsigned int startDelay = 5000;
const unsigned int adrenalinaDelay = 3000;

int analogSensorValues[CANT_ANALOG_SENSORS];
int sensorValues[CANT_ALL_SENSORS];

int motorSpeedR;
int motorSpeedL;

bool state = false;
bool blink = false;
bool button_state;
bool onMemory = false;

unsigned long blinkTimer;
unsigned long runTimer;
unsigned long startTimer;
unsigned long adrenalinaTimerCenter;
unsigned long adrenalinaTimerLeft;
unsigned long adrenalinaTimerRight;

int memory = 0;


void setup() 
{
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
  startTimer = millis();

  unsigned long adrenalinaTimerCenter = 0;
  unsigned long adrenalinaTimerLeft = 0;
  unsigned long adrenalinaTimerRight = 0;
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
  Serial.print(motorSpeedL);
  Serial.print(" | Motor Speed R: ");
  Serial.println(motorSpeedR);
}

void handleMotorSpeed(unsigned long currentTime)
{
  motorSpeedL = baseSpeed;
  motorSpeedR = baseSpeed;
  onMemory = false;

  if(sensorValues[0] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_4[0];
    motorSpeedR = SPEED_MULTIPLIER_4[1];
    memory = -1;
    adrenalinaTimerRight = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
  }
  else if(sensorValues[7] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_4[1];
    motorSpeedR = SPEED_MULTIPLIER_4[0];
    memory = 1;
    adrenalinaTimerRight = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
  }
  else if(sensorValues[1] == 1 && sensorValues[2] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_2_5[0];
    motorSpeedR = SPEED_MULTIPLIER_2_5[1];
    memory = 0;
    adrenalinaTimerRight = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
  }
  else if(sensorValues[1] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_3[0];
    motorSpeedR = SPEED_MULTIPLIER_3[1];
    memory = -1;
    adrenalinaTimerRight = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
  }
  else if(sensorValues[6] == 1 && sensorValues[5] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_2_5[1];
    motorSpeedR = SPEED_MULTIPLIER_2_5[0];
    memory = 0;
    adrenalinaTimerRight = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
  }
  else if(sensorValues[6] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_3[1];
    motorSpeedR = SPEED_MULTIPLIER_3[0];
    memory = 1;
    adrenalinaTimerRight = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
  }
  else if(sensorValues[2] == 1 && sensorValues[3] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_1_5[0];
    motorSpeedR = SPEED_MULTIPLIER_1_5[1];
    memory = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerRight = 0;
    if(adrenalinaTimerLeft == 0)
    {
      adrenalinaTimerLeft = currentTime;
    }
  }
  else if(sensorValues[2] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_2[0];
    motorSpeedR = SPEED_MULTIPLIER_2[1];
    memory = 0;
    adrenalinaTimerRight = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
  }
  else if(sensorValues[5] == 1 && sensorValues[4] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_1_5[1];
    motorSpeedR = SPEED_MULTIPLIER_1_5[0];
    memory = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
    if(adrenalinaTimerRight == 0)
    {
      adrenalinaTimerRight = currentTime;
    }
  }
  else if(sensorValues[5] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_2[1];
    motorSpeedR = SPEED_MULTIPLIER_2[0];
    memory = 0;
    adrenalinaTimerRight = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
  }
  else if(sensorValues[3] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_1[0];
    motorSpeedR = SPEED_MULTIPLIER_1[1];
    memory = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerRight = 0;
    if(adrenalinaTimerLeft == 0)
    {
      adrenalinaTimerLeft = currentTime;
    }
  }
  else if(sensorValues[4] == 1)
  {
    motorSpeedL = SPEED_MULTIPLIER_1[1];
    motorSpeedR = SPEED_MULTIPLIER_1[0];
    memory = 0;
    adrenalinaTimerCenter = 0;
    adrenalinaTimerLeft = 0;
    if(adrenalinaTimerRight == 0)
    {
      adrenalinaTimerRight = currentTime;
    }
}
  else
  {
    if(memory == -1)
    {
      motorSpeedL = SPEED_MULTIPLIER_4[0];
      motorSpeedR = SPEED_MULTIPLIER_4[1]; 
      onMemory = true;
      adrenalinaTimerRight = 0;
      adrenalinaTimerCenter = 0;
      adrenalinaTimerLeft = 0;
    }
    else if(memory == 1)
    {
      motorSpeedL = SPEED_MULTIPLIER_4[1];
      motorSpeedR = SPEED_MULTIPLIER_4[0]; 
      onMemory = true;
      adrenalinaTimerRight = 0;
      adrenalinaTimerCenter = 0;
      adrenalinaTimerLeft = 0;
    }
    else
    {
      motorSpeedL = SPEED_MULTIPLIER_0[0];
      motorSpeedR = SPEED_MULTIPLIER_0[1];
      adrenalinaTimerLeft = 0;
      adrenalinaTimerRight = 0;
      if(adrenalinaTimerCenter == 0)
      {
        adrenalinaTimerCenter = currentTime;
      }

    }
  }

  if(motorSpeedL < 0)
  {
    digitalWrite(PIN_MOTOR_L_1, HIGH);   
    digitalWrite(PIN_MOTOR_L_2, LOW);
    motorSpeedL = -motorSpeedL;  
  }
  else
  {
    digitalWrite(PIN_MOTOR_L_1, LOW);   
    digitalWrite(PIN_MOTOR_L_2, HIGH);
  }

  if(motorSpeedR < 0)
  {
    digitalWrite(PIN_MOTOR_R_1, HIGH);   
    digitalWrite(PIN_MOTOR_R_2, LOW);
    motorSpeedR = -motorSpeedR;
  }
  else
  {
    digitalWrite(PIN_MOTOR_R_1, LOW);   
    digitalWrite(PIN_MOTOR_R_2, HIGH);
  }

  if(((adrenalinaTimerCenter > 0) && (currentTime - adrenalinaTimerCenter) >= adrenalinaDelay) || ((adrenalinaTimerLeft > 0) && (currentTime - adrenalinaTimerLeft) >= adrenalinaDelay) || ((adrenalinaTimerRight > 0) && (currentTime - adrenalinaTimerRight) >= adrenalinaDelay))
  {
    digitalWrite(PIN_MOTOR_R_1, LOW);   
    digitalWrite(PIN_MOTOR_R_2, HIGH);
    digitalWrite(PIN_MOTOR_L_1, LOW);   
    digitalWrite(PIN_MOTOR_L_2, HIGH);
    analogWrite(PIN_MOTOR_L_PWM, 255 - 18);
    analogWrite(PIN_MOTOR_R_PWM, 255);
    delay(300);

    unsigned long timer = millis();
    unsigned long current = millis();

    readSensorData();

    while(current - timer < 1000)
    {
      adrenalinaTimerRight = 0;
      adrenalinaTimerCenter = 0;
      adrenalinaTimerLeft = 0;
      readSensorData();
      handleMotorSpeed(current);
      current = millis();
    }
    
    //SPEED_MULTIPLIER_0 = {90, 90};
    readSensorData();
    while(sensorValues[0] != 1 || sensorValues[1] != 1 || sensorValues[2] != 1)
    {
      digitalWrite(PIN_MOTOR_R_1, LOW);   
      digitalWrite(PIN_MOTOR_R_2, HIGH);
      digitalWrite(PIN_MOTOR_L_1, LOW);   
      digitalWrite(PIN_MOTOR_L_2, HIGH);
      analogWrite(PIN_MOTOR_L_PWM, 90 - 18);
      analogWrite(PIN_MOTOR_R_PWM, 90);
      readSensorData();
    }
  }
  else
  {
    motorSpeedL = constrain(motorSpeedL - 18, MOTORS_MIN_PWM_VALUE, MOTORS_MAX_PWM_VALUE);
    motorSpeedR = constrain(motorSpeedR, MOTORS_MIN_PWM_VALUE, MOTORS_MAX_PWM_VALUE);
    analogWrite(PIN_MOTOR_L_PWM, motorSpeedL);
    analogWrite(PIN_MOTOR_R_PWM, motorSpeedR);
  }
}

void readSensorData()
{
  // Lectura de valores de sensores
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++)
    analogSensorValues[i] = analogRead(PINS_ANALOG_SENSORS[i]);

  sensorValues[0] = (LOGIC) ? !digitalRead(PINS_DIGITAL_SENSORS[0]) : digitalRead(PINS_DIGITAL_SENSORS[0]);

  //----analog to digital conversion
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    sensorValues[i + 1] = analogSensorValues[i] > 512 ? (LOGIC ? 0 : 1) : (LOGIC ? 1 : 0);
    }

  sensorValues[CANT_ALL_SENSORS - 1] = (LOGIC) ? !digitalRead(PINS_DIGITAL_SENSORS[1]) : digitalRead(PINS_DIGITAL_SENSORS[1]);

}

void run(unsigned long currentTime)
{
  if((currentTime - startTimer) >= startDelay || DEBUG)
  {
    if((currentTime - runTimer) >= runDelay)
    {
      handleMotorSpeed(currentTime);
  
      runTimer = currentTime;
      digitalWrite(PIN_LED, HIGH);
    }

    digitalWrite(PIN_LED, HIGH);
  }
  else if((blink && (currentTime - blinkTimer >= runBlinkDelayHigh)) || (!blink && (currentTime - blinkTimer >= runBlinkDelayLow)))
  {
    digitalWrite(PIN_LED, blink);
    blink = !blink;
    blinkTimer = currentTime; 
  }
}

void stop(unsigned int currentTime)
{
  if((currentTime - blinkTimer) >= blinkDelay)
  {
    analogWrite(PIN_MOTOR_L_PWM, 0);
    analogWrite(PIN_MOTOR_R_PWM, 0);
    digitalWrite(PIN_LED, LOW);
    //blink = !blink;
    //blinkTimer = currentTime;
    memory = 0;
  }
}

void loop() {
  unsigned long currentTime = millis();
  readSensorData();
  if(state)
  {
    run(currentTime);
  }
  else
  {
    stop(currentTime);
  }
  printSerialData();

  if(button_state && !digitalRead(PIN_BUTTON))
  {
    state = !state;
    Serial.println(state ? "RUN" : "STOP");
    startTimer = millis();
    blinkTimer = millis();
    blink = false;
    memory = 0;
  }

  button_state = digitalRead(PIN_BUTTON);
  //delay(1);
}
