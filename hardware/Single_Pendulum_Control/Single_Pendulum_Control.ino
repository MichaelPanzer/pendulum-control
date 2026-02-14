#include <FastAccelStepper.h>
#include <Adafruit_AS5600.h>
#include <BasicLinearAlgebra.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
using namespace BLA;

//Sensors
#define limSw 11
Adafruit_AS5600 as5600;


//Stepper Driver
#define dirPin 14 //a0
#define stepPin 9 

#define SW_RX 4
#define SW_TX 3
SoftwareSerial SERIAL_PORT(SW_RX, SW_TX);
TMC2209Stepper driver(&SERIAL_PORT, 0.11f, 0b00);

//Constants
#define PI 3.1415926535897932384626433832795
#define microstep 8

const float totalWidth = 0.556;
const float buffer = 0.015;

const float stepsPerRevolution = 200*microstep;
const float pulleyTeeth = 20;
const float beltPitch = 0.002;

const float MAX_SPEED = 0.2;

BLA::Matrix<4,1> state;
float time, lastTime, v, dt;

BLA::Matrix<4,1> fpUp = {PI/2, 0, 0, 0};
BLA::Matrix<1,4> k = { 199.6155074, 44.07988199, -111.80339887, -79.79097216};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;


float toSteps(float x){
  return stepsPerRevolution*x/(pulleyTeeth*beltPitch);
}

float toDistance(float steps){
  return pulleyTeeth*beltPitch*steps/stepsPerRevolution;
}

Matrix<4,1> calcState(float theta, float x, float xDot, BLA::Matrix<4,1> state, float dt){
  BLA::Matrix<4,1> newState;
  float dTheta = (theta - state(0));

  if(dt<1e-6){
    dt = 1e-6;
  }
  newState(0) = theta;
  newState(1) =  dTheta / dt;
  newState(2) = x;
  newState(3) = xDot;

  return newState;
}

void setup(){  
  Serial.begin(57600);

  //SETUP ENCODER
  if (!as5600.begin()) {
    Serial.println("Could not find AS5600 sensor, check wiring!");
    while (1)
    delay(10);
  }
  Serial.println("AS5600 found!");
  as5600.enableWatchdog(false);
  as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
  as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
  // setup filters
  as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
  as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
  // Reset position settings to defaults
  as5600.setZPosition(as5600.getRawAngle());
  as5600.setMPosition(4095);
  as5600.setMaxAngle(4095);

  //SETUP STEPPER DRIVER
  SERIAL_PORT.begin(9600);
  driver.begin();
  driver.toff(5);
  if (microstep==1) driver.microsteps(0);//full step mode
  else driver.microsteps(microstep);
  Serial.println(driver.microsteps());

  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);
  stepper->setDirectionPin(dirPin);
  stepper->setAutoEnable(true);   // <---- ADD THIS

  // HOMING
  pinMode(limSw, INPUT_PULLUP);  
  stepper->setAcceleration(toSteps(10));  // steps/sec^2
  stepper->setSpeedInHz(toSteps(0.1));   
  stepper->runForward();
  while(digitalRead(limSw)){
    //Serial.println("homing");
  }
  stepper->forceStop();
  delay(200);

  stepper->setCurrentPosition(0);
  stepper->moveTo(toSteps(-totalWidth / 2.0));
  while (stepper->isRunning()) {
    // wait
  }
  // Reset zero again
  stepper->setCurrentPosition(0);
  Serial.println("homed");

  state = {as5600.getAngle()*PI/4095., 0, toDistance(stepper->getCurrentPosition()), 0};

  delay(3000);
  lastTime = millis()*1e-3;
}
 

void loop(){  
  time = millis()*1e-3;
  dt = time-lastTime;
  state = calcState(as5600.getAngle()*PI/4095., toDistance(stepper->getCurrentPosition()), v, state, dt);
  lastTime = time;

  if (abs(state(2)) > totalWidth/2 - buffer) {
    stepper->moveTo(0);
    delay(5000);
    v = 0;
  }

  //LQR TO CALC ACCELERATION
  v += (dt*(k * (state-fpUp))(0));
  if(v>MAX_SPEED) v=MAX_SPEED;
  else if (v<-MAX_SPEED) v = -MAX_SPEED;
  //APPLY ACCELERATION;
  stepper->setSpeedInHz(abs(toSteps(v)));

  if (v >= 0) stepper->runForward();
  else stepper->runBackward();
}








