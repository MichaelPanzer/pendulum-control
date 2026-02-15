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
#define SW_TX 6
SoftwareSerial SERIAL_PORT(SW_RX, SW_TX);
TMC2209Stepper driver(&SERIAL_PORT, 0.11f, 0b00);

//Constants
#define PI 3.1415926535897932384626433832795

#define totalWidth 0.556
#define buffer 0.030

#define microstep 8

#define stepsPerRevolution (200*microstep)
#define pulleyTeeth 60
#define beltPitch 2e-3

#define MAX_SPEED 1.0
#define MAX_ACCEL 40000

BLA::Matrix<4,1> state;
float time, lastTime, v, dt;

BLA::Matrix<4,1> fpUp = {PI, 0, 0, 0};
BLA::Matrix<1,4> k = {107.75202347,  21.42132424, -31.6227766,  -26.11407771};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

float theta;
float llast_theta;
float last_dt;

int toSteps(float x){
  return stepsPerRevolution*x/(pulleyTeeth*beltPitch);
}

float toDistance(int steps){
  return pulleyTeeth*beltPitch*steps/stepsPerRevolution;
}

Matrix<4,1> calcState(float theta, float last_theta, float llast_theta, float x, float xDot, float dt, float last_dt){
  BLA::Matrix<4,1> newState;
  //float dTheta = (theta - last_theta);
  newState(0) = theta;
  newState(1) =  ((-dt-last_dt)*(theta-last_theta) - (-dt)*(theta-llast_theta)) /((-dt)*(-dt-last_dt)*(-last_dt));
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
  SERIAL_PORT.begin(115200);
  driver.begin();
  driver.toff(5);
  driver.rms_current(2000);
  if (microstep==1) driver.microsteps(0);//full step mode
  else driver.microsteps(microstep);
  Serial.println(driver.microsteps());

  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);
  stepper->setDirectionPin(dirPin);
  stepper->setAutoEnable(true);   // <---- ADD THIS

  // HOMING
  pinMode(limSw, INPUT_PULLUP);  
  stepper->setSpeedInHz(toSteps(0.1));   
  stepper->setAcceleration(MAX_ACCEL);
  stepper->runForward();
  while(digitalRead(limSw)){
    //Serial.println("homing");
  }
  stepper->forceStop();
  delay(200);

  stepper->setCurrentPosition(0);
  stepper->moveTo(toSteps(-0.5*totalWidth - buffer));
  while (stepper->isRunning()) {
    // wait
  }
  // Reset zero again
  stepper->setCurrentPosition(0);
  Serial.println("homed");

  state = {as5600.getAngle()*2*PI/4095., 0, 0, 0};

  delay(3000);
  lastTime = micros()*1e-6;
  last_dt = 1e-6;
}
 

void loop(){  
  time = micros()*1e-6;
  dt = time-lastTime;
  theta = as5600.getAngle() * 2*PI / 4095.0;

  state = calcState(theta, state(0), llast_theta, toDistance(stepper->getCurrentPosition()), v, dt, last_dt);
  llast_theta = state(0);
  lastTime = time;
  last_dt = dt;

  
  if (abs(state(2)) > 0.5*totalWidth - buffer) {
    stepper->forceStop();
    delay(100);
    stepper->setSpeedInHz(abs(toSteps(0.2)));
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
  
  Serial.println(state(1), 5);
  delay(1);

}








