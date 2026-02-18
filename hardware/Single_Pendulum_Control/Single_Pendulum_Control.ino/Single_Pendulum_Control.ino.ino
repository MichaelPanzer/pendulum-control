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

#define stepsPerRevolution (200*(float)microstep)
#define pulleyTeeth 40.0
#define beltPitch 2e-3

#define MAX_SPEED 1.6
#define MAX_ACCEL 130000

#define FILTER_CONST 1.8 //1.5 //min value of 1, Higher value reduces high-speed filter %
#define FILTER_LIMIT 0.90 //from 0 to 1, Baseline filter % at low speed
#define FILTER_FLOOR 0.55 //from 0 to 1, Minimum filter % at high speed


BLA::Matrix<4,1> state;
float time, lastTime, v, dt, x, a;

BLA::Matrix<4,1> fpUp = {PI, 0, 0, 0};
//BLA::Matrix<1,4> k = {107.75202347,  21.42132424, -31.6227766,  -26.11407771};
BLA::Matrix<1,4> k = {
180.17666490589082, 40.72958209175653, -61.91391873669039, -49.948372792073776
};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

float theta;
float llast_theta;
float last_dt;

int toSteps(float x){
  return stepsPerRevolution*x/(pulleyTeeth*beltPitch);
}

float toDistance(int steps){
  return pulleyTeeth*beltPitch*(float)steps/stepsPerRevolution;
}

float der2(float th, float lth, float llth, float dt, float ldt){
  return (sq(dt)*(llth-lth) + 2.*dt*ldt*(th-lth) + sq(ldt)*(th-lth)) / (dt*ldt*(dt+ldt));
}

void setup(){  
  Serial.begin(115200);

  //SETUP ENCODER
  if (!as5600.begin()) {
    Serial.println("Could not find AS5600 sensor, check wiring!");
    while (1)
    delay(10);
  }

  Serial.println("AS5600 found!");
  as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
  as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
  // setup filters
  as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
  as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);

  //Wait for pendulum to stop moving
  int llastPos = as5600.getRawAngle();
  int lastPos = llastPos+1;
  delay(50);
  while(as5600.getRawAngle() != lastPos || llastPos != lastPos){
    llastPos = lastPos;
    lastPos = as5600.getRawAngle();
    delay(500);
    Serial.print("Waiting to stabilize, Angle = ");
    Serial.println(lastPos);

  }
  // Reset encoder position to 0
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
  stepper->moveTo(toSteps(-0.5*(totalWidth)));
  while (stepper->isRunning()) {
    // wait
  }
  // Reset zero again
  stepper->setCurrentPosition(0);
  Serial.println("homed");


  delay(3000);
  lastTime = micros()*1e-6;
  last_dt = 1e-6;

  state = {(4095-as5600.getAngle()) * 2.0*PI/4095.0, 0., 0., 0.};
  theta = state(0);
  llast_theta = state(0);
}
 

void loop(){  
  time = micros()*1e-6;
  dt = time-lastTime;
  int a = as5600.getAngle();
  while(a==4095)
    theta = (4095-as5600.getAngle()) * 2.0*PI/4095.0;
    a = as5600.getAngle();

  a = (FILTER_LIMIT-FILTER_FLOOR)*pow(FILTER_CONST, -abs(state(1))) + FILTER_FLOOR; //higher speed -> lower a & less weight on the previous states
  float d = der2(theta, state(0), llast_theta, dt, last_dt);
  state(1) = (1-a)*d + a*state(1);
  
  state(3) = toDistance(1000000/stepper->getCurrentSpeedInUs());
  state(2) = toDistance(stepper->getCurrentPosition());

  llast_theta = state(0);
  state(0) = theta;

  lastTime = time;
  last_dt = dt;

  if (abs(state(2))+buffer > 0.5*totalWidth || theta<0.6*PI || theta>1.6*PI){
    Serial.println(theta);
    Serial.println(state(0));
    stepper->forceStop();
    delay(100);
    stepper->setSpeedInHz(abs(toSteps(0.2)));
    stepper->moveTo(0);
    delay(5000);
    v = 0;
    state(1) = 0;
  }
  

  //LQR TO CALC ACCELERATION
  v += dt* (k * (fpUp-state))(0);
  if(v>MAX_SPEED) v=MAX_SPEED;
  else if (v<-MAX_SPEED) v = -MAX_SPEED;
  
  //APPLY ACCELERATION;
  stepper->setSpeedInHz(abs(toSteps(v)));
  if (v >= 0) stepper->runForward();
  else stepper->runBackward();

  /*
  Serial.print(a, 5);
  Serial.print(", ");
  Serial.print(d, 5);
  Serial.print(", ");
  Serial.println(state(1), 5);
  */
  //delay(20);

}








