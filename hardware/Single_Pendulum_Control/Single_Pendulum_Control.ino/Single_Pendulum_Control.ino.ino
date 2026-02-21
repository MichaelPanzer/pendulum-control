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

#define MAX_SPEED 0.8
#define MAX_ACCEL 100000

#define FILTER_CONST 1.8 //1.5 //min value of 1, Higher value reduces high-speed filter %
#define FILTER_LIMIT 0.90 //from 0 to 1, Baseline filter % at low speed
#define FILTER_FLOOR 0.55 //from 0 to 1, Minimum filter % at high speed

#define L 0.295
#define G 9.81

#define STAB_LIMIT 0.8 //distance from vertical for lQR control
#define ENERGY_CONST 0.011372574219845546 //Constant for normalized energy expression

#define HOMING()                                        \
  stepper->setSpeedInHz(toSteps(0.1));                  \
  stepper->runForward();                                \
                                                        \
  while (digitalRead(limSw)) {                          \
    /* homing */                                        \
  }                                                     \
                                                        \
  stepper->forceStop();                                 \
  delay(200);                                           \
                                                        \
  stepper->setCurrentPosition(0);                       \
  stepper->moveTo(toSteps(-0.5*(totalWidth)));          \
                                                        \
  while (stepper->isRunning()) {                        \
    /* wait */                                          \
  }                                                     \
                                                        \
  stepper->setCurrentPosition(0);                       \
  Serial.println("homed");                              \




BLA::Matrix<4,1> state;
float time, lastTime, v, dt, x, a;

BLA::Matrix<4,1> fpUp = {PI, 0, 0, 0};
//BLA::Matrix<1,4> k = {107.75202347,  21.42132424, -31.6227766,  -26.11407771};
BLA::Matrix<1,4> k = {
//181.4979469962424, 41.27060085617614, -54.7722557505162, -49.17024780213365
203.67626620582544, 46.612113778021914, -61.834694240084225, -57.14166434532145
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

//This function creates a soft speed limit with a high degree monomial function
int softSpeedLim(float x){
  return toSteps( MAX_SPEED* (1.0 - min(pow(1.3*abs(x) / (0.5*totalWidth - buffer), 2), 1.0)) );
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
  stepper->setAcceleration(MAX_ACCEL);
  HOMING();


  delay(3000);
  lastTime = micros()*1e-6;
  last_dt = 1e-6;

  state = {(4095-as5600.getAngle()) * 2.0*PI/4095.0, 0., 0., 0.};
  theta = state(0);
  llast_theta = state(0);
}
 

void loop(){  
  //Enforce Movement Limits
  if (abs(state(2))+buffer > 0.5*totalWidth){
    stepper->forceStop();
    Serial.println("CRASH");

    HOMING();
    delay(100);
    stepper->setSpeedInHz(abs(toSteps(0.2)));
    stepper->moveTo(0);
    delay(5000);
    v = 0;
    state(1) = 0;
  }

  //Calculate state
  time = micros()*1e-6;
  dt = time-lastTime;

  //This loop is to handle errors in I2C
  int a = as5600.getAngle();
  /*
  if(abs(fmod(state(1)*dt + state(0), 2*PI) - ((4095-a) * 2.0*PI/4095.0)) > PI*0.3)
    while(a==4095){
      a = as5600.getAngle();
      Serial.println(time);
    }
  }
  */
  theta = (4095-a) * 2.0*PI/4095.0;

  a = (FILTER_LIMIT-FILTER_FLOOR)*pow(FILTER_CONST, -abs(state(1))) + FILTER_FLOOR; //higher speed -> lower a & less weight on the previous states
  float d = der2(theta, state(0), llast_theta, dt, last_dt);
  state(1) = (1-a)*d + a*state(1);
  
  state(3) = toDistance(1000000/stepper->getCurrentSpeedInUs());
  state(2) = toDistance(stepper->getCurrentPosition());

  llast_theta = state(0);
  state(0) = theta;

  lastTime = time;
  last_dt = dt;

  //SWING UP
  if (ENERGY_CONST*sq(state(1)) - cos(state(0)) < 1){ 
    Serial.print("ERECTING | Speed = ");
    Serial.print(softSpeedLim(state(2)));
    Serial.print(", x = ");
    Serial.println(state(2));

    if (state(1)*cos(state(0)) < 0){
      if(state(2) > 0){//slow down if nearing positive limit
        stepper->setSpeedInHz(softSpeedLim(state(2)));
      }
      else {
        stepper->setSpeedInHz(toSteps(MAX_SPEED));
      }

      stepper->runForward();
    }
    else {
      if(state(2) < 0){ //Slow down if nearing negative limit
        stepper->setSpeedInHz(softSpeedLim(state(2)));
      }
      else {
        stepper->setSpeedInHz(toSteps(MAX_SPEED));
      }

      stepper->runBackward();
    }
  }
  //COAST 
  else if(theta<PI-STAB_LIMIT || theta>PI+STAB_LIMIT){

  }
  //LQR Control
  else{
    Serial.print("LQR | ");
    Serial.print(a, 5);
    Serial.print(", ");
    Serial.print(d, 5);
    Serial.print(", ");
    Serial.println(state(1), 5);

    v += dt* (k * (fpUp-state))(0);
    if(v>MAX_SPEED) v=MAX_SPEED;
    else if (v<-MAX_SPEED) v = -MAX_SPEED;
    
    //APPLY ACCELERATION;
    stepper->setSpeedInHz(abs(toSteps(v)));
    if (v >= 0) stepper->runForward();
    else stepper->runBackward();
  }
}








