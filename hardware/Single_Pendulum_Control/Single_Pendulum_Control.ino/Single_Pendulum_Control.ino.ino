#include <FastAccelStepper.h>
#include <Adafruit_AS5600.h>
#include <BasicLinearAlgebra.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
using namespace BLA;

//Sensors
#define LIM_SW 11
Adafruit_AS5600 as5600;


//Stepper Driver
#define DIR_PIN 14 //a0
#define STEP_PIN 9 

#define SW_RX 4
#define SW_TX 6
SoftwareSerial serialPort(SW_RX, SW_TX);
TMC2209Stepper driver(&serialPort, 0.11f, 0b00);

//Constants
#define PI 3.1415926535897932384626433832795

#define TOTAL_WIDTH 0.556
#define MOVEMENT_BUFFER 0.030

#define microstep 8

#define stepsPerRevolution (200.0*(float)microstep)
#define PULLEY_TEETH 40.0
#define BELT_PITCH 2e-3

#define MAX_SPEED_LQR 0.9
#define MAX_SPEED 0.7
#define MAX_ACCEL 90000

//Encoder Filter Params
#define FILTER_CONST 1.8 //1.5 //min value of 1, Higher value reduces high-speed filter %
#define FILTER_LIMIT 0.90 //from 0 to 1, Baseline filter % at low speed
#define FILTER_FLOOR 0.55 //from 0 to 1, Minimum filter % at high speed


#define LQR_MAX_ANGLE 0.5 //distance from vertical for lQR control
#define ENERGY_CONST 0.010100685652095763 //0.011372574219845546 //Constant for normalized energy expression

//Soft limit params
#define SOFT_LIM_START 0.2
#define SOFT_LIM_END 0.5

#define HOMING()                                        \
  Serial.println("Begin Homing");                       \
  stepper->setSpeedInHz(toSteps(0.1));                  \
  stepper->runForward();                                \
                                                        \
  while (digitalRead(LIM_SW)) {                          \
    /* homing */                                        \
  }                                                     \
                                                        \
  stepper->forceStop();                                 \
  delay(200);                                           \
                                                        \
  stepper->setCurrentPosition(0);                       \
  stepper->moveTo(toSteps(-0.5*(TOTAL_WIDTH)));          \
                                                        \
  while (stepper->isRunning()) {                        \
    /* wait */                                          \
  }                                                     \
                                                        \
  stepper->setCurrentPosition(0);                       \
  Serial.println("homed");                              \


BLA::Matrix<4,1> state;
float time, lastTime, dt;
float theta;
float llast_theta;
float last_dt;
float v;
float a;

BLA::Matrix<4,1> fpUp = {PI, 0, 0, 0};
BLA::Matrix<1,4> k = {
//190.58346292018933, 45.949676015077905, -60.55300708194896, -52.506193013906625
198.26643771785666, 43.94529586609255, -61.8346942400838, -56.5417634050763
};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;



int toSteps(float x){
  return stepsPerRevolution*x/(PULLEY_TEETH*BELT_PITCH);
}

float toDistance(int steps){
  return PULLEY_TEETH*BELT_PITCH*(float)steps/stepsPerRevolution;
}

float der2(float th, float lth, float llth, float dt, float ldt){
  return (sq(dt)*(llth-lth) + 2.*dt*ldt*(th-lth) + sq(ldt)*(th-lth)) / (dt*ldt*(dt+ldt));
}

//This function creates a soft speed limit as the cart approaches movement limits
float softSpeedLim(float x, float x_dot, float start, float end){
  //Checks to make sure that cart is moving towards limit and outside of full speed zone 
  float normX = abs(x) / (0.5*TOTAL_WIDTH - MOVEMENT_BUFFER);

  if( (x>0 && x_dot>0) || (x<0 && x_dot<0)){
    if(normX>end){
      // Serial.print("   PAST LIMIT   ");
      return 0;
    } else if(normX>start){ //Apply soft lim function
      // Serial.print("   LIMIT   ");
      return MAX_SPEED * (end-normX) / (end-start);
    }
  }

  // Serial.print("   NO LIMIT   ");
  return MAX_SPEED;
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
  serialPort.begin(115200);
  driver.begin();
  driver.toff(5);
  driver.rms_current(2000);
  if (microstep==1) driver.microsteps(0);//full step mode
  else driver.microsteps(microstep);
  Serial.println(driver.microsteps());

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setAutoEnable(true);   // <---- ADD THIS

  // HOMING
  pinMode(LIM_SW, INPUT_PULLUP);  
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
  if (abs(state(2))+MOVEMENT_BUFFER > 0.5*TOTAL_WIDTH){
    stepper->forceStop();
    Serial.println("CRASH");

    HOMING();
    delay(100);
    stepper->setSpeedInHz(abs(toSteps(0.2)));
    stepper->moveTo(0);
    delay(5000);
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
  if(theta<PI-LQR_MAX_ANGLE || theta>PI+LQR_MAX_ANGLE){
    if (ENERGY_CONST*sq(state(1)) - cos(state(0)) < 1){ //Normalized energy 
      Serial.print("ERECTING | ");
      
      if (state(1)*cos(state(0)) < 0){
        stepper->setSpeedInHz(toSteps( softSpeedLim(state(2), 1.0, SOFT_LIM_START, SOFT_LIM_END) ));
        stepper->runForward();
      }
      else {
        stepper->setSpeedInHz(toSteps( softSpeedLim(state(2), -1.0, SOFT_LIM_START, SOFT_LIM_END) ));
        stepper->runBackward();
      }
    }
    //COAST 
    else {
      Serial.print("Coast | ");
      
      stepper->setSpeedInHz(toSteps(min( softSpeedLim(state(2), state(3), SOFT_LIM_START, SOFT_LIM_END), abs(state(3)) ))); //Apply soft limits to slow down as approaching limit
      if(state(3)>0) stepper->runForward();
      else stepper->runBackward();
    }
  }
  //LQR Control
  else{
    Serial.print("LQR | ");

    //APPLY ACCELERATION;
    v += dt* (k * (fpUp-state))(0);
    if (v > MAX_SPEED)    v = MAX_SPEED;
    else if(v < -MAX_SPEED)   v = -MAX_SPEED;

    //APPLY ACCELERATION;
    stepper->setSpeedInHz(abs(toSteps(v)));
    if (v >= 0) stepper->runForward();
    else stepper->runBackward();
  }

  /*
  Serial.print("x_dot: ");
  Serial.print(state(3));
  Serial.print(", x: ");
  Serial.print(state(2));
  Serial.print(", θ_dot: ");
  Serial.print(state(1));
  Serial.print(", θ: ");
  Serial.println(state(0));
  */

  Serial.println();
}








