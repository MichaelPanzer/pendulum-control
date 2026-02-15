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

#define stepsPerRevolution 200*microstep
#define pulleyTeeth 40
#define beltPitch 2e-3

#define MAX_SPEED 0.8
#define MAX_ACCEL 100

BLA::Matrix<4,1> state;
float time, lastTime, v, dt;

BLA::Matrix<4,1> fpUp = {PI, 0, 0, 0};
BLA::Matrix<1,4> k = { 230.08892961 , 73.56678896, -70.71067812, -61.06097946};

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;


int toSteps(float x){
  return stepsPerRevolution*x/(pulleyTeeth*beltPitch);
}

float toDistance(float steps){
  return pulleyTeeth*beltPitch*steps/stepsPerRevolution;
}

Matrix<4,1> calcState(float theta, float x, float xDot, BLA::Matrix<4,1> state, float dt){
  BLA::Matrix<4,1> newState;
  float dTheta = (theta - state(0));


  newState(0) = theta;
  newState(1) =  dTheta / dt;
  newState(2) = x;
  newState(3) = xDot;

  return newState;
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
  as5600.enableWatchdog(false);
  as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  as5600.setHysteresis(AS5600_HYSTERESIS_OFF);
  as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
  // setup filters
  as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
  as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
  // Reset position settings to defaults
  as5600.setZPosition(as5600.getRawAngle());
  as5600.setMPosition(0);
  as5600.setMaxAngle(4096/4-1);

  delay(5000);
}
 

void loop(){  
  time = micros()*1e-6;
  
  Serial.print("(");
  Serial.print(time, 6);
  Serial.print(", ");
  Serial.print(as5600.getAngle());
  Serial.println("),");

}








