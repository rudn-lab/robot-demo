#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include<SoftPWM.h>
#include<EEPROM.h>

// Uses PID code from https://curiores.com/positioncontrol
// https://github.com/curiores/ArduinoTutorials/tree/main/encoderControl
// which is used under MIT license

#define BackFwd 14
#define BackBwd 15
#define FrontFwd 16
#define FrontBwd 17

#define FrontEncoderA 2
#define BackEncoderA 3
#define FrontEncoderB 4
#define BackEncoderB 5

bool isPort = false;

void setup() {
  Serial.begin(57600);

  if(EEPROM.read(0) == 'p'){isPort = true;}

  SoftPWMBegin();
  SoftPWMSet(FrontFwd, 0);
  SoftPWMSet(FrontBwd, 0);
  SoftPWMSet(BackFwd, 0);
  SoftPWMSet(BackBwd, 0);

  pinMode(FrontEncoderA, INPUT);
  pinMode(FrontEncoderB, INPUT);
  pinMode(BackEncoderA, INPUT);
  pinMode(BackEncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(FrontEncoderA),readFrontEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(BackEncoderA),readBackEncoder,RISING);
}


volatile int posFrontI = 0;
void readFrontEncoder(){
  int b = digitalRead(FrontEncoderB);
  if(b > 0){
    posFrontI++;
  }
  else{
    posFrontI--;
  }
}
volatile int posBackI = 0;
void readBackEncoder(){
  int b = digitalRead(BackEncoderB);
  if(b > 0){
    posBackI++;
  }
  else{
    posBackI--;
  }
}

// PID constants
float kp = 1;
float kd = 0.025;
float ki = 0.0;

void setMotor(byte motorIndex, int dir, int pwmVal);
void run_PID(byte motorIdx, int pos, int target, long currT, float deltaT, float* eprev, float* eintegral) {
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-(*eprev))/(deltaT);

  // integral
  *eintegral = (*eintegral) + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki * (*eintegral);

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  // signal the motor

  setMotor(motorIdx, dir,pwr);

  // store previous error
  *eprev = e;
}

#define FRONT_MOTOR 0
#define BACK_MOTOR 1

long prevT = 0;
float eprevFront = 0;
float eintegralFront = 0;
float eprevBack = 0;
float eintegralBack = 0;

void loop() {
  // set target position
  int targetFront = 1200;
  int targetBack = 1200;
  //int target = 250*sin(prevT/1e6);


  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;


  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int posFront = 0; 
  int posBack = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posFront = posFrontI;
    posBack = posBackI;
  }
  if(!isPort){
      // On starboard side, encoders are facing the opposite way, so the counter is also moving the opposite way 
      posFront = -posFront;
      posBack = -posBack;
    }
 
  run_PID(FRONT_MOTOR, posFront, targetFront, currT, deltaT, &eprevFront, &eintegralFront);
  run_PID(BACK_MOTOR, posBack, targetBack, currT, deltaT, &eprevBack, &eintegralBack);

  if (isPort) {
    Serial.print("P:");
  } else {
    Serial.print("S:");
  }
  Serial.print(targetFront);
  Serial.print(" ");
  Serial.print(posFront);
  Serial.print(" ");
  Serial.print(targetBack);
  Serial.print(" ");
  Serial.print(posBack);
  Serial.println();
}


const byte motors[2][2] = {
  {FrontFwd, FrontBwd},
  {BackFwd, BackBwd},
};

void setMotor(byte motorIndex, int dir, int pwmVal){
  // The directions to move the motor are reversed on the starboard side
  // equality on bools == XNOR
  // Logic is flipped if isPort == false

  if((dir == -1) == isPort){
    SoftPWMSet(motors[motorIndex][0], pwmVal);
    SoftPWMSet(motors[motorIndex][1], 0);
  }
  else if((dir == 1) == isPort){
    SoftPWMSet(motors[motorIndex][0], 0);
    SoftPWMSet(motors[motorIndex][1], pwmVal);
  }
  else{
    SoftPWMSet(motors[motorIndex][0], 0);
    SoftPWMSet(motors[motorIndex][1], 0);
  }  
}
