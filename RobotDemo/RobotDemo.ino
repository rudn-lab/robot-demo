#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include<SoftPWM.h>
#include<EEPROM.h>

// Uses PID code from https://curiores.com/positioncontrol
// https://github.com/curiores/ArduinoTutorials/tree/main/encoderControl
// which is used under MIT license

#define FrontFwd 14
#define FrontBwd 15
#define BackFwd 16
#define BackBwd 17

#define BackEncoderA 2
#define FrontEncoderA 3
#define BackEncoderB 4
#define FrontEncoderB 5

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

int targetFront = 0;
int targetBack = 0;

int oldOutput[] = {0,0,0,0};

void loop() {
  // set target position
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

  while(Serial.available()) {
    char direction = Serial.read();
    if (direction=='F' || direction == 'B'){
      int how_much = Serial.parseInt();
      if (direction=='F'){targetFront=how_much;}
      else {targetBack=how_much;}
    }
  }

  int newOutput[] = {targetFront, posFront, targetBack, posBack};
  bool doOutput = false;
  for(byte i=0; i<4; i++){
    if (newOutput[i] != oldOutput[i]){doOutput = true;}
    oldOutput[i] = newOutput[i];
  }

  if(doOutput){
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
}


const byte motors[2][2] = {
  {FrontFwd, FrontBwd},
  {BackFwd, BackBwd},
};

void setMotor(byte motorIndex, int dir, int pwmVal){
  // If the power to send the motor is too small, we won't.
  if(pwmVal < 10) {
    SoftPWMSet(motors[motorIndex][0], 0);
    SoftPWMSet(motors[motorIndex][1], 0);
    return;
  }

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
