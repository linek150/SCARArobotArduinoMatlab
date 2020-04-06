
/*
    Name:       SCARAControl.ino
    Created:	12.10.2019 21:08:11
    Author:     PIOTR LINKOWSKI
*/
#include "Parameters.h"
#include "Global.h"
#include "Others.h"
#include "Steppermotor.h"
#include "DCmotor.h"
#include "Robot.h"
#include "Math.h"


Robot* scara;
DCmotor* dc1;
DCmotor* dc2;
Steppermotor* stepper;
int dc1Counter = 0;
int dc2Counter = 0;
void setup()
{
	//Change frequency of PWM pin 9&10
	TCCR2B = (TCCR2B & 0xF8) | 0x07;
	Serial.begin(9600);
	pinMode(13, OUTPUT);


	//creats motors
	dc1 = new DCmotor(dc1PWMPin, dc1Dir1Pin, dc1Dir2Pin,dc1LimitSwitchPin,dc1EncoderFirstPin,dc1EncoderSecondPin,&dc1Counter,&dc1CounterFirstInterruption,&dc1CounterSecondInterruption);
	dc2 = new DCmotor(dc2PWMPin, dc2Dir1Pin, dc2Dir2Pin, dc2LimitSwitchPin, dc2EncoderFirstPin, dc2EncoderSecondPin, &dc2Counter, &dc2CounterFirstInterruption, &dc2CounterSecondInterruption);
	stepper = new Steppermotor(stepperDirPin, stepperStepPin,stepperLimitSwitchPin,stepperEnablePin);
	//creat robot object
	scara = new Robot(dc1, dc2, stepper,electromagnetPin);
	//scara->matlabSetup();
}

void loop()
{
	
	scara->readSerial();
	doEveryPeriod(&blinker);
}
