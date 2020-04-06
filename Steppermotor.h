#pragma once	
#include "Motor.h"
#include "Parameters.h"
#include <AccelStepper.h>
class Steppermotor:
	public Motor
{
 public:
	Steppermotor(const byte dirPin, const byte stepPin,const byte limitSwitchPin, const byte enablePin,int interfaceType = 1);
	void updateControlValue();
	void setSpeedInAllBuffers(float speed);
	void setAngleInAllBuffers(float speed);//also in Stepps
	void applyControlValue();
	void prepareToMove();
	void finishMovement();
	void homing(float speedPercentage = 1);
	AccelStepper* motor;
	int* currentAngleInSteps;
	int angleInStepsBuffer = 0;
	int speedInStepsBuffer = 0;
	int* getAngle();
	float getAngleInRadians();
	byte enablePin;

};


