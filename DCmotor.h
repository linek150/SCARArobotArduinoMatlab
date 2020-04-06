#pragma once
#include "Motor.h"
#include <L298N.h>
#include "Parameters.h"

class DCmotor:
	public Motor
{
public:
	DCmotor(const byte PWMPin, const byte dir1Pin, const byte dir2Pin, byte limitSwitchPin, const byte dcEncoderFirstPin, const byte dcEncoderSecondPin, int* counter, void(*interruptionFunction1)(), void(*interruptionFunction2)(), float P = 1.f, float I = 0.f, float D = 0.f);
	void homing(float speedPercentage = 1);
	int* getAngle();
	float getAngleInRadians();
	void updateControlValue();
	void applyControlValue();

	L298N* motor;//Pointer to physical object control 
	int* currentAngleInPulsesP;
	int speedInPPSBuffer=0;
	void setSpeedInAllBuffers(float speed);
	void setAngleInAllBuffers(float angleInRad);
	void finishMovement();
	float PID[3];
private:
	int previousAngleInPulses = 0;
	bool firstUpdate = true;
	int angleBufferPulses = 0;
	long int previousTime = 0;
	bool previouslyForward = true;
	

};


