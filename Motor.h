#pragma once
#include "Arduino.h"
class Motor
{
public:
	
	byte limitSwitchPin;
	bool running = false;
	float angleBuffer=0;
	float angleBufferRad=0;
	int controlValue=0;//used for pid 
	float speedInRadBuffer = 0;
	int homingSpeed;//TODO::add it to constructors and homing functions
	virtual void homing(float speedPercentage=1) = 0;
	virtual	int* getAngle() = 0;//Return current angle in pulses or steps
	virtual float getAngleInRadians() = 0;//Return current angle of motor in radians with respect to homing 0
	virtual void updateControlValue() = 0;
	virtual void setSpeedInAllBuffers(float speed) = 0;
	virtual void setAngleInAllBuffers(float angleInRad) = 0;
	virtual void applyControlValue()=0;
	virtual void finishMovement() = 0;
};


