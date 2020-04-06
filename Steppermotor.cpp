
#include "Steppermotor.h"

Steppermotor::Steppermotor(const byte dirPin, const byte stepPin,const byte limitSwitchPin,const byte enablePin, int interfaceType = 1)
{
	pinMode(limitSwitchPin, INPUT_PULLUP);
	pinMode(enablePin, OUTPUT);
	this->motor = new AccelStepper(interfaceType, stepPin, dirPin);
	this->limitSwitchPin = limitSwitchPin;
	
	this->motor->setCurrentPosition(0);
	this->motor->setMaxSpeed(maxStepperSpeed);
	this->motor->setAcceleration(stepperAcceleration);
	this->currentAngleInSteps = new int(0);
	this->motor->setCurrentPosition(0);
	this->angleBuffer = 0;
	
	this->enablePin = enablePin;
	digitalWrite(this->enablePin, HIGH);//turns off motor
}

void Steppermotor::homing(float speedPercentage = 1)
{
	digitalWrite(this->enablePin, LOW);//turns on motor
	this->motor->setSpeed(-60);
	this->motor->runSpeed();
	while (digitalRead(this->limitSwitchPin) == HIGH)
	{
		this->motor->runSpeed();
	}
	this->motor->stop();
	this->motor->setCurrentPosition(0);
	*(this->currentAngleInSteps) = 0;
	this->motor->setMaxSpeed(maxStepperSpeed);
	this->motor->setAcceleration(stepperAcceleration);
	digitalWrite(this->enablePin, HIGH);//turns off motor
}
int* Steppermotor::getAngle()//return angle in steps
{
	return this->currentAngleInSteps;
}
void Steppermotor::prepareToMove()
{
	digitalWrite(this->enablePin, LOW);//turns on motor
	this->motor->moveTo(this->angleInStepsBuffer);
	//this->motor->setSpeed(this->speedInStepsBuffer);//vhanging
}
void Steppermotor::finishMovement()
{
	*(this->currentAngleInSteps) = angleInStepsBuffer;
	digitalWrite(this->enablePin, HIGH);//turns off motor
}

void Steppermotor::updateControlValue()
{
	
	
}
void Steppermotor::applyControlValue()
{
	this->motor->run();
}

void Steppermotor::setSpeedInAllBuffers(float speed)
{
	this->speedInRadBuffer = speed;
	this->speedInStepsBuffer = int(lround(speed*SPR));
}
void Steppermotor::setAngleInAllBuffers(float angleInRad)
{
	this->angleBufferRad = angleInRad;
	this->angleInStepsBuffer =int(lround(angleInRad * SPR));
}
float Steppermotor::getAngleInRadians()
{
	return (*(this->currentAngleInSteps) * DPS)*(PI / 180);
}


