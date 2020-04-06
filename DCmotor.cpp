#include "DCmotor.h"

DCmotor::DCmotor(const byte PWMPin, const byte dir1Pin, const byte dir2Pin,const byte limitSwitchPin,const byte encoderFirstPin,const byte encoderSecondPin, int* counter, void(*firstInterruptionFunction)(), void(*secondInterruptionFunction)(), float P = 1.f, float I = 0.f, float D = 0.f)//First motor with pins and corresponding encoder counter
{
	this->motor = new L298N(PWMPin, dir1Pin, dir2Pin);

	this->limitSwitchPin = limitSwitchPin;
	this->currentAngleInPulsesP = counter;
	this->angleBuffer = 0;
	this->PID[0] = P;
	this->PID[1] = I;
	this->PID[2] = D;
	this->controlValue = 0;
	*(this->currentAngleInPulsesP) = 0;
	pinMode(encoderFirstPin, INPUT);
	pinMode(encoderSecondPin, INPUT);
	pinMode(limitSwitchPin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(encoderFirstPin), firstInterruptionFunction, CHANGE);
	attachInterrupt(digitalPinToInterrupt(encoderSecondPin), secondInterruptionFunction, CHANGE);


}
void DCmotor::homing(float speedPercentage = 1)
{
	this->motor->setSpeed(minSpeedOfDcMotor*2*(speedPercentage));
	//this->motor->forward();
	while (digitalRead(this->limitSwitchPin) == HIGH)
	{
		this->motor->backward();
	}
	this->motor->stop();
	delay(150);
	*(this->currentAngleInPulsesP) = 0;
}
void DCmotor::applyControlValue()
{
	if(this->controlValue > minSpeedOfDcMotor || this->controlValue < -minSpeedOfDcMotor)
	{
		if (((previouslyForward && this->controlValue < 0) || (!previouslyForward && this->controlValue > 0))&& !firstUpdate)//If change direction stop for one loop and it is not first move in sequence
		{
			this->motor->stop();
			previouslyForward = !(previouslyForward);
			return;
		}
		firstUpdate = false;//We make first move in this sequence 
		this->motor->setSpeed(abs(this->controlValue));
		if (this->controlValue > 0)
		{
			this->motor->forward();
			previouslyForward = true;
		}
		else
		{
			this->motor->backward();
			previouslyForward = false;
		}
	}
	else
	{
		this->motor->stop();
		firstUpdate = true;//Prepare for the next movement becouse that one was last for this sequence
	}
	

}
void DCmotor::setSpeedInAllBuffers(float speed)

{
	this->speedInRadBuffer = speed;
	this->speedInPPSBuffer = speed * PPR;
}
void DCmotor::setAngleInAllBuffers(float angleInRad)
{
	this->angleBufferRad = angleInRad;
	this->angleBufferPulses = angleInRad * PPR;

}
void DCmotor::finishMovement()
{
	controlValue = 0;
	previousTime = 0;
	previousAngleInPulses = 0;
	previouslyForward = true;

}



void DCmotor::updateControlValue()
{
	float currentError = 0;
	float speedError = 0;
	currentError = this->angleBufferPulses - float(*(this->currentAngleInPulsesP));
	if (currentError > OneAndAHalfDegreeInPulses )//If error is more than 5 degree control speed if less control position
	{
		speedError=this->speedInPPSBuffer-abs((previousAngleInPulses - *(currentAngleInPulsesP)) / ((millis() - previousTime)/1000));
		previousTime = millis();
		previousAngleInPulses = *(currentAngleInPulsesP);
		this->controlValue = this->controlValue+speedError * PredutionForDCMotor*this->PID[0];
		if (controlValue < minSpeedOfDcMotor)controlValue = minSpeedOfDcMotor;
	}
	else
	{
		//P
		this->controlValue = currentError * this->PID[0];
		//I
		//D
	}
	if (this->controlValue > 255)this->controlValue = 255;
	if (this->controlValue < -255)this->controlValue = -255;
}
int* DCmotor::getAngle()//Return pointer to counter
{
	return this->currentAngleInPulsesP;
}
float DCmotor::getAngleInRadians()
{
	return (*(this->currentAngleInPulsesP) / PPD)*(PI / 180);
}


