#pragma once

#include "Others.h"
#include "DCmotor.h"
#include "math.h"
#include "Steppermotor.h"

class Robot
{
public:
	Robot(DCmotor* firstArm,DCmotor* SecondArm,Steppermotor* platform,byte electroMagnetPin);
	~Robot();
	DCmotor* firstArm;
	DCmotor* secondArm;
	Steppermotor* platform;
	Motor* allMotors[4];//stepper,firstarm,secondarm;requires NULL at the end
	byte numberOfMotors = 3;
	float relativeSpeed;//range 0-100;
	float coordinates[numberOfCoordinates];

	void matlabSetup();
	void readSerial();
	
	
	
	void goByTodoNada();
private:
	byte electroMagnetPin = 0;
	int numberOfPointsForLine = 0;
	void updateCurrentPositionBufferInCoordinates();
	void correctSecondArmAngleBuffer();
	float getCorrectSecondArmAngleForForwardCinematic();
	float* calculateDelta(floatInBytes* destinationCoordinates);
	void setMotorSpeeds(float* speeds);
	bool updateAnglesInBuffersFromCoordinates(floatInBytes* XYZ, byte configuration = 'E');
	float* inversCinematic(float X1, float Y, float Z, byte configuration);//returns pointer to array of angles [q1,q2,q3];

	float* calculateSpeedForAllMotors(float speed);//return pointer to array with speeds stepper,firstDC,secondDC all in rad/sec 

	void goToAnglesInBuffer();
	void sendMessage(String message);
	void goByLineTo(floatInBytes* coordinates,float speed);
	bool updateAnglesInBuffersFromCoordinates(float X, float Y, float Z, byte configuration = 'X');

	void homing();
	void program1();

};

