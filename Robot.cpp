#include "Robot.h"



Robot::Robot(DCmotor* firstArm, DCmotor* secondArm, Steppermotor* platform, byte electroMagnetPin)
{
	pinMode(electroMagnetPin, OUTPUT);
	digitalWrite(electroMagnetPin, LOW);
	this->electroMagnetPin = electroMagnetPin;
	this->firstArm = firstArm;
	this->secondArm = secondArm;
	this->platform = platform;
	for (int i = 0; i < numberOfCoordinates; i++)
	{
		this->coordinates[i] = -1;
	}
	this->allMotors[0] = platform;//q1
	this->allMotors[1] = firstArm;//q2
	this->allMotors[2] = secondArm;//q3
	this->allMotors[3] = NULL;//needed in other functions

}
void Robot::correctSecondArmAngleBuffer()
{
	secondArm->setAngleInAllBuffers(secondArm->angleBufferRad - firstArm->angleBufferRad*correctionForSecondArm);
}
float Robot::getCorrectSecondArmAngleForForwardCinematic()
{
	return secondArm->getAngleInRadians()+ firstArm->getAngleInRadians()*correctionForSecondArm;
}


void Robot::homing()
{
		delay(100);
		blinker(TURNON);
		delay(1000);
		int i = 0;
		firstArm->homing(0.5);
		platform->homing();
		
		secondArm->homing();
		
		blinker(TURNOFF);
}

float* Robot::calculateSpeedForAllMotors(float speedPercentage)//return pointer to array with speeds stepper,firstDC,secondDC all in rad/sec 
{

	float* returnSpeeds = new float[3];
	if (speedPercentage <= 0)
	{
		this->sendMessage("angles set but speed can not be <= 0");
		for (int i = 0; i < 3; i++)
		{
			returnSpeeds[i] = 0;
		}
		return returnSpeeds;
	}
	bool firstLongest, secondLongest;
	firstLongest = false;
	secondLongest = false;
	float distanceStepper, distanceFirstArm, distanceSecondArm, time1, time2, time3, timeneeded;
	//get distances in rad
	distanceStepper = abs(this->platform->getAngleInRadians() - this->platform->angleBufferRad);//distance in rad
	distanceFirstArm = abs(this->firstArm->getAngleInRadians() - this->firstArm->angleBufferRad);
	distanceSecondArm = abs(this->secondArm->getAngleInRadians() - this->secondArm->angleBufferRad);
	//calculate needed time based on maxspeed
	time1 = distanceStepper / maxStepperSpeedRadPS;
	time2 = distanceFirstArm / maxDCSpeedRadPS;
	time3 = distanceSecondArm / maxDCSpeedRadPS;
	//find gratest time
	if (max(time1, time2) == time1)
	{
		if (max(time1, time3) == time1)//first time is the longest
		{
			firstLongest = true;
		}
	}
	else if (max(time2, time3) == time2)//second time is the greatest
	{
		secondLongest = true;
	}
	//calculat time needed for movement 
	if (firstLongest)
	{
		timeneeded = distanceStepper / (speedPercentage*maxStepperSpeed);
	}
	else if (secondLongest)
	{
		timeneeded = distanceFirstArm / (speedPercentage*maxDCSpeedRadPS);
	}
	else
	{
		timeneeded = distanceSecondArm / (speedPercentage*maxDCSpeedRadPS);
	}
	// calculate speeds
	returnSpeeds[0] = distanceStepper / (timeneeded);
	returnSpeeds[1] = distanceFirstArm / (timeneeded);
	returnSpeeds[2] = distanceSecondArm / (timeneeded);
	return returnSpeeds;
}

void Robot::updateCurrentPositionBufferInCoordinates()//Recalculate current coordinates of robot   TODO: add angles to altitude q1
{
	float q1, q2, q3;
	q1 = 0;
	q2 = 0;
	q3 = 0;
	q1 = this->platform->getAngleInRadians()*stepperRadiansToMilimiters;//distance in z from max low position
	q2 = this->firstArm->getAngleInRadians();//angle of first arm
	q3 = getCorrectSecondArmAngleForForwardCinematic()*secondArmTransmitionRatio;//angle of second arm
	
	
	this->coordinates[0] = L1 * sin(q2) + L2 * cos(q2 - q3);//calculate X
	this->coordinates[1] = L1 * cos(q2) - L2 * sin(q2 - q3);//Y
	this->coordinates[2] = H + q1;//Z
}

void Robot::readSerial()//Incoming format byte-type,byte-speed,3 floats
{
	static int i = 0;
	static int numberOfBytes = 0;
	static int indexOfSpeed = 0;
	if (i == 0)//Counting number of needed bytes for cordinates or angles
	{
		while (allMotors[i] != NULL)
		{
			i++;
		}
		numberOfBytes = i * sizeof(float) +sizeof(float)+ sizeof(byte);//first byte for datatype second float speed
		indexOfSpeed = i;
	}

	if (Serial.available() == numberOfBytes)
	{
		byte incomingDataType;
		float speed;
		floatInBytes* floatNumbers = new floatInBytes[(numberOfBytes - sizeof(incomingDataType)) / sizeof(float)];
		for (int i = 0; i < (numberOfBytes - sizeof(incomingDataType)) / sizeof(float) + sizeof(incomingDataType); i++)//WORKS correctly
		{
			if (i == 0)incomingDataType = Serial.read();
			else
			{
				for (int j = 0; j < sizeof(float); j++)
				{
					floatNumbers[i-1].binary[j] = Serial.read();
				}
			}
		}
		blinker(3, 80);

		switch (incomingDataType)
		{
		case coordinatesQuery://WORKS
		{
			this->updateCurrentPositionBufferInCoordinates();//update current position in XYZ
			for (int i = 0; i < numberOfCoordinates; i++)
			{
				Serial.write((byte*)&this->coordinates[i], sizeof(float));
			}
			break;
		}
		case anglesQuery://WORKS
		{
			float value = -1;
			int i = 0;
			while (this->allMotors[i] != NULL)
			{

				value = this->allMotors[i]->getAngleInRadians();//returns angles in radians
				Serial.write((byte*)&value, sizeof(float));
				i++;
			}
			break;
		}
		case anglesInBuffersInRad:
		{
			float value = -1;
			int i = 0;
			while (this->allMotors[i] != NULL)
			{

				value = this->allMotors[i]->angleBufferRad;//returns angles in RAD
				Serial.write((byte*)&value, sizeof(float));
				i++;
			}

			break;
		}
		case destinationInRad://set angles and speeds buffers
		{
			int i = 0;
			//Check if any angle is less then 0 if so dont set any angle
			bool anyAngleLessThenZero = false;
			for (int i = 0; i < numberOfMotors; i++)
			{
				if (floatNumbers[i].number < 0)anyAngleLessThenZero = true;
			}
			if (anyAngleLessThenZero)
			{
				sendMessage("All angles have to be at least 0");
				break;
			}

			while (allMotors[i] != NULL)
			{

				allMotors[i]->setAngleInAllBuffers(floatNumbers[i].number);
				i++;
			}
			correctSecondArmAngleBuffer();
			float* speeds = this->calculateSpeedForAllMotors(floatNumbers[indexOfSpeed].number);
			this->setMotorSpeeds(speeds);
			delete[] speeds;

			break;
		}
		case destinationInCoordinatesCONCAVE:
		case destinationInCoordinatesCONVEX:
		{
			updateAnglesInBuffersFromCoordinates(floatNumbers, incomingDataType);//update buffers in radians in every motor
			float* speeds;
			speeds = this->calculateSpeedForAllMotors(floatNumbers[indexOfSpeed].number);
			this->setMotorSpeeds(speeds);
			delete[] speeds;
			break;
		}
		case goToDestinationInCoordinatesByLine:
		{
			goByLineTo(floatNumbers,floatNumbers[indexOfSpeed].number);
			break;
		}
		case goToAnglesInBuffers:
		{
			goToAnglesInBuffer();
			break;
		}
		case homingRequest:
		{
			homing();
			break;
		}
		case programN1:
		{
			program1();
			break;
		}
		case programN2:
		{
			digitalWrite(this->electroMagnetPin, !digitalRead(electroMagnetPin));
			break;
		}

		}
		delete[] floatNumbers;

	}

}


void Robot::matlabSetup()
{
	byte key = 255;
	Serial.write(key);
	while (1)
	{
		blinker(2, 500);
		if (Serial.available() == 1)
		{
			byte numberBuffer = Serial.read();
			if (numberBuffer == 5)
			{
				blinker(5);
				break;
			}
			else
			{
				blinker(3, 1000);
			}
		}
	}
}

void Robot::goToAnglesInBuffer()//////////////////////////////////////////////////////////////////////////UNCOMMENT THIS FUNCTION
{
	
	long int previousTime = 0;
	this->platform->prepareToMove();//TODO ADD SPEED COTROL OF STEPPER AND DC 
	do
	{
		if (millis() - previousTime > PeriodOfPIDInMili)
		{
			for (int i = 0; i < numberOfMotors; i++)//TODO CHANGE i=1 TO 0
			{
				this->allMotors[i]->updateControlValue();
			}
			previousTime = millis();
		}
		for (int i = 0; i < numberOfMotors; i++)//TO FO CHANGE i=0 To 0
		{
			this->allMotors[i]->applyControlValue();
		}


	} while (firstArm->motor->isMoving() || secondArm->motor->isMoving() || platform->motor->run());
	//for (int i = 1; i < numberOfMotors; i++)//TO GO CHANGE i=0 To 0
	//{
	
		this->platform->finishMovement();
	//}
	
}

float* Robot::inversCinematic(float X1, float Y, float Z,byte configuration)//Returns value of Q1-milimiters,Q2-rad,Q3-rad 
{
	float X = 0;
	if (X1 == 0)X = 0.000000001;
	else { X = X1; };
	float X2 = X * X;
	float Y2 = Y * Y;
	if (sqrt(X2+Y2) <= L1 + L2)
	{
		float* q = new float[3];//0-q1,1-q2,2-q3 

		float L22 = L2 * L2;
		float L12 = L1 * L1;
		if (configuration == CONCAVE)
		{
			q[1] = PI / 2.f - (acos((float)(-L22 + L12 + X2 + Y2) / (float)(2.f * L1*sqrt(X2 + Y2)))) - atan2(Y, X);
			q[2] = asin((X2 + Y2 - L12 - L22) / (2.f * L1*L2));
		}
		else
		{
			q[1] = PI / 2.f - atan2(Y, X) + acos((L22 - X2 - Y2 - L12) / (-2.f * sqrt(X2 + Y2)*L1));
			q[2] = PI - asin((X2 + Y2 - L12 - L22) / (2.f * L1*L2));
		}
		q[0] = Z - H;//In milimiters
		if (q[0] < maxq1 &&	q[0]>=0 && q[1]>=0 && q[1] < maxq2 && q[2] < maxq3 && q[2]>=0)return q;
	}
	sendMessage("Out of range...");
	return NULL;
}

void Robot::setMotorSpeeds(float* speeds)
{
	int i = 0;
	while (allMotors[i] != NULL)
	{
		allMotors[i]->setSpeedInAllBuffers(speeds[i]);
		i++;
	}

}

bool Robot::updateAnglesInBuffersFromCoordinates(floatInBytes* XYZ,byte configuration='E')
{
	float X, Y, Z;
	X = XYZ[0].number;
	Y = XYZ[1].number;
	Z = XYZ[2].number;
	
	float* q=this->inversCinematic(X, Y, Z, configuration);
	
	if (q != NULL)
	{
		this->platform->setAngleInAllBuffers(q[0] * stepperMilimitersToRadians);
		this->firstArm->setAngleInAllBuffers(q[1]);
		this->secondArm->setAngleInAllBuffers(q[2]*inversOfSecondArmTransmisionRation);
		correctSecondArmAngleBuffer();
		delete [] q;
		return true;
	}
	return false;

}

bool Robot::updateAnglesInBuffersFromCoordinates(float X,float Y,float Z, byte configuration = 'X')
{

	float* q = this->inversCinematic(X, Y, Z, configuration);

	if (q != NULL)
	{
		this->platform->setAngleInAllBuffers(q[0] * stepperMilimitersToRadians);
		this->firstArm->setAngleInAllBuffers(q[1]);
		this->secondArm->setAngleInAllBuffers(q[2] * inversOfSecondArmTransmisionRation);
		correctSecondArmAngleBuffer();
		delete[] q;
		return true;
	}
	return false;

}

void Robot::sendMessage(String message)
{
	blinker(TURNON);
	String messageToSend = message + "	SEND byte to continue<3...";
	Serial.println(messageToSend);
	while (Serial.available() != 1)
	{
	}
	Serial.read();
	blinker(TURNOFF);
}

float* Robot::calculateDelta(floatInBytes* destinationCoordinates)
{
	float* deltaXY = new float[2];
	float differenceInX, differenceInY;
	this->updateCurrentPositionBufferInCoordinates();
	differenceInX = destinationCoordinates[0].number - this->coordinates[0];
	differenceInY = destinationCoordinates[1].number - this->coordinates[1];
	float greaterDistance;
	if (differenceInX >= differenceInY)greaterDistance = differenceInX;
	else { greaterDistance = differenceInY; }
	numberOfPointsForLine = greaterDistance * numberOfPointsPerMilimiter;//Add one for last point witch is destination
	if (numberOfPointsForLine == 0)
	{
		sendMessage("Pass greater distance");
		delete[] deltaXY;
		return NULL;
	}
	deltaXY[0] = differenceInX / numberOfPointsForLine;
	deltaXY[1] = differenceInY / numberOfPointsForLine;

	return deltaXY;
}

void Robot::goByLineTo(floatInBytes* coordinates,float speed)
{
	float* deltaXY = calculateDelta(coordinates);//remeber to delete XY;
	if (deltaXY == NULL)return;
	float** pointsToGoThrough = new float*[numberOfPointsForLine];
	int numberOfDimensionsOfLine=2;
	float currentX = this->coordinates[0];
	float currentY=this-> coordinates[1];
	//Creating array for points

	for (int i = 0; i < numberOfPointsForLine; i++)
	{
		pointsToGoThrough[i] = new float[numberOfDimensionsOfLine];
		


		pointsToGoThrough[i][0] = currentX + deltaXY[0];
		pointsToGoThrough[i][1] = currentY + deltaXY[1];
		currentX += deltaXY[0];
		currentY += deltaXY[1];
	}

	delete[] deltaXY;

	bool outFlag = false;
	for (int i = 0; i < numberOfPointsForLine; i++)
	{
		floatInBytes* currentPointCoordinates = new floatInBytes[3];
		
		
		currentPointCoordinates[0].number = pointsToGoThrough[i][0];
		currentPointCoordinates[1].number = pointsToGoThrough[i][1];
		currentPointCoordinates[2].number = this->coordinates[2];
		
		if(!updateAnglesInBuffersFromCoordinates(currentPointCoordinates))outFlag=true;
		//calculateSpeedForAllMotors(speed);
		goToAnglesInBuffer();
		delete[] currentPointCoordinates;
		if (outFlag)break;

	}

	
	for (int i = 0; i < numberOfPointsForLine; i++)
	{
		delete[] pointsToGoThrough[i];
	}
	delete[] pointsToGoThrough;

}

void Robot::program1()
{
	
	updateAnglesInBuffersFromCoordinates(182, 210, 220);
	goToAnglesInBuffer();
	
	
	//First Point
	updateAnglesInBuffersFromCoordinates(330, 0, 220);

	float* speeds;
	speeds = this->calculateSpeedForAllMotors(0.5);
	this->setMotorSpeeds(speeds);
	delete[] speeds;

	goToAnglesInBuffer();
	digitalWrite(this->electroMagnetPin, HIGH);
	updateAnglesInBuffersFromCoordinates(330, 0, 193);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(330, 0, 220);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(240, 110, 220);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(240, 110, 195);
	goToAnglesInBuffer();
	digitalWrite(this->electroMagnetPin, LOW);

	//Second Point
	updateAnglesInBuffersFromCoordinates(350, 30, 220);
	goToAnglesInBuffer();
	digitalWrite(this->electroMagnetPin, HIGH);
	updateAnglesInBuffersFromCoordinates(350, 30, 195);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(350, 30, 220);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(240, 110, 220);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(240, 110, 195);
	goToAnglesInBuffer();
	digitalWrite(this->electroMagnetPin, LOW);

	//Third point

	updateAnglesInBuffersFromCoordinates(380, 85, 220);
	goToAnglesInBuffer();
	digitalWrite(this->electroMagnetPin, HIGH);
	updateAnglesInBuffersFromCoordinates(380, 85, 195);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(380, 85, 220);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(240, 110, 220);
	goToAnglesInBuffer();
	updateAnglesInBuffersFromCoordinates(240, 110, 195);
	goToAnglesInBuffer();
	digitalWrite(this->electroMagnetPin, LOW);

	updateAnglesInBuffersFromCoordinates(182, 210, 220);
	goToAnglesInBuffer();

	
}