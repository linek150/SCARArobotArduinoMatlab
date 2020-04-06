#pragma once
#ifndef PAPAMETERS
#define PARAMETERS
/*
If LED is ON there is message to be read via usb point and Arduino is waiting for one character

One rotation of stepper is 8mm 
q1-hight from lowest position 
q2-angle from y axis for first arm
q3- angle from 90 wiht first arm for second arm

*/
#define correctionForSecondArm 0.9764
#define L1 210.f
#define L2 182.f
#define H 185.f
#define minSpeedOfDcMotor  30
#define PredutionForDCMotor 0.1
#define secondArmTransmitionRatio 0.29032258064
#define inversOfSecondArmTransmisionRation 3.444444444
#define stepperRadiansToMilimiters 1.273239544
#define stepperMilimitersToRadians 0.785398

#define microSteps 8

#define maxq3 3.490658504//In rad
#define maxq2 2.750597//In rad
#define maxq1 60//In milimiters

#define numberOfPointsPerMilimiter 2.5

//Max Speed of motors
#define maxStepperSpeed  500//for a4988
#define maxStepperSpeedRadPS 52.3598//rad/s
#define stepperAcceleration 500
#define maxDCSpeedRadPS 1.570796//rad/s

//DC1 PINS
#define dc1EncoderFirstPin 2 //Yellow
#define dc1EncoderSecondPin 3 //Green
#define dc1LimitSwitchPin 50 //normally 5V
#define dc1Dir1Pin 26 //Orange
#define dc1Dir2Pin 27 //Yellow
#define dc1PWMPin 9 //Red
//DC2 PIN
#define dc2EncoderFirstPin 18//Orange
#define dc2EncoderSecondPin 19//Green
#define dc2LimitSwitchPin 51//normally 5V
#define dc2Dir1Pin 36//Red
#define dc2Dir2Pin 37//White
#define dc2PWMPin 10//Blue

//STEPPER PIN

#define stepperDirPin 22//Blue
#define stepperStepPin  23//White
#define stepperLimitSwitchPin 52//normally 5V
#define stepperEnablePin 53//HIGH turns off stepper
//Grey encoder power 5V arduino

#define electromagnetPin 49

//Various needed value
#define numberOfCoordinates 3
#define PPD 18.333333333//pulses of dc encoder  per degree
#define OneAndAHalfDegreeInPulses 27
#define PPR 1050.4226244//pulses per redian of dc motor
#define DPS 1.8 //degree per step of stepper
#define SPR 31.830988//Steps per radian
#define TURNON -1
#define TURNOFF -2
#define PeriodOfPIDInMili 20
//Serial action types
#define goToAnglesInBuffers 'g'
#define coordinatesQuery 'p'//Returns 3 int X,Y,Z in milimiters 
#define anglesQuery 'a'//Return 3 floats degStepper,degDc1,degDc2 in degrees
#define anglesInBuffersInRad 'R'//Returns 3 floats degStepper,degDc1,degDc2 in buffer in Rad
#define destinationInRad 'r'
#define goToDestinationInCoordinatesByLine 'l'
#define homingRequest 'h'
#define destinationInCoordinatesCONCAVE 'E'
#define CONCAVE 'E'
/*
y
^ |||||||||||
| |||||||||||
| ||||				THIS IS CONCAVE
| ||||
| ||||
| ||||
------------------>X
*/
#define destinationInCoordinatesCONVEX 'X'
#define CONVEX 'X'
#define programN1 '1'
#define programN2 '2'
#endif // !PAPAMETERS