#pragma once
#include "Parameters.h"
#include "Arduino.h"
#include "Global.h"
typedef union
{
	float number;
	byte binary[4];
}floatInBytes;


void dc1CounterFirstInterruption();
void dc1CounterSecondInterruption();
void dc2CounterFirstInterruption();
void dc2CounterSecondInterruption();

void blinker(int numberOfBlinks=2, int pulseDuration = 200);
void doEveryPeriod(void(*funcionToDo)(int, int), int periodInMilliSec=300);
