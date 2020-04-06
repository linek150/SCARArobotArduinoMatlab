#include "Others.h"



void dc1CounterFirstInterruption()
{
	if ((digitalRead(dc1EncoderSecondPin) == LOW && digitalRead(dc1EncoderFirstPin) == HIGH) || (digitalRead(dc1EncoderSecondPin) == HIGH && digitalRead(dc1EncoderFirstPin) == LOW))
	{
		dc1Counter++;
	}
	else
	{
		dc1Counter--;
	}
}
void dc1CounterSecondInterruption()
{
	if ((digitalRead(dc1EncoderSecondPin) == LOW && digitalRead(dc1EncoderFirstPin) == LOW) || (digitalRead(dc1EncoderSecondPin) == HIGH && digitalRead(dc1EncoderFirstPin) == HIGH))
	{
		dc1Counter++;
	}
	else
	{
		dc1Counter--;
	}
}
void dc2CounterFirstInterruption()
{
	if ((digitalRead(dc2EncoderSecondPin) == LOW && digitalRead(dc2EncoderFirstPin) == HIGH) || (digitalRead(dc2EncoderSecondPin) == HIGH && digitalRead(dc2EncoderFirstPin) == LOW))
	{
		dc2Counter++;
	}
	else
	{
		dc2Counter--;
	}
}
void dc2CounterSecondInterruption()
{
	if ((digitalRead(dc2EncoderSecondPin) == LOW && digitalRead(dc2EncoderFirstPin) == LOW) || (digitalRead(dc2EncoderSecondPin) == HIGH && digitalRead(dc2EncoderFirstPin) == HIGH))
	{
		dc2Counter++;
	}
	else
	{
		dc2Counter--;
	}
}
void blinker(int numberOfBlinks=2, int pulseDuration = 200)
{
	if (numberOfBlinks == TURNON)
	{
		digitalWrite(13, HIGH);
	}
	if (numberOfBlinks == TURNOFF)
	{
		digitalWrite(13, LOW);
	}
	for (; numberOfBlinks > 0; numberOfBlinks--)
	{
		digitalWrite(13, HIGH);
		delay(pulseDuration);
		digitalWrite(13, LOW);
		delay(pulseDuration);
	}
}

void doEveryPeriod(void(*funcionToDo)(int,int), int periodInMilliSec=300)
{
	static long previousTime = 0;
	if (millis() - previousTime > periodInMilliSec)
	{
		funcionToDo(2,200);
		previousTime = millis();
	}
}