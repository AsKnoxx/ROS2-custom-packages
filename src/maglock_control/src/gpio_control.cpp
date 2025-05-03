#include "gpio_control.h"

GPIOControl::GPIOControl(int mode)
{
	GPIO::setmode(mode);
}

GPIOControl::~GPIOControl()
{
	GPIO::cleanup();
}

void GPIOControl::setupPin(int pin, int direction)
{
	GPIO::setup(pin, direction);
	if (direction == GPIO::OUT)
		pinStates_[pin] = false;
}

void GPIOControl::setHigh(int pin)
{
	GPIO::output(pin, GPIO::HIGH);
	pinStates_[pin] = true;
}

void GPIOControl::setLow(int pin)
{
	GPIO::output(pin, GPIO::LOW);
	pinStates_[pin] = false;
}

void toggle(int pin)
{
	if (pinStates_[pin] == false)
		setHigh(pin);
	else
		setLow(pin);
}

bool GPIOControl::getState(int pin) const
{
	return pinStates_[pin];
}
