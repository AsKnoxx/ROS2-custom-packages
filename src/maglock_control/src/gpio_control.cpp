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

void GPIOControl::toggle(int pin)
{
	if (pinStates_.find(pin) == pinStates_.end())
		return;

	pinStates_[pin] ? setLow(pin) : setHigh(pin);
}

bool GPIOControl::getState(int pin) const
{
	if (pinStates_.find(pin) == pinStates_.end())
		return false;
	return pinStates_[pin];
}
