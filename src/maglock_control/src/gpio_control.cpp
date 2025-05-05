#include "gpio_control.h"

GPIOControl::GPIOControl(NumberingMode mode)
{
	GPIO::setmode(mode);
}

GPIOControl::~GPIOControl()
{
	GPIO::cleanup();
}

void GPIOControl::setupPin(int pin, Direction direction)
{
	GPIO::setup(pin, direction);
	if (direction == Out)
	{
		pinStates_[pin] = false;
		pinDirection_[pin] = Out;
	}
	else if (direction == In)
	{
		pinDirection_[pin] = In;
	}
}

void GPIOControl::setHigh(int pin)
{
	if (!pinDirection_.contains(pin) || pinDirection_[pin] == In)
		return;

	GPIO::output(pin, GPIO::HIGH);
	pinStates_[pin] = true;
}

void GPIOControl::setLow(int pin)
{
	if (!pinDirection_.contains(pin) || pinDirection_[pin] == In)
		return;

	GPIO::output(pin, GPIO::LOW);
	pinStates_[pin] = false;
}

void GPIOControl::toggle(int pin)
{
	if (!pinDirection_.contains(pin) || pinDirection_[pin] == In)
		return;

	pinStates_[pin] ? setLow(pin) : setHigh(pin);
}


bool GPIOControl::readPinState(int pin) const
{
	if (pinDirection_[pin] == In)
		return GPIO::input(pin);
	return pinStates_.at(pin);
}
