#include "gpio_control.h"

GPIOControl::GPIOControl(int pin) : pinNumber_(pin)
{
	GPIO::setmode(GPIO::BOARD);
	GPIO::setup(pinNumber_, GPIO::OUT);
}

GPIOControl::~GPIOControl()
{
	GPIO::cleanup(pinNumber_);
}

void GPIOControl::setHigh()
{
	GPIO::output(pinNumber_, GPIO::HIGH);
	currentPinState_ = true;
}

void GPIOControl::setLow()
{
	GPIO::output(pinNumber_, GPIO::LOW);
	currentPinState_ = false;
}

bool GPIOControl::getPinState() const
{
	return currentPinState_;
}

void GPIOControl::togglePinState()
{
	// Toggle the pin: if HIGH, set to LOW, otherwise HIGH
	getPinState() ? setLow() : setHigh();
}
