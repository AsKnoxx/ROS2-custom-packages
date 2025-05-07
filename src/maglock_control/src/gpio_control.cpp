#include "gpio_control.h"

GPIOControl::GPIOControl(NumberingMode mode)
{
	GPIO::setmode(mode);
	mode_ = mode;
}

GPIOControl::~GPIOControl()
{
	GPIO::cleanup();
}

void GPIOControl::checkNumMode()
{
	switch (mode_)
	{
	case BCM:
		std::cout << "BCM numbering mode set\n";
		return;
	case CVM:
		std::cout << "CVM numbering mode set\n";
		return;
	case TEGRA:
		std::cout << "TEGRA_SOC numbering mode set\n";
		return;
	default:
		std::cout << "BOARD numbering mode set\n";
		return;
	}
}

void GPIOControl::setupPin(int pin, Direction direction)
{
	GPIO::setup(pin, direction);

	// Tracks the state and direction for output  pins and tracks just direction
	// for input pins
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
	if (!pinDirection_.contains(pin))
	{
		std::cerr << "Error: Pin " << pin << " has not been configured.\n";
		return;
	}
	else if (pinDirection_[pin] == In)
	{
		std::cerr << "Error: Pin " << pin << " is an input pin.\n";
		return;
	}

	GPIO::output(pin, GPIO::HIGH);
	pinStates_[pin] = true;
}

void GPIOControl::setLow(int pin)
{
	if (!pinDirection_.contains(pin))
	{
		std::cerr << "Error: Pin " << pin << " has not been configured.\n";
		return;
	}
	else if (pinDirection_[pin] == In)
	{
		std::cerr << "Error: Pin " << pin << " is an input pin.\n";
		return;
	}

	GPIO::output(pin, GPIO::LOW);
	pinStates_[pin] = false;
}

void GPIOControl::toggle(int pin)
{
	if (!pinDirection_.contains(pin))
	{
		std::cerr << "Error: Pin " << pin << " has not been configured.\n";
		return;
	}
	else if (pinDirection_[pin] == In)
	{
		std::cerr << "Error: Pin " << pin << " is an input pin.\n";
		return;
	}
	pinStates_[pin] ? setLow(pin) : setHigh(pin);
}


bool GPIOControl::readPinState(int pin) const
{
	// Checks if the pin has not been configured
	if (pinDirection_.find(pin) == pinDirection_.end())
	{
		std::cerr << "Error: Pin " << pin << " has not been configured.\n";
		return false;
	}
	else if (pinDirection_[pin] == In)
		return GPIO::input(pin);
	return pinStates_.at(pin);
}
