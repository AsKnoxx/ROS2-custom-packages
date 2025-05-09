#include "gpio_control/gpio_control.h"
#include <iostream>

GPIOControl::GPIOControl(NumberingMode mode)
{
    // Cast the int-based enum back to the JetsonGPIO enum class
    GPIO::setmode(static_cast<GPIO::NumberingModes>(mode));
    mode_ = mode;
}

GPIOControl::~GPIOControl()
{
    GPIO::cleanup();
}

void GPIOControl::printNumMode()
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
    // Map Direction enum to GPIO::IN and GPIO::OUT
    if (direction == Out)
    {
        GPIO::setup(pin, GPIO::OUT);  // Output mode
    }
    else if (direction == In)
    {
        GPIO::setup(pin, GPIO::IN);   // Input mode
    }

    // Track the state and direction for output pins and just direction for input pins
    if (direction == Out)
    {
        pinStates_[pin] = false;  // Default state for output pins
        pinDirection_[pin] = Out;
    }
    else if (direction == In)
    {
        pinDirection_[pin] = In;
    }
}


void GPIOControl::setHigh(int pin)
{
    if (pinDirection_.find(pin) == pinDirection_.end())
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
    if (pinDirection_.find(pin) == pinDirection_.end())
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
    if (pinDirection_.find(pin) == pinDirection_.end())
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
    if (pinDirection_.find(pin) == pinDirection_.end())
    {
        std::cerr << "Error: Pin " << pin << " has not been configured.\n";
        return false;
    }
    else if (pinDirection_.at(pin) == In)
    {
        return GPIO::input(pin);
    }

    return pinStates_.at(pin);
}

void GPIOControl::pinFunction(int pin)
{
    if (pinDirection_.find(pin) == pinDirection_.end())
    {
        std::cerr << "Error: Pin " << pin << " has not been configured.\n";
        return;
    }
    else if (pinDirection_[pin] == In)
    {
        std::cout << "Pin " << pin << " is an input pin.\n";
        return;
    }
    std::cout << "Pin " << pin << " is an output pin.\n";
}
