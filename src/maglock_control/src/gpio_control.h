#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include <JetsonGPIO.h>

class GPIOControl
{
	public:
		// Constructs an object for a given pin
		GPIOControl(int pin);

		// Destructor that cleans up the GPIO pin
		~GPIOControl();

		// Sets the GPIO pin to HIGH
		void setHigh();

		// Sets the GPIO pin to LOW
		void setLow();

		// Returns the current state of the pin
		bool getPinState() const;

		// Toggles the pin state from HIGH to LOW and vice versa
		void togglePinState();

	private:
		int pinNumber_;
		bool currentPinState_;
};

#endif
