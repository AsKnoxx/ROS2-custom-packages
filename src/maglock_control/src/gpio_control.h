#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include <JetsonGPIO.h>
#include <map>

class GPIOControl
{
	public:
		// Constructs an object that uses board numbering method
		GPIOControl(int mode = GPIO::BOARD);

		// Destructor that cleans up all GPIO pins
		~GPIOControl();

		// Sets a pin to either input or output
		void setupPin(int pin, int direction);

		// Sets a pin to HIGH
		void setHigh(int pin);

		// Sets a pin to LOW
		void setLow(int pin);

		// Switches a pin from HIGH to LOW and vice versa
		void toggle(int pin);

		// Gets the pin's state
		void getState(int pin) const;

	private:
		std::map<int, bool> pinStates_;
};

#endif
