#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include <JetsonGPIO.h>
#include <map>

class GPIOControl
{
	public:
		enum NumberingMode
		{
			BOARD = GPIO::BOARD, 
			BCM = GPIO::BCM,
			CVM = GPIO::CVM,
			TEGRA = GPIO::TEGRA_SOC
		};

		enum Direction
		{
			In = GPIO::INPUT,
			Out = GPIO::OUTPUT
		};

		// Constructs an object that uses board numbering method by default
		GPIOControl(NumberingMode mode = BOARD);

		// Destructor that cleans up all GPIO pins
		~GPIOControl();

		// Checks the current numbering mode
		void checkNumMode();

		// Sets a pin to either input or output
		void setupPin(int pin, Direction direction);

		// Sets a pin to HIGH
		void setHigh(int pin);

		// Sets a pin to LOW
		void setLow(int pin);

		// Switches a pin from HIGH to LOW and vice versa
		void toggle(int pin);

		// Reads the pin's state
		bool readPinState(int pin) const;

	private:
		std::map<int, bool> pinStates_;
		std::map<int, Direction> pinDirection_;
		NumberingMode mode_;
};

#endif
