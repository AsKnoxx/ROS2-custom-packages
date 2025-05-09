#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include <JetsonGPIO.h>
#include <map>

class GPIOControl
{
public:
    enum NumberingMode
    {
        BOARD = static_cast<int>(GPIO::BOARD),
        BCM = static_cast<int>(GPIO::BCM),
        CVM = static_cast<int>(GPIO::CVM),
        TEGRA = static_cast<int>(GPIO::TEGRA_SOC)
    };

    enum Direction
    {
        In = static_cast<int>(GPIO::IN),
        Out = static_cast<int>(GPIO::OUT)
    };

    // Constructs an object that uses board numbering method by default
    GPIOControl(NumberingMode mode = BOARD);

    // Destructor that cleans up all GPIO pins
    ~GPIOControl();

    // Prints the current numbering mode
    void printNumMode();

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

    // Prints the pin as output or input
    void pinFunction(int pin);

private:
    std::map<int, bool> pinStates_;
    std::map<int, Direction> pinDirection_;
    NumberingMode mode_;
};

#endif
