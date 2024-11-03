#include "pico/stdlib.h"

class led
{   
public:

    led(const int PIN);

    ~led(){}

    void set(bool setVal);

    const int _pin;
};