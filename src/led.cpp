#include "led.hpp"

led::led(const int PIN) : _pin(PIN)
{
    gpio_init(PIN);
    gpio_set_dir(PIN, true);
}

void led::set(bool setVal)
{
    gpio_put(_pin, setVal);
}