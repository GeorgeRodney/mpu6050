#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <string>
#include <vector>
#include <cstdio>

class WIFI
{
public:
    WIFI()

    ~WIFI()
    const char *SSID;
    const char *PASSWORD;
    const char *SERVER_IP;
    const int SERVER_PORT;

    void connect_to_wifi();
}