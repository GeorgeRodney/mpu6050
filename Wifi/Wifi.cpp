#include <stdio.h>
#include "pico/cyw43_arch.h" // Required for Wi-Fi functions

class WIFI
{

    public:
        WIFI();

        ~WIFI();
        const char *SSID = "SETUP-1D9B";
        const char *PASSWORD = "coast9460animal";
        const char *SERVER_IP = "127.0.0.1";
        const int SERVER_PORT = 65432;

        // Function to connect to WiFi
        void connect_to_wifi() {
            if (cyw43_arch_init()) {
                printf("Failed to initialize WiFi\n");
                return;
            }

            cyw43_arch_enable_sta_mode();
            printf("Connecting to WiFi...\n");
            if (cyw43_arch_wifi_connect_blocking(SSID, PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK)) {
                printf("Failed to connect to WiFi\n");
                return;
            }

            printf("Connected to SETUP-1D9B!\n");
        }
};