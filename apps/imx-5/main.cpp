#include <Arduino.h>
#include <array>

#include "uart.h"


/** Initialize the serial port for `printf` usage */
static void init_printf();


static std::array<UART, 3> uarts = {
    UART(2, 1, 1500000),  //< UART connected to RX=D1, TX=D0
    UART(4, 3, 1500000),  //< UART connected to RX=D3, TX=D2
    UART(44, 43, 1500000) //< UART connected to RX=D7, TX=D6
};


void setup()
{
    init_printf();

    for (auto &uart : uarts) {
        uart.init();
    }

    printf("Setup complete\n");
}


void loop()
{
    static uint32_t t_last_imu = 0;
    /* Ensure everything in this loop iteration is in the same "tick" */
    const uint32_t t_now       = millis();
    const uint32_t t_now_u     = micros();

    if ((t_now - t_last_imu) >= 1000) {
        t_last_imu = t_now;
        printf("t_now: %u ms, %u us\n", t_now, t_now_u);
        for (auto &uart : uarts) {
            uart.printf("[%u][%lu]\n", uart.instance, micros());
        }
    }

    for (auto &uart : uarts) {
        if (uart.available()) {
            const auto rx = uart.readString();
            uart.printf("[%u][%lu] '%s'\n", uart.instance, micros(), rx.c_str());
        }
    }
}


static void init_printf()
{
    /* Use `build_flags = -D MONITOR_SPEED=${this.monitor_speed}` from platformio.ini,
     * so that the baud rate is always set to the same value in both places. */
    Serial.begin(MONITOR_SPEED);

#if ARDUINO_USB_MODE
    /* Wait for USB peripheral to enumerate, and USB CDC serial connection to
     * open on boards with native USB (otherwise Serial is immediately true),
     * but use a timeout to prevent blocking if the USB is not attached at all. */
    for (const unsigned int t_start = millis(); (millis() - t_start) < 1000;) {
        if (Serial) {
            /* Wait a bit more for the monitor to open the serial port
             * (~250 ms seems to work well on few tested computers). */
            delay(300);
            break;
        }
    }
#endif /* ARDUINO_USB_MODE */
}
