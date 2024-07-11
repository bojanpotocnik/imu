#include <Arduino.h>


/** Initialize the serial port for `printf` usage */
static void init_serial();


void setup()
{
    init_serial();

    printf("Setup complete\n");
}

void loop()
{
    static uint32_t t_last_imu = 0;
    /* Ensure everything in this loop iteration is in the same "tick" */
    const uint32_t t_now       = millis();

    if ((t_now - t_last_imu) >= 200) {
        printf("t_now=%u\n", t_now);
        t_last_imu = t_now;
    }
}


static void init_serial()
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
