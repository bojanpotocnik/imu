#include <Arduino.h>
#include <array>

#include "i2c.h"
#include "uart.h"
#include "imx_imu.h"


/** Initialize the serial port for `printf` usage */
static void init_printf();

static void on_iis_receive(int num_bytes);
static void on_iis_request();


// Note: use crossover cables (TX/RX swapped on one end, so that TX goes to RX and vice versa)
static std::array<UART, 2> uarts = {
    // UART0 (RX=D8, TX=D7) also prints debug messages, so avoid it for now
    UART(3, 2, 921600),  //< UART1 (RX=D2, TX=D1)
    UART(5, 4, 921600),  //< UART2 (RX=D4, TX=D3)
};

// Note: use normal cables (SDA goes to SDA, SCL to SCL)
static I2C iim = I2C(9, 8, 400000);   //< I2C master connected to SDA=D10, SCL=D9
static I2C iis = I2C(6, 43, 0x69, on_iis_receive,
                     on_iis_request); //< I2C slave with address 0x69 connected to SDA=D5, SCL=D6

/** IMX-5 IMU connected to UART1 */
static IMX imx(uarts[0]);

void setup()
{
    init_printf();

    pinMode(1, INPUT_PULLDOWN); // GPIO1 (D0) is connected to the optional 1.27 mm pin header

    for (auto &uart : uarts) {
        uart.init();
    }

    iim.init();
    iis.init();

    imx.init();

    log_i("Setup complete");
}


void loop()
{
    static uint32_t t_last_imu = 0;
    /* Ensure everything in this loop iteration is in the same "tick" */
    const uint32_t t_now       = millis();
    const uint32_t t_now_u     = micros();

    imx.loop();

    if ((t_now - t_last_imu) >= 1000) {
        t_last_imu = t_now;
        printf("t_now: %u ms, %u us\n", t_now, t_now_u);
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

static void on_iis_receive(int num_bytes)
{
    printf("on_iis_receive(%d)\n", num_bytes);
}

static void on_iis_request()
{
    printf("on_iis_request\n");
    iis.write("Hello!");
}
