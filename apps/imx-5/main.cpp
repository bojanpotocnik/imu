#include <Arduino.h>
#include <array>

#include "i2c.h"
#include "imx_imu.h"
#include "ins.h"
#include "uart.h"


/** Initialize the serial port for `printf` usage */
static void init_printf();

static void on_iis_receive(int num_bytes);
static void on_iis_request();


// Note: use crossover cables (TX/RX swapped on one end, so that TX goes to RX and vice versa)
static std::array<UART, 2> uarts = {
    // UART0 (RX=D8, TX=D7) also prints debug messages, so avoid it for now
    UART(3, 2, 921600, 1, 4096, 256), //< UART1 (RX=D2, TX=D1)
    UART(5, 4, 921600, 100),          //< UART2 (RX=D4, TX=D3)
};

// Note: use normal cables (SDA goes to SDA, SCL to SCL)
static I2C iim = I2C(9, 8, 400000);   //< I2C master connected to SDA=D10, SCL=D9
static I2C iis = I2C(6, 43, 0x69, on_iis_receive,
                     on_iis_request); //< I2C slave with address 0x69 connected to SDA=D5, SCL=D6

/** IMX-5 IMU connected to UART1 */
static IMX imx(uarts[0]);


static bool imx_setup()
{
    return imx.init() &&                                       // Initialization disables all data
           imx.enableData(IMX::DataSet::SYSTEM, 1000) &&       // Temperature and other sensors
           imx.enableData(IMX::DataSet::INS_AHRS_EULER, 10) && // AHRS at 100 Hz
           imx.enableData(IMX::DataSet::IMU, 1);               // IMU data at 1000 Hz
}

void setup()
{
    init_printf();

    pinMode(1, INPUT_PULLDOWN); // GPIO1 (D0) is connected to the optional 1.27 mm pin header

    for (auto &uart : uarts) {
        uart.init();
    }

    iim.init();
    iis.init();

    assert(imx_setup());

    log_i("setup OK");
}


void loop()
{
    const int parsed = imx.loop();

    if (imx.timeoutOccurred) {
        if (!imx_setup()) {
            return;
        }
        imx.timeoutOccurred = false;
    }

    if (parsed > 0) {
        log_d("YPR 08%x, %6.1f %6.1f %6.1f deg || "
              "IMU 08%x %5.1f/%5.1f | %5.1f/%5.1f | %5.1f/%5.1f m/s2 || "
              "n=%d",
              imx.d.ins.insStatus,
              imx.d.ins.theta[2] * C_RAD2DEG_F,
              imx.d.ins.theta[1] * C_RAD2DEG_F,
              imx.d.ins.theta[0] * C_RAD2DEG_F,
              imx.imu.last.status,
              imx.imu.min.acc[0],
              imx.imu.max.acc[0],
              imx.imu.min.acc[1],
              imx.imu.max.acc[1],
              imx.imu.min.acc[2],
              imx.imu.max.acc[2],
              parsed);
    }
    else if (parsed < 0) {
        log_v("Parsed %d", parsed);
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
    log_v("I2C-C sent %d B", num_bytes);
}

static void on_iis_request()
{
    log_v("I2C-C requesting data");

    INS::sensor_data_t data{
        .hdwStatus = imx.d.ins.hdwStatus,
        .imuStatus = imx.imu.last.status,
        .insStatus = imx.d.ins.insStatus,
        .ahrs{.yaw   = imx.d.ins.theta[2] * C_RAD2DEG_F,
              .pitch = imx.d.ins.theta[1] * C_RAD2DEG_F,
              .roll  = imx.d.ins.theta[0] * C_RAD2DEG_F},
        .accel{.min{.x = imx.imu.min.acc[0], .y = imx.imu.min.acc[1], .z = imx.imu.min.acc[2]},
              .max{.x = imx.imu.max.acc[0], .y = imx.imu.max.acc[1], .z = imx.imu.max.acc[2]}}
    };
    imx.imu.reset();

    const size_t n_w = iis.write(reinterpret_cast<uint8_t *>(&data), sizeof(data));
    if (n_w != sizeof(data)) {
        log_e("Wrote %d of %d B", n_w, sizeof(data));
    }
    else {
        log_v("Wrote %d", n_w);
    }
}
