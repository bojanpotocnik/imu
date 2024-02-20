#include <Arduino.h>
#include <Wire.h>
#include "imu.hpp"


/**
 * Initialize the serial port for `printf` usage
 *
 * @param baud        Baud rate for the serial port.
 * @param usb_timeout Timeout in milliseconds to wait for the USB CDC serial connection to open
 *                    on boards with native USB (otherwise this is ignored).
 * @param usb_delay   Delay in milliseconds to wait after the USB CDC serial connection is open.
 */
static void init_serial(unsigned int baud = 921600, unsigned int usb_timeout = 1000, unsigned int usb_delay = 300);

/**
 * Initialize the I2C bus and optionally scan it for attached devices
 *
 * @param scl        Pin number for the I2C SCL line.
 * @param sda        Pin number for the I2C SDA line.
 * @param frequency  I2C bus frequency in Hz.
 * @param scan       If true, scan the I2C bus for attached devices.
 */
static void init_i2c(uint8_t scl, uint8_t sda, uint32_t frequency, bool scan = false);


/** IMU sensor instance */
static IMU imu;


void setup()
{
    init_serial();
    init_i2c(44, 43, 400000, true);

    imu.begin();
#ifdef DEBUG
    imu.printSensorTable();
#endif
    imu.enableSensor(IMU::OrientationSensor::ROTATION_VECTOR, 1);

    printf("Setup complete\n");
}

void loop()
{
    static uint32_t t_last_imu = 0;
    const uint32_t t_now = millis(); /*< Ensure everything in this loop iteration is in the same "tick" */

    /* Handle pending sensor data */
    imu.handle();

    if (Serial.available() && Serial.read() == 't')
    {
        printf("Taring IMU\n");
        imu.tareNow(false, SH2_TARE_BASIS_ROTATION_VECTOR);
    }

    /* Process */

    if ((t_now - t_last_imu) >= 100)
    {
        float yaw, pitch, roll;
        t_last_imu = t_now;

        if (imu.getEuler(yaw, pitch, roll))
        {
            printf("Euler angles: %f, %f, %f\n", yaw, pitch, roll);
        }
    }
}


static void init_serial(const unsigned int baud, const unsigned int usb_timeout, const unsigned int usb_delay)
{
    Serial.begin(baud);

#if ARDUINO_USB_MODE
    /* Wait for USB CDC serial connection to open on boards with native USB (otherwise this is immediately true),
     * but use a timeout to prevent blocking if the USB is not attached at all. */
    for (const unsigned int t_start = millis(); (millis() - t_start) < usb_timeout;)
    {
        if (Serial)
        {
            /* Wait a bit more for the monitor to open the serial port (~250 ms seems to work well) */
            if (usb_delay > 0)
            {
                delay(usb_delay);
            }
            break;
        }
    }
#endif /* ARDUINO_USB_MODE */
}

static void init_i2c(uint8_t scl, uint8_t sda, uint32_t frequency, bool scan)
{
    Wire.begin(sda, scl, frequency);

    if (scan)
    {
        printf("I2C scan...\n");

        for (uint8_t address = 0; address <= (0xFF >> 1); address++)
        {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0)
            {
                printf("I2C device found at 0x%02x\n", address);
            }
        }

        printf("I2C scan done\n");
    }
}
