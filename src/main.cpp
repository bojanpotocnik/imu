#include <Arduino.h>
#include <Wire.h>


/**
 * When debugging, wait a bit after boot to allow the serial monitor to be ready
 *
 * @param delay_ms  Delay in milliseconds from first call to this function to it returning false.
 *
 * @return True if the delay is still ongoing, false when the specified delay has passed.
 */
static bool boot_delay(unsigned int delay_ms = 500);

/**
 * Scan for I2C devices, one address each call to prevent blocking the main loop
 *
 * @param reset  Reset the address to 0x00.
 *
 * @return True if the scan is still ongoing, false when the scan is complete.
 */
static bool i2c_scan_loop(bool reset = false);


static const int PIN_SDA = 43;
static const int PIN_SCL = 44;


void setup()
{
    Serial.begin(921600);
    Wire.begin(PIN_SDA, PIN_SCL, 400000);
}

void loop()
{
    if (boot_delay())
    {
        return;
    }

    if (i2c_scan_loop())
    {
        return;
    }
    delay(1000);
    i2c_scan_loop(true);
}


static bool boot_delay(const unsigned int delay_ms)
{
#if DEBUG
    static unsigned long t_boot = millis();

    if ((millis() - t_boot) < delay_ms)
    {
        return true;
    }
#endif
    return false;
}


static bool i2c_scan_loop(const bool reset)
{
    static uint8_t address = 0;

    if (reset)
    {
        address = 0;
    }

    if (address == 0)
    {
        Serial.print("I2C scan...\n");
    }
    else if (address >= 127)
    {
        return false;
    }

    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
        Serial.printf("I2C device found at 0x%02x ", address);
    }
    address++;

    if (address >= 127)
    {
        Serial.print("I2C scan done\n");
        return false;
    }

    return true;
}
