#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED

#include <Wire.h>
#include <driver/i2c.h> //< I2C_NUM_* constants
#include <functional>


class I2C : public TwoWire
{
public:
    /**
     * Function called when a peripheral device receives a transmission from a controller device
     *
     * @param num_bytes Number of bytes read (received) from the controller device.
     */
    using OnReceive = void (*)(int num_bytes);
    /** Function called when a controller device requests data from a peripheral device */
    using OnRequest = void (*)();

    const uint32_t frequency;
    /** 7-bit I2C slave address or 0xFF if instance is in master mode */
    const uint8_t address;

    /**
     * Construct an I2C controller (master) driver instance
     *
     * The instance number is automatically assigned based on the number of instances created.
     *
     * @param sda       SDA pin number.
     * @param scl       SCL pin number.
     * @param freq_khz  Frequency of the I2C bus, in Hz..
     *
     * @see TwoWire::TwoWire
     * @see TwoWire::begin
     */
    I2C(int8_t sda, int8_t scl, uint32_t frequency = 400000)
        : TwoWire(instances[total_instances++]), frequency(frequency), address(0xFF)
    {
        assert(frequency > 1000); // Ensure that constructor with `address` is not mistakenly called
        this->sda  = sda;
        this->scl  = scl;
        bufferSize = 256;
    }

    /**
     * Construct an I2C peripheral (slave) driver instance
     *
     * The instance number is automatically assigned based on the number of instances created.
     *
     * @param sda         SDA pin number.
     * @param scl         SCL pin number.
     * @param address     7-bit I2C address of the peripheral device.
     * @param on_receive  See OnReceive.
     * @param on_request  See OnRequest.
     * @param freq_khz    Frequency of the I2C bus, in Hz.
     *
     * @see TwoWire::TwoWire
     * @see TwoWire::begin
     */
    I2C(int8_t sda, int8_t scl, uint8_t address, OnReceive on_receive, OnRequest on_request,
        uint32_t frequency = 400000)
        : TwoWire(instances[total_instances++]), frequency(frequency), address(address)
    {
        this->sda  = sda;
        this->scl  = scl;
        bufferSize = 256;
        onReceive(on_receive);
        onRequest(on_request);
    }


    void init()
    {
        bool ok = (address == 0xFF) ? TwoWire::begin(sda, scl, frequency)
                                    : TwoWire::begin(address, sda, scl, frequency);
        assert(ok);

        setTimeOut(50);
    }


private:
    static constexpr uint8_t instances[] = {I2C_NUM_0, I2C_NUM_1};
    static uint8_t total_instances;
};


#endif // I2C_H_INCLUDED
