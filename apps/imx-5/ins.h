#ifndef INS_H_INCLUDED
#define INS_H_INCLUDED

#include <Wire.h>


class INS
{
public:
    /** Structure to hold 3D vector data */
    typedef struct __attribute__((packed)) {
        /**
         * Longitudinal (back to front, X) axis - points forward
         *
         * Right-hand rotation around this axis, facing forward (+), increases the roll angle.
         */
        float x;
        /**
         * Lateral (left to right, Y) axis - points to the right
         *
         * Right-hand rotation around this axis, facing right (+), increases the pitch angle.
         */
        float y;
        /**
         * Vertical (up to down, Z) axis - points down
         *
         * Right-hand rotation around this axis, facing down (+), increases the yaw angle.
         */
        float z;
    } vec3_t;

    /** Structure to hold IMU data */
    typedef struct __attribute__((packed)) {
        /** Hardware status flags (`eHdwStatusFlags`) or 0 if sensor is not operational */
        uint32_t hdwStatus;
        /** IMU Status (`eImuStatus`) or 0 if sensor is not operational */
        uint32_t imuStatus;
        /** INS status flags (`eInsStatusFlags`) or 0 if sensor is not operational */
        uint32_t insStatus;
        /**
         * Attitude and Heading Reference System (AHRS) data as Tait–Bryan (Euler) angles
         */
        struct __attribute__((packed)) {
            /**
             * Yaw (heading) in range from -180 to 180 degrees
             *
             * Yaw increases with right-hand (clockwise) rotation around the
             * Z axis, facing the positive direction of the Z axis (down).
             *
             *     0° is magnetic north or front,
             *    90° is east (clockwise rotation around Z),
             *   -90° is west (counter-clockwise rotation around Z),
             * +-180° is south (backwards).
             */
            float yaw;
            /**
             * Pitch (inclination, tilt) in range from -180 to 180 degrees
             *
             * Pitch increases with right-hand (clockwise) rotation around the
             * Y axis, facing the positive direction of the Y axis (right).
             *
             *     0° is level,
             *    90° is nose up,
             *   -90° is nose down,
             * +-180° is upside down (flipped).
             */
            float pitch;
            /**
             * Roll (bank, side tilt) in range from -180 to 180 degrees
             *
             * Roll increases with right-hand (clockwise) rotation around the
             * X axis, facing the positive direction of the X axis (forward).
             *
             *    0° is level,
             *   90° is tilted to the right side,
             *  -90° is tilted to the left side.
             *
             * Note that the roll range is only 90° to avoid gimbal lock,
             * and yaw and pitch will be changed to represent what would
             * be the actual orientation if the roll range was +-180°.
             */
            float roll;
        } ahrs;

        /**
         * Inertial Measurement Unit (IMU) accelerometer data
         */
        struct __attribute__((packed)) {
            /** Minimum acceleration (m/s^2) since last read */
            vec3_t min;
            /** Maximum acceleration (m/s^2) since last read */
            vec3_t max;
        } accel;
    } sensor_data_t;

    const sensor_data_t &d = sensorData;

    INS(TwoWire &i2c, uint8_t address) : i2c{i2c}, address{address} {}

    bool init()
    {
        // This is a very simple sensor, not requiring any initialization.
        // Try to read the data once to test the connection.
        if (!read()) {
            return false;
        }

        log_i("INS init OK");
        return true;
    }

    bool read()
    {
        decltype(sensorData) newData;
        size_t n;

        n = i2c.requestFrom(static_cast<uint16_t>(address), sizeof(newData), true);
        if (n != sizeof(newData)) {
            log_e("0x%02x sent %d of %d B", address, n, sizeof(newData));
            return false;
        }

        n = i2c.readBytes(reinterpret_cast<uint8_t *>(&newData), sizeof(newData));
        if (n != sizeof(newData)) {
            log_e("Read %d of %d B", n, sizeof(newData));
            return false;
        }

        sensorData = newData;
        return true;
    }

private:
    TwoWire &i2c;
    const uint8_t address;

    sensor_data_t sensorData{};
};


#endif // INS_H_INCLUDED
