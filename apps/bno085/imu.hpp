#pragma once

#include <map>
#include <initializer_list>
#include <cmath>

#include "SparkFun_BNO08x_Arduino_Library.h"


/**
 * Extended BNO08x class with additional functionality
 */
class IMU : public BNO08x {
public:
    static const std::map<sh2_SensorId_t, const char *> SH2_SENSOR_NAMES;

    /**
     * Sensors providing orientation data
     *
     * One of the primary outputs of BNO08X is an estimation of the device orientation. Fusing data from
     * accelerometers, gyroscopes and magnetometers has a rich history of usage for estimating orientation.
     * BNO08X provides multiple estimates that have different tradeoffs as described below (note that the
     * term rotation vector used below is derived from Google’s definition in Android 4.4).
     */
    enum class OrientationSensor : sh2_SensorId_t {
        /**
         * Geomagnetic Rotation Vector
         *
         * The geomagnetic rotation vector is an orientation output that is expressed as a quaternion referenced
         * to magnetic north and gravity. It is produced by fusing the outputs of the accelerometer and magnetometer.
         * The gyroscope is specifically excluded in order to produce a rotation vector output using less power than
         * is required to produce the Rotation Vector. The consequences of removing the gyroscope are:
         *  - Less responsive output since the highly dynamic outputs of the gyroscope are not used
         *  - More errors in the presence of varying magnetic fields
         *
         * Maximum data rate: 90 Hz
         * Calibration: Nominal
         * Rotation error: 4.5° (Dynamic), 3.0° (Static)
         */
        GEOMAGNETIC_ROTATION_VECTOR = SH2_GEOMAGNETIC_ROTATION_VECTOR,
        /**
         * Game Rotation Vector
         *
         * The game rotation vector is an orientation output that is expressed as a quaternion with no specific
         * reference for heading, while roll and pitch are referenced against gravity. It is produced by fusing
         * the outputs of the accelerometer and the gyroscope (i.e. no magnetometer). The game rotation vector
         * does not use the magnetometer to correct the gyroscopes drift in yaw. This is a deliberate omission
         * (as specified by Google) to allow gaming applications to use a smoother representation of the
         * orientation without the jumps that an instantaneous correction provided by a magnetic field update
         * could provide. Long term the output will likely drift in yaw due to the characteristics of gyroscopes,
         * but this is seen as preferable for this output versus a corrected output.
         *
         * Maximum data rate: 400 Hz
         * Calibration: Nominal
         * Non-heading error: 2.5° (Dynamic), 1.5° (Static)
         * Heading drift: 0.5°/min
         */
        GAME_ROTATION_VECTOR = SH2_GAME_ROTATION_VECTOR,
        /**
         * AR/VR Stabilized Game Rotation vector
         *
         * While the magnetometer is removed from the calculation of the game rotation vector, the accelerometer
         * itself can create a potential correction in the rotation vector produced (i.e. the estimate of gravity
         * changes). For applications (typically augmented or virtual reality applications) where a sudden jump
         * can be disturbing, the output is adjusted to prevent these jumps in a manner that takes account of the
         * velocity of the sensor system. This process is called AR/VR stabilization. An Flash Record System
         * record is provided to allow configuration of this feature.
         *
         * Maximum data rate: 400 Hz (same as GAME_ROTATION_VECTOR)
         */
        STABILIZED_GAME_ROTATION_VECTOR = SH2_ARVR_STABILIZED_GRV,
        /**
         * Rotation Vector
         *
         * The rotation vector provides an orientation output that is expressed as a quaternion referenced to
         * magnetic north and gravity. It is produced by fusing the outputs of the accelerometer, gyroscope and
         * magnetometer. The rotation vector is the most accurate orientation estimate available. The magnetometer
         * provides correction in yaw to reduce drift and the gyroscope enables the most responsive performance.
         *
         * Maximum data rate: 400 Hz
         * Calibration: Nominal
         * Rotation Error: 3.5° (Dynamic), 2.0° (Static)
         */
        ROTATION_VECTOR = SH2_ROTATION_VECTOR,
        /**
         * AR/VR Stabilized Rotation Vector
         *
         * Estimates of the magnetic field and the roll/pitch of the device can create a potential correction in
         * the rotation vector produced. For applications (typically augmented or virtual reality applications)
         * where a sudden jump can be disturbing, the output is adjusted to prevent these jumps in a manner that
         * takes account of the velocity of the sensor system. This process is called AR/VR stabilization. An
         * Flash Record System record is provided to allow configuration of this feature.
         *
         * Maximum data rate: 400 Hz (same as ROTATION_VECTOR)
         */
        STABILIZED_ROTATION_VECTOR = SH2_ARVR_STABILIZED_RV,
        /**
         * Gyro rotation Vector
         *
         * Head tracking systems within a virtual reality headset require low latency processing of motion. To
         * facilitate this, the BNO08X can provide a rotation vector at rates up to 1kHz. The gyro rotation Vector
         * provides an alternate orientation to the standard rotation vector. Compared to the standard rotation
         * vector the gyro rotation vector has an optimized processing path and correction style (correction is
         * the adjustments made to the output based on more accurate estimates of gravity, mag field, angular
         * velocity) that is suitable for head tracking applications. By default the Gyro rotation vector provides
         * an orientation output that is expressed as a quaternion. It can be configured via FRS record to be based
         * on either the rotation vector (using the magnetometer) or the game rotation vector (ignoring the
         * magnetometer).
         *
         * Maximum data rate: 1000 Hz
         */
        GYRO_ROTATION_VECTOR = SH2_GYRO_INTEGRATED_RV
    };

    /**
     * Initialize the BNO08x sensor
     *
     * @return true if the sensor was found and initialized successfully.
     */
    bool begin()
    {
        printf("[IMU] Init\n");
        /* For ease of use, try both possible I2C addresses.
         * Interrupt or reset pins are not required as the sensor firmware reads
         * and fuses the data internally, and can be controlled via I2C. */
        if (!(BNO08x::begin(0x4A) || BNO08x::begin(0x4B)))
        {
            printf("[IMU] BNO08x not found\n");
            return false;
        }
        if (!modeOn())
        {
            printf("[IMU] Mode ON FAIL\n");
            return false;
        }

        /* Product IDs are read during initialization */
        const auto &p = prodIds.entry[0];
        printf("[IMU] OK (0x%x v%d.%d.%d.%d)\n",
               p.swPartNumber, p.swVersionMajor, p.swVersionMinor, p.swVersionPatch, p.swBuildNumber);

        initialized = true;
        return true;
    }

    /**
     * Print a table of all supported sensors and their metadata
     */
    void printSensorTable() const
    {
        assert(initialized);

        printf("| ID | %-18s | %-22s | vME | vMH | vSH | Range | Res | T min | T max |\n", "Sensor Name", "Vendor");

        /* Iterate over all {sensor ID, sensor name} pairs and metadata of each supported sensor */
        for (const auto &sensor: SH2_SENSOR_NAMES)
        {
            const sh2_SensorId_t sensor_id = sensor.first;
            const char *const sensor_name = sensor.second;
            sh2_SensorMetadata_t md;
            int sh2_status;

            sh2_status = sh2_getMetadata(sensor_id, &md);
            if (sh2_status == SH2_ERR_BAD_PARAM)
            {
                /* This sensor is not supported on this module */
                continue;
            }
            else if (sh2_status != SH2_OK)
            {
                /* Other unexpected errors */
                printf("[IMU] %s metadata read FAIL (%s)\n", sensor_name, sh2_error_name(sh2_status));
                continue;
            }

            /*      | ID** | %-18s | %-22s | vME | vMH | vSH | Range | Res | T min | T max | */
            printf("| %02x | %-18s | %-22s | %3d | %3d | %3d | %5d | %3d | %5d | %5d |\n",
                   sensor_id, sensor_name, md.vendorId,
                   md.meVersion, md.mhVersion, md.shVersion,
                   md.range, md.resolution,
                   md.minPeriod_uS / 1000, md.maxPeriod_uS / 1000);
        }

        printf("|----|--------------------|------------------------|-----|-----|-----|-------|-----|-------|-------|\n");
    }

    /**
     * Enable a sensor reports with specified interval
     *
     * @param sensor        Sensor to enable.
     * @param interval_ms   Report interval in milliseconds.
     *
     * @return true if the sensor was enabled successfully.
     */
    bool enableSensor(const OrientationSensor sensor, const uint16_t interval_ms) const
    {
        const auto sensor_id = static_cast<sh2_SensorId_t>(sensor);
        const sh2_SensorConfig_t config = {
                .alwaysOnEnabled = true,
                .reportInterval_us = interval_ms * 1000u
        };
        assert(initialized);

        int sh2_status = sh2_setSensorConfig(sensor_id, &config);
        if (sh2_status != SH2_OK)
        {
            printf("[IMU] %s enable FAIL (%s)\n", SH2_SENSOR_NAMES.at(sensor_id), sh2_error_name(sh2_status));
            return false;
        }

        return true;
    }

    /**
     * Process new sensor events - shall be called frequently (every loop)
     *
     * @return true if any sensor events were processed.
     */
    bool handle()
    {
        unsigned int count = 0;

        assert(initialized);

        /* Process all sensor events received since the last call to this function */
        while (getSensorEvent())
        {
            count++;
        }

        return count > 0;
    }

    bool getEuler(float &yaw, float &pitch, float &roll)
    {
        const sh2_SensorId_t sensor_id = getSensorEventID();

        /* Make sure that the current sensor value "loaded" in the library holds the correct data */
        if (sensor_id != SH2_ROTATION_VECTOR)
        {
            printf("[IMU] Wrong sensor event ID (0x%02x)\n", sensor_id);
            return false;
        }

        yaw = radToDeg(getYaw());
        pitch = radToDeg(getPitch());
        roll = radToDeg(getRoll());
        return true;
    }

    static const char *sh2_error_name(const int sh2_status)
    {
        static const std::map<int, const char *> sh2_status_map = {
                {SH2_OK,                 "OK"},
                {SH2_ERR,                "ERR"},
                {SH2_ERR_BAD_PARAM,      "BAD_PARAM"},
                {SH2_ERR_OP_IN_PROGRESS, "OP_IN_PROGRESS"},
                {SH2_ERR_IO,             "IO"},
                {SH2_ERR_HUB,            "HUB"},
                {SH2_ERR_TIMEOUT,        "TIMEOUT"}
        };
        static char buff[sizeof("0xFFFFFFFF")];

        const auto &it = sh2_status_map.find(sh2_status);
        if (it != sh2_status_map.end())
        {
            return it->second;
        }
        else
        {
            printf("0x%x", sh2_status);
            return buff;
        }
    }

    constexpr static float radToDeg(const float rad)
    {
        return rad * 180.0f / static_cast<float>(M_PI);
    }

private:
    bool initialized = false;
};

const std::map<sh2_SensorId_t, const char *> IMU::SH2_SENSOR_NAMES = {
        /* Accelerometer */
        {SH2_RAW_ACCELEROMETER,            "Accel Raw"},
        {SH2_ACCELEROMETER,                "Accel"},
        {SH2_LINEAR_ACCELERATION,          "Accel Linear"},
        {SH2_GRAVITY,                      "Gravity"},
        /* Gyroscope */
        {SH2_RAW_GYROSCOPE,                "Gyro Raw"},
        {SH2_GYROSCOPE_CALIBRATED,         "Gyro Cal"},
        {SH2_GYROSCOPE_UNCALIBRATED,       "Gyro UnCal"},
        /* Magnetometer */
        {SH2_RAW_MAGNETOMETER,             "Mag Raw"},
        {SH2_MAGNETIC_FIELD_CALIBRATED,    "Mag Cal"},
        {SH2_MAGNETIC_FIELD_UNCALIBRATED,  "Mag UnCal"},
        /* Sensor Fusion */
        {SH2_ROTATION_VECTOR,              "RotVec"},
        {SH2_GAME_ROTATION_VECTOR,         "RotVec Game"},
        {SH2_GEOMAGNETIC_ROTATION_VECTOR,  "RotVec Geo"},
        {SH2_GYRO_INTEGRATED_RV,           "RotVec Gyro"},
        {SH2_ARVR_STABILIZED_RV,           "RotVec AR/VR"},
        {SH2_ARVR_STABILIZED_GRV,          "RotVec AR/VR Game"},
        /* Motion Engine */
        {SH2_TAP_DETECTOR,                 "Tap Detector"},
        {SH2_STEP_DETECTOR,                "Step Detector"},
        {SH2_STEP_COUNTER,                 "Step Counter"},
        {SH2_STABILITY_DETECTOR,           "Stability Detector"},
        {SH2_STABILITY_CLASSIFIER,         "Stability Class"},
        {SH2_PERSONAL_ACTIVITY_CLASSIFIER, "Activity Class"},
        {SH2_SIGNIFICANT_MOTION,           "Motion"},
        {SH2_SHAKE_DETECTOR,               "Shake"},
        {SH2_FLIP_DETECTOR,                "Flip"},
        {SH2_PICKUP_DETECTOR,              "Pickup"},
        {SH2_SLEEP_DETECTOR,               "Sleep"},
        {SH2_TILT_DETECTOR,                "Tilt"},
        {SH2_POCKET_DETECTOR,              "Pocket"},
        {SH2_CIRCLE_DETECTOR,              "Circle"},
        /* Environmental sensors on a secondary I2C interface */
        {SH2_PRESSURE,                     "Pressure"},
        {SH2_AMBIENT_LIGHT,                "Ambient Light"},
        {SH2_HUMIDITY,                     "Humidity"},
        {SH2_PROXIMITY,                    "Proximity"},
        {SH2_TEMPERATURE,                  "Temperature"},
        {SH2_HEART_RATE_MONITOR,           "Heart Rate"},
        {SH2_RESERVED,                     "Reserved"},
        /* Other */
        {SH2_IZRO_MOTION_REQUEST,          "Motion Req"},
};
