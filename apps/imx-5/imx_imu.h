#ifndef IMU_IMX_IMU_H_INCLUDED
#define IMU_IMX_IMU_H_INCLUDED

#include <cstdint>

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include "uart.h"

// This needs to be included after system headers (e.g. Arduino.h)
// to prevent redefinition of platform constants from specreg.h.
#include "libs/inertial-sense-sdk/src/ISComm.h"
#include "libs/inertial-sense-sdk/src/data_sets.h"


class IMX
{
public:
    using SensorData = struct {
        dev_info_t dev_info;
        nvm_flash_cfg_t flash_cfg;
        sys_params_t sys_params;
        ins_1_t ins;
    };

    /**
     * Data sets available for streaming from the IMX sensor
     *
     * https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/#data-sets-dids
     */
    enum class DataSet {
        /** Inertial Navigation System (INS) and Attitude Heading Reference System (AHRS) in Euler
           angles */
        INS_AHRS_EULER = DID_INS_1,
        /** Inertial Navigation System (INS) and Attitude Heading Reference System (AHRS) in
           quaternions */
        INS_AHRS_QUAT  = DID_INS_2,
        /** Inertial Measurement Unit (IMU) data */
        IMU            = DID_IMU,
        /** Barometric pressure sensor data */
        BAROMETER      = DID_BAROMETER,
        /** Magnetometer sensor output */
        MAGNETOMETER   = DID_MAGNETOMETER,
        /** System sensor information */
        SYSTEM         = DID_SYS_PARAMS,
    };

    /**
     * The last received sensor data
     */
    const SensorData &d = m_data;

    class
    {
    public:
        /** Minimum received values since last call to `reset()` */
        imus_t min;
        /** Maximum received values since last call to `reset()` */
        imus_t max;
        /** Last received IMU data */
        imu_t last;

        void reset()
        {
            for (int i = 0; i < 3; i++) {
                min.acc[i] = std::numeric_limits<float>::max();
                max.acc[i] = std::numeric_limits<float>::min();
                min.pqr[i] = std::numeric_limits<float>::max();
                max.pqr[i] = std::numeric_limits<float>::min();
            }
            last = {};
        }
    } imu{};

    /** This flag is only set internally and can be read and cleared by the user */
    bool timeoutOccurred = false;

    /**
     * Construct a new Inertial Sense IMX sensor driver object
     *
     * @param uart  UART instance for communication with the IMU.
     */
    explicit IMX(UART &uart) : uart(uart) {}

    // Prevent (accidental) copying instances of this class
    IMX(const IMX &)            = delete;
    IMX &operator=(const IMX &) = delete;
    // Also prevent move, since it's not needed
    IMX(IMX &&)                 = delete;
    IMX &operator=(IMX &&)      = delete;

    /**
     * Initialize the IMX sensor driver
     *
     * This disables all sensor data broadcasts, use `enableData` to enable reporting of specific
     * data sets.
     *
     * @return `true` if the initialization was successful, `false` otherwise
     *         (error is logged internally).
     */
    bool init();

    /**
     * Enable streaming of a specific data set from the IMX sensor
     *
     * @param data_set   The data set to enable streaming for.
     * @param period_ms  The period in milliseconds at which to stream the data, or
     *                   0 for a one-time message.
     *                   Note that periodMs is rounded to the nearest multiple of the
     *                   sensor's internal source update rate for a specific data set.
     *                   https://docs.inertialsense.com/user-manual/com-protocol/isb/#data-source-update-rates
     *
     * @return `true` if the data set was enabled successfully, `false` otherwise
     *         (error is logged internally).
     */
    bool enableData(DataSet data_set, uint16_t period_ms = 0);

    /**
     * Process incoming data from the IMU
     *
     * @return If received UART data was processed, but no full packet was received,
     *         the negative number representing number of bytes read from UART is returned.
     *         If some packet was received, the positive number representing number of bytes
     *         read from UART is returned.
     *         If no data was received, 0 is returned.
     */
    int loop();

private:
    /** UART instance for communication with the IMU */
    UART &uart;
    /** Buffer for incoming messages */
    uint8_t rx_buffer[2 * PKT_BUF_SIZE] = {};
    /** Inertial Sense SDK communication instance */
    is_comm_instance_t comm             = {};
    /** Last received positive or negative acknowledgement packet type */
    eISBPacketFlags last_rx_ack         = PKT_TYPE_INVALID;
    /** Last received ISB DATA packet data ID */
    eDataIDs last_rx_did                = DID_NULL;
    /** The milliseconds timestamp when the last UART data was received */
    uint32_t last_rx_timestamp          = 0;
    /** Maximum allowed duration (ms) to wait for any UART data, or 0 to disable the timeout */
    uint32_t rx_timeout                 = 0;

    /** Last received sensor data */
    SensorData m_data = {};

    /**
     * Inertial Sense SDK callback to write data to the serial port
     *
     * @param port  The pointer to instance of this class, which caused the
     *              invocation of this callback with API call.
     * @param buf   The buffer to write.
     * @param len   The number of bytes from the buffer to write.
     *
     * @return The number of bytes written.
     */
    static int commPortWrite(int this_ptr, const uint8_t *buf, int len);

    /**
     * Send an InertialSense binary (ISB) packet
     *
     * @param pkt_type   ISB packet flags which include the packet type.
     * @param did        ISB data ID.
     * @param timeout    Time in milliseconds to wait for a response, or 0 to not wait.
     * @param data       Pointer to payload data.
     * @param data_size  Size in bytes of the payload data.
     * @param offset     Offset of the payload data into the data set structure.
     *
     * @return `true` if the packet was sent successfully, `false` otherwise
     *         (error is logged internally).
     */
    bool write(eISBPacketFlags pkt_type, eDataIDs did, uint16_t timeout, void *data = nullptr,
               uint16_t offset = 0, uint16_t data_size = 0);
    template <typename T>
    bool write(eISBPacketFlags pkt_type, eDataIDs did, uint16_t timeout, T &data,
               uint16_t offset = 0, uint16_t data_size = sizeof(T));
    bool write(eISBPacketFlags pkt_type, uint16_t timeout);

    bool waitAck(uint16_t timeout);
    bool waitIsbDid(eDataIDs did, uint16_t timeout);

    /**
     * Copy only new received data to the destination structure
     *
     * This behaves as normal memory copy when `getData(offset=0, size=0)` is used,
     * but copies only the new data when offset and size are specified.
     */
    template <typename T>
    static void copyDataToStruct(T &dataset_data, const p_data_t *data);

    /**
     * Send a request to the IMX sensor to start streaming data
     *
     * @param did       The data ID to request (see `DID_*` from `data_sets.h`).
     * @param interval  How often the data shall be sent from the sensor, in multiples
     *                  of the sensor's internal data source update rate, or
     *                  0 for a one-time message and turn off.
     * @param timeout   If not 0, the time in milliseconds to wait for the response.
     *                  If `period_ms` is non-zero, only the first response is waited for.
     * @param offset    Offset into data structure to request, or
     *                  0 for entire data structure (or up to `size` bytes).
     * @param size      Length of data from `offset` to request, or
     *                  0 for the entire remaining structure data.
     *
     * @return `true` if the request was sent successfully, `false` otherwise
     *         (error is logged internally).
     */
    bool getData(eDataIDs did, uint16_t interval = 0, uint16_t timeout = 0, unsigned int offset = 0,
                 size_t size = 0);

    /** Handle a received `ptype` packet of protocol `pro_type` */
    void handlePacket(eISBPacketFlags pkt_type, protocol_type_t pro_type);
    /** Handle packet parsing failure */
    void handlePacketParseError(eParseErrorType err_type) const;
    /** Handle InertialSense binary (ISB) packet */
    void handlePacketISB(const p_data_t &data);
};


#endif // IMU_IMX_IMU_H_INCLUDED
