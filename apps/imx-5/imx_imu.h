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
    /**
     * Construct a new Inertial Sense IMX sensor driver object
     *
     * @param uart  UART instance for communication with the IMU.
     */
    explicit IMX(UART &uart) : uart(uart) {}

    /**
     * Initialize the IMX sensor driver
     *
     * This disables all sensor data broadcasts.
     *
     * @return `true` if the initialization was successful, `false` otherwise
     *         (error is logged internally).
     */
    bool init();

    /**
     * Process incoming data from the IMU
     */
    void loop();

    // Prevent (accidental) copying instances of this class
    IMX(const IMX &)            = delete;
    IMX &operator=(const IMX &) = delete;
    // Also prevent move, since it's not needed
    IMX(IMX &&)                 = delete;
    IMX &operator=(IMX &&)      = delete;

private:
    /** UART instance for communication with the IMU */
    UART &uart;
    /** Buffer for incoming messages */
    uint8_t rx_buffer[PKT_BUF_SIZE] = {};
    /** Inertial Sense SDK communication instance */
    is_comm_instance_t comm         = {};
    /** Last received data ID */
    eDataIDs last_rx_did            = DID_NULL;

    /** Current sensor configuration saved in flash */
    nvm_flash_cfg_t flash_cfg = {};

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
    bool getData(eDataIDs did, uint32_t interval = 0, uint32_t timeout = 0,
                 unsigned int offset = 0, size_t size = 0);

    /** Handle a received packet of type `ptype` */
    void handlePacket(protocol_type_t ptype);
    /** Handle packet parsing failure */
    void handlePacketParseError(eParseErrorType err_type) const;
    /** Handle InertialSense binary (ISB) packet */
    void handlePacketISB(const p_data_t &data);
};


#endif // IMU_IMX_IMU_H_INCLUDED
