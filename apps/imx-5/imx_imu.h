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
     */
    void init();

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

    /** Handle a received packet of type `ptype` */
    void handlePacket(protocol_type_t ptype);
    /** Handle packet parsing failure */
    void handlePacketParseError(eParseErrorType err_type) const;
    /** Handle InertialSense binary (ISB) packet */
    void handlePacketISB(eDataIDs did, const bufPtr_t &payload);
};


#endif // IMU_IMX_IMU_H_INCLUDED
