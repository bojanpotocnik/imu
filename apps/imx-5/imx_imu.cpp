#include "imx_imu.h"

static_assert(sizeof(double) == 8, "Inertial Sense SDK requires 64 bit double support");
static_assert(sizeof(int) >= sizeof(uintptr_t),
              "Successful cast of IMX pointer to int cannot be guaranteed");

bool IMX::init()
{
    // Initialize comm interface - call this before doing any comm functions
    is_comm_init(&comm, rx_buffer, sizeof(rx_buffer));

    if (!getData(DID_FLASH_CONFIG, 0, 500)) {
        return false;
    }

    // Stop all the broadcasts on the device
    is_comm_stop_broadcasts_all_ports(commPortWrite, reinterpret_cast<int>(this), &comm);

    return true;
}

bool IMX::getData(eDataIDs did, uint32_t interval, uint32_t timeout, unsigned int offset,
                  size_t size)
{
    is_comm_get_data(commPortWrite, reinterpret_cast<int>(this), &comm, did, offset, size,
                     interval);

    if (timeout > 0) {
        const auto start = millis();

        while ((millis() - start) < timeout) {
            last_rx_did = DID_NULL;
            loop();
            if (last_rx_did == did) {
                log_d("DID %d response received in %d ms", did, millis() - start);
                return true;
            }
        }
        log_e("DID %d response timeout %d ms", did, timeout);
        return false;
    }

    return true;
}

int IMX::commPortWrite(int this_ptr, const uint8_t *buf, int len)
{
    auto obj = reinterpret_cast<IMX *>(this_ptr);

    return static_cast<int>(obj->uart.write(buf, len));
}

template <typename T>
void IMX::copyDataToStruct(T &dataset_data, const p_data_t *data)
{
    const int r = copyDataPToStructP(&dataset_data, data, sizeof(dataset_data));
    assert(r == 0);
}


void IMX::loop()
{
    // Read data in chunks, to prevent the overhead of calling available() and read() for each byte
    size_t nrx;

    while ((nrx = uart.available()) > 0) {
        const uint32_t now = millis();
        uint8_t buffer[sizeof(rx_buffer)];

        nrx = uart.readBytes(buffer, std::min(nrx, sizeof(buffer)));

        for (size_t n = 0; n < nrx; n++) {
            const protocol_type_t ptype = is_comm_parse_byte_timeout(&comm, buffer[n], now);

            if (ptype != _PTYPE_NONE) {
                handlePacket(ptype);
            }
        }
    }
}

void IMX::handlePacket(protocol_type_t ptype)
{
    switch (ptype) {
        case _PTYPE_NONE: {
            // No complete valid data available yet
            break;
        }
        case _PTYPE_PARSE_ERROR: {
            // Invalid data or checksum error
            handlePacketParseError(comm.rxErrorType);
            break;
        }
        case _PTYPE_INERTIAL_SENSE_ACK:
            // Inertial Sense binary acknowledge (ack) or acknowledge (PID_ACK, PID_NACK) packet
        case _PTYPE_INERTIAL_SENSE_CMD:
            // Inertial Sense binary command (PID_GET_DATA, PID_STOP_BROADCASTS...) packet
        case _PTYPE_INERTIAL_SENSE_DATA: {
            // Inertial Sense binary data (PID_SET_DATA, PID_DATA) packet
            handlePacketISB(p_data_t{.hdr = comm.rxPkt.dataHdr, .ptr = comm.rxPkt.data.ptr});
            break;
        }
        case _PTYPE_NMEA: {
            // NMEA (National Marine Electronics Association) packet
            log_w("Ignored NMEA packet");
            break;
        }
        case _PTYPE_UBLOX: {
            // uBlox binary packet
            log_w("Ignored uBlox packet");
            break;
        }
        case _PTYPE_RTCM3: {
            // RTCM3 binary (Radio Technical Commission for Maritime Services) packet
            log_w("Ignored RTCM3 packet");
            break;
        }
        case _PTYPE_SPARTN: {
            // SPARTN binary packet
            log_w("Ignored SPARTN packet");
            break;
        }
        case _PTYPE_SONY: {
            // Sony binary packet
            log_w("Ignored Sony packet");
            break;
        }
        default: {
            log_e("Invalid packet %d", ptype);
            break;
        }
    }
}

void IMX::handlePacketParseError(eParseErrorType err_type) const
{
    static const std::array<const char *const, NUM_EPARSE_ERRORS> errors{
        "Invalid preamble",   // EPARSE_INVALID_PREAMBLE
        "Invalid size",       // EPARSE_INVALID_SIZE
        "Invalid checksum",   // EPARSE_INVALID_CHKSUM
        "Invalid data type",  // EPARSE_INVALID_DATATYPE
        "Missing EoS marker", // EPARSE_MISSING_EOS_MARKER
        "Incomplete packet",  // EPARSE_INCOMPLETE_PACKET
        "Invalid header",     // EPARSE_INVALID_HEADER
        "Invalid payload",    // EPARSE_INVALID_PAYLOAD
        "Rx buffer flushed",  // EPARSE_RXBUFFER_FLUSHED
        "Stream unparseable"  // EPARSE_STREAM_UNPARSEABLE
    };
    const auto err = static_cast<unsigned int>(err_type);

    if (comm.rxErrorCount <= 0) {
        return;
    }
    if (err >= errors.size()) {
        log_e("Parse error %d invalid", err);
        return;
    }
    log_e("Parse error %d '%s' (%d TX, %d RX, %d ERR)", err, errors[err], comm.txPktCount,
          comm.rxPktCount, comm.rxErrorCount);
}

void IMX::handlePacketISB(const p_data_t &data)
{
    last_rx_did = static_cast<eDataIDs>(data.hdr.id);

    // TODO: Implement parsing of ISB data packets
    switch (last_rx_did) {
        case DID_NULL: {
            break;
        }
        case DID_FLASH_CONFIG: {
            copyDataToStruct(flash_cfg, &data);
            log_d("IMU=%d ms, Nav=%d ms, GPS=%d ms", flash_cfg.startupImuDtMs,
                  flash_cfg.startupNavDtMs, flash_cfg.startupGPSDtMs);
            break;
        }
        default: log_e("Unhandled ISB DID %d", last_rx_did); break;
    }
}
