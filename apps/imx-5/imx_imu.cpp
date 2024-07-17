#include "imx_imu.h"

static_assert(sizeof(double) == 8, "Inertial Sense SDK requires 64 bit double support");
static_assert(sizeof(int) >= sizeof(uintptr_t),
              "Successful cast of IMX pointer to int cannot be guaranteed");

bool IMX::init()
{
    bool ok = false;

    // Initialize comm interface - call this before doing any comm functions
    is_comm_init(&comm, rx_buffer, sizeof(rx_buffer));

    // If the sensor is currently streaming data a lot of data, the following command
    // or the sensor response may be lost - send it multiple times until successful.
    for (int i = 0; i < 10; i++) {
        if (write(PKT_TYPE_STOP_BROADCASTS_ALL_PORTS, 100)) {
            ok = true;
            break;
        }
        is_comm_reset_parser(&comm);
    }
    if (!ok) {
        log_e("Failed to stop broadcasts");
        return false;
    }

    if (!(getData(DID_SYS_PARAMS, 0, 100) &&  // Read system parameters
          getData(DID_FLASH_CONFIG, 0, 500))) // Read configuration from flash
    {
        return false;
    }

    log_i("IMX init OK");
    return true;
}

bool IMX::getData(eDataIDs did, uint16_t interval, uint16_t timeout, unsigned int offset,
                  size_t size)
{
    assert(did <= std::numeric_limits<uint16_t>::max());
    assert(size <= std::numeric_limits<uint16_t>::max());
    assert((offset <= std::numeric_limits<uint16_t>::max()) && (offset <= size));

    p_data_get_t get = {.id     = static_cast<uint16_t>(did),
                        .size   = static_cast<uint16_t>(size),
                        .offset = static_cast<uint16_t>(offset),
                        .period = interval};
    return write(PKT_TYPE_GET_DATA, did, timeout, get);
}

int IMX::commPortWrite(int this_ptr, const uint8_t *buf, int len)
{
    auto obj = reinterpret_cast<IMX *>(this_ptr);

    return static_cast<int>(obj->uart.write(buf, len));
}

bool IMX::write(eISBPacketFlags pkt_type, eDataIDs did, uint16_t timeout, void *data,
                uint16_t offset, uint16_t data_size)
{
    if (is_comm_write(commPortWrite, reinterpret_cast<int>(this), &comm, pkt_type, did, data_size,
                      offset, data) < 0)
    {
        log_e("write(%d, %d, %d, %d, p) FAIL", pkt_type, did, data_size, offset, data);
        return false;
    }
    if (timeout <= 0) {
        return true;
    }
    switch (pkt_type) {
        case PKT_TYPE_GET_DATA: {
            return waitIsbDid(did, timeout);
        }
        case PKT_TYPE_SET_DATA:
        case PKT_TYPE_STOP_BROADCASTS_ALL_PORTS:
        case PKT_TYPE_STOP_DID_BROADCAST:
        case PKT_TYPE_STOP_BROADCASTS_CURRENT_PORT: {
            return waitAck(timeout);
        }
        default: {
            log_e("Unknown pkt_type %d", pkt_type);
            return false;
        }
    }
}

template <typename T>
bool IMX::write(eISBPacketFlags pkt_type, eDataIDs did, uint16_t timeout, T &data, uint16_t offset,
                uint16_t data_size)
{
    return write(pkt_type, did, timeout, &data, offset, data_size);
}

bool IMX::write(eISBPacketFlags pkt_type, uint16_t timeout)
{
    return write(pkt_type, DID_NULL, timeout);
}

bool IMX::waitAck(uint16_t timeout)
{
    const auto t_start = millis();
    do {
        last_rx_ack = PKT_TYPE_INVALID;
        loop();
        if (last_rx_ack == PKT_TYPE_ACK) {
#ifdef DEBUG
            log_d("ACK in %d ms", millis() - t_start);
#endif
            return true;
        }
        else if (last_rx_ack == PKT_TYPE_NACK) {
            log_w("NACK in %d ms", millis() - t_start);
            return false;
        }
    } while ((millis() - t_start) < timeout);

    log_e("Timeout %d ms", timeout);
    return false;
}

bool IMX::waitIsbDid(eDataIDs did, uint16_t timeout)
{
    const auto t_start = millis();
    do {
        last_rx_did = DID_NULL;
        loop();
        if (last_rx_did == did) {
#ifdef DEBUG
            log_d("DID %d in %d ms", did, millis() - t_start);
#endif
            return true;
        }
    } while ((millis() - t_start) < timeout);

    log_e("Timeout %d ms", timeout);
    return false;
}

bool IMX::enableData(IMX::DataSet data_set, uint16_t period_ms)
{
    const auto &cfg = d.flash_cfg;
    uint32_t source_update_rate;
    uint32_t period_multiple;
    uint32_t period_ms_actual;

    if ((cfg.startupNavDtMs <= 0) || (cfg.startupImuDtMs <= 0) || (cfg.startupGPSDtMs <= 0) ||
        (cfg.startupImuDtMs > cfg.startupNavDtMs))
    {
        log_e("Invalid flash configuration (NavDtMs=%d, ImuDtMs=%d, GPSDtMs=%d)",
              cfg.startupNavDtMs, cfg.startupImuDtMs, cfg.startupGPSDtMs);
        return false;
    }

    switch (data_set) {
        case DataSet::INS_AHRS_EULER: // fallthrough
        case DataSet::INS_AHRS_QUAT:  source_update_rate = cfg.startupNavDtMs; break;
        case DataSet::IMU:            source_update_rate = cfg.startupImuDtMs; break;
        case DataSet::BAROMETER:      source_update_rate = 8; break;
        case DataSet::MAGNETOMETER:   source_update_rate = 10; break;
        default:                      source_update_rate = 1; break;
    }

    // Calculate the period-multiple value for the desired period (round up to prevent 0)
    period_multiple = (period_ms + source_update_rate - 1) / source_update_rate;

    period_ms_actual = period_multiple * source_update_rate;
    if (period_ms_actual != period_ms) {
        log_w("DID %d period %d ms rounded to %dx%d=%d ms", static_cast<int>(data_set), period_ms,
              period_multiple, source_update_rate, period_ms_actual);
    }

    return getData(static_cast<eDataIDs>(data_set), period_multiple);
}

template <typename T>
void IMX::copyDataToStruct(T &dataset_data, const p_data_t *data)
{
    const int r = copyDataPToStructP(&dataset_data, data, sizeof(dataset_data));
    assert(r == 0);
}

void IMX::loop()
{
    // Read data in chunks, to prevent the overhead of calling available() and read() for each byte.
    // Only read a single chunk at once, to prevent this function blocking forever in case that
    // the data is received faster than it can be processed.
    size_t n_waiting = uart.available();

    while (n_waiting > 0) {
        const uint32_t now = millis();
        uint8_t buffer[sizeof(rx_buffer)];

        const auto n_read  = uart.readBytes(buffer, std::min(n_waiting, sizeof(buffer)));
        n_waiting         -= n_read;

        for (size_t n = 0; n < n_read; n++) {
            const auto pro_type = is_comm_parse_byte_timeout(&comm, buffer[n], now);

            if (pro_type != _PTYPE_NONE) {
                handlePacket(static_cast<eISBPacketFlags>(comm.rxPkt.flags & PKT_TYPE_MASK),
                             pro_type);
            }
        }
    }
}

void IMX::handlePacket(eISBPacketFlags pkt_type, protocol_type_t pro_type)
{
    switch (pro_type) {
        case _PTYPE_PARSE_ERROR: {
            // Invalid data or checksum error
            handlePacketParseError(comm.rxErrorType);
            break;
        }
        case _PTYPE_INERTIAL_SENSE_ACK: {
            // Inertial Sense binary acknowledge (ack) or acknowledge (PID_ACK, PID_NACK) packet
            if ((pkt_type == PKT_TYPE_ACK) || (pkt_type == PKT_TYPE_NACK)) {
                last_rx_ack = pkt_type;
            }
            else {
                log_e("Received invalid N/ACK packet %d", pkt_type);
            }
            break;
        }
        case _PTYPE_INERTIAL_SENSE_CMD: {
            // Inertial Sense binary command (PID_GET_DATA, PID_STOP_BROADCASTS...) packet
            log_w("Ignored ISB CMD packet %d", pkt_type);
            break;
        }
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
            log_e("Invalid pkt=%d pro=%d", pkt_type, pro_type);
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
#ifdef DEBUG
    log_d("DID=%d", last_rx_did);
#endif

    switch (last_rx_did) {
        case DID_NULL: {
            break;
        }
        case DID_DEV_INFO: {
            copyDataToStruct(m_data.dev_info, &data);
            break;
        }
        case DID_SYS_PARAMS: {
            copyDataToStruct(m_data.sys_params, &data);
            break;
        }
        case DID_FLASH_CONFIG: {
            copyDataToStruct(m_data.flash_cfg, &data);
            break;
        }
        case DID_INS_1: {
            copyDataToStruct(m_data.ins, &data);
            break;
        }
        case DID_IMU: {
            copyDataToStruct(m_data.imu, &data);
            break;
        }
        default: {
            log_e("Unhandled ISB DID %d", last_rx_did);
            break;
        }
    }
}
