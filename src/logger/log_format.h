/*
 * ACS4 Flight Computer — Binary Log Record Format
 *
 * All records are packed structs with a common header (msg_id + timestamp).
 * Written sequentially to a dual-buffer, flushed to SD card by LoggerThread.
 *
 * Endianness: little-endian (ARM Cortex-M7 native).
 * Integer encodings chosen to minimise record size while preserving
 * useful resolution for post-flight analysis.
 *
 * Producer: any thread via flight_logger::log()
 * Consumer: LoggerThread (flush to SD), log_decoder.py (offline decode)
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace acs
{

/* ── Message IDs ──────────────────────────────────────────────────────────── */

enum class LogMsgId : uint8_t
{
    IMU     = 0x01,
    NAV     = 0x02,
    CTRL    = 0x03,
    BARO    = 0x04,
    MAG     = 0x05,
    EVENT   = 0x06,
};

/* ── Common header (5 bytes) ──────────────────────────────────────────────── */

struct __attribute__((packed)) LogHeader
{
    uint8_t  msg_id;
    uint32_t timestamp_us;
};

static_assert(sizeof(LogHeader) == 5, "LogHeader must be 5 bytes");

/* ── MSG 0x01: IMU (17 bytes) ─────────────────────────────────────────────
 *   accel: 0.001 m/s² per LSB  (±32.767 m/s² range)
 *   gyro:  0.01  rad/s per LSB (±327.67 rad/s range)
 *   temp:  0.01  °C per LSB    (±327.67 °C)
 */
struct __attribute__((packed)) LogImu
{
    LogHeader hdr;          /* msg_id = 0x01 */
    int16_t   accel[3];     /* [milli-m/s²] */
    int16_t   gyro[3];      /* [centi-rad/s] */
};

static_assert(sizeof(LogImu) == 17, "LogImu must be 17 bytes");

/* ── MSG 0x02: NAV state (31 bytes) ──────────────────────────────────────
 *   quat: Q15 fixed-point (val * 32767)
 *   pos:  mm  (int32 for ±2147 km range)
 *   vel:  cm/s
 */
struct __attribute__((packed)) LogNav
{
    LogHeader hdr;          /* msg_id = 0x02 */
    int16_t   quat[4];     /* Q15 */
    int32_t   pos[3];      /* [mm] NED */
    int16_t   vel[3];      /* [cm/s] NED */
};

static_assert(sizeof(LogNav) == 31, "LogNav must be 31 bytes");

/* ── MSG 0x03: Control (14 bytes) ────────────────────────────────────────
 *   servo: 0.01 degree per LSB
 *   flight_state: enum value
 */
struct __attribute__((packed)) LogCtrl
{
    LogHeader hdr;          /* msg_id = 0x03 */
    int16_t   servo[4];    /* [centi-deg] */
    uint8_t   flight_state;
};

static_assert(sizeof(LogCtrl) == 14, "LogCtrl must be 14 bytes");

/* ── MSG 0x04: Barometer (13 bytes) ──────────────────────────────────────
 *   pressure: raw Pa (uint32_t, fits up to ~4.2 MPa)
 *   altitude: mm    (int32_t, ±2147 km range)
 */
struct __attribute__((packed)) LogBaro
{
    LogHeader hdr;          /* msg_id = 0x04 */
    uint32_t  pressure_pa;
    int32_t   altitude_mm;
};

static_assert(sizeof(LogBaro) == 13, "LogBaro must be 13 bytes");

/* ── MSG 0x05: Magnetometer (11 bytes) ───────────────────────────────────
 *   field: 0.01 µT per LSB (±327.67 µT)
 */
struct __attribute__((packed)) LogMag
{
    LogHeader hdr;          /* msg_id = 0x05 */
    int16_t   field[3];    /* [centi-µT] */
};

static_assert(sizeof(LogMag) == 11, "LogMag must be 11 bytes");

/* ── MSG 0x06: Event (8 bytes) ───────────────────────────────────────────
 *   Generic event marker (FSM transition, error, pyro fire, etc.)
 */
struct __attribute__((packed)) LogEvent
{
    LogHeader hdr;          /* msg_id = 0x06 */
    uint8_t   event_code;
    uint16_t  aux;
};

static_assert(sizeof(LogEvent) == 8, "LogEvent must be 8 bytes");

/* ── Maximum record size (for buffer math) ───────────────────────────────── */

inline constexpr size_t LOG_MAX_RECORD_SIZE = sizeof(LogNav); /* 31 bytes */

/* ── File magic & version header written at start of each log file ─────── */

struct __attribute__((packed)) LogFileHeader
{
    uint8_t  magic[4];     /* "ACS4" */
    uint16_t version;      /* format version, bump on breaking changes */
    uint32_t sysclk_hz;   /* system clock for timestamp calibration */
    uint32_t boot_time_ms; /* millis since power-on at log start */
};

static_assert(sizeof(LogFileHeader) == 14, "LogFileHeader must be 14 bytes");

inline constexpr uint8_t  LOG_MAGIC[4]       = {'A', 'C', 'S', '4'};
inline constexpr uint16_t LOG_FORMAT_VERSION  = 1;

}  // namespace acs
