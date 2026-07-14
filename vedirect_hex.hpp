#pragma once
/**
 * vedirect_hex.hpp
 * ================
 * VE.Direct HEX protocol — the get/set register mechanism used by
 * VictronConnect to read and change device settings.
 *
 * Frame format (ASCII, over the same 19200 8N1 serial line):
 *
 *   :[command][data ...][checksum]\n
 *
 *   - ':'       start of hex frame (may interrupt a text frame anywhere)
 *   - command   single hex nibble  (1=Ping 3=AppVer 4=ProdId 6=Restart
 *                                   7=Get  8=Set    A=Async)
 *   - data      hex byte pairs; for Get/Set: un16 register id (little
 *               endian), un8 flags, then the value bytes (little endian)
 *   - checksum  un8 chosen so that  command + all bytes ≡ 0x55 (mod 256)
 *   - '\n'      end of frame
 *
 * Responses mirror the command number (:7 for Get, :8 for Set, :5 for
 * Ping, :A for unsolicited async updates, :3 Unknown, :4 Frame error).
 * The flags byte in a Get/Set response reports errors:
 *   bit0 = unknown register id, bit1 = not supported, bit2 = parameter error
 *
 * References:
 *   VE.Direct HEX protocol PDFs (MPPT / BMV / Phoenix Inverter),
 *   https://www.victronenergy.com/support-and-downloads/technical-information
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <unistd.h>

// ─────────────────────────────────────────────────────────────────────────────
// Hex helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace vehex {

inline int nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

inline char hex_digit(int v) { return "0123456789ABCDEF"[v & 0xF]; }

// Build a complete frame ":<cmd><payload><checksum>\n"
inline std::string build_frame(uint8_t cmd, const std::vector<uint8_t>& payload) {
    int sum = cmd & 0xF;
    std::string f;
    f.reserve(4 + payload.size() * 2 + 3);
    f += ':';
    f += hex_digit(cmd & 0xF);
    for (uint8_t b : payload) {
        f += hex_digit(b >> 4);
        f += hex_digit(b & 0xF);
        sum = (sum + b) & 0xFF;
    }
    uint8_t check = static_cast<uint8_t>((0x55 - sum) & 0xFF);
    f += hex_digit(check >> 4);
    f += hex_digit(check & 0xF);
    f += '\n';
    return f;
}

// Parse the body of a received frame (everything between ':' and '\n').
// On success: cmd = command/response nibble, data = payload bytes with the
// trailing checksum byte removed.  Returns false on malformed frame or bad
// checksum.
inline bool parse_frame(const std::string& body, uint8_t& cmd,
                        std::vector<uint8_t>& data) {
    if (body.size() < 3) return false;              // cmd + at least checksum
    int c = nibble(body[0]);
    if (c < 0) return false;
    if ((body.size() - 1) % 2 != 0) return false;   // byte pairs after cmd

    std::vector<uint8_t> bytes;
    bytes.reserve((body.size() - 1) / 2);
    for (size_t i = 1; i + 1 < body.size(); i += 2) {
        int hi = nibble(body[i]), lo = nibble(body[i + 1]);
        if (hi < 0 || lo < 0) return false;
        bytes.push_back(static_cast<uint8_t>((hi << 4) | lo));
    }

    int sum = c;
    for (uint8_t b : bytes) sum = (sum + b) & 0xFF;
    if (sum != 0x55) return false;                  // checksum byte included

    bytes.pop_back();                               // drop checksum
    cmd  = static_cast<uint8_t>(c);
    data = std::move(bytes);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Register catalogue — the settings VictronConnect exposes, per device class
// ─────────────────────────────────────────────────────────────────────────────
enum class VType : uint8_t { U8, U16, S16, U32, S32 };

inline int vtype_size(VType t) {
    switch (t) {
        case VType::U8:  return 1;
        case VType::U16: case VType::S16: return 2;
        default: return 4;
    }
}

// Device classes, derived from the text-protocol PID
enum : uint8_t { DEV_MPPT = 1, DEV_BMV = 2, DEV_INVERTER = 4, DEV_ALL = 0xFF };

inline uint8_t device_class_from_pid(int pid) {
    if (pid == 0x0203 || pid == 0x0205 || pid == 0x0300 || pid == 0xA389)
        return DEV_BMV;
    if ((pid & 0xFF00) == 0xA000) return DEV_MPPT;      // 0xA0xx solar chargers
    if ((pid & 0xFF00) == 0xA200) return DEV_INVERTER;  // 0xA2xx Phoenix
    return DEV_ALL;  // unknown device: expose everything, device will NAK
}

struct RegDef {
    const char* name;     // command-line name, e.g. "absorption_voltage"
    uint16_t    reg;      // register id
    VType       type;
    double      scale;    // engineering value = raw * scale
    const char* unit;
    bool        writable;
    uint8_t     devmask;  // which device classes support it
    const char* help;
};

// NOTE: this mirrors the VictronConnect settings pages.  Registers a given
// model doesn't support are reported by the device itself (flags byte) and
// skipped in `settings` output — no harm in probing.
inline const std::vector<RegDef>& registers() {
    static const std::vector<RegDef> t = {
        // ── Common / operation ─────────────────────────────────────────────
        {"device_mode",             0x0200, VType::U8,  1,     "",     true,  DEV_MPPT|DEV_INVERTER,
         "MPPT: 1=charger on, 4=charger off. Inverter: 2=on, 4=off, 5=eco"},
        {"device_state",            0x0201, VType::U8,  1,     "",     false, DEV_ALL, "operation state (read-only)"},

        // ── MPPT battery / charger settings (VictronConnect 'Battery') ────
        {"battery_voltage",         0xEDEF, VType::U8,  1,     "V",    true,  DEV_MPPT, "system voltage: 0=auto detect, 12/24/36/48"},
        {"max_charge_current",      0xEDF0, VType::U16, 0.1,   "A",    true,  DEV_MPPT, "maximum charge current"},
        {"battery_type",            0xEDF1, VType::U8,  1,     "",     true,  DEV_MPPT, "1..8=preset (rotary switch table), 255=user defined"},
        {"absorption_voltage",      0xEDF7, VType::U16, 0.01,  "V",    true,  DEV_MPPT, "absorption voltage"},
        {"float_voltage",           0xEDF6, VType::U16, 0.01,  "V",    true,  DEV_MPPT, "float voltage"},
        {"equalisation_voltage",    0xEDF4, VType::U16, 0.01,  "V",    true,  DEV_MPPT, "equalisation voltage"},
        {"temp_compensation",       0xEDF2, VType::S16, 0.01,  "mV/K", true,  DEV_MPPT, "temperature compensation (0xFFFF=disabled)"},
        {"batterysafe_mode",        0xEDFF, VType::U8,  1,     "",     true,  DEV_MPPT, "0=off 1=on"},
        {"adaptive_mode",           0xEDFE, VType::U8,  1,     "",     true,  DEV_MPPT, "adaptive absorption time: 0=off 1=on"},
        {"auto_equalise_mode",      0xEDFD, VType::U8,  1,     "",     true,  DEV_MPPT, "0=disabled, 1..250=repeat every n days"},
        {"bulk_time_limit",         0xEDFC, VType::U16, 0.01,  "h",    true,  DEV_MPPT, "maximum bulk time"},
        {"absorption_time_limit",   0xEDFB, VType::U16, 0.01,  "h",    true,  DEV_MPPT, "maximum absorption time"},
        {"tail_current",            0xEDE7, VType::U16, 0.1,   "A",    true,  DEV_MPPT, "absorption end tail current"},
        {"low_temp_charge_current", 0xEDE6, VType::U16, 0.1,   "A",    true,  DEV_MPPT, "max charge current below low_temp_level (0xFFFF=use max)"},
        {"auto_eq_stop_on_voltage", 0xEDE5, VType::U8,  1,     "",     true,  DEV_MPPT, "0=no 1=yes"},
        {"equalisation_current",    0xEDE4, VType::U8,  1,     "%",    true,  DEV_MPPT, "equalisation current level, % of max_charge_current"},
        {"equalisation_duration",   0xEDE3, VType::U16, 0.01,  "h",    true,  DEV_MPPT, "equalisation duration"},
        {"rebulk_voltage_offset",   0xED2E, VType::U16, 0.01,  "V",    true,  DEV_MPPT, "re-bulk voltage offset"},
        {"low_temp_level",          0xEDE0, VType::S16, 0.01,  "degC", true,  DEV_MPPT, "low temperature cut-off level"},
        {"bms_present",             0xEDE8, VType::U8,  1,     "",     true,  DEV_MPPT, "0=no 1=yes"},

        // ── MPPT load output (VictronConnect 'Load output') ────────────────
        {"load_mode",               0xEDAB, VType::U8,  1,     "",     true,  DEV_MPPT,
         "0=off 1=auto(BatteryLife) 2=alt1 3=alt2 4=always on 5=user1 6=user2"},
        {"load_switch_low_level",   0xED9C, VType::U16, 0.01,  "V",    true,  DEV_MPPT, "user-defined load switch-off voltage"},
        {"load_switch_high_level",  0xED9D, VType::U16, 0.01,  "V",    true,  DEV_MPPT, "user-defined load switch-on voltage"},
        {"load_output_state",       0xEDA8, VType::U8,  1,     "",     false, DEV_MPPT, "0=off 1=on (read-only)"},

        // ── MPPT diagnostics (read-only, handy next to settings) ───────────
        {"charger_temperature",     0xEDDB, VType::S16, 0.01,  "degC", false, DEV_MPPT, "charger internal temperature (read-only)"},

        // ── BMV settings (VictronConnect 'Battery settings') ───────────────
        {"battery_capacity",        0x1000, VType::U16, 1,     "Ah",   true,  DEV_BMV, "battery capacity"},
        {"charged_voltage",         0x1001, VType::U16, 0.1,   "V",    true,  DEV_BMV, "charged voltage"},
        {"tail_current_pct",        0x1002, VType::U16, 0.1,   "%",    true,  DEV_BMV, "tail current"},
        {"charged_detection_time",  0x1003, VType::U16, 1,     "min",  true,  DEV_BMV, "charged detection time"},
        {"charge_efficiency",       0x1004, VType::U16, 1,     "%",    true,  DEV_BMV, "charge efficiency factor"},
        {"peukert_coefficient",     0x1005, VType::U16, 0.01,  "",     true,  DEV_BMV, "Peukert coefficient"},
        {"current_threshold",       0x1006, VType::U16, 0.01,  "A",    true,  DEV_BMV, "current threshold"},
        {"ttg_delta_t",             0x1007, VType::U16, 1,     "min",  true,  DEV_BMV, "time-to-go averaging period"},
        {"discharge_floor",         0x1008, VType::U16, 0.1,   "%",    true,  DEV_BMV, "discharge floor / relay low SOC set"},
        {"relay_low_soc_clear",     0x1009, VType::U16, 0.1,   "%",    true,  DEV_BMV, "relay low SOC clear"},

        // ── Phoenix inverter settings ───────────────────────────────────────
        {"ac_out_voltage_setpoint", 0x0230, VType::U16, 0.01,  "V",    true,  DEV_INVERTER, "AC output voltage setpoint"},
        {"ac_out_setpoint_min",     0x0231, VType::U16, 0.01,  "V",    false, DEV_INVERTER, "setpoint lower bound (read-only)"},
        {"ac_out_setpoint_max",     0x0232, VType::U16, 0.01,  "V",    false, DEV_INVERTER, "setpoint upper bound (read-only)"},
        {"alarm_low_voltage_set",   0x0320, VType::U16, 0.01,  "V",    true,  DEV_INVERTER, "low battery voltage alarm set level"},
        {"alarm_low_voltage_clear", 0x0321, VType::U16, 0.01,  "V",    true,  DEV_INVERTER, "low battery voltage alarm clear level"},
    };
    return t;
}

inline const RegDef* find_register(const std::string& name) {
    for (const auto& r : registers())
        if (name == r.name) return &r;
    return nullptr;
}

inline const RegDef* find_register(uint16_t reg) {
    for (const auto& r : registers())
        if (reg == r.reg) return &r;
    return nullptr;
}

// Raw little-endian value → signed/unsigned 64-bit according to type
inline int64_t decode_value(const std::vector<uint8_t>& b, VType t) {
    uint64_t v = 0;
    for (size_t i = 0; i < b.size() && i < 8; ++i)
        v |= static_cast<uint64_t>(b[i]) << (8 * i);
    switch (t) {
        case VType::S16: return static_cast<int16_t>(v & 0xFFFF);
        case VType::S32: return static_cast<int32_t>(v & 0xFFFFFFFF);
        default:         return static_cast<int64_t>(v);
    }
}

inline std::vector<uint8_t> encode_value(int64_t v, VType t) {
    std::vector<uint8_t> b(vtype_size(t));
    for (size_t i = 0; i < b.size(); ++i)
        b[i] = static_cast<uint8_t>((static_cast<uint64_t>(v) >> (8 * i)) & 0xFF);
    return b;
}

// ─────────────────────────────────────────────────────────────────────────────
// HexBus — sends HEX commands on the serial fd and matches responses that the
// main reader thread feeds back via on_frame().  One transaction at a time;
// callers are serialized on bus_mu_.
// ─────────────────────────────────────────────────────────────────────────────
class HexBus {
public:
    struct Result {
        bool                 ok = false;
        std::string          error;      // set when !ok
        uint8_t              flags = 0;  // Get/Set response flags byte
        std::vector<uint8_t> value;      // raw value bytes (little endian)
    };

    // Serial fd lifecycle (main loop calls these on connect/disconnect)
    void attach(int fd)  { fd_.store(fd); }
    void detach()        { fd_.store(-1); fail_pending("serial disconnected"); }

    // Called from the serial reader thread for every valid ':' frame
    void on_frame(uint8_t rsp, const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!waiting_) return;                       // unsolicited (e.g. :A)

        if (rsp == 0xA) return;                      // async broadcast, ignore

        if (rsp == 0x3) { complete_error("device: unknown command"); return; }
        if (rsp == 0x4) { complete_error("device: frame error");     return; }

        if (rsp != expect_rsp_) return;

        // Get/Set responses start with register id (le16) + flags
        if (rsp == 0x7 || rsp == 0x8) {
            if (data.size() < 3) { complete_error("short response"); return; }
            uint16_t reg = static_cast<uint16_t>(data[0] | (data[1] << 8));
            if (reg != expect_reg_) return;          // response to someone else
            result_.flags = data[2];
            result_.value.assign(data.begin() + 3, data.end());
            if (result_.flags & 0x01)      result_.error = "device: unknown register id";
            else if (result_.flags & 0x02) result_.error = "device: register not supported";
            else if (result_.flags & 0x04) result_.error = "device: parameter error (value out of range)";
            result_.ok = result_.error.empty();
        } else {
            // Ping (:5), Done (:1) — payload is the whole response
            result_.value = data;
            result_.ok    = true;
        }
        waiting_ = false;
        cv_.notify_all();
    }

    Result ping()                       { return transact(0x1, {},                    0x5, 0);   }
    Result restart() {                    // device reboots, no response expected
        std::lock_guard<std::mutex> bus(bus_mu_);
        if (!send(build_frame(0x6, {})))
            return {false, "serial not connected", 0, {}};
        return {true, "", 0, {}};
    }

    Result get(uint16_t reg) {
        std::vector<uint8_t> p = { static_cast<uint8_t>(reg & 0xFF),
                                   static_cast<uint8_t>(reg >> 8), 0x00 };
        return transact(0x7, p, 0x7, reg);
    }

    Result set(uint16_t reg, const std::vector<uint8_t>& value) {
        std::vector<uint8_t> p = { static_cast<uint8_t>(reg & 0xFF),
                                   static_cast<uint8_t>(reg >> 8), 0x00 };
        p.insert(p.end(), value.begin(), value.end());
        return transact(0x8, p, 0x8, reg);
    }

private:
    bool send(const std::string& frame) {
        int fd = fd_.load();
        if (fd < 0) return false;
        std::lock_guard<std::mutex> lk(write_mu_);
        const char* p = frame.data();
        size_t left = frame.size();
        while (left > 0) {
            ssize_t n = ::write(fd, p, left);
            if (n < 0) { if (errno == EINTR) continue; return false; }
            p += n; left -= static_cast<size_t>(n);
        }
        return true;
    }

    Result transact(uint8_t cmd, const std::vector<uint8_t>& payload,
                    uint8_t expect_rsp, uint16_t expect_reg,
                    int timeout_ms = 2000, int retries = 2) {
        std::lock_guard<std::mutex> bus(bus_mu_);   // one transaction at a time

        for (int attempt = 0; attempt <= retries; ++attempt) {
            {
                std::lock_guard<std::mutex> lk(mu_);
                waiting_    = true;
                expect_rsp_ = expect_rsp;
                expect_reg_ = expect_reg;
                result_     = {};
            }
            if (!send(build_frame(cmd, payload))) {
                std::lock_guard<std::mutex> lk(mu_);
                waiting_ = false;
                return {false, "serial not connected", 0, {}};
            }
            std::unique_lock<std::mutex> lk(mu_);
            bool got = cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                    [&]{ return !waiting_; });
            if (got) return result_;
            waiting_ = false;                        // timed out, maybe retry
        }
        return {false, "timeout waiting for device response", 0, {}};
    }

    void complete_error(const std::string& e) {
        result_ = {false, e, 0, {}};
        waiting_ = false;
        cv_.notify_all();
    }

    void fail_pending(const std::string& e) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!waiting_) return;
        result_ = {false, e, 0, {}};
        waiting_ = false;
        cv_.notify_all();
    }

    std::atomic<int>        fd_{-1};
    std::mutex              write_mu_;   // serializes writes to fd
    std::mutex              bus_mu_;     // serializes transactions
    std::mutex              mu_;         // guards pending-transaction state
    std::condition_variable cv_;
    bool                    waiting_ = false;
    uint8_t                 expect_rsp_ = 0;
    uint16_t                expect_reg_ = 0;
    Result                  result_;
};

}  // namespace vehex
