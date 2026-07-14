/**
 * victron_ve_direct.cpp
 * =====================
 * Reads a Victron Energy device over the VE.Direct serial-text protocol
 * (USB-to-serial dongle) and broadcasts one JSON telemetry line per frame
 * over TCP — mirroring the picam-hailo network interface exactly.
 *
 *   DataServer   TCP data_port=8563  newline-delimited JSON, one per frame
 *   StatusServer TCP ctrl_port=8562  key=value replies to line commands:
 *                status | settings | list | get <reg> | set <reg> <value> |
 *                setraw <0xRRRR> <type> <int> | ping | restart | help
 *
 * Settings (the same parameters VictronConnect exposes) are read/written via
 * the VE.Direct HEX protocol — see vedirect_hex.hpp.
 *
 * JSON output (stdout + DataServer broadcast):
 *   {"ts_us":T,"frame":N,
 *    "device":{"pid":"0xA067","name":"SmartSolar MPPT 100|50",
 *              "serial":"HQ2241A3JKL","fw":"161"},
 *    "V":12.540,"I":0.150,"VPV":18.200,"PPV":4,
 *    "CS":5,"CS_name":"Float",
 *    "MPPT":2,"MPPT_name":"MPP tracker active",
 *    "ERR":0,"ERR_name":"No error",
 *    "LOAD":"ON","IL":0.150,
 *    "H19":123.45,"H20":0.12,"H21":45,"H22":0.98,"H23":52,"HSDS":5}
 *
 * Devices supported (auto-detected from PID field):
 *   BlueSolar MPPT, SmartSolar MPPT, BMV-700/702/712, Phoenix Inverter
 *   Any unknown device works too — raw fields are still emitted.
 *
 * Build (Linux / Raspberry Pi):
 *   cmake -B build && cmake --build build -j$(nproc)
 * Run:
 *   ./build/victron_ve_direct [--config config.ini]
 * Stream data:
 *   nc 127.0.0.1 8563
 * Query status:
 *   echo status | nc 127.0.0.1 8562
 */

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "config.hpp"
#include "mdns.hpp"
#include "vedirect_hex.hpp"

using Clock = std::chrono::steady_clock;

// ─────────────────────────────────────────────────────────────────────────────
// Global stop flag
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_stop{false};
static void signal_handler(int) { g_stop = true; }

// ─────────────────────────────────────────────────────────────────────────────
// Product ID → device name table
// ─────────────────────────────────────────────────────────────────────────────
static const char* pid_name(int pid) {
    switch (pid) {
        // BMV battery monitors
        case 0x0203: return "BMV-700";
        case 0x0205: return "BMV-702";
        case 0x0300: return "BMV-700H";
        case 0xA389: return "BMV-712 Smart";
        // BlueSolar MPPT
        case 0xA040: return "BlueSolar MPPT 75|50";
        case 0xA041: return "BlueSolar MPPT 150|35 rev1";
        case 0xA042: return "BlueSolar MPPT 75|15";
        case 0xA043: return "BlueSolar MPPT 100|15";
        case 0xA044: return "BlueSolar MPPT 100|30";
        case 0xA045: return "BlueSolar MPPT 100|50";
        case 0xA046: return "BlueSolar MPPT 150|35";
        case 0xA047: return "BlueSolar MPPT 150|45";
        case 0xA049: return "BlueSolar MPPT 100|50 rev2";
        case 0xA04A: return "BlueSolar MPPT 100|30 rev2";
        case 0xA04B: return "BlueSolar MPPT 150|35 rev3";
        case 0xA04C: return "BlueSolar MPPT 75|10";
        case 0xA04D: return "BlueSolar MPPT 150|45 rev2";
        case 0xA04E: return "BlueSolar MPPT 150|60";
        case 0xA04F: return "BlueSolar MPPT 150|45 rev3";
        case 0xA050: return "BlueSolar MPPT 150|35 rev4";
        case 0xA051: return "BlueSolar MPPT 150|100";
        case 0xA052: return "BlueSolar MPPT 150|85";
        case 0xA053: return "BlueSolar MPPT 75|15 rev2";
        case 0xA054: return "BlueSolar MPPT 75|10 rev2";
        case 0xA055: return "BlueSolar MPPT 100|15 rev2";
        case 0xA056: return "BlueSolar MPPT 100|30 rev3";
        case 0xA057: return "BlueSolar MPPT 100|50 rev3";
        case 0xA058: return "BlueSolar MPPT 150|35 rev5";
        case 0xA05A: return "BlueSolar MPPT 75|15 rev3";
        case 0xA05B: return "BlueSolar MPPT 100|15 rev3";
        case 0xA05C: return "BlueSolar MPPT 100|30 rev4";
        case 0xA05D: return "BlueSolar MPPT 100|50 rev4";
        case 0xA05E: return "BlueSolar MPPT 150|35 rev6";
        case 0xA05F: return "BlueSolar MPPT 150|45 rev4";
        // SmartSolar MPPT
        case 0xA060: return "SmartSolar MPPT 250|100";
        case 0xA061: return "SmartSolar MPPT 150|100";
        case 0xA062: return "SmartSolar MPPT 150|85";
        case 0xA063: return "SmartSolar MPPT 75|15";
        case 0xA064: return "SmartSolar MPPT 75|10";
        case 0xA065: return "SmartSolar MPPT 100|15";
        case 0xA066: return "SmartSolar MPPT 100|30";
        case 0xA067: return "SmartSolar MPPT 100|50";
        case 0xA068: return "SmartSolar MPPT 150|35";
        case 0xA069: return "SmartSolar MPPT 150|45";
        case 0xA06A: return "SmartSolar MPPT 150|60";
        case 0xA06B: return "SmartSolar MPPT 150|70";
        case 0xA06C: return "SmartSolar MPPT 150|85 rev2";
        case 0xA06D: return "SmartSolar MPPT 150|100 rev2";
        case 0xA06E: return "SmartSolar MPPT 250|60";
        case 0xA06F: return "SmartSolar MPPT 250|70";
        case 0xA070: return "SmartSolar MPPT 250|85";
        case 0xA071: return "SmartSolar MPPT 250|100 rev2";
        case 0xA072: return "SmartSolar MPPT 100|20 48V";
        case 0xA073: return "SmartSolar MPPT 150|45 rev2";
        case 0xA074: return "SmartSolar MPPT 150|60 rev2";
        case 0xA075: return "SmartSolar MPPT 150|70 rev2";
        case 0xA076: return "SmartSolar MPPT 150|85 rev3";
        case 0xA077: return "SmartSolar MPPT 150|100 rev3";
        case 0xA078: return "SmartSolar MPPT 75|15 rev2";
        case 0xA079: return "SmartSolar MPPT 75|10 rev2";
        case 0xA07A: return "SmartSolar MPPT 100|15 rev2";
        case 0xA07B: return "SmartSolar MPPT 100|30 rev2";
        case 0xA07C: return "SmartSolar MPPT 250|60 rev2";
        case 0xA07D: return "SmartSolar MPPT 250|70 rev2";
        case 0xA07E: return "SmartSolar MPPT 250|85 rev2";
        case 0xA07F: return "SmartSolar MPPT 250|100 rev3";
        // Phoenix Inverter
        case 0xA201: return "Phoenix Inverter 12V 250VA 230V";
        case 0xA209: return "Phoenix Inverter 24V 250VA 230V";
        case 0xA211: return "Phoenix Inverter 12V 375VA 230V";
        case 0xA219: return "Phoenix Inverter 24V 375VA 230V";
        case 0xA221: return "Phoenix Inverter 12V 500VA 230V";
        case 0xA229: return "Phoenix Inverter 24V 500VA 230V";
        case 0xA231: return "Phoenix Inverter 48V 500VA 230V";
        case 0xA239: return "Phoenix Inverter 12V 800VA 230V";
        case 0xA241: return "Phoenix Inverter 24V 800VA 230V";
        case 0xA249: return "Phoenix Inverter 48V 800VA 230V";
        case 0xA251: return "Phoenix Inverter 12V 1200VA 230V";
        case 0xA259: return "Phoenix Inverter 24V 1200VA 230V";
        case 0xA261: return "Phoenix Inverter 48V 1200VA 230V";
        default:     return "Unknown Victron Device";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Enum decoders for CS, MPPT, ERR
// ─────────────────────────────────────────────────────────────────────────────
static const char* cs_name(int cs) {
    switch (cs) {
        case 0:   return "Off";
        case 2:   return "Fault";
        case 3:   return "Bulk";
        case 4:   return "Absorption";
        case 5:   return "Float";
        case 7:   return "Equalize (manual)";
        case 9:   return "Starting up";
        case 245: return "Manual equalize";
        case 247: return "Test";
        case 252: return "External control";
        default:  return "Unknown";
    }
}

static const char* mppt_name(int m) {
    switch (m) {
        case 0:  return "Off";
        case 1:  return "Voltage/Current limited";
        case 2:  return "MPP tracker active";
        default: return "Unknown";
    }
}

static const char* err_name(int e) {
    switch (e) {
        case 0:   return "No error";
        case 1:   return "Battery temperature too high";
        case 2:   return "Battery voltage too high";
        case 17:  return "Charger temperature too high";
        case 18:  return "Charger over current";
        case 19:  return "Charger current reversed";
        case 20:  return "Bulk time limit exceeded";
        case 21:  return "Current sensor issue";
        case 26:  return "Terminals overheated";
        case 28:  return "Converter issue";
        case 33:  return "Input voltage too high (solar panel)";
        case 34:  return "Input current too high (solar panel)";
        case 38:  return "Input shutdown (excess battery voltage)";
        case 39:  return "Input shutdown (excess current from battery)";
        case 65:  return "Lost communication with MPPT";
        case 66:  return "Incompatible device";
        case 67:  return "BMS connection lost";
        case 114: return "CPU temperature too high";
        case 116: return "Calibration data lost";
        case 117: return "Incompatible firmware";
        case 118: return "Settings data invalid";
        case 119: return "Tester fail";
        case 121: return "DCDC converter issue";
        default:  return "Unknown";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// VE.Direct text-protocol parser
//
// The device sends frames of newline-delimited key\tvalue pairs, terminated
// by a "Checksum\t<byte>" line.  Sum of all bytes in a valid frame = 0 mod 256.
// ─────────────────────────────────────────────────────────────────────────────
class VeDirectParser {
public:
    using FrameCallback = std::function<void(
        const std::map<std::string, std::string>&)>;
    // Called for every syntactically valid HEX frame (checksum verified):
    // rsp = command/response nibble, data = payload bytes (checksum stripped)
    using HexCallback = std::function<void(
        uint8_t, const std::vector<uint8_t>&)>;

    FrameCallback on_frame;
    HexCallback   on_hex;

    void feed(uint8_t byte) {
        // HEX frames (":<cmd><data><check>\n") may interrupt a text frame at
        // any point.  Per the VE.Direct spec they are excluded from the text
        // checksum, and the interrupted text frame resumes afterwards.
        if (in_hex_) {
            if (byte == '\n') {
                process_hex();
                hex_line_.clear();
                in_hex_ = false;
            } else if (byte != '\r') {
                hex_line_ += static_cast<char>(byte);
                if (hex_line_.size() > 512) {   // runaway guard
                    hex_line_.clear();
                    in_hex_ = false;
                }
            }
            return;
        }
        if (byte == ':') {
            in_hex_ = true;
            hex_line_.clear();
            return;
        }

        sum_ = (sum_ + byte) & 0xFF;
        if (byte == '\n') {
            if (!line_.empty() && line_.back() == '\r')
                line_.pop_back();
            process_line();
            line_.clear();
        } else {
            line_ += static_cast<char>(byte);
        }
    }

    void reset() {
        line_.clear(); current_.clear(); sum_ = 0;
        hex_line_.clear(); in_hex_ = false;
    }

private:
    void process_hex() {
        uint8_t cmd;
        std::vector<uint8_t> data;
        if (!vehex::parse_frame(hex_line_, cmd, data)) {
            std::cerr << "[Parser] bad HEX frame: \":" << hex_line_ << "\"\n";
            return;
        }
        if (on_hex) on_hex(cmd, data);
    }

    void process_line() {
        auto tab = line_.find('\t');
        if (tab == std::string::npos) return;

        const std::string key = line_.substr(0, tab);
        const std::string val = line_.substr(tab + 1);

        if (key == "Checksum") {
            // VE.Direct checksum: the device chooses the checksum byte so that
            // the sum of ALL bytes in the frame (including every \r\n, including
            // the \r\n after the checksum byte itself) equals 0 mod 256.
            bool valid = (sum_ == 0);
            if (!valid)
                std::cerr << "[Parser] checksum FAIL sum=0x"
                          << std::hex << ((sum_ - '\r' - '\n') & 0xFF)
                          << std::dec << " fields=" << current_.size() << "\n";
            if (valid && !current_.empty() && on_frame)
                on_frame(current_);
            current_.clear();
            sum_ = 0;
        } else {
            current_[key] = val;
        }
    }

    std::string                        line_;
    std::string                        hex_line_;
    bool                               in_hex_ = false;
    std::map<std::string, std::string> current_;
    int                                sum_ = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// JSON formatting
//
// Converts the raw VE.Direct key→string map into a JSON line identical in
// structure to picam-hailo output: ts_us, frame counter, device block, then
// all known fields with proper SI scaling.  Unknown fields are appended as
// strings in a "raw" object so no data is silently dropped.
// ─────────────────────────────────────────────────────────────────────────────

// Returns (int_value, ok) if key exists and is a valid decimal/hex integer
static std::pair<long long, bool> get_int(
        const std::map<std::string,std::string>& m, const std::string& k) {
    auto it = m.find(k);
    if (it == m.end()) return {0, false};
    try {
        size_t pos;
        long long v = std::stoll(it->second, &pos, 0);  // base 0: auto-detect 0x hex
        return {v, true};
    } catch (...) { return {0, false}; }
}

static std::string format_frame(
        uint64_t                                   frame,
        int64_t                                    ts_us,
        const std::map<std::string,std::string>&   fields)
{
    // Known string fields (not emitted as raw)
    static const std::vector<std::string> known_str  = {"PID","SER#","FW","FWE","LOAD","Alarm","Relay","AR"};
    // Known numeric fields (not emitted as raw)
    static const std::vector<std::string> known_num  = {
        "V","VS","VM","DM","VPV","PPV","I","IL","P","CE","SOC","TTG",
        "H1","H2","H3","H4","H5","H6","H7","H8","H9","H10","H11",
        "H12","H13","H14","H15","H16","H17","H18","H19","H20","H21",
        "H22","H23","HSDS","CS","MPPT","ERR","OR","WARN","MPPT",
        "AC_OUT_V","AC_OUT_I","AC_OUT_S","WARN"
    };

    std::string j;
    j.reserve(512);

    j += "{\"ts_us\":";    j += std::to_string(ts_us);
    j += ",\"frame\":";    j += std::to_string(frame);

    // ── Device block ────────────────────────────────────────────────────────
    auto [pid_raw, have_pid] = get_int(fields, "PID");
    std::string pid_str  = have_pid ? fields.at("PID") : "0x0000";
    const char* dev_name = have_pid ? pid_name(static_cast<int>(pid_raw)) : "Unknown";
    std::string serial   = "";
    if (auto it = fields.find("SER#"); it != fields.end()) serial = it->second;
    std::string fw       = "";
    if (auto it = fields.find("FW");   it != fields.end()) fw     = it->second;

    j += ",\"device\":{\"pid\":\""; j += pid_str;
    j += "\",\"name\":\"";          j += dev_name;
    j += "\",\"serial\":\"";        j += serial;
    j += "\",\"fw\":\"";            j += fw; j += "\"}";

    // Helper: emit float field
    auto emit_f = [&](const char* key, const char* jkey, double scale) {
        auto [v, ok] = get_int(fields, key);
        if (!ok) return;
        char buf[64];
        snprintf(buf, sizeof(buf), ",\"%s\":%.3f", jkey, v * scale);
        j += buf;
    };

    // Helper: emit integer field
    auto emit_i = [&](const char* key, const char* jkey) {
        auto [v, ok] = get_int(fields, key);
        if (!ok) return;
        j += ",\""; j += jkey; j += "\":"; j += std::to_string(v);
    };

    // Helper: emit string field
    auto emit_s = [&](const char* key, const char* jkey) {
        auto it = fields.find(key);
        if (it == fields.end()) return;
        j += ",\""; j += jkey; j += "\":\""; j += it->second; j += "\"";
    };

    // ── Voltages and currents ────────────────────────────────────────────────
    emit_f("V",        "V",        0.001);  // battery voltage mV → V
    emit_f("VS",       "VS",       0.001);  // aux/starter voltage mV → V
    emit_f("VM",       "VM",       0.001);  // mid-point voltage mV → V
    emit_f("VPV",      "VPV",      0.001);  // panel voltage mV → V
    emit_f("I",        "I",        0.001);  // battery current mA → A
    emit_f("IL",       "IL",       0.001);  // load current mA → A
    emit_i("PPV",      "PPV");              // panel power W
    emit_i("P",        "P");                // battery power W
    emit_f("AC_OUT_V", "AC_OUT_V", 0.01);  // AC output voltage 0.01V → V
    emit_f("AC_OUT_I", "AC_OUT_I", 0.1);   // AC output current 0.1A → A
    emit_i("AC_OUT_S", "AC_OUT_S");         // AC apparent power VA

    // ── Battery monitor fields ────────────────────────────────────────────────
    emit_f("CE",  "CE",  0.001);   // consumed Ah, mAh → Ah
    emit_f("SOC", "SOC", 0.1);     // state of charge, 0.1% → %
    emit_i("TTG", "TTG");           // time to go, minutes

    // ── Charge state (enum) ───────────────────────────────────────────────────
    {
        auto [v, ok] = get_int(fields, "CS");
        if (ok) {
            j += ",\"CS\":"; j += std::to_string(v);
            j += ",\"CS_name\":\""; j += cs_name(static_cast<int>(v)); j += "\"";
        }
    }

    // ── MPPT tracker state (enum) ─────────────────────────────────────────────
    {
        auto [v, ok] = get_int(fields, "MPPT");
        if (ok) {
            j += ",\"MPPT\":"; j += std::to_string(v);
            j += ",\"MPPT_name\":\""; j += mppt_name(static_cast<int>(v)); j += "\"";
        }
    }

    // ── Error code (enum) ─────────────────────────────────────────────────────
    {
        auto [v, ok] = get_int(fields, "ERR");
        if (ok) {
            j += ",\"ERR\":"; j += std::to_string(v);
            j += ",\"ERR_name\":\""; j += err_name(static_cast<int>(v)); j += "\"";
        }
    }

    // ── Off reason (hex bitfield) ─────────────────────────────────────────────
    emit_s("OR",   "OR");
    // ── Load output ──────────────────────────────────────────────────────────
    emit_s("LOAD", "LOAD");
    // ── Alarm ────────────────────────────────────────────────────────────────
    emit_s("Alarm", "alarm");
    emit_i("AR",    "AR");

    // ── Yield history (0.01 kWh per count) ───────────────────────────────────
    emit_f("H19", "H19", 0.01);  // yield total    kWh
    emit_f("H20", "H20", 0.01);  // yield today    kWh
    emit_i("H21", "H21");         // max power today W
    emit_f("H22", "H22", 0.01);  // yield yesterday kWh
    emit_i("H23", "H23");         // max power yesterday W
    emit_i("HSDS","HSDS");        // day sequence number

    // ── BMV history ──────────────────────────────────────────────────────────
    // H1..H18 are various historical stats; emit as integers
    for (int n = 1; n <= 18; ++n) {
        auto key = "H" + std::to_string(n);
        // skip yield fields already emitted above
        if (n == 19 || n == 20 || n == 21 || n == 22 || n == 23) continue;
        auto [v, ok] = get_int(fields, key);
        if (ok) { j += ",\""; j += key; j += "\":"; j += std::to_string(v); }
    }

    // ── Any fields not yet emitted go into "raw" ─────────────────────────────
    // Build a set of already-handled keys
    static const std::vector<std::string> handled = {
        "PID","SER#","FW","FWE",
        "V","VS","VM","DM","VPV","PPV","I","IL","P","CE","SOC","TTG",
        "H1","H2","H3","H4","H5","H6","H7","H8","H9","H10","H11",
        "H12","H13","H14","H15","H16","H17","H18",
        "H19","H20","H21","H22","H23","HSDS",
        "CS","MPPT","ERR","OR","LOAD","Alarm","AR",
        "AC_OUT_V","AC_OUT_I","AC_OUT_S","WARN"
    };

    std::string raw;
    for (const auto& [k, v] : fields) {
        bool skip = false;
        for (const auto& h : handled) if (h == k) { skip = true; break; }
        if (skip) continue;
        if (!raw.empty()) raw += ',';
        raw += '"'; raw += k; raw += "\":\""; raw += v; raw += '"';
    }
    if (!raw.empty()) { j += ",\"raw\":{"; j += raw; j += '}'; }

    j += '}';
    return j;
}

// ─────────────────────────────────────────────────────────────────────────────
// Shared status — written by main loop, read by StatusServer
// ─────────────────────────────────────────────────────────────────────────────
struct VictronStatus {
    std::mutex  mu;
    uint64_t    frame_count  = 0;
    float       fps          = 0;
    std::string device_name  = "none";
    std::string serial       = "";
    std::string fw           = "";
    std::string pid          = "";
    std::string port;
    // Latest numeric readings (set on each frame if present)
    double V   = 0, I   = 0, VPV = 0, PPV = 0;
    double SOC = -1, CE  = 0, TTG = -1;
    double H20 = 0;  // yield today kWh
    int    CS  = -1, MPPT = -1, ERR = -1;
    std::string LOAD;
    std::string last_json;
};
static VictronStatus g_status;

// ─────────────────────────────────────────────────────────────────────────────
// Setting-value enum decoders (mirrors VictronConnect labels)
// ─────────────────────────────────────────────────────────────────────────────
static const char* battery_type_name(int v) {
    switch (v) {
        case 1:   return "Gel Victron long life (14.1V)";
        case 2:   return "Gel Victron deep discharge (14.3V)";
        case 3:   return "Gel Victron deep discharge (14.4V)";
        case 4:   return "AGM Victron deep discharge (14.7V)";
        case 5:   return "Tubular plate cyclic mode 1 (14.9V)";
        case 6:   return "Tubular plate cyclic mode 2 (15.1V)";
        case 7:   return "Tubular plate cyclic mode 3 (15.3V)";
        case 8:   return "LiFePO4 (14.2V)";
        case 255: return "User defined";
        default:  return "Unknown";
    }
}

static const char* load_mode_name(int v) {
    switch (v) {
        case 0:  return "Off";
        case 1:  return "Auto (BatteryLife)";
        case 2:  return "Alternative 1";
        case 3:  return "Alternative 2";
        case 4:  return "Always on";
        case 5:  return "User defined 1 (off < low, on > high)";
        case 6:  return "User defined 2 (on between low and high)";
        case 7:  return "Automatic energy selector (AES)";
        default: return "Unknown";
    }
}

static const char* device_mode_name(int v) {
    switch (v) {
        case 1:  return "Charger on";
        case 2:  return "Inverter on";
        case 4:  return "Off";
        case 5:  return "Eco";
        default: return "Unknown";
    }
}

// Format raw register value as "value[ unit][ (label)]"
static std::string format_reg_value(const vehex::RegDef& r,
                                    const std::vector<uint8_t>& raw) {
    int64_t v = vehex::decode_value(raw, r.type);
    char buf[96];
    if (r.scale == 1.0)
        snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(v));
    else
        snprintf(buf, sizeof(buf), "%.3f", v * r.scale);
    std::string out = buf;
    if (r.unit[0]) { out += ' '; out += r.unit; }

    const char* label = nullptr;
    if      (!strcmp(r.name, "battery_type")) label = battery_type_name(static_cast<int>(v));
    else if (!strcmp(r.name, "load_mode"))    label = load_mode_name(static_cast<int>(v));
    else if (!strcmp(r.name, "device_mode"))  label = device_mode_name(static_cast<int>(v));
    else if (!strcmp(r.name, "device_state")) label = cs_name(static_cast<int>(v));
    if (label) { out += " ("; out += label; out += ')'; }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// StatusServer — TCP plain-text on ctrl_port.
//   status                       existing key=value status dump
//   settings                     read every register the device supports
//   list                         show the register catalogue
//   get <name|0xRRRR>            read one setting
//   set <name> <value>           write one setting (engineering units)
//   setraw <0xRRRR> <u8|u16|s16|u32|s32> <int>   raw register write
//   ping / restart               HEX ping / device reboot
// ─────────────────────────────────────────────────────────────────────────────
class StatusServer {
public:
    StatusServer(int port, vehex::HexBus* bus, bool allow_set)
        : port_(port), bus_(bus), allow_set_(allow_set) {}
    ~StatusServer() { if (fd_ >= 0) ::close(fd_); }

    void start() {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(static_cast<uint16_t>(port_));
        ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        ::listen(fd_, 8);
        thread_ = std::thread(&StatusServer::acceptLoop, this);
        thread_.detach();
        std::cerr << "[Status] Listening on 0.0.0.0:" << port_ << "\n";
    }

private:
    void acceptLoop() {
        while (!g_stop) {
            int cfd = ::accept(fd_, nullptr, nullptr);
            if (cfd < 0) { if (errno == EINTR) continue; break; }
            std::thread([this, cfd]{ handle(cfd); }).detach();
        }
    }

    // Split a command line into whitespace-separated tokens
    static std::vector<std::string> tokenize(const std::string& s) {
        std::vector<std::string> out;
        std::istringstream iss(s);
        std::string t;
        while (iss >> t) out.push_back(t);
        return out;
    }

    // Resolve "<name>" or "0xRRRR" to a register.  For raw hex ids without a
    // catalogue entry, a synthetic read-only U32 entry is returned via `tmp`.
    static const vehex::RegDef* resolve(const std::string& arg,
                                        vehex::RegDef& tmp) {
        if (const auto* r = vehex::find_register(arg)) return r;
        if (arg.rfind("0x", 0) == 0 || arg.rfind("0X", 0) == 0) {
            try {
                unsigned long id = std::stoul(arg, nullptr, 16);
                if (id > 0xFFFF) return nullptr;
                if (const auto* r = vehex::find_register(
                        static_cast<uint16_t>(id))) return r;
                tmp = {"raw", static_cast<uint16_t>(id), vehex::VType::U32,
                       1.0, "", false, vehex::DEV_ALL, ""};
                return &tmp;
            } catch (...) { return nullptr; }
        }
        return nullptr;
    }

    std::string cmd_get(const std::vector<std::string>& tok) {
        if (tok.size() != 2) return "ok=false\nerror=usage: get <name|0xRRRR>\n";
        vehex::RegDef tmp{};
        const auto* r = resolve(tok[1], tmp);
        if (!r) return "ok=false\nerror=unknown register '" + tok[1] + "' (try: list)\n";

        auto res = bus_->get(r->reg);
        if (!res.ok) return "ok=false\nregister=" + std::string(r->name)
                          + "\nerror=" + res.error + "\n";

        char hexid[16];
        snprintf(hexid, sizeof(hexid), "0x%04X", r->reg);
        std::string reply = "ok=true\nregister=" + std::string(r->name)
                          + "\nid=" + hexid
                          + "\nvalue=" + format_reg_value(*r, res.value)
                          + "\nraw=";
        char b[8];
        for (uint8_t byte : res.value) {
            snprintf(b, sizeof(b), "%02X", byte);
            reply += b;
        }
        reply += "\n";
        return reply;
    }

    std::string cmd_set(const std::vector<std::string>& tok) {
        if (!allow_set_)
            return "ok=false\nerror=set disabled in config (control.allow_set)\n";
        if (tok.size() != 3) return "ok=false\nerror=usage: set <name> <value>\n";

        const auto* r = vehex::find_register(tok[1]);
        if (!r) return "ok=false\nerror=unknown setting '" + tok[1]
                     + "' (raw ids: use setraw)\n";
        if (!r->writable)
            return "ok=false\nerror=register '" + tok[1] + "' is read-only\n";

        double eng;
        try { eng = std::stod(tok[2]); }
        catch (...) { return "ok=false\nerror=invalid value '" + tok[2] + "'\n"; }

        // engineering → raw, round to nearest step
        int64_t raw = static_cast<int64_t>(
            eng / r->scale + (eng >= 0 ? 0.5 : -0.5));

        auto res = bus_->set(r->reg, vehex::encode_value(raw, r->type));
        if (!res.ok) return "ok=false\nregister=" + std::string(r->name)
                          + "\nerror=" + res.error + "\n";

        // Device echoes the value it actually stored — report that back
        return "ok=true\nregister=" + std::string(r->name)
             + "\nvalue=" + format_reg_value(*r, res.value) + "\n";
    }

    std::string cmd_setraw(const std::vector<std::string>& tok) {
        if (!allow_set_)
            return "ok=false\nerror=set disabled in config (control.allow_set)\n";
        if (tok.size() != 4)
            return "ok=false\nerror=usage: setraw <0xRRRR> <u8|u16|s16|u32|s32> <int>\n";

        unsigned long id;
        try { id = std::stoul(tok[1], nullptr, 16); }
        catch (...) { return "ok=false\nerror=invalid register id\n"; }
        if (id > 0xFFFF) return "ok=false\nerror=register id out of range\n";

        vehex::VType t;
        if      (tok[2] == "u8")  t = vehex::VType::U8;
        else if (tok[2] == "u16") t = vehex::VType::U16;
        else if (tok[2] == "s16") t = vehex::VType::S16;
        else if (tok[2] == "u32") t = vehex::VType::U32;
        else if (tok[2] == "s32") t = vehex::VType::S32;
        else return "ok=false\nerror=type must be u8|u16|s16|u32|s32\n";

        int64_t raw;
        try { raw = std::stoll(tok[3], nullptr, 0); }
        catch (...) { return "ok=false\nerror=invalid value\n"; }

        auto res = bus_->set(static_cast<uint16_t>(id),
                             vehex::encode_value(raw, t));
        if (!res.ok) return "ok=false\nerror=" + res.error + "\n";
        return "ok=true\n";
    }

    std::string cmd_settings() {
        // Determine device class from last seen PID
        int pid = 0;
        {
            std::lock_guard<std::mutex> lk(g_status.mu);
            try { if (!g_status.pid.empty()) pid = std::stoi(g_status.pid, nullptr, 0); }
            catch (...) {}
        }
        uint8_t devclass = vehex::device_class_from_pid(pid);

        std::string reply = "ok=true\n";
        int read_ok = 0, skipped = 0;
        for (const auto& r : vehex::registers()) {
            if (!(r.devmask & devclass)) continue;
            auto res = bus_->get(r.reg);
            if (!res.ok) {
                // register genuinely unsupported by this model → silently skip
                if (res.flags & 0x03) { ++skipped; continue; }
                reply += std::string(r.name) + "=ERROR (" + res.error + ")\n";
                continue;
            }
            reply += std::string(r.name) + "=" + format_reg_value(r, res.value) + "\n";
            ++read_ok;
        }
        reply += "read=" + std::to_string(read_ok)
               + "\nunsupported=" + std::to_string(skipped) + "\n";
        return reply;
    }

    std::string cmd_list() {
        std::string reply = "ok=true\n";
        char line[256];
        for (const auto& r : vehex::registers()) {
            const char* type =
                r.type == vehex::VType::U8  ? "u8"  :
                r.type == vehex::VType::U16 ? "u16" :
                r.type == vehex::VType::S16 ? "s16" :
                r.type == vehex::VType::U32 ? "u32" : "s32";
            snprintf(line, sizeof(line), "%s=0x%04X %s %s%s%s -- %s\n",
                     r.name, r.reg, type,
                     r.writable ? "rw" : "ro",
                     r.unit[0] ? " unit=" : "", r.unit,
                     r.help);
            reply += line;
        }
        return reply;
    }

    std::string cmd_ping() {
        auto res = bus_->ping();
        if (!res.ok) return "ok=false\nerror=" + res.error + "\n";
        std::string reply = "ok=true\n";
        if (res.value.size() >= 2) {
            // un16 version, bit14 flags candidate/official
            int ver = res.value[0] | (res.value[1] << 8);
            char b[64];
            snprintf(b, sizeof(b), "app_version=0x%04X\n", ver);
            reply += b;
        }
        return reply;
    }

    std::string cmd_restart() {
        if (!allow_set_)
            return "ok=false\nerror=set disabled in config (control.allow_set)\n";
        auto res = bus_->restart();
        if (!res.ok) return "ok=false\nerror=" + res.error + "\n";
        return "ok=true\nnote=device is rebooting\n";
    }

    static std::string cmd_help() {
        return
            "ok=true\n"
            "commands=status settings list get set setraw ping restart help\n"
            "usage_get=get <name|0xRRRR>\n"
            "usage_set=set <name> <value>          ; engineering units, e.g. set absorption_voltage 14.40\n"
            "usage_setraw=setraw <0xRRRR> <u8|u16|s16|u32|s32> <int>\n"
            "note=after a hex command the device pauses text telemetry for a few seconds\n";
    }

    void handle(int cfd) {
        struct timeval tv{2, 0};
        ::setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        std::string line;
        char c;
        while (::recv(cfd, &c, 1, 0) == 1) {
            if (c == '\n') break;
            if (c != '\r') line += c;
        }
        std::string reply;
        if (line == "status" || line.empty()) {
            std::lock_guard<std::mutex> lk(g_status.mu);
            char buf[1024];
            snprintf(buf, sizeof(buf),
                "ok=true\n"
                "frame=%llu\n"
                "fps=%.1f\n"
                "device=%s\n"
                "pid=%s\n"
                "serial=%s\n"
                "fw=%s\n"
                "port=%s\n"
                "V=%.3f\n"
                "I=%.3f\n"
                "VPV=%.3f\n"
                "PPV=%.0f\n"
                "CS=%d\n"
                "CS_name=%s\n"
                "ERR=%d\n"
                "ERR_name=%s\n"
                "LOAD=%s\n"
                "H20=%.2f\n"
                "\n",
                (unsigned long long)g_status.frame_count,
                g_status.fps,
                g_status.device_name.c_str(),
                g_status.pid.c_str(),
                g_status.serial.c_str(),
                g_status.fw.c_str(),
                g_status.port.c_str(),
                g_status.V, g_status.I, g_status.VPV, g_status.PPV,
                g_status.CS, cs_name(g_status.CS),
                g_status.ERR, err_name(g_status.ERR),
                g_status.LOAD.empty() ? "N/A" : g_status.LOAD.c_str(),
                g_status.H20);
            reply = buf;
            if (g_status.SOC >= 0) {
                char soc[64];
                snprintf(soc, sizeof(soc), "SOC=%.1f\nTTG=%.0f\n",
                         g_status.SOC, g_status.TTG);
                reply += soc;
            }
        } else {
            auto tok = tokenize(line);
            const std::string cmd = tok.empty() ? "" : tok[0];
            if      (cmd == "get")      reply = cmd_get(tok);
            else if (cmd == "set")      reply = cmd_set(tok);
            else if (cmd == "setraw")   reply = cmd_setraw(tok);
            else if (cmd == "settings") reply = cmd_settings();
            else if (cmd == "list")     reply = cmd_list();
            else if (cmd == "ping")     reply = cmd_ping();
            else if (cmd == "restart")  reply = cmd_restart();
            else if (cmd == "help")     reply = cmd_help();
            else reply = "ok=false\nerror=unknown command\n"
                         "commands=status settings list get set setraw ping restart help\n";
            reply += "\n";
        }
        ::send(cfd, reply.data(), reply.size(), MSG_NOSIGNAL);
        ::close(cfd);
    }

    int             port_;
    vehex::HexBus*  bus_;
    bool            allow_set_;
    int             fd_ = -1;
    std::thread     thread_;
};

// ─────────────────────────────────────────────────────────────────────────────
// DataServer — TCP broadcast, one JSON line per Victron frame
// Identical pattern to picam-hailo DetectionServer.
// ─────────────────────────────────────────────────────────────────────────────
class DataServer {
public:
    explicit DataServer(int port) : port_(port) {}
    ~DataServer() { if (listenFd_ >= 0) ::close(listenFd_); }

    void start() {
        listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(static_cast<uint16_t>(port_));
        ::bind(listenFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        ::listen(listenFd_, 16);
        thread_ = std::thread(&DataServer::acceptLoop, this);
        thread_.detach();
        std::cerr << "[Data] Listening on 0.0.0.0:" << port_ << "\n";
    }

    void broadcast(const std::string& line) {
        std::string msg = line + "\n";
        std::lock_guard<std::mutex> lk(mu_);
        std::vector<int> alive;
        for (int fd : clients_) {
            ssize_t n = ::send(fd, msg.data(), msg.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
            if (n > 0 || (n < 0 && errno == EAGAIN)) {
                alive.push_back(fd);
            } else {
                std::cerr << "\n[Data] Client fd=" << fd
                          << " dropped: " << strerror(errno) << "\n";
                ::close(fd);
            }
        }
        clients_ = std::move(alive);
    }

private:
    void acceptLoop() {
        while (!g_stop) {
            sockaddr_in peer{};
            socklen_t   plen = sizeof(peer);
            int cfd = ::accept(listenFd_,
                               reinterpret_cast<sockaddr*>(&peer), &plen);
            if (cfd < 0) { if (errno == EINTR) continue; break; }
            char ip[INET_ADDRSTRLEN];
            ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
            std::cerr << "\n[Data] Client: " << ip
                      << ":" << ntohs(peer.sin_port) << "\n";
            std::lock_guard<std::mutex> lk(mu_);
            clients_.push_back(cfd);
        }
    }

    int                port_;
    int                listenFd_ = -1;
    std::thread        thread_;
    mutable std::mutex mu_;
    std::vector<int>   clients_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Serial port — POSIX termios, 19200 8N1 raw
// Returns fd >= 0 on success, -1 on failure.
// ─────────────────────────────────────────────────────────────────────────────
static int open_serial(const std::string& path, int baud = 19200) {
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[Serial] open(" << path << "): " << strerror(errno) << "\n";
        return -1;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "[Serial] tcgetattr: " << strerror(errno) << "\n";
        ::close(fd);
        return -1;
    }

    speed_t speed = B19200;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            std::cerr << "[Serial] Unknown baud " << baud << ", using 19200\n";
            speed = B19200;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;  // 8 data bits
    tty.c_cflag &= ~PARENB;                         // no parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                        // no hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);        // no software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag  = 0;  // raw output
    tty.c_lflag  = 0;  // raw input (no echo, no signals, no canonical)

    tty.c_cc[VMIN]  = 1;   // block until at least 1 byte
    tty.c_cc[VTIME] = 10;  // or 1-second inter-byte timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "[Serial] tcsetattr: " << strerror(errno) << "\n";
        ::close(fd);
        return -1;
    }

    // Switch back to blocking mode
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    tcflush(fd, TCIFLUSH);
    return fd;
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGHUP,  signal_handler);
    std::signal(SIGPIPE, SIG_IGN);

    std::string cfg_path = "config.ini";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i + 1 < argc) cfg_path = argv[++i];
        else if (a == "--help" || a == "-h") {
            std::cout << "Usage: " << argv[0] << " [--config config.ini]\n"
                      << "  Stream data : nc 127.0.0.1 8563\n"
                      << "  Status      : echo status | nc 127.0.0.1 8562\n"
                      << "  Settings    : echo settings | nc 127.0.0.1 8562\n"
                      << "  Read one    : echo 'get absorption_voltage' | nc 127.0.0.1 8562\n"
                      << "  Write one   : echo 'set absorption_voltage 14.40' | nc 127.0.0.1 8562\n"
                      << "  Catalogue   : echo list | nc 127.0.0.1 8562\n";
            return 0;
        }
    }

    Config cfg(cfg_path);

    const std::string device       = cfg.get_str("device.port", "/dev/ttyUSB0");
    const int         baud         = cfg.get_int("device.baud", 19200);
    const int         ctrl_port    = cfg.get_int("output.ctrl_port", 8562);
    const int         data_port    = cfg.get_int("output.data_port", 8563);
    const int         retry_s      = cfg.get_int("device.retry_secs", 5);
    const bool        mdns_enabled = cfg.get_bool("mdns.enabled", true);
    const std::string mdns_name    = cfg.get_str("mdns.name", "victron-ve-direct");
    const bool        allow_set    = cfg.get_bool("control.allow_set", true);

    {
        std::lock_guard<std::mutex> lk(g_status.mu);
        g_status.port = device;
    }

    std::cerr << "[Config] device  : " << device << "  baud=" << baud << "\n"
              << "[Config] status  : 0.0.0.0:" << ctrl_port << "\n"
              << "[Config] data    : 0.0.0.0:" << data_port << "\n"
              << "[Config] mdns    : " << (mdns_enabled ? mdns_name : "disabled") << "\n"
              << "[Config] set     : " << (allow_set ? "enabled" : "disabled") << "\n";

    // ── Servers ───────────────────────────────────────────────────────────────
    vehex::HexBus hex_bus;
    DataServer    data_srv(data_port);
    StatusServer  status_srv(ctrl_port, &hex_bus, allow_set);
    data_srv.start();
    status_srv.start();

    MdnsAnnouncer mdns(mdns_name, {
        {"_victron-data._tcp",   static_cast<uint16_t>(data_port)},
        {"_victron-status._tcp", static_cast<uint16_t>(ctrl_port)},
    });
    if (mdns_enabled) mdns.start();

    std::cerr << "[Main] Running.\n"
              << "[Main]   Data stream : nc 127.0.0.1 " << data_port << "\n"
              << "[Main]   Status      : echo status | nc 127.0.0.1 " << ctrl_port << "\n";

    // ── Main loop: open serial → parse → broadcast ────────────────────────────
    uint64_t frame_count = 0;
    auto     t_start     = Clock::now();
    auto     t_last_fps  = Clock::now();
    uint64_t frames_since = 0;

    while (!g_stop) {
        // Open / reopen serial port
        int fd = open_serial(device, baud);
        if (fd < 0) {
            std::cerr << "[Serial] Retrying in " << retry_s << "s...\n";
            for (int i = 0; i < retry_s * 10 && !g_stop; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        std::cerr << "[Serial] Opened " << device << " @ " << baud << " baud\n";
        hex_bus.attach(fd);

        VeDirectParser parser;
        parser.on_hex = [&](uint8_t rsp, const std::vector<uint8_t>& data) {
            hex_bus.on_frame(rsp, data);
        };

        parser.on_frame = [&](const std::map<std::string,std::string>& fields) {
            ++frame_count;
            ++frames_since;

            auto ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            // ── Format JSON ───────────────────────────────────────────────────
            std::string json = format_frame(frame_count, ts_us, fields);

            // ── Publish ───────────────────────────────────────────────────────
            std::cout << json << "\n" << std::flush;
            data_srv.broadcast(json);

            // ── Update shared status ──────────────────────────────────────────
            {
                std::lock_guard<std::mutex> lk(g_status.mu);
                g_status.frame_count = frame_count;
                g_status.last_json   = json;

                auto [pid_v, have_pid] = get_int(fields, "PID");
                if (have_pid) {
                    g_status.device_name = pid_name(static_cast<int>(pid_v));
                    g_status.pid         = fields.at("PID");
                }
                if (auto it = fields.find("SER#"); it != fields.end()) g_status.serial = it->second;
                if (auto it = fields.find("FW");   it != fields.end()) g_status.fw     = it->second;

                auto scale_get = [&](const char* k, double scale) -> double {
                    auto [v, ok] = get_int(fields, k);
                    return ok ? v * scale : 0.0;
                };
                g_status.V   = scale_get("V",   0.001);
                g_status.I   = scale_get("I",   0.001);
                g_status.VPV = scale_get("VPV", 0.001);
                g_status.PPV = scale_get("PPV", 1.0);
                g_status.SOC = scale_get("SOC", 0.1);
                g_status.CE  = scale_get("CE",  0.001);
                g_status.TTG = scale_get("TTG", 1.0);
                g_status.H20 = scale_get("H20", 0.01);

                auto [cs,   ok_cs]   = get_int(fields, "CS");
                auto [mppt, ok_mppt] = get_int(fields, "MPPT");
                auto [err,  ok_err]  = get_int(fields, "ERR");
                if (ok_cs)   g_status.CS   = static_cast<int>(cs);
                if (ok_mppt) g_status.MPPT = static_cast<int>(mppt);
                if (ok_err)  g_status.ERR  = static_cast<int>(err);
                if (auto it = fields.find("LOAD"); it != fields.end())
                    g_status.LOAD = it->second;
            }

            // ── 1 Hz stderr stats ─────────────────────────────────────────────
            auto now = Clock::now();
            double since = std::chrono::duration<double>(now - t_last_fps).count();
            if (since >= 1.0) {
                double fps     = frames_since / since;
                double avg_fps = frame_count /
                    std::chrono::duration<double>(now - t_start).count();

                {
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    g_status.fps = static_cast<float>(fps);
                }

                // Read values under lock for display
                double V, I, VPV, PPV, H20;
                int CS, ERR;
                std::string LOAD;
                {
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    V = g_status.V; I = g_status.I;
                    VPV = g_status.VPV; PPV = g_status.PPV;
                    H20 = g_status.H20;
                    CS = g_status.CS; ERR = g_status.ERR;
                    LOAD = g_status.LOAD;
                }

                fprintf(stderr,
                    "\r[%6llu] fps:%.1f avg:%.1f  "
                    "V:%5.2fV I:%+.3fA  PV:%5.1fV %3.0fW  "
                    "CS:%-20s  ERR:%d  LOAD:%-3s  "
                    "today:%.2fkWh   ",
                    (unsigned long long)frame_count, fps, avg_fps,
                    V, I, VPV, PPV,
                    cs_name(CS), ERR,
                    LOAD.empty() ? "N/A" : LOAD.c_str(),
                    H20);
                fflush(stderr);

                t_last_fps   = now;
                frames_since = 0;
            }
        };  // end parser.on_frame

        // ── Read loop ─────────────────────────────────────────────────────────
        uint8_t byte;
        while (!g_stop) {
            ssize_t n = ::read(fd, &byte, 1);
            if (n == 1) {
                parser.feed(byte);
            } else if (n == 0) {
                // EOF — device disconnected
                break;
            } else {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                std::cerr << "\n[Serial] read error: " << strerror(errno) << "\n";
                break;
            }
        }

        hex_bus.detach();
        ::close(fd);
        fd = -1;
        parser.reset();

        if (g_stop) break;

        std::cerr << "\n[Serial] Disconnected. Retrying in " << retry_s << "s...\n";
        for (int i = 0; i < retry_s * 10 && !g_stop; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    fprintf(stderr, "\n");
    double total = std::chrono::duration<double>(Clock::now() - t_start).count();
    std::cerr << "[Main] Done. frames=" << frame_count
              << "  avg_fps=" << (total > 0 ? frame_count / total : 0) << "\n";
    return 0;
}
