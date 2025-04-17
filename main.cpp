#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono> // Required for sleep_for
#include <cstdlib>  // For std::system
#include <json.hpp> // Include the JSON library
#include <boost/asio.hpp>    // Include Boost ASIO for TCP communication
#include "flight_control_sample.hpp" // Still needed for monitoredTakeoff/Landing
#include "flight_sample.hpp"         // Still needed for FlightSample class definition
#include "dji_linux_helpers.hpp"
#include <limits> // For numeric limits
#include <fstream> // For file reading AND LOGGING
#include <cmath> // For std::abs, std::isnan, pow, sqrt, tan, atan, M_PI
#include <atomic> // For thread-safe stop flag
#include <cstring> // For strchr
#include <stdexcept> // For standard exceptions
#include <array> // For std::array used in read_some buffer
#include <ctime>  // For checking polling timestamp
#include <streambuf> // For TeeBuf
#include <mutex>     // For TeeBuf thread safety
#include <memory>    // For unique_ptr
#include <iomanip>   // For std::put_time in timestamp

// Include the headers that define Control flags, CtrlData, FlightController, and Vehicle
#include "dji_control.hpp"           // Defines Control class, CtrlData, enums
#include "dji_flight_controller.hpp" // Defines FlightController
#include "dji_vehicle.hpp"           // Defines Vehicle class which contains Control*
#include "dji_telemetry.hpp"         // For Telemetry types
#include "dji_status.hpp"            // For VehicleStatus enums/constants
#include "dji_ack.hpp"               // For ACK::getError

// Define M_PI if not already defined (likely in cmath)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace DJI::OSDK;
using json = nlohmann::json;
using boost::asio::ip::tcp;

// --- Logging Setup ---

// TeeBuf writes output to two streambufs (e.g., console and file)
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* sb1, std::streambuf* sb2) : sb1_(sb1), sb2_(sb2) {}

protected:
    // Called when buffer is full or on explicit flush/endl
    virtual int sync() override {
        std::lock_guard<std::mutex> lock(mutex_);
        int r1 = sb1_->pubsync();
        int r2 = sb2_->pubsync();
        return (r1 == 0 && r2 == 0) ? 0 : -1;
    }

    // Called when a character is written
    virtual int_type overflow(int_type c = traits_type::eof()) override {
        if (traits_type::eq_int_type(c, traits_type::eof())) {
            return sync() == -1 ? traits_type::eof() : traits_type::not_eof(c);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        int_type const r1 = sb1_->sputc(c);
        int_type const r2 = sb2_->sputc(c);

        if (traits_type::eq_int_type(r1, traits_type::eof()) ||
            traits_type::eq_int_type(r2, traits_type::eof())) {
            return traits_type::eof(); // Indicate error if either fails
        }
        return traits_type::not_eof(c); // Indicate success
    }

private:
    std::streambuf* sb1_;
    std::streambuf* sb2_;
    std::mutex mutex_; // Protect concurrent writes from different threads
};

// RAII class to manage redirection and restoration of streams
class LogRedirector {
public:
    LogRedirector(const std::string& log_filename)
        : log_file_(log_filename, std::ios::app), // Open in append mode
          original_cout_buf_(nullptr),
          original_cerr_buf_(nullptr)
    {
        if (!log_file_.is_open()) {
            // Use original cerr because redirection hasn't happened yet
            if (original_cerr_buf_) std::cerr.rdbuf(original_cerr_buf_);
            std::cerr << "FATAL ERROR: Could not open log file: " << log_filename << std::endl;
             // Restore original cerr buffer in case it was temporarily changed above
            if (original_cerr_buf_) std::cerr.rdbuf(original_cerr_buf_);
            // Log file couldn't be opened, so don't redirect
            return;
        }

        original_cout_buf_ = std::cout.rdbuf(); // Save original cout buffer
        original_cerr_buf_ = std::cerr.rdbuf(); // Save original cerr buffer

        // Use reset(new ...) instead of std::make_unique for C++11 compatibility
        cout_tee_buf_.reset(new TeeBuf(original_cout_buf_, log_file_.rdbuf()));
        cerr_tee_buf_.reset(new TeeBuf(original_cerr_buf_, log_file_.rdbuf())); // Also log cerr to the same file


        std::cout.rdbuf(cout_tee_buf_.get()); // Redirect cout
        std::cerr.rdbuf(cerr_tee_buf_.get()); // Redirect cerr

        std::cout << "\n--- Log Start [" << getCurrentTimestamp() << "] ---" << std::endl; // Added newline for separation
    }

    ~LogRedirector() {
         std::cout << "--- Log End [" << getCurrentTimestamp() << "] ---\n" << std::endl; // Added newline for separation

        // Flush streams before restoring
        std::cout.flush();
        std::cerr.flush();

        // Restore original buffers only if redirection actually happened
        if (cout_tee_buf_ && original_cout_buf_) { // Check if unique_ptr holds a buffer
            std::cout.rdbuf(original_cout_buf_);
        }
        if (cerr_tee_buf_ && original_cerr_buf_) { // Check if unique_ptr holds a buffer
            std::cerr.rdbuf(original_cerr_buf_);
        }

        // log_file_ is closed automatically by its destructor
        // unique_ptrs clean up TeeBuf instances automatically
    }

    // Disable copy/move semantics
    LogRedirector(const LogRedirector&) = delete;
    LogRedirector& operator=(const LogRedirector&) = delete;
    LogRedirector(LogRedirector&&) = delete;
    LogRedirector& operator=(LogRedirector&&) = delete;

private:
    std::ofstream log_file_;
    std::streambuf* original_cout_buf_;
    std::streambuf* original_cerr_buf_;
    std::unique_ptr<TeeBuf> cout_tee_buf_;
    std::unique_ptr<TeeBuf> cerr_tee_buf_;

    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        #ifdef _MSC_VER // Use secure version on Windows if available
        struct tm buf;
        localtime_s(&buf, &now_c);
        ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S");
        #else
        // Use thread-safe version if available (POSIX standard)
        struct tm buf;
        localtime_r(&now_c, &buf); // Use localtime_r for thread safety
        ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S");
        #endif
        return ss.str();
    }
};

// --- End Logging Setup ---


// --- Configurable Parameters (with defaults) ---
std::string TARGET_BEACON_ID = "BEACON-TX-ID:00005555";
float targetDistance = 8.0f;        // Target distance from the wall (meters)
float targetAzimuth = 0.0f;         // *Initial* target azimuth relative to the beacon (degrees) - Used in default wall follow & combined modes
// Forward Control (X-velocity)
float Kp_forward = 0.5;             // Proportional gain for forward movement
float max_forward_speed = 0.8;      // Max speed towards/away from the wall
float forward_dead_zone = 0.2;      // Dead zone for forward movement (meters)
// Lateral Control (Y-velocity) - Used in default wall follow & combined modes
float Kp_lateral = 0.02;            // Proportional gain for lateral movement
float max_lateral_speed = 0.5;      // Max speed sideways
float azimuth_dead_zone = 1.5;      // Dead zone for *initial* lateral movement (degrees)
// Yaw Control (Yaw Rate) - Used for Yaw Lock & combined modes
float Kp_yaw = 0.03;                // Proportional gain for yaw rate control
float max_yaw_rate = 15.0;          // Max yaw rate (degrees/second)
float yaw_dead_zone = 1.0;          // Dead zone for yaw control (degrees)
// Vertical Cycling Parameters - NEW
float vertical_cycle_target_agl = 2.0; // Target height AGL for descent phase (meters)
int vertical_cycle_count = 3;          // Number of down-up cycles
float vertical_cycle_agl_tolerance = 0.15; // Allowed error for reaching target AGL (meters)
// Reused parameters for vertical cycling:
// - ascent_speed
// - descent_speed
// - target_beacon_range (target range when ascending back up)
// - beacon_range_tolerance (tolerance for being level with beacon)
float ascent_speed = 0.3;           // Speed for ascending (m/s, positive value) - Reused
float descent_speed = 0.3;          // Speed for descending (m/s, positive value) - Reused
float target_beacon_range = 3.0;    // Target range from beacon when ascending (meters) - Reused
float beacon_range_tolerance = 0.2; // Allowed error for beacon range checks (meters) - Reused
// --- End Configurable Parameters ---


// Persistent variables for tracking the current second and wall/beacon candidate data
std::string currentSecond = "";
// General Wall/Beacon Data (Used by multiple modes)
float lowestRange = std::numeric_limits<float>::max(); // Closest wall candidate range (any sensor) - Used by 'w'
bool hasAnonData = false;                              // Flag if any anon data seen this second - Used by 'w'
float targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN(); // Sensed beacon azimuth - Used by 'w', 'b'
float current_beacon_range = std::numeric_limits<float>::quiet_NaN(); // Sensed beacon range - Used by 'w', 'b', and vertical cycle modes
bool foundTargetBeacon = false;                        // Flag if target beacon seen this second - Used by 'w', 'b', and vertical cycle modes
// Specific Data for Yaw Lock & Combined Modes
float closestHorizAnonRange = std::numeric_limits<float>::max(); // Range of closest HORIZONTAL anon detection - Used by 'y', 'b'
float closestHorizAnonAzimuth = std::numeric_limits<float>::quiet_NaN(); // Azimuth of closest HORIZONTAL anon detection - Used by 'y', 'b'
bool hasHorizAnonData = false;                         // Flag if horizontal anon data seen this second - Used by 'y', 'b'


// Global variable for default Python bridge script (Hardcoded)
std::string defaultPythonBridgeScript = "python_bridge.py";

// Flag to control the processing loop
std::atomic<bool> stopProcessingFlag(false);
std::thread processingThread;

// Monitoring Thread Globals
std::atomic<bool> stopMonitoringFlag(false);
std::thread monitoringThread;
const int TELEMETRY_TIMEOUT_SECONDS = 5;

// Function to load preferences
void loadPreferences() {
    std::cout << "Loading preferences..." << std::endl; // Logged
    std::ifstream preferencesFile("preferences.txt");
    if (preferencesFile.is_open()) {
        std::string line;
        while (std::getline(preferencesFile, line)) {
            // Trim leading/trailing whitespace
            line.erase(0, line.find_first_not_of(" \t\n\r\f\v"));
            line.erase(line.find_last_not_of(" \t\n\r\f\v") + 1);

            // Skip empty lines or comments
            if (line.empty() || line[0] == '#') continue;

            size_t equalsPos = line.find('=');
            if (equalsPos == std::string::npos) {
                std::cerr << "Warning: Skipping invalid line in preferences file: " << line << std::endl; // Logged
                continue;
            }

            std::string key = line.substr(0, equalsPos);
            std::string value = line.substr(equalsPos + 1);

            try {
                if (key == "target_beacon_id") {
                    TARGET_BEACON_ID = value;
                    std::cout << "  TARGET_BEACON_ID set to: " << TARGET_BEACON_ID << " (from preferences file)" << std::endl; // Logged
                } else if (key == "targetdistance") {
                    targetDistance = std::stof(value);
                    std::cout << "  targetDistance set to: " << targetDistance << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "target_azimuth") {
                    targetAzimuth = std::stof(value);
                    std::cout << "  targetAzimuth (Initial) set to: " << targetAzimuth << " degrees (from preferences file)" << std::endl; // Logged
                } else if (key == "kp_forward") {
                    Kp_forward = std::stof(value);
                    std::cout << "  Kp_forward set to: " << Kp_forward << " (from preferences file)" << std::endl; // Logged
                } else if (key == "max_forward_speed") {
                    max_forward_speed = std::stof(value);
                    std::cout << "  max_forward_speed set to: " << max_forward_speed << " m/s (from preferences file)" << std::endl; // Logged
                } else if (key == "forward_dead_zone") {
                    forward_dead_zone = std::stof(value);
                    std::cout << "  forward_dead_zone set to: " << forward_dead_zone << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "kp_lateral") {
                    Kp_lateral = std::stof(value);
                    std::cout << "  Kp_lateral set to: " << Kp_lateral << " (from preferences file)" << std::endl; // Logged
                } else if (key == "max_lateral_speed") {
                    max_lateral_speed = std::stof(value);
                    std::cout << "  max_lateral_speed set to: " << max_lateral_speed << " m/s (from preferences file)" << std::endl; // Logged
                } else if (key == "azimuth_dead_zone") {
                    azimuth_dead_zone = std::stof(value);
                    std::cout << "  azimuth_dead_zone set to: " << azimuth_dead_zone << " degrees (from preferences file)" << std::endl; // Logged
                } else if (key == "kp_yaw") { // Yaw Lock Param
                    Kp_yaw = std::stof(value);
                    std::cout << "  Kp_yaw set to: " << Kp_yaw << " (from preferences file)" << std::endl; // Logged
                } else if (key == "max_yaw_rate") { // Yaw Lock Param
                    max_yaw_rate = std::stof(value);
                    std::cout << "  max_yaw_rate set to: " << max_yaw_rate << " deg/s (from preferences file)" << std::endl; // Logged
                } else if (key == "yaw_dead_zone") { // Yaw Lock Param
                    yaw_dead_zone = std::stof(value);
                    std::cout << "  yaw_dead_zone set to: " << yaw_dead_zone << " degrees (from preferences file)" << std::endl; // Logged
                } else if (key == "vertical_cycle_target_agl") { // NEW
                    vertical_cycle_target_agl = std::stof(value);
                    std::cout << "  vertical_cycle_target_agl set to: " << vertical_cycle_target_agl << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "vertical_cycle_count") { // NEW
                    vertical_cycle_count = std::stoi(value);
                    std::cout << "  vertical_cycle_count set to: " << vertical_cycle_count << " (from preferences file)" << std::endl; // Logged
                } else if (key == "vertical_cycle_agl_tolerance") { // NEW
                    vertical_cycle_agl_tolerance = std::stof(value);
                    std::cout << "  vertical_cycle_agl_tolerance set to: " << vertical_cycle_agl_tolerance << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "ascent_speed") { // Reused Param
                    ascent_speed = std::stof(value);
                    std::cout << "  ascent_speed set to: " << ascent_speed << " m/s (from preferences file)" << std::endl; // Logged
                } else if (key == "descent_speed") { // Reused Param
                    descent_speed = std::stof(value);
                    std::cout << "  descent_speed set to: " << descent_speed << " m/s (from preferences file)" << std::endl; // Logged
                } else if (key == "target_beacon_range") { // Reused Param
                    target_beacon_range = std::stof(value);
                    std::cout << "  target_beacon_range (Base) set to: " << target_beacon_range << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "beacon_range_tolerance") { // Reused Param
                    beacon_range_tolerance = std::stof(value);
                    std::cout << "  beacon_range_tolerance set to: " << beacon_range_tolerance << " meters (from preferences file)" << std::endl; // Logged
                }


            } catch (const std::invalid_argument& ia) {
                std::cerr << "Warning: Invalid number format for key '" << key << "' in preferences file: " << value << std::endl; // Logged
            } catch (const std::out_of_range& oor) {
                std::cerr << "Warning: Value out of range for key '" << key << "' in preferences file: " << value << std::endl; // Logged
            } catch (...) {
                 std::cerr << "Warning: Unknown error parsing line for key '" << key << "' in preferences file: " << value << std::endl; // Logged
            }
        }
        preferencesFile.close();
        std::cout << "Finished loading preferences." << std::endl; // Logged
    } else {
        std::cout << "Preferences file ('preferences.txt') not found. Using default values:" << std::endl; // Logged
        std::cout << "  Default TARGET_BEACON_ID: " << TARGET_BEACON_ID << std::endl; // Logged
        std::cout << "  Default targetDistance: " << targetDistance << " meters" << std::endl; // Logged
        std::cout << "  Default targetAzimuth (Initial): " << targetAzimuth << " degrees" << std::endl; // Logged
        std::cout << "  Default Kp_forward: " << Kp_forward << std::endl; // Logged
        std::cout << "  Default max_forward_speed: " << max_forward_speed << " m/s" << std::endl; // Logged
        std::cout << "  Default forward_dead_zone: " << forward_dead_zone << " meters" << std::endl; // Logged
        std::cout << "  Default Kp_lateral: " << Kp_lateral << std::endl; // Logged
        std::cout << "  Default max_lateral_speed: " << max_lateral_speed << " m/s" << std::endl; // Logged
        std::cout << "  Default azimuth_dead_zone: " << azimuth_dead_zone << " degrees" << std::endl; // Logged
        std::cout << "  Default Kp_yaw: " << Kp_yaw << std::endl; // Logged NEW
        std::cout << "  Default max_yaw_rate: " << max_yaw_rate << " deg/s" << std::endl; // Logged NEW
        std::cout << "  Default yaw_dead_zone: " << yaw_dead_zone << " degrees" << std::endl; // Logged NEW
        std::cout << "  Default vertical_cycle_target_agl: " << vertical_cycle_target_agl << " meters" << std::endl; // Logged NEW
        std::cout << "  Default vertical_cycle_count: " << vertical_cycle_count << std::endl; // Logged NEW
        std::cout << "  Default vertical_cycle_agl_tolerance: " << vertical_cycle_agl_tolerance << " meters" << std::endl; // Logged NEW
        std::cout << "  Default ascent_speed: " << ascent_speed << " m/s" << std::endl; // Logged Reused
        std::cout << "  Default descent_speed: " << descent_speed << " m/s" << std::endl; // Logged Reused
        std::cout << "  Default target_beacon_range (Base): " << target_beacon_range << " meters" << std::endl; // Logged Reused
        std::cout << "  Default beacon_range_tolerance: " << beacon_range_tolerance << " meters" << std::endl; // Logged Reused
    }
}

struct RadarObject {
    std::string timestamp;
    std::string sensor;
    std::string src;
    float X, Y, Z;
    float Xdir, Ydir, Zdir;
    float Range, RangeRate, Pwr, Az, El;
    std::string ID;
    float Xsize, Ysize, Zsize;
    float Conf;
};

// Display full radar object details
void displayRadarObjects(const std::vector<RadarObject>& objects) {
    for (const auto& obj : objects) {
        std::cout << "Radar Object:\n" // Logged
                  << "  Timestamp: " << obj.timestamp << "\n" // Logged
                  << "  Sensor: " << obj.sensor << "\n" // Logged
                  << "  Source: " << obj.src << "\n" // Logged
                  << "  ID: " << obj.ID << "\n" // Logged
                  << "  X: " << obj.X << " Y: " << obj.Y << " Z: " << obj.Z << "\n" // Logged
                  << "  Xdir: " << obj.Xdir << " Ydir: " << obj.Ydir << " Zdir: " << obj.Zdir << "\n" // Logged
                  << "  Range: " << obj.Range << " Range Rate: " << obj.RangeRate << "\n" // Logged
                  << "  Power: " << obj.Pwr << " Azimuth: " << obj.Az << " Elevation: " << obj.El << "\n" // Logged
                  << "  Xsize: " << obj.Xsize << " Ysize: " << obj.Ysize << " Zsize: " << obj.Zsize << "\n" // Logged
                  << "  Confidence: " << obj.Conf << "\n" // Logged
                  << "----------------------------------------" << std::endl; // Logged
    }
}

// Display minimal radar object details
void displayRadarObjectsMinimal(const std::vector<RadarObject>& objects) {
    for (const auto& obj : objects) {
        std::cout << "Radar Object (Minimal):\n" // Logged
                  << "  Timestamp: " << obj.timestamp << "\n" // Logged
                  << "  Sensor: " << obj.sensor << "\n" // Logged
                  << "  ID: " << obj.ID << "\n" // Logged
                  << "  Range: " << obj.Range << "\n" // Logged
                  << "  Azimuth: " << obj.Az << "\n" // Logged
                  << "  Elevation: " << obj.El << "\n" // Logged
                  << "----------------------------------------" << std::endl; // Logged
    }
}

// --- ORIGINAL FUNCTION [w] ---
void extractBeaconAndWallData(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return;
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
        if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;
        if (!currentSecond.empty() && objSecond < currentSecond) {
            continue;
        }
        if (objSecond != currentSecond) {
            if (!currentSecond.empty() && (hasAnonData || foundTargetBeacon)) {
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr) {
                    float velocity_x = 0.0f;
                    if (hasAnonData) { // Use overall closest anon range for 'w' mode
                        float difference = lowestRange - targetDistance;
                        if (std::abs(difference) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * difference, max_forward_speed));
                        }
                    }
                    float velocity_y = 0.0f;
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) { // Use beacon azimuth for 'w' mode lateral control
                        float azimuth_error = targetBeaconAzimuth - targetAzimuth;
                        if (std::abs(azimuth_error) > azimuth_dead_zone) {
                            velocity_y = std::max(-max_lateral_speed, std::min(Kp_lateral * azimuth_error, max_lateral_speed));
                        }
                    }
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, 0, 0); // Z vel and Yaw rate are 0
                    std::cout << "[Default Follow] Control Status: \n" // Logged
                              << "TargetWall=" << targetDistance << " | CurrentWall(AnyAnon)=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n" // Logged
                              << "TargetBeaconAz=" << targetAzimuth << " | CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n" // Logged
                              << "Computed Velocity(X=" << velocity_x << ", Y=" << velocity_y << ")" << std::endl; // Logged
                    std::cout << "--------------------------------------" << std::endl; // Logged
                    vehicle->control->flightCtrl(ctrlData);
                } else if (hasAnonData || foundTargetBeacon) {
                     std::cout << "[Default Follow] (Flight Control Disabled or Not Available)\n"
                               << "TargetWall=" << targetDistance << " | CurrentWall(AnyAnon)=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n" // Logged
                               << "TargetBeaconAz=" << targetAzimuth << " | CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") // Logged
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl; // Logged
                }
            }
            // Reset ALL state variables for the new second
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN();
            foundTargetBeacon = false;
            closestHorizAnonRange = std::numeric_limits<float>::max();
            closestHorizAnonAzimuth = std::numeric_limits<float>::quiet_NaN();
            hasHorizAnonData = false;
        }

        // Accumulate general data for the current second
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) {
                 targetBeaconAzimuth = obj.Az;
                 current_beacon_range = obj.Range;
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) lowestRange = obj.Range; // Track overall closest anon

            // Accumulate yaw-specific data even if not used by this function's control
             if (obj.sensor == "R_Az" || obj.sensor == "R_El_R_Az") {
                 hasHorizAnonData = true;
                 if (obj.Range < closestHorizAnonRange) {
                     closestHorizAnonRange = obj.Range;
                     closestHorizAnonAzimuth = obj.Az;
                 }
            }
        }
         if (stopProcessingFlag.load()) return;
    }
}
// --- END ORIGINAL FUNCTION ---

// --- FUNCTION FOR YAW LOCK TEST [y] ---
void extractBeaconAndWallData_YawLock(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {

    // Accumulate data within the current second
    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return; // Check stop flag during accumulation

        // Extract second from timestamp (same logic as other functions)
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
        if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;

        // If this is the first object of a new second, process the *previous* second's data
        // and reset accumulation variables.
        if (objSecond != currentSecond) {
            if (!currentSecond.empty() && hasHorizAnonData) { // Only process if we had valid horizontal data

                // --- Control Logic ---
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr) {
                    float velocity_x = 0.0f;
                    float yaw_rate = 0.0f;

                    // --- Forward Velocity Control (based on closest horizontal anon range) ---
                    float distance_error = closestHorizAnonRange - targetDistance;
                    if (std::abs(distance_error) > forward_dead_zone) {
                        velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * distance_error, max_forward_speed));
                    }

                    // --- Yaw Rate Control (based on closest horizontal anon azimuth) ---
                    // Azimuth is the error we want to correct to zero.
                    float yaw_azimuth_error = closestHorizAnonAzimuth;
                    if (std::abs(yaw_azimuth_error) > yaw_dead_zone) {
                        yaw_rate = Kp_yaw * yaw_azimuth_error;
                        // Clamp the yaw rate
                        yaw_rate = std::max(-max_yaw_rate, std::min(yaw_rate, max_yaw_rate));
                    }

                    // --- Set Control Flags and Data ---
                    // We control horizontal velocity and yaw rate. Vertical velocity and lateral velocity are zero.
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | // Control X velocity
                                          DJI::OSDK::Control::VERTICAL_VELOCITY |   // Keep Z velocity at 0
                                          DJI::OSDK::Control::YAW_RATE |          // Control Yaw Rate
                                          DJI::OSDK::Control::HORIZONTAL_BODY |   // Interpret X/Y in drone's body frame
                                          DJI::OSDK::Control::STABLE_ENABLE;      // Use DJI's stabilization

                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, 0.0f, 0.0f, yaw_rate); // Y and Z velocity are 0

                    // --- Logging ---
                    std::cout << "[Yaw Lock] Control Status: \n"
                              << "  TargetWallDist=" << targetDistance << " | CurrentWallDist(Horiz)=" << closestHorizAnonRange << "\n"
                              << "  TargetWallAz=0" << " | CurrentWallAz(Horiz)=" << closestHorizAnonAzimuth << "\n"
                              << "  Computed Vel(X)=" << velocity_x << " | Computed YawRate=" << yaw_rate << " deg/s"
                              << std::endl;
                    std::cout << "--------------------------------------" << std::endl;

                    // --- Send Command ---
                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasHorizAnonData) { // Control disabled but we have data
                     std::cout << "[Yaw Lock] (Flight Control Disabled or Not Available)\n"
                               << "  TargetWallDist=" << targetDistance << " | CurrentWallDist(Horiz)=" << closestHorizAnonRange << "\n"
                               << "  TargetWallAz=0" << " | CurrentWallAz(Horiz)=" << closestHorizAnonAzimuth
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl;
                }
            } // End processing previous second

            // --- Reset ALL state variables for the new second ---
            currentSecond = objSecond;
            closestHorizAnonRange = std::numeric_limits<float>::max();
            closestHorizAnonAzimuth = std::numeric_limits<float>::quiet_NaN();
            hasHorizAnonData = false;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN();
            foundTargetBeacon = false;
        } // End if (new second)

        // --- Data Accumulation for the *current* second ---
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) {
                 targetBeaconAzimuth = obj.Az;
                 current_beacon_range = obj.Range;
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) lowestRange = obj.Range;

            if (obj.sensor == "R_Az" || obj.sensor == "R_El_R_Az") {
                 hasHorizAnonData = true;
                 if (obj.Range < closestHorizAnonRange) {
                     closestHorizAnonRange = obj.Range;
                     closestHorizAnonAzimuth = obj.Az;
                 }
            }
        }

        if (stopProcessingFlag.load()) return;
    } // End loop through objects
}
// --- END FUNCTION FOR YAW LOCK TEST ---

// --- FUNCTION FOR COMBINED MODE [b] ---
void extractBeaconAndWallData_Combined(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {

    // Accumulate data within the current second
    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return; // Check stop flag during accumulation

        // Extract second from timestamp
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
        if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;

        // If this is the first object of a new second, process the *previous* second's data
        // and reset accumulation variables.
        if (objSecond != currentSecond) {
            // Process previous second only if we have the necessary data (horizontal anon for yaw/dist, beacon for lateral)
            if (!currentSecond.empty() && (hasHorizAnonData || foundTargetBeacon)) {

                // --- Control Logic ---
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr) {
                    float velocity_x = 0.0f;
                    float velocity_y = 0.0f;
                    float yaw_rate = 0.0f;

                    // --- Forward Velocity Control (based on closest *horizontal* anon range) ---
                    if (hasHorizAnonData) {
                        float distance_error = closestHorizAnonRange - targetDistance;
                        if (std::abs(distance_error) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * distance_error, max_forward_speed));
                        }
                    } // else velocity_x remains 0 if no horizontal anon data

                    // --- Lateral Velocity Control (based on *target beacon* azimuth) ---
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) {
                        float lateral_azimuth_error = targetBeaconAzimuth - targetAzimuth; // Compare beacon az to initial target az
                        if (std::abs(lateral_azimuth_error) > azimuth_dead_zone) {
                            velocity_y = std::max(-max_lateral_speed, std::min(Kp_lateral * lateral_azimuth_error, max_lateral_speed));
                        }
                    } // else velocity_y remains 0 if no beacon found

                    // --- Yaw Rate Control (based on closest *horizontal* anon azimuth) ---
                    if (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth)) {
                        float yaw_azimuth_error = closestHorizAnonAzimuth; // Target is 0 azimuth relative to wall
                        if (std::abs(yaw_azimuth_error) > yaw_dead_zone) {
                            yaw_rate = Kp_yaw * yaw_azimuth_error;
                            // Clamp the yaw rate
                            yaw_rate = std::max(-max_yaw_rate, std::min(yaw_rate, max_yaw_rate));
                        }
                    } // else yaw_rate remains 0 if no horizontal anon data

                    // --- Set Control Flags and Data ---
                    // Control X vel, Y vel, and Yaw Rate. Z vel is 0.
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | // Control X and Y velocity
                                          DJI::OSDK::Control::VERTICAL_VELOCITY |   // Keep Z velocity at 0
                                          DJI::OSDK::Control::YAW_RATE |          // Control Yaw Rate
                                          DJI::OSDK::Control::HORIZONTAL_BODY |   // Interpret X/Y in drone's body frame
                                          DJI::OSDK::Control::STABLE_ENABLE;      // Use DJI's stabilization

                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, 0.0f, yaw_rate); // Z velocity is 0

                    // --- Logging ---
                    std::cout << "[Combined Mode] Control Status: \n"
                              << "  TargetWallDist=" << targetDistance << " | CurrentWallDist(Horiz)=" << (hasHorizAnonData ? std::to_string(closestHorizAnonRange) : "N/A") << "\n"
                              << "  TargetBeaconAz=" << targetAzimuth << " | CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "  TargetWallAz=0" << " | CurrentWallAz(Horiz)=" << (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) ? std::to_string(closestHorizAnonAzimuth) : "N/A") << "\n"
                              << "  Computed Vel(X=" << velocity_x << ", Y=" << velocity_y << ") | Computed YawRate=" << yaw_rate << " deg/s"
                              << std::endl;
                    std::cout << "--------------------------------------" << std::endl;

                    // --- Send Command ---
                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasHorizAnonData || foundTargetBeacon) { // Control disabled but we have data
                     std::cout << "[Combined Mode] (Flight Control Disabled or Not Available)\n"
                               << "  TargetWallDist=" << targetDistance << " | CurrentWallDist(Horiz)=" << (hasHorizAnonData ? std::to_string(closestHorizAnonRange) : "N/A") << "\n"
                               << "  TargetBeaconAz=" << targetAzimuth << " | CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                               << "  TargetWallAz=0" << " | CurrentWallAz(Horiz)=" << (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) ? std::to_string(closestHorizAnonAzimuth) : "N/A")
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl;
                }
            } // End processing previous second

            // --- Reset ALL state variables for the new second ---
            currentSecond = objSecond;
            closestHorizAnonRange = std::numeric_limits<float>::max();
            closestHorizAnonAzimuth = std::numeric_limits<float>::quiet_NaN();
            hasHorizAnonData = false;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN();
            foundTargetBeacon = false;
        } // End if (new second)

        // --- Data Accumulation for the *current* second (Required for ALL controls) ---
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) { // Store first beacon detection
                 targetBeaconAzimuth = obj.Az;
                 current_beacon_range = obj.Range;
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true; // Mark general anon data
            if (obj.Range < lowestRange) lowestRange = obj.Range; // Track overall closest (for 'w' mode consistency if needed elsewhere)

            // Accumulate closest *horizontal* anon data specifically
            if (obj.sensor == "R_Az" || obj.sensor == "R_El_R_Az") {
                 hasHorizAnonData = true;
                 if (obj.Range < closestHorizAnonRange) {
                     closestHorizAnonRange = obj.Range;
                     closestHorizAnonAzimuth = obj.Az;
                 }
            }
        }

        if (stopProcessingFlag.load()) return; // Check stop flag again after processing object
    } // End loop through objects
}
// --- END FUNCTION FOR COMBINED MODE ---


// --- NEW VERTICAL CYCLE FUNCTIONS [1, 2, 3] ---

// State for Vertical Cycling modes
enum class VerticalCyclePhase { ALIGNING_INITIAL, DESCENDING, ASCENDING, COMPLETED_ALL };

// Helper function to get phase name string
std::string getVerticalCyclePhaseName(VerticalCyclePhase phase) {
    switch(phase) {
        case VerticalCyclePhase::ALIGNING_INITIAL: return "ALIGNING_INITIAL";
        case VerticalCyclePhase::DESCENDING: return "DESCENDING";
        case VerticalCyclePhase::ASCENDING: return "ASCENDING";
        case VerticalCyclePhase::COMPLETED_ALL: return "COMPLETED_ALL";
        default: return "UNKNOWN";
    }
}

// Base logic for Vertical Cycle modes [1] (Mirrors 'w' horizontally)
void extractBeaconAndWallData_VerticalCycle(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    static VerticalCyclePhase current_phase = VerticalCyclePhase::ALIGNING_INITIAL;
    static int current_cycle = 0;
    static std::thread::id current_thread_id;

    // Reset state if thread restarts
    if (current_thread_id != std::this_thread::get_id()) {
        current_phase = VerticalCyclePhase::ALIGNING_INITIAL;
        current_cycle = 0;
        current_thread_id = std::this_thread::get_id();
        std::cout << "[Vert Cycle W] State reset for new thread run." << std::endl; // Logged
    }

    // Accumulate data within the current second
    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return; // Check stop flag during accumulation

        // Extract second from timestamp
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
        if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;

        // If this is the first object of a new second, process the *previous* second's data
        if (objSecond != currentSecond) {
            // Process previous second only if we have relevant data (anon or beacon)
            if (!currentSecond.empty() && (hasAnonData || foundTargetBeacon)) {

                // --- Control Logic ---
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr && vehicle->subscribe != nullptr) {
                    float velocity_x = 0.0f;
                    float velocity_y = 0.0f;
                    float velocity_z = 0.0f;
                    float yaw_rate = 0.0f; // Yaw not controlled in this mode
                    float current_height_agl = -1.0f; // Initialize AGL height

                    // --- Horizontal Control (Mirrors 'w') ---
                    // Forward Velocity based on *any* closest anon detection
                    if (hasAnonData) {
                        float distance_error = lowestRange - targetDistance;
                        if (std::abs(distance_error) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * distance_error, max_forward_speed));
                        }
                    }
                    // Lateral Velocity based on *beacon* azimuth
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) {
                        float lateral_azimuth_error = targetBeaconAzimuth - targetAzimuth;
                        if (std::abs(lateral_azimuth_error) > azimuth_dead_zone) {
                            velocity_y = std::max(-max_lateral_speed, std::min(Kp_lateral * lateral_azimuth_error, max_lateral_speed));
                        }
                    } else if (current_phase != VerticalCyclePhase::ALIGNING_INITIAL && current_phase != VerticalCyclePhase::COMPLETED_ALL) {
                         std::cout << "***BEACON NOT FOUND***" << std::endl;
                         velocity_y = 0.0f; // Stop lateral movement if beacon lost during active cycle
                    }

                    // --- Vertical Cycle State Machine ---
                    bool horizontal_dist_ok = hasAnonData && (std::abs(lowestRange - targetDistance) <= position_distance_tolerance);
                    bool lateral_beacon_az_ok = foundTargetBeacon && !std::isnan(targetBeaconAzimuth) && (std::abs(targetBeaconAzimuth - targetAzimuth) <= position_azimuth_tolerance);
                    bool beacon_range_ok = foundTargetBeacon && !std::isnan(current_beacon_range) && (std::abs(current_beacon_range - target_beacon_range) <= beacon_range_tolerance);

                    switch (current_phase) {
                        case VerticalCyclePhase::ALIGNING_INITIAL:
                            velocity_z = 0.0f;
                            if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl; }
                            // Check if horizontal distance, lateral beacon azimuth, AND beacon range are met
                            if (horizontal_dist_ok && lateral_beacon_az_ok && beacon_range_ok) {
                                std::cout << "[Vert Cycle W] Initial position confirmed. Starting Cycle " << current_cycle + 1 << "." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::DESCENDING;
                            }
                            // Horizontal control continues trying to align
                            break;

                        case VerticalCyclePhase::DESCENDING:
                            current_height_agl = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                            if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl; } // Log beacon loss but continue descent based on height
                            // Check AGL height
                            if (current_height_agl > vertical_cycle_target_agl + vertical_cycle_agl_tolerance) {
                                velocity_z = -descent_speed;
                            } else { // Reached target AGL
                                velocity_z = 0.0f;
                                std::cout << "[Vert Cycle W] Target AGL reached (" << current_height_agl << "m). Starting ascent." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::ASCENDING;
                            }
                            // Horizontal control continues
                            break;

                        case VerticalCyclePhase::ASCENDING:
                            if (foundTargetBeacon && !std::isnan(current_beacon_range)) {
                                // Check beacon range
                                if (current_beacon_range > target_beacon_range + beacon_range_tolerance) {
                                    velocity_z = ascent_speed;
                                } else { // Reached target beacon range
                                    velocity_z = 0.0f;
                                    current_cycle++;
                                    std::cout << "[Vert Cycle W] Target beacon range reached (" << current_beacon_range << "m). Cycle " << current_cycle << " completed." << std::endl; // Logged
                                    if (current_cycle >= vertical_cycle_count) {
                                        current_phase = VerticalCyclePhase::COMPLETED_ALL;
                                        std::cout << "[Vert Cycle W] All cycles finished." << std::endl; // Logged
                                    } else {
                                        // Ready for next descent, just need to wait for next second's data
                                        current_phase = VerticalCyclePhase::DESCENDING;
                                        std::cout << "[Vert Cycle W] Starting Cycle " << current_cycle + 1 << " descent." << std::endl; // Logged
                                    }
                                }
                            } else { // Beacon lost during ascent
                                 std::cout << "***BEACON NOT FOUND***" << std::endl;
                                 velocity_z = 0.0f; // Stop ascending
                                 velocity_y = 0.0f; // Stop lateral movement as well
                                 std::cerr << "[Vert Cycle W] Warning: Beacon lost during ascent. Hovering." << std::endl; // Logged
                            }
                            // Horizontal control continues (except lateral if beacon lost)
                            break;

                        case VerticalCyclePhase::COMPLETED_ALL:
                            velocity_x = 0.0f;
                            velocity_y = 0.0f;
                            velocity_z = 0.0f;
                            yaw_rate = 0.0f;
                            // Remain in this state
                            break;
                    }

                    // --- Send Command ---
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, velocity_z, yaw_rate);

                    // --- Logging ---
                    std::cout << "[Vert Cycle W] Ctl Status: Cycle=" << current_cycle << "/" << vertical_cycle_count << " Phase=" << getVerticalCyclePhaseName(current_phase) << "\n"
                              << "  TgtWallDist=" << targetDistance << " | CurWallDist(Any)=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                              << "  TgtBeaconAz=" << targetAzimuth << " | CurBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "  TgtBeaconRng=" << target_beacon_range << " | CurBeaconRng=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << "\n"
                              << "  TgtAGL=" << vertical_cycle_target_agl << " | CurAGL=" << (current_height_agl >= 0.0f ? std::to_string(current_height_agl) : "N/A") << "\n"
                              << "  Computed Vel(X=" << velocity_x << ", Y=" << velocity_y << ", Z=" << velocity_z << ") | YawRate=" << yaw_rate << " deg/s"
                              << std::endl;
                    std::cout << "--------------------------------------" << std::endl;

                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasAnonData || foundTargetBeacon) { // Control disabled but we have data
                     if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl;} // Log beacon loss even if control disabled
                     std::cout << "[Vert Cycle W] (Flight Control Disabled or Not Available) Cycle=" << current_cycle << "/" << vertical_cycle_count << " Phase=" << getVerticalCyclePhaseName(current_phase) << "\n"
                               << "  TgtWallDist=" << targetDistance << " | CurWallDist(Any)=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                               << "  TgtBeaconAz=" << targetAzimuth << " | CurBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                               << "  TgtBeaconRng=" << target_beacon_range << " | CurBeaconRng=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << "\n"
                               << "  TgtAGL=" << vertical_cycle_target_agl
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl;
                }
            } // End processing previous second

            // --- Reset ALL state variables for the new second ---
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN();
            foundTargetBeacon = false;
            closestHorizAnonRange = std::numeric_limits<float>::max();
            closestHorizAnonAzimuth = std::numeric_limits<float>::quiet_NaN();
            hasHorizAnonData = false;
        } // End if (new second)

        // --- Data Accumulation for the *current* second ---
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) {
                 targetBeaconAzimuth = obj.Az;
                 current_beacon_range = obj.Range;
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) lowestRange = obj.Range;

            if (obj.sensor == "R_Az" || obj.sensor == "R_El_R_Az") {
                 hasHorizAnonData = true;
                 if (obj.Range < closestHorizAnonRange) {
                     closestHorizAnonRange = obj.Range;
                     closestHorizAnonAzimuth = obj.Az;
                 }
            }
        }
        if (stopProcessingFlag.load()) return;
    } // End loop through objects
}


// Base logic for Vertical Cycle modes [2] (Mirrors 'y' horizontally)
void extractBeaconAndWallData_YawLock_VerticalCycle(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    static VerticalCyclePhase current_phase = VerticalCyclePhase::ALIGNING_INITIAL;
    static int current_cycle = 0;
    static std::thread::id current_thread_id;

    // Reset state if thread restarts
    if (current_thread_id != std::this_thread::get_id()) {
        current_phase = VerticalCyclePhase::ALIGNING_INITIAL;
        current_cycle = 0;
        current_thread_id = std::this_thread::get_id();
        std::cout << "[Vert Cycle Y] State reset for new thread run." << std::endl; // Logged
    }

    // Accumulate data within the current second
    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return; // Check stop flag during accumulation

        // Extract second from timestamp
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
        if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;

        // If this is the first object of a new second, process the *previous* second's data
        if (objSecond != currentSecond) {
            // Process previous second only if we have relevant data (horizontal anon or beacon)
            if (!currentSecond.empty() && (hasHorizAnonData || foundTargetBeacon)) {

                // --- Control Logic ---
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr && vehicle->subscribe != nullptr) {
                    float velocity_x = 0.0f;
                    float velocity_y = 0.0f; // Lateral not controlled in this mode
                    float velocity_z = 0.0f;
                    float yaw_rate = 0.0f;
                    float current_height_agl = -1.0f; // Initialize AGL height

                    // --- Horizontal/Yaw Control (Mirrors 'y') ---
                    // Forward Velocity based on *horizontal* anon range
                    if (hasHorizAnonData) {
                        float distance_error = closestHorizAnonRange - targetDistance;
                        if (std::abs(distance_error) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * distance_error, max_forward_speed));
                        }
                    }
                    // Yaw Rate based on *horizontal* anon azimuth
                    if (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth)) {
                        float yaw_azimuth_error = closestHorizAnonAzimuth; // Target is 0 azimuth relative to wall
                        if (std::abs(yaw_azimuth_error) > yaw_dead_zone) {
                            yaw_rate = Kp_yaw * yaw_azimuth_error;
                            yaw_rate = std::max(-max_yaw_rate, std::min(yaw_rate, max_yaw_rate));
                        }
                    }

                    // --- Vertical Cycle State Machine ---
                    bool horizontal_dist_ok = hasHorizAnonData && (std::abs(closestHorizAnonRange - targetDistance) <= position_distance_tolerance);
                    bool yaw_ok = hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) && (std::abs(closestHorizAnonAzimuth) <= position_azimuth_tolerance); // Using position_azimuth_tolerance for yaw alignment check
                    bool beacon_range_ok = foundTargetBeacon && !std::isnan(current_beacon_range) && (std::abs(current_beacon_range - target_beacon_range) <= beacon_range_tolerance);

                    switch (current_phase) {
                        case VerticalCyclePhase::ALIGNING_INITIAL:
                            velocity_z = 0.0f;
                            if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl; }
                            // Check if horizontal distance, yaw, AND beacon range are met
                            if (horizontal_dist_ok && yaw_ok && beacon_range_ok) {
                                std::cout << "[Vert Cycle Y] Initial position confirmed. Starting Cycle " << current_cycle + 1 << "." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::DESCENDING;
                            }
                            // Horizontal/Yaw control continues trying to align
                            break;

                        case VerticalCyclePhase::DESCENDING:
                            current_height_agl = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                            if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl; } // Log beacon loss but continue descent
                            // Check AGL height
                            if (current_height_agl > vertical_cycle_target_agl + vertical_cycle_agl_tolerance) {
                                velocity_z = -descent_speed;
                            } else { // Reached target AGL
                                velocity_z = 0.0f;
                                std::cout << "[Vert Cycle Y] Target AGL reached (" << current_height_agl << "m). Starting ascent." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::ASCENDING;
                            }
                            // Horizontal/Yaw control continues
                            break;

                        case VerticalCyclePhase::ASCENDING:
                            if (foundTargetBeacon && !std::isnan(current_beacon_range)) {
                                // Check beacon range
                                if (current_beacon_range > target_beacon_range + beacon_range_tolerance) {
                                    velocity_z = ascent_speed;
                                } else { // Reached target beacon range
                                    velocity_z = 0.0f;
                                    current_cycle++;
                                    std::cout << "[Vert Cycle Y] Target beacon range reached (" << current_beacon_range << "m). Cycle " << current_cycle << " completed." << std::endl; // Logged
                                    if (current_cycle >= vertical_cycle_count) {
                                        current_phase = VerticalCyclePhase::COMPLETED_ALL;
                                        std::cout << "[Vert Cycle Y] All cycles finished." << std::endl; // Logged
                                    } else {
                                        current_phase = VerticalCyclePhase::DESCENDING;
                                        std::cout << "[Vert Cycle Y] Starting Cycle " << current_cycle + 1 << " descent." << std::endl; // Logged
                                    }
                                }
                            } else { // Beacon lost during ascent
                                 std::cout << "***BEACON NOT FOUND***" << std::endl;
                                 velocity_z = 0.0f; // Stop ascending
                                 std::cerr << "[Vert Cycle Y] Warning: Beacon lost during ascent. Hovering vertically." << std::endl; // Logged
                            }
                            // Horizontal/Yaw control continues
                            break;

                        case VerticalCyclePhase::COMPLETED_ALL:
                            velocity_x = 0.0f;
                            velocity_y = 0.0f;
                            velocity_z = 0.0f;
                            yaw_rate = 0.0f;
                            // Remain in this state
                            break;
                    }

                    // --- Send Command ---
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, velocity_z, yaw_rate);

                    // --- Logging ---
                    std::cout << "[Vert Cycle Y] Ctl Status: Cycle=" << current_cycle << "/" << vertical_cycle_count << " Phase=" << getVerticalCyclePhaseName(current_phase) << "\n"
                              << "  TgtWallDist=" << targetDistance << " | CurWallDist(Horiz)=" << (hasHorizAnonData ? std::to_string(closestHorizAnonRange) : "N/A") << "\n"
                              << "  TgtWallAz=0" << " | CurWallAz(Horiz)=" << (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) ? std::to_string(closestHorizAnonAzimuth) : "N/A") << "\n"
                              << "  TgtBeaconRng=" << target_beacon_range << " | CurBeaconRng=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << "\n"
                              << "  TgtAGL=" << vertical_cycle_target_agl << " | CurAGL=" << (current_height_agl >= 0.0f ? std::to_string(current_height_agl) : "N/A") << "\n"
                              << "  Computed Vel(X=" << velocity_x << ", Y=" << velocity_y << ", Z=" << velocity_z << ") | YawRate=" << yaw_rate << " deg/s"
                              << std::endl;
                    std::cout << "--------------------------------------" << std::endl;

                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasHorizAnonData || foundTargetBeacon) { // Control disabled but we have data
                     if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl;} // Log beacon loss even if control disabled
                     std::cout << "[Vert Cycle Y] (Flight Control Disabled or Not Available) Cycle=" << current_cycle << "/" << vertical_cycle_count << " Phase=" << getVerticalCyclePhaseName(current_phase) << "\n"
                               << "  TgtWallDist=" << targetDistance << " | CurWallDist(Horiz)=" << (hasHorizAnonData ? std::to_string(closestHorizAnonRange) : "N/A") << "\n"
                               << "  TgtWallAz=0" << " | CurWallAz(Horiz)=" << (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) ? std::to_string(closestHorizAnonAzimuth) : "N/A") << "\n"
                               << "  TgtBeaconRng=" << target_beacon_range << " | CurBeaconRng=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << "\n"
                               << "  TgtAGL=" << vertical_cycle_target_agl
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl;
                }
            } // End processing previous second

            // --- Reset ALL state variables for the new second ---
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN();
            foundTargetBeacon = false;
            closestHorizAnonRange = std::numeric_limits<float>::max();
            closestHorizAnonAzimuth = std::numeric_limits<float>::quiet_NaN();
            hasHorizAnonData = false;
        } // End if (new second)

        // --- Data Accumulation for the *current* second ---
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) {
                 targetBeaconAzimuth = obj.Az;
                 current_beacon_range = obj.Range;
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) lowestRange = obj.Range;

            if (obj.sensor == "R_Az" || obj.sensor == "R_El_R_Az") {
                 hasHorizAnonData = true;
                 if (obj.Range < closestHorizAnonRange) {
                     closestHorizAnonRange = obj.Range;
                     closestHorizAnonAzimuth = obj.Az;
                 }
            }
        }
        if (stopProcessingFlag.load()) return;
    } // End loop through objects
}


// Base logic for Vertical Cycle modes [3] (Mirrors 'b' horizontally)
void extractBeaconAndWallData_Combined_VerticalCycle(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    static VerticalCyclePhase current_phase = VerticalCyclePhase::ALIGNING_INITIAL;
    static int current_cycle = 0;
    static std::thread::id current_thread_id;

    // Reset state if thread restarts
    if (current_thread_id != std::this_thread::get_id()) {
        current_phase = VerticalCyclePhase::ALIGNING_INITIAL;
        current_cycle = 0;
        current_thread_id = std::this_thread::get_id();
        std::cout << "[Vert Cycle B] State reset for new thread run." << std::endl; // Logged
    }

    // Accumulate data within the current second
    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return; // Check stop flag during accumulation

        // Extract second from timestamp
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
        if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;

        // If this is the first object of a new second, process the *previous* second's data
        if (objSecond != currentSecond) {
            // Process previous second only if we have relevant data (horizontal anon or beacon)
            if (!currentSecond.empty() && (hasHorizAnonData || foundTargetBeacon)) {

                // --- Control Logic ---
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr && vehicle->subscribe != nullptr) {
                    float velocity_x = 0.0f;
                    float velocity_y = 0.0f;
                    float velocity_z = 0.0f;
                    float yaw_rate = 0.0f;
                    float current_height_agl = -1.0f; // Initialize AGL height

                    // --- Horizontal/Yaw Control (Mirrors 'b') ---
                    // Forward Velocity based on *horizontal* anon range
                    if (hasHorizAnonData) {
                        float distance_error = closestHorizAnonRange - targetDistance;
                        if (std::abs(distance_error) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * distance_error, max_forward_speed));
                        }
                    }
                    // Lateral Velocity based on *beacon* azimuth
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) {
                        float lateral_azimuth_error = targetBeaconAzimuth - targetAzimuth;
                        if (std::abs(lateral_azimuth_error) > azimuth_dead_zone) {
                            velocity_y = std::max(-max_lateral_speed, std::min(Kp_lateral * lateral_azimuth_error, max_lateral_speed));
                        }
                    } else if (current_phase != VerticalCyclePhase::ALIGNING_INITIAL && current_phase != VerticalCyclePhase::COMPLETED_ALL) {
                         std::cout << "***BEACON NOT FOUND***" << std::endl;
                         velocity_y = 0.0f; // Stop lateral movement if beacon lost during active cycle
                    }
                    // Yaw Rate based on *horizontal* anon azimuth
                    if (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth)) {
                        float yaw_azimuth_error = closestHorizAnonAzimuth; // Target is 0 azimuth relative to wall
                        if (std::abs(yaw_azimuth_error) > yaw_dead_zone) {
                            yaw_rate = Kp_yaw * yaw_azimuth_error;
                            yaw_rate = std::max(-max_yaw_rate, std::min(yaw_rate, max_yaw_rate));
                        }
                    }

                    // --- Vertical Cycle State Machine ---
                    bool horizontal_dist_ok = hasHorizAnonData && (std::abs(closestHorizAnonRange - targetDistance) <= position_distance_tolerance);
                    bool lateral_beacon_az_ok = foundTargetBeacon && !std::isnan(targetBeaconAzimuth) && (std::abs(targetBeaconAzimuth - targetAzimuth) <= position_azimuth_tolerance);
                    bool yaw_ok = hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) && (std::abs(closestHorizAnonAzimuth) <= position_azimuth_tolerance); // Using position_azimuth_tolerance for yaw alignment check
                    bool beacon_range_ok = foundTargetBeacon && !std::isnan(current_beacon_range) && (std::abs(current_beacon_range - target_beacon_range) <= beacon_range_tolerance);

                    switch (current_phase) {
                        case VerticalCyclePhase::ALIGNING_INITIAL:
                            velocity_z = 0.0f;
                            if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl; }
                            // Check if horizontal distance, lateral beacon azimuth, yaw, AND beacon range are met
                            if (horizontal_dist_ok && lateral_beacon_az_ok && yaw_ok && beacon_range_ok) {
                                std::cout << "[Vert Cycle B] Initial position confirmed. Starting Cycle " << current_cycle + 1 << "." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::DESCENDING;
                            }
                            // Horizontal/Yaw control continues trying to align
                            break;

                        case VerticalCyclePhase::DESCENDING:
                            current_height_agl = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                            if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl; } // Log beacon loss but continue descent
                            // Check AGL height
                            if (current_height_agl > vertical_cycle_target_agl + vertical_cycle_agl_tolerance) {
                                velocity_z = -descent_speed;
                            } else { // Reached target AGL
                                velocity_z = 0.0f;
                                std::cout << "[Vert Cycle B] Target AGL reached (" << current_height_agl << "m). Starting ascent." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::ASCENDING;
                            }
                            // Horizontal/Yaw control continues (except lateral if beacon lost)
                            break;

                        case VerticalCyclePhase::ASCENDING:
                            if (foundTargetBeacon && !std::isnan(current_beacon_range)) {
                                // Check beacon range
                                if (current_beacon_range > target_beacon_range + beacon_range_tolerance) {
                                    velocity_z = ascent_speed;
                                } else { // Reached target beacon range
                                    velocity_z = 0.0f;
                                    current_cycle++;
                                    std::cout << "[Vert Cycle B] Target beacon range reached (" << current_beacon_range << "m). Cycle " << current_cycle << " completed." << std::endl; // Logged
                                    if (current_cycle >= vertical_cycle_count) {
                                        current_phase = VerticalCyclePhase::COMPLETED_ALL;
                                        std::cout << "[Vert Cycle B] All cycles finished." << std::endl; // Logged
                                    } else {
                                        current_phase = VerticalCyclePhase::DESCENDING;
                                        std::cout << "[Vert Cycle B] Starting Cycle " << current_cycle + 1 << " descent." << std::endl; // Logged
                                    }
                                }
                            } else { // Beacon lost during ascent
                                 std::cout << "***BEACON NOT FOUND***" << std::endl;
                                 velocity_z = 0.0f; // Stop ascending
                                 velocity_y = 0.0f; // Stop lateral movement as well
                                 std::cerr << "[Vert Cycle B] Warning: Beacon lost during ascent. Hovering." << std::endl; // Logged
                            }
                            // Horizontal/Yaw control continues (except lateral if beacon lost)
                            break;

                        case VerticalCyclePhase::COMPLETED_ALL:
                            velocity_x = 0.0f;
                            velocity_y = 0.0f;
                            velocity_z = 0.0f;
                            yaw_rate = 0.0f;
                            // Remain in this state
                            break;
                    }

                    // --- Send Command ---
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, velocity_z, yaw_rate);

                    // --- Logging ---
                    std::cout << "[Vert Cycle B] Ctl Status: Cycle=" << current_cycle << "/" << vertical_cycle_count << " Phase=" << getVerticalCyclePhaseName(current_phase) << "\n"
                              << "  TgtWallDist=" << targetDistance << " | CurWallDist(Horiz)=" << (hasHorizAnonData ? std::to_string(closestHorizAnonRange) : "N/A") << "\n"
                              << "  TgtBeaconAz=" << targetAzimuth << " | CurBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "  TgtWallAz=0" << " | CurWallAz(Horiz)=" << (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) ? std::to_string(closestHorizAnonAzimuth) : "N/A") << "\n"
                              << "  TgtBeaconRng=" << target_beacon_range << " | CurBeaconRng=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << "\n"
                              << "  TgtAGL=" << vertical_cycle_target_agl << " | CurAGL=" << (current_height_agl >= 0.0f ? std::to_string(current_height_agl) : "N/A") << "\n"
                              << "  Computed Vel(X=" << velocity_x << ", Y=" << velocity_y << ", Z=" << velocity_z << ") | YawRate=" << yaw_rate << " deg/s"
                              << std::endl;
                    std::cout << "--------------------------------------" << std::endl;

                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasHorizAnonData || foundTargetBeacon) { // Control disabled but we have data
                     if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl;} // Log beacon loss even if control disabled
                     std::cout << "[Vert Cycle B] (Flight Control Disabled or Not Available) Cycle=" << current_cycle << "/" << vertical_cycle_count << " Phase=" << getVerticalCyclePhaseName(current_phase) << "\n"
                               << "  TgtWallDist=" << targetDistance << " | CurWallDist(Horiz)=" << (hasHorizAnonData ? std::to_string(closestHorizAnonRange) : "N/A") << "\n"
                               << "  TgtBeaconAz=" << targetAzimuth << " | CurBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                               << "  TgtWallAz=0" << " | CurWallAz(Horiz)=" << (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) ? std::to_string(closestHorizAnonAzimuth) : "N/A") << "\n"
                               << "  TgtBeaconRng=" << target_beacon_range << " | CurBeaconRng=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << "\n"
                               << "  TgtAGL=" << vertical_cycle_target_agl
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl;
                }
            } // End processing previous second

            // --- Reset ALL state variables for the new second ---
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN();
            foundTargetBeacon = false;
            closestHorizAnonRange = std::numeric_limits<float>::max();
            closestHorizAnonAzimuth = std::numeric_limits<float>::quiet_NaN();
            hasHorizAnonData = false;
        } // End if (new second)

        // --- Data Accumulation for the *current* second ---
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) {
                 targetBeaconAzimuth = obj.Az;
                 current_beacon_range = obj.Range;
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) lowestRange = obj.Range;

            if (obj.sensor == "R_Az" || obj.sensor == "R_El_R_Az") {
                 hasHorizAnonData = true;
                 if (obj.Range < closestHorizAnonRange) {
                     closestHorizAnonRange = obj.Range;
                     closestHorizAnonAzimuth = obj.Az;
                 }
            }
        }
        if (stopProcessingFlag.load()) return;
    } // End loop through objects
}

// --- END NEW VERTICAL CYCLE FUNCTIONS ---


// Parses JSON radar data
std::vector<RadarObject> parseRadarData(const std::string& jsonData) {
    std::vector<RadarObject> radarObjects;
    if (jsonData.empty() || jsonData == "{}") return radarObjects;

    try {
        auto jsonFrame = json::parse(jsonData);
        if (!jsonFrame.contains("objects") || !jsonFrame["objects"].is_array()) return radarObjects;

        for (const auto& obj : jsonFrame["objects"]) {
            if (!obj.is_object()) {
                std::cerr << "Skipping non-
