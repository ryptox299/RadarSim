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
#include <iomanip>   // For std::put_time in timestamp, std::setw, std::left

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
// Vertical Cycling Parameters
float vertical_cycle_target_agl = 2.0; // Target height AGL for descent phase (meters)
int vertical_cycle_count = 3;          // Number of down-up cycles
float vertical_cycle_agl_tolerance = 0.15; // Allowed error for reaching target AGL (meters)
// Reused parameters for vertical cycling:
float ascent_speed = 0.3;           // Speed for ascending (m/s, positive value) - Reused
float descent_speed = 0.3;          // Speed for descending (m/s, positive value) - Reused
float target_beacon_range = 3.0;    // Target range from beacon when level (meters) - Reused
float beacon_range_tolerance = 0.2; // Allowed error for beacon range checks (meters) - Reused
// Tolerances for checking position before starting vertical cycle descent
float position_distance_tolerance = 0.3; // Allowed error for wall distance check (meters)
float position_azimuth_tolerance = 2.0;  // Allowed error for azimuth/yaw checks (degrees)
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
                }
                // --- ADDED PARAMETER LOADING ---
                else if (key == "vertical_cycle_target_agl") {
                    vertical_cycle_target_agl = std::stof(value);
                    std::cout << "  vertical_cycle_target_agl set to: " << vertical_cycle_target_agl << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "vertical_cycle_count") {
                    vertical_cycle_count = std::stoi(value);
                    std::cout << "  vertical_cycle_count set to: " << vertical_cycle_count << " (from preferences file)" << std::endl; // Logged
                } else if (key == "vertical_cycle_agl_tolerance") {
                    vertical_cycle_agl_tolerance = std::stof(value);
                    std::cout << "  vertical_cycle_agl_tolerance set to: " << vertical_cycle_agl_tolerance << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "ascent_speed") {
                    ascent_speed = std::stof(value);
                    std::cout << "  ascent_speed set to: " << ascent_speed << " m/s (from preferences file)" << std::endl; // Logged
                } else if (key == "descent_speed") {
                    descent_speed = std::stof(value);
                    std::cout << "  descent_speed set to: " << descent_speed << " m/s (from preferences file)" << std::endl; // Logged
                } else if (key == "target_beacon_range") {
                    target_beacon_range = std::stof(value);
                    std::cout << "  target_beacon_range (Level) set to: " << target_beacon_range << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "beacon_range_tolerance") {
                    beacon_range_tolerance = std::stof(value);
                    std::cout << "  beacon_range_tolerance set to: " << beacon_range_tolerance << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "position_distance_tolerance") {
                    position_distance_tolerance = std::stof(value);
                    std::cout << "  position_distance_tolerance set to: " << position_distance_tolerance << " meters (from preferences file)" << std::endl; // Logged
                } else if (key == "position_azimuth_tolerance") {
                    position_azimuth_tolerance = std::stof(value);
                    std::cout << "  position_azimuth_tolerance set to: " << position_azimuth_tolerance << " degrees (from preferences file)" << std::endl; // Logged
                }
                // --- END ADDED PARAMETER LOADING ---
                 else {
                     // Optionally log unrecognized keys
                     // std::cout << "  Ignoring unrecognized key: " << key << std::endl;
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
        std::cout << "  Default Kp_yaw: " << Kp_yaw << std::endl; // Logged
        std::cout << "  Default max_yaw_rate: " << max_yaw_rate << " deg/s" << std::endl; // Logged
        std::cout << "  Default yaw_dead_zone: " << yaw_dead_zone << " degrees" << std::endl; // Logged
        // --- ADDED DEFAULT PRINTOUTS ---
        std::cout << "  Default vertical_cycle_target_agl: " << vertical_cycle_target_agl << " meters" << std::endl; // Logged
        std::cout << "  Default vertical_cycle_count: " << vertical_cycle_count << std::endl; // Logged
        std::cout << "  Default vertical_cycle_agl_tolerance: " << vertical_cycle_agl_tolerance << " meters" << std::endl; // Logged
        std::cout << "  Default ascent_speed: " << ascent_speed << " m/s" << std::endl; // Logged
        std::cout << "  Default descent_speed: " << descent_speed << " m/s" << std::endl; // Logged
        std::cout << "  Default target_beacon_range (Level): " << target_beacon_range << " meters" << std::endl; // Logged
        std::cout << "  Default beacon_range_tolerance: " << beacon_range_tolerance << " meters" << std::endl; // Logged
        std::cout << "  Default position_distance_tolerance: " << position_distance_tolerance << " meters" << std::endl; // Logged
        std::cout << "  Default position_azimuth_tolerance: " << position_azimuth_tolerance << " degrees" << std::endl; // Logged
        // --- END ADDED DEFAULT PRINTOUTS ---
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

// --- ORIGINAL FUNCTION [w] --- // UPDATED LOGGING
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
                    } else if (!foundTargetBeacon) {
                        // Log beacon loss silently unless needed elsewhere
                        // std::cout << "***BEACON NOT FOUND***" << std::endl;
                    }
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, 0, 0); // Z vel and Yaw rate are 0

                    // --- UPDATED LOGGING ('w') ---
                    std::cout << "[Wall Follow] Control Status:\n" // Changed Title
                              << "  TargetWall  = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n" // Fixed width, precision
                              << "  CurrentWall = " << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n" // Separate line
                              // Removed Beacon Azimuth lines
                              << "\n" // Blank line before velocity
                              << "  Computed Velocity(X=" << velocity_x << ", Y=" << velocity_y << ")" << std::endl; // Separate line
                    std::cout << "--------------------------------------" << std::endl; // Logged
                    // --- END UPDATED LOGGING ---

                    vehicle->control->flightCtrl(ctrlData);
                } else if (hasAnonData) { // Only log if there's wall data when control disabled
                     // --- UPDATED LOGGING ('w' - No Control) ---
                     std::cout << "[Wall Follow] (Flight Control Disabled or Not Available)\n"
                               << "  TargetWall  = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n"
                               << "  CurrentWall = " << std::to_string(lowestRange)
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl; // Logged
                     // --- END UPDATED LOGGING ---
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
// --- END ORIGINAL FUNCTION --


// --- FUNCTION FOR YAW LOCK TEST [y] --- // UPDATED LOGGING
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
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY |
                                          DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE |
                                          DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;

                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, 0.0f, 0.0f, yaw_rate); // Y and Z velocity are 0

                    // --- UPDATED LOGGING ('y') ---
                    std::cout << "[Yaw Lock] Control Status: \n"
                              << "  TargetWall  = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n" // Renamed, formatted
                              << "  CurrentWall = " << std::to_string(closestHorizAnonRange) << "\n" // Renamed
                              << "\n" // Blank line
                              << "  TargetWallAz= " << std::fixed << std::setprecision(1) << std::setw(3) << 0.0f << "\n" // Formatted
                              << "  CurrentWAz  = " << std::to_string(closestHorizAnonAzimuth) << "\n" // Renamed
                              << "\n" // Blank line
                              << "  Computed Vel(X)=" << velocity_x << " | Computed YawRate=" << yaw_rate << " deg/s"
                              << std::endl;
                    std::cout << "--------------------------------------" << std::endl;
                    // --- END UPDATED LOGGING ---

                    // --- Send Command ---
                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasHorizAnonData) { // Control disabled but we have data
                     // --- UPDATED LOGGING ('y' - No Control) ---
                     std::cout << "[Yaw Lock] (Flight Control Disabled or Not Available)\n"
                               << "  TargetWall  = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n"
                               << "  CurrentWall = " << std::to_string(closestHorizAnonRange) << "\n"
                               << "\n"
                               << "  TargetWallAz= " << std::fixed << std::setprecision(1) << std::setw(3) << 0.0f << "\n"
                               << "  CurrentWAz  = " << std::to_string(closestHorizAnonAzimuth)
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl;
                     // --- END UPDATED LOGGING ---
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

// --- FUNCTION FOR COMBINED MODE [b] --- // UPDATED LOGGING
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
                    } else if (!foundTargetBeacon) { // Log if beacon not found
                         std::cout << "***BEACON NOT FOUND***" << std::endl;
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
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY |
                                          DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE |
                                          DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;

                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, 0.0f, yaw_rate); // Z velocity is 0

                    // --- UPDATED LOGGING ('b') ---
                    std::cout << "[Combined Mode] Control Status: \n"
                              << "  TargetWall    = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n" // Renamed, formatted
                              << "  CurrentWall   = " << (hasHorizAnonData ? std::to_string(closestHorizAnonRange) : "N/A") << "\n" // Renamed
                              << "\n" // Blank line
                              << "  TargetWallAz  = " << std::fixed << std::setprecision(1) << std::setw(3) << 0.0f << "\n" // Formatted
                              << "  CurrentWAz    = " << (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) ? std::to_string(closestHorizAnonAzimuth) : "N/A") << "\n" // Renamed
                              << "\n" // Blank line
                              << "  TargetBeaconAz= " << std::fixed << std::setprecision(1) << std::setw(3) << targetAzimuth << "\n" // Formatted
                              << "  CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "\n" // Blank line
                              << "  Computed Vel(X=" << velocity_x << ", Y=" << velocity_y << ") | Computed YawRate=" << yaw_rate << " deg/s"
                              << std::endl;
                    std::cout << "--------------------------------------" << std::endl;
                    // --- END UPDATED LOGGING ---

                    // --- Send Command ---
                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasHorizAnonData || foundTargetBeacon) { // Control disabled but we have data
                     if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl;}
                     // --- UPDATED LOGGING ('b' - No Control) ---
                     std::cout << "[Combined Mode] (Flight Control Disabled or Not Available)\n"
                               << "  TargetWall    = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n"
                               << "  CurrentWall   = " << (hasHorizAnonData ? std::to_string(closestHorizAnonRange) : "N/A") << "\n"
                               << "\n"
                               << "  TargetWallAz  = " << std::fixed << std::setprecision(1) << std::setw(3) << 0.0f << "\n"
                               << "  CurrentWAz    = " << (hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) ? std::to_string(closestHorizAnonAzimuth) : "N/A") << "\n"
                               << "\n"
                               << "  TargetBeaconAz= " << std::fixed << std::setprecision(1) << std::setw(3) << targetAzimuth << "\n"
                               << "  CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A")
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl;
                     // --- END UPDATED LOGGING ---
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
                    } else if (current_phase != VerticalCyclePhase::COMPLETED_ALL) { // Log beacon loss if not finished
                         std::cout << "***BEACON NOT FOUND***" << std::endl;
                         if (current_phase == VerticalCyclePhase::ASCENDING || current_phase == VerticalCyclePhase::DESCENDING) {
                             velocity_y = 0.0f; // Stop lateral movement if beacon lost during active cycle
                         }
                    }

                    // --- Vertical Cycle State Machine ---
                    bool beacon_range_ok = foundTargetBeacon && !std::isnan(current_beacon_range) && (std::abs(current_beacon_range - target_beacon_range) <= beacon_range_tolerance);
                    // Conditions for starting descent (Horizontal alignment for 'w' + beacon range)
                    bool horizontal_dist_ok = hasAnonData && (std::abs(lowestRange - targetDistance) <= position_distance_tolerance);
                    bool lateral_beacon_az_ok = foundTargetBeacon && !std::isnan(targetBeaconAzimuth) && (std::abs(targetBeaconAzimuth - targetAzimuth) <= position_azimuth_tolerance);

                    switch (current_phase) {
                        case VerticalCyclePhase::ALIGNING_INITIAL:
                            velocity_z = 0.0f; // Maintain altitude while aligning
                            // Check if horizontal distance, lateral beacon azimuth, AND beacon range are met
                            if (horizontal_dist_ok && lateral_beacon_az_ok && beacon_range_ok) {
                                std::cout << "[Vert Cycle W] Initial position confirmed. Starting Cycle " << current_cycle + 1 << "." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::DESCENDING;
                            } else {
                                // Optionally adjust Z velocity to reach beacon range if not already there
                                if (foundTargetBeacon && !std::isnan(current_beacon_range) && std::abs(current_beacon_range - target_beacon_range) > beacon_range_tolerance) {
                                     velocity_z = (current_beacon_range < target_beacon_range) ? ascent_speed : -descent_speed;
                                }
                            }
                            // Horizontal control continues trying to align
                            break;

                        case VerticalCyclePhase::DESCENDING:
                            current_height_agl = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                            // Check AGL validity
                            if (current_height_agl < 0.0f) {
                                std::cerr << "[Vert Cycle W] Warning: Invalid AGL height data (" << current_height_agl << "). Hovering." << std::endl; // Logged
                                velocity_z = 0.0f;
                                break; // Skip rest of phase logic for this cycle
                            }
                            // Check AGL height
                            if (current_height_agl > vertical_cycle_target_agl + vertical_cycle_agl_tolerance) {
                                velocity_z = -descent_speed;
                            } else { // Reached target AGL
                                velocity_z = 0.0f;
                                std::cout << "[Vert Cycle W] Target AGL reached (" << current_height_agl << "m). Starting ascent." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::ASCENDING;
                            }
                            // Horizontal control continues (lateral stops if beacon lost)
                            break;

                        case VerticalCyclePhase::ASCENDING:
                            if (foundTargetBeacon && !std::isnan(current_beacon_range)) {
                                // Check beacon range
                                if (current_beacon_range < target_beacon_range - beacon_range_tolerance) { // Only ascend if below target
                                    velocity_z = ascent_speed;
                                } else if (current_beacon_range > target_beacon_range + beacon_range_tolerance) { // Descend if overshot
                                    velocity_z = -descent_speed;
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
                                 // Beacon not found message printed by horizontal control section
                                 velocity_z = 0.0f; // Stop ascending/descending
                                 std::cerr << "[Vert Cycle W] Warning: Beacon lost during ascent/leveling. Hovering vertically." << std::endl; // Logged
                            }
                            // Horizontal control continues (lateral stops if beacon lost)
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

                    // --- Logging --- // (No changes requested for vertical cycle logs yet)
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
                    bool beacon_range_ok = foundTargetBeacon && !std::isnan(current_beacon_range) && (std::abs(current_beacon_range - target_beacon_range) <= beacon_range_tolerance);
                    // Conditions for starting descent (Horizontal alignment for 'y' + beacon range)
                    bool horizontal_dist_ok = hasHorizAnonData && (std::abs(closestHorizAnonRange - targetDistance) <= position_distance_tolerance);
                    bool yaw_ok = hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) && (std::abs(closestHorizAnonAzimuth) <= position_azimuth_tolerance);

                    switch (current_phase) {
                        case VerticalCyclePhase::ALIGNING_INITIAL:
                            velocity_z = 0.0f;
                            if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl; }
                            // Check if horizontal distance, yaw, AND beacon range are met
                            if (horizontal_dist_ok && yaw_ok && beacon_range_ok) {
                                std::cout << "[Vert Cycle Y] Initial position confirmed. Starting Cycle " << current_cycle + 1 << "." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::DESCENDING;
                            } else {
                                // Optionally adjust Z velocity to reach beacon range
                                if (foundTargetBeacon && !std::isnan(current_beacon_range) && std::abs(current_beacon_range - target_beacon_range) > beacon_range_tolerance) {
                                     velocity_z = (current_beacon_range < target_beacon_range) ? ascent_speed : -descent_speed;
                                }
                            }
                            // Horizontal/Yaw control continues trying to align
                            break;

                        case VerticalCyclePhase::DESCENDING:
                            current_height_agl = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                            if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl; } // Log beacon loss but continue descent
                             // Check AGL validity
                            if (current_height_agl < 0.0f) {
                                std::cerr << "[Vert Cycle Y] Warning: Invalid AGL height data (" << current_height_agl << "). Hovering." << std::endl; // Logged
                                velocity_z = 0.0f;
                                break; // Skip rest of phase logic for this cycle
                            }
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
                                if (current_beacon_range < target_beacon_range - beacon_range_tolerance) { // Ascend if below
                                    velocity_z = ascent_speed;
                                } else if (current_beacon_range > target_beacon_range + beacon_range_tolerance) { // Descend if above
                                    velocity_z = -descent_speed;
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
                                 velocity_z = 0.0f; // Stop ascending/descending
                                 std::cerr << "[Vert Cycle Y] Warning: Beacon lost during ascent/leveling. Hovering vertically." << std::endl; // Logged
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

                    // --- Logging --- // (No changes requested for vertical cycle logs yet)
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
                    } else if (current_phase != VerticalCyclePhase::COMPLETED_ALL) { // Log beacon loss if not finished
                         std::cout << "***BEACON NOT FOUND***" << std::endl;
                         if (current_phase == VerticalCyclePhase::ASCENDING || current_phase == VerticalCyclePhase::DESCENDING) {
                             velocity_y = 0.0f; // Stop lateral movement if beacon lost during active cycle
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
                    bool beacon_range_ok = foundTargetBeacon && !std::isnan(current_beacon_range) && (std::abs(current_beacon_range - target_beacon_range) <= beacon_range_tolerance);
                    // Conditions for starting descent (Horizontal alignment for 'b' + beacon range)
                    bool horizontal_dist_ok = hasHorizAnonData && (std::abs(closestHorizAnonRange - targetDistance) <= position_distance_tolerance);
                    bool lateral_beacon_az_ok = foundTargetBeacon && !std::isnan(targetBeaconAzimuth) && (std::abs(targetBeaconAzimuth - targetAzimuth) <= position_azimuth_tolerance);
                    bool yaw_ok = hasHorizAnonData && !std::isnan(closestHorizAnonAzimuth) && (std::abs(closestHorizAnonAzimuth) <= position_azimuth_tolerance);

                    switch (current_phase) {
                        case VerticalCyclePhase::ALIGNING_INITIAL:
                            velocity_z = 0.0f;
                            // Check if horizontal distance, lateral beacon azimuth, yaw, AND beacon range are met
                            if (horizontal_dist_ok && lateral_beacon_az_ok && yaw_ok && beacon_range_ok) {
                                std::cout << "[Vert Cycle B] Initial position confirmed. Starting Cycle " << current_cycle + 1 << "." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::DESCENDING;
                            } else {
                                // Optionally adjust Z velocity to reach beacon range
                                if (foundTargetBeacon && !std::isnan(current_beacon_range) && std::abs(current_beacon_range - target_beacon_range) > beacon_range_tolerance) {
                                     velocity_z = (current_beacon_range < target_beacon_range) ? ascent_speed : -descent_speed;
                                }
                            }
                            // Horizontal/Yaw control continues trying to align
                            break;

                        case VerticalCyclePhase::DESCENDING:
                            current_height_agl = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                             // Check AGL validity
                            if (current_height_agl < 0.0f) {
                                std::cerr << "[Vert Cycle B] Warning: Invalid AGL height data (" << current_height_agl << "). Hovering." << std::endl; // Logged
                                velocity_z = 0.0f;
                                break; // Skip rest of phase logic for this cycle
                            }
                            // Check AGL height
                            if (current_height_agl > vertical_cycle_target_agl + vertical_cycle_agl_tolerance) {
                                velocity_z = -descent_speed;
                            } else { // Reached target AGL
                                velocity_z = 0.0f;
                                std::cout << "[Vert Cycle B] Target AGL reached (" << current_height_agl << "m). Starting ascent." << std::endl; // Logged
                                current_phase = VerticalCyclePhase::ASCENDING;
                            }
                            // Horizontal/Yaw control continues (lateral stops if beacon lost)
                            break;

                        case VerticalCyclePhase::ASCENDING:
                            if (foundTargetBeacon && !std::isnan(current_beacon_range)) {
                                // Check beacon range
                                if (current_beacon_range < target_beacon_range - beacon_range_tolerance) { // Ascend if below
                                    velocity_z = ascent_speed;
                                } else if (current_beacon_range > target_beacon_range + beacon_range_tolerance) { // Descend if above
                                    velocity_z = -descent_speed;
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
                                 // Beacon not found message printed by horizontal control section
                                 velocity_z = 0.0f; // Stop ascending/descending
                                 std::cerr << "[Vert Cycle B] Warning: Beacon lost during ascent/leveling. Hovering vertically." << std::endl; // Logged
                            }
                            // Horizontal/Yaw control continues (lateral stops if beacon lost)
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

                    // --- Logging --- // (No changes requested for vertical cycle logs yet)
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
                // --- CORRECTED LINE ---
                std::cerr << "Skipping non-object item in 'objects' array." << std::endl; // Logged
                continue;
            }
            RadarObject radarObj;
            radarObj.timestamp = obj.value("timestamp", json(nullptr)).dump();
            radarObj.sensor = obj.value("sensor", "N/A");
            radarObj.src = obj.value("src", "N/A");
            radarObj.X = obj.value("X", 0.0f); radarObj.Y = obj.value("Y", 0.0f); radarObj.Z = obj.value("Z", 0.0f);
            radarObj.Xdir = obj.value("Xdir", 0.0f); radarObj.Ydir = obj.value("Ydir", 0.0f); radarObj.Zdir = obj.value("Zdir", 0.0f);
            radarObj.Range = obj.value("Range", 0.0f); radarObj.RangeRate = obj.value("RangeRate", 0.0f);
            radarObj.Pwr = obj.value("Pwr", 0.0f); radarObj.Az = obj.value("Az", 0.0f); radarObj.El = obj.value("El", 0.0f);
            if (obj.contains("ID") && obj["ID"].is_string()) radarObj.ID = obj.value("ID", "N/A");
            else if (obj.contains("ID")) radarObj.ID = obj["ID"].dump(); else radarObj.ID = "N/A";
            radarObj.Xsize = obj.value("Xsize", 0.0f); radarObj.Ysize = obj.value("Ysize", 0.0f); radarObj.Zsize = obj.value("Zsize", 0.0f);
            radarObj.Conf = obj.value("Conf", 0.0f);
            radarObjects.push_back(radarObj);
        }
    } catch (const json::parse_error& e) {
        std::cerr << "JSON Parsing Error: " << e.what() << " at offset " << e.byte << ". Data: [" << jsonData.substr(0, 200) << "...]" << std::endl; // Logged
        return {};
    } catch (const json::type_error& e) {
        std::cerr << "JSON Type Error: " << e.what() << ". Data: [" << jsonData.substr(0, 200) << "...]" << std::endl; // Logged
        return {};
    }
    return radarObjects;
}

// Runs the python bridge script
void runPythonBridge(const std::string& scriptName) {
    std::cout << "Starting Python bridge (" << scriptName << ")..." << std::endl; // Logged
    if (std::system(("python3 " + scriptName + " &").c_str()) != 0) {
        std::cerr << "Failed to start Python bridge script '" << scriptName << "'." << std::endl; // Logged
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Python bridge potentially started." << std::endl; // Logged
}

// Stops the python bridge script
void stopPythonBridge(const std::string& scriptName) {
    std::cout << "Stopping Python bridge (" << scriptName << ")..." << std::endl; // Logged
    std::system(("pkill -f " + scriptName).c_str()); // Ignore result
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Sent SIGTERM to " << scriptName << "." << std::endl; // Logged
}

// Connects to the python bridge via TCP
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket) {
    tcp::resolver resolver(io_context);
    int retries = 5;
    while (retries-- > 0) {
        try {
            if(socket.is_open()) { socket.close(); } // Simpler close
            auto endpoints = resolver.resolve("127.0.0.1", "5000");
            boost::system::error_code ec;
            boost::asio::connect(socket, endpoints, ec);
            if (!ec) { std::cout << "Connected to Python bridge." << std::endl; return true; } // Logged
            else { std::cerr << "Connection attempt failed: " << ec.message() << std::endl; } // Logged
        } catch (const std::exception& e) {
            std::cerr << "Exception during connection attempt: " << e.what() << std::endl; // Logged
        }
        if(retries > 0 && !stopProcessingFlag.load()) {
             std::cout << "Retrying connection in 2 seconds... (" << retries << " attempts left)" << std::endl; // Logged
             for (int i = 0; i < 2 && !stopProcessingFlag.load(); ++i) std::this_thread::sleep_for(std::chrono::seconds(1));
             if (stopProcessingFlag.load()) { std::cout << "Stop requested during connection retry." << std::endl; break; } // Logged
        } else if (stopProcessingFlag.load()) { std::cout << "Stop requested during connection retry." << std::endl; break; } // Logged
    }
     std::cerr << "Failed to connect to Python bridge after multiple attempts." << std::endl; // Logged
     return false;
}

// Placeholder Callbacks
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }
void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }

// Enum for processing modes
enum class ProcessingMode {
    WALL_FOLLOW,
    PROCESS_FULL,
    PROCESS_MINIMAL,
    WALL_FOLLOW_YAW_LOCK,                 // Mode for Yaw Lock Test [y]
    WALL_FOLLOW_BEACON_LATERAL_YAW_LOCK,  // Mode for Combined Yaw Lock + Beacon Lateral [b]
    WALL_FOLLOW_VERTICAL_CYCLE,           // Mode for Vertical Cycle (W Base) [1]
    WALL_FOLLOW_YAW_LOCK_VERTICAL_CYCLE,  // Mode for Vertical Cycle (Y Base) [2]
    WALL_FOLLOW_COMBINED_VERTICAL_CYCLE   // Mode for Vertical Cycle (B Base) [3]
};

// Processing loop function
void processingLoopFunction(const std::string bridgeScriptName, Vehicle* vehicle, bool enableControlCmd, ProcessingMode mode) {
    std::cout << "Processing thread started. Bridge: " << bridgeScriptName << ", Control Enabled: " << std::boolalpha << enableControlCmd << ", Mode: " << static_cast<int>(mode) << std::endl; // Logged
    int functionTimeout = 1;

    runPythonBridge(bridgeScriptName);
    boost::asio::io_context io_context;
    tcp::socket socket(io_context);

    if (!connectToPythonBridge(io_context, socket)) {
        std::cerr << "Processing thread: Initial connection failed. Exiting thread." << std::endl; // Logged
        stopPythonBridge(bridgeScriptName);
        if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
            std::cout << "[Processing Thread] Releasing control authority (initial connection fail)..." << std::endl; // Logged
            ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
            if (ACK::getError(releaseAck)) ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority"); // Logged via ACK
            else std::cout << "[Processing Thread] Control authority released." << std::endl; // Logged
        }
        return; // Exit thread
    }

    std::cout << "Processing thread: Connection successful. Reading data stream..." << std::endl; // Logged
    std::string received_data_buffer;
    std::array<char, 4096> read_buffer;
    bool connection_error_occurred = false;

    // Reset ALL state variables used in processing functions before the loop starts
    currentSecond = "";
    lowestRange = std::numeric_limits<float>::max();
    hasAnonData = false;
    targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
    current_beacon_range = std::numeric_limits<float>::quiet_NaN();
    foundTargetBeacon = false;
    closestHorizAnonRange = std::numeric_limits<float>::max();
    closestHorizAnonAzimuth = std::numeric_limits<float>::quiet_NaN();
    hasHorizAnonData = false;


    while (!stopProcessingFlag.load()) {
        boost::system::error_code error;
        size_t len = socket.read_some(boost::asio::buffer(read_buffer), error);

        if (error) { // Handle read error
            if (error == boost::asio::error::eof) std::cerr << "Processing thread: Connection closed by Python bridge (EOF)." << std::endl; // Logged
            else std::cerr << "Processing thread: Error reading from socket: " << error.message() << std::endl; // Logged
            connection_error_occurred = true;
            break;
        }

        if (len > 0) { // Process received data
            received_data_buffer.append(read_buffer.data(), len);
            size_t newline_pos;
            while ((newline_pos = received_data_buffer.find('\n')) != std::string::npos) {
                std::string jsonData = received_data_buffer.substr(0, newline_pos);
                received_data_buffer.erase(0, newline_pos + 1);
                if (stopProcessingFlag.load()) break;
                if (!jsonData.empty()) {
                    try {
                        auto radarObjects = parseRadarData(jsonData);
                        if (!radarObjects.empty()) {
                            // Switch case includes all modes
                            switch (mode) {
                                case ProcessingMode::WALL_FOLLOW:
                                    extractBeaconAndWallData(radarObjects, vehicle, enableControlCmd);
                                    break;
                                case ProcessingMode::PROCESS_FULL:
                                    displayRadarObjects(radarObjects);
                                    break;
                                case ProcessingMode::PROCESS_MINIMAL:
                                    displayRadarObjectsMinimal(radarObjects);
                                    break;
                                case ProcessingMode::WALL_FOLLOW_YAW_LOCK:
                                    extractBeaconAndWallData_YawLock(radarObjects, vehicle, enableControlCmd);
                                    break;
                                case ProcessingMode::WALL_FOLLOW_BEACON_LATERAL_YAW_LOCK:
                                    extractBeaconAndWallData_Combined(radarObjects, vehicle, enableControlCmd);
                                    break;
                                case ProcessingMode::WALL_FOLLOW_VERTICAL_CYCLE:
                                    extractBeaconAndWallData_VerticalCycle(radarObjects, vehicle, enableControlCmd);
                                    break;
                                case ProcessingMode::WALL_FOLLOW_YAW_LOCK_VERTICAL_CYCLE:
                                    extractBeaconAndWallData_YawLock_VerticalCycle(radarObjects, vehicle, enableControlCmd);
                                    break;
                                case ProcessingMode::WALL_FOLLOW_COMBINED_VERTICAL_CYCLE:
                                    extractBeaconAndWallData_Combined_VerticalCycle(radarObjects, vehicle, enableControlCmd);
                                    break;
                            }
                        }
                    } catch (const std::exception& e) {
                         std::cerr << "Error processing data: " << e.what() << "\nSnippet: [" << jsonData.substr(0, 100) << "...]" << std::endl; // Logged
                    }
                }
            }
            if (stopProcessingFlag.load()) break;
        } else {
             std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Brief pause if no data
        }
    }

    // --- Cleanup ---
    if (stopProcessingFlag.load()) std::cout << "[Processing Thread] Stop requested manually." << std::endl; // Logged
    else if (connection_error_occurred) std::cout << "[Processing Thread] Exiting due to connection error." << std::endl; // Logged
    else std::cout << "[Processing Thread] Data stream ended." << std::endl; // Logged

    if (socket.is_open()) { socket.close(); }
    stopPythonBridge(bridgeScriptName);

    // Release Control Authority if enabled
    if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
        std::cout << "[Processing Thread] Sending final zero velocity command..." << std::endl; // Logged
        uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
        DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
        vehicle->control->flightCtrl(stopData);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "[Processing Thread] Releasing control authority..." << std::endl; // Logged
        ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
        if (ACK::getError(releaseAck)) {
             ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority (on exit)"); // Logged via ACK
             std::cerr << "[Processing Thread] Warning: Failed to release control authority on exit." << std::endl; // Logged
        } else {
             std::cout << "[Processing Thread] Control authority released." << std::endl; // Logged
        }
    }
    std::cout << "Processing thread finished." << std::endl; // Logged
}

// Helper function to get mode name string
std::string getModeName(uint8_t mode) {
    switch(mode) {
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL: return "MANUAL_CTRL";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE: return "ATTITUDE";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS: return "P_GPS";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL: return "NAVI_SDK_CTRL";
        case 31: return "Mode 31";
        default: return "Other (" + std::to_string(mode) + ")";
    }
}

// Background thread function for monitoring
void monitoringLoopFunction(Vehicle* vehicle) {
    std::cout << "[Monitoring] Thread started." << std::endl; // Logged
    bool telemetry_timed_out = false;
    bool warned_unexpected_status = false;
    uint8_t previous_flight_status = DJI::OSDK::VehicleStatus::FlightStatus::STOPED;
    time_t last_valid_poll_time = 0;
    bool in_sdk_control_mode = false;
    bool warned_not_in_sdk_mode = false;
    const uint8_t EXPECTED_SDK_MODE = DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL;

    while (!stopMonitoringFlag.load()) {
        if (vehicle == nullptr || vehicle->subscribe == nullptr) {
             if (!telemetry_timed_out) {
                 std::cerr << "\n**** MONITORING ERROR: Vehicle/subscribe object null. Stopping. ****" << std::endl << std::endl; // Logged
                 telemetry_timed_out = true;
             }
             break;
        }

        uint8_t current_flight_status = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
        uint8_t current_display_mode = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        // float current_height = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_HEIGHT_FUSION>(); // AGL data available here
        bool valid_poll = (current_flight_status <= DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR);

        if (valid_poll) {
            time_t current_time = std::time(nullptr);
            last_valid_poll_time = current_time;
            if (telemetry_timed_out) {
                 std::cout << "[Monitoring] Telemetry poll recovered." << std::endl; // Logged
                 telemetry_timed_out = false;
            }

            // Check Flight Status Change
            if (current_flight_status != DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) {
                if ( (previous_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
                     (current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND || current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::STOPED) &&
                     !warned_unexpected_status) {
                     std::cerr << "\n**** MONITORING WARNING: Flight status changed unexpectedly from IN_AIR to " << (int)current_flight_status << ". ****" << std::endl << std::endl; // Logged
                     warned_unexpected_status = true;
                }
            } else {
                 warned_unexpected_status = false;
            }
            previous_flight_status = current_flight_status;

            // Check Expected SDK Mode
            bool is_expected_mode = (current_display_mode == EXPECTED_SDK_MODE);
            if (is_expected_mode) {
                if (!in_sdk_control_mode) {
                    std::cout << "\n**** MONITORING INFO: Entered SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << " / " << (int)EXPECTED_SDK_MODE << ") ****" << std::endl << std::endl; // Logged
                    in_sdk_control_mode = true;
                    warned_not_in_sdk_mode = false;
                }
            } else { // Not in expected mode
                 if (in_sdk_control_mode || !warned_not_in_sdk_mode) {
                      // Check if processing thread is stopping/stopped
                      if (!stopProcessingFlag.load() && processingThread.joinable()) {
                          std::string current_mode_name = getModeName(current_display_mode);
                          std::cerr << "\n**** MONITORING WARNING: NOT in expected SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << "). Current: " << current_mode_name << " (" << (int)current_display_mode << ") ****" << std::endl << std::endl; // Logged
                          warned_not_in_sdk_mode = true;
                      }
                      in_sdk_control_mode = false;
                 }
            }
        } else { // Invalid poll
            if (!telemetry_timed_out) {
                 std::cerr << "\n**** MONITORING WARNING: Polling telemetry returned potentially invalid data (FlightStatus=" << (int)current_flight_status << "). ****" << std::endl << std::endl; // Logged
                 telemetry_timed_out = true;
            }
        }

        // Check Telemetry Timeout
        time_t current_time_for_timeout_check = std::time(nullptr);
        if (last_valid_poll_time > 0 && (current_time_for_timeout_check - last_valid_poll_time > TELEMETRY_TIMEOUT_SECONDS)) {
            if (!telemetry_timed_out) {
                std::cerr << "\n**** MONITORING TIMEOUT: No valid telemetry for over " << TELEMETRY_TIMEOUT_SECONDS << " seconds. ****" << std::endl << std::endl; // Logged
                telemetry_timed_out = true;
            }
        } else if (last_valid_poll_time == 0) { // Check if never received first poll
             static time_t start_time = 0; if (start_time == 0) start_time = current_time_for_timeout_check;
             if (current_time_for_timeout_check - start_time > TELEMETRY_TIMEOUT_SECONDS * 2) {
                  if (!telemetry_timed_out) {
                       std::cerr << "\n**** MONITORING TIMEOUT: Never received valid telemetry poll after " << TELEMETRY_TIMEOUT_SECONDS * 2 << " seconds. ****" << std::endl << std::endl; // Logged
                       telemetry_timed_out = true;
                  }
             }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "[Monitoring] Thread finished." << std::endl; // Logged
}


int main(int argc, char** argv) {
    // --- Instantiate LogRedirector early in main ---
    LogRedirector logger("run_log.txt");

    loadPreferences(); // Load preferences first

    // --- Mode Selection Removed - Hardcoded to Live Mode ---
    bool enableFlightControl = true;
    // defaultPythonBridgeScript is already set globally

    std::cout << "Starting in Live Mode. Flight control enabled. Default bridge: " << defaultPythonBridgeScript << std::endl; // Logged


    // --- OSDK Initialization ---
    int functionTimeout = 1;
    Vehicle* vehicle = nullptr;
    FlightSample* flightSample = nullptr;
    LinuxSetup* linuxEnvironment = nullptr;
    int telemetrySubscriptionFrequency = 10; // Increased frequency
    int pkgIndex = 0;
    bool monitoringEnabled = false;

    if (enableFlightControl) {
        std::cout << "Initializing DJI OSDK..." << std::endl; // Logged
        linuxEnvironment = new LinuxSetup(argc, argv);
        vehicle = linuxEnvironment->getVehicle();
        if (vehicle == nullptr || vehicle->control == nullptr || vehicle->subscribe == nullptr) {
            std::cerr << "ERROR: Vehicle not initialized or interfaces unavailable. Disabling flight control." << std::endl; // Logged
             if (linuxEnvironment) { delete linuxEnvironment; linuxEnvironment = nullptr; }
             vehicle = nullptr;
             enableFlightControl = false;
             std::cout << "OSDK Initialization Failed. Flight control disabled." << std::endl; // Logged
        } else { // OSDK Init seems OK
            std::cout << "Attempting to obtain Control Authority..." << std::endl; // Logged
            ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
            if (ACK::getError(ctrlAuthAck)) {
                 ACK::getErrorCodeMessage(ctrlAuthAck, __func__); // Logged via ACK
                 std::cerr << "Failed to obtain control authority. Disabling flight control." << std::endl; // Logged
                 if (linuxEnvironment) { delete linuxEnvironment; linuxEnvironment = nullptr; }
                 vehicle = nullptr;
                 enableFlightControl = false;
                 std::cout << "Obtaining Control Authority Failed. Flight control disabled." << std::endl; // Logged
            } else { // Authority Obtained
                 std::cout << "Obtained Control Authority." << std::endl; // Logged
                 flightSample = new FlightSample(vehicle);
                 std::cout << "OSDK Initialized and Flight Sample created." << std::endl; // Logged

                 // --- Setup Telemetry Subscription for Monitoring AND AGL ---
                 std::cout << "Setting up Telemetry Subscription for Monitoring & AGL..." << std::endl; // Logged
                 ACK::ErrorCode subscribeAck = vehicle->subscribe->verify(functionTimeout);
                 if (ACK::getError(subscribeAck)) {
                      ACK::getErrorCodeMessage(subscribeAck, __func__); // Logged via ACK
                      std::cerr << "Error verifying subscription package list. Monitoring & AGL will be disabled." << std::endl; // Logged
                 } else {
                      Telemetry::TopicName topicList[] = {
                          Telemetry::TOPIC_STATUS_FLIGHT,
                          Telemetry::TOPIC_STATUS_DISPLAYMODE,
                          Telemetry::TOPIC_HEIGHT_FUSION // Added AGL Topic
                      };
                      int numTopic = sizeof(topicList) / sizeof(topicList[0]);
                      bool topicStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList,
                                                                                       false, telemetrySubscriptionFrequency);

                      if (topicStatus) {
                            std::cout << "Successfully initialized package " << pkgIndex << " with Flight Status, Display Mode, and Height Fusion topics." << std::endl; // Logged Updated msg
                            ACK::ErrorCode startAck = vehicle->subscribe->startPackage(pkgIndex, functionTimeout);
                            if (ACK::getError(startAck)) {
                                 ACK::getErrorCodeMessage(startAck, "startPackage"); // Logged via ACK
                                 std::cerr << "Error starting subscription package " << pkgIndex << ". Monitoring & AGL will be disabled." << std::endl; // Logged
                                 vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
                            } else {
                                 std::cout << "Successfully started package " << pkgIndex << "." << std::endl; // Logged
                                 std::cout << "Starting monitoring thread..." << std::endl; // Logged
                                 stopMonitoringFlag.store(false);
                                 monitoringThread = std::thread(monitoringLoopFunction, vehicle);
                                 monitoringEnabled = true;
                                 std::cout << "[Main] Monitoring thread launched." << std::endl; // Logged
                            }
                      } else {
                           std::cerr << "Error initializing package " << pkgIndex << " from topic list. Monitoring & AGL will be disabled." << std::endl; // Logged
                      }
                 }
            }
        }
    }

    std::cout << "INFO: Flight control is " << (enableFlightControl ? "ENABLED" : "DISABLED") << ". Proceeding to main menu." << std::endl; // Logged Keep this info line

    // --- Main Command Loop ---
    bool keepRunning = true;
    while (keepRunning) {
        // --- Updated Menu Display - Includes [1, 2, 3] ---
        std::cout << "\n--- Main Menu ---\n" // Logged
                  << (enableFlightControl ? "| [t] Monitored Takeoff                     |\n| [w] Start Wall+Beacon Following (Default) |\n| [y] Start Wall Following + Yaw Lock       |\n| [b] Start Combined Mode (Y+W)             |\n| [1] Start Vertical Cycle (W Base)         |\n| [2] Start Vertical Cycle (Y Base)         |\n| [3] Start Vertical Cycle (B Base)         |\n"
                                           : "| [t] Takeoff (DISABLED)                    |\n| [w] Wall/Beacon Following (No Control)    |\n| [y] Wall Following + Yaw Lock (No Control)|\n| [b] Combined Mode (Y+W) (No Control)      |\n| [1] Vertical Cycle (W Base) (No Control)  |\n| [2] Vertical Cycle (Y Base) (No Control)  |\n| [3] Vertical Cycle (B Base) (No Control)  |\n")
                  << "| [e] Process Full Radar (No Control)       |\n" // Logged
                  << "| [f] Process Minimal Radar (No Control)    |\n" // Logged
                  << "| [q] Quit                                  |\n" // Logged
                  << "---------------------------------------------\n" // Logged
                  << "Enter command: "; // Logged (No endl here)

        std::string lineInput; char inputChar = 0;
        if (std::getline(std::cin, lineInput)) {
             std::cout << lineInput << std::endl; // Echo input to log
             if (!lineInput.empty()) inputChar = lineInput[0];
        }
        else { inputChar = 'q'; if (std::cin.eof()) std::cout << "\nEOF detected. "; std::cout << "Exiting." << std::endl; } // Logged

        // Stop processing thread if starting new task or quitting
         if (strchr("wefyb123", inputChar) != nullptr || inputChar == 'q') {
             if (processingThread.joinable()) {
                 std::cout << "Signalling processing thread to stop..." << std::endl; // Logged
                 stopProcessingFlag.store(true); processingThread.join();
                 std::cout << "Processing thread finished." << std::endl; // Logged
                 stopProcessingFlag.store(false);
             }
         }
         // Stop monitoring thread on quit
         if (inputChar == 'q' && monitoringThread.joinable()) {
             std::cout << "Signalling monitoring thread to stop..." << std::endl; // Logged
             stopMonitoringFlag.store(true); monitoringThread.join();
             std::cout << "Monitoring thread finished." << std::endl; // Logged
             stopMonitoringFlag.store(false); monitoringEnabled = false;
         }

        // --- Process Command - Includes cases '1', '2', '3' ---
        switch (inputChar) {
            case 't': // Takeoff
                if (enableFlightControl && flightSample && vehicle && vehicle->control) {
                    std::cout << "Attempting to obtain Control Authority for Takeoff..." << std::endl; // Logged
                    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                    if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Takeoff obtainCtrlAuthority"); // Logged via ACK
                         std::cerr << "Failed to obtain control authority for takeoff." << std::endl; // Logged
                         break;
                    } else {
                         std::cout << "Obtained Control Authority for Takeoff." << std::endl; // Logged
                         flightSample->monitoredTakeoff(); // Internal logging exists
                    }
                } else {
                     std::cout << "Flight control not enabled or available." << std::endl; // Logged
                }
                break;

             case 'w': { // Wall Following (Default)
                 std::string bridge = defaultPythonBridgeScript;
                 bool useControl = enableFlightControl;

                 std::cout << "Starting Wall+Beacon Following using bridge: " << bridge << std::endl; // Logged
                 if (useControl && vehicle && vehicle->control) {
                     std::cout << "Attempting to obtain Control Authority for Wall Following..." << std::endl; // Logged
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Wall Following obtainCtrlAuthority"); // Logged via ACK
                         std::cerr << "Failed to obtain control authority. Cannot start with control." << std::endl; // Logged
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Wall Following." << std::endl; // Logged
                     }
                 } else if (useControl) {
                      std::cerr << "Error: Cannot start with control as OSDK is not properly initialized." << std::endl; // Logged
                      break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, bridge, vehicle, useControl, ProcessingMode::WALL_FOLLOW);
                 break;
             }
             case 'y': { // Wall Following + Yaw Lock
                 std::cout << "Starting Wall Following + Yaw Lock using default bridge: " << defaultPythonBridgeScript << std::endl; // Logged
                 if (enableFlightControl && vehicle && vehicle->control) {
                     std::cout << "Attempting to obtain Control Authority for Yaw Lock Test..." << std::endl; // Logged
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Yaw Lock obtainCtrlAuthority"); // Logged via ACK
                         std::cerr << "Failed to obtain control authority. Cannot start Yaw Lock Test with control." << std::endl; // Logged
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Yaw Lock Test." << std::endl; // Logged
                     }
                 } else if (enableFlightControl) {
                      std::cerr << "Error: Cannot start Yaw Lock Test with control as OSDK control interface is not properly initialized." << std::endl; // Logged
                      break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW_YAW_LOCK);
                 break;
             }
             case 'b': { // Combined Mode: Yaw Lock + Beacon Lateral
                 std::cout << "Starting Combined Mode (Yaw Lock + Beacon Lateral) using default bridge: " << defaultPythonBridgeScript << std::endl; // Logged
                 if (enableFlightControl && vehicle && vehicle->control) {
                     std::cout << "Attempting to obtain Control Authority for Combined Mode..." << std::endl; // Logged
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Combined Mode obtainCtrlAuthority"); // Logged via ACK
                         std::cerr << "Failed to obtain control authority. Cannot start Combined Mode with control." << std::endl; // Logged
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Combined Mode." << std::endl; // Logged
                     }
                 } else if (enableFlightControl) {
                      std::cerr << "Error: Cannot start Combined Mode with control as OSDK control interface is not properly initialized." << std::endl; // Logged
                      break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW_BEACON_LATERAL_YAW_LOCK);
                 break;
             }
             case '1': { // Vertical Cycle (W Base)
                 std::cout << "Starting Vertical Cycle (W Base) using default bridge: " << defaultPythonBridgeScript << std::endl; // Logged
                 if (enableFlightControl && vehicle && vehicle->control) {
                     std::cout << "Attempting to obtain Control Authority for Vertical Cycle (W)..." << std::endl; // Logged
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Vertical Cycle (W) obtainCtrlAuthority"); // Logged via ACK
                         std::cerr << "Failed to obtain control authority. Cannot start Vertical Cycle (W) with control." << std::endl; // Logged
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Vertical Cycle (W)." << std::endl; // Logged
                     }
                 } else if (enableFlightControl) {
                      std::cerr << "Error: Cannot start Vertical Cycle (W) with control as OSDK control interface is not properly initialized." << std::endl; // Logged
                      break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW_VERTICAL_CYCLE);
                 break;
             }
             case '2': { // Vertical Cycle (Y Base)
                 std::cout << "Starting Vertical Cycle (Y Base) using default bridge: " << defaultPythonBridgeScript << std::endl; // Logged
                 if (enableFlightControl && vehicle && vehicle->control) {
                     std::cout << "Attempting to obtain Control Authority for Vertical Cycle (Y)..." << std::endl; // Logged
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Vertical Cycle (Y) obtainCtrlAuthority"); // Logged via ACK
                         std::cerr << "Failed to obtain control authority. Cannot start Vertical Cycle (Y) with control." << std::endl; // Logged
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Vertical Cycle (Y)." << std::endl; // Logged
                     }
                 } else if (enableFlightControl) {
                      std::cerr << "Error: Cannot start Vertical Cycle (Y) with control as OSDK control interface is not properly initialized." << std::endl; // Logged
                      break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW_YAW_LOCK_VERTICAL_CYCLE);
                 break;
             }
             case '3': { // Vertical Cycle (B Base)
                 std::cout << "Starting Vertical Cycle (B Base) using default bridge: " << defaultPythonBridgeScript << std::endl; // Logged
                 if (enableFlightControl && vehicle && vehicle->control) {
                     std::cout << "Attempting to obtain Control Authority for Vertical Cycle (B)..." << std::endl; // Logged
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Vertical Cycle (B) obtainCtrlAuthority"); // Logged via ACK
                         std::cerr << "Failed to obtain control authority. Cannot start Vertical Cycle (B) with control." << std::endl; // Logged
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Vertical Cycle (B)." << std::endl; // Logged
                     }
                 } else if (enableFlightControl) {
                      std::cerr << "Error: Cannot start Vertical Cycle (B) with control as OSDK control interface is not properly initialized." << std::endl; // Logged
                      break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW_COMBINED_VERTICAL_CYCLE);
                 break;
             }
             case 'e': // Process Full
             case 'f': { // Process Minimal
                 ProcessingMode procMode = (inputChar == 'e') ? ProcessingMode::PROCESS_FULL : ProcessingMode::PROCESS_MINIMAL;
                 std::cout << "Starting Radar Data processing (No Control) using bridge: " << defaultPythonBridgeScript << std::endl; // Logged
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, procMode);
                 break;
             }

            case 'q': { // Quit
                std::cout << "Exiting..." << std::endl; // Logged
                // Stop threads handled above

                if (monitoringEnabled && vehicle && vehicle->subscribe) {
                    std::cout << "Unsubscribing from telemetry..." << std::endl; // Logged
                    ACK::ErrorCode statusAck = vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
                    if(ACK::getError(statusAck)) {
                        ACK::getErrorCodeMessage(statusAck, __func__); // Logged via ACK
                        std::cerr << "Warning: Failed to unsubscribe from telemetry package " << pkgIndex << "." << std::endl; // Logged
                    } else {
                        std::cout << "Telemetry unsubscribed." << std::endl; // Logged
                    }
                }

                if (enableFlightControl && vehicle && vehicle->control) {
                    std::cout << "Sending Zero Velocity command before final release..." << std::endl; // Logged
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
                    vehicle->control->flightCtrl(stopData);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    std::cout << "Releasing Control Authority on Quit..." << std::endl; // Logged
                    ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
                     if (ACK::getError(releaseAck)) {
                         ACK::getErrorCodeMessage(releaseAck, "Quit releaseCtrlAuthority"); // Logged via ACK
                         std::cerr << "Warning: Failed to release control authority on quit." << std::endl; // Logged
                     } else {
                         std::cout << "Control Authority Released on Quit." << std::endl; // Logged
                     }
                }
                 if (linuxEnvironment) {
                    delete linuxEnvironment; linuxEnvironment = nullptr;
                 }
                 if (flightSample) {
                    delete flightSample; flightSample = nullptr;
                 }
                 vehicle = nullptr;

                keepRunning = false;
                break;
            }
            case 0: break; // Enter key pressed
            default: std::cout << "Invalid input." << std::endl; break; // Logged
        }
    } // End main loop

    // Final check to join threads if loop exited unexpectedly
     if (processingThread.joinable()) {
         std::cerr << "Warning: Main loop exited unexpectedly, ensuring processing thread is stopped." << std::endl; // Logged
         stopProcessingFlag.store(true);
         try { processingThread.join(); } catch (const std::system_error& e) { std::cerr << "Error joining processing thread: " << e.what() << std::endl; } // Logged
     }
     if (monitoringThread.joinable()) {
         std::cerr << "Warning: Main loop exited unexpectedly, ensuring monitoring thread is stopped." << std::endl; // Logged
         stopMonitoringFlag.store(true);
          try { monitoringThread.join(); } catch (const std::system_error& e) { std::cerr << "Error joining monitoring thread: " << e.what() << std::endl; } // Logged
     }

      // Ensure linuxEnvironment is deleted if it exists, which should handle the vehicle pointer
      if (linuxEnvironment) {
         delete linuxEnvironment;
         linuxEnvironment = nullptr;
         vehicle = nullptr; // Pointer is now invalid
      }
      // Ensure flightSample is deleted if it exists and wasn't handled by linuxEnvironment deletion
      if (flightSample) {
          delete flightSample;
          flightSample = nullptr;
      }


    std::cout << "Program terminated." << std::endl; // Logged (will also go to file via LogRedirector dtor)
    return 0;
    // LogRedirector destructor runs here, restoring original streams and closing file.
}
