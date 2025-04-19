#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono> // Required for sleep_for
#include <cstdlib>  // For std::system
#include <json.hpp> // Include the JSON library
#include <boost/asio.hpp>    // Include Boost ASIO for TCP communication
#include "dji_linux_helpers.hpp"
#include <limits> // For numeric limits
#include <fstream> // For file reading AND LOGGING
#include <cmath> // For std::abs, std::isnan
#include <atomic> // For thread-safe stop flag
#include <cstring> // For strchr, strncpy
#include <stdexcept> // For standard exceptions
#include <array> // For std::array used in read_some buffer
#include <ctime>  // For checking polling timestamp
#include <streambuf> // For TeeBuf
#include <mutex>     // For TeeBuf thread safety
#include <memory>    // For unique_ptr
#include <iomanip>   // For std::put_time in timestamp, std::setw, std::left
#include <algorithm> // For std::transform

// Include the headers that define Control flags, CtrlData, FlightController, and Vehicle
#include "dji_control.hpp"           // Defines Control class, CtrlData, enums
#include "dji_flight_controller.hpp" // Defines FlightController
#include "dji_vehicle.hpp"           // Defines Vehicle class which contains Control*
#include "dji_telemetry.hpp"         // For Telemetry types
#include "dji_status.hpp"            // For VehicleStatus enums/constants
#include "dji_ack.hpp"               // For ACK::getError

// --- GUI Includes (Correct Order) ---
// 1. GLEW must come first
// #define GLEW_STATIC // Define this if linking GLEW statically, remove/comment if dynamic
#include <GL/glew.h>

// 2. SDL
#include <SDL.h>

// 3. SDL OpenGL headers (AFTER GLEW)
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

// 4. ImGui
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"


// Define M_PI if not already defined (likely in cmath)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace DJI::OSDK;
using json = nlohmann::json;
using boost::asio::ip::tcp;

// --- Logging Setup ---
// (TeeBuf and LogRedirector classes remain exactly the same as before)
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


// --- Configurable Parameters (with defaults relevant to W and E) ---
std::string TARGET_BEACON_ID = "BEACON-TX-ID:00005555";
float targetDistance = 8.0f;        // Target distance from the wall (meters) - Used by W
float targetAzimuth = 0.0f;         // *Initial* target azimuth relative to the beacon (degrees) - Used by W
// Forward Control (X-velocity) - Used by W
float Kp_forward = 0.5;             // Proportional gain for forward movement
float max_forward_speed = 0.8;      // Max speed towards/away from the wall
float forward_dead_zone = 0.2;      // Dead zone for forward movement (meters)
// Lateral Control (Y-velocity) - Used by W
float Kp_lateral = 0.02;            // Proportional gain for lateral movement
float max_lateral_speed = 0.5;      // Max speed sideways
float azimuth_dead_zone = 1.5;      // Dead zone for *initial* lateral movement (degrees)
// Bridge Reconnection Parameter
bool enable_bridge_reconnection = false; // Default to false
// NEW PARAMETER for Drone Connection
bool connect_to_drone = true;       // Default to true (attempt connection)
// --- End Configurable Parameters ---


// Persistent variables for tracking the current second and wall/beacon candidate data (relevant to W and E)
std::string currentSecond = "";
// General Wall/Beacon Data (Used by W)
float lowestRange = std::numeric_limits<float>::max(); // Closest wall candidate range (any sensor) - Used by 'w'
bool hasAnonData = false;                              // Flag if any anon data seen this second - Used by 'w'
float targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN(); // Sensed beacon azimuth - Used by 'w'
bool foundTargetBeacon = false;                        // Flag if target beacon seen this second - Used by 'w'
// REMOVED Specific Data for Yaw Lock, Combined, R_El Modes
// REMOVED Vertical Cycle State


// Global variable for default Python bridge script (Hardcoded)
std::string defaultPythonBridgeScript = "python_bridge.py";

// Flags to control the processing loop and reconnection
std::atomic<bool> stopProcessingFlag(false);
std::atomic<bool> forceStopReconnectionFlag(false); // Flag to manually stop reconnect attempts
std::thread processingThread;

// Monitoring Thread Globals
std::atomic<bool> stopMonitoringFlag(false);
std::thread monitoringThread;
const int TELEMETRY_TIMEOUT_SECONDS = 5; // Timeout for telemetry data
const int RECONNECT_DELAY_SECONDS = 5; // Delay between reconnection attempts (used in processing loop)

// --- NEW: Radar Bridge Connection Status ---
enum class BridgeConnectionStatus {
    DISCONNECTED,
    CONNECTED,
    RECONNECTING
};
std::atomic<BridgeConnectionStatus> currentBridgeStatus(BridgeConnectionStatus::DISCONNECTED); // Global atomic variable
// --- End NEW ---

// Function to load preferences (Simplified)
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
                } else if (key == "enable_bridge_reconnection") {
                    std::string lower_value = value;
                    std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
                    if (lower_value == "true" || lower_value == "1") {
                        enable_bridge_reconnection = true;
                    } else if (lower_value == "false" || lower_value == "0") {
                        enable_bridge_reconnection = false;
                    } else {
                        std::cerr << "Warning: Invalid boolean value for 'enable_bridge_reconnection': " << value << ". Using default (false)." << std::endl;
                        enable_bridge_reconnection = false; // Default
                    }
                    std::cout << "  enable_bridge_reconnection set to: " << std::boolalpha << enable_bridge_reconnection << " (from preferences file)" << std::endl; // Logged
                } else if (key == "connect_to_drone") { // NEW PARAMETER
                    std::string lower_value = value;
                    std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
                    if (lower_value == "true" || lower_value == "1") {
                        connect_to_drone = true;
                    } else if (lower_value == "false" || lower_value == "0") {
                        connect_to_drone = false;
                    } else {
                        std::cerr << "Warning: Invalid boolean value for 'connect_to_drone': " << value << ". Using default (true)." << std::endl;
                        connect_to_drone = true; // Default
                    }
                    std::cout << "  connect_to_drone set to: " << std::boolalpha << connect_to_drone << " (from preferences file)" << std::endl; // Logged
                }
                 else {
                     // Optionally log unrecognized keys
                     std::cout << "  Ignoring unrecognized key in preferences: " << key << std::endl; // Logged
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
        std::cout << "  Default enable_bridge_reconnection: " << std::boolalpha << enable_bridge_reconnection << std::endl; // Logged default
        std::cout << "  Default connect_to_drone: " << std::boolalpha << connect_to_drone << std::endl; // Logged default
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

// Display full radar object details (Needed for 'e' and debug in 'w')
void displayRadarObjects(const std::vector<RadarObject>& objects) {
    // Check if there are any objects to display to avoid printing header unnecessarily
    if (objects.empty()) return;

    std::cout << "--- Begin Radar Objects for Second: " << currentSecond << " ---" << std::endl; // Logged - Header
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
     std::cout << "--- End Radar Objects for Second: " << currentSecond << " ---" << std::endl; // Logged - Footer
}

// --- REMOVED displayRadarObjectsMinimal ---

// --- FUNCTION FOR [w] --- // ADDED FULL LOGGING & REVERSED LATERAL
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

                // --- ADDED FULL LOGGING ---
                // displayRadarObjects(objects); // Displaying *incoming* objects for the *new* second is confusing here. Better to log state below.
                // --- END ADDED FULL LOGGING ---

                if (enableControl && vehicle != nullptr && vehicle->control != nullptr) {
                    float velocity_x = 0.0f;
                    if (hasAnonData) { // Use overall closest anon range for 'w' mode
                        float difference = lowestRange - targetDistance; // Uses the CURRENT targetDistance
                        if (std::abs(difference) > forward_dead_zone) { // Uses the CURRENT forward_dead_zone
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * difference, max_forward_speed)); // Uses CURRENT Kp_forward, max_forward_speed
                        }
                    }
                    float velocity_y = 0.0f;
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) { // Use beacon azimuth for 'w' mode lateral control
                        float azimuth_error = targetBeaconAzimuth - targetAzimuth; // Uses the CURRENT targetAzimuth
                        if (std::abs(azimuth_error) > azimuth_dead_zone) { // Uses the CURRENT azimuth_dead_zone
                            // REVERSED LATERAL CALCULATION
                            velocity_y = std::max(-max_lateral_speed, std::min(-Kp_lateral * azimuth_error, max_lateral_speed)); // Uses CURRENT Kp_lateral, max_lateral_speed
                        }
                    } else if (!foundTargetBeacon) {
                        std::cout << "***BEACON NOT FOUND***" << std::endl; // Log beacon loss if relevant
                    }
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, 0, 0); // Z vel and Yaw rate are 0

                    // --- Mode-Specific LOGGING ('w') ---
                    std::cout << "[Wall Follow] Control Status (Second: " << currentSecond << "):\n" // Log which second this applies to
                              << "  TargetWall  = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n"
                              << "  CurrentWall = " << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                              << "  TargetBeaconAz=" << targetAzimuth << "\n" // Added Target Beacon Az
                              << "  CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "\n"
                              << "  Computed Velocity(X=" << velocity_x << ", Y=" << velocity_y << ")" << std::endl;
                    std::cout << "--------------------------------------" << std::endl; // Logged
                    // --- END Mode-Specific LOGGING ---

                    vehicle->control->flightCtrl(ctrlData);
                } else if (hasAnonData || foundTargetBeacon) { // Only log if there's relevant data when control disabled
                     if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl;} // Log beacon loss
                     // --- Mode-Specific LOGGING ('w' - No Control) ---
                     std::cout << "[Wall Follow] (Flight Control Disabled or Not Available) (Second: " << currentSecond << "):\n" // Log which second
                               << "  TargetWall  = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n"
                               << "  CurrentWall = " << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                               << "  TargetBeaconAz=" << targetAzimuth << "\n" // Added Target Beacon Az
                               << "  CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A")
                               << std::endl;
                     std::cout << "--------------------------------------" << std::endl; // Logged
                     // --- END Mode-Specific LOGGING ---
                }
            }
            // Reset state variables for the new second (Only those used by W)
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            // current_beacon_range = std::numeric_limits<float>::quiet_NaN(); // Not directly used by W control logic
            foundTargetBeacon = false;
            // REMOVED resets for Horiz/Rel data
        }

        // Accumulate general data for the current second (Needed by W)
        // Use the CURRENT TARGET_BEACON_ID for comparison
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) { // Only store the first one encountered in the second
                 targetBeaconAzimuth = obj.Az;
                 // current_beacon_range = obj.Range; // Not directly used by W control logic
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) lowestRange = obj.Range; // Track overall closest anon range

            // REMOVED accumulation for Horiz/Rel data
        }
         if (stopProcessingFlag.load()) return;
    }
}
// --- END FUNCTION FOR [w] --

// --- REMOVED OTHER extractBeaconAndWallData_* FUNCTIONS ---

// Parses JSON radar data (Keep)
std::vector<RadarObject> parseRadarData(const std::string& jsonData) {
    std::vector<RadarObject> radarObjects;
    if (jsonData.empty() || jsonData == "{}") return radarObjects;

    try {
        auto jsonFrame = json::parse(jsonData);
        if (!jsonFrame.contains("objects") || !jsonFrame["objects"].is_array()) return radarObjects;

        for (const auto& obj : jsonFrame["objects"]) {
            if (!obj.is_object()) {
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

// Runs the python bridge script (Keep)
void runPythonBridge(const std::string& scriptName) {
    std::cout << "Starting Python bridge (" << scriptName << ")..." << std::endl; // Logged
    if (std::system(("python3 " + scriptName + " &").c_str()) != 0) {
        std::cerr << "Failed to start Python bridge script '" << scriptName << "'." << std::endl; // Logged
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Python bridge potentially started." << std::endl; // Logged
}

// Stops the python bridge script (Keep)
void stopPythonBridge(const std::string& scriptName) {
    std::cout << "Stopping Python bridge (" << scriptName << ")..." << std::endl; // Logged
    std::system(("pkill -f " + scriptName).c_str()); // Ignore result
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Sent SIGTERM to " << scriptName << "." << std::endl; // Logged
}

// Connects to the python bridge via TCP (Modified for Reconnection Logic and Forced Stop)
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket) {
    tcp::resolver resolver(io_context);
    int initial_retries = 5; // Number of quick initial retries
    int current_retry = 0;
    const int initial_delay_seconds = 2;
    // const int persistent_delay_seconds = 5; // Delay now handled in processingLoopFunction

    // Reset force stop flag before attempting connection
    forceStopReconnectionFlag.store(false);

    // --- Initial Connection Attempts ---
    while (current_retry < initial_retries) {
        // Check both stop flags
        if (stopProcessingFlag.load() || forceStopReconnectionFlag.load()) {
             std::cout << "[Connect Bridge] Stop requested during initial connection attempts." << std::endl; // Logged
             forceStopReconnectionFlag.store(false); // Reset flag
             currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status on stop
             return false;
        }
        try {
            if(socket.is_open()) { socket.close(); } // Close previous attempt if any
            auto endpoints = resolver.resolve("127.0.0.1", "5000");
            boost::system::error_code ec;
            boost::asio::connect(socket, endpoints, ec);
            if (!ec) {
                std::cout << "Connected to Python bridge." << std::endl; // Logged
                currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED); // Update status on success
                return true; // Success!
            }
            else {
                std::cerr << "Initial connection attempt " << (current_retry + 1) << "/" << initial_retries << " failed: " << ec.message() << std::endl; // Logged
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception during initial connection attempt " << (current_retry + 1) << "/" << initial_retries << ": " << e.what() << std::endl; // Logged
        }

        current_retry++;
        // Check flags again before sleeping
        if (current_retry < initial_retries && !stopProcessingFlag.load() && !forceStopReconnectionFlag.load()) {
             std::cout << "Retrying connection in " << initial_delay_seconds << " seconds..." << std::endl; // Logged
             // Sleep in smaller chunks to check flags more often
             for (int i = 0; i < initial_delay_seconds * 10 && !stopProcessingFlag.load() && !forceStopReconnectionFlag.load(); ++i) {
                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
             }
        }
    } // End initial retries loop

    // --- Initial Attempts Failed ---
    std::cerr << "Failed initial connection attempts to Python bridge." << std::endl; // Logged

    // Check if stop was requested during initial attempts
    if (stopProcessingFlag.load() || forceStopReconnectionFlag.load()) {
        std::cout << "[Connect Bridge] Stop requested after initial attempts failed." << std::endl;
        forceStopReconnectionFlag.store(false); // Reset flag
        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status on stop
        return false;
    }

    // --- Persistent Reconnection Attempts (Try once immediately after initial failure) ---
    // Use the CURRENT value of enable_bridge_reconnection
    if (!enable_bridge_reconnection) {
        std::cout << "Reconnection disabled. Giving up." << std::endl; // Logged
        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status on give up
        return false; // Give up if reconnection is disabled
    }

    // Try one more time immediately after initial failures before returning (delay handled by caller)
    std::cout << "Attempting final connection before handing back to caller..." << std::endl;
    try {
        if(socket.is_open()) { socket.close(); }
        auto endpoints = resolver.resolve("127.0.0.1", "5000");
        boost::system::error_code ec;
        boost::asio::connect(socket, endpoints, ec);
        if (!ec) {
            std::cout << "Final connection attempt successful." << std::endl; // Logged
            currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED); // Update status on success
            return true; // Success!
        } else {
            std::cerr << "Final connection attempt failed: " << ec.message() << std::endl;
        }
    } catch (const std::exception& e) {
         std::cerr << "Exception during final connection attempt: " << e.what() << std::endl;
    }

    // If we reach here, all initial attempts + one final attempt failed
    std::cout << "[Connect Bridge] All connection attempts failed." << std::endl;
    forceStopReconnectionFlag.store(false); // Reset flag just in case
    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status on final failure
    return false; // Indicate connection failure
}


// Placeholder Callbacks (Keep, needed by OSDK headers)
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }
void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }

// Enum for processing modes (Simplified)
enum class ProcessingMode {
    WALL_FOLLOW,     // Mode for [w]
    PROCESS_FULL     // Mode for [e]
    // REMOVED OTHER MODES
};

// Processing loop function (Modified for Reconnection Logic with Delay)
void processingLoopFunction(const std::string bridgeScriptName, Vehicle* vehicle, bool enableControlCmd, ProcessingMode mode) {
    std::cout << "Processing thread started. Bridge: " << bridgeScriptName << ", Control Enabled: " << std::boolalpha << enableControlCmd << ", Mode: " << static_cast<int>(mode) << ", Reconnect Enabled: " << enable_bridge_reconnection << std::endl; // Logged (Added reconnect status)
    int functionTimeout = 1; // Timeout for OSDK control calls

    runPythonBridge(bridgeScriptName);
    boost::asio::io_context io_context;
    tcp::socket socket(io_context);

    // Initial connection attempt (will handle retries/persistence based on flag)
    currentBridgeStatus.store(BridgeConnectionStatus::RECONNECTING); // Set status before attempting connection
    if (!connectToPythonBridge(io_context, socket)) {
        // connectToPythonBridge now handles the forceStop flag internally and logs appropriately
        // It also sets the status to DISCONNECTED on failure/stop
        std::cerr << "Processing thread: Initial connection failed or was stopped. Exiting thread." << std::endl; // Logged (Simplified message)
        stopPythonBridge(bridgeScriptName);
        // Release control if it was meant to be used
        if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
            std::cout << "[Processing Thread] Releasing control authority (connection fail/stop)..." << std::endl; // Logged
            ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
            if (ACK::getError(releaseAck)) ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority"); // Logged via ACK
            else std::cout << "[Processing Thread] Control authority released." << std::endl; // Logged
        }
        // Status is already set to DISCONNECTED by connectToPythonBridge on failure
        return; // Exit thread
    }
    // If we reach here, connectToPythonBridge succeeded and set status to CONNECTED.
    std::cout << "Processing thread: Connection successful. Reading data stream..." << std::endl; // Logged
    std::string received_data_buffer;
    std::array<char, 4096> read_buffer;
    bool connection_error_occurred = false; // Track if we are in an error state

    // Reset state variables used by 'w' before the loop starts
    currentSecond = "";
    lowestRange = std::numeric_limits<float>::max();
    hasAnonData = false;
    targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
    foundTargetBeacon = false;
    // REMOVED reset for unused state variables

    while (!stopProcessingFlag.load()) {
        boost::system::error_code error;
        size_t len = socket.read_some(boost::asio::buffer(read_buffer), error);

        if (error) { // Handle read error
            if (error == boost::asio::error::eof) std::cerr << "Processing thread: Connection closed by Python bridge (EOF)." << std::endl; // Logged
            else std::cerr << "Processing thread: Error reading from socket: " << error.message() << std::endl; // Logged

            connection_error_occurred = true; // Mark error state

            // If control was active, send a stop command before attempting reconnect
            if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
                 std::cout << "[Processing Thread] Connection error. Sending zero velocity command..." << std::endl; // Logged
                 uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
                 DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
                 vehicle->control->flightCtrl(stopData); // Send stop command
            }

            // Use CURRENT value of enable_bridge_reconnection for decision
            if (enable_bridge_reconnection) {
                std::cout << "Connection lost. Attempting to reconnect..." << std::endl; // Logged
                currentBridgeStatus.store(BridgeConnectionStatus::RECONNECTING); // Set status to RECONNECTING
                if (socket.is_open()) socket.close(); // Ensure socket is closed before reconnecting

                // --- ADD DELAY HERE ---
                std::cout << "Waiting " << RECONNECT_DELAY_SECONDS << " seconds before next connection attempt..." << std::endl;
                for (int i = 0; i < RECONNECT_DELAY_SECONDS * 10 && !stopProcessingFlag.load() && !forceStopReconnectionFlag.load(); ++i) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                // Check if stop was requested during the delay
                if (stopProcessingFlag.load() || forceStopReconnectionFlag.load()) {
                    std::cout << "[Processing Thread] Stop requested during reconnection delay." << std::endl;
                    forceStopReconnectionFlag.store(false); // Reset flag
                    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Set status
                    break; // Exit the main while loop
                }
                // --- END DELAY ---

                // Attempt persistent reconnection (this will now respect forceStopReconnectionFlag and update status internally)
                if (connectToPythonBridge(io_context, socket)) {
                    std::cout << "Reconnection successful. Resuming data processing." << std::endl; // Logged
                    // connectToPythonBridge set status to CONNECTED
                    connection_error_occurred = false; // Clear error state
                    received_data_buffer.clear(); // Clear buffer as we might have partial data from before disconnect
                    continue; // Go back to the start of the while loop to try reading again
                } else {
                    // Connection failed OR was force stopped
                    // connectToPythonBridge set status to DISCONNECTED
                    std::cerr << "Processing thread: Persistent reconnection failed or was stopped. Exiting." << std::endl; // Logged
                    break; // Exit the main while loop
                }
            } else {
                std::cerr << "Reconnection disabled. Stopping processing." << std::endl; // Logged
                currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Set status
                break; // Exit the main while loop if reconnection is disabled
            }
        }

        // If no error, process data
        connection_error_occurred = false; // Clear error state if read was successful
        // Ensure status is CONNECTED if we are successfully reading data
        if (currentBridgeStatus.load() != BridgeConnectionStatus::CONNECTED) {
            currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED);
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

                        // Simplified Switch case
                        switch (mode) {
                            case ProcessingMode::WALL_FOLLOW:
                                extractBeaconAndWallData(radarObjects, vehicle, enableControlCmd); // Uses CURRENT global parameters
                                break;
                            case ProcessingMode::PROCESS_FULL:
                                // Call displayRadarObjects directly for mode 'e'
                                // Need to update currentSecond for the display header
                                if (!radarObjects.empty()) {
                                    std::string ts_cleaned = radarObjects[0].timestamp;
                                    if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
                                    if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
                                    size_t dotPos = ts_cleaned.find('.');
                                    std::string objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;
                                    if (objSecond != currentSecond) {
                                        currentSecond = objSecond; // Update global second tracker
                                    }
                                }
                                displayRadarObjects(radarObjects); // Direct call for this mode
                                break;
                            // REMOVED OTHER CASES
                        }

                    } catch (const std::exception& e) {
                         std::cerr << "Error processing data: " << e.what() << "\nSnippet: [" << jsonData.substr(0, 100) << "...]" << std::endl; // Logged
                    }
                }
            }
            if (stopProcessingFlag.load()) break;
        } else {
             // If len is 0 and no error, it might just be a brief pause in data.
             // Add a small sleep to prevent busy-waiting in such cases.
             std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } // End main data reading loop

    // --- Cleanup (Keep) ---
    if (stopProcessingFlag.load()) std::cout << "[Processing Thread] Stop requested manually." << std::endl; // Logged
    else if (connection_error_occurred) std::cout << "[Processing Thread] Exiting due to unrecoverable connection error or forced stop." << std::endl; // Logged (Updated message)
    else std::cout << "[Processing Thread] Data stream ended gracefully (or loop exited)." << std::endl; // Logged (Updated message)

    if (socket.is_open()) { socket.close(); }
    stopPythonBridge(bridgeScriptName);

    // Release Control Authority if enabled AND vehicle objects are valid
    if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
        std::cout << "[Processing Thread] Sending final zero velocity command..." << std::endl; // Logged
        uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
        DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
        vehicle->control->flightCtrl(stopData);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "[Processing Thread] Releasing control authority..." << std::endl; // Logged
        ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
        // Only log error if it's a real error, not just "not obtained"
        if (ACK::getError(releaseAck) && releaseAck.info.cmd_set != 0x00 && releaseAck.info.cmd_id != 0x00) {
             ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority (on exit)"); // Logged via ACK
             std::cerr << "[Processing Thread] Warning: Failed to release control authority on exit." << std::endl; // Logged
        } else {
             std::cout << "[Processing Thread] Control authority released (or was not held)." << std::endl; // Logged
        }
    }
    std::cout << "Processing thread finished." << std::endl; // Logged
    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Ensure status is DISCONNECTED when thread ends
}


// Helper function to get mode name string (Keep for monitoring)
std::string getModeName(uint8_t mode) {
    switch(mode) {
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL: return "MANUAL_CTRL";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE: return "ATTITUDE";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS: return "P_GPS";
        case DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL: return "NAVI_SDK_CTRL";
        case 31: return "Mode 31"; // Example of an unknown/other mode
        default: return "Other (" + std::to_string(mode) + ")";
    }
}

// Background thread function for monitoring (Keep, generally useful)
void monitoringLoopFunction(Vehicle* vehicle) {
    // Ensure vehicle pointer is valid before starting loop
    if (vehicle == nullptr || vehicle->subscribe == nullptr) {
        std::cerr << "[Monitoring] Error: Invalid Vehicle object provided. Thread exiting." << std::endl;
        return;
    }

    std::cout << "[Monitoring] Thread started." << std::endl; // Logged
    bool telemetry_timed_out = false;
    bool warned_unexpected_status = false;
    uint8_t previous_flight_status = DJI::OSDK::VehicleStatus::FlightStatus::STOPED;
    time_t last_valid_poll_time = 0;
    bool in_sdk_control_mode = false;
    bool warned_not_in_sdk_mode = false;
    const uint8_t EXPECTED_SDK_MODE = DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL;

    while (!stopMonitoringFlag.load()) {
        // Re-check pointers inside loop in case they become invalid (though unlikely with current structure)
        if (vehicle == nullptr || vehicle->subscribe == nullptr) {
             if (!telemetry_timed_out) { // Avoid spamming log
                 std::cerr << "\n**** MONITORING ERROR: Vehicle/subscribe object became null. Stopping. ****" << std::endl << std::endl; // Logged
                 telemetry_timed_out = true;
             }
             break; // Exit if vehicle objects become invalid
        }

        // Read telemetry data
        uint8_t current_flight_status = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
        uint8_t current_display_mode = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        // float current_height = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_HEIGHT_FUSION>(); // AGL data available if needed
        bool valid_poll = (current_flight_status <= DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR); // Basic validity check


        if (valid_poll) {
            time_t current_time = std::time(nullptr);
            last_valid_poll_time = current_time;
            if (telemetry_timed_out) {
                 std::cout << "[Monitoring] Telemetry poll recovered." << std::endl; // Logged
                 telemetry_timed_out = false;
            }

            // Check Flight Status Change (e.g., unexpected landing)
            if (current_flight_status != DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) {
                if ( (previous_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
                     (current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND || current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::STOPED) &&
                     !warned_unexpected_status) {
                     std::cerr << "\n**** MONITORING WARNING: Flight status changed unexpectedly from IN_AIR to " << (int)current_flight_status << ". ****" << std::endl << std::endl; // Logged
                     warned_unexpected_status = true;
                }
            } else {
                 warned_unexpected_status = false; // Reset once back in air
            }
            previous_flight_status = current_flight_status;

            // Check Expected SDK Mode (only warn if control is supposed to be active)
            bool is_expected_mode = (current_display_mode == EXPECTED_SDK_MODE);
            if (is_expected_mode) {
                if (!in_sdk_control_mode) {
                    std::cout << "\n**** MONITORING INFO: Entered SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << " / " << (int)EXPECTED_SDK_MODE << ") ****" << std::endl << std::endl; // Logged
                    in_sdk_control_mode = true;
                    warned_not_in_sdk_mode = false;
                }
            } else { // Not in expected mode
                 // Only warn if we were previously in SDK mode or haven't warned yet, AND the processing thread is active (implying control *should* be active)
                 if ((in_sdk_control_mode || !warned_not_in_sdk_mode) && !stopProcessingFlag.load() && processingThread.joinable()) {
                      std::string current_mode_name = getModeName(current_display_mode);
                      std::cerr << "\n**** MONITORING WARNING: NOT in expected SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << "). Current: " << current_mode_name << " (" << (int)current_display_mode << ") ****" << std::endl << std::endl; // Logged
                      warned_not_in_sdk_mode = true;
                 }
                 in_sdk_control_mode = false;
            }
        } else { // Invalid poll data received
            if (!telemetry_timed_out) { // Avoid spamming log
                 std::cerr << "\n**** MONITORING WARNING: Polling telemetry returned potentially invalid data (FlightStatus=" << (int)current_flight_status << "). ****" << std::endl << std::endl; // Logged
                 telemetry_timed_out = true;
            }
        }

        // Check Telemetry Timeout
        time_t current_time_for_timeout_check = std::time(nullptr);
        if (last_valid_poll_time > 0 && (current_time_for_timeout_check - last_valid_poll_time > TELEMETRY_TIMEOUT_SECONDS)) {
            if (!telemetry_timed_out) { // Avoid spamming log
                std::cerr << "\n**** MONITORING TIMEOUT: No valid telemetry for over " << TELEMETRY_TIMEOUT_SECONDS << " seconds. ****" << std::endl << std::endl; // Logged
                telemetry_timed_out = true;
            }
        } else if (last_valid_poll_time == 0) { // Check if never received first poll
             // Use a static variable to track start time only once
             static time_t start_time = 0; if (start_time == 0) start_time = current_time_for_timeout_check;
             if (current_time_for_timeout_check - start_time > TELEMETRY_TIMEOUT_SECONDS * 2) { // Allow double timeout initially
                  if (!telemetry_timed_out) { // Avoid spamming log
                       std::cerr << "\n**** MONITORING TIMEOUT: Never received valid telemetry poll after " << TELEMETRY_TIMEOUT_SECONDS * 2 << " seconds. ****" << std::endl << std::endl; // Logged
                       telemetry_timed_out = true;
                  }
             }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1)); // Poll frequency
    }
    std::cout << "[Monitoring] Thread finished." << std::endl; // Logged
}


// Helper function to stop the processing thread if it's running
void stopProcessingThreadIfNeeded() {
    if (processingThread.joinable()) {
        std::cout << "[GUI] Signalling processing thread to stop..." << std::endl; // Logged
        stopProcessingFlag.store(true); // Signal normal stop first
        forceStopReconnectionFlag.store(true); // Also signal forced stop for reconnection loop
        processingThread.join();
        std::cout << "[GUI] Processing thread finished." << std::endl; // Logged
        stopProcessingFlag.store(false); // Reset flag for next potential thread start
        forceStopReconnectionFlag.store(false); // Reset flag
        // Status is set to DISCONNECTED inside the thread function upon exit
    } else {
         // Ensure status is DISCONNECTED if no thread is running
         currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
    }
}


int main(int argc, char** argv) {
    // --- Instantiate LogRedirector FIRST ---
    LogRedirector logger("run_log.txt");
    std::cout << "[Main] LogRedirector instantiated." << std::endl;

    std::cout << "Starting application: " << (argc > 0 ? argv[0] : "djiosdk-flightcontrol-gui") << std::endl; // Logged (Updated name)

    std::cout << "[Main] Calling loadPreferences()..." << std::endl;
    loadPreferences(); // Load preferences next (including connect_to_drone)
    std::cout << "[Main] Returned from loadPreferences()." << std::endl;

    // --- OSDK Initialization (Now Conditional) ---
    bool enableFlightControl = false; // Default to false, set true only if connection succeeds
    int functionTimeout = 1;
    Vehicle* vehicle = nullptr;
    LinuxSetup* linuxEnvironment = nullptr;
    int telemetrySubscriptionFrequency = 10;
    int pkgIndex = 0;
    bool monitoringEnabled = false;

    if (connect_to_drone) {
        std::cout << "[Main] Attempting to initialize DJI OSDK (connect_to_drone is true)..." << std::endl;
        linuxEnvironment = new LinuxSetup(argc, argv);
        vehicle = linuxEnvironment->getVehicle();

        if (vehicle == nullptr || vehicle->control == nullptr || vehicle->subscribe == nullptr) {
            std::cerr << "ERROR: Vehicle not initialized or interfaces unavailable. Flight control disabled." << std::endl;
            if (linuxEnvironment) { delete linuxEnvironment; linuxEnvironment = nullptr; }
            vehicle = nullptr;
            // enableFlightControl remains false
        } else {
            std::cout << "[Main] OSDK Vehicle instance OK." << std::endl;
            enableFlightControl = true; // Connection successful, enable flight control flag

            // Setup Telemetry Subscription for Monitoring
            std::cout << "Setting up Telemetry Subscription for Monitoring..." << std::endl;
            ACK::ErrorCode subscribeAck = vehicle->subscribe->verify(functionTimeout);
            if (ACK::getError(subscribeAck)) {
                 ACK::getErrorCodeMessage(subscribeAck, __func__);
                 std::cerr << "Error verifying subscription package list. Monitoring will be disabled." << std::endl;
            } else {
                 Telemetry::TopicName topicList[] = {
                     Telemetry::TOPIC_STATUS_FLIGHT,
                     Telemetry::TOPIC_STATUS_DISPLAYMODE,
                 };
                 int numTopic = sizeof(topicList) / sizeof(topicList[0]);
                 bool topicStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList, false, telemetrySubscriptionFrequency);

                 if (topicStatus) {
                       std::cout << "Successfully initialized telemetry package " << pkgIndex << "." << std::endl;
                       ACK::ErrorCode startAck = vehicle->subscribe->startPackage(pkgIndex, functionTimeout);
                       if (ACK::getError(startAck)) {
                            ACK::getErrorCodeMessage(startAck, "startPackage");
                            std::cerr << "Error starting subscription package " << pkgIndex << ". Monitoring disabled." << std::endl;
                            vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
                       } else {
                            std::cout << "Successfully started telemetry package " << pkgIndex << "." << std::endl;
                            std::cout << "Starting monitoring thread..." << std::endl;
                            stopMonitoringFlag.store(false);
                            monitoringThread = std::thread(monitoringLoopFunction, vehicle);
                            monitoringEnabled = true;
                       }
                 } else {
                      std::cerr << "Error initializing telemetry package " << pkgIndex << ". Monitoring disabled." << std::endl;
                 }
            }
        }
    } else {
        std::cout << "[Main] Skipping OSDK Initialization (connect_to_drone is false). Flight control disabled." << std::endl;
        // vehicle remains nullptr, enableFlightControl remains false
    }

    std::cout << "INFO: Flight control is " << (enableFlightControl ? "ENABLED" : "DISABLED")
              << ". Bridge Reconnection: " << std::boolalpha << enable_bridge_reconnection
              << ". Connect to Drone: " << connect_to_drone << "." << std::endl; // Added connect_to_drone status

    // --- GUI Initialization ---
    std::cout << "[GUI] Initializing SDL..." << std::endl;
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        // Cleanup OSDK if partially initialized
        if (linuxEnvironment) delete linuxEnvironment;
        return 1;
    }

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#elif defined(__APPLE__)
    // GL 3.2 Core + GLSL 150
    const char* glsl_version = "#version 150";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif

    // Create window with graphics context
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("DJI OSDK Control GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
    if (!window) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
         if (linuxEnvironment) delete linuxEnvironment;
        return 1;
    }
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) {
        std::cerr << "SDL_GL_CreateContext Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
         if (linuxEnvironment) delete linuxEnvironment;
        return 1;
    }
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Initialize OpenGL loader (GLEW)
    std::cout << "[GUI] Initializing OpenGL Loader (GLEW)..." << std::endl;
    glewExperimental = GL_TRUE; // Needed for core profile
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW: " << glewGetErrorString(err) << std::endl;
        SDL_GL_DeleteContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();
         if (linuxEnvironment) delete linuxEnvironment;
        return 1;
    }
    std::cout << "[GUI] Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    std::cout << "[GUI] OpenGL Version: " << glGetString(GL_VERSION) << std::endl; // Check version after loader init


    // Setup Dear ImGui context
    std::cout << "[GUI] Initializing ImGui..." << std::endl;
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context); // CORRECTED FUNCTION CALL
    ImGui_ImplOpenGL3_Init(glsl_version);

    // GUI state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    bool guiKeepRunning = true;

    // Buffer for Target Beacon ID Input
    char target_beacon_id_buffer[128]; // Adjust size as needed
    strncpy(target_beacon_id_buffer, TARGET_BEACON_ID.c_str(), sizeof(target_beacon_id_buffer) - 1);
    target_beacon_id_buffer[sizeof(target_beacon_id_buffer) - 1] = '\0'; // Ensure null termination

    std::cout << "[GUI] Entering main GUI loop..." << std::endl;
    // --- Main GUI Loop ---
    while (guiKeepRunning) {
        // Poll and handle events (inputs, window resize, etc.)
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event); // CORRECTED FUNCTION CALL
            if (event.type == SDL_QUIT)
                guiKeepRunning = false;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
                guiKeepRunning = false;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(); // CORRECTED FUNCTION CALL
        ImGui::NewFrame();

        // --- GUI Windows ---

        // --- NEW: Top Center Status Indicator ---
        {
            ImGuiViewport* viewport = ImGui::GetMainViewport();
            ImVec2 work_pos = viewport->WorkPos; // Top-left position of the viewport (usually 0,0)
            ImVec2 work_size = viewport->WorkSize; // Size of the viewport

            // Define colors for status
            const ImVec4 green = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
            const ImVec4 red = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
            const ImVec4 orange = ImVec4(1.0f, 0.65f, 0.0f, 1.0f);

            // Get current status and determine text/color
            BridgeConnectionStatus status = currentBridgeStatus.load();
            const char* statusText = "";
            ImVec4 statusColor = red; // Default to red

            switch(status) {
                case BridgeConnectionStatus::CONNECTED:
                    statusText = "Radar Bridge: Connected";
                    statusColor = green;
                    break;
                case BridgeConnectionStatus::DISCONNECTED:
                    statusText = "Radar Bridge: Disconnected";
                    statusColor = red;
                    break;
                case BridgeConnectionStatus::RECONNECTING:
                    statusText = "Radar Bridge: Attempting Reconnect...";
                    statusColor = orange;
                    break;
            }

            // Calculate text size to center it
            ImVec2 textSize = ImGui::CalcTextSize(statusText);

            // Set position for the status window (top center)
            ImVec2 windowPos = ImVec2(work_pos.x + (work_size.x - textSize.x) * 0.5f, work_pos.y + 10.0f); // 10 pixels from top
            ImGui::SetNextWindowPos(windowPos);
            ImGui::SetNextWindowBgAlpha(0.0f); // Make background transparent

            // Create a borderless, title-less, non-movable window
            ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoDecoration |
                                           ImGuiWindowFlags_AlwaysAutoResize |
                                           ImGuiWindowFlags_NoSavedSettings |
                                           ImGuiWindowFlags_NoFocusOnAppearing |
                                           ImGuiWindowFlags_NoNav |
                                           ImGuiWindowFlags_NoMove |
                                           ImGuiWindowFlags_NoBackground; // Explicitly no background

            ImGui::Begin("StatusIndicator", nullptr, windowFlags);
            ImGui::TextColored(statusColor, "%s", statusText);
            ImGui::End();
        }
        // --- END NEW ---


        // 1. Main Menu Window (Top Left)
        ImGui::Begin("Main Menu");
        {
            // Wall Following Button
            // Disable if flight control flag is false (set during init based on connect_to_drone and success)
            if (!enableFlightControl) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Start Wall+Beacon Following [w]")) {
                std::cout << "[GUI] 'w' button clicked." << std::endl;
                stopProcessingThreadIfNeeded(); // Stop previous task first
                forceStopReconnectionFlag.store(false); // Ensure flag is reset before starting

                // Double-check vehicle and control are valid before obtaining authority
                if (enableFlightControl && vehicle != nullptr && vehicle->control != nullptr) {
                    std::cout << "[GUI] Attempting to obtain Control Authority for Wall Following..." << std::endl;
                    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                    if (ACK::getError(ctrlAuthAck)) {
                        ACK::getErrorCodeMessage(ctrlAuthAck, "[GUI] Wall Following obtainCtrlAuthority");
                        std::cerr << "[GUI] Failed to obtain control authority. Cannot start 'w' with control." << std::endl;
                        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Ensure status is correct if thread doesn't start
                        // Optionally show an ImGui popup here
                    } else {
                        std::cout << "[GUI] Obtained Control Authority for Wall Following." << std::endl;
                        stopProcessingFlag.store(false); // Ensure this is reset too
                        // Status will be set inside processingLoopFunction
                        processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, true, ProcessingMode::WALL_FOLLOW);
                    }
                } else {
                     // This should only happen if enableFlightControl was somehow true but vehicle pointers are bad (unlikely now)
                     // Or if the button wasn't properly disabled.
                     std::cerr << "[GUI] Cannot start 'w' with control: Flight control disabled or OSDK not ready." << std::endl;
                     currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Ensure status is correct
                }
            }
            if (!enableFlightControl) {
                ImGui::EndDisabled();
                ImGui::SameLine(); // Add tooltip or text next to disabled button
                ImGui::TextDisabled("(Flight Control Disabled)");
            }

            // Process Full Radar Button
            if (ImGui::Button("Process Full Radar [e]")) {
                 std::cout << "[GUI] 'e' button clicked." << std::endl;
                 stopProcessingThreadIfNeeded(); // Stop previous task first
                 forceStopReconnectionFlag.store(false); // Ensure flag is reset before starting
                 std::cout << "[GUI] Starting Radar Data processing (Full, No Control)..." << std::endl;
                 stopProcessingFlag.store(false); // Ensure this is reset too
                 // Status will be set inside processingLoopFunction
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, ProcessingMode::PROCESS_FULL);
            }

            // NEW: Stop Reconnection Button
            // Show only if reconnection is enabled AND a processing thread might be active
            if (enable_bridge_reconnection) {
                // Only enable if a thread is running AND the current status is RECONNECTING
                bool should_enable_stop_reconnect = processingThread.joinable() && (currentBridgeStatus.load() == BridgeConnectionStatus::RECONNECTING);

                if (!should_enable_stop_reconnect) {
                    ImGui::BeginDisabled(); // Disable if no thread is running or not in reconnecting state
                }
                if (ImGui::Button("Stop Reconnection Attempt")) {
                    std::cout << "[GUI] 'Stop Reconnection Attempt' button clicked." << std::endl;
                    forceStopReconnectionFlag.store(true); // Signal the reconnection loop to stop
                    // The processing thread will update the status to DISCONNECTED when it stops
                }
                if (!should_enable_stop_reconnect) {
                    ImGui::EndDisabled();
                }
            }


            // Quit Button
            if (ImGui::Button("Quit [q]")) {
                std::cout << "[GUI] 'q' button clicked." << std::endl;
                guiKeepRunning = false; // Signal GUI loop to exit
            }
        }
        ImGui::End();


        // 2. Parameters Window (Top Right - Now with Input Fields and Logging)
        {
            ImGuiIO& current_io = ImGui::GetIO();
            ImVec2 window_pos = ImVec2(current_io.DisplaySize.x - 10.0f, 10.0f); // Position from top-right
            ImVec2 window_pos_pivot = ImVec2(1.0f, 0.0f); // Pivot at top-right
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Appearing, window_pos_pivot); // Use Appearing condition
            ImGui::SetNextWindowBgAlpha(0.65f); // Slightly less transparent
            ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_AlwaysAutoResize; // Allow move/resize, auto-resize width

            ImGui::Begin("Parameters", nullptr, window_flags);
            ImGui::Text("Status");
            ImGui::Separator();
            ImGui::Text("Connect to Drone: %s", connect_to_drone ? "True" : "False"); // Not editable runtime
            ImGui::Text("Flight Control: %s", enableFlightControl ? "Enabled" : "Disabled"); // Status display
            ImGui::Separator();
            ImGui::Text("Editable Parameters");
            ImGui::Separator();

            // --- Editable Parameters ---
            // Target Beacon ID
            if (ImGui::InputText("Target Beacon ID", target_beacon_id_buffer, sizeof(target_beacon_id_buffer))) {
                TARGET_BEACON_ID = target_beacon_id_buffer; // Update the global std::string
                std::cout << "[GUI Param Update] Target Beacon ID set to: " << TARGET_BEACON_ID << std::endl; // Log change
            }

            // Target Distance
            if (ImGui::InputFloat("Target Distance (m)", &targetDistance, 0.1f, 1.0f, "%.3f")) {
                 std::cout << "[GUI Param Update] Target Distance set to: " << targetDistance << std::endl; // Log change
            }

            // Target Azimuth
            if (ImGui::InputFloat("Target Azimuth (deg)", &targetAzimuth, 1.0f, 10.0f, "%.3f")) {
                 std::cout << "[GUI Param Update] Target Azimuth set to: " << targetAzimuth << std::endl; // Log change
            }

            ImGui::Separator();
            ImGui::Text("Forward Control:");
            // Kp Forward
            if (ImGui::InputFloat("Kp Forward", &Kp_forward, 0.01f, 0.1f, "%.4f")) {
                 std::cout << "[GUI Param Update] Kp Forward set to: " << Kp_forward << std::endl; // Log change
            }

            // Max Fwd Speed
            if (ImGui::InputFloat("Max Fwd Speed (m/s)", &max_forward_speed, 0.05f, 0.2f, "%.3f")) {
                 std::cout << "[GUI Param Update] Max Fwd Speed set to: " << max_forward_speed << std::endl; // Log change
            }

            // Fwd Dead Zone
            if (ImGui::InputFloat("Fwd Dead Zone (m)", &forward_dead_zone, 0.01f, 0.1f, "%.3f")) {
                 std::cout << "[GUI Param Update] Fwd Dead Zone set to: " << forward_dead_zone << std::endl; // Log change
            }

            ImGui::Separator();
            ImGui::Text("Lateral Control:");
             // Kp Lateral
            if (ImGui::InputFloat("Kp Lateral", &Kp_lateral, 0.001f, 0.01f, "%.5f")) {
                 std::cout << "[GUI Param Update] Kp Lateral set to: " << Kp_lateral << std::endl; // Log change
            }

            // Max Lat Speed
            if (ImGui::InputFloat("Max Lat Speed (m/s)", &max_lateral_speed, 0.05f, 0.2f, "%.3f")) {
                 std::cout << "[GUI Param Update] Max Lat Speed set to: " << max_lateral_speed << std::endl; // Log change
            }

            // Azimuth Dead Zone
            if (ImGui::InputFloat("Azimuth Dead Zone (deg)", &azimuth_dead_zone, 0.1f, 1.0f, "%.3f")) {
                 std::cout << "[GUI Param Update] Azimuth Dead Zone set to: " << azimuth_dead_zone << std::endl; // Log change
            }

            ImGui::Separator();
            ImGui::Text("Connection:");
            // Bridge Reconnect
            if (ImGui::Checkbox("Bridge Reconnect", &enable_bridge_reconnection)) {
                 std::cout << "[GUI Param Update] Bridge Reconnect set to: " << std::boolalpha << enable_bridge_reconnection << std::endl; // Log change
            }

            ImGui::End();
        }

        // --- Rendering ---
        ImGui::Render();
        int display_w, display_h;
        SDL_GetWindowSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    } // End GUI loop


    // --- Cleanup ---
    std::cout << "[GUI] Exited main GUI loop. Cleaning up..." << std::endl;

    // Stop threads cleanly
    stopProcessingThreadIfNeeded(); // Ensure processing thread is stopped

    if (monitoringThread.joinable()) {
        std::cout << "[GUI] Signalling monitoring thread to stop..." << std::endl;
        stopMonitoringFlag.store(true);
        monitoringThread.join();
        std::cout << "[GUI] Monitoring thread finished." << std::endl;
        // monitoringEnabled = false; // Flag not strictly needed anymore
    }

    // OSDK Cleanup (Conditional)
    if (connect_to_drone && vehicle != nullptr) { // Only cleanup if connection was attempted and vehicle exists
        if (monitoringEnabled && vehicle->subscribe != nullptr) { // Check subscribe pointer too
            std::cout << "[GUI] Unsubscribing from telemetry..." << std::endl;
            ACK::ErrorCode statusAck = vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
            if(ACK::getError(statusAck)) {
                ACK::getErrorCodeMessage(statusAck, "[GUI] Cleanup");
            } else {
                std::cout << "[GUI] Telemetry unsubscribed." << std::endl;
            }
        }
        if (enableFlightControl && vehicle->control != nullptr) { // Check control pointer too
            std::cout << "[GUI] Releasing Control Authority on Quit (if obtained)..." << std::endl;
            ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
             if (ACK::getError(releaseAck) && releaseAck.info.cmd_set != 0x00 && releaseAck.info.cmd_id != 0x00) {
                 ACK::getErrorCodeMessage(releaseAck, "[GUI] Quit releaseCtrlAuthority");
             } else {
                 std::cout << "[GUI] Control Authority Released on Quit (or was not held)." << std::endl;
             }
        }
    }
    // Delete linuxEnvironment only if it was created
    if (linuxEnvironment != nullptr) {
        std::cout << "[GUI] Deleting linuxEnvironment..." << std::endl;
        delete linuxEnvironment; linuxEnvironment = nullptr;
        vehicle = nullptr; // Pointer now invalid
    }

    // ImGui Cleanup
    std::cout << "[GUI] Cleaning up ImGui..." << std::endl;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown(); // CORRECTED FUNCTION CALL
    ImGui::DestroyContext();

    // SDL Cleanup
    std::cout << "[GUI] Cleaning up SDL..." << std::endl;
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::cout << "Application finished with exit code: 0" << std::endl;
    return 0;
    // LogRedirector destructor runs here
}
