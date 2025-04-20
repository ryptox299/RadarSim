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
#include <atomic> // For thread-safe stop flag, connection status
#include <cstring> // For strchr, strncpy
#include <stdexcept> // For standard exceptions
#include <array> // For std::array used in read_some buffer
#include <ctime>  // For checking polling timestamp
#include <streambuf> // For TeeBuf
#include <mutex>     // For TeeBuf thread safety and OSDK state
#include <memory>    // For unique_ptr
#include <iomanip>   // For std::put_time in timestamp, std::setw, std::left
#include <algorithm> // For std::transform
#include <future>    // For std::async, std::future (alternative to std::thread if preferred, but thread is fine)

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
// TeeBuf writes output to two streambufs (e.g., console and file)
class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* sb1, std::streambuf* sb2) : sb1_(sb1), sb2_(sb2) {}

protected:
    // Called when buffer is full or on explicit flush/endl
    virtual int sync() override {
        std::lock_guard<std::mutex> lock(mutex_); // Use internal mutex
        int r1 = sb1_->pubsync();
        int r2 = sb2_->pubsync();
        return (r1 == 0 && r2 == 0) ? 0 : -1;
    }

    // Called when a character is written
    virtual int_type overflow(int_type c = traits_type::eof()) override {
        if (traits_type::eq_int_type(c, traits_type::eof())) {
            return sync() == -1 ? traits_type::eof() : traits_type::not_eof(c);
        }

        std::lock_guard<std::mutex> lock(mutex_); // Use internal mutex
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
    std::mutex mutex_; // Mutex specific to TeeBuf
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
             if (original_cerr_buf_) std::cerr.rdbuf(original_cerr_buf_); // Temporarily restore if needed for error message
            std::cerr << "FATAL ERROR: Could not open log file: " << log_filename << std::endl;
             if (original_cerr_buf_) std::cerr.rdbuf(original_cerr_buf_); // Restore again just in case
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


// --- Configurable Parameters ---
std::string TARGET_BEACON_ID = "BEACON-TX-ID:00005555";
float targetDistance = 8.0f;
float targetAzimuth = 0.0f;
float Kp_forward = 0.5;
float max_forward_speed = 0.8;
float forward_dead_zone = 0.2;
float Kp_lateral = 0.02;
float max_lateral_speed = 0.5;
float azimuth_dead_zone = 1.5;
bool enable_bridge_reconnection = false;
bool connect_to_drone = true; // This is the variable the checkbox will bind to (desired state)
// --- End Configurable Parameters ---


// --- Persistent Radar Data Tracking ---
std::string currentSecond = "";
float lowestRange = std::numeric_limits<float>::max();
bool hasAnonData = false;
float targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
bool foundTargetBeacon = false;
// --- End Radar Data Tracking ---


// --- Global Script Name ---
std::string defaultPythonBridgeScript = "python_bridge.py";


// --- Threading and Control Flags ---
std::atomic<bool> stopProcessingFlag(false);
std::atomic<bool> forceStopReconnectionFlag(false);
std::thread processingThread;
std::atomic<bool> stopMonitoringFlag(false);
std::thread monitoringThread;
const int TELEMETRY_TIMEOUT_SECONDS = 5;
const int RECONNECT_DELAY_SECONDS = 5;

// --- NEW Connection Threading Variables ---
std::thread connectionThread;               // Thread object for the connection task
std::atomic<bool> isConnectingToDrone(false);   // Flag: True while connection thread is running
std::atomic<bool> connectionAttemptFinished(false); // Flag: True when connection thread completes
std::atomic<bool> connectionResult(false);       // Stores the success/failure result from the thread

// --- Radar Bridge Connection Status ---
enum class BridgeConnectionStatus { DISCONNECTED, CONNECTED, RECONNECTING };
std::atomic<BridgeConnectionStatus> currentBridgeStatus(BridgeConnectionStatus::DISCONNECTED);


// --- Global OSDK State ---
std::mutex osdkMutex; // Mutex to protect OSDK object creation/destruction and status flags
Vehicle* vehicle = nullptr;
LinuxSetup* linuxEnvironment = nullptr;
bool enableFlightControl = false; // Reflects ACTUAL OSDK connection status, managed by init/deinit
bool monitoringEnabled = false;
const int telemetrySubscriptionFrequency = 10;
int pkgIndex = 0; // Package index for telemetry


// --- Moved Definitions BEFORE Forward Declarations ---

// Struct for Radar Data
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

// Enum for processing modes
enum class ProcessingMode {
    WALL_FOLLOW,     // Mode for [w]
    PROCESS_FULL     // Mode for [e]
};


// --- Forward Declarations ---
bool initializeOSDK(int argc, char** argv);
void deinitializeOSDK();
void monitoringLoopFunction(Vehicle* vehiclePtr); // Takes pointer argument
void processingLoopFunction(const std::string bridgeScriptName, Vehicle* vehiclePtr, bool enableControlCmd, ProcessingMode mode); // Takes pointer argument
void stopProcessingThreadIfNeeded();
void loadPreferences();
std::vector<RadarObject> parseRadarData(const std::string& jsonData);
void extractBeaconAndWallData(const std::vector<RadarObject>& objects, Vehicle* vehiclePtr, bool enableControl); // Takes pointer argument
void displayRadarObjects(const std::vector<RadarObject>& objects);
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket);
void runPythonBridge(const std::string& scriptName);
void stopPythonBridge(const std::string& scriptName);
std::string getModeName(uint8_t mode);
void connectionTask(int argc, char** argv); // NEW: Function for the connection thread


// --- Function Implementations ---

// Function to load preferences from "preferences.txt"
void loadPreferences() {
    std::cout << "Loading preferences..." << std::endl;
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
                std::cerr << "Warning: Skipping invalid line in preferences file: " << line << std::endl; continue;
            }

            std::string key = line.substr(0, equalsPos);
            std::string value = line.substr(equalsPos + 1);

            try {
                if (key == "target_beacon_id") { TARGET_BEACON_ID = value; std::cout << "  TARGET_BEACON_ID set to: " << TARGET_BEACON_ID << " (from preferences file)" << std::endl; }
                else if (key == "targetdistance") { targetDistance = std::stof(value); std::cout << "  targetDistance set to: " << targetDistance << " meters (from preferences file)" << std::endl; }
                else if (key == "target_azimuth") { targetAzimuth = std::stof(value); std::cout << "  targetAzimuth (Initial) set to: " << targetAzimuth << " degrees (from preferences file)" << std::endl; }
                else if (key == "kp_forward") { Kp_forward = std::stof(value); std::cout << "  Kp_forward set to: " << Kp_forward << " (from preferences file)" << std::endl; }
                else if (key == "max_forward_speed") { max_forward_speed = std::stof(value); std::cout << "  max_forward_speed set to: " << max_forward_speed << " m/s (from preferences file)" << std::endl; }
                else if (key == "forward_dead_zone") { forward_dead_zone = std::stof(value); std::cout << "  forward_dead_zone set to: " << forward_dead_zone << " meters (from preferences file)" << std::endl; }
                else if (key == "kp_lateral") { Kp_lateral = std::stof(value); std::cout << "  Kp_lateral set to: " << Kp_lateral << " (from preferences file)" << std::endl; }
                else if (key == "max_lateral_speed") { max_lateral_speed = std::stof(value); std::cout << "  max_lateral_speed set to: " << max_lateral_speed << " m/s (from preferences file)" << std::endl; }
                else if (key == "azimuth_dead_zone") { azimuth_dead_zone = std::stof(value); std::cout << "  azimuth_dead_zone set to: " << azimuth_dead_zone << " degrees (from preferences file)" << std::endl; }
                else if (key == "enable_bridge_reconnection") {
                    std::string lower_value = value; std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
                    if (lower_value == "true" || lower_value == "1") enable_bridge_reconnection = true;
                    else if (lower_value == "false" || lower_value == "0") enable_bridge_reconnection = false;
                    else { std::cerr << "Warning: Invalid boolean value for 'enable_bridge_reconnection': " << value << ". Using default (false)." << std::endl; enable_bridge_reconnection = false; }
                    std::cout << "  enable_bridge_reconnection set to: " << std::boolalpha << enable_bridge_reconnection << " (from preferences file)" << std::endl;
                } else if (key == "connect_to_drone") {
                    std::string lower_value = value; std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
                    if (lower_value == "true" || lower_value == "1") connect_to_drone = true;
                    else if (lower_value == "false" || lower_value == "0") connect_to_drone = false;
                    else { std::cerr << "Warning: Invalid boolean value for 'connect_to_drone': " << value << ". Using default (true)." << std::endl; connect_to_drone = true; }
                    std::cout << "  connect_to_drone set to: " << std::boolalpha << connect_to_drone << " (from preferences file)" << std::endl;
                } else { std::cout << "  Ignoring unrecognized key in preferences: " << key << std::endl; }
            } catch (const std::invalid_argument& ia) { std::cerr << "Warning: Invalid number format for key '" << key << "' in preferences file: " << value << std::endl; }
              catch (const std::out_of_range& oor) { std::cerr << "Warning: Value out of range for key '" << key << "' in preferences file: " << value << std::endl; }
              catch (...) { std::cerr << "Warning: Unknown error parsing line for key '" << key << "' in preferences file: " << value << std::endl; }
        }
        preferencesFile.close();
        std::cout << "Finished loading preferences." << std::endl;
    } else {
        std::cout << "Preferences file ('preferences.txt') not found. Using default values:" << std::endl;
        std::cout << "  Default TARGET_BEACON_ID: " << TARGET_BEACON_ID << std::endl;
        std::cout << "  Default targetDistance: " << targetDistance << " meters" << std::endl;
        std::cout << "  Default targetAzimuth (Initial): " << targetAzimuth << " degrees" << std::endl;
        std::cout << "  Default Kp_forward: " << Kp_forward << std::endl;
        std::cout << "  Default max_forward_speed: " << max_forward_speed << " m/s" << std::endl;
        std::cout << "  Default forward_dead_zone: " << forward_dead_zone << " meters" << std::endl;
        std::cout << "  Default Kp_lateral: " << Kp_lateral << std::endl;
        std::cout << "  Default max_lateral_speed: " << max_lateral_speed << " m/s" << std::endl;
        std::cout << "  Default azimuth_dead_zone: " << azimuth_dead_zone << " degrees" << std::endl;
        std::cout << "  Default enable_bridge_reconnection: " << std::boolalpha << enable_bridge_reconnection << std::endl;
        std::cout << "  Default connect_to_drone: " << std::boolalpha << connect_to_drone << std::endl;
    }
}

// Display full radar object details
void displayRadarObjects(const std::vector<RadarObject>& objects) {
    if (objects.empty()) return;
    std::cout << "--- Begin Radar Objects for Second: " << currentSecond << " ---" << std::endl;
    for (const auto& obj : objects) {
        std::cout << "Radar Object:\n" << "  Timestamp: " << obj.timestamp << "\n" << "  Sensor: " << obj.sensor << "\n" << "  Source: " << obj.src << "\n" << "  ID: " << obj.ID << "\n" << "  X: " << obj.X << " Y: " << obj.Y << " Z: " << obj.Z << "\n" << "  Xdir: " << obj.Xdir << " Ydir: " << obj.Ydir << " Zdir: " << obj.Zdir << "\n" << "  Range: " << obj.Range << " Range Rate: " << obj.RangeRate << "\n" << "  Power: " << obj.Pwr << " Azimuth: " << obj.Az << " Elevation: " << obj.El << "\n" << "  Xsize: " << obj.Xsize << " Ysize: " << obj.Ysize << " Zsize: " << obj.Zsize << "\n" << "  Confidence: " << obj.Conf << "\n" << "----------------------------------------" << std::endl;
    }
     std::cout << "--- End Radar Objects for Second: " << currentSecond << " ---" << std::endl;
}

// Extract data for Wall Following mode [w]
void extractBeaconAndWallData(const std::vector<RadarObject>& objects, Vehicle* vehiclePtr, bool enableControlCmd) {
    // Use the passed vehiclePtr
    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return; // Check stop flag
        // Clean timestamp string
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
        if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
        // Extract second part of timestamp
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;

        // Skip old data if processing is lagging
        if (!currentSecond.empty() && objSecond < currentSecond) continue;

        // Process data from the previous second when the second changes
        if (objSecond != currentSecond) {
            if (!currentSecond.empty() && (hasAnonData || foundTargetBeacon)) {
                // Perform control action based on accumulated data
                if (enableControlCmd && vehiclePtr != nullptr && vehiclePtr->control != nullptr) {
                    float velocity_x = 0.0f, velocity_y = 0.0f;
                    // Calculate forward velocity based on wall distance
                    if (hasAnonData) {
                        float difference = lowestRange - targetDistance;
                        if (std::abs(difference) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * difference, max_forward_speed));
                        }
                    }
                    // Calculate lateral velocity based on beacon azimuth
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) {
                        float azimuth_error = targetBeaconAzimuth - targetAzimuth;
                        if (std::abs(azimuth_error) > azimuth_dead_zone) {
                            velocity_y = std::max(-max_lateral_speed, std::min(-Kp_lateral * azimuth_error, max_lateral_speed));
                        }
                    } else if (!foundTargetBeacon && enableControlCmd) { // Only log if control is active
                        std::cout << "***BEACON NOT FOUND***" << std::endl;
                    }
                    // Define control flags and data
                    uint8_t controlFlag = Control::HORIZONTAL_VELOCITY | Control::VERTICAL_VELOCITY | Control::YAW_RATE | Control::HORIZONTAL_BODY | Control::STABLE_ENABLE;
                    Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, 0, 0); // Yaw rate and vertical velocity are zero
                    // Log control status
                    std::cout << "[Wall Follow] Control Status (Second: " << currentSecond << "):\n" << "  TargetWall  = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n" << "  CurrentWall = " << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n" << "  TargetBeaconAz=" << targetAzimuth << "\n" << "  CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n\n" << "  Computed Velocity(X=" << velocity_x << ", Y=" << velocity_y << ")" << std::endl;
                    std::cout << "--------------------------------------" << std::endl;
                    // Send control command
                    vehiclePtr->control->flightCtrl(ctrlData);
                } else if (hasAnonData || foundTargetBeacon) { // Log status even if control disabled
                     if (!foundTargetBeacon) { std::cout << "***BEACON NOT FOUND***" << std::endl;} // Log beacon loss
                     std::cout << "[Wall Follow] (Flight Control Disabled or Not Available) (Second: " << currentSecond << "):\n" << "  TargetWall  = " << std::fixed << std::setprecision(1) << std::setw(3) << targetDistance << "\n" << "  CurrentWall = " << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n" << "  TargetBeaconAz=" << targetAzimuth << "\n" << "  CurrentBeaconAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << std::endl;
                     std::cout << "--------------------------------------" << std::endl;
                }
            }
            // Reset state variables for the new second
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            foundTargetBeacon = false;
        }

        // Accumulate data for the current second
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) { // Only store the first one encountered in the second
                targetBeaconAzimuth = obj.Az;
                foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) { // Check if ID contains "anon"
            hasAnonData = true;
            if (obj.Range < lowestRange) {
                lowestRange = obj.Range; // Track overall closest anon range
            }
        }
         if (stopProcessingFlag.load()) return; // Check stop flag again
    }
}

// Parses JSON radar data string into a vector of RadarObject structs
std::vector<RadarObject> parseRadarData(const std::string& jsonData) {
    std::vector<RadarObject> radarObjects;
    if (jsonData.empty() || jsonData == "{}") return radarObjects; // Handle empty data
    try {
        auto jsonFrame = json::parse(jsonData);
        // Check if the top-level structure contains "objects" array
        if (!jsonFrame.contains("objects") || !jsonFrame["objects"].is_array()) {
            // std::cerr << "Warning: JSON does not contain 'objects' array." << std::endl; // Optional warning
            return radarObjects;
        }
        // Iterate through each object in the "objects" array
        for (const auto& obj : jsonFrame["objects"]) {
            if (!obj.is_object()) {
                std::cerr << "Skipping non-object item in 'objects' array." << std::endl;
                continue;
            }
            RadarObject radarObj;
            // Safely extract values using .value(), providing defaults if key not found or wrong type
            radarObj.timestamp = obj.value("timestamp", json(nullptr)).dump(); // Keep as string initially
            radarObj.sensor = obj.value("sensor", "N/A");
            radarObj.src = obj.value("src", "N/A");
            radarObj.X = obj.value("X", 0.0f);
            radarObj.Y = obj.value("Y", 0.0f);
            radarObj.Z = obj.value("Z", 0.0f);
            radarObj.Xdir = obj.value("Xdir", 0.0f);
            radarObj.Ydir = obj.value("Ydir", 0.0f);
            radarObj.Zdir = obj.value("Zdir", 0.0f);
            radarObj.Range = obj.value("Range", 0.0f);
            radarObj.RangeRate = obj.value("RangeRate", 0.0f);
            radarObj.Pwr = obj.value("Pwr", 0.0f);
            radarObj.Az = obj.value("Az", 0.0f);
            radarObj.El = obj.value("El", 0.0f);
            // Handle ID which might be string or number in JSON
            if (obj.contains("ID") && obj["ID"].is_string()) {
                radarObj.ID = obj.value("ID", "N/A");
            } else if (obj.contains("ID")) {
                radarObj.ID = obj["ID"].dump(); // Convert number/other to string representation
            } else {
                radarObj.ID = "N/A";
            }
            radarObj.Xsize = obj.value("Xsize", 0.0f);
            radarObj.Ysize = obj.value("Ysize", 0.0f);
            radarObj.Zsize = obj.value("Zsize", 0.0f);
            radarObj.Conf = obj.value("Conf", 0.0f);
            radarObjects.push_back(radarObj);
        }
    } catch (const json::parse_error& e) {
        std::cerr << "JSON Parsing Error: " << e.what() << " at offset " << e.byte << ". Data: [" << jsonData.substr(0, 200) << "...]" << std::endl; // Log parse error
        return {}; // Return empty vector on error
    } catch (const json::type_error& e) {
        std::cerr << "JSON Type Error: " << e.what() << ". Data: [" << jsonData.substr(0, 200) << "...]" << std::endl; // Log type error
        return {}; // Return empty vector on error
    }
    return radarObjects;
}

// Runs the python bridge script in the background
void runPythonBridge(const std::string& scriptName) {
    std::cout << "Starting Python bridge (" << scriptName << ")..." << std::endl;
    // Use python3 explicitly, append '&' to run in background on Linux/macOS
    if (std::system(("python3 " + scriptName + " &").c_str()) != 0) {
        std::cerr << "Failed to start Python bridge script '" << scriptName << "'." << std::endl;
    }
    // Give the script a moment to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Python bridge potentially started." << std::endl;
}

// Stops the python bridge script using pkill
void stopPythonBridge(const std::string& scriptName) {
    std::cout << "Stopping Python bridge (" << scriptName << ")..." << std::endl;
    // Use pkill to find the process by name/command line
    // -f matches against the entire command line
    std::system(("pkill -f " + scriptName).c_str()); // Ignore result, might fail if not running
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Give it a moment to terminate
    std::cout << "Sent SIGTERM to " << scriptName << "." << std::endl;
}

// Connects to the python bridge via TCP, with retries
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket) {
    tcp::resolver resolver(io_context);
    int initial_retries = 5; // Number of quick initial retries
    int current_retry = 0;
    const int initial_delay_seconds = 2;

    // Reset force stop flag before attempting connection
    forceStopReconnectionFlag.store(false);

    // --- Initial Connection Attempts ---
    while (current_retry < initial_retries) {
        // Check stop flags at the beginning of each attempt
        if (stopProcessingFlag.load() || forceStopReconnectionFlag.load()) {
             std::cout << "[Connect Bridge] Stop requested during initial connection attempts." << std::endl;
             forceStopReconnectionFlag.store(false); // Reset flag
             currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status
             return false; // Stop requested
        }
        try {
            if(socket.is_open()) { socket.close(); } // Close previous attempt if any
            auto endpoints = resolver.resolve("127.0.0.1", "5000"); // Resolve localhost:5000
            boost::system::error_code ec;
            boost::asio::connect(socket, endpoints, ec); // Attempt connection
            if (!ec) { // Success
                std::cout << "Connected to Python bridge." << std::endl;
                currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED); // Update status
                return true; // Connection successful
            }
            else { // Failure
                // Log failure only on the last retry to avoid spamming
                if (current_retry == initial_retries - 1) {
                   std::cerr << "Initial connection attempt " << (current_retry + 1) << "/" << initial_retries << " failed: " << ec.message() << std::endl;
                }
            }
        } catch (const std::exception& e) {
             // Log exception only on the last retry
             if (current_retry == initial_retries - 1) {
                std::cerr << "Exception during initial connection attempt " << (current_retry + 1) << "/" << initial_retries << ": " << e.what() << std::endl;
             }
        }

        current_retry++;
        // Wait before next retry, checking stop flags frequently
        if (current_retry < initial_retries && !stopProcessingFlag.load() && !forceStopReconnectionFlag.load()) {
             // std::cout << "Retrying connection in " << initial_delay_seconds << " seconds..." << std::endl; // Optional: Reduce verbosity
             for (int i = 0; i < initial_delay_seconds * 10 && !stopProcessingFlag.load() && !forceStopReconnectionFlag.load(); ++i) {
                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
             }
        }
    } // End initial retries loop

    // --- Initial Attempts Failed ---
    std::cerr << "Failed initial connection attempts to Python bridge." << std::endl;

    // Check if stop was requested during retries
    if (stopProcessingFlag.load() || forceStopReconnectionFlag.load()) {
        std::cout << "[Connect Bridge] Stop requested after initial attempts failed." << std::endl;
        forceStopReconnectionFlag.store(false); // Reset flag
        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status
        return false;
    }

    // --- Persistent Reconnection (Try once more if enabled) ---
    // Check if reconnection is enabled globally
    if (!enable_bridge_reconnection) {
        std::cout << "Reconnection disabled. Giving up." << std::endl;
        currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status
        return false; // Give up if reconnection is disabled
    }

    // Try one more time immediately after initial failures before returning (caller handles further delays)
    std::cout << "Attempting final connection before handing back to caller..." << std::endl;
    try {
        if(socket.is_open()) { socket.close(); }
        auto endpoints = resolver.resolve("127.0.0.1", "5000");
        boost::system::error_code ec;
        boost::asio::connect(socket, endpoints, ec);
        if (!ec) { // Success on final attempt
            std::cout << "Final connection attempt successful." << std::endl;
            currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED); // Update status
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
    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Update status
    return false; // Indicate final connection failure
}

// Placeholder Callbacks (Required by OSDK headers but unused here)
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }
void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }

// Main processing thread function (handles radar data)
void processingLoopFunction(const std::string bridgeScriptName, Vehicle* vehiclePtr, bool enableControlCmd, ProcessingMode mode) {
    std::cout << "Processing thread started. Bridge: " << bridgeScriptName
              << ", Control Enabled: " << std::boolalpha << enableControlCmd
              << ", Mode: " << static_cast<int>(mode)
              << ", Reconnect Enabled: " << enable_bridge_reconnection << std::endl;
    int functionTimeout = 1; // Timeout for OSDK control calls

    runPythonBridge(bridgeScriptName); // Start the python script

    boost::asio::io_context io_context;
    tcp::socket socket(io_context);

    // Initial connection attempt (handles retries internally)
    currentBridgeStatus.store(BridgeConnectionStatus::RECONNECTING);
    if (!connectToPythonBridge(io_context, socket)) {
        // If connection fails initially or is stopped
        std::cerr << "Processing thread: Initial connection failed or was stopped. Exiting thread." << std::endl;
        stopPythonBridge(bridgeScriptName); // Ensure script is stopped
        // Release control if obtained but connection failed (and not stopped externally)
        if (!stopProcessingFlag.load() && !forceStopReconnectionFlag.load() && enableControlCmd && vehiclePtr != nullptr && vehiclePtr->control != nullptr) {
            std::cout << "[Processing Thread] Releasing control authority (connection fail)..." << std::endl;
            ACK::ErrorCode releaseAck = vehiclePtr->control->releaseCtrlAuthority(functionTimeout);
            if (ACK::getError(releaseAck)) ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority");
            else std::cout << "[Processing Thread] Control authority released." << std::endl;
        }
        // connectToPythonBridge already sets status to DISCONNECTED on failure
        return; // Exit thread
    }
    // If connection succeeds, connectToPythonBridge sets status to CONNECTED
    std::cout << "Processing thread: Connection successful. Reading data stream..." << std::endl;

    // Buffers and state variables for reading loop
    std::string received_data_buffer;
    std::array<char, 4096> read_buffer;
    bool connection_error_occurred = false; // Track connection state

    // Reset state variables for Wall Following mode
    currentSecond = "";
    lowestRange = std::numeric_limits<float>::max();
    hasAnonData = false;
    targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
    foundTargetBeacon = false;

    // Main data reading loop
    while (!stopProcessingFlag.load()) {
        boost::system::error_code error;
        size_t len = socket.read_some(boost::asio::buffer(read_buffer), error); // Read data from socket

        if (error) { // Handle read error (connection lost)
            if (error == boost::asio::error::eof) std::cerr << "Processing thread: Connection closed by Python bridge (EOF)." << std::endl;
            else std::cerr << "Processing thread: Error reading from socket: " << error.message() << std::endl;

            connection_error_occurred = true; // Mark error state

            // Send stop command immediately if controlling drone
            if (enableControlCmd && vehiclePtr != nullptr && vehiclePtr->control != nullptr) {
                 std::cout << "[Processing Thread] Connection error. Sending zero velocity command..." << std::endl;
                 uint8_t controlFlag = Control::HORIZONTAL_VELOCITY | Control::VERTICAL_VELOCITY | Control::YAW_RATE | Control::HORIZONTAL_BODY | Control::STABLE_ENABLE;
                 Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
                 vehiclePtr->control->flightCtrl(stopData); // Send stop command
            }

            // Attempt reconnection if enabled
            if (enable_bridge_reconnection) {
                std::cout << "Connection lost. Attempting to reconnect..." << std::endl;
                currentBridgeStatus.store(BridgeConnectionStatus::RECONNECTING); // Set status
                if (socket.is_open()) socket.close(); // Ensure socket is closed

                // Wait before retrying, checking stop flags
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

                // Attempt persistent reconnection (connectToPythonBridge handles retries/stop flags)
                if (connectToPythonBridge(io_context, socket)) {
                    std::cout << "Reconnection successful. Resuming data processing." << std::endl;
                    // connectToPythonBridge sets status to CONNECTED
                    connection_error_occurred = false; // Clear error state
                    received_data_buffer.clear(); // Clear buffer from before disconnect
                    continue; // Go back to the start of the while loop to try reading again
                } else {
                    // Connection failed persistently OR was force stopped
                    // connectToPythonBridge sets status to DISCONNECTED
                    std::cerr << "Processing thread: Persistent reconnection failed or was stopped. Exiting." << std::endl;
                    break; // Exit the main while loop
                }
            } else { // Reconnection disabled
                std::cerr << "Reconnection disabled. Stopping processing." << std::endl;
                currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Set status
                break; // Exit the main while loop
            }
        } // End error handling

        // If no error, process data
        connection_error_occurred = false; // Clear error state if read was successful
        // Ensure status is CONNECTED if we are successfully reading data
        if (currentBridgeStatus.load() != BridgeConnectionStatus::CONNECTED) {
            currentBridgeStatus.store(BridgeConnectionStatus::CONNECTED);
        }

        if (len > 0) { // Process received data if any
            received_data_buffer.append(read_buffer.data(), len); // Append new data
            size_t newline_pos;
            // Process data line by line (separated by newline)
            while ((newline_pos = received_data_buffer.find('\n')) != std::string::npos) {
                std::string jsonData = received_data_buffer.substr(0, newline_pos); // Extract one line
                received_data_buffer.erase(0, newline_pos + 1); // Remove processed line from buffer

                if (stopProcessingFlag.load()) break; // Check stop flag

                if (!jsonData.empty()) {
                    try {
                        // Parse the JSON line into RadarObject vector
                        auto radarObjects = parseRadarData(jsonData);

                        // Process based on the mode
                        switch (mode) {
                            case ProcessingMode::WALL_FOLLOW:
                                extractBeaconAndWallData(radarObjects, vehiclePtr, enableControlCmd);
                                break;
                            case ProcessingMode::PROCESS_FULL:
                                // Update currentSecond for display header
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
                                displayRadarObjects(radarObjects); // Display full data
                                break;
                        }
                    } catch (const std::exception& e) {
                         std::cerr << "Error processing data: " << e.what() << "\nSnippet: [" << jsonData.substr(0, 100) << "...]" << std::endl;
                    }
                }
            } // End processing lines in buffer
            if (stopProcessingFlag.load()) break; // Check stop flag again
        } else {
             // If len is 0 and no error, sleep briefly to avoid busy-waiting
             std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } // End main data reading loop (while !stopProcessingFlag)

    // --- Cleanup when loop exits ---
    if (stopProcessingFlag.load()) std::cout << "[Processing Thread] Stop requested manually." << std::endl;
    else if (connection_error_occurred) std::cout << "[Processing Thread] Exiting due to unrecoverable connection error or forced stop." << std::endl;
    else std::cout << "[Processing Thread] Data stream ended gracefully (or loop exited)." << std::endl;

    if (socket.is_open()) { socket.close(); } // Close socket
    stopPythonBridge(bridgeScriptName); // Stop python script

    // Release Control Authority if held (and not stopped externally)
    if (!stopProcessingFlag.load() && enableControlCmd && vehiclePtr != nullptr && vehiclePtr->control != nullptr) {
        std::cout << "[Processing Thread] Sending final zero velocity command..." << std::endl;
        uint8_t controlFlag = Control::HORIZONTAL_VELOCITY | Control::VERTICAL_VELOCITY | Control::YAW_RATE | Control::HORIZONTAL_BODY | Control::STABLE_ENABLE;
        Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
        vehiclePtr->control->flightCtrl(stopData);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Allow command to be sent

        std::cout << "[Processing Thread] Releasing control authority..." << std::endl;
        ACK::ErrorCode releaseAck = vehiclePtr->control->releaseCtrlAuthority(functionTimeout);
        // Log error only if it's a real failure, not just "not obtained"
        if (ACK::getError(releaseAck) && releaseAck.info.cmd_set != 0x00 && releaseAck.info.cmd_id != 0x00) {
             ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority (on exit)");
             std::cerr << "[Processing Thread] Warning: Failed to release control authority on exit." << std::endl;
        } else {
             std::cout << "[Processing Thread] Control authority released (or was not held)." << std::endl;
        }
    } else if (stopProcessingFlag.load()) {
        // If stopped externally, caller (deinitializeOSDK) handles implicit release via object deletion
        std::cout << "[Processing Thread] Exiting due to external stop signal. Control release responsibility transferred or object invalidated." << std::endl;
    }

    std::cout << "Processing thread finished." << std::endl;
    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Ensure status is DISCONNECTED when thread ends
}

// Helper function to get mode name string from display mode enum
std::string getModeName(uint8_t mode) {
    switch(mode) {
        case VehicleStatus::DisplayMode::MODE_MANUAL_CTRL: return "MANUAL_CTRL";
        case VehicleStatus::DisplayMode::MODE_ATTITUDE: return "ATTITUDE";
        case VehicleStatus::DisplayMode::MODE_P_GPS: return "P_GPS";
        case VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL: return "NAVI_SDK_CTRL";
        case 31: return "Mode 31"; // Example for unknown mode
        default: return "Other (" + std::to_string(mode) + ")";
    }
}

// Background thread function for monitoring drone telemetry (flight status, display mode)
void monitoringLoopFunction(Vehicle* vehiclePtr) {
    // Ensure valid vehicle pointer is passed
    if (vehiclePtr == nullptr || vehiclePtr->subscribe == nullptr) {
        std::cerr << "[Monitoring] Error: Invalid Vehicle object provided. Thread exiting." << std::endl;
        return;
    }
    std::cout << "[Monitoring] Thread started." << std::endl;
    // State variables for monitoring logic
    bool telemetry_timed_out = false;
    bool warned_unexpected_status = false;
    uint8_t previous_flight_status = VehicleStatus::FlightStatus::STOPED;
    time_t last_valid_poll_time = 0;
    bool in_sdk_control_mode = false;
    bool warned_not_in_sdk_mode = false;
    const uint8_t EXPECTED_SDK_MODE = VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL;

    // Monitoring loop
    while (!stopMonitoringFlag.load()) {
        // Sanity check pointers inside loop (less critical now, but safe)
        if (vehiclePtr == nullptr || vehiclePtr->subscribe == nullptr) {
             if (!telemetry_timed_out) { // Avoid spamming log
                 std::cerr << "\n**** MONITORING ERROR: Vehicle/subscribe object became null. Stopping. ****" << std::endl << std::endl;
                 telemetry_timed_out = true;
             }
             break; // Exit if vehicle objects become invalid
        }

        // Read telemetry values using the passed vehicle pointer
        uint8_t current_flight_status = vehiclePtr->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
        uint8_t current_display_mode = vehiclePtr->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        // Basic validity check (adjust if needed based on OSDK version/drone)
        bool valid_poll = (current_flight_status <= VehicleStatus::FlightStatus::IN_AIR);

        if (valid_poll) {
            time_t current_time = std::time(nullptr);
            last_valid_poll_time = current_time; // Update time of last valid poll
            // If previously timed out, log recovery
            if (telemetry_timed_out) {
                 std::cout << "[Monitoring] Telemetry poll recovered." << std::endl;
                 telemetry_timed_out = false;
            }

            // Check for unexpected change from IN_AIR state
            if (current_flight_status != VehicleStatus::FlightStatus::IN_AIR) {
                if ( (previous_flight_status == VehicleStatus::FlightStatus::IN_AIR) &&
                     (current_flight_status == VehicleStatus::FlightStatus::ON_GROUND || current_flight_status == VehicleStatus::FlightStatus::STOPED) &&
                     !warned_unexpected_status) {
                     std::cerr << "\n**** MONITORING WARNING: Flight status changed unexpectedly from IN_AIR to " << (int)current_flight_status << ". ****" << std::endl << std::endl;
                     warned_unexpected_status = true;
                }
            } else {
                 warned_unexpected_status = false; // Reset warning once back in air
            }
            previous_flight_status = current_flight_status; // Store current status for next iteration

            // Check if drone is in the expected SDK control mode
            bool is_expected_mode = (current_display_mode == EXPECTED_SDK_MODE);
            if (is_expected_mode) {
                if (!in_sdk_control_mode) { // Log entry into SDK mode
                    std::cout << "\n**** MONITORING INFO: Entered SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << " / " << (int)EXPECTED_SDK_MODE << ") ****" << std::endl << std::endl;
                    in_sdk_control_mode = true;
                    warned_not_in_sdk_mode = false; // Reset warning flag
                }
            } else { // Not in expected mode
                 // Warn only if previously in SDK mode or not warned yet, AND processing thread is active
                 if ((in_sdk_control_mode || !warned_not_in_sdk_mode) && !stopProcessingFlag.load() && processingThread.joinable()) {
                      std::string current_mode_name = getModeName(current_display_mode);
                      std::cerr << "\n**** MONITORING WARNING: NOT in expected SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << "). Current: " << current_mode_name << " (" << (int)current_display_mode << ") ****" << std::endl << std::endl;
                      warned_not_in_sdk_mode = true;
                 }
                 in_sdk_control_mode = false; // Update state
            }
        } else { // Invalid poll data received
            if (!telemetry_timed_out) { // Avoid spamming log
                 std::cerr << "\n**** MONITORING WARNING: Polling telemetry returned potentially invalid data (FlightStatus=" << (int)current_flight_status << "). ****" << std::endl << std::endl;
                 telemetry_timed_out = true;
            }
        }

        // Check Telemetry Timeout (no valid data for too long)
        time_t current_time_for_timeout_check = std::time(nullptr);
        if (last_valid_poll_time > 0 && (current_time_for_timeout_check - last_valid_poll_time > TELEMETRY_TIMEOUT_SECONDS)) {
            if (!telemetry_timed_out) { // Avoid spamming log
                std::cerr << "\n**** MONITORING TIMEOUT: No valid telemetry for over " << TELEMETRY_TIMEOUT_SECONDS << " seconds. ****" << std::endl << std::endl;
                telemetry_timed_out = true;
            }
        } else if (last_valid_poll_time == 0) { // Check if never received first poll
             static time_t start_time = 0; if (start_time == 0) start_time = current_time_for_timeout_check;
             // Allow more time initially before declaring timeout
             if (current_time_for_timeout_check - start_time > TELEMETRY_TIMEOUT_SECONDS * 2) {
                  if (!telemetry_timed_out) { // Avoid spamming log
                       std::cerr << "\n**** MONITORING TIMEOUT: Never received valid telemetry poll after " << TELEMETRY_TIMEOUT_SECONDS * 2 << " seconds. ****" << std::endl << std::endl;
                       telemetry_timed_out = true;
                  }
             }
        }

        // Wait before next poll
        std::this_thread::sleep_for(std::chrono::seconds(1));
    } // End monitoring loop (while !stopMonitoringFlag)

    std::cout << "[Monitoring] Thread finished." << std::endl;
}

// Helper function to signal and join the processing thread if it's running
void stopProcessingThreadIfNeeded() {
    // Check if thread is joinable (i.e., running or finished but not joined)
    if (!processingThread.joinable()) {
         // Ensure bridge status is consistent if no thread running
         currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
         return; // Nothing to stop
    }

    std::cout << "[Stop Thread] Signalling processing thread to stop..." << std::endl;
    stopProcessingFlag.store(true); // Signal the main loop in processing thread
    forceStopReconnectionFlag.store(true); // Signal the connect/reconnect loop too

    // Wait for the thread to finish execution
    processingThread.join(); // This blocks until the thread function returns
    std::cout << "[Stop Thread] Processing thread finished." << std::endl;

    // Reset flags after thread has joined
    stopProcessingFlag.store(false);
    forceStopReconnectionFlag.store(false);
    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED); // Ensure status is updated
}

// --- Implementation of initializeOSDK ---
// Initializes the DJI OSDK environment, vehicle, and telemetry subscription
// NOTE: This function is now called from the connectionTask thread OR directly on startup
bool initializeOSDK(int argc, char** argv) {
    std::lock_guard<std::mutex> lock(osdkMutex); // Lock for thread safety

    // Check if already initialized or in an inconsistent state
    if (linuxEnvironment != nullptr || vehicle != nullptr || enableFlightControl) {
        std::cerr << "[OSDK Init] Already initialized or in an inconsistent state." << std::endl;
        return enableFlightControl; // Return current actual status
    }

    std::cout << "[OSDK Init] Initializing..." << std::endl;
    try {
        // 1. Create LinuxSetup (Handles serial port, basic setup)
        // This constructor itself might block during functionalSetUp/getDroneVersion
        linuxEnvironment = new LinuxSetup(argc, argv); // Pass actual argc/argv
        if (!linuxEnvironment) {
            std::cerr << "[OSDK Init] Failed to create LinuxSetup." << std::endl;
            return false;
        }

        // 2. Get Vehicle instance (This also performs activation and handshake)
        vehicle = linuxEnvironment->getVehicle();
        // Check if vehicle, control, and subscribe interfaces are valid
        if (vehicle == nullptr || vehicle->control == nullptr || vehicle->subscribe == nullptr) {
            // This check often fails if the drone connection/handshake fails after retries
            std::cerr << "[OSDK Init] ERROR: Vehicle not initialized or essential interfaces unavailable (likely connection/handshake failure)." << std::endl;
            // Clean up partially created LinuxSetup
            delete linuxEnvironment;
            linuxEnvironment = nullptr;
            vehicle = nullptr; // Ensure pointer is null
            return false; // Indicate initialization failure
        }
        // If we get here, vehicle handshake was successful

        // 3. Set up Telemetry Subscription
        std::cout << "[OSDK Init] Vehicle instance OK. Setting up Telemetry..." << std::endl;
        int functionTimeout = 1; // OSDK API call timeout

        // Verify telemetry subscription capabilities
        ACK::ErrorCode subscribeAck = vehicle->subscribe->verify(functionTimeout);
        if (ACK::getError(subscribeAck)) {
             ACK::getErrorCodeMessage(subscribeAck, "[OSDK Init] subscribe->verify");
             std::cerr << "[OSDK Init] Error verifying subscription package list. Cannot initialize." << std::endl;
             // Clean up LinuxSetup and Vehicle
             delete linuxEnvironment; linuxEnvironment = nullptr; vehicle = nullptr;
             return false;
        }

        // Define telemetry topics to subscribe to
        Telemetry::TopicName topicList[] = {
            Telemetry::TOPIC_STATUS_FLIGHT,      // For flight status (on ground, in air, etc.)
            Telemetry::TOPIC_STATUS_DISPLAYMODE // For current flight mode (P-GPS, SDK Control, etc.)
        };
        int numTopic = sizeof(topicList) / sizeof(topicList[0]);
        pkgIndex = 0; // Use package index 0 (reset in case of re-initialization)

        // Initialize the telemetry package
        bool topicStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList,
            false, // Send data timestamped? false = use OSDK timestamp
            telemetrySubscriptionFrequency // Frequency (Hz)
        );
        if (!topicStatus) {
             std::cerr << "[OSDK Init] Error initializing telemetry package " << pkgIndex << ". Cannot initialize." << std::endl;
             // Clean up LinuxSetup and Vehicle
             delete linuxEnvironment; linuxEnvironment = nullptr; vehicle = nullptr;
             return false;
        }
        std::cout << "[OSDK Init] Telemetry package " << pkgIndex << " initialized." << std::endl;

        // Start receiving data for the package
        ACK::ErrorCode startAck = vehicle->subscribe->startPackage(pkgIndex, functionTimeout);
        if (ACK::getError(startAck)) {
            ACK::getErrorCodeMessage(startAck, "[OSDK Init] startPackage");
            std::cerr << "[OSDK Init] Error starting subscription package " << pkgIndex << ". Cannot initialize." << std::endl;
            // Attempt to remove the package before cleaning up
            vehicle->subscribe->removePackage(pkgIndex, functionTimeout); // Ignore error on removal attempt
            delete linuxEnvironment; linuxEnvironment = nullptr; vehicle = nullptr;
            return false;
        }
        std::cout << "[OSDK Init] Telemetry package " << pkgIndex << " started." << std::endl;

        // 4. Start Monitoring Thread
        std::cout << "[OSDK Init] Starting monitoring thread..." << std::endl;
        stopMonitoringFlag.store(false); // Ensure stop flag is clear
        // Ensure previous thread (if any) is finished before starting a new one
        if (monitoringThread.joinable()) {
            std::cout << "[OSDK Init] Joining previous monitoring thread..." << std::endl;
            monitoringThread.join();
            std::cout << "[OSDK Init] Previous monitoring thread joined." << std::endl;
        }
        monitoringThread = std::thread(monitoringLoopFunction, vehicle); // Pass current vehicle ptr
        monitoringEnabled = true; // Mark monitoring as active

        // 5. Set final status flag only on full success
        enableFlightControl = true; // Mark OSDK as ready for flight control commands
        std::cout << "[OSDK Init] Initialization successful." << std::endl;
        return true; // Success

    } catch (const std::exception& e) { // Catch standard exceptions
        std::cerr << "[OSDK Init] Exception during initialization: " << e.what() << std::endl;
        // Cleanup any partially created resources
        if (monitoringThread.joinable()) { stopMonitoringFlag.store(true); monitoringThread.join(); }
        if (vehicle && vehicle->subscribe) { vehicle->subscribe->removePackage(pkgIndex, 1); } // Attempt removal
        if (linuxEnvironment) { delete linuxEnvironment; linuxEnvironment = nullptr; }
        vehicle = nullptr; enableFlightControl = false; monitoringEnabled = false;
        return false; // Failure
    } catch (...) { // Catch any other unknown exceptions
        std::cerr << "[OSDK Init] Unknown exception during initialization." << std::endl;
        // Cleanup any partially created resources
        if (monitoringThread.joinable()) { stopMonitoringFlag.store(true); monitoringThread.join(); }
        if (vehicle && vehicle->subscribe) { vehicle->subscribe->removePackage(pkgIndex, 1); } // Attempt removal
        if (linuxEnvironment) { delete linuxEnvironment; linuxEnvironment = nullptr; }
        vehicle = nullptr; enableFlightControl = false; monitoringEnabled = false;
        return false; // Failure
    }
}

// --- Implementation of deinitializeOSDK ---
// Cleans up OSDK resources and stops related threads
void deinitializeOSDK() {
    std::lock_guard<std::mutex> lock(osdkMutex); // Lock the whole function for thread safety

    // Check if already deinitialized or nothing to do
    if (!enableFlightControl && linuxEnvironment == nullptr && vehicle == nullptr) {
        // std::cout << "[OSDK Deinit] Already deinitialized or nothing to do." << std::endl; // Optional log
        return;
    }
    std::cout << "[OSDK Deinit] Deinitializing..." << std::endl;

    // 1. Stop processing thread first (releases control if held)
    std::cout << "[OSDK Deinit] Stopping processing thread if running..." << std::endl;
    stopProcessingThreadIfNeeded(); // Signals and joins the thread

    // 2. Stop monitoring thread
    if (monitoringThread.joinable()) {
        std::cout << "[OSDK Deinit] Signalling monitoring thread to stop..." << std::endl;
        stopMonitoringFlag.store(true); // Signal the loop to exit
        monitoringThread.join(); // Wait for thread to finish
        std::cout << "[OSDK Deinit] Monitoring thread finished." << std::endl;
    } else {
        std::cout << "[OSDK Deinit] Monitoring thread not joinable." << std::endl;
    }
    monitoringEnabled = false; // Reset flag regardless

    // 3. Cleanup OSDK resources (Vehicle, Telemetry) - Only if vehicle exists
    if (vehicle != nullptr) {
        int functionTimeout = 1; // Timeout for OSDK API calls
        // Unsubscribe Telemetry if subscribe object exists
        if (vehicle->subscribe != nullptr) {
             std::cout << "[OSDK Deinit] Attempting to unsubscribe from telemetry..." << std::endl;
             // Attempt to remove the package; check error but continue cleanup
             ACK::ErrorCode statusAck = vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
             if(ACK::getError(statusAck)) {
                 ACK::getErrorCodeMessage(statusAck, "[OSDK Deinit] removePackage"); // Log error
             } else {
                 std::cout << "[OSDK Deinit] Telemetry package " << pkgIndex << " removed." << std::endl;
             }
        } else {
            std::cout << "[OSDK Deinit] Subscribe pointer invalid, skipping telemetry cleanup." << std::endl;
        }
        // Note: Control authority should have been released by processing thread
        // or is implicitly lost when vehicle object is deleted below.
    } else {
        std::cout << "[OSDK Deinit] Vehicle pointer invalid, skipping OSDK resource cleanup." << std::endl;
    }

    // 4. Delete LinuxSetup (which automatically deletes the associated Vehicle instance)
    if (linuxEnvironment != nullptr) {
        std::cout << "[OSDK Deinit] Deleting linuxEnvironment..." << std::endl;
        delete linuxEnvironment;
        linuxEnvironment = nullptr; // Mark as deleted
        vehicle = nullptr; // Vehicle pointer is now invalid
    } else {
        // Ensure vehicle pointer is null if environment wasn't created or already deleted
        vehicle = nullptr;
    }

    // 5. Reset final status flag
    enableFlightControl = false; // Mark OSDK as not ready
    std::cout << "[OSDK Deinit] Deinitialization finished." << std::endl;
}

// --- NEW Connection Task Function ---
// This function runs in a separate thread to handle the blocking initializeOSDK call
void connectionTask(int argc, char** argv) {
    std::cout << "[Conn Thread] Started." << std::endl;
    bool success = initializeOSDK(argc, argv); // Perform the blocking initialization
    connectionResult.store(success);          // Store the result
    connectionAttemptFinished.store(true);    // Signal that the attempt is complete
    std::cout << "[Conn Thread] Finished. Result: " << std::boolalpha << success << std::endl;
}


// --- Main Function ---
int main(int argc, char** argv) {
    // --- Pass argc and argv to global scope if needed by LinuxSetup later ---
    // It's generally better to pass them directly, as done in initializeOSDK and connectionTask
    // static int g_argc = argc;
    // static char** g_argv = argv;

    // Setup logging first
    LogRedirector logger("run_log.txt");
    std::cout << "[Main] LogRedirector instantiated." << std::endl;
    std::cout << "Starting application: " << (argc > 0 ? argv[0] : "djiosdk-flightcontrol-gui") << std::endl;

    // Load preferences
    std::cout << "[Main] Calling loadPreferences()..." << std::endl;
    loadPreferences(); // Loads initial 'connect_to_drone' state
    std::cout << "[Main] Returned from loadPreferences()." << std::endl;

    // Initial OSDK connection attempt based on preferences (on main thread before GUI starts)
    if (connect_to_drone) { // Check desired state from preferences
        std::cout << "[Main] Initial attempt to connect to drone (connect_to_drone=true)..." << std::endl;
        isConnectingToDrone.store(true); // Set flag *before* blocking call
        if (!initializeOSDK(argc, argv)) { // Pass actual argc/argv; BLOCKING CALL
            std::cerr << "[Main] Initial OSDK initialization failed. Starting disconnected." << std::endl;
            connect_to_drone = false; // Update desired state to reflect failure
        } else {
            std::cout << "[Main] Initial OSDK initialization successful." << std::endl;
            // connect_to_drone remains true, enableFlightControl was set in initializeOSDK
        }
        isConnectingToDrone.store(false); // Clear flag *after* blocking call finishes
    } else {
        std::cout << "[Main] Skipping initial OSDK Initialization (connect_to_drone is false)." << std::endl;
        enableFlightControl = false; // Ensure actual status is false
        monitoringEnabled = false;
    }

    // --- GUI Initialization ---
    std::cout << "[GUI] Initializing SDL..." << std::endl;
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        deinitializeOSDK(); // Cleanup OSDK if partially initialized
        return 1;
    }

    // Decide GL+GLSL versions
    #if defined(IMGUI_IMPL_OPENGL_ES2)
        const char* glsl_version = "#version 100"; SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0); SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    #elif defined(__APPLE__)
        const char* glsl_version = "#version 150"; SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    #else
        const char* glsl_version = "#version 130"; SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0); SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    #endif
    // Set SDL GL attributes
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1); SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24); SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    // Create SDL Window
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("DJI OSDK Control GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
    if (!window) { std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl; SDL_Quit(); deinitializeOSDK(); return 1; }
    // Create GL Context
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) { std::cerr << "SDL_GL_CreateContext Error: " << SDL_GetError() << std::endl; SDL_DestroyWindow(window); SDL_Quit(); deinitializeOSDK(); return 1; }
    SDL_GL_MakeCurrent(window, gl_context); SDL_GL_SetSwapInterval(1); // Enable vsync

    // Initialize GLEW
    std::cout << "[GUI] Initializing OpenGL Loader (GLEW)..." << std::endl;
    glewExperimental = GL_TRUE; GLenum err = glewInit();
    if (err != GLEW_OK) { std::cerr << "Failed to initialize GLEW: " << glewGetErrorString(err) << std::endl; SDL_GL_DeleteContext(gl_context); SDL_DestroyWindow(window); SDL_Quit(); deinitializeOSDK(); return 1; }
    std::cout << "[GUI] Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    std::cout << "[GUI] OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

    // Setup Dear ImGui context
    std::cout << "[GUI] Initializing ImGui..." << std::endl;
    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // GUI state variables
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    bool guiKeepRunning = true;
    char target_beacon_id_buffer[128]; strncpy(target_beacon_id_buffer, TARGET_BEACON_ID.c_str(), sizeof(target_beacon_id_buffer) - 1); target_beacon_id_buffer[sizeof(target_beacon_id_buffer) - 1] = '\0';
    // Note: connection_attempt_triggered is removed, logic handled by thread flags

    std::cout << "[GUI] Entering main GUI loop..." << std::endl;
    // --- Main GUI Loop ---
    while (guiKeepRunning) {
        // --- Handle Connection Thread Completion ---
        if (connectionAttemptFinished.load()) {
            std::cout << "[GUI Loop] Connection attempt finished signal received." << std::endl;
            if (connectionThread.joinable()) {
                connectionThread.join(); // Join the completed thread
                std::cout << "[GUI Loop] Connection thread joined." << std::endl;
            }
            // Update desired state based on thread result
            connect_to_drone = connectionResult.load();
            std::cout << "[GUI Loop] Updated connect_to_drone to: " << std::boolalpha << connect_to_drone << std::endl;

            // Reset flags *after* joining and processing result
            isConnectingToDrone.store(false);
            connectionAttemptFinished.store(false);
        }

        // Poll SDL events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) guiKeepRunning = false;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window)) guiKeepRunning = false;
        }

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Common GUI definitions
        const ImVec4 green(0.0f, 1.0f, 0.0f, 1.0f), red(1.0f, 0.0f, 0.0f, 1.0f), orange(1.0f, 0.65f, 0.0f, 1.0f);
        const float statusScale = 3.0f;
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImVec2 work_pos = viewport->WorkPos;
        ImVec2 work_size = viewport->WorkSize;
        float verticalPadding = 10.0f;

        // Read current flight control status safely
        bool currentFlightControlStatus;
        {
            std::lock_guard<std::mutex> lock(osdkMutex);
            currentFlightControlStatus = enableFlightControl; // Read the actual connected status
        }

        // --- Drone Status Indicator ---
        {
            const char* droneStatusText = currentFlightControlStatus ? "Drone: Connected" : "Drone: Disconnected";
            ImVec4 droneStatusColor = currentFlightControlStatus ? green : red;
            ImVec2 droneTextSize = ImGui::CalcTextSize(droneStatusText); droneTextSize.x *= statusScale; droneTextSize.y *= statusScale;
            ImVec2 droneWindowPos = ImVec2(work_pos.x + (work_size.x - droneTextSize.x) * 0.5f, work_pos.y + verticalPadding);
            ImGui::SetNextWindowPos(droneWindowPos, ImGuiCond_Always); ImGui::SetNextWindowBgAlpha(0.0f);
            ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground;
            ImGui::Begin("DroneStatusIndicator", nullptr, flags);
            ImGui::SetWindowFontScale(statusScale); ImGui::TextColored(droneStatusColor, "%s", droneStatusText); ImGui::SetWindowFontScale(1.0f);
            ImGui::End();
        }
        // --- Radar Bridge Status Indicator ---
        {
            BridgeConnectionStatus radarStatus = currentBridgeStatus.load(); const char* radarStatusText = ""; ImVec4 radarStatusColor = red;
            switch(radarStatus) { case BridgeConnectionStatus::CONNECTED: radarStatusText = "Radar Bridge: Connected"; radarStatusColor = green; break; case BridgeConnectionStatus::DISCONNECTED: radarStatusText = "Radar Bridge: Disconnected"; radarStatusColor = red; break; case BridgeConnectionStatus::RECONNECTING: radarStatusText = "Radar Bridge: Attempting Reconnect..."; radarStatusColor = orange; break; }
            ImVec2 radarTextSize = ImGui::CalcTextSize(radarStatusText); radarTextSize.x *= statusScale; radarTextSize.y *= statusScale;
            float estHeight = ImGui::CalcTextSize("X").y * statusScale + 5.0f; // Estimate height
            ImVec2 radarWindowPos = ImVec2(work_pos.x + (work_size.x - radarTextSize.x) * 0.5f, work_pos.y + verticalPadding + estHeight);
            ImGui::SetNextWindowPos(radarWindowPos, ImGuiCond_Always); ImGui::SetNextWindowBgAlpha(0.0f);
            ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground;
            ImGui::Begin("RadarStatusIndicator", nullptr, flags);
            ImGui::SetWindowFontScale(statusScale); ImGui::TextColored(radarStatusColor, "%s", radarStatusText); ImGui::SetWindowFontScale(1.0f);
            ImGui::End();
        }

        // --- Main Menu Window ---
        ImGui::Begin("Main Menu");
        {
            // Disable button if not connected
            if (!currentFlightControlStatus) { ImGui::BeginDisabled(); }
            // Wall Following Button
            if (ImGui::Button("Start Wall+Beacon Following [w]")) {
                std::cout << "[GUI] 'w' button clicked." << std::endl;
                stopProcessingThreadIfNeeded(); // Stop previous task
                forceStopReconnectionFlag.store(false);
                Vehicle* ptr = nullptr;
                bool auth = false;
                { // Lock scope for accessing vehicle and obtaining authority
                    std::lock_guard<std::mutex> lock(osdkMutex);
                    if (enableFlightControl && vehicle && vehicle->control) { // Check actual connection status and pointers
                        ptr = vehicle;
                        std::cout << "[GUI] Attempting to obtain Control Authority..." << std::endl;
                        int t = 1;
                        ACK::ErrorCode ack = ptr->control->obtainCtrlAuthority(t);
                        if (ACK::getError(ack)) {
                            ACK::getErrorCodeMessage(ack, "[GUI] Obtain Control");
                            std::cerr << "[GUI] Failed to obtain control authority." << std::endl;
                            ptr = nullptr; // Invalidate pointer if failed
                        } else {
                            std::cout << "[GUI] Obtained control authority." << std::endl;
                            auth = true;
                        }
                    } else {
                        std::cerr << "[GUI] Cannot start 'w': Drone not connected or control unavailable." << std::endl;
                    }
                } // Mutex released

                // Start thread only if successfully obtained authority
                if (ptr && auth) {
                    stopProcessingFlag.store(false);
                    processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, ptr, true, ProcessingMode::WALL_FOLLOW);
                } else {
                    // Ensure bridge status reflects reality if thread didn't start
                    currentBridgeStatus.store(BridgeConnectionStatus::DISCONNECTED);
                }
            }
            if (!currentFlightControlStatus) { ImGui::EndDisabled(); ImGui::SameLine(); ImGui::TextDisabled("(Drone Disconnected)"); }

            // Process Full Radar Button (No drone needed)
            if (ImGui::Button("Process Full Radar [e]")) {
                 std::cout << "[GUI] 'e' button clicked." << std::endl;
                 stopProcessingThreadIfNeeded(); // Stop previous task
                 forceStopReconnectionFlag.store(false);
                 std::cout << "[GUI] Starting Radar Data processing (Full, No Control)..." << std::endl;
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, ProcessingMode::PROCESS_FULL);
            }

            // Stop Reconnection Button for Radar Bridge
            if (enable_bridge_reconnection) {
                bool enable_stop = processingThread.joinable() && (currentBridgeStatus.load() == BridgeConnectionStatus::RECONNECTING);
                if (!enable_stop) ImGui::BeginDisabled();
                if (ImGui::Button("Stop Reconnection Attempt")) {
                    std::cout << "[GUI] Stop Reconnect clicked." << std::endl;
                    forceStopReconnectionFlag.store(true); // Signal bridge reconnect loop
                }
                if (!enable_stop) ImGui::EndDisabled();
            }

            // Quit Button
            if (ImGui::Button("Quit [q]")) {
                std::cout << "[GUI] 'q' button clicked." << std::endl;
                guiKeepRunning = false; // Exit GUI loop
            }
        } ImGui::End();

        // --- Parameters Window ---
        {
            ImGuiIO& current_io = ImGui::GetIO(); ImVec2 win_pos = ImVec2(current_io.DisplaySize.x - 10.0f, 10.0f); ImVec2 piv = ImVec2(1.0f, 0.0f);
            ImGui::SetNextWindowPos(win_pos, ImGuiCond_Appearing, piv); ImGui::SetNextWindowBgAlpha(0.65f);
            ImGuiWindowFlags flags_p = ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_AlwaysAutoResize;
            ImGui::Begin("Parameters", nullptr, flags_p);

            ImGui::Text("Status"); ImGui::Separator();
            ImGui::Text("Drone Status: %s", currentFlightControlStatus ? "Connected" : "Disconnected"); // Show actual status
            ImGui::Separator();

            ImGui::Text("Editable Parameters"); ImGui::Separator();
            if (ImGui::InputText("Target Beacon ID", target_beacon_id_buffer, sizeof(target_beacon_id_buffer))) { TARGET_BEACON_ID = target_beacon_id_buffer; std::cout << "[GUI Param Update] Target Beacon ID: " << TARGET_BEACON_ID << std::endl; }
            if (ImGui::InputFloat("Target Distance (m)", &targetDistance, 0.1f, 1.0f, "%.3f")) { std::cout << "[GUI Param Update] Target Distance: " << targetDistance << std::endl; }
            if (ImGui::InputFloat("Target Azimuth (deg)", &targetAzimuth, 1.0f, 10.0f, "%.3f")) { std::cout << "[GUI Param Update] Target Azimuth: " << targetAzimuth << std::endl; }
            ImGui::Separator(); ImGui::Text("Forward Control:");
            if (ImGui::InputFloat("Kp Forward", &Kp_forward, 0.01f, 0.1f, "%.4f")) { std::cout << "[GUI Param Update] Kp Forward: " << Kp_forward << std::endl; }
            if (ImGui::InputFloat("Max Fwd Speed (m/s)", &max_forward_speed, 0.05f, 0.2f, "%.3f")) { std::cout << "[GUI Param Update] Max Fwd Speed: " << max_forward_speed << std::endl; }
            if (ImGui::InputFloat("Fwd Dead Zone (m)", &forward_dead_zone, 0.01f, 0.1f, "%.3f")) { std::cout << "[GUI Param Update] Fwd Dead Zone: " << forward_dead_zone << std::endl; }
            ImGui::Separator(); ImGui::Text("Lateral Control:");
            if (ImGui::InputFloat("Kp Lateral", &Kp_lateral, 0.001f, 0.01f, "%.5f")) { std::cout << "[GUI Param Update] Kp Lateral: " << Kp_lateral << std::endl; }
            if (ImGui::InputFloat("Max Lat Speed (m/s)", &max_lateral_speed, 0.05f, 0.2f, "%.3f")) { std::cout << "[GUI Param Update] Max Lat Speed: " << max_lateral_speed << std::endl; }
            if (ImGui::InputFloat("Azimuth Dead Zone (deg)", &azimuth_dead_zone, 0.1f, 1.0f, "%.3f")) { std::cout << "[GUI Param Update] Azimuth Dead Zone: " << azimuth_dead_zone << std::endl; }
            ImGui::Separator(); ImGui::Text("Connection:");
            if (ImGui::Checkbox("Bridge Reconnect", &enable_bridge_reconnection)) { std::cout << "[GUI Param Update] Bridge Reconnect: " << std::boolalpha << enable_bridge_reconnection << std::endl; }

            // --- Connect to Drone Checkbox (Uses background thread) ---
            bool connecting_in_progress = isConnectingToDrone.load();
            // Disable checkbox while connection thread is running
            if (connecting_in_progress) { ImGui::BeginDisabled(); }

            // Checkbox reflects the *desired* state (connect_to_drone)
            bool checkbox_value = connect_to_drone;
            if (ImGui::Checkbox("##ConnectDroneCheckbox", &checkbox_value)) {
                // This block executes ONLY when the checkbox is clicked
                if (checkbox_value) { // User clicked to check (wants connection)
                    // Start connection thread ONLY if not already connecting AND not already connected
                    if (!connecting_in_progress && !currentFlightControlStatus) {
                        std::cout << "[GUI] Connect checkbox checked. Starting connection thread..." << std::endl;
                        isConnectingToDrone.store(true);       // Set flag: connection starting
                        connectionAttemptFinished.store(false); // Reset finished flag
                        connectionResult.store(false);          // Reset result flag

                        // Join previous thread if it exists and is joinable (shouldn't happen often here, but safe)
                        if (connectionThread.joinable()) {
                             connectionThread.join();
                        }
                        // Start the new connection thread
                        connectionThread = std::thread(connectionTask, argc, argv); // Pass argc, argv
                        connect_to_drone = true; // Update desired state immediately

                    } else if (connecting_in_progress) {
                         std::cout << "[GUI] Connection attempt already in progress." << std::endl;
                         // Don't change connect_to_drone, let the current attempt finish
                    } else { // Already connected
                         std::cout << "[GUI] Already connected." << std::endl;
                         connect_to_drone = true; // Ensure desired state is true
                    }
                } else { // User clicked to uncheck (wants disconnection)
                    // Only disconnect if NOT connecting AND currently connected
                    if (!connecting_in_progress && currentFlightControlStatus) {
                        std::cout << "[GUI] Connect checkbox unchecked. Disconnecting..." << std::endl;
                        deinitializeOSDK();       // Disconnect synchronously
                        connect_to_drone = false; // Update desired state
                        std::cout << "[GUI] Disconnected from drone." << std::endl;
                    } else if (connecting_in_progress) {
                        // If user unchecks while connecting, just update desired state.
                        // We cannot easily cancel initializeOSDK. The thread will finish.
                        std::cout << "[GUI] Connect checkbox unchecked while connection attempt in progress. Desired state set to disconnected." << std::endl;
                        connect_to_drone = false; // Update desired state
                    } else { // Already disconnected
                         std::cout << "[GUI] Already disconnected." << std::endl;
                         connect_to_drone = false; // Ensure desired state is false
                    }
                }
            }
            // End disabled state
            if (connecting_in_progress) { ImGui::EndDisabled(); }

            // Visible label and status text based on connection thread state
            ImGui::SameLine();
            if (connecting_in_progress) {
                ImGui::Text("Connecting to Drone...");
            } else {
                ImGui::Text("Connect to Drone");
            }
            // --- End Connect to Drone Checkbox ---

          ImGui::End();
        }

        // Rendering
        ImGui::Render();
        int dw, dh; SDL_GetWindowSize(window, &dw, &dh);
        glViewport(0, 0, dw, dh);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);

    } // End GUI loop

    // --- Cleanup ---
    std::cout << "[GUI] Exited main GUI loop. Cleaning up..." << std::endl;

    // Ensure connection thread is joined if it was running
    if (connectionThread.joinable()) {
        std::cout << "[Cleanup] Joining connection thread..." << std::endl;
        connectionThread.join();
        std::cout << "[Cleanup] Connection thread joined." << std::endl;
    }

    // Stop processing thread and monitoring thread, deinitialize OSDK
    deinitializeOSDK(); // This now handles stopping processing/monitoring threads internally

    // Cleanup GUI
    std::cout << "[GUI] Cleaning up ImGui..." << std::endl;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    // Cleanup SDL
    std::cout << "[GUI] Cleaning up SDL..." << std::endl;
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::cout << "Application finished with exit code: 0" << std::endl;
    return 0;
}
