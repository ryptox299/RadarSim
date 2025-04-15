// --- Existing Includes ---
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
#include <fstream> // For file reading
#include <cmath> // For std::abs, std::isnan, pow, sqrt, tan, atan, M_PI
#include <atomic> // For thread-safe stop flag
#include <cstring> // For strchr
#include <stdexcept> // For standard exceptions
#include <array> // For std::array used in read_some buffer
#include <ctime>  // For checking polling timestamp
#include <mutex> // For protecting GUI log buffer
#include <memory> // For std::unique_ptr
#include <future> // For std::async

// Include the headers that define Control flags, CtrlData, FlightController, and Vehicle
#include "dji_control.hpp"           // Defines Control class, CtrlData, enums
#include "dji_flight_controller.hpp" // Defines FlightController
#include "dji_vehicle.hpp"           // Defines Vehicle class which contains Control*
#include "dji_telemetry.hpp"         // For Telemetry types
#include "dji_status.hpp"            // For VehicleStatus enums/constants
#include "dji_ack.hpp"               // For ACK::getError

// --- ImGui / Backend Includes ---
#include "vendor/imgui/imgui.h" // Adjusted path
#include "vendor/imgui/backends/imgui_impl_sdl2.h" // Adjusted path
#include "vendor/imgui/backends/imgui_impl_opengl3.h" // Adjusted path
#include <SDL.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

// Define M_PI if not already defined (likely in cmath)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace DJI::OSDK;
using json = nlohmann::json;
using boost::asio::ip::tcp;

// --- Configurable Parameters (with defaults) ---
std::string TARGET_BEACON_ID = "BEACON-TX-ID:00005555";
float targetDistance = 8.0f;        // Target distance from the wall (meters)
float targetAzimuth = 0.0f;         // *Initial* target azimuth relative to the beacon (degrees)
// Forward Control (X-velocity)
float Kp_forward = 0.5;             // Proportional gain for forward movement
float max_forward_speed = 0.8;      // Max speed towards/away from the wall
float forward_dead_zone = 0.2;      // Dead zone for forward movement (meters)
// Lateral Control (Y-velocity)
float Kp_lateral = 0.02;            // Proportional gain for lateral movement
float max_lateral_speed = 0.5;      // Max speed sideways
float azimuth_dead_zone = 1.5;      // Dead zone for *initial* lateral movement (degrees)
// Descent/Ascent Parameters
float position_distance_tolerance = 0.3; // Allowed distance error for starting descent (meters)
float position_azimuth_tolerance = 2.0;  // Allowed azimuth error for starting descent & switching phases (degrees)
float descent_speed = 0.3;               // Speed for descending (m/s, positive value)
float min_floor_altitude = 0.5;          // Minimum altitude to maintain (meters AGL)
float target_beacon_range = 3.0;         // *Base* target range from beacon for ascent phase (meters, at zero azimuth offset)
float beacon_range_tolerance = 0.2;      // Allowed error for final beacon range (meters)
float ascent_speed = 0.3;                // Speed for ascending (m/s, positive value)
// Complex Vertical Test Parameters
int complex_cycles = 3;                  // Number of descent/ascent cycles
float lateral_step_meters = 1.0;         // Desired lateral displacement per alignment step (meters)
// --- End Configurable Parameters ---


// Persistent variables for tracking the current second and wall/beacon candidate data
std::string currentSecond = "";
float lowestRange = std::numeric_limits<float>::max(); // Closest wall candidate range
bool hasAnonData = false;
float targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN(); // Sensed beacon azimuth
float current_beacon_range = std::numeric_limits<float>::quiet_NaN(); // Sensed beacon range
bool foundTargetBeacon = false;

// Global variable for default Python bridge script (Hardcoded)
std::string defaultPythonBridgeScript = "python_bridge.py";

// Flag to control the processing loop
std::atomic<bool> stopProcessingFlag(false);
std::thread processingThread;

// Monitoring Thread Globals
std::atomic<bool> stopMonitoringFlag(false);
std::thread monitoringThread;
const int TELEMETRY_TIMEOUT_SECONDS = 5;

// --- GUI Related Globals ---
SDL_Window* window = nullptr;
SDL_GLContext gl_context = nullptr;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
std::ostringstream guiLogStream; // Buffer for GUI log messages
std::mutex guiLogMutex;          // Mutex to protect the log stream
std::string guiLogBuffer;        // String buffer to hold log content for ImGui

// --- OSDK Related Globals (from original main) ---
int functionTimeout = 1;
Vehicle* vehicle = nullptr; // Owned by linuxEnvironment
std::unique_ptr<FlightSample> flightSample = nullptr; // Use smart pointer
std::unique_ptr<LinuxSetup> linuxEnvironment = nullptr; // Use smart pointer
int telemetrySubscriptionFrequency = 1;
int pkgIndex = 0;
bool monitoringEnabled = false;
bool enableFlightControl = true; // Assume true initially, OSDK init might change it


// --- Helper Function to Log to GUI Buffer ---
// Use this instead of std::cout for messages you want in the GUI log window
void logToGui(const std::string& message) {
    std::lock_guard<std::mutex> lock(guiLogMutex);
    // Optional: Add timestamp or formatting
    guiLogStream << message << std::endl;
    // Also print to console for debugging?
    std::cout << message << std::endl; // Keep console output for now
}


// Function to load preferences
void loadPreferences() {
    logToGui("Loading preferences..."); // Use logToGui
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
                logToGui("Warning: Skipping invalid line in preferences file: " + line); // Use logToGui
                continue;
            }

            std::string key = line.substr(0, equalsPos);
            std::string value = line.substr(equalsPos + 1);

            try {
                // --- Use logToGui for output ---
                if (key == "target_beacon_id") {
                    TARGET_BEACON_ID = value;
                    logToGui("  TARGET_BEACON_ID set to: " + TARGET_BEACON_ID + " (from preferences file)");
                } else if (key == "targetdistance") {
                    targetDistance = std::stof(value);
                    logToGui("  targetDistance set to: " + std::to_string(targetDistance) + " meters (from preferences file)");
                } else if (key == "target_azimuth") {
                    targetAzimuth = std::stof(value);
                    logToGui("  targetAzimuth (Initial) set to: " + std::to_string(targetAzimuth) + " degrees (from preferences file)");
                } else if (key == "kp_forward") {
                    Kp_forward = std::stof(value);
                    logToGui("  Kp_forward set to: " + std::to_string(Kp_forward) + " (from preferences file)");
                } else if (key == "max_forward_speed") {
                    max_forward_speed = std::stof(value);
                    logToGui("  max_forward_speed set to: " + std::to_string(max_forward_speed) + " m/s (from preferences file)");
                } else if (key == "forward_dead_zone") {
                    forward_dead_zone = std::stof(value);
                    logToGui("  forward_dead_zone set to: " + std::to_string(forward_dead_zone) + " meters (from preferences file)");
                } else if (key == "kp_lateral") {
                    Kp_lateral = std::stof(value);
                    logToGui("  Kp_lateral set to: " + std::to_string(Kp_lateral) + " (from preferences file)");
                } else if (key == "max_lateral_speed") {
                    max_lateral_speed = std::stof(value);
                    logToGui("  max_lateral_speed set to: " + std::to_string(max_lateral_speed) + " m/s (from preferences file)");
                } else if (key == "azimuth_dead_zone") {
                    azimuth_dead_zone = std::stof(value);
                    logToGui("  azimuth_dead_zone set to: " + std::to_string(azimuth_dead_zone) + " degrees (from preferences file)");
                } else if (key == "position_distance_tolerance") {
                    position_distance_tolerance = std::stof(value);
                    logToGui("  position_distance_tolerance set to: " + std::to_string(position_distance_tolerance) + " meters (from preferences file)");
                } else if (key == "position_azimuth_tolerance") {
                    position_azimuth_tolerance = std::stof(value);
                    logToGui("  position_azimuth_tolerance set to: " + std::to_string(position_azimuth_tolerance) + " degrees (from preferences file)");
                } else if (key == "descent_speed") {
                    descent_speed = std::stof(value);
                    logToGui("  descent_speed set to: " + std::to_string(descent_speed) + " m/s (from preferences file)");
                } else if (key == "min_floor_altitude") {
                    min_floor_altitude = std::stof(value);
                    logToGui("  min_floor_altitude set to: " + std::to_string(min_floor_altitude) + " meters (from preferences file)");
                } else if (key == "target_beacon_range") {
                    target_beacon_range = std::stof(value);
                    logToGui("  target_beacon_range (Base) set to: " + std::to_string(target_beacon_range) + " meters (from preferences file)");
                } else if (key == "beacon_range_tolerance") {
                    beacon_range_tolerance = std::stof(value);
                    logToGui("  beacon_range_tolerance set to: " + std::to_string(beacon_range_tolerance) + " meters (from preferences file)");
                } else if (key == "ascent_speed") {
                    ascent_speed = std::stof(value);
                    logToGui("  ascent_speed set to: " + std::to_string(ascent_speed) + " m/s (from preferences file)");
                } else if (key == "complex_cycles") {
                    complex_cycles = std::stoi(value);
                    logToGui("  complex_cycles set to: " + std::to_string(complex_cycles) + " (from preferences file)");
                } else if (key == "lateral_step_meters") { // Renamed from azimuth_step
                    lateral_step_meters = std::stof(value);
                    logToGui("  lateral_step_meters set to: " + std::to_string(lateral_step_meters) + " meters (from preferences file)");
                }
            } catch (const std::invalid_argument& ia) {
                 logToGui("Warning: Invalid number format for key '" + key + "' in preferences file: " + value); // Use logToGui
            } catch (const std::out_of_range& oor) {
                 logToGui("Warning: Value out of range for key '" + key + "' in preferences file: " + value); // Use logToGui
            } catch (...) {
                  logToGui("Warning: Unknown error parsing line for key '" + key + "' in preferences file: " + value); // Use logToGui
            }
        }
        preferencesFile.close();
        logToGui("Finished loading preferences."); // Use logToGui
    } else {
        logToGui("Preferences file ('preferences.txt') not found. Using default values:"); // Use logToGui
        logToGui("  Default TARGET_BEACON_ID: " + TARGET_BEACON_ID);
        logToGui("  Default targetDistance: " + std::to_string(targetDistance) + " meters");
        logToGui("  Default targetAzimuth (Initial): " + std::to_string(targetAzimuth) + " degrees");
        logToGui("  Default Kp_forward: " + std::to_string(Kp_forward));
        logToGui("  Default max_forward_speed: " + std::to_string(max_forward_speed) + " m/s");
        logToGui("  Default forward_dead_zone: " + std::to_string(forward_dead_zone) + " meters");
        logToGui("  Default Kp_lateral: " + std::to_string(Kp_lateral));
        logToGui("  Default max_lateral_speed: " + std::to_string(max_lateral_speed) + " m/s");
        logToGui("  Default azimuth_dead_zone: " + std::to_string(azimuth_dead_zone) + " degrees");
        logToGui("  Default position_distance_tolerance: " + std::to_string(position_distance_tolerance) + " meters");
        logToGui("  Default position_azimuth_tolerance: " + std::to_string(position_azimuth_tolerance) + " degrees");
        logToGui("  Default descent_speed: " + std::to_string(descent_speed) + " m/s");
        logToGui("  Default min_floor_altitude: " + std::to_string(min_floor_altitude) + " meters");
        logToGui("  Default target_beacon_range (Base): " + std::to_string(target_beacon_range) + " meters"); // Updated desc
        logToGui("  Default beacon_range_tolerance: " + std::to_string(beacon_range_tolerance) + " meters");
        logToGui("  Default ascent_speed: " + std::to_string(ascent_speed) + " m/s");
        logToGui("  Default complex_cycles: " + std::to_string(complex_cycles));
        logToGui("  Default lateral_step_meters: " + std::to_string(lateral_step_meters) + " meters"); // Renamed
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
    // --- Use logToGui ---
    for (const auto& obj : objects) {
        logToGui( "Radar Object:\n"
                  "  Timestamp: " + obj.timestamp + "\n"
                  "  Sensor: " + obj.sensor + "\n"
                  "  Source: " + obj.src + "\n"
                  "  ID: " + obj.ID + "\n"
                  "  X: " + std::to_string(obj.X) + " Y: " + std::to_string(obj.Y) + " Z: " + std::to_string(obj.Z) + "\n"
                  "  Xdir: " + std::to_string(obj.Xdir) + " Ydir: " + std::to_string(obj.Ydir) + " Zdir: " + std::to_string(obj.Zdir) + "\n"
                  "  Range: " + std::to_string(obj.Range) + " Range Rate: " + std::to_string(obj.RangeRate) + "\n"
                  "  Power: " + std::to_string(obj.Pwr) + " Azimuth: " + std::to_string(obj.Az) + " Elevation: " + std::to_string(obj.El) + "\n"
                  "  Xsize: " + std::to_string(obj.Xsize) + " Ysize: " + std::to_string(obj.Ysize) + " Zsize: " + std::to_string(obj.Zsize) + "\n"
                  "  Confidence: " + std::to_string(obj.Conf) + "\n"
                  "----------------------------------------");
    }
}

// Display minimal radar object details
void displayRadarObjectsMinimal(const std::vector<RadarObject>& objects) {
    // --- Use logToGui ---
    for (const auto& obj : objects) {
       logToGui( "Radar Object (Minimal):\n"
                 "  Timestamp: " + obj.timestamp + "\n"
                 "  Sensor: " + obj.sensor + "\n"
                 "  ID: " + obj.ID + "\n"
                 "  Range: " + std::to_string(obj.Range) + "\n"
                 "  Azimuth: " + std::to_string(obj.Az) + "\n"
                 "  Elevation: " + std::to_string(obj.El) + "\n"
                 "----------------------------------------");
    }
}

// --- ORIGINAL FUNCTION - DO NOT EDIT (Except logging) ---
void extractBeaconAndWallData(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    // --- Use logToGui ---
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
                    if (hasAnonData) {
                        float difference = lowestRange - targetDistance;
                        if (std::abs(difference) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * difference, max_forward_speed));
                        }
                    }
                    float velocity_y = 0.0f;
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) {
                        float azimuth_error = targetBeaconAzimuth - targetAzimuth;
                        if (std::abs(azimuth_error) > azimuth_dead_zone) {
                            velocity_y = std::max(-max_lateral_speed, std::min(Kp_lateral * azimuth_error, max_lateral_speed));
                        }
                    }
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, 0, 0);
                    logToGui( "Control Status: \n"
                              "TargetWall=" + std::to_string(targetDistance) + " | CurrentWall=" + (hasAnonData ? std::to_string(lowestRange) : "N/A") + "\n"
                              "TargetAzimuth=" + std::to_string(targetAzimuth) + " | CurrentAzimuth=" + (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") + "\n"
                              "Computed Velocity(X=" + std::to_string(velocity_x) + ", Y=" + std::to_string(velocity_y) + ")" );
                    logToGui( "--------------------------------------" );
                    vehicle->control->flightCtrl(ctrlData);
                } else if (hasAnonData || foundTargetBeacon) {
                     logToGui( "(Flight Control Disabled or Not Available)" );
                     logToGui( "--------------------------------------" );
                }
            }
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN(); // Reset beacon range too
            foundTargetBeacon = false;
        }
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) {
                 targetBeaconAzimuth = obj.Az;
                 current_beacon_range = obj.Range; // Store range
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) lowestRange = obj.Range;
        }
         if (stopProcessingFlag.load()) return;
    }
}
// --- END ORIGINAL FUNCTION --


// --- FUNCTION FOR SIMPLE VERTICAL TEST [x] - DO NOT EDIT (Except logging) ---
void extractBeaconAndWallData_FullTest(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    // --- Use logToGui ---
    static bool descend_active = false;
    static bool ascent_active = false;
    static std::thread::id current_thread_id;
    if (current_thread_id != std::this_thread::get_id()) {
        descend_active = false;
        ascent_active = false;
        current_thread_id = std::this_thread::get_id();
        logToGui("[Simple Vert] State reset for new thread run.");
    }

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
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr && vehicle->subscribe != nullptr) {
                    float velocity_x = 0.0f;
                    float velocity_y = 0.0f;
                    float velocity_z = 0.0f;
                    float current_height = -1.0f;
                    std::string vertical_status = "Holding Alt";
                    if (hasAnonData) {
                        float difference = lowestRange - targetDistance;
                        if (std::abs(difference) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * difference, max_forward_speed));
                        }
                    }
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) {
                        float azimuth_error = targetBeaconAzimuth - targetAzimuth; // Using initial targetAzimuth
                        if (std::abs(azimuth_error) > azimuth_dead_zone) { // Use initial dead zone here
                            velocity_y = std::max(-max_lateral_speed, std::min(Kp_lateral * azimuth_error, max_lateral_speed));
                        }
                    }
                    bool distance_ok = hasAnonData && (std::abs(lowestRange - targetDistance) <= position_distance_tolerance);
                    bool azimuth_ok = foundTargetBeacon && !std::isnan(targetBeaconAzimuth) && (std::abs(targetBeaconAzimuth - targetAzimuth) <= position_azimuth_tolerance);
                    if (distance_ok && azimuth_ok && !descend_active && !ascent_active) {
                        logToGui("[Simple Vert] Position confirmed (Dist: " + std::to_string(lowestRange) + ", Az: " + std::to_string(targetBeaconAzimuth) + "). Starting descent.");
                        descend_active = true;
                    }
                    if (descend_active && !ascent_active) {
                        current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                        if (current_height > min_floor_altitude) {
                            velocity_z = -descent_speed;
                            vertical_status = "Descending";
                        } else {
                            velocity_z = 0.0f;
                            vertical_status = "Min Alt Reached, Starting Ascent";
                            logToGui("[Simple Vert] Minimum altitude reached. Starting ascent phase.");
                            ascent_active = true;
                            descend_active = false;
                        }
                    } else if (ascent_active) {
                        current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                        if (foundTargetBeacon && !std::isnan(current_beacon_range)) {
                            if (current_beacon_range > target_beacon_range + beacon_range_tolerance) { // Uses fixed target_beacon_range
                                velocity_z = ascent_speed;
                                vertical_status = "Ascending (Beacon Range)";
                            } else {
                                velocity_z = 0.0f;
                                vertical_status = "Target Beacon Range Reached";
                            }
                        } else {
                            velocity_z = 0.0f;
                            vertical_status = "Ascending (Beacon Lost)";
                            logToGui("[Simple Vert] Warning: Beacon lost during ascent phase. Hovering."); // Use logToGui for cerr
                        }
                    }
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, velocity_z, 0);
                    logToGui( "[Simple Vert] Control Status: \n"
                              "TargetWall=" + std::to_string(targetDistance) + " | CurrentWall=" + (hasAnonData ? std::to_string(lowestRange) : "N/A") + "\n"
                              "TargetAzimuth=" + std::to_string(targetAzimuth) + " | CurrentAzimuth=" + (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") + "\n"
                              "TargetBeaconRange=" + std::to_string(target_beacon_range) + " | CurrentBeaconRange=" + (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") + "\n"
                              "Computed Velocity(X=" + std::to_string(velocity_x) + ", Y=" + std::to_string(velocity_y) + ", Z=" + std::to_string(velocity_z) + ")\n"
                              "Vertical Status: " + vertical_status + " | Current Height: " + (current_height >= 0.0f ? std::to_string(current_height) : "N/A"));
                    logToGui("--------------------------------------");
                    vehicle->control->flightCtrl(ctrlData);
                } else if (hasAnonData || foundTargetBeacon) {
                     std::string status = "[Simple Vert] (Flight Control Disabled or Not Available)";
                     if (enableControl && vehicle && !vehicle->subscribe) {
                         status = "[Simple Vert] (Vehicle Subscribe Not Available - Cannot get height)";
                     }
                     logToGui(status);
                     logToGui( "TargetWall=" + std::to_string(targetDistance) + " | CurrentWall=" + (hasAnonData ? std::to_string(lowestRange) : "N/A") + "\n"
                               "TargetAzimuth=" + std::to_string(targetAzimuth) + " | CurrentAzimuth=" + (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") + "\n"
                               "TargetBeaconRange=" + std::to_string(target_beacon_range) + " | CurrentBeaconRange=" + (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A"));
                     logToGui("--------------------------------------");
                }
            }
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN();
            foundTargetBeacon = false;
        }
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
        }
         if (stopProcessingFlag.load()) return;
    }
}
// --- END FUNCTION FOR SIMPLE VERTICAL TEST ---


// --- COMPLEX VERTICAL TEST FUNCTION [c] ---
// --- Use logToGui ---
void extractBeaconAndWallData_ComplexVertical(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {

    // State for the complex vertical test
    enum class ComplexPhase { INITIAL_POSITIONING, DESCENDING, ALIGNING_FOR_ASCENT, ASCENDING, ALIGNING_FOR_DESCENT, FINISHED };
    static ComplexPhase current_phase = ComplexPhase::INITIAL_POSITIONING;
    static int current_cycle = 0;
    static float current_target_lateral_offset_meters = 0.0f; // Cumulative desired lateral offset (m)
    static float current_target_azimuth_offset = 0.0f;      // Calculated total azimuth offset (deg) needed

    // Reset state if thread restarts
    static std::thread::id current_thread_id;
    if (current_thread_id != std::this_thread::get_id()) {
        current_phase = ComplexPhase::INITIAL_POSITIONING;
        current_cycle = 0;
        current_target_lateral_offset_meters = 0.0f;
        current_target_azimuth_offset = 0.0f;
        current_thread_id = std::this_thread::get_id();
        logToGui("[Complex Vert] State reset for new thread run.");
    }

    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return;

        // Extract second from timestamp
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') ts_cleaned.erase(0, 1);
        if (!ts_cleaned.empty() && ts_cleaned.back() == '"') ts_cleaned.pop_back();
        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        objSecond = (dotPos != std::string::npos) ? ts_cleaned.substr(0, dotPos) : ts_cleaned;

        // Skip old data
        if (!currentSecond.empty() && objSecond < currentSecond) {
            continue;
        }

        // Process data from previous second if new second detected
        if (objSecond != currentSecond) {
            if (!currentSecond.empty() && (hasAnonData || foundTargetBeacon)) {

                // --- Movement Logic --
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr && vehicle->subscribe != nullptr) {
                    float velocity_x = 0.0f;
                    float velocity_y = 0.0f;
                    float velocity_z = 0.0f;
                    float current_height = -1.0f; // AGL
                    std::string phase_status_str = "Unknown";
                    float runtime_target_slant_range = target_beacon_range; // Default to base range
                    float azimuth_error = std::numeric_limits<float>::quiet_NaN(); // Initialize azimuth error

                    // --- Always calculate Horizontal Velocities ---
                    if (hasAnonData) {
                        float distance_error = lowestRange - targetDistance;
                        if (std::abs(distance_error) > forward_dead_zone) {
                            velocity_x = std::max(-max_forward_speed, std::min(Kp_forward * distance_error, max_forward_speed));
                        }
                    }

                    // Calculate the total target azimuth (initial + offset) in degrees
                    float runtime_target_azimuth = targetAzimuth + current_target_azimuth_offset;
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) {
                        azimuth_error = targetBeaconAzimuth - runtime_target_azimuth; // Calculate error based on *current* target azimuth
                        if (std::abs(azimuth_error) > position_azimuth_tolerance) { // Use tolerance as deadband
                             velocity_y = std::max(-max_lateral_speed, std::min(Kp_lateral * azimuth_error, max_lateral_speed));
                        }
                    } else if (current_phase != ComplexPhase::INITIAL_POSITIONING && current_phase != ComplexPhase::FINISHED) {
                        velocity_y = 0.0f;
                         logToGui("[Complex Vert] Warning: Beacon lost during active phase. Halting lateral motion."); // Use logToGui
                    }
                    // --- End Horizontal Velocity Calculation ---


                    // --- Phase-Based Vertical Velocity and State Transitions ---
                    // Recalculate alignment based on potentially non-NaN azimuth_error
                    bool is_azimuth_aligned = foundTargetBeacon && !std::isnan(azimuth_error) && (std::abs(azimuth_error) <= position_azimuth_tolerance);
                    bool is_distance_aligned = hasAnonData && (std::abs(lowestRange - targetDistance) <= position_distance_tolerance);
                    // is_beacon_range_met is now declared and calculated *inside* ASCENDING case

                    switch (current_phase) {
                        case ComplexPhase::INITIAL_POSITIONING:
                            phase_status_str = "Initial Positioning";
                            velocity_z = 0.0f;
                            // Use initial azimuth dead zone for the very first alignment check
                            if (is_distance_aligned && foundTargetBeacon && !std::isnan(azimuth_error) && (std::abs(azimuth_error) <= azimuth_dead_zone)) {
                                logToGui("[Complex Vert] Initial position confirmed (Dist: " + std::to_string(lowestRange) + ", Az: " + std::to_string(targetBeaconAzimuth) + "). Starting Cycle " + std::to_string(current_cycle + 1) + "...");
                                current_phase = ComplexPhase::DESCENDING;
                            }
                            break;

                        case ComplexPhase::DESCENDING:
                            phase_status_str = "Descending (Cycle " + std::to_string(current_cycle + 1) + ")";
                            current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
                            if (current_height > min_floor_altitude) {
                                velocity_z = -descent_speed;
                            } else {
                                velocity_z = 0.0f;
                                // Increment desired lateral offset
                                current_target_lateral_offset_meters += lateral_step_meters;
                                // Calculate the required azimuth offset (degrees) for this lateral offset
                                if (std::abs(targetDistance) > 1e-6) { // Avoid division by zero
                                    current_target_azimuth_offset = std::atan(current_target_lateral_offset_meters / targetDistance) * 180.0 / M_PI;
                                } else {
                                    current_target_azimuth_offset = 0.0; // Or handle error appropriately
                                    logToGui("[Complex Vert] Error: targetDistance is near zero, cannot calculate target azimuth."); // Use logToGui
                                }
                                runtime_target_azimuth = targetAzimuth + current_target_azimuth_offset; // Update for logging
                                logToGui("[Complex Vert] Min altitude reached. New Target Lateral Offset: " + std::to_string(current_target_lateral_offset_meters) + "m.");
                                logToGui("[Complex Vert] Aligning for ascent (Calculated Target Az Total: " + std::to_string(runtime_target_azimuth) + " deg).");
                                current_phase = ComplexPhase::ALIGNING_FOR_ASCENT;
                            }
                            break;

                        case ComplexPhase::ALIGNING_FOR_ASCENT:
                            phase_status_str = "Aligning for Ascent (Cycle " + std::to_string(current_cycle + 1) + ")";
                            velocity_z = 0.0f;
                            current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>(); // Get height for logging
                            if (is_azimuth_aligned) { // Check alignment against calculated runtime target azimuth
                                logToGui("[Complex Vert] Azimuth aligned (Az: " + std::to_string(targetBeaconAzimuth) + "). Starting ascent.");
                                current_phase = ComplexPhase::ASCENDING;
                            }
                            // else: velocity_x and velocity_y continue to drive alignment
                            break;

                        case ComplexPhase::ASCENDING:
                            { // <<<--- Start new scope for case variables
                                phase_status_str = "Ascending (Cycle " + std::to_string(current_cycle + 1) + ")";
                                current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>(); // Get height for logging

                                // Calculate Dynamic Target Slant Range for this phase
                                {
                                    // Use the current runtime_target_azimuth (calculated from lateral offset)
                                    double runtime_azimuth_rad = runtime_target_azimuth * M_PI / 180.0;
                                    if (std::abs(std::cos(runtime_azimuth_rad)) < 1e-6) {
                                        runtime_azimuth_rad = std::copysign(M_PI / 2.0 - 1e-6, runtime_azimuth_rad);
                                    }
                                    // Y_target is the *current* desired lateral offset
                                    double Y_target = current_target_lateral_offset_meters;
                                    // Calculate required slant range based on base range (vertical height) and current lateral offset
                                    runtime_target_slant_range = static_cast<float>(std::sqrt(std::pow(target_beacon_range, 2) + std::pow(Y_target, 2)));
                                }
                                // Update range met condition based on dynamic range - MOVED INSIDE CASE
                                bool is_beacon_range_met = foundTargetBeacon && !std::isnan(current_beacon_range) && (current_beacon_range <= runtime_target_slant_range + beacon_range_tolerance);

                                if (foundTargetBeacon && !std::isnan(current_beacon_range)) {
                                    if (!is_beacon_range_met) {
                                        velocity_z = ascent_speed;
                                    } else {
                                        // Target dynamic range met
                                        velocity_z = 0.0f;
                                        current_cycle++;
                                        logToGui("[Complex Vert] Target beacon range reached (Range: " + std::to_string(current_beacon_range) + ", Target: " + std::to_string(runtime_target_slant_range) + "). Cycle " + std::to_string(current_cycle) + " complete.");
                                        if (current_cycle >= complex_cycles) {
                                            current_phase = ComplexPhase::FINISHED;
                                            logToGui("[Complex Vert] All cycles finished.");
                                        } else {
                                            // Increment desired lateral offset for next descent
                                            current_target_lateral_offset_meters += lateral_step_meters;
                                            // Calculate the required azimuth offset (degrees)
                                            if (std::abs(targetDistance) > 1e-6) {
                                                current_target_azimuth_offset = std::atan(current_target_lateral_offset_meters / targetDistance) * 180.0 / M_PI;
                                            } else {
                                                current_target_azimuth_offset = 0.0;
                                                logToGui("[Complex Vert] Error: targetDistance is near zero, cannot calculate target azimuth."); // Use logToGui
                                            }
                                            runtime_target_azimuth = targetAzimuth + current_target_azimuth_offset; // Update for logging
                                            logToGui("[Complex Vert] New Target Lateral Offset: " + std::to_string(current_target_lateral_offset_meters) + "m.");
                                            logToGui("[Complex Vert] Aligning for next descent (Calculated Target Az Total: " + std::to_string(runtime_target_azimuth) + " deg).");
                                            current_phase = ComplexPhase::ALIGNING_FOR_DESCENT;
                                        }
                                    }
                                } else {
                                    // Beacon lost during ascent
                                    velocity_z = 0.0f;
                                    phase_status_str = "Ascending (Beacon Lost)";
                                    logToGui("[Complex Vert] Warning: Beacon lost during ascent. Hovering vertically."); // Use logToGui
                                }
                            } // <<<--- End new scope for case variables
                            break;

                        case ComplexPhase::ALIGNING_FOR_DESCENT:
                             phase_status_str = "Aligning for Descent (Cycle " + std::to_string(current_cycle + 1) + ")";
                             velocity_z = 0.0f;
                             current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>(); // Get height for logging
                             if (is_azimuth_aligned) { // Check alignment against calculated runtime target azimuth
                                 logToGui("[Complex Vert] Azimuth aligned (Az: " + std::to_string(targetBeaconAzimuth) + "). Starting descent.");
                                 current_phase = ComplexPhase::DESCENDING;
                             }
                             // else: velocity_x and velocity_y continue to drive alignment
                             break;

                        case ComplexPhase::FINISHED:
                            phase_status_str = "Finished";
                            velocity_x = 0.0f;
                            velocity_y = 0.0f;
                            velocity_z = 0.0f;
                            current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>(); // Get height for logging
                            break;
                    }
                    // --- End Phase Logic ---

                    // --- Send Command ---
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, velocity_z, 0);

                    // Log control status summary - Updated Azimuth/Lateral info
                    logToGui("[Complex Vert] Control Status: \n"
                              "Cycle: " + std::to_string(current_cycle) + "/" + std::to_string(complex_cycles) + " | Phase: " + phase_status_str + "\n"
                              "TargetWall=" + std::to_string(targetDistance) + " | CurrentWall=" + (hasAnonData ? std::to_string(lowestRange) : "N/A") + "\n"
                              "TargetLateralOffset=" + std::to_string(current_target_lateral_offset_meters) + "m | TargetAzTotal(calc)=" + std::to_string(runtime_target_azimuth) + "deg | CurrentAz=" + (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") + "\n"
                              " | AzError=" + (!std::isnan(azimuth_error) ? std::to_string(azimuth_error) : "N/A") + "\n"
                              "TargetBeaconRange(Dyn)=" + std::to_string(runtime_target_slant_range) + " | CurrentBeaconRange=" + (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") + "\n"
                              "Computed Velocity(X=" + std::to_string(velocity_x) + ", Y=" + std::to_string(velocity_y) + ", Z=" + std::to_string(velocity_z) + ")\n"
                              "Current Height: " + (current_height >= 0.0f ? std::to_string(current_height) : "N/A"));
                    logToGui("--------------------------------------");

                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasAnonData || foundTargetBeacon) { // Control disabled or unavailable
                     // ... (logging for disabled control remains the same, maybe update az display) ...
                     std::string status = "[Complex Vert] (Flight Control Disabled or Not Available)";
                     if (enableControl && vehicle && !vehicle->subscribe) {
                         status = "[Complex Vert] (Vehicle Subscribe Not Available - Cannot get height)";
                     }
                     logToGui(status);
                     float runtime_target_azimuth = targetAzimuth + current_target_azimuth_offset;
                     float runtime_target_slant_range = target_beacon_range;
                      if (current_phase == ComplexPhase::ASCENDING || current_phase == ComplexPhase::ALIGNING_FOR_DESCENT) {
                            double Y_target = current_target_lateral_offset_meters;
                            runtime_target_slant_range = static_cast<float>(std::sqrt(std::pow(target_beacon_range, 2) + std::pow(Y_target, 2)));
                      }
                      float azimuth_error = std::numeric_limits<float>::quiet_NaN();
                      if(foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) {
                          azimuth_error = targetBeaconAzimuth - runtime_target_azimuth;
                      }
                     logToGui( "Cycle: " + std::to_string(current_cycle) + "/" + std::to_string(complex_cycles) + " | Phase: " + std::to_string(static_cast<int>(current_phase)) + "\n" // Log phase enum value
                               "TargetWall=" + std::to_string(targetDistance) + " | CurrentWall=" + (hasAnonData ? std::to_string(lowestRange) : "N/A") + "\n"
                               "TargetLateralOffset=" + std::to_string(current_target_lateral_offset_meters) + "m | TargetAzTotal(calc)=" + std::to_string(runtime_target_azimuth) + "deg | CurrentAz=" + (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") + "\n"
                               " | AzError=" + (!std::isnan(azimuth_error) ? std::to_string(azimuth_error) : "N/A") + "\n"
                               "TargetBeaconRange(Dyn)=" + std::to_string(runtime_target_slant_range) + " | CurrentBeaconRange=" + (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A"));
                     logToGui("--------------------------------------");
                }
            }

            // Update to new second and reset per-second state variables for next cycle's accumulation
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
            current_beacon_range = std::numeric_limits<float>::quiet_NaN(); // Reset beacon range
            foundTargetBeacon = false;
        }

        // --- Data Accumulation for Current Second ---
        if (obj.ID == TARGET_BEACON_ID) {
            if (!foundTargetBeacon) { // Store first detection's data
                 targetBeaconAzimuth = obj.Az;
                 current_beacon_range = obj.Range; // Store range
                 foundTargetBeacon = true;
            }
        }
        else if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) lowestRange = obj.Range;
        }
         if (stopProcessingFlag.load()) return;
    }
}
// --- END COMPLEX VERTICAL TEST FUNCTION ---


// Parses JSON radar data
std::vector<RadarObject> parseRadarData(const std::string& jsonData) {
    // --- Use logToGui ---
    std::vector<RadarObject> radarObjects;
    if (jsonData.empty() || jsonData == "{}") return radarObjects;

    try {
        auto jsonFrame = json::parse(jsonData);
        if (!jsonFrame.contains("objects") || !jsonFrame["objects"].is_array()) return radarObjects;

        for (const auto& obj : jsonFrame["objects"]) {
            if (!obj.is_object()) {
                logToGui("Skipping non-object item in 'objects' array."); // Use logToGui
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
        logToGui("JSON Parsing Error: " + std::string(e.what()) + " at offset " + std::to_string(e.byte) + ". Data: [" + jsonData.substr(0, 200) + "...]"); // Use logToGui
        return {};
    } catch (const json::type_error& e) {
        logToGui("JSON Type Error: " + std::string(e.what()) + ". Data: [" + jsonData.substr(0, 200) + "...]"); // Use logToGui
        return {};
    }
    return radarObjects;
}

// Runs the python bridge script
void runPythonBridge(const std::string& scriptName) {
    // --- Use logToGui ---
    logToGui("Starting Python bridge (" + scriptName + ")...");
    if (std::system(("python3 " + scriptName + " &").c_str()) != 0) {
        logToGui("Failed to start Python bridge script '" + scriptName + "'.");
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    logToGui("Python bridge potentially started.");
}

// Stops the python bridge script
void stopPythonBridge(const std::string& scriptName) {
    // --- Use logToGui ---
    logToGui("Stopping Python bridge (" + scriptName + ")...");
    std::system(("pkill -f " + scriptName).c_str()); // Ignore result
    std::this_thread::sleep_for(std::chrono::seconds(1));
    logToGui("Sent SIGTERM to " + scriptName + ".");
}

// Connects to the python bridge via TCP
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket) {
    // --- Use logToGui ---
    tcp::resolver resolver(io_context);
    int retries = 5;
    while (retries-- > 0) {
        try {
            if(socket.is_open()) { socket.close(); } // Simpler close
            auto endpoints = resolver.resolve("127.0.0.1", "5000");
            boost::system::error_code ec;
            boost::asio::connect(socket, endpoints, ec);
            if (!ec) { logToGui("Connected to Python bridge."); return true; }
            else { logToGui("Connection attempt failed: " + ec.message()); }
        } catch (const std::exception& e) {
            logToGui("Exception during connection attempt: " + std::string(e.what()));
        }
        if(retries > 0 && !stopProcessingFlag.load()) {
             logToGui("Retrying connection in 2 seconds... (" + std::to_string(retries) + " attempts left)");
             for (int i = 0; i < 2 && !stopProcessingFlag.load(); ++i) std::this_thread::sleep_for(std::chrono::seconds(1));
             if (stopProcessingFlag.load()) { logToGui("Stop requested during connection retry."); break; }
        } else if (stopProcessingFlag.load()) { logToGui("Stop requested during connection retry."); break; }
    }
     logToGui("Failed to connect to Python bridge after multiple attempts.");
     return false;
}

// Placeholder Callbacks
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }
void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) { /* Unused */ }

// Enum for processing modes
enum class ProcessingMode {
    NONE, // Add a none state
    WALL_FOLLOW,
    PROCESS_FULL,
    PROCESS_MINIMAL,
    WALL_FOLLOW_FULL_TEST,        // Mode for Simple Vertical Test [x]
    WALL_FOLLOW_COMPLEX_VERTICAL // Mode for Complex Vertical Test [c]
};

// Processing loop function - No reconnection, releases authority on exit
void processingLoopFunction(const std::string bridgeScriptName, Vehicle* vehiclePtr, bool enableControlCmd, ProcessingMode mode) {
    // --- Use logToGui ---
    logToGui("Processing thread started. Bridge: " + bridgeScriptName + ", Control Enabled: " + (enableControlCmd ? "true" : "false") + ", Mode: " + std::to_string(static_cast<int>(mode)));
    int localFunctionTimeout = 1; // Use local variable

    runPythonBridge(bridgeScriptName);
    boost::asio::io_context io_context;
    tcp::socket socket(io_context);

    if (!connectToPythonBridge(io_context, socket)) {
        logToGui("Processing thread: Initial connection failed. Exiting thread.");
        stopPythonBridge(bridgeScriptName);
        if (enableControlCmd && vehiclePtr != nullptr && vehiclePtr->control != nullptr) {
            logToGui("[Processing Thread] Releasing control authority (initial connection fail)...");
            ACK::ErrorCode releaseAck = vehiclePtr->control->releaseCtrlAuthority(localFunctionTimeout);
            if (ACK::getError(releaseAck)) ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority");
            else logToGui("[Processing Thread] Control authority released.");
        }
        return; // Exit thread
    }

    logToGui("Processing thread: Connection successful. Reading data stream...");
    std::string received_data_buffer;
    std::array<char, 4096> read_buffer;
    bool connection_error_occurred = false;

    // Reset state variables used in processing functions
    currentSecond = ""; lowestRange = std::numeric_limits<float>::max(); hasAnonData = false;
    targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN();
    current_beacon_range = std::numeric_limits<float>::quiet_NaN();
    foundTargetBeacon = false;

    while (!stopProcessingFlag.load()) {
        boost::system::error_code error;
        size_t len = socket.read_some(boost::asio::buffer(read_buffer), error);

        if (error) { // Handle read error
            if (error == boost::asio::error::eof) logToGui("Processing thread: Connection closed by Python bridge (EOF).");
            else logToGui("Processing thread: Error reading from socket: " + error.message());
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
                            // --- Switch Case ---
                            switch (mode) {
                                case ProcessingMode::WALL_FOLLOW:
                                    extractBeaconAndWallData(radarObjects, vehiclePtr, enableControlCmd);
                                    break;
                                case ProcessingMode::PROCESS_FULL:
                                    displayRadarObjects(radarObjects);
                                    break;
                                case ProcessingMode::PROCESS_MINIMAL:
                                    displayRadarObjectsMinimal(radarObjects);
                                    break;
                                case ProcessingMode::WALL_FOLLOW_FULL_TEST: // Simple Vertical
                                    extractBeaconAndWallData_FullTest(radarObjects, vehiclePtr, enableControlCmd);
                                    break;
                                case ProcessingMode::WALL_FOLLOW_COMPLEX_VERTICAL: // Complex Vertical
                                    extractBeaconAndWallData_ComplexVertical(radarObjects, vehiclePtr, enableControlCmd);
                                    break;
                                case ProcessingMode::NONE: // Should not happen if started correctly
                                     logToGui("[Processing Thread] Error: ProcessingMode::NONE encountered.");
                                     break;

                            }
                        }
                    } catch (const std::exception& e) {
                         logToGui("Error processing data: " + std::string(e.what()) + "\nSnippet: [" + jsonData.substr(0, 100) + "...]");
                    }
                }
            }
            if (stopProcessingFlag.load()) break;
        } else {
             std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Brief pause if no data
        }
    }

    // --- Cleanup ---
    if (stopProcessingFlag.load()) logToGui("[Processing Thread] Stop requested manually.");
    else if (connection_error_occurred) logToGui("[Processing Thread] Exiting due to connection error.");
    else logToGui("[Processing Thread] Data stream ended.");

    if (socket.is_open()) { socket.close(); }
    stopPythonBridge(bridgeScriptName);

    // Release Control Authority if enabled
    if (enableControlCmd && vehiclePtr != nullptr && vehiclePtr->control != nullptr) {
        logToGui("[Processing Thread] Sending final zero velocity command...");
        uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
        DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
        vehiclePtr->control->flightCtrl(stopData);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        logToGui("[Processing Thread] Releasing control authority...");
        ACK::ErrorCode releaseAck = vehiclePtr->control->releaseCtrlAuthority(localFunctionTimeout);
        if (ACK::getError(releaseAck)) {
             ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority (on exit)");
             logToGui("[Processing Thread] Warning: Failed to release control authority on exit."); // Use logToGui
        } else {
             logToGui("[Processing Thread] Control authority released.");
        }
    }
    logToGui("Processing thread finished."); // Use logToGui
}

// Helper function to get mode name string
std::string getModeName(uint8_t mode) {
    // ... (remains unchanged) ...
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
void monitoringLoopFunction(Vehicle* vehiclePtr) {
    // --- Use logToGui ---
    logToGui("[Monitoring] Thread started.");
    bool telemetry_timed_out = false;
    bool warned_unexpected_status = false;
    uint8_t previous_flight_status = DJI::OSDK::VehicleStatus::FlightStatus::STOPED;
    time_t last_valid_poll_time = 0;
    bool in_sdk_control_mode = false;
    bool warned_not_in_sdk_mode = false;
    const uint8_t EXPECTED_SDK_MODE = DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL;

    while (!stopMonitoringFlag.load()) {
        if (vehiclePtr == nullptr || vehiclePtr->subscribe == nullptr) {
             if (!telemetry_timed_out) {
                 logToGui("\n**** MONITORING ERROR: Vehicle/subscribe object null. Stopping. ****\n");
                 telemetry_timed_out = true;
             }
             break;
        }

        uint8_t current_flight_status = vehiclePtr->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
        uint8_t current_display_mode = vehiclePtr->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        bool valid_poll = (current_flight_status <= DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR);

        if (valid_poll) {
            time_t current_time = std::time(nullptr);
            last_valid_poll_time = current_time;
            if (telemetry_timed_out) {
                 logToGui("[Monitoring] Telemetry poll recovered.");
                 telemetry_timed_out = false;
            }

            // Check Flight Status Change
            if (current_flight_status != DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) {
                if ( (previous_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
                     (current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND || current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::STOPED) &&
                     !warned_unexpected_status) {
                     logToGui("\n**** MONITORING WARNING: Flight status changed unexpectedly from IN_AIR to " + std::to_string((int)current_flight_status) + ". ****\n");
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
                    logToGui("\n**** MONITORING INFO: Entered SDK Control Mode (" + getModeName(EXPECTED_SDK_MODE) + " / " + std::to_string((int)EXPECTED_SDK_MODE) + ") ****\n");
                    in_sdk_control_mode = true;
                    warned_not_in_sdk_mode = false;
                }
            } else { // Not in expected mode
                 if (in_sdk_control_mode || !warned_not_in_sdk_mode) {
                      // Check if processing thread is stopping/stopped
                      if (!stopProcessingFlag.load() && processingThread.joinable()) {
                          std::string current_mode_name = getModeName(current_display_mode);
                          logToGui("\n**** MONITORING WARNING: NOT in expected SDK Control Mode (" + getModeName(EXPECTED_SDK_MODE) + "). Current: " + current_mode_name + " (" + std::to_string((int)current_display_mode) + ") ****\n");
                          warned_not_in_sdk_mode = true;
                      }
                      in_sdk_control_mode = false;
                 }
            }
        } else { // Invalid poll
            if (!telemetry_timed_out) {
                 logToGui("\n**** MONITORING WARNING: Polling telemetry returned potentially invalid data (FlightStatus=" + std::to_string((int)current_flight_status) + "). ****\n");
                 telemetry_timed_out = true;
            }
        }

        // Check Telemetry Timeout
        time_t current_time_for_timeout_check = std::time(nullptr);
        if (last_valid_poll_time > 0 && (current_time_for_timeout_check - last_valid_poll_time > TELEMETRY_TIMEOUT_SECONDS)) {
            if (!telemetry_timed_out) {
                logToGui("\n**** MONITORING TIMEOUT: No valid telemetry for over " + std::to_string(TELEMETRY_TIMEOUT_SECONDS) + " seconds. ****\n");
                telemetry_timed_out = true;
            }
        } else if (last_valid_poll_time == 0) { // Check if never received first poll
             static time_t start_time = 0; if (start_time == 0) start_time = current_time_for_timeout_check;
             if (current_time_for_timeout_check - start_time > TELEMETRY_TIMEOUT_SECONDS * 2) {
                  if (!telemetry_timed_out) {
                       logToGui("\n**** MONITORING TIMEOUT: Never received valid telemetry poll after " + std::to_string(TELEMETRY_TIMEOUT_SECONDS * 2) + " seconds. ****\n");
                       telemetry_timed_out = true;
                  }
             }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    logToGui("[Monitoring] Thread finished.");
}


// --- GUI Initialization ---
bool initGui() {
    // Setup SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        logToGui("Error: SDL_Init failed: " + std::string(SDL_GetError()));
        return false;
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
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3); // Use 3.3 for compatibility
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
    window = SDL_CreateWindow("RadarSim Control", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
    if (!window) {
        logToGui("Error: SDL_CreateWindow failed: " + std::string(SDL_GetError()));
        SDL_Quit();
        return false;
    }
    gl_context = SDL_GL_CreateContext(window);
    if (!gl_context) {
        logToGui("Error: SDL_GL_CreateContext failed: " + std::string(SDL_GetError()));
        SDL_DestroyWindow(window);
        SDL_Quit();
        return false;
    }
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    logToGui("GUI Initialized successfully.");
    return true;
}

// --- GUI Cleanup ---
void cleanupGui() {
    // Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    if (ImGui::GetCurrentContext()) { // Check if context exists before destroying
        ImGui::DestroyContext();
    }

    // Cleanup SDL
    if (gl_context) {
       SDL_GL_DeleteContext(gl_context);
       gl_context = nullptr;
    }
    if (window) {
       SDL_DestroyWindow(window);
       window = nullptr;
    }
    SDL_Quit();
    logToGui("GUI Cleaned up.");
}

// --- Forward Declarations for Actions ---
void startProcessing(ProcessingMode mode, bool control, const std::string& bridge = defaultPythonBridgeScript);
void stopProcessing();
void performTakeoff();
void quitApplication(bool& runningFlag);


// --- GUI Rendering Function ---
void renderGui(bool& keepRunning) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // Get main viewport size and position
    const ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    // Use 0.30f for 30% height
    float bottom_panel_height = main_viewport->WorkSize.y * 0.30f;
    float top_panel_height = main_viewport->WorkSize.y - bottom_panel_height;

    // Define window flags for fixed panels
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar |
                                    ImGuiWindowFlags_NoResize |
                                    ImGuiWindowFlags_NoMove |
                                    ImGuiWindowFlags_NoCollapse |
                                    ImGuiWindowFlags_NoScrollbar | // Disable scrollbar for the window itself
                                    ImGuiWindowFlags_NoScrollWithMouse;


    // --- Controls Window (Top Panel) ---
    ImGui::SetNextWindowPos(main_viewport->WorkPos);
    ImGui::SetNextWindowSize(ImVec2(main_viewport->WorkSize.x, top_panel_height));
    ImGui::Begin("Controls Panel", nullptr, window_flags); // Use Begin with flags

    ImGui::Text("Flight Control: %s", (enableFlightControl ? "ENABLED" : "DISABLED"));
    ImGui::Separator();

    if (ImGui::Button("Monitored Takeoff") && enableFlightControl) {
        logToGui("Takeoff button pressed.");
        performTakeoff();
    }
    if (!enableFlightControl && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
         ImGui::SetTooltip("Flight control must be enabled during OSDK initialization.");
    }
    if (!enableFlightControl) { // Keep button disabled text separate
        ImGui::SameLine(); ImGui::TextDisabled("(Disabled)");
    }


    ImGui::Separator();
    ImGui::Text("Wall+Beacon Following:");
    if (ImGui::Button("Start Default (W)")) {
        logToGui("Start Default Wall Following button pressed.");
        startProcessing(ProcessingMode::WALL_FOLLOW, enableFlightControl, defaultPythonBridgeScript);
    }
    if (ImGui::Button("Start TEST (H)") && enableFlightControl) {
         logToGui("Start TEST Wall Following button pressed.");
         startProcessing(ProcessingMode::WALL_FOLLOW, true, "python_bridge_LOCAL.py"); // Force control for test
    }
     if (!enableFlightControl && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) { // Tooltip for disabled test button
         ImGui::SetTooltip("Enable Flight Control during OSDK init to use TEST mode.");
    }
    if (!enableFlightControl) { // Keep button disabled text separate
         ImGui::SameLine(); ImGui::TextDisabled("(TEST Disabled)");
    }



    ImGui::Separator();
    ImGui::Text("Vertical Tests:");
     if (ImGui::Button("Start Simple Vertical (X)") && enableFlightControl) {
        logToGui("Start Simple Vertical Test button pressed.");
        startProcessing(ProcessingMode::WALL_FOLLOW_FULL_TEST, enableFlightControl);
    }
    if (!enableFlightControl && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Flight control must be enabled during OSDK initialization.");
    }
    if (!enableFlightControl) { // Keep button disabled text separate
         ImGui::SameLine(); ImGui::TextDisabled("(Disabled)");
    }

     if (ImGui::Button("Start Complex Vertical (C)") && enableFlightControl) {
        logToGui("Start Complex Vertical Test button pressed.");
        startProcessing(ProcessingMode::WALL_FOLLOW_COMPLEX_VERTICAL, enableFlightControl);
    }
     if (!enableFlightControl && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Flight control must be enabled during OSDK initialization.");
     }
    if (!enableFlightControl) { // Keep button disabled text separate
         ImGui::SameLine(); ImGui::TextDisabled("(Disabled)");
    }

    ImGui::Separator();
    ImGui::Text("Radar Data Processing (No Control):");
     if (ImGui::Button("Process Full (E)")) {
        logToGui("Process Full Radar button pressed.");
        startProcessing(ProcessingMode::PROCESS_FULL, false);
     }
     if (ImGui::Button("Process Minimal (F)")) {
        logToGui("Process Minimal Radar button pressed.");
        startProcessing(ProcessingMode::PROCESS_MINIMAL, false);
     }

    ImGui::Separator();
    if (processingThread.joinable()) {
        if (ImGui::Button("Stop Current Task")) {
             logToGui("Stop Current Task button pressed.");
             stopProcessing();
        }
    } else {
        ImGui::TextDisabled("No task running.");
    }


    ImGui::Separator();
    if (ImGui::Button("Quit (Q)")) {
        logToGui("Quit button pressed.");
        quitApplication(keepRunning);
    }

    ImGui::End(); // End Controls Panel


    // --- Log Window (Bottom Panel) ---
    ImGui::SetNextWindowPos(ImVec2(main_viewport->WorkPos.x, main_viewport->WorkPos.y + top_panel_height));
    ImGui::SetNextWindowSize(ImVec2(main_viewport->WorkSize.x, bottom_panel_height));
    ImGui::Begin("Log Output Panel", nullptr, window_flags); // Use Begin with flags

    // Update string buffer from stream safely
    {
        std::lock_guard<std::mutex> lock(guiLogMutex);
        guiLogBuffer = guiLogStream.str(); // Copy stream content to string buffer
    }

    // Use c_str() for ImGui
    ImGui::InputTextMultiline("##Log", (char*)guiLogBuffer.c_str(), guiLogBuffer.size() + 1, ImVec2(-FLT_MIN, -FLT_MIN), ImGuiInputTextFlags_ReadOnly);

    // *** CHANGE: Use SetScrollY to force scroll to bottom ***
    ImGui::SetScrollY(ImGui::GetScrollMaxY());


    ImGui::End(); // End Log Output Panel


    // --- Error Message Window (Placeholder - unchanged) ---
    // ...


    // Rendering
    ImGui::Render();
    ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
}

// --- Action Implementations ---

void stopProcessing() {
    if (processingThread.joinable()) {
        logToGui("Signalling processing thread to stop...");
        stopProcessingFlag.store(true);
        try {
             processingThread.join();
             logToGui("Processing thread joined.");
        } catch (const std::system_error& e) {
             logToGui("Error joining processing thread: " + std::string(e.what()));
        }
        stopProcessingFlag.store(false); // Reset flag after join
    } else {
        // Don't log if no thread running, it's normal state
    }
}

void startProcessing(ProcessingMode mode, bool control, const std::string& bridge) {
    // Stop existing thread first
    stopProcessing();

    bool requiresControl = (mode == ProcessingMode::WALL_FOLLOW ||
                            mode == ProcessingMode::WALL_FOLLOW_FULL_TEST ||
                            mode == ProcessingMode::WALL_FOLLOW_COMPLEX_VERTICAL);

    bool useControlForThread = control && requiresControl && enableFlightControl;

     // Specific checks for modes requiring subscribe
    if (useControlForThread && (mode == ProcessingMode::WALL_FOLLOW_FULL_TEST || mode == ProcessingMode::WALL_FOLLOW_COMPLEX_VERTICAL)) {
         if (!vehicle || !vehicle->subscribe) {
              logToGui("Error: Cannot start vertical test with control - OSDK subscribe interface not available.");
              useControlForThread = false; // Cannot proceed with control
              return; // Exit without starting thread
         }
    }

    // Check if control is requested but not possible
    if (control && requiresControl && !useControlForThread) {
         logToGui("Warning: Task requires control, but flight control is disabled or OSDK not ready. Starting without control.");
         if (mode == ProcessingMode::WALL_FOLLOW_FULL_TEST || mode == ProcessingMode::WALL_FOLLOW_COMPLEX_VERTICAL) {
             logToGui("Error: Vertical tests cannot run without flight control. Task aborted.");
             return; // Abort vertical tests if control isn't truly available
         }
    }


    logToGui("Attempting to start processing task (Mode: " + std::to_string(static_cast<int>(mode)) + ", Control: " + (useControlForThread ? "Yes" : "No") + ", Bridge: " + bridge + ")");

    if (useControlForThread) {
         if (!vehicle || !vehicle->control) {
            logToGui("Error: Cannot obtain control authority - OSDK vehicle/control not available.");
            return; // Don't start thread
         }
        logToGui("Attempting to obtain Control Authority...");
        ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
        if (ACK::getError(ctrlAuthAck)) {
            ACK::getErrorCodeMessage(ctrlAuthAck, "startProcessing obtainCtrlAuthority");
            logToGui("Failed to obtain control authority. Cannot start task with control.");
            return; // Don't start the thread
        } else {
            logToGui("Obtained Control Authority.");
        }
    }

    // Start the new thread
    stopProcessingFlag.store(false); // Ensure flag is false before starting
    processingThread = std::thread(processingLoopFunction, bridge, vehicle, useControlForThread, mode);
}

void performTakeoff() {
     if (!enableFlightControl) {
        logToGui("Takeoff failed: Flight control is disabled.");
        return;
     }
     if (!vehicle || !vehicle->control) {
         logToGui("Takeoff failed: OSDK vehicle/control not initialized.");
         return;
     }
     // Create FlightSample if it hasn't been created yet
     if (!flightSample) {
         logToGui("Creating FlightSample instance...");
         // Use C++11 way to create unique_ptr
         flightSample = std::unique_ptr<FlightSample>(new FlightSample(vehicle));
     }

    logToGui("Attempting to obtain Control Authority for Takeoff...");
    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
    if (ACK::getError(ctrlAuthAck)) {
         ACK::getErrorCodeMessage(ctrlAuthAck, "Takeoff obtainCtrlAuthority");
         logToGui("Failed to obtain control authority for takeoff.");
    } else {
         logToGui("Obtained Control Authority for Takeoff. Executing monitored takeoff...");
         // Run takeoff in a separate thread to avoid blocking GUI
         auto takeoffFuture = std::async(std::launch::async, [&]() {
             flightSample->monitoredTakeoff();
             logToGui("Monitored takeoff thread finished.");
             // Optionally release authority here
             // vehicle->control->releaseCtrlAuthority(functionTimeout);
         });
         logToGui("Monitored takeoff command issued (running in background).");
    }
}

void stopMonitoring() {
     if (monitoringThread.joinable()) {
        logToGui("Signalling monitoring thread to stop...");
        stopMonitoringFlag.store(true);
         try {
             monitoringThread.join();
             logToGui("Monitoring thread joined.");
         } catch (const std::system_error& e) {
             logToGui("Error joining monitoring thread: " + std::string(e.what()));
         }
        stopMonitoringFlag.store(false); // Reset flag
        monitoringEnabled = false; // Update status
    }
}


void quitApplication(bool& runningFlag) {
    logToGui("Initiating shutdown sequence...");


    // Stop background threads
    stopProcessing();
    stopMonitoring();

    // OSDK Cleanup
    if (monitoringEnabled && vehicle && vehicle->subscribe) {
        logToGui("Unsubscribing from telemetry...");
        ACK::ErrorCode statusAck = vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
        if(ACK::getError(statusAck)) {
            ACK::getErrorCodeMessage(statusAck, __func__);
            logToGui("Warning: Failed to unsubscribe from telemetry package " + std::to_string(pkgIndex) + ".");
        } else {
            logToGui("Telemetry unsubscribed.");
        }
    }

    if (enableFlightControl && vehicle && vehicle->control) {
        logToGui("Sending Zero Velocity command before final release...");
        uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
        DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
        vehicle->control->flightCtrl(stopData);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        logToGui("Releasing Control Authority on Quit...");
        ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
         if (ACK::getError(releaseAck)) {
             ACK::getErrorCodeMessage(releaseAck, "Quit releaseCtrlAuthority");
             logToGui("Warning: Failed to release control authority on quit.");
         } else {
             logToGui("Control Authority Released on Quit.");
         }
    }

     // Reset smart pointers - handles deletion
     flightSample.reset();
     linuxEnvironment.reset();
     vehicle = nullptr;

     logToGui("OSDK resources released.");

     runningFlag = false; // Signal the main loop to exit AFTER cleanup

}


// --- Main Function ---
int main(int argc, char** argv) {
    loadPreferences(); // Load preferences first

    // --- OSDK Initialization (Attempt) ---
    logToGui("Initializing DJI OSDK...");
    try {
        // Use C++11 way to create unique_ptr
        linuxEnvironment = std::unique_ptr<LinuxSetup>(new LinuxSetup(argc, argv));
        vehicle = linuxEnvironment->getVehicle();

        if (vehicle == nullptr || vehicle->control == nullptr || vehicle->subscribe == nullptr) {
            logToGui("ERROR: Vehicle not initialized or interfaces unavailable. Disabling flight control.");
             linuxEnvironment.reset();
             vehicle = nullptr;
             enableFlightControl = false;
        } else { // OSDK Init seems OK
            logToGui("Attempting initial Control Authority check...");
            ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
            if (ACK::getError(ctrlAuthAck)) {
                 ACK::getErrorCodeMessage(ctrlAuthAck, __func__);
                 logToGui("Failed initial control authority check. Disabling flight control.");
                 linuxEnvironment.reset();
                 vehicle = nullptr;
                 enableFlightControl = false;
            } else { // Authority Obtained
                 logToGui("Initial Control Authority check successful.");
                 // Use C++11 way to create unique_ptr
                 flightSample = std::unique_ptr<FlightSample>(new FlightSample(vehicle));
                 logToGui("FlightSample instance created.");
                 logToGui("OSDK Initialized successfully.");

                 // --- Setup Telemetry Subscription for Monitoring ---
                 logToGui("Setting up Telemetry Subscription for Monitoring...");
                 ACK::ErrorCode subscribeAck = vehicle->subscribe->verify(functionTimeout);
                 if (ACK::getError(subscribeAck)) {
                      ACK::getErrorCodeMessage(subscribeAck, __func__);
                      logToGui("Error verifying subscription package list. Monitoring will be disabled.");
                 } else {
                      Telemetry::TopicName topicList[] = {
                          Telemetry::TOPIC_STATUS_FLIGHT,
                          Telemetry::TOPIC_STATUS_DISPLAYMODE,
                          Telemetry::TOPIC_HEIGHT_FUSION
                      };
                      int numTopic = sizeof(topicList) / sizeof(topicList[0]);
                      bool topicStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList,
                                                                                       false, telemetrySubscriptionFrequency);

                      if (topicStatus) {
                            logToGui("Successfully initialized package " + std::to_string(pkgIndex) + " with Flight Status, Display Mode, and Height topics.");
                            ACK::ErrorCode startAck = vehicle->subscribe->startPackage(pkgIndex, functionTimeout);
                            if (ACK::getError(startAck)) {
                                 ACK::getErrorCodeMessage(startAck, "startPackage");
                                 logToGui("Error starting subscription package " + std::to_string(pkgIndex) + ". Monitoring will be disabled.");
                                 vehicle->subscribe->removePackage(pkgIndex, functionTimeout); // Attempt cleanup
                            } else {
                                 logToGui("Successfully started package " + std::to_string(pkgIndex) + ".");
                                 logToGui("Starting monitoring thread...");
                                 stopMonitoringFlag.store(false);
                                 monitoringThread = std::thread(monitoringLoopFunction, vehicle);
                                 monitoringEnabled = true;
                                 logToGui("[Main] Monitoring thread launched.");
                            }
                      } else {
                           logToGui("Error initializing package " + std::to_string(pkgIndex) + " from topic list. Monitoring will be disabled.");
                      }
                 }
                 // Initial authority obtained, release it now. Tasks will re-obtain if needed.
                 logToGui("Releasing initial control authority (tasks will re-obtain)...");
                 vehicle->control->releaseCtrlAuthority(functionTimeout); // Ignore result for now
            }
        }
    } catch (const std::exception& e) {
         logToGui("EXCEPTION during OSDK Initialization: " + std::string(e.what()));
         linuxEnvironment.reset();
         vehicle = nullptr;
         enableFlightControl = false;
    } catch (...) {
         logToGui("UNKNOWN EXCEPTION during OSDK Initialization.");
         linuxEnvironment.reset();
         vehicle = nullptr;
         enableFlightControl = false;
    }

    logToGui("INFO: Flight control is " + std::string(enableFlightControl ? "ENABLED" : "DISABLED"));

    // --- Initialize GUI ---
    if (!initGui()) {
        logToGui("GUI Initialization Failed. Exiting.");
        bool temp = false;
        quitApplication(temp);
        return 1;
    }


    // --- Main GUI Loop ---
    bool keepRunning = true;
    while (keepRunning) {
        // Poll and handle events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                quitApplication(keepRunning);
            }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window)) {
                quitApplication(keepRunning);
            }
        }

        // If keepRunning was set to false by quitApplication, break the loop
        if (!keepRunning) {
            break;
        }

        // Render the GUI frame
        renderGui(keepRunning);

        // Small sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } // End main loop


    // --- Cleanup ---
    logToGui("Exiting main loop...");
    // Ensure threads stopped (should be handled by quitApplication if loop exited normally)
    stopProcessing();
    stopMonitoring();

    cleanupGui();

    // OSDK resources should have been cleaned by quitApplication. Final check.
     if (linuxEnvironment) {
         logToGui("Warning: OSDK linuxEnvironment still exists at final cleanup. Forcing reset.");
         linuxEnvironment.reset();
         vehicle = nullptr;
         flightSample.reset();
     }

    logToGui("Program terminated.");
    return 0;
}
