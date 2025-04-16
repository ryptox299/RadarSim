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

// Function to load preferences
void loadPreferences() {
    std::cout << "Loading preferences...\n";
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
                std::cerr << "Warning: Skipping invalid line in preferences file: " << line << std::endl;
                continue;
            }

            std::string key = line.substr(0, equalsPos);
            std::string value = line.substr(equalsPos + 1);

            try {
                if (key == "target_beacon_id") {
                    TARGET_BEACON_ID = value;
                    std::cout << "  TARGET_BEACON_ID set to: " << TARGET_BEACON_ID << " (from preferences file)\n";
                } else if (key == "targetdistance") {
                    targetDistance = std::stof(value);
                    std::cout << "  targetDistance set to: " << targetDistance << " meters (from preferences file)\n";
                } else if (key == "target_azimuth") {
                    targetAzimuth = std::stof(value);
                    std::cout << "  targetAzimuth (Initial) set to: " << targetAzimuth << " degrees (from preferences file)\n";
                } else if (key == "kp_forward") {
                    Kp_forward = std::stof(value);
                    std::cout << "  Kp_forward set to: " << Kp_forward << " (from preferences file)\n";
                } else if (key == "max_forward_speed") {
                    max_forward_speed = std::stof(value);
                    std::cout << "  max_forward_speed set to: " << max_forward_speed << " m/s (from preferences file)\n";
                } else if (key == "forward_dead_zone") {
                    forward_dead_zone = std::stof(value);
                    std::cout << "  forward_dead_zone set to: " << forward_dead_zone << " meters (from preferences file)\n";
                } else if (key == "kp_lateral") {
                    Kp_lateral = std::stof(value);
                    std::cout << "  Kp_lateral set to: " << Kp_lateral << " (from preferences file)\n";
                } else if (key == "max_lateral_speed") {
                    max_lateral_speed = std::stof(value);
                    std::cout << "  max_lateral_speed set to: " << max_lateral_speed << " m/s (from preferences file)\n";
                } else if (key == "azimuth_dead_zone") {
                    azimuth_dead_zone = std::stof(value);
                    std::cout << "  azimuth_dead_zone set to: " << azimuth_dead_zone << " degrees (from preferences file)\n";
                } else if (key == "position_distance_tolerance") {
                    position_distance_tolerance = std::stof(value);
                    std::cout << "  position_distance_tolerance set to: " << position_distance_tolerance << " meters (from preferences file)\n";
                } else if (key == "position_azimuth_tolerance") {
                    position_azimuth_tolerance = std::stof(value);
                    std::cout << "  position_azimuth_tolerance set to: " << position_azimuth_tolerance << " degrees (from preferences file)\n";
                } else if (key == "descent_speed") {
                    descent_speed = std::stof(value);
                    std::cout << "  descent_speed set to: " << descent_speed << " m/s (from preferences file)\n";
                } else if (key == "min_floor_altitude") {
                    min_floor_altitude = std::stof(value);
                    std::cout << "  min_floor_altitude set to: " << min_floor_altitude << " meters (from preferences file)\n";
                } else if (key == "target_beacon_range") {
                    target_beacon_range = std::stof(value);
                    std::cout << "  target_beacon_range (Base) set to: " << target_beacon_range << " meters (from preferences file)\n";
                } else if (key == "beacon_range_tolerance") {
                    beacon_range_tolerance = std::stof(value);
                    std::cout << "  beacon_range_tolerance set to: " << beacon_range_tolerance << " meters (from preferences file)\n";
                } else if (key == "ascent_speed") {
                    ascent_speed = std::stof(value);
                    std::cout << "  ascent_speed set to: " << ascent_speed << " m/s (from preferences file)\n";
                } else if (key == "complex_cycles") {
                    complex_cycles = std::stoi(value);
                    std::cout << "  complex_cycles set to: " << complex_cycles << " (from preferences file)\n";
                } else if (key == "lateral_step_meters") { // Renamed from azimuth_step
                    lateral_step_meters = std::stof(value);
                    std::cout << "  lateral_step_meters set to: " << lateral_step_meters << " meters (from preferences file)\n";
                }
            } catch (const std::invalid_argument& ia) {
                std::cerr << "Warning: Invalid number format for key '" << key << "' in preferences file: " << value << std::endl;
            } catch (const std::out_of_range& oor) {
                std::cerr << "Warning: Value out of range for key '" << key << "' in preferences file: " << value << std::endl;
            } catch (...) {
                 std::cerr << "Warning: Unknown error parsing line for key '" << key << "' in preferences file: " << value << std::endl;
            }
        }
        preferencesFile.close();
        std::cout << "Finished loading preferences.\n";
    } else {
        std::cout << "Preferences file ('preferences.txt') not found. Using default values:\n";
        std::cout << "  Default TARGET_BEACON_ID: " << TARGET_BEACON_ID << std::endl;
        std::cout << "  Default targetDistance: " << targetDistance << " meters\n";
        std::cout << "  Default targetAzimuth (Initial): " << targetAzimuth << " degrees\n";
        std::cout << "  Default Kp_forward: " << Kp_forward << std::endl;
        std::cout << "  Default max_forward_speed: " << max_forward_speed << " m/s\n";
        std::cout << "  Default forward_dead_zone: " << forward_dead_zone << " meters\n";
        std::cout << "  Default Kp_lateral: " << Kp_lateral << std::endl;
        std::cout << "  Default max_lateral_speed: " << max_lateral_speed << " m/s\n";
        std::cout << "  Default azimuth_dead_zone: " << azimuth_dead_zone << " degrees\n";
        std::cout << "  Default position_distance_tolerance: " << position_distance_tolerance << " meters\n";
        std::cout << "  Default position_azimuth_tolerance: " << position_azimuth_tolerance << " degrees\n";
        std::cout << "  Default descent_speed: " << descent_speed << " m/s\n";
        std::cout << "  Default min_floor_altitude: " << min_floor_altitude << " meters\n";
        std::cout << "  Default target_beacon_range (Base): " << target_beacon_range << " meters\n"; // Updated desc
        std::cout << "  Default beacon_range_tolerance: " << beacon_range_tolerance << " meters\n";
        std::cout << "  Default ascent_speed: " << ascent_speed << " m/s\n";
        std::cout << "  Default complex_cycles: " << complex_cycles << "\n";
        std::cout << "  Default lateral_step_meters: " << lateral_step_meters << " meters\n"; // Renamed
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
    // ... (remains unchanged) ...
    for (const auto& obj : objects) {
        std::cout << "Radar Object:\n"
                  << "  Timestamp: " << obj.timestamp << "\n"
                  << "  Sensor: " << obj.sensor << "\n"
                  << "  Source: " << obj.src << "\n"
                  << "  ID: " << obj.ID << "\n"
                  << "  X: " << obj.X << " Y: " << obj.Y << " Z: " << obj.Z << "\n"
                  << "  Xdir: " << obj.Xdir << " Ydir: " << obj.Ydir << " Zdir: " << obj.Zdir << "\n"
                  << "  Range: " << obj.Range << " Range Rate: " << obj.RangeRate << "\n"
                  << "  Power: " << obj.Pwr << " Azimuth: " << obj.Az << " Elevation: " << obj.El << "\n"
                  << "  Xsize: " << obj.Xsize << " Ysize: " << obj.Ysize << " Zsize: " << obj.Zsize << "\n"
                  << "  Confidence: " << obj.Conf << "\n"
                  << "----------------------------------------\n";
    }
}

// Display minimal radar object details
void displayRadarObjectsMinimal(const std::vector<RadarObject>& objects) {
    // ... (remains unchanged) ...
    for (const auto& obj : objects) {
        std::cout << "Radar Object (Minimal):\n"
                  << "  Timestamp: " << obj.timestamp << "\n"
                  << "  Sensor: " << obj.sensor << "\n"
                  << "  ID: " << obj.ID << "\n"
                  << "  Range: " << obj.Range << "\n"
                  << "  Azimuth: " << obj.Az << "\n"
                  << "  Elevation: " << obj.El << "\n"
                  << "----------------------------------------\n";
    }
}

// --- ORIGINAL FUNCTION - DO NOT EDIT ---
void extractBeaconAndWallData(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    // ... (remains unchanged) ...
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
                    std::cout << "Control Status: \n"
                              << "TargetWall=" << targetDistance << " | CurrentWall=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                              << "TargetAzimuth=" << targetAzimuth << " | CurrentAzimuth=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "Computed Velocity(X=" << velocity_x << ", Y=" << velocity_y << ")" << std::endl;
                    std::cout << "--------------------------------------\n";
                    vehicle->control->flightCtrl(ctrlData);
                } else if (hasAnonData || foundTargetBeacon) {
                     std::cout << "(Flight Control Disabled or Not Available)" << std::endl;
                     std::cout << "--------------------------------------\n";
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
// --- END ORIGINAL FUNCTION ---


// --- FUNCTION FOR SIMPLE VERTICAL TEST [x] - DO NOT EDIT ---
void extractBeaconAndWallData_FullTest(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) {
    // ... (remains unchanged) ...
    static bool descend_active = false;
    static bool ascent_active = false;
    static std::thread::id current_thread_id;
    if (current_thread_id != std::this_thread::get_id()) {
        descend_active = false;
        ascent_active = false;
        current_thread_id = std::this_thread::get_id();
        std::cout << "[Simple Vert] State reset for new thread run." << std::endl;
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
                        std::cout << "[Simple Vert] Position confirmed (Dist: " << lowestRange << ", Az: " << targetBeaconAzimuth << "). Starting descent." << std::endl;
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
                            std::cout << "[Simple Vert] Minimum altitude reached. Starting ascent phase." << std::endl;
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
                            std::cerr << "[Simple Vert] Warning: Beacon lost during ascent phase. Hovering." << std::endl;
                        }
                    }
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY |
                                          DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY |
                                          DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, velocity_z, 0);
                    std::cout << "[Simple Vert] Control Status: \n"
                              << "TargetWall=" << targetDistance << " | CurrentWall=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                              << "TargetAzimuth=" << targetAzimuth << " | CurrentAzimuth=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                              << "TargetBeaconRange=" << target_beacon_range << " | CurrentBeaconRange=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << "\n"
                              << "Computed Velocity(X=" << velocity_x << ", Y=" << velocity_y << ", Z=" << velocity_z << ")\n"
                              << "Vertical Status: " << vertical_status << " | Current Height: " << (current_height >= 0.0f ? std::to_string(current_height) : "N/A") << std::endl;
                    std::cout << "--------------------------------------\n";
                    vehicle->control->flightCtrl(ctrlData);
                } else if (hasAnonData || foundTargetBeacon) {
                     std::string status = "[Simple Vert] (Flight Control Disabled or Not Available)";
                     if (enableControl && vehicle && !vehicle->subscribe) {
                         status = "[Simple Vert] (Vehicle Subscribe Not Available - Cannot get height)";
                     }
                     std::cout << status << std::endl;
                     std::cout << "TargetWall=" << targetDistance << " | CurrentWall=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                               << "TargetAzimuth=" << targetAzimuth << " | CurrentAzimuth=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A") << "\n"
                               << "TargetBeaconRange=" << target_beacon_range << " | CurrentBeaconRange=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << std::endl;
                     std::cout << "--------------------------------------\n";
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
// Implements multi-cycle descent/ascent with lateral step (meters) and dynamic range target
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
        std::cout << "[Complex Vert] State reset for new thread run." << std::endl;
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

                // --- Movement Logic ---
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
                         std::cerr << "[Complex Vert] Warning: Beacon lost during active phase. Halting lateral motion." << std::endl;
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
                                std::cout << "[Complex Vert] Initial position confirmed (Dist: " << lowestRange << ", Az: " << targetBeaconAzimuth << "). Starting Cycle " << current_cycle + 1 << " Descent." << std::endl;
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
                                    std::cerr << "[Complex Vert] Error: targetDistance is near zero, cannot calculate target azimuth." << std::endl;
                                }
                                runtime_target_azimuth = targetAzimuth + current_target_azimuth_offset; // Update for logging
                                std::cout << "[Complex Vert] Min altitude reached. New Target Lateral Offset: " << current_target_lateral_offset_meters << "m." << std::endl;
                                std::cout << "[Complex Vert] Aligning for ascent (Calculated Target Az Total: " << runtime_target_azimuth << " deg)." << std::endl;
                                current_phase = ComplexPhase::ALIGNING_FOR_ASCENT;
                            }
                            break;

                        case ComplexPhase::ALIGNING_FOR_ASCENT:
                            phase_status_str = "Aligning for Ascent (Cycle " + std::to_string(current_cycle + 1) + ")";
                            velocity_z = 0.0f;
                            current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>(); // Get height for logging
                            if (is_azimuth_aligned) { // Check alignment against calculated runtime target azimuth
                                std::cout << "[Complex Vert] Azimuth aligned (Az: " << targetBeaconAzimuth << "). Starting ascent." << std::endl;
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
                                        std::cout << "[Complex Vert] Target beacon range reached (Range: " << current_beacon_range << ", Target: " << runtime_target_slant_range << "). Cycle " << current_cycle << " complete." << std::endl;
                                        if (current_cycle >= complex_cycles) {
                                            current_phase = ComplexPhase::FINISHED;
                                            std::cout << "[Complex Vert] All cycles finished." << std::endl;
                                        } else {
                                            // Increment desired lateral offset for next descent
                                            current_target_lateral_offset_meters += lateral_step_meters;
                                            // Calculate the required azimuth offset (degrees)
                                            if (std::abs(targetDistance) > 1e-6) {
                                                current_target_azimuth_offset = std::atan(current_target_lateral_offset_meters / targetDistance) * 180.0 / M_PI;
                                            } else {
                                                current_target_azimuth_offset = 0.0;
                                                std::cerr << "[Complex Vert] Error: targetDistance is near zero, cannot calculate target azimuth." << std::endl;
                                            }
                                            runtime_target_azimuth = targetAzimuth + current_target_azimuth_offset; // Update for logging
                                            std::cout << "[Complex Vert] New Target Lateral Offset: " << current_target_lateral_offset_meters << "m." << std::endl;
                                            std::cout << "[Complex Vert] Aligning for next descent (Calculated Target Az Total: " << runtime_target_azimuth << " deg)." << std::endl;
                                            current_phase = ComplexPhase::ALIGNING_FOR_DESCENT;
                                        }
                                    }
                                } else {
                                    // Beacon lost during ascent
                                    velocity_z = 0.0f;
                                    phase_status_str = "Ascending (Beacon Lost)";
                                    std::cerr << "[Complex Vert] Warning: Beacon lost during ascent. Hovering vertically." << std::endl;
                                }
                            } // <<<--- End new scope for case variables
                            break;

                        case ComplexPhase::ALIGNING_FOR_DESCENT:
                             phase_status_str = "Aligning for Descent (Cycle " + std::to_string(current_cycle + 1) + ")";
                             velocity_z = 0.0f;
                             current_height = vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>(); // Get height for logging
                             if (is_azimuth_aligned) { // Check alignment against calculated runtime target azimuth
                                 std::cout << "[Complex Vert] Azimuth aligned (Az: " << targetBeaconAzimuth << "). Starting descent." << std::endl;
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
                    std::cout << "[Complex Vert] Control Status: \n"
                              << "Cycle: " << current_cycle << "/" << complex_cycles << " | Phase: " << phase_status_str << "\n"
                              << "TargetWall=" << targetDistance << " | CurrentWall=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                              << "TargetLateralOffset=" << current_target_lateral_offset_meters << "m | TargetAzTotal(calc)=" << runtime_target_azimuth << "deg | CurrentAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A")
                              << " | AzError=" << (!std::isnan(azimuth_error) ? std::to_string(azimuth_error) : "N/A") << "\n"
                              << "TargetBeaconRange(Dyn)=" << runtime_target_slant_range << " | CurrentBeaconRange=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << "\n"
                              << "Computed Velocity(X=" << velocity_x << ", Y=" << velocity_y << ", Z=" << velocity_z << ")\n"
                              << "Current Height: " << (current_height >= 0.0f ? std::to_string(current_height) : "N/A") << std::endl;
                    std::cout << "--------------------------------------\n";

                    vehicle->control->flightCtrl(ctrlData);

                } else if (hasAnonData || foundTargetBeacon) { // Control disabled or unavailable
                     // ... (logging for disabled control remains the same, maybe update az display) ...
                     std::string status = "[Complex Vert] (Flight Control Disabled or Not Available)";
                     if (enableControl && vehicle && !vehicle->subscribe) {
                         status = "[Complex Vert] (Vehicle Subscribe Not Available - Cannot get height)";
                     }
                     std::cout << status << std::endl;
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
                     std::cout << "Cycle: " << current_cycle << "/" << complex_cycles << " | Phase: " << static_cast<int>(current_phase) << "\n"
                               << "TargetWall=" << targetDistance << " | CurrentWall=" << (hasAnonData ? std::to_string(lowestRange) : "N/A") << "\n"
                               << "TargetLateralOffset=" << current_target_lateral_offset_meters << "m | TargetAzTotal(calc)=" << runtime_target_azimuth << "deg | CurrentAz=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A")
                               << " | AzError=" << (!std::isnan(azimuth_error) ? std::to_string(azimuth_error) : "N/A") << "\n"
                               << "TargetBeaconRange(Dyn)=" << runtime_target_slant_range << " | CurrentBeaconRange=" << (foundTargetBeacon && !std::isnan(current_beacon_range) ? std::to_string(current_beacon_range) : "N/A") << std::endl;
                     std::cout << "--------------------------------------\n";
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
    // ... (remains unchanged) ...
    std::vector<RadarObject> radarObjects;
    if (jsonData.empty() || jsonData == "{}") return radarObjects;

    try {
        auto jsonFrame = json::parse(jsonData);
        if (!jsonFrame.contains("objects") || !jsonFrame["objects"].is_array()) return radarObjects;

        for (const auto& obj : jsonFrame["objects"]) {
            if (!obj.is_object()) {
                std::cerr << "Skipping non-object item in 'objects' array." << std::endl;
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
        std::cerr << "JSON Parsing Error: " << e.what() << " at offset " << e.byte << ". Data: [" << jsonData.substr(0, 200) << "...]" << std::endl;
        return {};
    } catch (const json::type_error& e) {
        std::cerr << "JSON Type Error: " << e.what() << ". Data: [" << jsonData.substr(0, 200) << "...]" << std::endl;
        return {};
    }
    return radarObjects;
}

// Runs the python bridge script
void runPythonBridge(const std::string& scriptName) {
    // ... (remains unchanged) ...
    std::cout << "Starting Python bridge (" << scriptName << ")...\n";
    if (std::system(("python3 " + scriptName + " &").c_str()) != 0) {
        std::cerr << "Failed to start Python bridge script '" << scriptName << "'.\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Python bridge potentially started.\n";
}

// Stops the python bridge script
void stopPythonBridge(const std::string& scriptName) {
    // ... (remains unchanged) ...
    std::cout << "Stopping Python bridge (" << scriptName << ")...\n";
    std::system(("pkill -f " + scriptName).c_str()); // Ignore result
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Sent SIGTERM to " << scriptName << ".\n";
}

// Connects to the python bridge via TCP
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket) {
    // ... (remains unchanged) ...
    tcp::resolver resolver(io_context);
    int retries = 5;
    while (retries-- > 0) {
        try {
            if(socket.is_open()) { socket.close(); } // Simpler close
            auto endpoints = resolver.resolve("127.0.0.1", "5000");
            boost::system::error_code ec;
            boost::asio::connect(socket, endpoints, ec);
            if (!ec) { std::cout << "Connected to Python bridge.\n"; return true; }
            else { std::cerr << "Connection attempt failed: " << ec.message() << std::endl; }
        } catch (const std::exception& e) {
            std::cerr << "Exception during connection attempt: " << e.what() << std::endl;
        }
        if(retries > 0 && !stopProcessingFlag.load()) {
             std::cout << "Retrying connection in 2 seconds... (" << retries << " attempts left)" << std::endl;
             for (int i = 0; i < 2 && !stopProcessingFlag.load(); ++i) std::this_thread::sleep_for(std::chrono::seconds(1));
             if (stopProcessingFlag.load()) { std::cout << "Stop requested during connection retry." << std::endl; break; }
        } else if (stopProcessingFlag.load()) { std::cout << "Stop requested during connection retry." << std::endl; break; }
    }
     std::cerr << "Failed to connect to Python bridge after multiple attempts." << std::endl;
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
    WALL_FOLLOW_FULL_TEST,        // Mode for Simple Vertical Test [x]
    WALL_FOLLOW_COMPLEX_VERTICAL // Mode for Complex Vertical Test [c]
};

// Processing loop function - No reconnection, releases authority on exit
void processingLoopFunction(const std::string bridgeScriptName, Vehicle* vehicle, bool enableControlCmd, ProcessingMode mode) {
    // ... (remains unchanged) ...
    std::cout << "Processing thread started. Bridge: " << bridgeScriptName << ", Control Enabled: " << std::boolalpha << enableControlCmd << ", Mode: " << static_cast<int>(mode) << std::endl;
    int functionTimeout = 1;

    runPythonBridge(bridgeScriptName);
    boost::asio::io_context io_context;
    tcp::socket socket(io_context);

    if (!connectToPythonBridge(io_context, socket)) {
        std::cerr << "Processing thread: Initial connection failed. Exiting thread." << std::endl;
        stopPythonBridge(bridgeScriptName);
        if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
            std::cout << "[Processing Thread] Releasing control authority (initial connection fail)..." << std::endl;
            ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
            if (ACK::getError(releaseAck)) ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority");
            else std::cout << "[Processing Thread] Control authority released." << std::endl;
        }
        return; // Exit thread
    }

    std::cout << "Processing thread: Connection successful. Reading data stream..." << std::endl;
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
            if (error == boost::asio::error::eof) std::cerr << "Processing thread: Connection closed by Python bridge (EOF)." << std::endl;
            else std::cerr << "Processing thread: Error reading from socket: " << error.message() << std::endl;
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
                            // --- Updated Switch Case ---
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
                                case ProcessingMode::WALL_FOLLOW_FULL_TEST: // Simple Vertical
                                    extractBeaconAndWallData_FullTest(radarObjects, vehicle, enableControlCmd);
                                    break;
                                case ProcessingMode::WALL_FOLLOW_COMPLEX_VERTICAL: // Complex Vertical
                                    extractBeaconAndWallData_ComplexVertical(radarObjects, vehicle, enableControlCmd);
                                    break;
                            }
                        }
                    } catch (const std::exception& e) {
                         std::cerr << "Error processing data: " << e.what() << "\nSnippet: [" << jsonData.substr(0, 100) << "...]" << std::endl;
                    }
                }
            }
            if (stopProcessingFlag.load()) break;
        } else {
             std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Brief pause if no data
        }
    }

    // --- Cleanup ---
    if (stopProcessingFlag.load()) std::cout << "[Processing Thread] Stop requested manually." << std::endl;
    else if (connection_error_occurred) std::cout << "[Processing Thread] Exiting due to connection error." << std::endl;
    else std::cout << "[Processing Thread] Data stream ended." << std::endl;

    if (socket.is_open()) { socket.close(); }
    stopPythonBridge(bridgeScriptName);

    // Release Control Authority if enabled
    if (enableControlCmd && vehicle != nullptr && vehicle->control != nullptr) {
        std::cout << "[Processing Thread] Sending final zero velocity command..." << std::endl;
        uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
        DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
        vehicle->control->flightCtrl(stopData);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "[Processing Thread] Releasing control authority..." << std::endl;
        ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
        if (ACK::getError(releaseAck)) {
             ACK::getErrorCodeMessage(releaseAck, "[Processing Thread] releaseCtrlAuthority (on exit)");
             std::cerr << "[Processing Thread] Warning: Failed to release control authority on exit." << std::endl;
        } else {
             std::cout << "[Processing Thread] Control authority released." << std::endl;
        }
    }
    std::cout << "Processing thread finished." << std::endl;
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
void monitoringLoopFunction(Vehicle* vehicle) {
    // ... (remains unchanged) ...
    std::cout << "[Monitoring] Thread started." << std::endl;
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
                 std::cerr << "\n**** MONITORING ERROR: Vehicle/subscribe object null. Stopping. ****\n" << std::endl;
                 telemetry_timed_out = true;
             }
             break;
        }

        uint8_t current_flight_status = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
        uint8_t current_display_mode = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
        bool valid_poll = (current_flight_status <= DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR);

        if (valid_poll) {
            time_t current_time = std::time(nullptr);
            last_valid_poll_time = current_time;
            if (telemetry_timed_out) {
                 std::cout << "[Monitoring] Telemetry poll recovered." << std::endl;
                 telemetry_timed_out = false;
            }

            // Check Flight Status Change
            if (current_flight_status != DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) {
                if ( (previous_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
                     (current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND || current_flight_status == DJI::OSDK::VehicleStatus::FlightStatus::STOPED) &&
                     !warned_unexpected_status) {
                     std::cerr << "\n**** MONITORING WARNING: Flight status changed unexpectedly from IN_AIR to " << (int)current_flight_status << ". ****\n" << std::endl;
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
                    std::cout << "\n**** MONITORING INFO: Entered SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << " / " << (int)EXPECTED_SDK_MODE << ") ****\n" << std::endl;
                    in_sdk_control_mode = true;
                    warned_not_in_sdk_mode = false;
                }
            } else { // Not in expected mode
                 if (in_sdk_control_mode || !warned_not_in_sdk_mode) {
                      // Check if processing thread is stopping/stopped
                      if (!stopProcessingFlag.load() && processingThread.joinable()) {
                          std::string current_mode_name = getModeName(current_display_mode);
                          std::cerr << "\n**** MONITORING WARNING: NOT in expected SDK Control Mode (" << getModeName(EXPECTED_SDK_MODE) << "). Current: " << current_mode_name << " (" << (int)current_display_mode << "). RC may have control. ****\n" << std::endl;
                          warned_not_in_sdk_mode = true;
                      }
                      in_sdk_control_mode = false;
                 }
            }
        } else { // Invalid poll
            if (!telemetry_timed_out) {
                 std::cerr << "\n**** MONITORING WARNING: Polling telemetry returned potentially invalid data (FlightStatus=" << (int)current_flight_status << "). ****\n" << std::endl;
                 telemetry_timed_out = true;
            }
        }

        // Check Telemetry Timeout
        time_t current_time_for_timeout_check = std::time(nullptr);
        if (last_valid_poll_time > 0 && (current_time_for_timeout_check - last_valid_poll_time > TELEMETRY_TIMEOUT_SECONDS)) {
            if (!telemetry_timed_out) {
                std::cerr << "\n**** MONITORING TIMEOUT: No valid telemetry for over " << TELEMETRY_TIMEOUT_SECONDS << " seconds. ****\n" << std::endl;
                telemetry_timed_out = true;
            }
        } else if (last_valid_poll_time == 0) { // Check if never received first poll
             static time_t start_time = 0; if (start_time == 0) start_time = current_time_for_timeout_check;
             if (current_time_for_timeout_check - start_time > TELEMETRY_TIMEOUT_SECONDS * 2) {
                  if (!telemetry_timed_out) {
                       std::cerr << "\n**** MONITORING TIMEOUT: Never received valid telemetry poll after " << TELEMETRY_TIMEOUT_SECONDS * 2 << " seconds. ****\n" << std::endl;
                       telemetry_timed_out = true;
                  }
             }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "[Monitoring] Thread finished." << std::endl;
}


int main(int argc, char** argv) {
    // ... (remains unchanged until Process Command section) ...
    loadPreferences(); // Load preferences first

    // --- Mode Selection Removed - Hardcoded to Live Mode ---
    bool enableFlightControl = true;
    // defaultPythonBridgeScript is already set globally

    std::cout << "Starting in Live Mode. Flight control enabled. Default bridge: " << defaultPythonBridgeScript << "\n";


    // --- OSDK Initialization ---
    int functionTimeout = 1;
    Vehicle* vehicle = nullptr;
    FlightSample* flightSample = nullptr;
    LinuxSetup* linuxEnvironment = nullptr;
    int telemetrySubscriptionFrequency = 1;
    int pkgIndex = 0;
    bool monitoringEnabled = false;

    if (enableFlightControl) { // This will always be true now unless OSDK init fails
        std::cout << "Initializing DJI OSDK...\n";
        linuxEnvironment = new LinuxSetup(argc, argv);
        vehicle = linuxEnvironment->getVehicle();
        if (vehicle == nullptr || vehicle->control == nullptr || vehicle->subscribe == nullptr) {
            std::cerr << "ERROR: Vehicle not initialized or interfaces unavailable. Disabling flight control." << std::endl;
             if (linuxEnvironment) { delete linuxEnvironment; linuxEnvironment = nullptr; }
             vehicle = nullptr;
             enableFlightControl = false; // Fallback in case of init error
             std::cout << "OSDK Initialization Failed. Flight control disabled." << std::endl;
        } else { // OSDK Init seems OK
            std::cout << "Attempting to obtain Control Authority...\n";
            ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
            if (ACK::getError(ctrlAuthAck)) {
                 ACK::getErrorCodeMessage(ctrlAuthAck, __func__);
                 std::cerr << "Failed to obtain control authority. Disabling flight control." << std::endl;
                 if (linuxEnvironment) { delete linuxEnvironment; linuxEnvironment = nullptr; }
                 vehicle = nullptr;
                 enableFlightControl = false; // Fallback
                 std::cout << "Obtaining Control Authority Failed. Flight control disabled." << std::endl;
            } else { // Authority Obtained
                 std::cout << "Obtained Control Authority." << std::endl;
                 flightSample = new FlightSample(vehicle);
                 std::cout << "OSDK Initialized and Flight Sample created." << std::endl;

                 // --- Setup Telemetry Subscription for Monitoring (ADDED HEIGHT) ---
                 std::cout << "Setting up Telemetry Subscription for Monitoring..." << std::endl;
                 ACK::ErrorCode subscribeAck = vehicle->subscribe->verify(functionTimeout);
                 if (ACK::getError(subscribeAck)) {
                      ACK::getErrorCodeMessage(subscribeAck, __func__);
                      std::cerr << "Error verifying subscription package list. Monitoring will be disabled." << std::endl;
                 } else {
                      // --- ADDED Telemetry::TOPIC_HEIGHT_FUSION ---
                      Telemetry::TopicName topicList[] = {
                          Telemetry::TOPIC_STATUS_FLIGHT,
                          Telemetry::TOPIC_STATUS_DISPLAYMODE,
                          Telemetry::TOPIC_HEIGHT_FUSION
                      };
                      int numTopic = sizeof(topicList) / sizeof(topicList[0]);
                      bool topicStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList,
                                                                                       false, telemetrySubscriptionFrequency);

                      if (topicStatus) {
                            std::cout << "Successfully initialized package " << pkgIndex << " with Flight Status, Display Mode, and Height topics." << std::endl; // Updated msg
                            ACK::ErrorCode startAck = vehicle->subscribe->startPackage(pkgIndex, functionTimeout);
                            if (ACK::getError(startAck)) {
                                 ACK::getErrorCodeMessage(startAck, "startPackage");
                                 std::cerr << "Error starting subscription package " << pkgIndex << ". Monitoring will be disabled." << std::endl;
                                 vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
                            } else {
                                 std::cout << "Successfully started package " << pkgIndex << "." << std::endl;
                                 std::cout << "Starting monitoring thread..." << std::endl;
                                 stopMonitoringFlag.store(false);
                                 monitoringThread = std::thread(monitoringLoopFunction, vehicle);
                                 monitoringEnabled = true;
                                 std::cout << "[Main] Monitoring thread launched." << std::endl;
                            }
                      } else {
                           std::cerr << "Error initializing package " << pkgIndex << " from topic list. Monitoring will be disabled." << std::endl;
                      }
                 }
            }
        }
    }

    std::cout << "INFO: Flight control is " << (enableFlightControl ? "ENABLED" : "DISABLED") << ". Proceeding to main menu.\n"; // Keep this info line

    // --- Main Command Loop ---
    bool keepRunning = true;
    while (keepRunning) {
        // --- Updated Menu Display ---
        std::cout << "\n--- Main Menu ---\n"
                  << (enableFlightControl ? "| [t] Monitored Takeoff                     |\n| [w] Start Wall+Beacon Following (Default) |\n| [h] Start Wall+Beacon Following (TEST)    |\n| [x] simple vertical test                  |\n| [c] complex vertical test                 |\n"
                                          : "| [t] Takeoff (DISABLED)                    |\n| [w] Wall/Beacon Following (No Control)    |\n| [h] Wall/Beacon Following (No Control)    |\n| [x] simple vertical test (No Control)     |\n| [c] complex vertical test (No Control)    |\n")
                  << "| [e] Process Full Radar (No Control)       |\n"
                  << "| [f] Process Minimal Radar (No Control)    |\n"
                  << "| [q] Quit                                  |\n"
                  << "---------------------------------------------\n"
                  << "Enter command: ";

        std::string lineInput; char inputChar = 0;
        if (std::getline(std::cin, lineInput)) { if (!lineInput.empty()) inputChar = lineInput[0]; }
        else { inputChar = 'q'; if (std::cin.eof()) std::cout << "\nEOF detected. "; std::cout << "Exiting." << std::endl; }

        // Stop processing thread if starting new task ('w','e','h','f','x','c') or quitting ('q')
         if (strchr("wehfx", inputChar) != nullptr || inputChar == 'c' || inputChar == 'q') { // Added 'c'
             if (processingThread.joinable()) {
                 std::cout << "Signalling processing thread to stop..." << std::endl;
                 stopProcessingFlag.store(true); processingThread.join();
                 std::cout << "Processing thread finished." << std::endl;
                 stopProcessingFlag.store(false);
             }
         }
         // Stop monitoring thread on quit
         if (inputChar == 'q' && monitoringThread.joinable()) {
             std::cout << "Signalling monitoring thread to stop..." << std::endl;
             stopMonitoringFlag.store(true); monitoringThread.join();
             std::cout << "Monitoring thread finished." << std::endl;
             stopMonitoringFlag.store(false); monitoringEnabled = false;
         }

        // --- Process Command ---
        switch (inputChar) {
            case 't': // Takeoff
                // ... (remains unchanged) ...
                if (enableFlightControl && flightSample && vehicle && vehicle->control) {
                    std::cout << "Attempting to obtain Control Authority for Takeoff...\n";
                    ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                    if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Takeoff obtainCtrlAuthority");
                         std::cerr << "Failed to obtain control authority for takeoff." << std::endl;
                         break;
                    } else {
                         std::cout << "Obtained Control Authority for Takeoff." << std::endl;
                         flightSample->monitoredTakeoff();
                    }
                } else {
                     std::cout << "Flight control not enabled or available." << std::endl;
                }
                break;

             case 'w': // Wall Following (Default)
             case 'h': { // Wall Following (TEST)
                // ... (remains unchanged) ...
                 std::string bridge = (inputChar == 'w') ? defaultPythonBridgeScript : "python_bridge_LOCAL.py";
                 bool useControl = (inputChar == 'w') ? enableFlightControl : true;
                 if (inputChar == 'h' && !enableFlightControl) { std::cout << "Error: Flight control must be enabled for TEST control.\n"; break; }

                 std::cout << "Starting Wall+Beacon Following using bridge: " << bridge << "\n";
                 if (useControl && vehicle && vehicle->control) {
                     std::cout << "Attempting to obtain Control Authority for Wall Following...\n";
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Wall Following obtainCtrlAuthority");
                         std::cerr << "Failed to obtain control authority. Cannot start with control.\n";
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Wall Following." << std::endl;
                     }
                 } else if (useControl) {
                      std::cerr << "Error: Cannot start with control as OSDK is not properly initialized.\n";
                      break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, bridge, vehicle, useControl, ProcessingMode::WALL_FOLLOW);
                 break;
             }
             case 'x': { // Simple Vertical Test
                // ... (remains unchanged) ...
                 std::cout << "Starting Simple Vertical Test using default bridge: " << defaultPythonBridgeScript << "\n";
                 if (enableFlightControl && vehicle && vehicle->control && vehicle->subscribe) {
                     std::cout << "Attempting to obtain Control Authority for Simple Vertical Test...\n";
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Simple Vertical Test obtainCtrlAuthority");
                         std::cerr << "Failed to obtain control authority. Cannot start Simple Vertical Test with control.\n";
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Simple Vertical Test." << std::endl;
                     }
                 } else if (enableFlightControl) {
                     std::string reason = (!vehicle || !vehicle->control) ? "OSDK control" : "OSDK subscribe";
                     std::cerr << "Error: Cannot start Simple Vertical Test with control as " << reason << " is not properly initialized.\n";
                     break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW_FULL_TEST);
                 break;
            }
             case 'c': { // Complex Vertical Test
                // ... (remains unchanged) ...
                 std::cout << "Starting Complex Vertical Test using default bridge: " << defaultPythonBridgeScript << "\n";
                 if (enableFlightControl && vehicle && vehicle->control && vehicle->subscribe) {
                     std::cout << "Attempting to obtain Control Authority for Complex Vertical Test...\n";
                     ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
                     if (ACK::getError(ctrlAuthAck)) {
                         ACK::getErrorCodeMessage(ctrlAuthAck, "Complex Vertical Test obtainCtrlAuthority");
                         std::cerr << "Failed to obtain control authority. Cannot start Complex Vertical Test with control.\n";
                         break;
                     } else {
                         std::cout << "Obtained Control Authority for Complex Vertical Test." << std::endl;
                     }
                 } else if (enableFlightControl) {
                     std::string reason = (!vehicle || !vehicle->control) ? "OSDK control" : "OSDK subscribe";
                     std::cerr << "Error: Cannot start Complex Vertical Test with control as " << reason << " is not properly initialized.\n";
                     break;
                 }
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW_COMPLEX_VERTICAL);
                 break;
            }
             case 'e': // Process Full
             case 'f': { // Process Minimal
                // ... (remains unchanged) ...
                 ProcessingMode procMode = (inputChar == 'e') ? ProcessingMode::PROCESS_FULL : ProcessingMode::PROCESS_MINIMAL;
                 std::cout << "Starting Radar Data processing (No Control) using bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, procMode);
                 break;
             }

            case 'q': { // Quit
                // ... (remains unchanged) ...
                std::cout << "Exiting..." << std::endl;
                // Stop threads handled above

                if (monitoringEnabled && vehicle && vehicle->subscribe) {
                    std::cout << "Unsubscribing from telemetry..." << std::endl;
                    ACK::ErrorCode statusAck = vehicle->subscribe->removePackage(pkgIndex, functionTimeout);
                    if(ACK::getError(statusAck)) {
                        ACK::getErrorCodeMessage(statusAck, __func__);
                        std::cerr << "Warning: Failed to unsubscribe from telemetry package " << pkgIndex << "." << std::endl;
                    } else {
                        std::cout << "Telemetry unsubscribed." << std::endl;
                    }
                }

                if (enableFlightControl && vehicle && vehicle->control) {
                    std::cout << "Sending Zero Velocity command before final release..." << std::endl;
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
                    vehicle->control->flightCtrl(stopData);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    std::cout << "Releasing Control Authority on Quit...\n";
                    ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
                     if (ACK::getError(releaseAck)) {
                         ACK::getErrorCodeMessage(releaseAck, "Quit releaseCtrlAuthority");
                         std::cerr << "Warning: Failed to release control authority on quit." << std::endl;
                     } else {
                         std::cout << "Control Authority Released on Quit." << std::endl;
                     }
                }
                 if (linuxEnvironment) {
                    delete linuxEnvironment; linuxEnvironment = nullptr;
                 }
                 if (flightSample) flightSample = nullptr;
                 vehicle = nullptr;

                keepRunning = false;
                break;
            }
            case 0: break; // Enter key pressed
            default: std::cout << "Invalid input." << std::endl; break;
        }
    } // End main loop

    // Final check to join threads if loop exited unexpectedly
    // ... (remains unchanged) ...
     if (processingThread.joinable()) {
         std::cerr << "Warning: Main loop exited unexpectedly, ensuring processing thread is stopped." << std::endl;
         stopProcessingFlag.store(true);
         try { processingThread.join(); } catch (const std::system_error& e) { std::cerr << "Error joining processing thread: " << e.what() << std::endl; }
     }
     if (monitoringThread.joinable()) {
         std::cerr << "Warning: Main loop exited unexpectedly, ensuring monitoring thread is stopped." << std::endl;
         stopMonitoringFlag.store(true);
          try { monitoringThread.join(); } catch (const std::system_error& e) { std::cerr << "Error joining monitoring thread: " << e.what() << std::endl; }
     }

      if (linuxEnvironment) {
         delete linuxEnvironment;
         linuxEnvironment = nullptr;
         vehicle = nullptr;
         flightSample = nullptr;
      }

    std::cout << "Program terminated." << std::endl;
    return 0;
}
