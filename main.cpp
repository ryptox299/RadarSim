#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstdlib>  // For std::system
#include <json.hpp> // Include the JSON library
#include <boost/asio.hpp>    // Include Boost ASIO for TCP communication
#include "flight_control_sample.hpp" // Still needed for monitoredTakeoff/Landing
#include "flight_sample.hpp"         // Still needed for FlightSample class definition
#include "dji_linux_helpers.hpp"
#include <limits> // For numeric limits
#include <fstream> // For file reading
#include <cmath> // For std::abs, std::isnan
#include <atomic> // For thread-safe stop flag
#include <cstring> // For strchr
#include <stdexcept> // For standard exceptions
#include <array> // For std::array used in read_some buffer
// Include the headers that define Control flags, CtrlData, FlightController, and Vehicle
#include "dji_control.hpp"           // Defines Control class, CtrlData, enums
#include "dji_flight_controller.hpp" // Defines FlightController (though not used for this specific command)
#include "dji_vehicle.hpp"           // Defines Vehicle class which contains Control*

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry; // If using Telemetry data elsewhere
using json = nlohmann::json;
using boost::asio::ip::tcp;

// ***** ADDED ***** Target Beacon ID Constant
const std::string TARGET_BEACON_ID = "BEACON-TX-ID:00005555";

// Persistent variables for tracking the current second and wall/beacon candidate data
// ***** These are reset on disconnect in processingLoopFunction *****
std::string currentSecond = "";  // Tracks the current second being processed
float lowestRange = std::numeric_limits<float>::max();  // Tracks the lowest range for the current second (potential wall)
bool hasAnonData = false;  // Tracks whether any "anon" ID data exists for this second (wall candidate)
float targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN(); // Tracks the azimuth of the target beacon for the current second
bool foundTargetBeacon = false; // Tracks if the target beacon was found this second
// ********************************************************************

// Global variable for default Python bridge script (set by initial mode selection)
std::string defaultPythonBridgeScript = "python_bridge.py";

// Global variable for target distance (forward/backward from wall)
float targetDistance = 8.0f;

// Flag to control the processing loop in a separate thread
std::atomic<bool> stopProcessingFlag(false);
std::thread processingThread; // Thread object for radar processing

// ***** ADDED ***** Lateral Control Parameters (NEEDS TUNING)
float Kp_lateral = 0.02; // Proportional gain for lateral control based on azimuth (Azimuth is in degrees, output is m/s)
float max_lateral_speed = 0.5; // Maximum lateral speed in m/s
float azimuth_dead_zone = 1.5; // Azimuth tolerance in degrees (don't correct if beacon is within +/- this angle)


// Function to load preferences (e.g., target distance) from a file
void loadPreferences() {
    std::ifstream preferencesFile("preferences.txt");
    if (preferencesFile.is_open()) {
        std::string line;
        while (std::getline(preferencesFile, line)) {
            if (line.find("targetdistance=") == 0) {
                try {
                    targetDistance = std::stof(line.substr(line.find("=") + 1));
                    std::cout << "Target distance set to: " << targetDistance << " meters (from preferences file)\n";
                } catch (const std::invalid_argument& ia) {
                     std::cerr << "Invalid target distance format in preferences file: " << line << std::endl;
                } catch (const std::out_of_range& oor) {
                     std::cerr << "Target distance out of range in preferences file: " << line << std::endl;
                }
            }
            // ***** ADDED ***** Load lateral control parameters from file (optional)
            else if (line.find("kp_lateral=") == 0) {
                 try { Kp_lateral = std::stof(line.substr(line.find("=") + 1)); std::cout << "Kp_lateral set to: " << Kp_lateral << " (from preferences file)\n"; } catch (...) { std::cerr << "Invalid Kp_lateral format.\n"; }
            } else if (line.find("max_lateral_speed=") == 0) {
                 try { max_lateral_speed = std::stof(line.substr(line.find("=") + 1)); std::cout << "max_lateral_speed set to: " << max_lateral_speed << " (from preferences file)\n"; } catch (...) { std::cerr << "Invalid max_lateral_speed format.\n"; }
            } else if (line.find("azimuth_dead_zone=") == 0) {
                 try { azimuth_dead_zone = std::stof(line.substr(line.find("=") + 1)); std::cout << "azimuth_dead_zone set to: " << azimuth_dead_zone << " (from preferences file)\n"; } catch (...) { std::cerr << "Invalid azimuth_dead_zone format.\n"; }
            }
        }
        preferencesFile.close();
    } else {
        std::cout << "Preferences file not found. Using default values.\n";
        std::cout << "Default target distance: " << targetDistance << " meters.\n";
        // ***** ADDED ***** Log default lateral params
        std::cout << "Default Kp_lateral: " << Kp_lateral << std::endl;
        std::cout << "Default max_lateral_speed: " << max_lateral_speed << std::endl;
        std::cout << "Default azimuth_dead_zone: " << azimuth_dead_zone << std::endl;
    }
}

// REMOVED CtrlData constructor definition - it's already in libdjiosdk-core.a

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

// displayRadarObjects and displayRadarObjectsMinimal remain unchanged
void displayRadarObjects(const std::vector<RadarObject>& objects) {
    for (const auto& obj : objects) {
        std::cout << "Radar Object:\n"
                  << "  Timestamp: " << obj.timestamp << "\n"
                  << "  Sensor: " << obj.sensor << "\n"
                  << "  Source: " << obj.src << "\n"
                  << "  ID: " << obj.ID << "\n"
                  << "  X: " << obj.X << " "
                  << "  Y: " << obj.Y << " "
                  << "  Z: " << obj.Z << "\n"
                  << "  Xdir: " << obj.Xdir << " "
                  << "  Ydir: " << obj.Ydir << " "
                  << "  Zdir: " << obj.Zdir << "\n"
                  << "  Range: " << obj.Range << "\n"
                  << "  Range Rate: " << obj.RangeRate << "\n"
                  << "  Power: " << obj.Pwr << "\n"
                  << "  Azimuth: " << obj.Az << "\n"
                  << "  Elevation: " << obj.El << "\n"
                  << "  Xsize: " << obj.Xsize << " "
                  << "  Ysize: " << obj.Ysize << " "
                  << "  Zsize: " << obj.Zsize << "\n"
                  << "  Confidence: " << obj.Conf << "\n"
                  << "----------------------------------------\n";
    }
}

void displayRadarObjectsMinimal(const std::vector<RadarObject>& objects) {
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

// Modified function accepts Vehicle pointer (can be nullptr)
// Now also includes logic to stop based on stopProcessingFlag
// Velocity commands are now relative to the drone's BODY FRAME using control->flightCtrl
// Includes lateral control based on target beacon azimuth
void extractBeaconAndWallData(const std::vector<RadarObject>& objects, Vehicle* vehicle, bool enableControl) { // Parameter is Vehicle*

    for (const auto& obj : objects) {
         if (stopProcessingFlag.load()) return; // Check flag before processing each object

        // Extract the second from the timestamp string (handle potential quotes from .dump())
        std::string ts_cleaned = obj.timestamp;
        if (!ts_cleaned.empty() && ts_cleaned.front() == '"') {
             ts_cleaned.erase(0, 1); // Remove leading quote
        }
         if (!ts_cleaned.empty() && ts_cleaned.back() == '"') {
             ts_cleaned.pop_back(); // Remove trailing quote
        }

        std::string objSecond;
        size_t dotPos = ts_cleaned.find('.');
        if (dotPos != std::string::npos) {
             objSecond = ts_cleaned.substr(0, dotPos);
        } else {
             objSecond = ts_cleaned; // Handle cases without milliseconds or if it wasn't a number originally
        }


        // Simple string comparison might fail if timestamps aren't perfectly ordered.
        // Consider parsing to a comparable time format if strict ordering is needed.
        // For now, assuming roughly sequential seconds.
        // Ignore data from old seconds (basic check)
        // ***** NOTE: This check uses the GLOBAL currentSecond which is reset on disconnect *****
        if (!currentSecond.empty() && objSecond < currentSecond) {
            // std::cout << "Skipping old data: " << obj.timestamp << std::endl; // Debugging
            continue;
        }

        // If this is a new second - Process data from the *previous* second
        // ***** NOTE: This check uses the GLOBAL currentSecond which is reset on disconnect *****
        if (objSecond != currentSecond) {
            // Output the result for the previous second, if any, and command drone movement
            // ***** NOTE: This check uses the GLOBAL state vars which are reset on disconnect *****
            if (!currentSecond.empty() && (hasAnonData || foundTargetBeacon)) { // Process if we have wall OR beacon data

                if (hasAnonData) {
                    std::cout << currentSecond << ": Likely WALL candidate distance: " << lowestRange << " meters. ";
                } else {
                    std::cout << currentSecond << ": No WALL candidate found. ";
                }
                if (foundTargetBeacon) {
                    std::cout << "Target BEACON Azimuth: " << targetBeaconAzimuth << " deg.\n";
                } else {
                    std::cout << "Target BEACON not found.\n";
                }


                // --- Movement Logic ---
                // Use the passed-in vehicle pointer and its 'control' member
                if (enableControl && vehicle != nullptr && vehicle->control != nullptr) { // Check enableControl, vehicle AND control pointers

                    // -- Calculate Forward/Backward Velocity (X) based on Wall Distance --
                    float velocity_x = 0.0f; // Default to no forward/backward movement
                    if (hasAnonData) { // Only adjust X if wall is detected
                        float difference = lowestRange - targetDistance; // Error = current - target
                        float Kp = 0.5; // Proportional gain (NEEDS TUNING)
                        float max_speed = 0.8; // Maximum speed in m/s (NEEDS TUNING)
                        float dead_zone = 0.2; // Distance tolerance in meters (NEEDS TUNING)

                        if (std::abs(difference) > dead_zone) { // Only move if outside the dead zone
                            velocity_x = Kp * difference;
                            // Clamp velocity to max_speed
                            velocity_x = std::max(-max_speed, std::min(max_speed, velocity_x));
                        }
                        // If within dead zone, velocity_x remains 0
                    }

                    // -- Calculate Lateral Velocity (Y) based on Beacon Azimuth --
                    float velocity_y = 0.0f; // Default to no lateral movement
                    if (foundTargetBeacon && !std::isnan(targetBeaconAzimuth)) { // Check if beacon was found and Azimuth is valid
                        if (std::abs(targetBeaconAzimuth) > azimuth_dead_zone) {
                            // Azimuth is likely in degrees, Kp_lateral should scale it appropriately to m/s
                            // Positive Azimuth means beacon is to the right -> move right (positive Y velocity)
                            // Negative Azimuth means beacon is to the left -> move left (negative Y velocity)
                            velocity_y = Kp_lateral * targetBeaconAzimuth;
                            // Clamp lateral velocity
                            velocity_y = std::max(-max_lateral_speed, std::min(max_lateral_speed, velocity_y));
                        }
                        // If within dead zone, velocity_y remains 0
                    }

                    // Define the control flag for body frame velocity control
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | // Control horizontal velocity (X and Y)
                                          DJI::OSDK::Control::VERTICAL_VELOCITY   | // Control vertical velocity (set to 0)
                                          DJI::OSDK::Control::YAW_RATE            | // Control yaw rate (set to 0)
                                          DJI::OSDK::Control::HORIZONTAL_BODY     | // Use Body Frame for horizontal
                                          DJI::OSDK::Control::STABLE_ENABLE;        // Use stable mode (brake when input is 0)


                    // Create CtrlData struct for movement (always create, might be zero)
                    DJI::OSDK::Control::CtrlData ctrlData(controlFlag, velocity_x, velocity_y, 0, 0); // flag, x, y, z, yaw(rate)

                    // Log computed velocities and target/current values
                    std::cout << "Control Status: TargetWall=" << targetDistance
                                << " | CurrentWall=" << (hasAnonData ? std::to_string(lowestRange) : "N/A")
                                << " | TargetAzimuth=0"
                                << " | CurrentAzimuth=" << (foundTargetBeacon && !std::isnan(targetBeaconAzimuth) ? std::to_string(targetBeaconAzimuth) : "N/A")
                                << " | Computed Velocity(X=" << velocity_x << ", Y=" << velocity_y << ") (Body Frame)" << std::endl;

                    // Send control command using the base Control object
                    vehicle->control->flightCtrl(ctrlData);


                    // Optional: Add specific log if holding position vs adjusting
                    if (velocity_x == 0.0f && velocity_y == 0.0f) {
                         // std::cout << "--> Holding position (within dead zones)." << std::endl;
                    } else {
                         // std::cout << "--> Sending adjustment command." << std::endl;
                    }


                } else if (hasAnonData || foundTargetBeacon) { // Only print if wall or beacon data was found (and control was intended but vehicle/control is null)
                     std::cout << "(Flight Control Disabled or Not Available)\n";
                }
                 // --- End Movement Logic ---

            }

            // Update to the new second and reset tracking variables for the *next* second
            // ***** NOTE: These assignments use the GLOBAL state vars which are reset on disconnect *****
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();
            hasAnonData = false;
            targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN(); // Reset beacon azimuth
            foundTargetBeacon = false; // Reset beacon found flag
        }

        // --- Data Accumulation for the Current Second ---

        // Process BEACON IDs (Generic - keep for potential other beacons)
        if (obj.ID.find("BEACON") != std::string::npos && obj.ID != TARGET_BEACON_ID) {
            size_t firstDigit = obj.ID.find_first_of("0123456789");
            if (firstDigit != std::string::npos) {
                std::string numericalValue = obj.ID.substr(firstDigit);
                // std::cout << "Other Beacon " << numericalValue << " located at " << obj.Range << " meters\n"; // Optional logging
            } else {
                 // std::cout << "Other Beacon (no number) located at " << obj.Range << " meters\n"; // Optional logging
            }
        }

        // Process anon IDs (potential wall returns)
        if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) {
                lowestRange = obj.Range;
            }
        }

        // ***** ADDED ***** Process specific target beacon
        // Store the azimuth of the *first* detection of the target beacon within this second
        if (!foundTargetBeacon && obj.ID == TARGET_BEACON_ID) {
            targetBeaconAzimuth = obj.Az;
            foundTargetBeacon = true;
            // Optional: Log instant detection
            // std::cout << "Target Beacon (" << TARGET_BEACON_ID << ") detected this second at Azimuth: " << targetBeaconAzimuth << std::endl;
        }

        // --- End Data Accumulation ---


         if (stopProcessingFlag.load()) return; // Check flag again after processing
    }
}
// parseRadarData - prints raw data ONLY on parse error
std::vector<RadarObject> parseRadarData(const std::string& jsonData) {
    std::vector<RadarObject> radarObjects;
    if (jsonData.empty() || jsonData == "{}") { // Handle empty or minimal JSON
        return radarObjects;
    }

    // Use json::parse directly, exceptions will be caught by the caller
    try { // Added try-catch specifically around parse
        auto jsonFrame = json::parse(jsonData); // This might throw

        // Check if "objects" key exists and is an array
        if (!jsonFrame.contains("objects") || !jsonFrame["objects"].is_array()) { // Corrected key name
            // Treat it as valid but empty for now.
            return radarObjects; // Return empty vector
        }

        for (const auto& obj : jsonFrame["objects"]) {
            // Basic check if obj is an object
            if (!obj.is_object()) {
                std::cerr << "Skipping non-object item in 'objects' array." << std::endl;
                continue;
            }

            RadarObject radarObj;
            // Use .value("key", default_value) for robustness against missing keys

            // Use .dump() for timestamp to handle numbers or strings
            radarObj.timestamp = obj.value("timestamp", json(nullptr)).dump();

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
            // Handle potential non-string ID safely
            if (obj.contains("ID") && obj["ID"].is_string()) {
                 radarObj.ID = obj.value("ID", "N/A");
            } else if (obj.contains("ID")) {
                 // If ID exists but isn't a string, dump it to string representation
                 radarObj.ID = obj["ID"].dump();
                 // std::cerr << "Warning: Radar object ID is not a string: " << radarObj.ID << std::endl;
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
        // This catch remains in processingLoopFunction, but added one here for immediate feedback
        std::cerr << "JSON Parsing Error in parseRadarData: " << e.what() << " at offset " << e.byte << std::endl;
        std::cerr << "Problematic raw JSON data: [" << jsonData << "]" << std::endl;
        // Return empty vector on parse error to prevent further issues
        return {};
    } catch (const json::type_error& e) {
        // Catch potential type errors during value access (though .value() helps)
        std::cerr << "JSON Type Error in parseRadarData: " << e.what() << std::endl;
        std::cerr << "Problematic raw JSON data: [" << jsonData << "]" << std::endl;
        return {};
    }


    // Note: Type errors (e.g., trying to read a string as a number) within the loop
    // might still throw json::type_error, which will be caught by the caller.

    return radarObjects;
}


// Function to run the python bridge script
void runPythonBridge(const std::string& scriptName) {
    std::cout << "Starting Python bridge (" << scriptName << ")...\n";
    int result = std::system(("python3 " + scriptName + " &").c_str());
    if (result != 0) {
        std::cerr << "Failed to start Python bridge script '" << scriptName << "'. Error code: " << result << "\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(3)); // Give script time to start
    std::cout << "Python bridge potentially started.\n";
}

// Function to stop the python bridge script (Using SIGTERM)
void stopPythonBridge(const std::string& scriptName) {
    std::cout << "Stopping Python bridge (" << scriptName << ")...\n";
    // Use default pkill (sends SIGTERM)
    int result = std::system(("pkill -f " + scriptName).c_str());
    // No error check needed - it's okay if the process wasn't found
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait 1 second for OS
    std::cout << "Sent SIGTERM and waited briefly for " << scriptName << ".\n";
}

// Function to connect to the python bridge via TCP
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket) {
    tcp::resolver resolver(io_context);
    int retries = 5; // Number of connection attempts
    while (retries-- > 0) {
        try {
            // Ensure socket is closed before attempting a new connection in the loop
            if(socket.is_open()) {
                 boost::system::error_code ignored_ec;
                 socket.shutdown(tcp::socket::shutdown_both, ignored_ec);
                 socket.close();
            }

            auto endpoints = resolver.resolve("127.0.0.1", "5000");
            boost::system::error_code ec;
            boost::asio::connect(socket, endpoints, ec); // Attempt connection
            if (!ec) {
                 std::cout << "Connected to Python bridge.\n";
                 return true; // Success
            } else {
                 std::cerr << "Connection attempt failed: " << ec.message() << "\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception during connection attempt: " << e.what() << "\n";
        }

        // Wait before retrying, but check stop flag
        if(retries > 0 && !stopProcessingFlag.load()) {
             std::cout << "Retrying connection in 2 seconds... (" << retries << " attempts left)\n";
             std::this_thread::sleep_for(std::chrono::seconds(2));
        } else if (stopProcessingFlag.load()) {
             std::cout << "Stop requested during connection retry.\n";
             break; // Exit retry loop if stop requested
        }
    }
     // If loop finishes without success
     std::cerr << "Failed to connect to Python bridge after multiple attempts.\n";
     return false; // Failure
}

// Callbacks for obtaining/releasing control (Placeholders - not used in current logic)
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) {
    if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ObtainJoystickCtrlAuthoritySuccess) {
        DSTATUS("ObtainJoystickCtrlAuthoritySuccess (Callback - Currently Unused)");
    } else { DERROR("Failed to obtain Control Authority (Callback - Currently Unused). ErrorCode: %d", errorCode); }
}

void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) {
     if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ReleaseJoystickCtrlAuthoritySuccess) {
        DSTATUS("ReleaseJoystickCtrlAuthoritySuccess (Callback - Currently Unused)");
    } else { DERROR("Failed to release Control Authority (Callback - Currently Unused). ErrorCode: %d", errorCode); }
}


// Enum for different processing modes
enum class ProcessingMode {
    WALL_FOLLOW,             // Process wall data and potentially move drone
    PROCESS_FULL,            // Process and display full radar data
    PROCESS_MINIMAL,         // Process and display minimal radar data
    // PROCESS_BEACON_WALL_ONLY is handled by calling WALL_FOLLOW with enableControl=false
};


// ***** REVISED processingLoopFunction with CORRECT Reconnection Logic & State Reset *****
void processingLoopFunction(const std::string bridgeScriptName, Vehicle* vehicle, bool enableControlCmd, ProcessingMode mode) {
    std::cout << "Processing thread started. Bridge: " << bridgeScriptName << ", Control Enabled: " << std::boolalpha << enableControlCmd << std::endl;

    int total_reconnect_attempts = 0;
    const int MAX_RECONNECT_ATTEMPTS = 3; // Limit overall attempts to restart the bridge/connection

    // Outer loop manages the overall connection lifecycle (including reconnects)
    while (!stopProcessingFlag.load()) { // Check stop flag at the start of each connection attempt cycle

        // --- Connection Phase ---
        if (total_reconnect_attempts > 0) { // Log if this is a reconnect attempt
             std::cout << "--- Reconnect Attempt " << total_reconnect_attempts << " ---" << std::endl;
        }

        runPythonBridge(bridgeScriptName); // Start/Restart the bridge script

        boost::asio::io_context io_context;
        tcp::socket socket(io_context);

        if (!connectToPythonBridge(io_context, socket)) {
            std::cerr << "Processing thread: Connection failed for this attempt.\n";
            stopPythonBridge(bridgeScriptName); // Ensure bridge is stopped after failed connection attempt
            total_reconnect_attempts++;
            if (total_reconnect_attempts >= MAX_RECONNECT_ATTEMPTS) {
                 std::cerr << "Maximum reconnect attempts (" << MAX_RECONNECT_ATTEMPTS << ") reached. Giving up.\n";
                 break; // Exit outer loop - giving up on reconnecting
            }
            // Check stop flag before waiting
            if (stopProcessingFlag.load()) {
                 std::cout << "Stop requested after failed connection attempt.\n";
                 break; // Exit outer loop
            }
            std::cout << "Waiting 5 seconds before next reconnect attempt ("
                      << total_reconnect_attempts << "/" << MAX_RECONNECT_ATTEMPTS << ")...\n";
            // Sleep in smaller chunks to check stop flag more frequently
             for (int i = 0; i < 5; ++i) {
                 if (stopProcessingFlag.load()) break;
                 std::this_thread::sleep_for(std::chrono::seconds(1));
             }
             if (stopProcessingFlag.load()) {
                  std::cout << "Stop requested during reconnect wait.\n";
                  break; // Exit outer loop
             }
            continue; // Go to next iteration of outer loop to retry connection
        }

        // --- Data Processing Phase (If Connection Successful) ---
        // Reset total reconnect attempts if connection successful
        total_reconnect_attempts = 0;
        std::cout << "Processing thread: Connection successful. Reading data stream...\n";

        std::string received_data_buffer;
        std::array<char, 4096> read_buffer;
        bool connection_lost_in_inner_loop = false; // Flag to indicate why inner loop exited

        // Inner loop reads data until error or stop signal
        while (!stopProcessingFlag.load()) {
            boost::system::error_code error;
            size_t len = socket.read_some(boost::asio::buffer(read_buffer), error);

            if (error) { // Handle any socket read error
                if (error == boost::asio::error::eof) {
                    std::cerr << "Processing thread: Connection closed by Python bridge (EOF).\n";
                } else {
                    std::cerr << "Processing thread: Error reading from socket: " << error.message() << "\n";
                }

                // Clean up the socket resource immediately
                if (socket.is_open()) {
                     boost::system::error_code ignored_ec;
                     socket.shutdown(tcp::socket::shutdown_both, ignored_ec);
                     socket.close();
                }
                connection_lost_in_inner_loop = true; // Mark that connection was lost
                break; // Break inner loop to trigger reconnection logic below
            }

            // Process data if read was successful
            if (len > 0) {
                received_data_buffer.append(read_buffer.data(), len);
                size_t newline_pos;
                // Process all complete lines found in the buffer
                while ((newline_pos = received_data_buffer.find('\n')) != std::string::npos) {
                    std::string jsonData = received_data_buffer.substr(0, newline_pos);
                    received_data_buffer.erase(0, newline_pos + 1);

                    // Check stop flag frequently within data processing
                    if (stopProcessingFlag.load()) break;

                    if (!jsonData.empty()) {
                        try {
                            auto radarObjects = parseRadarData(jsonData);
                            if (!radarObjects.empty()) {
                                // Process data based on the mode set when the thread was launched
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
                                }
                            }
                        } catch (const std::exception& e) {
                             // Catch potential errors during parsing or data extraction
                             std::cerr << "Error processing data: " << e.what() << "\nSnippet: [" << jsonData.substr(0, 100) << "...]\n";
                        }
                    }
                } // End while processing lines in buffer

                // Check stop flag again after processing buffer contents
                if (stopProcessingFlag.load()) break;

            } else {
                 // No data read, but no error? Could happen. Brief pause.
                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } // End inner data reading loop


        // --- Post-Inner Loop Handling ---
        if (stopProcessingFlag.load()) {
             std::cout << "Stop requested, exiting processing loop.\n";
             // Ensure socket is closed if inner loop exited due to stop flag
             if (socket.is_open()) {
                 boost::system::error_code ignored_ec;
                 socket.shutdown(tcp::socket::shutdown_both, ignored_ec);
                 socket.close();
             }
             break; // Exit outer loop if stopped manually
        }

        // If we are here, the inner loop exited because connection_lost_in_inner_loop is true
        if (connection_lost_in_inner_loop) {
             std::cout << "Connection lost detected. Handling reconnect...\n";
             stopPythonBridge(bridgeScriptName); // Stop the potentially dead bridge

             // ***** ADDED: Reset persistent processing state before reconnect attempt *****
             std::cout << "Resetting processing state variables (currentSecond, etc.)...\n";
             currentSecond = ""; // Reset the last known second
             lowestRange = std::numeric_limits<float>::max(); // Reset wall tracking
             hasAnonData = false;                             // Reset wall tracking
             targetBeaconAzimuth = std::numeric_limits<float>::quiet_NaN(); // Reset beacon tracking
             foundTargetBeacon = false;                                     // Reset beacon tracking
             // ***************************************************************************


             // Increment attempt counter before checking limit
             total_reconnect_attempts++;
             if (total_reconnect_attempts >= MAX_RECONNECT_ATTEMPTS) {
                  std::cerr << "Maximum reconnect attempts (" << MAX_RECONNECT_ATTEMPTS << ") reached. Giving up.\n";
                  break; // Exit outer loop
             }

             // Wait before the next attempt, checking for stop signal during wait
             std::cout << "Waiting 5 seconds before reconnect attempt "
                       << total_reconnect_attempts << "/" << MAX_RECONNECT_ATTEMPTS << "...\n";
             // Sleep in smaller chunks to check stop flag more frequently
             for (int i = 0; i < 5; ++i) {
                 if (stopProcessingFlag.load()) break;
                 std::this_thread::sleep_for(std::chrono::seconds(1));
             }

             if (stopProcessingFlag.load()) {
                  std::cout << "Stop requested during reconnect wait.\n";
                  break; // Exit outer loop
             }

             // If not stopped and attempts remain, continue to the next iteration of the outer loop
             std::cout << "Proceeding to next reconnect attempt...\n";
             // The 'continue' is implicit as the outer loop's condition will be checked again

        } else {
             // Should not happen if stop flag check is correct, but as a safeguard:
             std::cout << "Inner loop exited unexpectedly without stop flag or connection loss.\n";
             break; // Exit outer loop
        }

    } // End outer loop (reconnection attempts or stop signal)

    std::cout << "Processing thread: Exiting outer loop.\n";

    // Final cleanup attempt for the bridge script, just in case
    stopPythonBridge(bridgeScriptName);
    std::cout << "Processing thread finished.\n";
    // Only print the menu prompt when the thread *actually* finishes and exits.
    std::cout << "\n>>> Press Enter to return to the main menu... <<<\n";
}
// ***** END REVISED processingLoopFunction *****


int main(int argc, char** argv) {
    // Load preferences for target distance and lateral control
    loadPreferences();

    // --- Initial Mode Selection ---
    char modeSelection;
    bool enableFlightControl = false; // Flag if OSDK should be initialized and flight commands allowed
    std::cout << "Select Operating Mode:\n";
    std::cout << "  (a) Live Mode (Enables Flight Control, uses live data bridge: python_bridge.py)\n";
    std::cout << "  (b) Test Mode (Disables Flight Control, uses test data bridge: python_bridge_LOCAL.py)\n";
    while (true) {
        std::cout << "Enter your choice: ";
        // Use getline for safer input
        std::string lineInput;
        if (std::getline(std::cin, lineInput) && !lineInput.empty()) {
            modeSelection = lineInput[0];
        } else {
             modeSelection = 0; // Invalid input
        }

        if (modeSelection == 'a' || modeSelection == 'A') {
            std::cout << "Live Mode selected. Flight control enabled. Default bridge: python_bridge.py\n";
            enableFlightControl = true;
            defaultPythonBridgeScript = "python_bridge.py";
            break;
        } else if (modeSelection == 'b' || modeSelection == 'B') {
            std::cout << "Test Mode selected. Flight control disabled. Default bridge: python_bridge_LOCAL.py\n";
            enableFlightControl = false;
            defaultPythonBridgeScript = "python_bridge_LOCAL.py";
            break;
        } else {
            std::cout << "Invalid selection. Please choose 'a' or 'b'.\n";
        }
    }

    // --- OSDK Initialization (if flight control enabled) ---
    int functionTimeout = 1; // Timeout for async calls in SECONDS
    Vehicle* vehicle = nullptr;
    FlightSample* flightSample = nullptr; // Still needed for Takeoff/Landing wrappers
    LinuxSetup* linuxEnvironment = nullptr;

    if (enableFlightControl) {
        std::cout << "Initializing DJI OSDK...\n";
        linuxEnvironment = new LinuxSetup(argc, argv);
        vehicle = linuxEnvironment->getVehicle();
        if (vehicle == nullptr || vehicle->control == nullptr) {
            std::cerr << "ERROR: Vehicle not initialized or control interface unavailable. Flight control will be disabled.\n";
             if (linuxEnvironment) delete linuxEnvironment; linuxEnvironment = nullptr;
             vehicle = nullptr;
             enableFlightControl = false;
             std::cout << "Switching to Test Mode functionality due to OSDK connection issue.\n";
             defaultPythonBridgeScript = "python_bridge_LOCAL.py";
        } else {
            std::cout << "Attempting to obtain Control Authority...\n";
            ACK::ErrorCode ctrlAuthAck = vehicle->control->obtainCtrlAuthority(functionTimeout);
            if (ACK::getError(ctrlAuthAck)) {
                 ACK::getErrorCodeMessage(ctrlAuthAck, __func__);
                 std::cerr << "Failed to obtain control authority. Disabling flight control." << std::endl;
                 if (linuxEnvironment) delete linuxEnvironment; linuxEnvironment = nullptr;
                 vehicle = nullptr;
                 enableFlightControl = false;
                 std::cout << "Switching to Test Mode functionality due to authority failure.\n";
                 defaultPythonBridgeScript = "python_bridge_LOCAL.py";
            } else {
                 std::cout << "Obtained Control Authority.\n";
                 flightSample = new FlightSample(vehicle);
                 std::cout << "OSDK Initialized and Flight Sample created.\n";
            }
        }
    } else {
         std::cout << "Running without Flight Control (Test Mode selected or OSDK init failed).\n";
    }

    if (enableFlightControl) {
        std::cout << "INFO: Flight control is ENABLED proceeding to main menu.\n";
    } else {
        std::cout << "INFO: Flight control is DISABLED proceeding to main menu.\n";
    }


    // --- Main Command Loop ---
    bool keepRunning = true;
    while (keepRunning) {
        std::cout << "\n--- Main Menu ---\n";
        if (enableFlightControl) {
             std::cout << "| [t] Monitored Takeoff                     |\n";
             std::cout << "| [l] Monitored Landing                     |\n";
             std::cout << "| [w] Start Wall+Beacon Following (Default) |\n";
             std::cout << "| [h] Start Wall+Beacon Following (TEST)    |\n";
        } else {
             std::cout << "| [t] Takeoff (DISABLED)                    |\n";
             std::cout << "| [l] Landing (DISABLED)                    |\n";
             std::cout << "| [w] Wall/Beacon Following (No Control)    |\n";
             std::cout << "| [h] Wall/Beacon Following (No Control)    |\n";
        }
        std::cout << "| [e] Process Full Radar (No Control)       |\n";
        std::cout << "| [f] Process Minimal Radar (No Control)    |\n";
        std::cout << "| [g] Process Beacon/Wall Only (No Control) |\n";
        std::cout << "| [s] STOP Current Processing Thread        |\n";
        std::cout << "| [q] Quit                                  |\n";
        std::cout << "---------------------------------------------\n";
        std::cout << "Enter command: ";

        std::string lineInput;
        char inputChar = 0;

        if (std::getline(std::cin, lineInput)) {
             if (!lineInput.empty()) {
                 inputChar = lineInput[0];
             } else {
                 inputChar = 0; // User pressed Enter
             }
        } else {
             inputChar = 'q'; // Treat EOF (Ctrl+D) as quit
             std::cout << "\nInput stream closed. Exiting.\n";
        }


        // --- Stop existing thread if a new processing task is started or quitting ---
         // This logic ensures the previous thread is stopped and joined before starting a new one.
         if (strchr("wehgs", inputChar) != nullptr || inputChar == 'q') {
             if (processingThread.joinable()) {
                 std::cout << "Signalling previous processing thread to stop...\n"; // Changed log
                 stopProcessingFlag.store(true); // Signal the thread to stop its loops
                 processingThread.join();        // Wait for the thread function to complete its cleanup and exit
                 std::cout << "Previous processing thread has finished.\n"; // Changed log
                 stopProcessingFlag.store(false); // Reset flag for the potential next thread
             }
         }


        // --- Process Command ---
        switch (inputChar) {
            case 't': // Takeoff
                if (enableFlightControl && flightSample) {
                    // Consider adding obtain/release authority around specific actions if needed
                    flightSample->monitoredTakeoff();
                } else {
                     std::cout << "Flight control not enabled or available.\n";
                }
                break;
            case 'l': // Landing
                 if (enableFlightControl && flightSample) {
                    // Consider adding obtain/release authority around specific actions if needed
                    flightSample->monitoredLanding();
                } else {
                     std::cout << "Flight control not enabled or available.\n";
                }
                 break;

             case 'w': // Start Wall Following (Default Bridge, Control if Enabled)
                 std::cout << "Starting Wall+Beacon Following using default bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false); // Ensure flag is reset before starting thread
                 // Launch the thread with its own reconnection logic
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW);
                 break;

             case 'h': // Start Wall Following (TEST Bridge, Control if Enabled)
                 if (!enableFlightControl) {
                     std::cout << "Error: Flight control must be enabled (select Live Mode 'a' at start) to use this option effectively.\n";
                 } else {
                     std::cout << "Starting Wall+Beacon Following using TEST bridge: python_bridge_LOCAL.py\n";
                     stopProcessingFlag.store(false); // Ensure flag is reset
                     // Launch the thread with its own reconnection logic
                     processingThread = std::thread(processingLoopFunction, "python_bridge_LOCAL.py", vehicle, enableFlightControl, ProcessingMode::WALL_FOLLOW);
                 }
                 break;

             case 'e': // Process Full Radar Data (Default Bridge, No Control)
                 std::cout << "Starting Full Radar Data processing using default bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, ProcessingMode::PROCESS_FULL);
                 break;

             case 'f': // Process Minimal Radar Data (Default Bridge, No Control)
                  std::cout << "Starting Minimal Radar Data processing using default bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, ProcessingMode::PROCESS_MINIMAL);
                 break;

             case 'g': // Process Beacon/Wall Only (Default Bridge, No Control)
                  std::cout << "Starting Beacon/Wall Data processing (display only) using default bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false);
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, ProcessingMode::WALL_FOLLOW);
                 break;

            case 's': // STOP Current Processing Thread
                 // The stop/join logic is handled at the start of the loop if a new command is issued.
                 // This 's' case just signals the thread without waiting here.
                 if (processingThread.joinable()) {
                     std::cout << "Stop command received. Signalling active thread to stop...\n";
                     stopProcessingFlag.store(true); // Signal the thread
                 } else {
                     std::cout << "No processing thread is currently active.\n";
                 }
                 // No need for the "Press Enter" prompt here, main loop will show menu again.
                break;

            case 'q': { // Quit
                std::cout << "Exiting...\n";
                // Stop thread handled above (ensures thread is joined before proceeding)

                // Release authority and cleanup OSDK resources
                if (enableFlightControl && vehicle && vehicle->control) {
                    std::cout << "Sending Zero Velocity command before releasing control..." << std::endl;
                    uint8_t controlFlag = DJI::OSDK::Control::HORIZONTAL_VELOCITY | DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::YAW_RATE | DJI::OSDK::Control::HORIZONTAL_BODY | DJI::OSDK::Control::STABLE_ENABLE;
                    DJI::OSDK::Control::CtrlData stopData(controlFlag, 0, 0, 0, 0);
                    vehicle->control->flightCtrl(stopData);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Short delay

                    std::cout << "Releasing Control Authority...\n";
                    ACK::ErrorCode releaseAck = vehicle->control->releaseCtrlAuthority(functionTimeout);
                     if (ACK::getError(releaseAck)) {
                         ACK::getErrorCodeMessage(releaseAck, __func__);
                         std::cerr << "Warning: Failed to release control authority." << std::endl;
                     } else {
                         std::cout << "Control Authority Released.\n";
                     }
                }
                 // Cleanup OSDK environment objects
                 if (linuxEnvironment) {
                    delete linuxEnvironment; linuxEnvironment = nullptr;
                 }
                 if (flightSample) {
                     delete flightSample; flightSample = nullptr;
                 }

                keepRunning = false; // Exit main loop
                break;
            }

            case 0: // User pressed Enter
                 // Do nothing, loop will redisplay menu
                 break;

            default:
                std::cout << "Invalid input.\n";
                break;
        } // End switch
    } // End while(keepRunning)

    // Final check to join thread if loop exited unexpectedly (e.g., error)
     if (processingThread.joinable()) {
         std::cerr << "Warning: Main loop exited unexpectedly, ensuring processing thread is stopped.\n";
         stopProcessingFlag.store(true);
         processingThread.join();
     }
     // Final cleanup of OSDK resources if not done via 'q'
      if (linuxEnvironment) {
         delete linuxEnvironment; linuxEnvironment = nullptr;
      }
      if (flightSample) {
          delete flightSample; flightSample = nullptr;
      }

    std::cout << "Program terminated.\n";
    return 0;
}
