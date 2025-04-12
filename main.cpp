#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstdlib>  // For std::system
#include <json.hpp> // Include the JSON library
#include <boost/asio.hpp>    // Include Boost ASIO for TCP communication
#include "flight_control_sample.hpp"
#include "flight_sample.hpp"
#include "dji_linux_helpers.hpp"
#include <limits> // For numeric limits
#include <fstream> // For file reading
#include <cmath> // For std::abs
#include <atomic> // For thread-safe stop flag
#include <cstring> // For strchr
#include <stdexcept> // For standard exceptions
#include <array> // For std::array used in read_some buffer

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using json = nlohmann::json;
using boost::asio::ip::tcp;

// Persistent variables for tracking the current second and wall candidate data
std::string currentSecond = "";  // Tracks the current second being processed
float lowestRange = std::numeric_limits<float>::max();  // Tracks the lowest range for the current second
bool hasAnonData = false;  // Tracks whether any "anon" ID data exists for this second

// Global variable for default Python bridge script (set by initial mode selection)
std::string defaultPythonBridgeScript = "python_bridge.py";

// Global variable for target distance
float targetDistance = 8.0f;

// Flag to control the processing loop in a separate thread
std::atomic<bool> stopProcessingFlag(false);
std::thread processingThread; // Thread object for radar processing

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
        }
        preferencesFile.close();
    } else {
        std::cout << "Preferences file not found. Using default target distance: " << targetDistance << " meters.\n";
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

// Modified function accepts FlightSample pointer (can be nullptr)
// Now also includes logic to stop based on stopProcessingFlag
void extractBeaconAndWallData(const std::vector<RadarObject>& objects, FlightSample* flightSample, bool enableControl) {
     // Reset persistent variables at the start of processing a new batch if necessary
     // This might depend on how frequently this function is called relative to data batches
     // For now, assuming it processes a stream and state persists across calls within a run.

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
        if (!currentSecond.empty() && objSecond < currentSecond) {
            // std::cout << "Skipping old data: " << obj.timestamp << std::endl; // Debugging
            continue;
        }

        // If this is a new second
        if (objSecond != currentSecond) {
            // Output the result for the previous second, if any, and command drone movement
            if (!currentSecond.empty() && hasAnonData) {
                std::cout << currentSecond << ": Likely WALL candidate distance: " << lowestRange << " meters\n";

                // --- Movement Logic ---
                if (enableControl && flightSample != nullptr) { // Check BOTH flag and pointer

                    // ***** CORRECTED DIFFERENCE CALCULATION *****
                    float difference = lowestRange - targetDistance; // Error = current - target
                    // ***** END CORRECTION *****

                    float Kp = 0.5; // Proportional gain (NEEDS TUNING)
                    float max_speed = 0.8; // Maximum speed in m/s (NEEDS TUNING)
                    float dead_zone = 0.2; // Distance tolerance in meters (NEEDS TUNING)
                    uint32_t command_duration_ms = 200; // Duration to send velocity command (NEEDS TUNING)

                    if (std::abs(difference) > dead_zone) { // Only move if outside the dead zone
                        // Note: The sign of velocity_x is now correct due to the changed difference calculation
                        float velocity_x = Kp * difference;
                        // Clamp velocity to max_speed
                        velocity_x = std::max(-max_speed, std::min(max_speed, velocity_x));

                        std::cout << "Adjusting position: Current=" << lowestRange
                                  << ", Target=" << targetDistance
                                  << ", VelocityX=" << velocity_x << std::endl;

                        // Send velocity command (assuming X is forward/backward axis)
                        flightSample->velocityAndYawRateCtrl({velocity_x, 0, 0}, 0, command_duration_ms);

                    } else {
                        std::cout << "Wall is within target distance tolerance.\n";
                        // Send zero velocity to stop any residual movement
                        flightSample->velocityAndYawRateCtrl({0, 0, 0}, 0, command_duration_ms);
                    }
                } else if (hasAnonData) { // Only print if wall data was found
                     std::cout << "(Flight Control Disabled or Not Available)\n";
                }
                 // --- End Movement Logic ---

            }

            // Update to the new second and reset tracking variables
            currentSecond = objSecond;
            lowestRange = std::numeric_limits<float>::max();  // Reset lowest range
            hasAnonData = false;  // Reset flag for anon data
        }

        // Process BEACON IDs
        if (obj.ID.find("BEACON") != std::string::npos) {
            size_t firstDigit = obj.ID.find_first_of("0123456789");
            if (firstDigit != std::string::npos) {
                std::string numericalValue = obj.ID.substr(firstDigit);
                std::cout << "Beacon " << numericalValue << " located at " << obj.Range << " meters\n";
            } else {
                 std::cout << "Beacon (no number) located at " << obj.Range << " meters\n";
            }
        }

        // Process anon IDs (potential wall returns)
        if (obj.ID.find("anon") != std::string::npos) {
            hasAnonData = true;
            if (obj.Range < lowestRange) {
                lowestRange = obj.Range;
            }
        }
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
    auto jsonFrame = json::parse(jsonData); // This might throw

    // Check if "objects" key exists and is an array
    if (!jsonFrame.contains("objects") || !jsonFrame["objects"].is_array()) {
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
        radarObj.ID = obj.value("ID", "N/A"); // Assume ID is usually a string
        radarObj.Xsize = obj.value("Xsize", 0.0f);
        radarObj.Ysize = obj.value("Ysize", 0.0f);
        radarObj.Zsize = obj.value("Zsize", 0.0f);
        radarObj.Conf = obj.value("Conf", 0.0f);

        radarObjects.push_back(radarObj);
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
    if (result != 0) {
        // pkill returns non-zero if the process wasn't found (already stopped) or on error.
        // std::cerr << "pkill command may have failed or process not found for " << scriptName << ". Result: " << result << std::endl;
    }
    // Keep a small delay to allow the OS time to potentially process the signal
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait 0.5 seconds
    std::cout << "Sent SIGTERM and waited briefly for " << scriptName << ".\n";
}

// Function to connect to the python bridge via TCP
bool connectToPythonBridge(boost::asio::io_context& io_context, tcp::socket& socket) {
    tcp::resolver resolver(io_context);
    int retries = 5;
    while (retries-- > 0) {
        try {
            auto endpoints = resolver.resolve("127.0.0.1", "5000");
            boost::system::error_code ec;
            boost::asio::connect(socket, endpoints, ec);
            if (!ec) {
                 std::cout << "Connected to Python bridge.\n";
                 return true; // Success
            } else {
                 std::cerr << "Connection attempt failed: " << ec.message() << "\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "Error during connection attempt: " << e.what() << "\n";
        }
        if(retries > 0) {
             std::cout << "Retrying connection in 2 seconds... (" << retries << " attempts left)\n";
             std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
     std::cerr << "Failed to connect to Python bridge after multiple attempts.\n";
     return false; // Failure
}

// Callbacks for obtaining/releasing control
void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) {
    if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ObtainJoystickCtrlAuthoritySuccess) {
        DSTATUS("ObtainJoystickCtrlAuthoritySuccess");
    } else {
        DERROR("Failed to obtain Joystick Control Authority. ErrorCode: %d", errorCode);
    }
}

void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData) {
    if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ReleaseJoystickCtrlAuthoritySuccess) {
        DSTATUS("ReleaseJoystickCtrlAuthoritySuccess");
    } else {
         DERROR("Failed to release Joystick Control Authority. ErrorCode: %d", errorCode);
    }
}

// Enum for different processing modes
enum class ProcessingMode {
    WALL_FOLLOW,             // Process wall data and potentially move drone
    PROCESS_FULL,            // Process and display full radar data
    PROCESS_MINIMAL,         // Process and display minimal radar data
    PROCESS_BEACON_WALL_ONLY // Process beacon/wall data, no movement
};


// ***** REVISED processingLoopFunction with socket.read_some *****
void processingLoopFunction(const std::string bridgeScriptName, FlightSample* flightSample, bool enableControlCmd, ProcessingMode mode) {
    std::cout << "Processing thread started. Bridge: " << bridgeScriptName << ", Control Enabled: " << std::boolalpha << enableControlCmd << std::endl;

    runPythonBridge(bridgeScriptName); // Start the specific bridge script

    boost::asio::io_context io_context;
    tcp::socket socket(io_context);

    if (!connectToPythonBridge(io_context, socket)) {
        std::cerr << "Processing thread: Failed to connect to Python bridge. Stopping bridge and exiting thread.\n";
        stopPythonBridge(bridgeScriptName);
        std::cout << "\n>>> Press Enter to return to the main menu... <<<\n"; // Prompt user
        return;
    }

    std::cout << "Processing thread: Reading data stream...\n";

    std::string received_data_buffer; // Buffer to hold potentially incomplete data across reads
    std::array<char, 4096> read_buffer; // Temporary buffer for each read_some call

    while (!stopProcessingFlag.load()) {
        boost::system::error_code error;
        // Read whatever data is available (up to buffer size)
        size_t len = socket.read_some(boost::asio::buffer(read_buffer), error);

        if (error == boost::asio::error::eof) {
            std::cerr << "Processing thread: Connection closed by Python bridge.\n";
            break; // Connection closed cleanly
        } else if (error) {
            std::cerr << "Processing thread: Error reading from socket: " << error.message() << "\n";
            break; // Other error
        }

        if (len > 0) {
            // Append newly read data to our persistent buffer
            received_data_buffer.append(read_buffer.data(), len);

            // Process all complete lines found in the buffer
            size_t newline_pos;
            while ((newline_pos = received_data_buffer.find('\n')) != std::string::npos) {
                // Extract the complete line (up to, but not including, the newline)
                std::string jsonData = received_data_buffer.substr(0, newline_pos);

                // Remove the processed line (including the newline) from the buffer
                received_data_buffer.erase(0, newline_pos + 1);

                if (stopProcessingFlag.load()) break; // Check flag frequently

                if (!jsonData.empty()) {
                    try {
                        auto radarObjects = parseRadarData(jsonData);
                        if (!radarObjects.empty()) {
                           switch (mode) {
                                case ProcessingMode::WALL_FOLLOW:
                                    extractBeaconAndWallData(radarObjects, flightSample, enableControlCmd);
                                    break;
                                case ProcessingMode::PROCESS_FULL:
                                    displayRadarObjects(radarObjects);
                                    break;
                                case ProcessingMode::PROCESS_MINIMAL:
                                    displayRadarObjectsMinimal(radarObjects);
                                    break;
                                case ProcessingMode::PROCESS_BEACON_WALL_ONLY:
                                    extractBeaconAndWallData(radarObjects, nullptr, false);
                                    break;
                            }
                        }
                    } catch (const json::parse_error& e) {
                         std::cerr << "DEBUG: Raw data causing error: [" << jsonData << "]" << std::endl;
                         std::cerr << "JSON Parsing Error: " << e.what() << " at offset " << e.byte << std::endl;
                    } catch (const std::exception& e) {
                         std::cerr << "Error processing received data: " << e.what() << std::endl;
                         std::cerr << "Problematic data was: [" << jsonData << "]" << std::endl;
                    }
                } else {
                    // std::cout << "Received empty line." << std::endl; // Debugging
                }
            } // End while processing lines in buffer
            if (stopProcessingFlag.load()) break; // Check flag after processing buffer contents
        }
        // Optional slight delay if CPU usage is high, but read_some is blocking so maybe not needed
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } // End while(!stopProcessingFlag.load())

    std::cout << "Processing thread: Loop finished or stopped.\n";
    if (socket.is_open()) {
        boost::system::error_code ignored_ec;
        socket.shutdown(tcp::socket::shutdown_both, ignored_ec);
        socket.close();
    }
    stopPythonBridge(bridgeScriptName);
    std::cout << "Processing thread finished.\n";
    std::cout << "\n>>> Press Enter to return to the main menu... <<<\n";
}
// ***** END REVISED processingLoopFunction *****


int main(int argc, char** argv) {
    // Load preferences for target distance
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
    FlightSample* flightSample = nullptr;
    LinuxSetup* linuxEnvironment = nullptr;

    if (enableFlightControl) {
        std::cout << "Initializing DJI OSDK...\n";
        // ***** Add OSDK Logging Configuration Here if possible *****
        // Example (Hypothetical - FIND THE REAL API CALL):
        // DJI::OSDK::Log::instance().setOutputStream(&std::cerr);
        // **********************************************************

        linuxEnvironment = new LinuxSetup(argc, argv);
        vehicle = linuxEnvironment->getVehicle();
        // Check if vehicle pointer is valid after initialization attempt
        if (vehicle == nullptr) {
            std::cerr << "ERROR: Vehicle not initialized or not connected. Flight control will be unavailable.\n";
             if (linuxEnvironment) delete linuxEnvironment; linuxEnvironment = nullptr;
             // vehicle = nullptr; // Already null
             enableFlightControl = false; // Force disable flight control
             std::cout << "Switching to Test Mode functionality due to OSDK connection issue.\n";
             defaultPythonBridgeScript = "python_bridge_LOCAL.py"; // Use local bridge if OSDK fails
        } else {
            std::cout << "Attempting to obtain Joystick Control Authority...\n";
            // Add timeout (in ms) and pkg_timeout (usually 2)
            vehicle->flightController->obtainJoystickCtrlAuthorityAsync(ObtainJoystickCtrlAuthorityCB, nullptr, functionTimeout * 1000, 2);
            std::this_thread::sleep_for(std::chrono::seconds(2)); // Allow time for callback
            flightSample = new FlightSample(vehicle);
            std::cout << "OSDK Initialized and Flight Sample created.\n";
        }
    } else {
         std::cout << "Running without Flight Control (Test Mode selected or OSDK init failed).\n";
    }


    // --- Main Command Loop ---
    bool keepRunning = true;
    while (keepRunning) {
        std::cout << "\n--- Main Menu ---\n";
        if (enableFlightControl) {
             std::cout << "| [t] Monitored Takeoff                     |\n";
             std::cout << "| [l] Monitored Landing                     |\n";
             std::cout << "| [w] Start Wall Following (Default Bridge) |\n";
             std::cout << "| [h] Start Wall Following (TEST Bridge)    |\n";
        } else {
             std::cout << "| [t] Takeoff (DISABLED)                    |\n";
             std::cout << "| [l] Landing (DISABLED)                    |\n";
             std::cout << "| [w] Wall Following (DISABLED - No Control)|\n";
             std::cout << "| [h] Wall Following (DISABLED - No Control)|\n";
        }
        std::cout << "| [e] Process Full Radar (No Control)       |\n";
        std::cout << "| [f] Process Minimal Radar (No Control)    |\n";
        std::cout << "| [g] Process Beacon/Wall Only (No Control) |\n";
        std::cout << "| [s] STOP Current Processing Thread        |\n";
        std::cout << "| [q] Quit                                  |\n";
        std::cout << "---------------------------------------------\n";
        std::cout << "Enter command: ";

        // Use std::getline for input
        std::string lineInput;
        char inputChar = 0; // Default to null char / invalid

        if (std::getline(std::cin, lineInput)) {
             if (!lineInput.empty()) {
                 inputChar = lineInput[0]; // Use the first character of the line
             } else {
                 // User pressed Enter without typing anything - treat as needing a refresh / invalid command
                 inputChar = 0; // Explicitly set to invalid/null
                 // std::cout << "Enter pressed. Displaying menu again.\n"; // Optional feedback
             }
        } else {
             // Error reading line (e.g., Ctrl+D / EOF)
             inputChar = 'q'; // Treat as quit
             std::cout << "\nInput stream closed. Exiting.\n";
        }


        // --- Stop existing thread if a new processing task is started or quitting ---
         if (strchr("wehgs", inputChar) != nullptr || inputChar == 'q') { // Check if input requires stopping previous task
             if (processingThread.joinable()) {
                 std::cout << "Stopping previous processing thread...\n";
                 stopProcessingFlag.store(true); // Signal thread to stop
                 processingThread.join();       // Wait for thread to finish
                 std::cout << "Previous thread stopped.\n";
                 stopProcessingFlag.store(false); // Reset flag for next use
             }
         }


        // --- Process Command ---
        switch (inputChar) {
            case 't': // Takeoff
                if (enableFlightControl && flightSample) {
                    flightSample->monitoredTakeoff();
                } else {
                     std::cout << "Flight control not enabled or available.\n";
                }
                break;
            case 'l': // Landing
                 if (enableFlightControl && flightSample) {
                    flightSample->monitoredLanding();
                } else {
                     std::cout << "Flight control not enabled or available.\n";
                }
                break;

             case 'w': // Start Wall Following (Default Bridge, Control if Enabled)
                 std::cout << "Starting Wall Following using default bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false); // Ensure flag is false before starting
                 // Start thread: uses default bridge, passes flightSample, enables control CMDs *only if* flight control is enabled overall
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, flightSample, enableFlightControl, ProcessingMode::WALL_FOLLOW);
                 break;

             case 'h': // Start Wall Following (TEST Bridge, Control if Enabled)
                 if (!enableFlightControl) {
                     std::cout << "Error: Flight control must be enabled (select Live Mode 'a' at start) to use this option.\n";
                 } else {
                     std::cout << "Starting Wall Following using TEST bridge: python_bridge_LOCAL.py\n";
                     stopProcessingFlag.store(false);
                     // Start thread: uses LOCAL bridge, passes flightSample, enables control CMDs because enableFlightControl is true here
                     processingThread = std::thread(processingLoopFunction, "python_bridge_LOCAL.py", flightSample, true, ProcessingMode::WALL_FOLLOW);
                 }
                 break;

             case 'e': // Process Full Radar Data (Default Bridge, No Control)
                 std::cout << "Starting Full Radar Data processing using default bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false);
                 // Start thread: uses default bridge, passes nullptr for flightSample, disables control CMDs
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, ProcessingMode::PROCESS_FULL);
                 break;

             case 'f': // Process Minimal Radar Data (Default Bridge, No Control)
                  std::cout << "Starting Minimal Radar Data processing using default bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false);
                 // Start thread: uses default bridge, passes nullptr for flightSample, disables control CMDs
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, ProcessingMode::PROCESS_MINIMAL);
                 break;

             case 'g': // Process Beacon/Wall Only (Default Bridge, No Control)
                  std::cout << "Starting Beacon/Wall Data processing using default bridge: " << defaultPythonBridgeScript << "\n";
                 stopProcessingFlag.store(false);
                 // Start thread: uses default bridge, passes nullptr for flightSample, disables control CMDs
                 processingThread = std::thread(processingLoopFunction, defaultPythonBridgeScript, nullptr, false, ProcessingMode::PROCESS_BEACON_WALL_ONLY);
                 break;

            case 's': // STOP Current Processing Thread
                // The check and join logic is handled at the beginning of the loop now.
                // This case is primarily to explicitly stop without starting something new.
                 std::cout << "Stop command received. Any active processing thread has been signalled to stop.\n";
                 // If the user just pressed 's', the join happened above. We just loop back to the menu.
                 // Add prompt here too, in case the user expected immediate feedback after 's'
                 std::cout << "\n>>> Press Enter to return to the main menu... <<<\n";
                break;

            case 'q': { // Quit
                std::cout << "Exiting...\n";
                 // Stop processing thread handled above

                if (enableFlightControl && vehicle && flightSample) {
                    std::cout << "Releasing Joystick Control Authority...\n";
                    // Use synchronous release or wait after async
                    vehicle->flightController->releaseJoystickCtrlAuthoritySync(functionTimeout);
                    // Or async: vehicle->flightController->releaseJoystickCtrlAuthorityAsync(ReleaseJoystickCtrlAuthorityCB, nullptr);
                    // std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait if async

                    // Consider landing the drone if it's flying
                     std::cout << "Ensure drone is landed before quitting.\n";
                     // flightSample->monitoredLanding(); // Optional safety landing

                    delete flightSample; flightSample = nullptr;
                }
                 if (linuxEnvironment) {
                    delete linuxEnvironment; linuxEnvironment = nullptr; // Clean up LinuxSetup
                 }
                keepRunning = false; // Exit main loop
                break;
            }

            case 0: // User pressed Enter without any other input
                 // Do nothing, loop will redisplay menu
                 break;

            default:
                std::cout << "Invalid input.\n";
                break;
        } // End switch
    } // End while(keepRunning)

    // Final cleanup check for the thread in case the loop exited unexpectedly
     if (processingThread.joinable()) {
         stopProcessingFlag.store(true);
         processingThread.join();
     }

    std::cout << "Program terminated.\n";
    return 0;
}
