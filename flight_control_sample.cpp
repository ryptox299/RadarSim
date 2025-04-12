#include "flight_control_sample.hpp"
#include "flight_sample.hpp"
#include "dji_linux_helpers.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void ObtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData)
{
  if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ObtainJoystickCtrlAuthoritySuccess)
  {
    DSTATUS("ObtainJoystickCtrlAuthoritySuccess");
  }
}

void ReleaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData)
{
  if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ReleaseJoystickCtrlAuthoritySuccess)
  {
    DSTATUS("ReleaseJoystickCtrlAuthoritySuccess");
  }
}

int main(int argc, char** argv) {
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  // ErrorCode::ErrorCodeType ret = vehicle->flightController->obtainJoystickCtrlAuthoritySync(functionTimeout);
  vehicle->flightController->obtainJoystickCtrlAuthorityAsync(ObtainJoystickCtrlAuthorityCB, nullptr ,functionTimeout, 2);
  FlightSample* flightSample = new FlightSample(vehicle);
  // Display interactive prompt
  std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Monitored Takeoff + Landing MOLELLELELE                               |"
      << std::endl;
  std::cout
      << "| [b] Monitored Takeoff + Position Control + Landing             |"
      << std::endl;
  std::cout << "| [c] Monitored Takeoff + Position Control + Force Landing "
               "Avoid Ground  |"
            << std::endl;
  std::cout << "| [d] Monitored Takeoff + Velocity Control + Landing |"
            << std::endl;

  char inputChar;
  std::cin >> inputChar;

  switch (inputChar) {
    case 'a':
    {
      flightSample->monitoredTakeoff();
      flightSample->monitoredLanding();
      break;
    }
    case 'b':
    {
      flightSample->monitoredTakeoff();

      DSTATUS("Take off over!\n");
      flightSample->moveByPositionOffset((FlightSample::Vector3f){0, 6, 6}, 30, 0.8, 1);
      DSTATUS("Step 1 over!\n");
      flightSample->moveByPositionOffset((FlightSample::Vector3f){6, 0, -3}, -30, 0.8, 1);
      DSTATUS("Step 2 over!\n");
      flightSample->moveByPositionOffset((FlightSample::Vector3f){-6, -6, 0}, 0, 0.8, 1);
      DSTATUS("Step 3 over!\n");
      flightSample->monitoredLanding();
      break;
    }

    /*! @NOTE: case 'c' only support for m210 V2 and M300*/
    case 'c':
    {
      /*!  Take off */
      flightSample->monitoredTakeoff();
      vehicle->flightController->setCollisionAvoidanceEnabledSync(
          FlightController::AvoidEnable::AVOID_ENABLE, 1);

      /*! Move to higher altitude */
      flightSample->moveByPositionOffset((FlightSample::Vector3f){0, 0, 30}, 0, 0.8, 1);

      /*! Move a short distance*/
      flightSample->moveByPositionOffset((FlightSample::Vector3f){10, 0, 0}, 0, 0.8, 1);

      /*! Set aircraft current position as new home location */
      flightSample->setNewHomeLocation();

      /*! Set new go home altitude */
      flightSample->setGoHomeAltitude(50);

      /*! Move to another position */
      flightSample->moveByPositionOffset((FlightSample::Vector3f){20, 0, 0}, 0, 0.8, 1);

      vehicle->flightController->setCollisionAvoidanceEnabledSync(
        FlightController::AvoidEnable::AVOID_DISABLE, 1);
      /*! go home and confirm landing */
      flightSample->goHomeAndConfirmLanding();

      vehicle->flightController->setCollisionAvoidanceEnabledSync(
        FlightController::AvoidEnable::AVOID_ENABLE, 1);
      break;
    }

    case 'd':
    {
      flightSample->monitoredTakeoff();
      DSTATUS("Take off over!\n");

      flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){0, 0, 5.0}, 0, 2000);
      DSTATUS("Step 1 over!EmergencyBrake for 2s\n");
      flightSample->emergencyBrake();
      sleep(2);
      flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){-1.5, 2, 0}, 0, 2000);
      DSTATUS("Step 2 over!EmergencyBrakefor 2s\n");
      flightSample->emergencyBrake();
      sleep(2);
      flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){3, 0, 0}, 0, 2500);
      DSTATUS("Step 3 over!EmergencyBrake for 2s\n");
      flightSample->emergencyBrake();
      sleep(2);
      flightSample->velocityAndYawRateCtrl((FlightSample::Vector3f){-1.6, -2, 0}, 0, 2200);
      DSTATUS("Step 4 over!EmergencyBrake for 2s\n");
      flightSample->emergencyBrake();
      sleep(2);

      flightSample->monitoredLanding();
    }

    default:
      break;
  }

  // ret = vehicle->flightController->releaseJoystickCtrlAuthoritySync(functionTimeout);
  vehicle->flightController->releaseJoystickCtrlAuthorityAsync(ReleaseJoystickCtrlAuthorityCB, nullptr ,functionTimeout, 2);
  delete flightSample;
  return 0;
}
