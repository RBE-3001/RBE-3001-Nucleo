
/**
 * @file PidServer.h
 * @brief PidServer for the RBE3001 robotic arm
 *
 * @section RBE3001 - Nucleo Firmware - PidServer
 *
 * Instructions
 * ------------
 * This class implements a communication server that can be
 * used to send given setpoints (in joint space) to the robotic arm.
 * Setpoints generated in MATLAB and sent over HDI will be made available
 * to the `event()' function below. See the code in `PidServer.cpp' for
 * for more details.
 *
 * IMPORTANT - Multiple communication servers can run in parallel, as shown
 *             in the main file of this firmware
 *             (see 'Part 2b' in /src/Main.cpp). To ensure that communication
 *             packets generated in MATLAB are routServoed to the appropriate
 *             server, we use unique identifiers. The identifier for this
 *             server is the integer number 37.
 *             In general, the identifier can be any 4-byte unsigned
 *             integer number.
 */

#ifndef LAB_SERVER
#define LAB_SERVER

#include <PID_Bowler.h>
#include <PacketEvent.h>
#include "../drivers/MyPid.h"
#include <cmath>              // needed for std::abs

#define LAB_SERVER_ID 30      // identifier for this server

/**
 *  @brief Class that receives setpoints through HID and sends them to
 *         the PID controller. Extends the `PacketEventAbstract' class.
 */
class LabServer: public PacketEventAbstract
{
 private:
  PIDimp ** myPidObjects;    // array of PidServers - one for each joint
  int myNumberOfPidChannels;
  Servo gripperServo;
  float MOTORHIGH_TORQUE = 3168.63;
  float MOTORHIGH_VOLTAGE = 1;
  float MOTORLOW_TORQUE = 2472.12;
  float MOTORLOW_VOLTAGE = 0.714;

  float GRAVITYCOMP_SCALINGFACTOR = 178.5;
  float GRAVITYCOMP_JOINT1 = -0.53193*GRAVITYCOMP_SCALINGFACTOR;
  float GRAVITYCOMP_JOINT2 = -0.47511*GRAVITYCOMP_SCALINGFACTOR;

 public:
  LabServer (PIDimp ** pidObjects, int numberOfPidChannels, PinName gripperPin)
 	 : PacketEventAbstract(LAB_SERVER_ID), gripperServo(gripperPin, 5)
  {
    myPidObjects = pidObjects;
    myNumberOfPidChannels = numberOfPidChannels;
  }

  // This method is called every time a packet from MATLAB is received
  // via HID
  void event(float * buffer);

  float map(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

};

#endif /* end of include guard: RBE3001_PID_SERVER */
