#pragma once

#include "DriverConstants.hpp"
#include "DriverTypes.hpp"
#include "external/robotis/c++/include/dynamixel_sdk/dynamixel_sdk.h"
#include <array>
#include <vector>

namespace isaac {
namespace Dynamixel {

class Driver {
    private:
        dynamixel::PacketHandler *packetHandler;
        dynamixel::PortHandler *portHandler;

    public:
        Configuration configuration;

        void connect();
        void disconnect();
        dynamixel::PacketHandler *getPacketHandler();
        dynamixel::PortHandler *getPortHandler();
        std::vector<ServoSpeed> getPresentSpeeds(std::vector<int> &servoIds);
        bool openPort();
        int rpmToSpeed(double rpm);
        bool setBaudRate();
        void setConfiguration(Configuration _configuration);
        void setMovingSpeeds(std::vector<ServoSpeed> &speeds);
        void setTorqueLimit(std::vector<int> &servoIds, int limit);
        double speedToRpm(int speed);
        void toggleTorque(std::vector<int> &servoIds, bool enabled);
};

} // namespace Dynamixel
} // namespace isaac