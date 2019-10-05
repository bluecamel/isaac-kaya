#pragma once

#include "driver_constants.hpp"
#include "driver_types.hpp"
#include "util.hpp"
#include "external/robotis/c++/include/dynamixel_sdk/dynamixel_sdk.h"
#include <array>
#include <vector>

namespace dynamixel_sdk = dynamixel;

namespace isaac {
namespace dynamixel {

class Driver {
    private:
        dynamixel_sdk::PacketHandler *packet_handler_;
        dynamixel_sdk::PortHandler *port_handler_;

    public:
        Configuration configuration_;

        void Connect();
        void Disconnect();
        dynamixel_sdk::PacketHandler *GetPacketHandler();
        dynamixel_sdk::PortHandler *GetPortHandler();
        std::vector<ServoSpeed> GetPresentSpeeds(std::vector<int> &servo_ids);
        bool OpenPort();
        int RpmToSpeed(double rpm);
        bool SetBaudRate();
        void SetConfiguration(Configuration configuration);
        void SetMovingSpeeds(std::vector<ServoSpeed> &speeds);
        void SetTorqueLimit(std::vector<int> &servo_ids, int limit);
        double SpeedToRpm(int speed);
        void ToggleTorque(std::vector<int> &servo_ids, bool enabled);
};

} // namespace dynamixel
} // namespace isaac