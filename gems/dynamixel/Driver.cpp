#include "Driver.hpp"
#include <sstream>
#include <algorithm>
#include <cmath>

namespace isaac {
namespace Dynamixel {

void Driver::connect() {
    portHandler = getPortHandler();
    packetHandler = getPacketHandler();

    if (!openPort()) {
        throw "Failed to open port.";
    }

    if (!setBaudRate()) {
        throw "Failed to set baud rate.";
    }
}

void Driver::disconnect() {
    portHandler->closePort();
}

dynamixel::PacketHandler *Driver::getPacketHandler() {
    if (configuration.debug == true) {
        std::printf("Creating packet handler for protocol version %f.\n", configuration.protocol_version);
    }

    return dynamixel::PacketHandler::getPacketHandler(configuration.protocol_version);
}

dynamixel::PortHandler *Driver::getPortHandler() {
    if (configuration.debug == true) {
        std::printf("Creating port handler for device %s.\n", configuration.device_name.c_str());
    }

    return dynamixel::PortHandler::getPortHandler(configuration.device_name.c_str());
}

std::vector<ServoSpeed> Driver::getPresentSpeeds(std::vector<int> &servoIds) {
    uint8_t error;
    int result;
    int servo_id; // TODO: eh
    uint16_t present_speed; // TODO: eh
    float rpm; // TODO: switch to ServoRpm?
    std::string servo_id_string; // TODO: eh

    std::vector<ServoSpeed> speeds;

    for(std::vector<int>::iterator i = servoIds.begin(); i != servoIds.end(); ++i) {
        servo_id = *i;
        servo_id_string = NumberToString(servo_id);

        ServoSpeed speed = { servo_id, 0 };

        if (configuration.debug == true) {
            std::printf("Getting present speed for servo ID %s.\n", servo_id_string.c_str());
        }

        result = packetHandler->read2ByteTxRx(portHandler, servo_id, configuration.control_table.moving_speed, &present_speed, &error);

        if (result != configuration.control_values.success) {
            std::printf("Error communicating with servo ID %s: %s (result: %i) while getting present speed.\n", servo_id_string.c_str(), packetHandler->getTxRxResult(result), result);
            // TODO: throw? retry? stop?
        } else if (error != 0) {
            if (configuration.debug == true) {
                std::printf("Error getting present speed for servo ID %s: %s\n", servo_id_string.c_str(), packetHandler->getRxPacketError(error));
            }
            // TODO: throw? retry? stop?
        } else {
            rpm = speedToRpm(present_speed);

            if (configuration.debug == true) {
                std::printf("Got present speed for servo ID %s: %i (%f rpm).\n", servo_id_string.c_str(), present_speed, rpm);
            }

            speed.speed = present_speed;
        }

        speeds.push_back(speed);
    }

    return speeds;
}

bool Driver::openPort() {
    return portHandler->openPort();
}

int Driver::rpmToSpeed(double rpm) {
    if (rpm == 0) {
        return 0;
    } else if (rpm > 0) { // CW
        int value = (int) std::nearbyint(rpm / configuration.control_values.moving_speed_rpm_unit) + configuration.control_values.moving_speed_cw_minimum;
        return clamp(value, configuration.control_values.moving_speed_cw_minimum, configuration.control_values.moving_speed_cw_maximum);
    } else { // CCW
        int value = (int) std::nearbyint(-rpm / configuration.control_values.moving_speed_rpm_unit) + configuration.control_values.moving_speed_ccw_minimum;
        return clamp(value, configuration.control_values.moving_speed_ccw_minimum, configuration.control_values.moving_speed_ccw_maximum);
    }
}

bool Driver::setBaudRate() {
    return portHandler->setBaudRate(configuration.baud_rate);
}

void Driver::setConfiguration(Configuration _configuration) {
    configuration = _configuration;
}

void Driver::setMovingSpeeds(std::vector<ServoSpeed> &speeds) {
    uint8_t error;
    int result;
    int servo_id; // TODO: eh
    int speed; // TODO: eh
    float rpm; // TODO: change to ServoRpm?
    std::string servo_id_string; // TODO: eh

    for(std::vector<ServoSpeed>::iterator i = speeds.begin(); i != speeds.end(); ++i) {
        // TODO: I'm not sure why I have to use 0 index here
        servo_id = i[0].servo_id;
        speed = i[0].speed;
        rpm = speedToRpm(speed);
        servo_id_string = NumberToString(servo_id);

        if (configuration.debug == true) {
            std::printf("Setting moving speed for servo ID %s to %i (%f rpm).\n", servo_id_string.c_str(), speed, rpm);
        }

        result = packetHandler->write2ByteTxRx(portHandler, servo_id, configuration.control_table.moving_speed, speed, &error);

        if (result != configuration.control_values.success) {
            std::printf("Error communicating with servo ID %s: %s (result: %i) while setting moving speed.\n", servo_id_string.c_str(), packetHandler->getTxRxResult(result), result);
            // TODO: throw? retry? stop?
        } else if (error != 0) {
            std::printf("Error setting moving speed for servo ID %s to %i (%f rpm): %s\n", servo_id_string.c_str(), speed, rpm, packetHandler->getRxPacketError(error));
            // TODO: throw? retry? stop?
        } else {
            if (configuration.debug == true) {
                std::printf("Set moving speed for servo ID %s to %i (%f rpm).\n", servo_id_string.c_str(), speed, rpm);
            }
        }
    }
}

void Driver::setTorqueLimit(std::vector<int> &servoIds, int limit) {
    uint8_t error;
    int result;

    for(std::vector<int>::iterator i = servoIds.begin(); i != servoIds.end(); ++i) {
        int servo_id = *i;
        std::string servo_id_string = NumberToString(servo_id);

        if (configuration.debug == true) {
            std::printf("Setting torque limit to %i.\n", limit);
        }

        result = packetHandler->write2ByteTxRx(portHandler, servo_id, configuration.control_table.torque_limit, limit, &error);

        if (result != configuration.control_values.success) {
            std::printf("Error communicating with servo ID %s: %s (result: %i) while setting torque limit to %i.\n", servo_id_string.c_str(), packetHandler->getTxRxResult(result), result, limit);
            // TODO: throw? stop?
        } else if (error != 0) {
            std::printf("Error setting torque limit to %i for servo ID %s: %s.\n", limit, servo_id_string.c_str(), packetHandler->getRxPacketError(error));
            // TODO: throw? stop?
        } else {
            if (configuration.debug == true) {
                std::printf("Set torque limit to %i for servo ID %s.\n", limit, servo_id_string.c_str()); // TODO: name?
            }
        }
    }
}

double Driver::speedToRpm(int speed) {
    if (speed > 0 && speed <= configuration.control_values.moving_speed_cw_maximum) { // CW
        return (double) speed * configuration.control_values.moving_speed_rpm_unit;
    } else if (speed >= configuration.control_values.moving_speed_ccw_minimum && speed <= configuration.control_values.moving_speed_ccw_maximum) { // CCW
        return - (double) (speed - configuration.control_values.moving_speed_ccw_minimum) * configuration.control_values.moving_speed_rpm_unit;
    } else {
        return 0.0;
    }
}

void Driver::toggleTorque(std::vector<int> &servoIds, bool enabled) {
    uint8_t error;
    int result;
    int torque_enable_value = enabled ? 1 : 0; // TODO: value struct?

    for(std::vector<int>::iterator i = servoIds.begin(); i != servoIds.end(); ++i) {
        int servo_id = *i;
        std::string servo_id_string = NumberToString(servo_id);

        if (configuration.debug == true) {
            std::printf("%s torque for servo ID %s (torque enable address: %i, torque enable value: %i).\n", enabled ? "Enabling" : "Disabling", servo_id_string.c_str(), configuration.control_table.torque_enable, torque_enable_value);
        }

        result = packetHandler->write1ByteTxRx(portHandler, servo_id, configuration.control_table.torque_enable, torque_enable_value, &error);

        if (result != configuration.control_values.success) {
            std::printf("Error communicating with servo ID %s: %s (result: %i) while %s torque.\n", servo_id_string.c_str(), packetHandler->getTxRxResult(result), result, enabled ? "enabling" : "disabling");
            // TODO: throw? stop?
        } else if (error != 0) {
            std::printf("Error %s torque for servo ID %s: %s.\n", enabled ? "enabling" : "disabling", servo_id_string.c_str(), packetHandler->getRxPacketError(error));
            // TODO: throw? stop?
        } else {
            if (configuration.debug == true) {
                std::printf("%s torque for servo ID %s.\n", enabled ? "Enabled" : "Disabled", servo_id_string.c_str()); // TODO: name?
            }
        }
    }
}

} // namespace Dynamixel
} // namespace isaac
