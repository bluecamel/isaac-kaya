#include "driver.hpp"
#include <sstream>
#include <algorithm>
#include <cmath>

namespace isaac {
namespace dynamixel {

void Driver::Connect() {
    port_handler_ = GetPortHandler();
    packet_handler_ = GetPacketHandler();

    if (!OpenPort()) {
        throw "Failed to open port.";
    }

    if (!SetBaudRate()) {
        throw "Failed to set baud rate.";
    }
}

void Driver::Disconnect() {
    port_handler_->closePort();
}

dynamixel_sdk::PacketHandler *Driver::GetPacketHandler() {
    if (configuration_.debug == true) {
        std::printf("Creating packet handler for protocol version %f.\n", configuration_.protocol_version);
    }

    return dynamixel_sdk::PacketHandler::getPacketHandler(configuration_.protocol_version);
}

dynamixel_sdk::PortHandler *Driver::GetPortHandler() {
    if (configuration_.debug == true) {
        std::printf("Creating port handler for device %s.\n", configuration_.device_name.c_str());
    }

    return dynamixel_sdk::PortHandler::getPortHandler(configuration_.device_name.c_str());
}

std::vector<ServoSpeed> Driver::GetPresentSpeeds(std::vector<int> &servo_ids) {
    uint8_t error;
    int result;
    uint16_t present_speed;
    float rpm;
    int servo_id;
    std::string servo_id_string;

    std::vector<ServoSpeed> speeds;

    for(std::vector<int>::iterator i = servo_ids.begin(); i != servo_ids.end(); ++i) {
        servo_id = *i;
        servo_id_string = NumberToString(servo_id);

        ServoSpeed speed = { servo_id, 0 };

        if (configuration_.debug == true) {
            std::printf("Getting present speed for servo ID %s.\n", servo_id_string.c_str());
        }

        result = packet_handler_->read2ByteTxRx(port_handler_, servo_id, configuration_.control_table.moving_speed, &present_speed, &error);

        if (result != configuration_.control_values.success) {
            std::printf("Error communicating with servo ID %s: %s (result: %i) while getting present speed.\n", servo_id_string.c_str(), packet_handler_->getTxRxResult(result), result);
            throw error;
        } else if (error != 0) {
            if (configuration_.debug == true) {
                std::printf("Error getting present speed for servo ID %s: %s\n", servo_id_string.c_str(), packet_handler_->getRxPacketError(error));
            }
            throw error;
        } else {
            rpm = SpeedToRpm(present_speed);

            if (configuration_.debug == true) {
                std::printf("Got present speed for servo ID %s: %i (%f rpm).\n", servo_id_string.c_str(), present_speed, rpm);
            }

            speed.speed = present_speed;
        }

        speeds.push_back(speed);
    }

    return speeds;
}

bool Driver::OpenPort() {
    return port_handler_->openPort();
}

int Driver::RpmToSpeed(double rpm) {
    if (rpm == 0) {
        return 0;
    } else if (rpm > 0) { // CW
        int value = (int) std::nearbyint(rpm / configuration_.control_values.moving_speed_rpm_unit) + configuration_.control_values.moving_speed_cw_minimum;
        return clamp(value, configuration_.control_values.moving_speed_cw_minimum, configuration_.control_values.moving_speed_cw_maximum);
    } else { // CCW
        int value = (int) std::nearbyint(-rpm / configuration_.control_values.moving_speed_rpm_unit) + configuration_.control_values.moving_speed_ccw_minimum;
        return clamp(value, configuration_.control_values.moving_speed_ccw_minimum, configuration_.control_values.moving_speed_ccw_maximum);
    }
}

bool Driver::SetBaudRate() {
    return port_handler_->setBaudRate(configuration_.baud_rate);
}

void Driver::SetConfiguration(Configuration configuration) {
    configuration_ = configuration;
}

void Driver::SetMovingSpeeds(std::vector<ServoSpeed> &speeds) {
    uint8_t error;
    int result;
    float rpm;
    int servo_id;
    std::string servo_id_string;
    int speed;

    for(std::vector<ServoSpeed>::iterator i = speeds.begin(); i != speeds.end(); ++i) {
        // TODO: I'm not sure why I have to use 0 index here
        servo_id = i[0].servo_id;
        speed = i[0].speed;
        rpm = SpeedToRpm(speed);
        servo_id_string = NumberToString(servo_id);

        if (configuration_.debug == true) {
            std::printf("Setting moving speed for servo ID %s to %i (%f rpm).\n", servo_id_string.c_str(), speed, rpm);
        }

        result = packet_handler_->write2ByteTxRx(port_handler_, servo_id, configuration_.control_table.moving_speed, speed, &error);

        if (result != configuration_.control_values.success) {
            std::printf("Error communicating with servo ID %s: %s (result: %i) while setting moving speed.\n", servo_id_string.c_str(), packet_handler_->getTxRxResult(result), result);
            throw error;
        } else if (error != 0) {
            std::printf("Error setting moving speed for servo ID %s to %i (%f rpm): %s\n", servo_id_string.c_str(), speed, rpm, packet_handler_->getRxPacketError(error));
            throw error;
        } else {
            if (configuration_.debug == true) {
                std::printf("Set moving speed for servo ID %s to %i (%f rpm).\n", servo_id_string.c_str(), speed, rpm);
            }
        }
    }
}

void Driver::SetTorqueLimit(std::vector<int> &servo_ids, int limit) {
    uint8_t error;
    int result;

    for(std::vector<int>::iterator i = servo_ids.begin(); i != servo_ids.end(); ++i) {
        int servo_id = *i;
        std::string servo_id_string = NumberToString(servo_id);

        if (configuration_.debug == true) {
            std::printf("Setting torque limit to %i.\n", limit);
        }

        result = packet_handler_->write2ByteTxRx(port_handler_, servo_id, configuration_.control_table.torque_limit, limit, &error);

        if (result != configuration_.control_values.success) {
            std::printf("Error communicating with servo ID %s: %s (result: %i) while setting torque limit to %i.\n", servo_id_string.c_str(), packet_handler_->getTxRxResult(result), result, limit);
            throw error;
        } else if (error != 0) {
            std::printf("Error setting torque limit to %i for servo ID %s: %s.\n", limit, servo_id_string.c_str(), packet_handler_->getRxPacketError(error));
            throw error;
        } else {
            if (configuration_.debug == true) {
                std::printf("Set torque limit to %i for servo ID %s.\n", limit, servo_id_string.c_str());
            }
        }
    }
}

double Driver::SpeedToRpm(int speed) {
    if (speed > 0 && speed <= configuration_.control_values.moving_speed_cw_maximum) { // CW
        return (double) speed * configuration_.control_values.moving_speed_rpm_unit;
    } else if (speed >= configuration_.control_values.moving_speed_ccw_minimum && speed <= configuration_.control_values.moving_speed_ccw_maximum) { // CCW
        return - (double) (speed - configuration_.control_values.moving_speed_ccw_minimum) * configuration_.control_values.moving_speed_rpm_unit;
    } else {
        return 0.0;
    }
}

void Driver::ToggleTorque(std::vector<int> &servo_ids, bool enabled) {
    uint8_t error;
    int result;
    int torque_enable_value = enabled ? configuration_.control_values.torque_enable : configuration_.control_values.torque_disable;

    for(std::vector<int>::iterator i = servo_ids.begin(); i != servo_ids.end(); ++i) {
        int servo_id = *i;
        std::string servo_id_string = NumberToString(servo_id);

        if (configuration_.debug == true) {
            std::printf("%s torque for servo ID %s (torque enable address: %i, torque enable value: %i).\n", enabled ? "Enabling" : "Disabling", servo_id_string.c_str(), configuration_.control_table.torque_enable, torque_enable_value);
        }

        result = packet_handler_->write1ByteTxRx(port_handler_, servo_id, configuration_.control_table.torque_enable, torque_enable_value, &error);

        if (result != configuration_.control_values.success) {
            std::printf("Error communicating with servo ID %s: %s (result: %i) while %s torque.\n", servo_id_string.c_str(), packet_handler_->getTxRxResult(result), result, enabled ? "enabling" : "disabling");
            throw error;
        } else if (error != 0) {
            std::printf("Error %s torque for servo ID %s: %s.\n", enabled ? "enabling" : "disabling", servo_id_string.c_str(), packet_handler_->getRxPacketError(error));
            throw error;
        } else {
            if (configuration_.debug == true) {
                std::printf("%s torque for servo ID %s.\n", enabled ? "Enabled" : "Disabled", servo_id_string.c_str());
            }
        }
    }
}

} // namespace dynamixel
} // namespace isaac
