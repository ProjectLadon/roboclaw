/**
 *
 * Copyright (c) 2018 Carroll Vance.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "roboclaw_driver.h"

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

namespace roboclaw {

    const uint8_t driver::BASE_ADDRESS          = 0x80;
    const uint8_t driver::MAX_QUEUE_DEPTH       = 50;
    const uint32_t driver::DEFAULT_BAUDRATE     = 38400;
    const float driver::AMPS_SCALE              = 100.0f;
    const float driver::VOLTS_SCALE             = 10.0f;
    const uint32_t driver::DEFAULT_TIMEOUT_MS   = 200;
    const int32_t driver::PID_CONST_MULT        = 0x10000;

    driver::driver(
            const std::string port, 
            const uint32_t baudrate, 
            const uint32_t timeout_ms,
            rclcpp::Node *node)
    {
        serial = std::shared_ptr<TimeoutSerial>(new TimeoutSerial(port, baudrate));
        serial->setTimeout(boost::posix_time::milliseconds(timeout_ms));
        log_node = node;
        run_enable = true;
        worker_thread = std::unique_ptr<std::thread>(new std::thread(&driver::worker, this));
    }

    driver::~driver()
    {
        run_enable = false;
        worker_thread->join();
    }
        
    void driver::set_timeout_ms(const uint32_t to)
    {
        serial->setTimeout(boost::posix_time::milliseconds(to));
    }

    void driver::crc16_reset() 
    {
        crc = 0;
    }

    uint16_t driver::crc16(uint8_t *packet, size_t nBytes) 
    {

        for (size_t byte = 0; byte < nBytes; byte++) {

            crc = crc ^ ((uint16_t) packet[byte] << 8);

            for (uint8_t bit = 0; bit < 8; bit++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc = crc << 1;
            }
        }

        return crc;
    }

    size_t driver::txrx(uint8_t address,
                        uint8_t command,
                        uint8_t *tx_data,
                        size_t tx_length,
                        uint8_t *rx_data,
                        size_t rx_length,
                        bool tx_crc, bool rx_crc) 
    {

        boost::mutex::scoped_lock lock(serial_mutex);

        std::vector<uint8_t> packet;

        if (tx_crc)
            packet.resize(tx_length + 4);
        else
            packet.resize(tx_length + 2);

        // Header
        packet[0] = address;
        packet[1] = command;

        crc16_reset();
        crc16(&packet[0], 2);

        // Data
        if (tx_length > 0 && tx_data != nullptr)
            memcpy(&packet[2], tx_data, tx_length);

        // CRC
        if (tx_crc) {
            unsigned int crc = crc16(&packet[2], tx_length);

            // RoboClaw expects big endian / MSB first
            packet[tx_length + 2] = (uint8_t) ((crc >> 8) & 0xFF);
            packet[tx_length + 2 + 1] = (uint8_t) (crc & 0xFF);

        }

        serial->write((char*)&packet[0], packet.size());
        // RCLCPP_INFO(log_node->get_logger(), "Serial write complete");

        size_t want_bytes;
        if (rx_crc)
            want_bytes = rx_length + 2;
        else
            want_bytes = rx_length;

        std::vector<char> response_vector;

        response_vector = serial->read(want_bytes);
        // RCLCPP_INFO(log_node->get_logger(), "Serial read complete");

        size_t bytes_received = response_vector.size();

        uint8_t* response = (uint8_t*) &response_vector[0];

        if (bytes_received != want_bytes)
        {
            RCLCPP_ERROR(log_node->get_logger(),"Timeout reading from RoboClaw. Wanted %ld bytes got %ld", want_bytes, bytes_received);
            throw timeout_exception("Timeout reading from RoboClaw.");
        }

        // Check CRC
        if (rx_crc) {
            unsigned int crc_calculated = crc16(&response[0], bytes_received - 2);
            unsigned int crc_received = 0;

            // RoboClaw generates big endian / MSB first
            crc_received += response[bytes_received - 2] << 8;
            crc_received += response[bytes_received - 1];

            if (crc_calculated != crc_received) {
                RCLCPP_ERROR(log_node->get_logger(),"Expected CRC 0x%x, received 0x%x", crc_calculated, crc_received);
                RCLCPP_INFO(log_node->get_logger(), "Received string length: %ld", bytes_received);

                throw roboclaw::crc_exception("Roboclaw CRC mismatch");
            }

            memcpy(rx_data, &response[0], bytes_received - 2);
        } else {
            memcpy(rx_data, &response[0], bytes_received);
        }

        if (!rx_crc)
            return bytes_received;
        else
            return bytes_received - 2;
    }

    void driver::read_current_limit(uint8_t address, uint8_t channel)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        (channel == 1) ? current_limit_ready[address].first = false : current_limit_ready[address].second = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_current_limit, this, address, channel)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_current_limit");
    }
    void driver::exec_read_current_limit(uint8_t address, uint8_t channel)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[8];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_current_limit on channel %d", channel);
        uint8_t command = (channel == 1) ? (uint8_t)StatusCmds::ReadMaxCurrM1 : (uint8_t)StatusCmds::ReadMaxCurrM2;

        txrx(address, command, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);
        float output;
        output = (float)decode_int32(&rx_buffer[0])/driver::AMPS_SCALE;
        (channel == 1) ? current_limit_ready[address].first = true : current_limit_ready[address].second = true;
        (channel == 1) ? current_limit[address].first = output : current_limit[address].second = output;
    }

    void driver::read_velocity_pid(uint8_t address, uint8_t channel)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        (channel == 1) ? velocity_pid_ready[address].first = false : velocity_pid_ready[address].second = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_velocity_pid, this, address, channel)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_velocity_pid");
    }
    void driver::exec_read_velocity_pid(uint8_t address, uint8_t channel)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[16];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_velocity_pid on channel %d", channel);
        uint8_t command = (channel == 1) ? (uint8_t)AdvMotorControlCmds::ReadVelPIDM1 : (uint8_t)AdvMotorControlCmds::ReadVelPIDM2;

        txrx(address, command, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);
        velocity_pid_t output;
        output.p = (float)decode_int32(&rx_buffer[0])/driver::PID_CONST_MULT;
        output.i = (float)decode_int32(&rx_buffer[4])/driver::PID_CONST_MULT;
        output.d = (float)decode_int32(&rx_buffer[8])/driver::PID_CONST_MULT;
        output.qpps = decode_uint32(&rx_buffer[12]);
        (channel == 1) ? velocity_pid_ready[address].first = true : velocity_pid_ready[address].second = true;
        (channel == 1) ? velocity_pid_data[address].first = output : velocity_pid_data[address].second = output;
    }

    void driver::read_position_pid(uint8_t address, uint8_t channel)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        (channel == 1) ? position_pid_ready[address].first = false : position_pid_ready[address].second = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_position_pid, this, address, channel)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_position_pid");
    }
    void driver::exec_read_position_pid(uint8_t address, uint8_t channel)
    {
        
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[28];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_position_pid on channel %d", channel);
        uint8_t command = (channel == 1) ? (uint8_t)AdvMotorControlCmds::ReadPosPIDM1 : (uint8_t)AdvMotorControlCmds::ReadPosPIDM2;

        txrx(address, command, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);
        position_pid_t output;
        output.p        = (float)decode_int32(&rx_buffer[0])/driver::PID_CONST_MULT;
        output.i        = (float)decode_int32(&rx_buffer[4])/driver::PID_CONST_MULT;
        output.d        = (float)decode_int32(&rx_buffer[8])/driver::PID_CONST_MULT;
        output.max_i    = (float)decode_int32(&rx_buffer[12])/driver::PID_CONST_MULT;
        output.deadzone = decode_uint32(&rx_buffer[16]);
        output.min_pos  = decode_int32(&rx_buffer[20]);
        output.max_pos  = decode_int32(&rx_buffer[24]);
        (channel == 1) ? position_pid_ready[address].first = true : position_pid_ready[address].second = true;
        (channel == 1) ? position_pid_data[address].first = output : position_pid_data[address].second = output;
    }
    
    // Write current settings to EEPROM
    void driver::write_eeprom(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_write_eeprom, this, address)));
    }
    void driver::exec_write_eeprom(uint8_t address)
    {
        uint8_t rx_buffer[1];
        txrx(address, (uint8_t)StatusCmds::WriteSettingsEEPROM, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, false);
    }

    // Restores settings to EEPROM values
    void driver::read_eeprom(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_eeprom, this, address)));
    }
    void driver::exec_read_eeprom(uint8_t address)
    {
        uint8_t rx_buffer[4];
        txrx(address, (uint8_t)StatusCmds::WriteSettingsEEPROM, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);
    }

    void driver::read_version(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        versions_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_version, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_version");
    }
    void driver::exec_read_version(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[48];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_version");

        txrx(address, (uint8_t)StatusCmds::ReadFirmwareVersion, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        std::string version = std::string(reinterpret_cast< char const * >(rx_buffer));
        trim(version);
        versions[address] = version;
        versions_ready[address] = true;
    }

    void driver::read_status(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        status_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_status, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_status");
    }
    void driver::exec_read_status(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[4];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_status");

        txrx(address, (uint8_t)StatusCmds::ReadStatus, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e1 = decode_uint32(rx_buffer);

        status[address] = e1;
        status_ready[address] = true;
    }

    void driver::read_motor_pwm(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        motor_pwm_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_motor_pwm, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_motor_pwm");
    }
    void driver::exec_read_motor_pwm(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[4];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_motor_pwm");

        txrx(address, (uint8_t)StatusCmds::ReadMotorPWMs, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        int16_t e1 = decode_int16(&rx_buffer[0]);
        int16_t e2 = decode_int16(&rx_buffer[2]);

        motor_pwm[address] = std::pair<int16_t, int16_t>(e1, e2);
        motor_pwm_ready[address] = true;
    }

    void driver::read_encoders(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        encoders_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_encoders, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_encoders");
    }
    void driver::exec_read_encoders(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[8];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_encoders");

        txrx(address, (uint8_t)EncoderCmds::ReadEncoderCnts, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        int32_t e1 = decode_int32(&rx_buffer[0]);
        int32_t e2 = decode_int32(&rx_buffer[4]);

        encoders[address] = std::pair<int, int>((int)e1, (int)e2);
        encoders_ready[address] = true;
    }

    void driver::read_velocity(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        velocities_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_velocity, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_velocity for node %d", address - driver::BASE_ADDRESS);
    }
    void driver::exec_read_velocity(uint8_t address)
    {
        // TODO: Fix this function so it handles velocity signs correctly.
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[8];

        txrx(address, (uint8_t)EncoderCmds::ReadAvgSpeeds, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);
        int32_t e1 = decode_int32(&rx_buffer[0]);
        int32_t e2 = decode_int32(&rx_buffer[4]);

        velocities[address] = std::pair<int, int>((int)e1, (int)e2);
        velocities_ready[address] = true;
    }

    void driver::read_position_errors(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        posn_err_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_position_errors, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_position_errors");
    }
    void driver::exec_read_position_errors(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[8];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_position_errors");

        txrx(address, (uint8_t)EncoderCmds::ReadPosErr, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        int32_t e1 = decode_int32(&rx_buffer[0]);
        int32_t e2 = decode_int32(&rx_buffer[4]);

        posn_errors[address] = std::pair<int, int>((int)e1, (int)e2);
        posn_err_ready[address] = true;
    }

    void driver::read_motor_currents(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        motor_currents_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_motor_currents, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_motor_currents");
    }
    void driver::exec_read_motor_currents(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[4];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_motor_currents");

        txrx(address, (uint8_t)StatusCmds::ReadMotorCurrents, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        int16_t e1 = decode_int16(&rx_buffer[0]);
        int16_t e2 = decode_int16(&rx_buffer[2]);
        
        motor_currents[address] = std::pair<float, float>(((float)e1)/AMPS_SCALE, ((float)e2)/AMPS_SCALE);
        motor_currents_ready[address] = true;
    }

    void driver::read_logic_voltage(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        logic_voltages_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_logic_voltage, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_logic_voltages");
    }
    void driver::exec_read_logic_voltage(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[2];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_logic_voltages");

        txrx(address, (uint8_t)StatusCmds::ReadLogicVoltage, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint16_t e1 = decode_uint16(&rx_buffer[0]);

        logic_voltages[address] = (float)e1/VOLTS_SCALE;
        logic_voltages_ready[address] = true;
    }

    void driver::read_motor_voltage(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        motor_voltages_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_motor_voltage, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_motor_voltages");
    }
    void driver::exec_read_motor_voltage(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[2];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_motor_voltages");

        txrx(address, (uint8_t)StatusCmds::ReadMainVoltage, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint16_t e1 = decode_uint16(&rx_buffer[0]);

        motor_voltages[address] = (float)e1/VOLTS_SCALE;
        motor_voltages_ready[address] = true;
    }

    void driver::read_volt_current(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        motor_currents_ready[address] = false;
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_read_volt_current, this, address)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_volt_current");
    }
    void driver::exec_read_volt_current(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[4];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_volt_current");

        // read motor currents
        txrx(address, (uint8_t)StatusCmds::ReadMotorCurrents, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        int16_t e1 = decode_int16(&rx_buffer[0]);
        int16_t e2 = decode_int16(&rx_buffer[2]);
        
        motor_currents[address] = std::pair<float, float>(((float)e1)/AMPS_SCALE, ((float)e2)/AMPS_SCALE);
        motor_currents_ready[address] = true;

        // read logic voltage
        txrx(address, (uint8_t)StatusCmds::ReadLogicVoltage, nullptr, 0, rx_buffer, 2, false, true);

        uint16_t v1 = decode_uint16(&rx_buffer[0]);

        logic_voltages[address] = (float)v1/VOLTS_SCALE;
        logic_voltages_ready[address] = true;

        // read motor voltage
        txrx(address, (uint8_t)StatusCmds::ReadMainVoltage, nullptr, 0, rx_buffer, 2, false, true);

        v1 = decode_uint16(&rx_buffer[0]);

        motor_voltages[address] = (float)v1/VOLTS_SCALE;
        motor_voltages_ready[address] = true;
    }

    void driver::reset_encoder(uint8_t address, uint8_t channel, int32_t value) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_reset_encoder, this, address, channel, value)));
        
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing reset_encoders");
    }
    void driver::exec_reset_encoder(uint8_t address, uint8_t channel, int32_t value) 
    {
        uint8_t rx_buffer[1];
        // RCLCPP_INFO(log_node->get_logger(), "Executing reset_encoders");
        uint8_t command = (channel == 1) ? (uint8_t)EncoderCmds::ResetM1 : (uint8_t)EncoderCmds::ResetM2;

        uint8_t tx_buffer[4];
        encode_int32(value, tx_buffer);
        
        txrx(address, command, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_current_limit(uint8_t address, uint8_t channel, float limit)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_current_limit, this, address, channel, limit)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing set_current_limit");
    }
    void driver::exec_set_current_limit(uint8_t address, uint8_t channel, float limit)
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[8];
        // RCLCPP_INFO(log_node->get_logger(), "Executing set_current_limit on channel %d", channel);
        uint8_t command = (channel == 1) ? (uint8_t)StatusCmds::SetMaxCurrM1 : (uint8_t)StatusCmds::SetMaxCurrM2;
        memset(&tx_buffer[0], 0, sizeof(tx_buffer));
        encode_int32((int32_t)(limit * driver::AMPS_SCALE), &tx_buffer[0]);
        txrx(address, command, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_velocity(uint8_t address, std::pair<int32_t, int32_t> speed) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_velocity, this, address, speed)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing set_velocity");
    }
    void driver::exec_set_velocity(uint8_t address, std::pair<int32_t, int32_t> speed)
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[8];
        // RCLCPP_INFO(log_node->get_logger(), "Executing set_velocity");

        // RoboClaw expects big endian / MSB first
        encode_int32(speed.first, &tx_buffer[0]);
        encode_int32(speed.second, &tx_buffer[4]);

        txrx(address, (uint8_t)AdvMotorControlCmds::SetSpdM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_velocity_single(uint8_t address, uint8_t channel, int32_t speed) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_velocity_single, this, address, channel, speed)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueing set_velocity_single; address: %d, channel: %d, speed: %d", address, channel, speed);
    }
    void driver::exec_set_velocity_single(uint8_t address, uint8_t channel, int32_t speed) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[4];
        // RCLCPP_INFO(log_node->get_logger(), "Executing set_velocity_single; address: %d, channel: %d, speed: %d", address, channel, speed);
        uint8_t command = (channel == 1) ? (uint8_t)AdvMotorControlCmds::SetSpdM1 : (uint8_t)AdvMotorControlCmds::SetSpdM2;

        // RoboClaw expects big endian / MSB first
        encode_int32(speed, &tx_buffer[0]);

        txrx(address, command, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_position(uint8_t address, std::pair<int32_t, int32_t> position) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_position, this, address, position)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueing set_position");
    }
    void driver::exec_set_position(uint8_t address, std::pair<int32_t, int32_t> position) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[9];
        // RCLCPP_INFO(log_node->get_logger(), "Executing set_position");
        encode_int32(position.first, &tx_buffer[0]);
        encode_int32(position.second, &tx_buffer[4]);
        tx_buffer[8] = 1;   // Buffer argument -- implements this cmd immediately

        txrx(address, (uint8_t)AdvMotorControlCmds::SetPosM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }
        
    void driver::set_position_single(uint8_t address, uint8_t channel, int position)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_position_single, this, address, channel, position)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueing set_position_single address: %d channel: %d position: %d", address, channel, position);
    }
    void driver::exec_set_position_single(uint8_t address, uint8_t channel, int32_t position)
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[5];
        // RCLCPP_INFO(log_node->get_logger(), "Executing set_position_single");
    
        uint8_t command = (channel == 1) ? (uint8_t)AdvMotorControlCmds::SetPosM1 : (uint8_t)AdvMotorControlCmds::SetPosM2;

        // RoboClaw expects big endian / MSB first
        encode_int32(position, &tx_buffer[0]);
        tx_buffer[4] = 1;   // Buffer argument -- implements this cmd immediately
        
        txrx(address, command, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_duty(uint8_t address, std::pair<int16_t, int16_t> duty) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_duty, this, address, duty)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueing set_duty");
    }
    void driver::exec_set_duty(uint8_t address, std::pair<int16_t, int16_t> duty) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[4];
        // RCLCPP_INFO(log_node->get_logger(), "Executing set_duty");
    
        // RoboClaw expects big endian / MSB first
        encode_int16(duty.first, &tx_buffer[0]);
        encode_int16(duty.second, &tx_buffer[2]);

        txrx(address, (uint8_t)AdvMotorControlCmds::SetDutyM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_duty_single(uint8_t address, uint8_t channel, int16_t duty) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_duty_single, this, address, channel, duty)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueing set_duty_single address: %d channel: %d duty: %d", address, channel, duty);
    }
    void driver::exec_set_duty_single(uint8_t address, uint8_t channel, int16_t duty) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[2];
        // RCLCPP_INFO(log_node->get_logger(), "Executing set_duty_single");
    
        uint8_t command = (channel == 1) ? (uint8_t)AdvMotorControlCmds::SetDutyM1 : (uint8_t)AdvMotorControlCmds::SetDutyM2;

        // RoboClaw expects big endian / MSB first
        encode_int16(duty, &tx_buffer[0]);

        txrx(address, command, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_velocity_pid(uint8_t address, uint8_t channel, velocity_pid_t k)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_velocity_pid, this, address, channel, k)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing set_velocity_pid on channel %d", channel);
    }
    void driver::exec_set_velocity_pid(uint8_t address, uint8_t channel, velocity_pid_t pid)
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[16];
        RCLCPP_INFO(log_node->get_logger(), "Executing set_velocity_pid on address %d channel %d with values qpps %u, p %6.4f, i %6.4f, d %6.4f", 
                    address, channel, pid.qpps, pid.p, pid.i, pid.d);
    
        uint8_t command = (channel == 1) ? (uint8_t)AdvMotorControlCmds::SetVelPIDM1 : (uint8_t)AdvMotorControlCmds::SetVelPIDM2;

        encode_int32((int32_t)(pid.d * driver::PID_CONST_MULT), &tx_buffer[0]);
        encode_int32((int32_t)(pid.p * driver::PID_CONST_MULT), &tx_buffer[4]);
        encode_int32((int32_t)(pid.i * driver::PID_CONST_MULT), &tx_buffer[8]);
        encode_uint32(pid.qpps, &tx_buffer[12]);

        txrx(address, command, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_position_pid(uint8_t address, uint8_t channel, position_pid_t k)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_pair(address, std::bind<void>(&driver::exec_set_position_pid, this, address, channel, k)));
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing set_position_pid on channel %d", channel);
    }
    void driver::exec_set_position_pid(uint8_t address, uint8_t channel, position_pid_t pid)
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[28];
        RCLCPP_INFO(log_node->get_logger(), "Executing set_position_pid on address %d channel %d with values deadzone %u, max_pos %d, min_pos %d, p %6.4f, i %6.4f, d %6.4f, max_i %6.4f", 
                    address, channel, pid.deadzone, pid.max_pos, pid.min_pos, pid.p, pid.i, pid.d, pid.max_i);
    
        uint8_t command = (channel == 1) ? (uint8_t)AdvMotorControlCmds::SetPosPIDM1 : (uint8_t)AdvMotorControlCmds::SetPosPIDM2;

        encode_int32((int32_t)(pid.d * driver::PID_CONST_MULT), &tx_buffer[0]);
        encode_int32((int32_t)(pid.p * driver::PID_CONST_MULT), &tx_buffer[4]);
        encode_int32((int32_t)(pid.i * driver::PID_CONST_MULT), &tx_buffer[8]);
        encode_int32((int32_t)(pid.max_i * driver::PID_CONST_MULT), &tx_buffer[12]);
        encode_int32((int32_t)pid.deadzone, &tx_buffer[16]);
        encode_int32(pid.max_pos, &tx_buffer[20]);
        encode_int32(pid.min_pos, &tx_buffer[24]);

        txrx(address, command, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);

    }

    bool driver::get_logic_voltage(uint8_t address, float &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((logic_voltages.find(address) != logic_voltages.end()) 
            and (logic_voltages_ready[address]))
        {
            result = logic_voltages[address];
            return true;
        }
        return false;
    }
    bool driver::get_motor_voltage(uint8_t address, float &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((motor_voltages.find(address) != motor_voltages.end()) 
            and (motor_voltages_ready[address]))
        {
            result = motor_voltages[address];
            return true;
        }
        return false;
    }
    bool driver::get_motor_current(uint8_t address, std::pair<float, float> &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((motor_currents.find(address) != motor_currents.end())
            and (motor_currents_ready[address]))
        {
            result = motor_currents[address];
            return true;
        }
        return false;
    }
    bool driver::get_version(uint8_t address, std::string &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((versions.find(address) != versions.end()) and (versions_ready[address]))
        {
            result = versions[address];
            return true;
        }
        return false;
    }
    bool driver::get_encoders(uint8_t address, std::pair<int, int> &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((encoders.find(address) != encoders.end()) and (encoders_ready[address]))
        {
            result = encoders[address];
            return true;
        }
        return false;
    }
    bool driver::get_motor_pwm(uint8_t address, std::pair<int16_t, int16_t> &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((motor_pwm.find(address) != motor_pwm.end()) and (motor_pwm_ready[address]))
        {
            result = motor_pwm[address];
            return true;
        }
        return false;

    }
    bool driver::get_velocity(uint8_t address, std::pair<int, int> &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((velocities.find(address) != velocities.end()) and 
            (velocities_ready[address]))
        {
            result = velocities[address];
            return true;
        }
        return false;
    }
    bool driver::get_position_errors(uint8_t address, std::pair<int, int> &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((posn_errors.find(address) != posn_errors.end()) 
            and (posn_err_ready[address]))
        {
            result = posn_errors[address];
            return true;
        }
        return false;
    }
    bool driver::get_status(uint8_t address, uint32_t &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if ((status.find(address) != status.end()) and (status_ready[address]))
        {
            result = status[address];
            return true;
        }
        return false;
    }
    bool driver::get_velocity_pid(uint8_t address, uint8_t channel, velocity_pid_t &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (velocity_pid_data.find(address) != velocity_pid_data.end())
        {
            if ((channel == 1) and velocity_pid_ready[address].first)
            {
                result = velocity_pid_data[address].first;
                return true;
            }
            else if ((channel == 2) and velocity_pid_ready[address].second)
            {
                result = velocity_pid_data[address].second;
                return true;
            }
        }
        return false;
    }
    bool driver::get_position_pid(uint8_t address, uint8_t channel, position_pid_t &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (position_pid_data.find(address) != position_pid_data.end())
        {
            if ((channel == 1) and position_pid_ready[address].first)
            {
                result = position_pid_data[address].first;
                return true;
            }
            else if ((channel == 2) and position_pid_ready[address].second)
            {
                result = position_pid_data[address].second;
                return true;
            }
        }
        return false;
    }

    bool driver::get_current_limit(uint8_t address, uint8_t channel, float &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (current_limit.find(address) != current_limit.end())
        {
            if ((channel == 1) and current_limit_ready[address].first)
            {
                result = current_limit[address].first;
                return true;
            }
            else if ((channel == 2) and current_limit_ready[address].second)
            {
                result = current_limit[address].second;
                return true;
            }
        }
        return false;

    }

    uint32_t driver::decode_uint32(uint8_t *buf)
    {
        uint32_t e1 = 0;
        e1 += buf[0] << 24;
        e1 += buf[1] << 16;
        e1 += buf[2] << 8;
        e1 += buf[3];
        return e1;
    }

    int32_t driver::decode_int32(uint8_t *buf)
    {
        return(static_cast<int32_t>(decode_uint32(buf)));
    }

    uint16_t driver::decode_uint16(uint8_t *buf)
    {
        uint16_t e1 = 0;
        e1 += buf[0] << 8;
        e1 += buf[1];
        return e1;
    }

    int16_t driver::decode_int16(uint8_t *buf)
    {
        return(static_cast<int16_t>(decode_uint16(buf)));
    }

    void driver::encode_uint32(uint32_t val, uint8_t *buf)
    {
        buf[0] = (uint8_t) ((val >> 24) & 0xFF);
        buf[1] = (uint8_t) ((val >> 16) & 0xFF);
        buf[2] = (uint8_t) ((val >> 8) & 0xFF);
        buf[3] = (uint8_t) (val & 0xFF);
    }

    void driver::encode_int32(int32_t val, uint8_t *buf)
    {
        encode_uint32(static_cast<uint32_t>(val), buf);
    }

    void driver::encode_uint16(uint16_t val, uint8_t *buf)
    {
        buf[0] = (uint8_t) ((val >> 8) & 0xFF);
        buf[1] = (uint8_t) (val & 0xFF);
    }

    void driver::encode_int16(int16_t val, uint8_t *buf)
    {
        encode_uint16(static_cast<uint16_t>(val), buf);
    }

    void driver::worker()
    {
        while(run_enable)
        {
            boost::mutex::scoped_lock qlock(queue_mutex);

            if (!command_queue.empty())
            {
                if (command_queue.size() > MAX_QUEUE_DEPTH)
                {
                    RCLCPP_ERROR(log_node->get_logger(), "Queue size: %ld", command_queue.size());
                }
                cmd_t cmd = command_queue.front();
                command_queue.pop();
                qlock.unlock();
                // RCLCPP_INFO(log_node->get_logger(), "Sending command %d...", std::get<0>(cmd));
                try { cmd.second(); }
                catch(roboclaw::crc_exception &e)
                {
                    RCLCPP_ERROR(log_node->get_logger(), "RoboClaw CRC error on node %d!", (cmd.first - driver::BASE_ADDRESS));
                    RCLCPP_INFO(log_node->get_logger(), "Clearing input buffer");
                    try
                    {
                        std::vector<char> tmp = serial->read(1000);
                    }
                    catch(timeout_exception &e) {}
                } 
                catch(timeout_exception &e)
                {
                    RCLCPP_ERROR(log_node->get_logger(), "RoboClaw timeout on node %d!", (cmd.first - driver::BASE_ADDRESS));
                }

            } 
            else 
            {
                qlock.unlock();  // not clear this is required
            }
        }
    }

}