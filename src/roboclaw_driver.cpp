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
    const uint8_t driver::MAX_QUEUE_DEPTH       = 20;
    const uint32_t driver::DEFAULT_BAUDRATE     = 38400;
    const float driver::AMPS_SCALE              = 100.0f;
    const float driver::VOLTS_SCALE             = 10.0f;
    const uint32_t driver::DEFAULT_TIMEOUT_MS   = 200;

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
            RCLCPP_ERROR(log_node->get_logger(),"Timeout reading from RoboClaw. Wanted %d bytes got %d", want_bytes, bytes_received);
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
                RCLCPP_INFO(log_node->get_logger(), "Received string length: %d", bytes_received);

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

    void driver::read_version(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        command_queue.push(std::make_tuple(CommandType::GetVersion, address, 0, 0));
        versions_ready[address] = false;
        RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_version");
    }
    void driver::exec_read_version(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[48];
        RCLCPP_INFO(log_node->get_logger(), "Executing read_version");

        txrx(address, (uint8_t)StatusCmds::ReadFirmwareVersion, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        std::string version = std::string(reinterpret_cast< char const * >(rx_buffer));
        trim(version);
        versions[address] = version;
        versions_ready[address] = true;
    }

    void driver::read_encoders(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        if (command_queue.size() > MAX_QUEUE_DEPTH) return;
        command_queue.push(std::make_tuple(CommandType::GetEncoders, address, 0, 0));
        encoders_ready[address] = false;
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_encoders");
    }
    void driver::exec_read_encoders(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[5];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_encoders");

        txrx(address, (uint8_t)EncoderCmds::ReadPosM1, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e1 = 0;

        e1 += rx_buffer[0] << 24;
        e1 += rx_buffer[1] << 16;
        e1 += rx_buffer[2] << 8;
        e1 += rx_buffer[3];

        txrx(address, (uint8_t)EncoderCmds::ReadPosM2, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e2 = 0;

        e2 += rx_buffer[0] << 24;
        e2 += rx_buffer[1] << 16;
        e2 += rx_buffer[2] << 8;
        e2 += rx_buffer[3];

        encoders[address] = std::pair<int, int>((int) (int32_t) e1, (int) (int32_t) e2);
        encoders_ready[address] = true;
    }

    void driver::read_velocity(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        if (command_queue.size() > MAX_QUEUE_DEPTH) return;
        command_queue.push(std::make_tuple(CommandType::GetVelocities, address, 0, 0));
        velocities_ready[address] = false;
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_velocity");
    }
    void driver::exec_read_velocity(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[5];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_velocity");

        txrx(address, (uint8_t)EncoderCmds::ReadSpdM1, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e1 = 0;

        e1 += rx_buffer[0] << 24;
        e1 += rx_buffer[1] << 16;
        e1 += rx_buffer[2] << 8;
        e1 += rx_buffer[3];

        txrx(address, (uint8_t)EncoderCmds::ReadSpdM2, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e2 = 0;

        e2 += rx_buffer[0] << 24;
        e2 += rx_buffer[1] << 16;
        e2 += rx_buffer[2] << 8;
        e2 += rx_buffer[3];

        velocities[address] = std::pair<int, int>((int) (int32_t) e1, (int) (int32_t) e2);
        velocities_ready[address] = true;
    }

    void driver::read_motor_currents(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        if (command_queue.size() > MAX_QUEUE_DEPTH) return;
        command_queue.push(std::make_tuple(CommandType::GetMotorCurrents, address, 0, 0));
        motor_currents_ready[address] = false;
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_motor_currents");
    }
    void driver::exec_read_motor_currents(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[4];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_motor_currents");

        txrx(address, (uint8_t)StatusCmds::ReadMotorCurrents, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint16_t e1 = 0;
        uint16_t e2 = 0;

        e1 += rx_buffer[0] << 8;
        e1 += rx_buffer[1];
        e2 += rx_buffer[2] << 8;
        e2 += rx_buffer[3];
        
        motor_currents[address] = std::pair<float, float>(((float)e1)/AMPS_SCALE, ((float)e2)/AMPS_SCALE);
        motor_currents_ready[address] = true;
    }

    void driver::read_logic_voltage(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        if (command_queue.size() > MAX_QUEUE_DEPTH) return;
        command_queue.push(std::make_tuple(CommandType::GetLogicVoltage, address, 0, 0));
        logic_voltages_ready[address] = false;
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_logic_voltages");
    }
    void driver::exec_read_logic_voltage(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[2];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_logic_voltages");

        txrx(address, (uint8_t)StatusCmds::ReadLogicVoltage, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint16_t e1 = 0;

        e1 += rx_buffer[0] << 8;
        e1 += rx_buffer[1];

        logic_voltages[address] = (float)e1/VOLTS_SCALE;
        logic_voltages_ready[address] = true;
    }

    void driver::read_motor_voltage(uint8_t address)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        boost::mutex::scoped_lock dlock(data_mutex);
        if (command_queue.size() > MAX_QUEUE_DEPTH) return;
        command_queue.push(std::make_tuple(CommandType::GetMotorVoltage, address, 0, 0));
        motor_voltages_ready[address] = false;
        // RCLCPP_INFO(log_node->get_logger(), "Enqueuing read_motor_voltages");
    }
    void driver::exec_read_motor_voltage(uint8_t address)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        uint8_t rx_buffer[2];
        // RCLCPP_INFO(log_node->get_logger(), "Executing read_motor_voltages");

        txrx(address, (uint8_t)StatusCmds::ReadMainVoltage, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint16_t e1 = 0;

        e1 += rx_buffer[0] << 8;
        e1 += rx_buffer[1];

        motor_voltages[address] = (float)e1/VOLTS_SCALE;
        motor_voltages_ready[address] = true;
    }

    void driver::reset_encoders(uint8_t address) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_tuple(CommandType::ResetEncoders, address, 0, 0));
        RCLCPP_INFO(log_node->get_logger(), "Enqueuing reset_encoders");
    }
    void driver::exec_reset_encoders(uint8_t address) 
    {
        uint8_t rx_buffer[1];
        RCLCPP_INFO(log_node->get_logger(), "Executing reset_encoders");
        txrx(address, (uint8_t)EncoderCmds::ResetAll, nullptr, 0, rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_velocity(uint8_t address, std::pair<int, int> speed) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_tuple(CommandType::SetVelocity, address, speed.first, speed.second));
        RCLCPP_INFO(log_node->get_logger(), "Enqueuing set_velocity");
    }
    void driver::exec_set_velocity(uint8_t address, int speed1, int speed2)
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[8];
        RCLCPP_INFO(log_node->get_logger(), "Executing set_velocity");

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((speed1 >> 24) & 0xFF);
        tx_buffer[1] = (uint8_t) ((speed1 >> 16) & 0xFF);
        tx_buffer[2] = (uint8_t) ((speed1 >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (speed1 & 0xFF);

        tx_buffer[4] = (uint8_t) ((speed2 >> 24) & 0xFF);
        tx_buffer[5] = (uint8_t) ((speed2 >> 16) & 0xFF);
        tx_buffer[6] = (uint8_t) ((speed2 >> 8) & 0xFF);
        tx_buffer[7] = (uint8_t) (speed2 & 0xFF);

        txrx(address, (uint8_t)AdvMotorControlCmds::SetSpdM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_velocity_single(uint8_t address, uint8_t channel, int speed) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_tuple(CommandType::SetVelocitySingle, address, channel, speed));
        RCLCPP_INFO(log_node->get_logger(), "Enqueing set_velocity_single");
    }
    void driver::exec_set_velocity_single(uint8_t address, uint8_t channel, int speed) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[4];
        RCLCPP_INFO(log_node->get_logger(), "Executing set_velocity_single");


        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((speed >> 24) & 0xFF);
        tx_buffer[1] = (uint8_t) ((speed >> 16) & 0xFF);
        tx_buffer[2] = (uint8_t) ((speed >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (speed & 0xFF);

        if (channel == 1)
        {
            txrx(address, (uint8_t)AdvMotorControlCmds::SetSpdM1, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
        }
        else if (channel == 2)
        {
            txrx(address, (uint8_t)AdvMotorControlCmds::SetSpdM2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
        }
    }

    void driver::set_position(uint8_t address, std::pair<int, int> position) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_tuple(CommandType::SetPosition, address, position.first, position.second));
        RCLCPP_INFO(log_node->get_logger(), "Enqueing set_position");
    }
    void driver::exec_set_position(uint8_t address, int posn1, int posn2) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[9];
        RCLCPP_INFO(log_node->get_logger(), "Executing set_position");

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((posn1 >> 24) & 0xFF);
        tx_buffer[1] = (uint8_t) ((posn1 >> 16) & 0xFF);
        tx_buffer[2] = (uint8_t) ((posn1 >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (posn1 & 0xFF);

        tx_buffer[4] = (uint8_t) ((posn2 >> 24) & 0xFF);
        tx_buffer[5] = (uint8_t) ((posn2 >> 16) & 0xFF);
        tx_buffer[6] = (uint8_t) ((posn2 >> 8) & 0xFF);
        tx_buffer[7] = (uint8_t) (posn2 & 0xFF);

        tx_buffer[8] = 1;   // Buffer argument -- implements this cmd immediately

        txrx(address, (uint8_t)AdvMotorControlCmds::SetPosM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }
        
    void driver::set_position_single(uint8_t address, uint8_t channel, int position)
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_tuple(CommandType::SetPositionSingle, address, channel, position));
        RCLCPP_INFO(log_node->get_logger(), "Enqueing set_position_single");
    }
    void driver::exec_set_position_single(uint8_t address, uint8_t channel, int position)
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[5];
        RCLCPP_INFO(log_node->get_logger(), "Executing set_position_single");
    
        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((position >> 24) & 0xFF);
        tx_buffer[1] = (uint8_t) ((position >> 16) & 0xFF);
        tx_buffer[2] = (uint8_t) ((position >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (position & 0xFF);
        
        tx_buffer[4] = 1;   // Buffer argument -- implements this cmd immediately

        if (channel == 1)
        {
            txrx(address, (uint8_t)AdvMotorControlCmds::SetPosM1, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
        }
        else if (channel == 2)
        {
            txrx(address, (uint8_t)AdvMotorControlCmds::SetPosM2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
        }
    }

    void driver::set_duty(uint8_t address, std::pair<int, int> duty) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_tuple(CommandType::SetDuty, address, duty.first, duty.second));
        RCLCPP_INFO(log_node->get_logger(), "Enqueing set_duty");
    }
    void driver::exec_set_duty(uint8_t address, int duty1, int duty2) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[4];
        RCLCPP_INFO(log_node->get_logger(), "Executing set_duty");
    
        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((duty1 >> 8) & 0xFF);
        tx_buffer[1] = (uint8_t) (duty1 & 0xFF);

        tx_buffer[2] = (uint8_t) ((duty2 >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (duty2 & 0xFF);

        txrx(address, (uint8_t)AdvMotorControlCmds::SetDutyM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_duty_single(uint8_t address, uint8_t channel, int duty) 
    {
        boost::mutex::scoped_lock qlock(queue_mutex);
        command_queue.push(std::make_tuple(CommandType::SetDutySingle, address, channel, duty));
        RCLCPP_INFO(log_node->get_logger(), "Enqueing set_duty_single");
    }
    void driver::exec_set_duty_single(uint8_t address, uint8_t channel, int duty) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[2];
        RCLCPP_INFO(log_node->get_logger(), "Executing set_duty_single");
    
        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((duty >> 8) & 0xFF);
        tx_buffer[1] = (uint8_t) (duty & 0xFF);

        if (channel == 1)
        {
            txrx(address, (uint8_t)AdvMotorControlCmds::SetDutyM1, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
        }
        else if (channel == 2)
        {
            txrx(address, (uint8_t)AdvMotorControlCmds::SetDutyM2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
        }
    }

    bool driver::get_logic_voltage(uint8_t address, float &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (logic_voltages.find(address) != logic_voltages.end())
        {
            if (logic_voltages_ready[address])
            {
                result = logic_voltages[address];
                return true;
            }
        }
        return false;
    }
    bool driver::get_motor_voltage(uint8_t address, float &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (motor_voltages.find(address) != motor_voltages.end())
        {
            if (motor_voltages_ready[address])
            {
                result = motor_voltages[address];
                return true;
            }
        }
        return false;
    }
    bool driver::get_motor_current(uint8_t address, std::pair<float, float> &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (motor_currents.find(address) != motor_currents.end())
        {
            if (motor_currents_ready[address])
            {
                result = motor_currents[address];
                return true;
            }
        }
        return false;
    }
    bool driver::get_version(uint8_t address, std::string &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (versions.find(address) != versions.end())
        {
            if (versions_ready[address])
            {
                result = versions[address];
                return true;
            }
        }
        return false;
    }
    bool driver::get_encoders(uint8_t address, std::pair<int, int> &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (encoders.find(address) != encoders.end())
        {
            if (encoders_ready[address])
            {
                result = encoders[address];
                return true;
            }
        }
        return false;
    }
    bool driver::get_velocity(uint8_t address, std::pair<int, int> &result)
    {
        boost::mutex::scoped_lock dlock(data_mutex);
        if (velocities.find(address) != velocities.end())
        {
            if (velocities_ready[address])
            {
                result = velocities[address];
                return true;
            }
        }
        return false;
    }

    void driver::worker()
    {
        while(run_enable)
        {
            boost::mutex::scoped_lock qlock(queue_mutex);

            if (!command_queue.empty())
            {
                cmd_t cmd = command_queue.front();
                command_queue.pop();
                qlock.unlock();
                // RCLCPP_INFO(log_node->get_logger(), "Sending command %d...", std::get<0>(cmd));
                try
                {
                    switch (std::get<0>(cmd))
                    {
                        case SetVelocity:       { exec_set_velocity(std::get<1>(cmd), std::get<2>(cmd), std::get<3>(cmd)); break; }
                        case SetVelocitySingle: { exec_set_velocity_single(std::get<1>(cmd), std::get<2>(cmd), std::get<3>(cmd)); break; }
                        case SetDuty:           { exec_set_duty(std::get<1>(cmd), std::get<2>(cmd), std::get<3>(cmd)); break; }
                        case SetDutySingle:     { exec_set_duty_single(std::get<1>(cmd), std::get<2>(cmd), std::get<3>(cmd)); break; }
                        case SetPosition:       { exec_set_position(std::get<1>(cmd), std::get<2>(cmd), std::get<3>(cmd)); break; }
                        case SetPositionSingle: { exec_set_position_single(std::get<1>(cmd), std::get<2>(cmd), std::get<3>(cmd)); break; }
                        case ResetEncoders:     { exec_reset_encoders(std::get<1>(cmd)); break; }
                        case GetLogicVoltage:   { exec_read_logic_voltage(std::get<1>(cmd)); break; }
                        case GetMotorVoltage:   { exec_read_motor_voltage(std::get<1>(cmd)); break; }
                        case GetMotorCurrents:  { exec_read_motor_currents(std::get<1>(cmd)); break; }
                        case GetEncoders:       { exec_read_encoders(std::get<1>(cmd)); break; }
                        case GetVelocities:     { exec_read_velocity(std::get<1>(cmd)); break; }
                        case GetVersion:        { exec_read_version(std::get<1>(cmd)); break; }
                        default:
                            break;
                    }
                }
                catch(roboclaw::crc_exception &e)
                {
                    RCLCPP_ERROR(log_node->get_logger(), "RoboClaw CRC error on node %d!", (std::get<1>(cmd) - driver::BASE_ADDRESS));
                } 
                catch(timeout_exception &e)
                {
                    RCLCPP_ERROR(log_node->get_logger(), "RoboClaw timeout on node %d!", (std::get<1>(cmd) - driver::BASE_ADDRESS));
                }

            } 
            else 
            {
                qlock.unlock();  // not clear this is required
            }
        }
    }

}