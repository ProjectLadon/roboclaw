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

    const uint8_t driver::BASE_ADDRESS      = 0x80;
    const uint32_t driver::DEFAULT_BAUDRATE = 115200;
    const float driver::AMPS_SCALE          = 100.0f;
    const float driver::VOLTS_SCALE         = 10.0f;

    driver::driver(std::string port, unsigned int baudrate) 
    {
        serial = std::shared_ptr<TimeoutSerial>(new TimeoutSerial(port, baudrate));
        serial->setTimeout(boost::posix_time::milliseconds(200));
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

        size_t want_bytes;
        if (rx_crc)
            want_bytes = rx_length + 2;
        else
            want_bytes = rx_length;

        std::vector<char> response_vector;

        response_vector = serial->read(want_bytes);

        size_t bytes_received = response_vector.size();

        uint8_t* response = (uint8_t*) &response_vector[0];

        if (bytes_received != want_bytes)
            throw timeout_exception("Timeout reading from RoboClaw");

        // Check CRC
        if (rx_crc) {
            unsigned int crc_calculated = crc16(&response[0], bytes_received - 2);
            unsigned int crc_received = 0;

            // RoboClaw generates big endian / MSB first
            crc_received += response[bytes_received - 2] << 8;
            crc_received += response[bytes_received - 1];

            if (crc_calculated != crc_received) {
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

    std::string driver::get_version(uint8_t address) 
    {

        uint8_t rx_buffer[48];

        txrx(address, (uint8_t)StatusCmds::ReadFirmwareVersion, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        std::string version = std::string(reinterpret_cast< char const * >(rx_buffer));
        trim(version);

        return version;

    }

    std::pair<int, int> driver::get_encoders(uint8_t address) 
    {

        uint8_t rx_buffer[5];

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

        return std::pair<int, int>((int) (int32_t) e1, (int) (int32_t) e2);
    }

    std::pair<int, int> driver::get_velocity(uint8_t address) 
    {

        uint8_t rx_buffer[5];

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

        return std::pair<int, int>((int) (int32_t) e1, (int) (int32_t) e2);
    }

    std::pair<float, float> driver::get_motor_currents(uint8_t address)
    {
        uint8_t rx_buffer[4];

        txrx(address, (uint8_t)StatusCmds::ReadMotorCurrents, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint16_t e1 = 0;
        uint16_t e2 = 0;

        e1 += rx_buffer[0] << 8;
        e1 += rx_buffer[1];
        e2 += rx_buffer[2] << 8;
        e2 += rx_buffer[3];
        
        return std::pair<float, float>(((float)e1)/AMPS_SCALE, ((float)e2)/AMPS_SCALE);
    }

    float driver::get_logic_voltage(uint8_t address)
    {
        uint8_t rx_buffer[2];

        txrx(address, (uint8_t)StatusCmds::ReadLogicVoltage, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint16_t e1 = 0;

        e1 += rx_buffer[0] << 8;
        e1 += rx_buffer[1];

        return (float)e1/VOLTS_SCALE;

    }

    float driver::get_motor_voltage(uint8_t address)
    {
        uint8_t rx_buffer[2];

        txrx(address, (uint8_t)StatusCmds::ReadMainVoltage, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint16_t e1 = 0;

        e1 += rx_buffer[0] << 8;
        e1 += rx_buffer[1];

        return (float)e1/VOLTS_SCALE;

    }

    void driver::reset_encoders(uint8_t address) 
    {
        uint8_t rx_buffer[1];
        txrx(address, (uint8_t)EncoderCmds::ResetAll, nullptr, 0, rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_velocity(uint8_t address, std::pair<int, int> speed) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[8];

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((speed.first >> 24) & 0xFF);
        tx_buffer[1] = (uint8_t) ((speed.first >> 16) & 0xFF);
        tx_buffer[2] = (uint8_t) ((speed.first >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (speed.first & 0xFF);

        tx_buffer[4] = (uint8_t) ((speed.second >> 24) & 0xFF);
        tx_buffer[5] = (uint8_t) ((speed.second >> 16) & 0xFF);
        tx_buffer[6] = (uint8_t) ((speed.second >> 8) & 0xFF);
        tx_buffer[7] = (uint8_t) (speed.second & 0xFF);

        txrx(address, (uint8_t)AdvMotorControlCmds::SetSpdM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_position(uint8_t address, std::pair<int, int> position) 
    {
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[8];

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((position.first >> 24) & 0xFF);
        tx_buffer[1] = (uint8_t) ((position.first >> 16) & 0xFF);
        tx_buffer[2] = (uint8_t) ((position.first >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (position.first & 0xFF);

        tx_buffer[4] = (uint8_t) ((position.second >> 24) & 0xFF);
        tx_buffer[5] = (uint8_t) ((position.second >> 16) & 0xFF);
        tx_buffer[6] = (uint8_t) ((position.second >> 8) & 0xFF);
        tx_buffer[7] = (uint8_t) (position.second & 0xFF);

        txrx(address, (uint8_t)AdvMotorControlCmds::SetPosM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }
        
    void driver::set_position_single(uint8_t address, uint8_t channel, int position)
{
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[4];

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((position >> 24) & 0xFF);
        tx_buffer[1] = (uint8_t) ((position >> 16) & 0xFF);
        tx_buffer[2] = (uint8_t) ((position >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (position & 0xFF);

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
        uint8_t rx_buffer[1];
        uint8_t tx_buffer[4];

        // RoboClaw expects big endian / MSB first
        tx_buffer[0] = (uint8_t) ((duty.first >> 8) & 0xFF);
        tx_buffer[1] = (uint8_t) (duty.first & 0xFF);

        tx_buffer[2] = (uint8_t) ((duty.second >> 8) & 0xFF);
        tx_buffer[3] = (uint8_t) (duty.second & 0xFF);

        txrx(address, (uint8_t)AdvMotorControlCmds::SetDutyM1M2, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

}