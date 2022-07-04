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

#include "roboclaw_roscore.h"

#include <map>
#include <string>
#include <iostream>

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

namespace roboclaw 
{

    RoboclawCore::RoboclawCore(string name) : Node(name)
    {
        // Declare the parameters
        this->declare_parameter<std::string>("serial_port", "");    // serial device name
        this->declare_parameter<int64_t>("baudrate", driver::DEFAULT_BAUDRATE);   // default baud rate
        this->declare_parameter<int64_t>("num_claws", 1);       // Number of roboclaws connected

        // Declare a couple of variables...
        std::string serial_port;
        int64_t baudrate;

        // Read the parameters in, throw and error if the serial port name is short
        this->get_parameter("serial_port", serial_port);
        if(serial_port.size() < 1)
        {
            throw std::runtime_error("Must specify serial port");
        }
        this->get_parameter("baudrate", baudrate);
        this->get_parameter("num_roboclaws", mClawCnt);

        // initialize the driver
        mRoboclaw = new driver(serial_port, baudrate);

        // zero all the encoders
        for (int r = 0; r < mClawCnt; r++)
        {
            mRoboclaw->reset_encoders(driver::BASE_ADDRESS + r);
        }

        // create publishers and subscribers
        mEncodersPub = this->create_publisher<roboclaw::msg::EncoderSteps>(
            "~/posn_out", 10);
        mVoltsAmpsPub = this->create_publisher<roboclaw::msg::MotorVoltsAmps>(
            "~/volts_amps_out", 10);

        mVelCmdSub = this->create_subscription<roboclaw::msg::MotorVelocity>(
            "~/motor_vel_cmd", 10, bind(&RoboclawCore::velocity_callback, this, _1));
        mVelCmdSingleSub = this->create_subscription<roboclaw::msg::MotorVelocitySingle>(
            "~/motor_vel_single_cmd", 10, bind(&RoboclawCore::velocity_single_callback, this, _1));
        mPosCmdSub = this->create_subscription<roboclaw::msg::MotorPosition>(
            "~/motor_pos_cmd", 10, bind(&RoboclawCore::position_callback, this, _1));
        mPosCmdSingleSub = this->create_subscription<roboclaw::msg::MotorPositionSingle>(
            "~/motor_pos_single_cmd", 10, bind(&RoboclawCore::position_single_callback, this, _1));
        mDutyCmdSingleSub = this->create_subscription<roboclaw::msg::MotorDutySingle>(
            "~/motor_duty_single_cmd", 10, bind(&RoboclawCore::duty_single_callback, this, _1));

        // create periodic callback
        mPubTimer = this->create_wall_timer(
            100ms, bind(&RoboclawCore::timer_callback, this));

    }

    RoboclawCore::~RoboclawCore() {
        for (int r = 0; r < mClawCnt; r++)
            mRoboclaw->set_duty(driver::BASE_ADDRESS + r, std::pair<int, int>(0, 0));
    }

    void RoboclawCore::duty_single_callback(const roboclaw::msg::MotorDutySingle &msg) 
    {
        if ((msg.channel > 2) or (msg.channel < 1))
        {
            RCLCPP_ERROR(this->get_logger(),  "Roboclaw channel must be either 1 or 2");
            return;
        }

        try 
        {
            mRoboclaw->set_duty_single(driver::BASE_ADDRESS + msg.index, msg.channel, msg.mot_duty);

        } 
        catch(roboclaw::crc_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw CRC error during set velocity!");
        } 
        catch(timeout_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw timeout during set velocity!");
        }

    }

    void RoboclawCore::velocity_callback(const roboclaw::msg::MotorVelocity &msg) 
    {
        // mLastVelCmd = rclcpp::Time::now(RCL_ROS_TIME);

        try 
        {
            mRoboclaw->set_velocity(driver::BASE_ADDRESS + msg.index, std::pair<int, int>(msg.mot1_vel_sps, msg.mot2_vel_sps));
        } 
        catch(roboclaw::crc_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw CRC error during set velocity!");
        } 
        catch(timeout_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw timeout during set velocity!");
        }

    }

    void RoboclawCore::velocity_single_callback(const roboclaw::msg::MotorVelocitySingle &msg) 
    {
        if ((msg.channel > 2) or (msg.channel < 1))
        {
            RCLCPP_ERROR(this->get_logger(),  "Roboclaw channel must be either 1 or 2");
            return;
        }

        try 
        {
            mRoboclaw->set_velocity_single(driver::BASE_ADDRESS + msg.index, msg.channel, msg.mot_vel_sps);

        } 
        catch(roboclaw::crc_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw CRC error during set velocity!");
        } 
        catch(timeout_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw timeout during set velocity!");
        }

    }

    void RoboclawCore::position_callback(const roboclaw::msg::MotorPosition &msg) 
    {
        // mLastPosCmd = rclcpp::Time::now();

        try 
        {
            mRoboclaw->set_position(driver::BASE_ADDRESS + msg.index, std::pair<int, int>(msg.mot1_pos_steps, msg.mot2_pos_steps));
        } 
        catch(roboclaw::crc_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw CRC error during set position!");
        } 
        catch(timeout_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw timout during set set position!");
        }
    }

    void RoboclawCore::position_single_callback(const roboclaw::msg::MotorPositionSingle &msg) 
    {
        if ((msg.channel > 2) or (msg.channel < 1))
        {
            RCLCPP_ERROR(this->get_logger(),  "Roboclaw channel must be either 1 or 2");
            return;
        }

        try 
        {
            mRoboclaw->set_position_single(driver::BASE_ADDRESS + msg.index, msg.channel, msg.mot_pos_steps);
        } 
        catch(roboclaw::crc_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw CRC error during set position!");
        } 
        catch(timeout_exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RoboClaw timout during set set position!");
        }

    }

    void RoboclawCore::timer_callback() {

        // Publish encoders
        for (int r = 0; r < mClawCnt; r++) {
            try 
            {
                auto msg_encs = roboclaw::msg::EncoderSteps();
                auto msg_amps = roboclaw::msg::MotorVoltsAmps();
                auto encs = mRoboclaw->get_encoders(driver::BASE_ADDRESS + r);
                auto amps = mRoboclaw->get_motor_currents(driver::BASE_ADDRESS + r);
                
                msg_encs.index = r;
                msg_encs.mot1_enc_steps = encs.first;
                msg_encs.mot2_enc_steps = encs.second;

                msg_amps.index = r;
                msg_amps.motor_volts = mRoboclaw->get_motor_voltage(driver::BASE_ADDRESS + r);
                msg_amps.logic_volts = mRoboclaw->get_logic_voltage(driver::BASE_ADDRESS + r);
                msg_amps.mot1_amps = amps.first;
                msg_amps.mot2_amps = amps.second;

                mEncodersPub->publish(msg_encs);
                mVoltsAmpsPub->publish(msg_amps);
            } 
            catch(roboclaw::crc_exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "RoboClaw CRC error during getting encoders!");
                continue;
            } 
            catch(timeout_exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "RoboClaw timout during getting encoders!");
                continue;
            }
        }
    }
}
