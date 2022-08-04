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
#include <functional>

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
        mRoboclaw = new driver(serial_port, baudrate, this);

        // zero all the encoders
        for (int r = 0; r < mClawCnt; r++)
        {
            mRoboclaw->reset_encoders(driver::BASE_ADDRESS + r);
        }

        for (uint8_t r = 0; r < mClawCnt; r++)
        {
            // create publishers and subscribers
            mEncodersPub[r] = this->create_publisher<roboclaw::msg::EncoderSteps>(
                "~/" + to_string(r) + "/posn_out", 10);
            mVoltsAmpsPub[r] = this->create_publisher<roboclaw::msg::MotorVoltsAmps>(
                "~/" + to_string(r) + "/volts_amps_out", 10);
            mVelocityPub[r] = this->create_publisher<roboclaw::msg::EncoderVelocity>(
                "~/" + to_string(r) + "/velocity_out", 10);

            function<void(const roboclaw::msg::MotorVelocity &)> vel_cb 
                = bind(&RoboclawCore::velocity_callback, this, r, _1);
            function<void(const roboclaw::msg::MotorVelocitySingle &)> vel_single_cb 
                = bind(&RoboclawCore::velocity_single_callback, this, r, _1);
            function<void(const roboclaw::msg::MotorPosition &)> posn_cb 
                = bind<void>(&RoboclawCore::position_callback, this, r, _1);
            function<void(const roboclaw::msg::MotorPositionSingle &)> posn_single_cb 
                = bind<void>(&RoboclawCore::position_single_callback, this, r, _1);
            function<void(const roboclaw::msg::MotorDutySingle &)> duty_single_cb 
                = bind<void>(&RoboclawCore::duty_single_callback, this, r, _1);
            mVelCmdSub[r] = this->create_subscription<roboclaw::msg::MotorVelocity>(
                "~/" + to_string(r) + "/motor_vel_cmd", 10, vel_cb);
            mVelCmdSingleSub[r] = this->create_subscription<roboclaw::msg::MotorVelocitySingle>(
                "~/" + to_string(r) + "/motor_vel_single_cmd", 10, vel_single_cb);
            mPosCmdSub[r] = this->create_subscription<roboclaw::msg::MotorPosition>(
                "~/" + to_string(r) + "/motor_pos_cmd", 10, posn_cb);
            mPosCmdSingleSub[r] = this->create_subscription<roboclaw::msg::MotorPositionSingle>(
                "~/" + to_string(r) + "/motor_pos_single_cmd", 10, posn_single_cb);
            mDutyCmdSingleSub[r] = this->create_subscription<roboclaw::msg::MotorDutySingle>(
                "~/" + to_string(r) + "/motor_duty_single_cmd", 10, duty_single_cb);
        }

        // create periodic callback to trigger data fetching
        mPubTimer = this->create_wall_timer(
            100ms, bind(&RoboclawCore::timer_callback, this));

        // Create worker thread to poll for returned data
        mRunEnable = true;
        mPubWorkerThread = unique_ptr<thread>(new thread(&RoboclawCore::pub_worker, this));
    }

    RoboclawCore::~RoboclawCore() {
        for (int r = 0; r < mClawCnt; r++)
        {
            mRoboclaw->set_duty(driver::BASE_ADDRESS + r, std::pair<int, int>(0, 0));
        }
        mRunEnable = false;
        mPubWorkerThread->join();
    }

    void RoboclawCore::duty_single_callback(uint8_t idx, const roboclaw::msg::MotorDutySingle &msg) 
    {
        if ((msg.channel > 2) or (msg.channel < 1))
        {
            RCLCPP_ERROR(this->get_logger(),  "Roboclaw channel must be either 1 or 2");
            return;
        }

        mRoboclaw->set_duty_single(driver::BASE_ADDRESS + idx, msg.channel, msg.mot_duty);
    }

    void RoboclawCore::velocity_callback(uint8_t idx, const roboclaw::msg::MotorVelocity &msg) 
    {
        mRoboclaw->set_velocity(driver::BASE_ADDRESS + idx, std::pair<int, int>(msg.mot1_vel_sps, msg.mot2_vel_sps));
    }

    void RoboclawCore::velocity_single_callback(uint8_t idx, const roboclaw::msg::MotorVelocitySingle &msg) 
    {
        if ((msg.channel > 2) or (msg.channel < 1))
        {
            RCLCPP_ERROR(this->get_logger(),  "Roboclaw channel must be either 1 or 2");
            return;
        }
        mRoboclaw->set_velocity_single(driver::BASE_ADDRESS + idx, msg.channel, msg.mot_vel_sps);
    }

    void RoboclawCore::position_callback(uint8_t idx, const roboclaw::msg::MotorPosition &msg) 
    {
        mRoboclaw->set_position(driver::BASE_ADDRESS + idx, std::pair<int, int>(msg.mot1_pos_steps, msg.mot2_pos_steps));
    }

    void RoboclawCore::position_single_callback(uint8_t idx, const roboclaw::msg::MotorPositionSingle &msg) 
    {
        if ((msg.channel > 2) or (msg.channel < 1))
        {
            RCLCPP_ERROR(this->get_logger(),  "Roboclaw channel must be either 1 or 2");
            return;
        }
        mRoboclaw->set_position_single(driver::BASE_ADDRESS + idx, msg.channel, msg.mot_pos_steps);
    }

    void RoboclawCore::timer_callback() {

        // Request data
        for (int r = 0; r < mClawCnt; r++) 
        {
            mRoboclaw->read_encoders(driver::BASE_ADDRESS + r);
            mRoboclaw->read_velocity(driver::BASE_ADDRESS + r);
            mRoboclaw->read_motor_currents(driver::BASE_ADDRESS + r);
            mRoboclaw->read_motor_voltage(driver::BASE_ADDRESS + r);
            mRoboclaw->read_logic_voltage(driver::BASE_ADDRESS + r);
            mDataArrived[r] = false;
        }
    }

    void RoboclawCore::pub_worker()
    {
        while (mRunEnable)
        {
            for (int r = 0; r < mClawCnt; r++) 
            {
                float logicV, motorV;
                pair<float, float> currents;
                pair<int, int> encoders, velocity;
                if (!mDataArrived[r] 
                    && mRoboclaw->get_logic_voltage((driver::BASE_ADDRESS + r), logicV)
                    && mRoboclaw->get_motor_voltage((driver::BASE_ADDRESS + r), motorV)
                    && mRoboclaw->get_motor_current((driver::BASE_ADDRESS + r), currents)
                    && mRoboclaw->get_encoders((driver::BASE_ADDRESS + r), encoders)
                    && mRoboclaw->get_velocity((driver::BASE_ADDRESS + r), velocity)
                ) {
                    mDataArrived[r] = true;
                    auto msg_encs = roboclaw::msg::EncoderSteps();
                    auto msg_amps = roboclaw::msg::MotorVoltsAmps();
                    auto msg_vel = roboclaw::msg::EncoderVelocity();

                    msg_encs.index = r;
                    msg_encs.mot1_enc_steps = encoders.first;
                    msg_encs.mot2_enc_steps = encoders.second;

                    msg_vel.index = r;
                    msg_vel.mot1_enc_vel = velocity.first;
                    msg_vel.mot2_enc_vel = velocity.second;

                    msg_amps.index = r;
                    msg_amps.motor_volts = motorV;
                    msg_amps.logic_volts = logicV;
                    msg_amps.mot1_amps = currents.first;
                    msg_amps.mot2_amps = currents.second;

                    mEncodersPub[r]->publish(msg_encs);
                    mVoltsAmpsPub[r]->publish(msg_amps);
                    mVelocityPub[r]->publish(msg_vel);
                }
            }
        }
    }
}
