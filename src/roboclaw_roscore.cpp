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
#include <limits>

using namespace std;
using namespace std::chrono_literals;

namespace roboclaw 
{

    RoboclawCore::RoboclawCore(string name) : Node(name)
    {
        // Declare the parameters
        this->declare_parameter<std::string>("serial_port", "");    // serial device name
        this->declare_parameter<int64_t>("baudrate", driver::DEFAULT_BAUDRATE);   // default baud rate
        this->declare_parameter<int32_t>("num_claws", 1);       // Number of roboclaws connected
        this->declare_parameter<int32_t>("timeout_ms", driver::DEFAULT_TIMEOUT_MS);       // Serial timeout, in milliseconds
        this->declare_parameter<int32_t>("data_request_period_ms", 100);    // period between position/current/voltage requests in ms

        // Declare a couple of variables...
        std::string serial_port;
        int64_t baudrate = driver::DEFAULT_BAUDRATE;
        uint32_t timeout = driver::DEFAULT_TIMEOUT_MS;
        uint32_t request_period = 100;

        // Read the parameters in, throw and error if the serial port name is short
        this->get_parameter("serial_port", serial_port);
        if(serial_port.size() < 1)
        {
            throw std::runtime_error("Must specify serial port");
        }
        this->get_parameter("baudrate", baudrate);
        this->get_parameter("num_claws", mClawCnt);
        this->get_parameter("timeout_ms", timeout);
        this->get_parameter("data_request_period_ms", request_period);

        // get position limits
        for (int i = 0; i < mClawCnt; i++)
        {
            string limit_param_base = "posn_limits.node" + to_string(i) + ".chan";
            string throttle_param_1 = "rate_limit.node" + to_string(i) + ".chan1";
            string throttle_param_2 = "rate_limit.node" + to_string(i) + ".chan2";
            string limit_param_upper_1 = limit_param_base + "1.upper_limit";
            string limit_param_upper_2 = limit_param_base + "2.upper_limit";
            string limit_param_lower_1 = limit_param_base + "1.lower_limit";
            string limit_param_lower_2 = limit_param_base + "2.lower_limit";
            string limit_param_center_1 = limit_param_base + "1.center";
            string limit_param_center_2 = limit_param_base + "2.center";
            this->declare_parameter<int32_t>(limit_param_upper_1, numeric_limits<int32_t>::max());
            this->declare_parameter<int32_t>(limit_param_upper_2, numeric_limits<int32_t>::max());
            this->declare_parameter<int32_t>(limit_param_lower_1, numeric_limits<int32_t>::min());
            this->declare_parameter<int32_t>(limit_param_lower_2, numeric_limits<int32_t>::min());
            this->declare_parameter<int32_t>(limit_param_center_1, 0);
            this->declare_parameter<int32_t>(limit_param_center_2, 0);
            this->declare_parameter<uint8_t>(throttle_param_1, 0);
            this->declare_parameter<uint8_t>(throttle_param_2, 0);
            int32_t max1, max2, min1, min2, center1, center2;
            uint8_t throttle_1, throttle_2;
            this->get_parameter(limit_param_upper_1, max1);
            this->get_parameter(limit_param_upper_2, max2);
            this->get_parameter(limit_param_lower_1, min1);
            this->get_parameter(limit_param_lower_2, min2);
            this->get_parameter(limit_param_center_1, center1);
            this->get_parameter(limit_param_center_2, center2);
            this->get_parameter(throttle_param_1, throttle_1);
            this->get_parameter(throttle_param_2, throttle_2);
            mPosnLimits.push_back(make_tuple(make_limits(max1, min1, center1), make_limits(max2, min2, center2)));
            mCmdThrottleLimit.push_back(pair<uint8_t, uint8_t>(throttle_1, throttle_2));
            mCmdThrottleCounter.push_back(pair<uint8_t, uint8_t>(0, 0));
        }

        // initialize the driver
        mRoboclaw = new driver(serial_port, baudrate, timeout, this);

        // zero all the encoders
        for (int r = 0; r < mClawCnt; r++)
        {
            // Commenting this out because it's not something every one needs
            // mRoboclaw->reset_encoders(driver::BASE_ADDRESS + r);
        }

        mDataArrived = unique_ptr<vector<atomic<bool>>>(new vector<atomic<bool>>(mClawCnt));

        for (uint8_t r = 0; r < mClawCnt; r++)
        {
            mDataArrived->at(r) = false;
            RCLCPP_INFO(this->get_logger(),  "Creating publishers for node %d", r);
            // create publishers and subscribers
            mEncodersPub.emplace_back(this->create_publisher<roboclaw::msg::EncoderSteps>(
                "~/claw" + to_string(r) + "/posn_out", 10));
            mErrorsPub.emplace_back(this->create_publisher<roboclaw::msg::EncoderSteps>(
                "~/claw" + to_string(r) + "/posn_err", 10));
            mVoltsAmpsPub.emplace_back(this->create_publisher<roboclaw::msg::MotorVoltsAmps>(
                "~/claw" + to_string(r) + "/volts_amps_out", 10));
            mVelocityPub.emplace_back(this->create_publisher<roboclaw::msg::EncoderVelocity>(
                "~/claw" + to_string(r) + "/velocity_out", 10));
            mStatusPub.emplace_back(this->create_publisher<roboclaw::msg::Status>(
                "~/claw" + to_string(r) + "/status", 10));

            RCLCPP_INFO(this->get_logger(),  "Creating subscribers for node %d", r);
            mVelCmdSub.emplace_back(this->create_subscription<roboclaw::msg::MotorVelocity>(
                "~/claw" + to_string(r) + "/motor_vel_cmd", 10, 
                function<void(const roboclaw::msg::MotorVelocity &)>(
                bind(&RoboclawCore::velocity_callback, this, r, std::placeholders::_1)
            )));
            mVelCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorVelocitySingle>(
                "~/claw" + to_string(r) + "/motor_vel_single_cmd/chan1", 10, 
                function<void(const roboclaw::msg::MotorVelocitySingle &)>(
                bind(&RoboclawCore::velocity_single_callback, this, r, 1, std::placeholders::_1)
            )));
            mVelCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorVelocitySingle>(
                "~/claw" + to_string(r) + "/motor_vel_single_cmd/chan2", 10, 
                function<void(const roboclaw::msg::MotorVelocitySingle &)>(
                bind(&RoboclawCore::velocity_single_callback, this, r, 2, std::placeholders::_1)
            )));
            mPosCmdSub.emplace_back(this->create_subscription<roboclaw::msg::MotorPosition>(
                "~/claw" + to_string(r) + "/motor_pos_cmd", 10, 
                function<void(const roboclaw::msg::MotorPosition &)>(
                bind<void>(&RoboclawCore::position_callback, this, r, std::placeholders::_1)
            )));
            mPosCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorPositionSingle>(
                "~/claw" + to_string(r) + "/motor_pos_single_cmd/chan1", 10,  
                function<void(const roboclaw::msg::MotorPositionSingle &)>(
                bind<void>(&RoboclawCore::position_single_callback, this, r, 1, std::placeholders::_1)
            )));
            mPosCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorPositionSingle>(
                "~/claw" + to_string(r) + "/motor_pos_single_cmd/chan2", 10,  
                function<void(const roboclaw::msg::MotorPositionSingle &)>(
                bind<void>(&RoboclawCore::position_single_callback, this, r, 2, std::placeholders::_1)
            )));
            mDutyCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorDutySingle>(
                "~/claw" + to_string(r) + "/motor_duty_single_cmd/chan1", 10, 
                function<void(const roboclaw::msg::MotorDutySingle &)>(
                bind<void>(&RoboclawCore::duty_single_callback, this, r, 1, std::placeholders::_1)
            )));
            mDutyCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorDutySingle>(
                "~/claw" + to_string(r) + "/motor_duty_single_cmd/chan2", 10, 
                function<void(const roboclaw::msg::MotorDutySingle &)>(
                bind<void>(&RoboclawCore::duty_single_callback, this, r, 2, std::placeholders::_1)
            )));

            RCLCPP_INFO(this->get_logger(),  "Initialization complete for roboclaw %d", r);
        }

        // create periodic callback to trigger data fetching
        mPubTimer = this->create_wall_timer(
            (request_period * 1ms), bind(&RoboclawCore::timer_callback, this));
            // 100ms, bind(&RoboclawCore::timer_callback, this));

        // Create worker thread to access serial port
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

    bool RoboclawCore::bad_inputs(uint8_t idx, uint8_t chan)
    {
        if ((chan > 2) or (chan < 1))
        {
            RCLCPP_ERROR(this->get_logger(),  "Roboclaw channel must be either 1 or 2; is %d", chan);
            return true;
        }
        if (idx >= mClawCnt)
        {
            RCLCPP_ERROR(this->get_logger(),  "Only %d roboclaws are connected; you called for %d", mClawCnt, idx);
            return true;
        }
        return false;
    }

    void RoboclawCore::duty_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorDutySingle &msg) 
    {
        // RCLCPP_INFO(this->get_logger(), "Received a single motor duty coImmand");
        if (bad_inputs(idx, chan)) return;
        if (!is_throttle_clear(idx, chan)) return;
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping single duty command");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Setting a single motor duty command");
        mRoboclaw->set_duty_single(driver::BASE_ADDRESS + idx, chan, msg.mot_duty);
    }

    void RoboclawCore::velocity_callback(uint8_t idx, const roboclaw::msg::MotorVelocity &msg) 
    {
        if(bad_inputs(idx, 1)) return;
        if ((!is_throttle_clear(idx, 1)) or (!is_throttle_clear(idx, 2))) return;
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping dual velocity command");
            return;
        }
        mRoboclaw->set_velocity(driver::BASE_ADDRESS + idx, std::pair<int, int>(msg.mot1_vel_sps, msg.mot2_vel_sps));
    }

    void RoboclawCore::velocity_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorVelocitySingle &msg) 
    {
        if(bad_inputs(idx, chan)) return;
        if (!is_throttle_clear(idx, chan)) return;
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping single velocity command");
            return;
        }
        mRoboclaw->set_velocity_single(driver::BASE_ADDRESS + idx, msg.channel, msg.mot_vel_sps);
    }

    void RoboclawCore::position_callback(uint8_t idx, const roboclaw::msg::MotorPosition &msg) 
    {
        if(bad_inputs(idx, 1)) return;
        if ((!is_throttle_clear(idx, 1)) or (!is_throttle_clear(idx, 2))) return;
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping dual position command");
            return;
        }
        mRoboclaw->set_position(driver::BASE_ADDRESS + idx, std::pair<int, int>(
            bound_input(msg.mot1_pos_steps, get_max_posn(get<0>(mPosnLimits[idx])), get_min_posn(get<0>(mPosnLimits[idx]))), 
            bound_input(msg.mot2_pos_steps, get_max_posn(get<1>(mPosnLimits[idx])), get_min_posn(get<1>(mPosnLimits[idx])))));
    }

    void RoboclawCore::position_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorPositionSingle &msg) 
    {
        if(bad_inputs(idx, chan)) return;
        if (!is_throttle_clear(idx, chan)) return;
        int32_t max = numeric_limits<int32_t>::max(); 
        int32_t min = numeric_limits<int32_t>::min();
        if (msg.channel == 1)
        {
            min = get_min_posn(get<0>(mPosnLimits[idx]));
            max = get_max_posn(get<0>(mPosnLimits[idx]));
        }
        else if (msg.channel == 2)
        {
            min = get_min_posn(get<1>(mPosnLimits[idx]));
            max = get_max_posn(get<1>(mPosnLimits[idx]));
        }
        int32_t cmd = bound_input(msg.mot_pos_steps, max, min);
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping single position command");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Setting channel %d position on index %d to %d", chan, idx, cmd);
        mRoboclaw->set_position_single(driver::BASE_ADDRESS + idx, msg.channel, cmd);
    }

    void RoboclawCore::timer_callback() {

        // Request data
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping feedback");
            return;
        }
        for (int r = 0; r < mClawCnt; r++) 
        {
            mRoboclaw->read_encoders(driver::BASE_ADDRESS + r);
            mRoboclaw->read_velocity(driver::BASE_ADDRESS + r);
            mRoboclaw->read_motor_currents(driver::BASE_ADDRESS + r);
            mRoboclaw->read_motor_voltage(driver::BASE_ADDRESS + r);
            mRoboclaw->read_logic_voltage(driver::BASE_ADDRESS + r);
            mRoboclaw->read_position_errors(driver::BASE_ADDRESS + r);
            mRoboclaw->read_status(driver::BASE_ADDRESS + r);
            mDataArrived->at(r) = false;
        }
    }

    bool RoboclawCore::is_throttle_clear(uint8_t idx, uint8_t chan)
    {
        if(bad_inputs(idx, chan)) return false;
        if (chan == 1) 
        { 
            mCmdThrottleCounter[idx].first++;
            if (mCmdThrottleCounter[idx].first > mCmdThrottleLimit[idx].first)
            {
                mCmdThrottleCounter[idx].first = 0;
                return true;
            }
        }
        else if (chan == 2)
        {
            mCmdThrottleCounter[idx].second++;
            if (mCmdThrottleCounter[idx].second > mCmdThrottleLimit[idx].second)
            {
                mCmdThrottleCounter[idx].second = 0;
                return true;
            }
        }
        return false;
    }

    void RoboclawCore::pub_worker()
    {
        while (mRunEnable)
        {
            for (int r = 0; r < mClawCnt; r++) 
            {
                float logicV, motorV;
                pair<float, float> currents;
                pair<int, int> encoders, velocity, errors;
                uint32_t status;
                if (!mDataArrived->at(r) 
                    and mRoboclaw->get_logic_voltage((driver::BASE_ADDRESS + r), logicV)
                    and mRoboclaw->get_motor_voltage((driver::BASE_ADDRESS + r), motorV)
                    and mRoboclaw->get_motor_current((driver::BASE_ADDRESS + r), currents)
                    and mRoboclaw->get_encoders((driver::BASE_ADDRESS + r), encoders)
                    and mRoboclaw->get_velocity((driver::BASE_ADDRESS + r), velocity)
                    and mRoboclaw->get_position_errors((driver::BASE_ADDRESS + r), errors)
                    and mRoboclaw->get_status((driver::BASE_ADDRESS + r), status)
                ) {
                    mDataArrived->at(r) = true;
                    auto msg_encs = roboclaw::msg::EncoderSteps();
                    auto msg_amps = roboclaw::msg::MotorVoltsAmps();
                    auto msg_vel = roboclaw::msg::EncoderVelocity();
                    auto msg_errs = roboclaw::msg::EncoderSteps();
                    auto msg_stat = roboclaw::msg::Status();

                    msg_errs.index = r;
                    msg_errs.mot1_enc_steps = errors.first;
                    msg_errs.mot2_enc_steps = errors.second;

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

                    msg_stat.index                      = r;
                    msg_stat.status                     = status;
                    msg_stat.normal                     = (status == 0);
                    msg_stat.estop                      = status & 0x00000001;
                    msg_stat.temp_err                   = status & 0x00000002;
                    msg_stat.temp2_err                  = status & 0x00000004;
                    msg_stat.main_overvolt_err          = status & 0x00000008;
                    msg_stat.logic_overvolt_err         = status & 0x00000010;
                    msg_stat.logic_undervolt_err        = status & 0x00000020;
                    msg_stat.m1_driver_fault            = status & 0x00000040;
                    msg_stat.m2_driver_fault            = status & 0x00000080;
                    msg_stat.m1_speed_err               = status & 0x00000100;
                    msg_stat.m2_speed_err               = status & 0x00000200;
                    msg_stat.m1_posn_err                = status & 0x00000400;
                    msg_stat.m2_posn_err                = status & 0x00000800;
                    msg_stat.m1_current_err             = status & 0x00001000;
                    msg_stat.m2_current_err             = status & 0x00002000;
                    msg_stat.m1_overcurrent_warn        = status & 0x00010000;
                    msg_stat.m2_overcurrent_warn        = status & 0x00020000;
                    msg_stat.main_overvolt_warn         = status & 0x00040000;
                    msg_stat.main_undervolt_warn        = status & 0x00080000;
                    msg_stat.temp_warn                  = status & 0x00100000;
                    msg_stat.temp2_warn                 = status & 0x00200000;
                    msg_stat.s4_trig                    = status & 0x00400000;
                    msg_stat.s5_trig                    = status & 0x00800000;
                    msg_stat.speed_err_limit_warn       = status & 0x01000000;
                    msg_stat.position_err_limit_warn    = status & 0x02000000;

                    mEncodersPub[r]->publish(msg_encs);
                    mVoltsAmpsPub[r]->publish(msg_amps);
                    mVelocityPub[r]->publish(msg_vel);
                    mErrorsPub[r]->publish(msg_errs);
                    mStatusPub[r]->publish(msg_stat);
                }
            }
        }
    }
}
