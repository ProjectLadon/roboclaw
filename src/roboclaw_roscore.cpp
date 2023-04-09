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
#include "rclcpp/subscription_options.hpp"

#include <map>
#include <string>
#include <iostream>
#include <functional>
#include <limits>

using namespace std;
using namespace std::chrono_literals;

namespace roboclaw 
{

    #define POSN_LIMIT_NAMES(i) string limit_param_base = "posn_limits.node" + to_string(i) + ".chan";\
                                string throttle_param_1 = "rate_limit.node" + to_string(i) + ".chan1";\
                                string throttle_param_2 = "rate_limit.node" + to_string(i) + ".chan2";\
                                string limit_param_upper_1 = limit_param_base + "1.upper_limit";\
                                string limit_param_upper_2 = limit_param_base + "2.upper_limit";\
                                string limit_param_lower_1 = limit_param_base + "1.lower_limit";\
                                string limit_param_lower_2 = limit_param_base + "2.lower_limit";\
                                string limit_param_center_1 = limit_param_base + "1.center";\
                                string limit_param_center_2 = limit_param_base + "2.center";

    #define ENABLE_NAMES(i) string enable_param_base = "enable.node" + to_string(i) + ".";\
                            string enable_posn_1 = enable_param_base + "position.chan1";\
                            string enable_posn_2 = enable_param_base + "position.chan2";\
                            string enable_posn_both = enable_param_base + "position.both";\
                            string enable_velocity_1 = enable_param_base + "velocity.chan1";\
                            string enable_velocity_2 = enable_param_base + "velocity.chan2";\
                            string enable_velocity_both = enable_param_base + "velocity.both";\
                            string enable_duty_1 = enable_param_base + "duty.chan1";\
                            string enable_duty_2 = enable_param_base + "duty.chan2";

    #define PID_PARAM_NAMES(i, j)   string pid_enable_param_base = "pid.node" + to_string(i) + ".chan" + to_string(j);\
                                    string pid_enable_position_base = pid_enable_param_base + ".pos";\
                                    string pid_enable_velocity_base = pid_enable_param_base + ".vel";\
                                    string pid_position_enable = pid_enable_position_base + ".enable";\
                                    string pid_velocity_enable = pid_enable_velocity_base + ".enable";\
                                    string pid_pos_p = pid_enable_position_base + ".p";\
                                    string pid_pos_i = pid_enable_position_base + ".i";\
                                    string pid_pos_d = pid_enable_position_base + ".d";\
                                    string pid_pos_max_i = pid_enable_position_base + ".max_i";\
                                    string pid_pos_deadzone = pid_enable_position_base + ".deadzone";\
                                    string pid_pos_max = pid_enable_position_base + ".max";\
                                    string pid_pos_min = pid_enable_position_base + ".min";\
                                    string pid_vel_p = pid_enable_velocity_base + ".p";\
                                    string pid_vel_i = pid_enable_velocity_base + ".i";\
                                    string pid_vel_d = pid_enable_velocity_base + ".d";\
                                    string pid_vel_qpps = pid_enable_velocity_base + ".qpps";\
                                    string current_limit_enable = "current_limit.node" + to_string(i) + ".chan" + to_string(j) + ".enable";\
                                    string current_limit_value = "current_limit.node" + to_string(i) + ".chan" + to_string(j) + ".limit"; 

    RoboclawCore::RoboclawCore(string name) : Node(name)
    {
        declare_core_params();
        fetch_core_params();
        fetch_position_limits();
        create_services();
        create_publishers();
        create_subscribers();
        create_timers();
        create_pid_params();

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

    void RoboclawCore::declare_core_params()
    {
        this->declare_parameter<std::string>("serial_port", "");    // serial device name
        this->declare_parameter<int64_t>("baudrate", driver::DEFAULT_BAUDRATE);   // default baud rate
        this->declare_parameter<int32_t>("num_claws", 1);       // Number of roboclaws connected
        this->declare_parameter<int32_t>("timeout_ms", driver::DEFAULT_TIMEOUT_MS);       // Serial timeout, in milliseconds
        this->declare_parameter<float>("posn_hz", -1.0);    // frequency of position fetching
        this->declare_parameter<float>("posn_err_hz", -1.0);    // frequency of position error fetching
        this->declare_parameter<float>("velocity_hz", -1.0);    // frequency of velocity fetching
        this->declare_parameter<float>("status_hz", -1.0);    // frequency of status fetching
        this->declare_parameter<float>("volt_amp_hz", -1.0);    // frequency of volt/amp fetching
        this->declare_parameter<float>("pwm_hz", -1.0);    // frequency of motor PWM fetching
        this->declare_parameter<bool>("statistics_enable", false);   // enable/disable statistics

        this->get_parameter("num_claws", mClawCnt);
        for (int i = 0; i < mClawCnt; i++)
        {
            ENABLE_NAMES(i);
            this->declare_parameter<bool>(enable_posn_1, true);
            this->declare_parameter<bool>(enable_posn_2, true);
            this->declare_parameter<bool>(enable_posn_both, true);
            this->declare_parameter<bool>(enable_velocity_1, true);
            this->declare_parameter<bool>(enable_velocity_2, true);
            this->declare_parameter<bool>(enable_velocity_both, true);
            this->declare_parameter<bool>(enable_duty_1, true);
            this->declare_parameter<bool>(enable_duty_2, true);
            POSN_LIMIT_NAMES(i);
            this->declare_parameter<int32_t>(limit_param_upper_1, numeric_limits<int32_t>::max());
            this->declare_parameter<int32_t>(limit_param_upper_2, numeric_limits<int32_t>::max());
            this->declare_parameter<int32_t>(limit_param_lower_1, numeric_limits<int32_t>::min());
            this->declare_parameter<int32_t>(limit_param_lower_2, numeric_limits<int32_t>::min());
            this->declare_parameter<int32_t>(limit_param_center_1, 0);
            this->declare_parameter<int32_t>(limit_param_center_2, 0);
            this->declare_parameter<uint8_t>(throttle_param_1, 0);
            this->declare_parameter<uint8_t>(throttle_param_2, 0);
        }

        mParamSub = std::make_shared<rclcpp::ParameterEventHandler>(this);
    }

    void RoboclawCore::fetch_core_params()
    {
        // Declare a couple of variables...
        std::string serial_port;
        int64_t baudrate = driver::DEFAULT_BAUDRATE;

        // Read the parameters in, throw and error if the serial port name is short
        this->get_parameter("serial_port", serial_port);
        if(serial_port.size() < 1)
        {
            throw std::runtime_error("Must specify serial port");
        }
        this->get_parameter("baudrate", baudrate);
        this->get_parameter("timeout_ms", mTimeoutMs);

        // initialize the driver
        mRoboclaw = new driver(serial_port, baudrate, mTimeoutMs, this);
    }

    void RoboclawCore::fetch_position_limits()
    {
        // get position limits
        for (int i = 0; i < mClawCnt; i++)
        {
            POSN_LIMIT_NAMES(i);
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
    }

    void RoboclawCore::create_services()
    {
        RCLCPP_INFO(this->get_logger(),  "Creating roboclaw services");
        mGetPosPIDSrv = this->create_service<roboclaw::srv::GetPositionPid>("~/get_position_pid", 
            function<void(const shared_ptr<roboclaw::srv::GetPositionPid::Request>,
            shared_ptr<roboclaw::srv::GetPositionPid::Response>)>(
            bind<void>(&RoboclawCore::get_posn_pid_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mGetVelPIDSrv = this->create_service<roboclaw::srv::GetVelocityPid>("~/get_velocity_pid", 
            function<void(const shared_ptr<roboclaw::srv::GetVelocityPid::Request>,
            shared_ptr<roboclaw::srv::GetVelocityPid::Response>)>(
            bind<void>(&RoboclawCore::get_vel_pid_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mSetPosPIDSrv = this->create_service<roboclaw::srv::SetPositionPid>("~/set_position_pid", 
            function<void(const shared_ptr<roboclaw::srv::SetPositionPid::Request>,
            shared_ptr<roboclaw::srv::SetPositionPid::Response>)>(
            bind<void>(&RoboclawCore::set_posn_pid_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mSetVelPIDSrv = this->create_service<roboclaw::srv::SetVelocityPid>("~/set_velocity_pid", 
            function<void(const shared_ptr<roboclaw::srv::SetVelocityPid::Request>,
            shared_ptr<roboclaw::srv::SetVelocityPid::Response>)>(
            bind<void>(&RoboclawCore::set_vel_pid_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mResetEncoderSrv = this->create_service<roboclaw::srv::ResetEncoder>("~/reset_encoder", 
            function<void(const shared_ptr<roboclaw::srv::ResetEncoder::Request>,
            shared_ptr<roboclaw::srv::ResetEncoder::Response>)>(
            bind<void>(&RoboclawCore::reset_encoder_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mResetMotorSrv = this->create_service<roboclaw::srv::ResetMotor>("~/reset_motor", 
            function<void(const shared_ptr<roboclaw::srv::ResetMotor::Request>,
            shared_ptr<roboclaw::srv::ResetMotor::Response>)>(
            bind<void>(&RoboclawCore::reset_motor_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mWriteEEPROMSrv = this->create_service<roboclaw::srv::WriteEeprom>("~/write_eeprom", 
            function<void(const shared_ptr<roboclaw::srv::WriteEeprom::Request>,
            shared_ptr<roboclaw::srv::WriteEeprom::Response>)>(
            bind<void>(&RoboclawCore::write_eeprom_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mReadEEPROMSrv = this->create_service<roboclaw::srv::ReadEeprom>("~/read_eeprom", 
            function<void(const shared_ptr<roboclaw::srv::ReadEeprom::Request>,
            shared_ptr<roboclaw::srv::ReadEeprom::Response>)>(
            bind<void>(&RoboclawCore::read_eeprom_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mGetCurrentLimitSrv = this->create_service<roboclaw::srv::GetCurrentLimit>("~/get_current_limit", 
            function<void(const shared_ptr<roboclaw::srv::GetCurrentLimit::Request>,
            shared_ptr<roboclaw::srv::GetCurrentLimit::Response>)>(
            bind<void>(&RoboclawCore::get_current_limit_cb, this, std::placeholders::_1, std::placeholders::_2)));
        mSetCurrentLimitSrv = this->create_service<roboclaw::srv::SetCurrentLimit>("~/set_current_limit", 
            function<void(const shared_ptr<roboclaw::srv::SetCurrentLimit::Request>,
            shared_ptr<roboclaw::srv::SetCurrentLimit::Response>)>(
            bind<void>(&RoboclawCore::set_current_limit_cb, this, std::placeholders::_1, std::placeholders::_2)));
    }

    void RoboclawCore::create_publishers()
    {
        for (uint8_t r = 0; r < mClawCnt; r++)
        {
            RCLCPP_INFO(this->get_logger(),  "Creating publishers for node %d", r);
            // create publishers and subscribers
            mEncodersPub.emplace_back(this->create_publisher<roboclaw::msg::EncoderStepsStamped>(
                "~/claw" + to_string(r) + "/posn_out", 10));
            mErrorsPub.emplace_back(this->create_publisher<roboclaw::msg::EncoderStepsStamped>(
                "~/claw" + to_string(r) + "/posn_err", 10));
            mVoltsAmpsPub.emplace_back(this->create_publisher<roboclaw::msg::MotorVoltsAmpsStamped>(
                "~/claw" + to_string(r) + "/volts_amps_out", 10));
            mVelocityPub.emplace_back(this->create_publisher<roboclaw::msg::EncoderVelocityStamped>(
                "~/claw" + to_string(r) + "/velocity_out", 10));
            mStatusPub.emplace_back(this->create_publisher<roboclaw::msg::StatusStamped>(
                "~/claw" + to_string(r) + "/status", 10));
            mPwmPub.emplace_back(this->create_publisher<roboclaw::msg::MotorPwmStamped>(
                "~/claw" + to_string(r) + "/pwm_out", 10));
        }
    }

    void RoboclawCore::create_subscribers()
    {
        for (uint8_t r = 0; r < mClawCnt; r++)
        {
            RCLCPP_INFO(this->get_logger(),  "Creating subscribers for node %d", r);
            ENABLE_NAMES(r);
            auto options = rclcpp::SubscriptionOptions();
            bool stats_enable;
            this->get_parameter("statistics_enable", stats_enable);

            if (stats_enable) { options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable; }

            bool sub;
            this->get_parameter(enable_velocity_both, sub);
            if (sub)
            {
                options.topic_stats_options.publish_topic = "~/claw" + to_string(r) + "/statistics/motor_vel_cmd";
                mVelCmdSub.emplace_back(this->create_subscription<roboclaw::msg::MotorVelocityStamped>(
                    "~/claw" + to_string(r) + "/motor_vel_cmd", 10, 
                    function<void(const roboclaw::msg::MotorVelocityStamped &)>(
                        bind(&RoboclawCore::velocity_callback, this, r, std::placeholders::_1)),
                    options
                ));
            }

            this->get_parameter(enable_velocity_1, sub);
            if (sub)
            {
                options.topic_stats_options.publish_topic = "~/claw" + to_string(r) + "/statistics/motor_vel_single_cmd/chan1";
                mVelCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorVelocitySingleStamped>(
                    "~/claw" + to_string(r) + "/motor_vel_single_cmd/chan1", 10, 
                    function<void(const roboclaw::msg::MotorVelocitySingleStamped &)>(
                        bind(&RoboclawCore::velocity_single_callback, this, r, 1, std::placeholders::_1)),
                    options
                ));
            }

            this->get_parameter(enable_velocity_2, sub);
            if (sub)
            {
                options.topic_stats_options.publish_topic = "~/claw" + to_string(r) + "/statistics/motor_vel_single_cmd/chan2";
                mVelCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorVelocitySingleStamped>(
                    "~/claw" + to_string(r) + "/motor_vel_single_cmd/chan2", 10, 
                    function<void(const roboclaw::msg::MotorVelocitySingleStamped &)>(
                        bind(&RoboclawCore::velocity_single_callback, this, r, 2, std::placeholders::_1)),
                    options
                ));
            }

            this->get_parameter(enable_posn_both, sub);
            if (sub)
            {
                options.topic_stats_options.publish_topic = "~/claw" + to_string(r) + "/statistics/motor_pos_cmd";
                mPosCmdSub.emplace_back(this->create_subscription<roboclaw::msg::MotorPositionStamped>(
                    "~/claw" + to_string(r) + "/motor_pos_cmd", 10, 
                    function<void(const roboclaw::msg::MotorPositionStamped &)>(
                        bind<void>(&RoboclawCore::position_callback, this, r, std::placeholders::_1)),
                    options
                ));
            }

            this->get_parameter(enable_posn_1, sub);
            if (sub)
            {
                options.topic_stats_options.publish_topic = "~/claw" + to_string(r) + "/statistics/motor_pos_single_cmd/chan1";
                mPosCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorPositionSingleStamped>(
                    "~/claw" + to_string(r) + "/motor_pos_single_cmd/chan1", 10,  
                    function<void(const roboclaw::msg::MotorPositionSingleStamped &)>(
                        bind<void>(&RoboclawCore::position_single_callback, this, r, 1, std::placeholders::_1)), 
                    options
                ));
            }

            this->get_parameter(enable_posn_2, sub);
            if (sub)
            {
                options.topic_stats_options.publish_topic = "~/claw" + to_string(r) + "/statistics/motor_pos_single_cmd/chan2";
                mPosCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorPositionSingleStamped>(
                    "~/claw" + to_string(r) + "/motor_pos_single_cmd/chan2", 10,  
                    function<void(const roboclaw::msg::MotorPositionSingleStamped &)>(
                        bind<void>(&RoboclawCore::position_single_callback, this, r, 2, std::placeholders::_1)),
                    options
                ));
            }

            this->get_parameter(enable_duty_1, sub);
            if (sub)
            {
                options.topic_stats_options.publish_topic = "~/claw" + to_string(r) + "/statistics/motor_duty_single_cmd/chan1";
                mDutyCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorDutySingleStamped>(
                    "~/claw" + to_string(r) + "/motor_duty_single_cmd/chan1", 10, 
                    function<void(const roboclaw::msg::MotorDutySingleStamped &)>(
                        bind<void>(&RoboclawCore::duty_single_callback, this, r, 1, std::placeholders::_1)),
                    options
                ));
            }

            this->get_parameter(enable_duty_2, sub);
            if (sub)
            {
                options.topic_stats_options.publish_topic = "~/claw" + to_string(r) + "/statistics/motor_duty_single_cmd/chan2";
                mDutyCmdSingleSub.emplace_back(this->create_subscription<roboclaw::msg::MotorDutySingleStamped>(
                    "~/claw" + to_string(r) + "/motor_duty_single_cmd/chan2", 10, 
                    function<void(const roboclaw::msg::MotorDutySingleStamped &)>(
                        bind<void>(&RoboclawCore::duty_single_callback, this, r, 2, std::placeholders::_1)),
                    options
                ));
            }
        }
    }

    void RoboclawCore::create_timers()
    {
        // create periodic callback to trigger data fetching
        float hz;
        this->get_parameter("posn_hz", hz);
        if (hz > 0.01) { mEncoderTimer = this->create_wall_timer(
            (1000ms/hz), bind(&RoboclawCore::timer_encoder_cb, this)); }
        this->get_parameter("posn_err_hz", hz);
        if (hz > 0.01) { mEncoderErrTimer = this->create_wall_timer(
            (1000ms/hz), bind(&RoboclawCore::timer_encoder_err_cb, this)); }
        this->get_parameter("velocity_hz", hz);
        if (hz > 0.01) { mVelocityTimer = this->create_wall_timer(
            (1000ms/hz), bind(&RoboclawCore::timer_velocity_cb, this)); }
        this->get_parameter("status_hz", hz);
        if (hz > 0.01) { mStatusTimer = this->create_wall_timer(
            (1000ms/hz), bind(&RoboclawCore::timer_status_cb, this)); }
        this->get_parameter("volt_amp_hz", hz);
        if (hz > 0.01) { mVoltAmpTimer = this->create_wall_timer(
            (1000ms/hz), bind(&RoboclawCore::timer_volt_amp_cb, this)); }
        this->get_parameter("pwm_hz", hz);
        if (hz > 0.01) { mPwmTimer = this->create_wall_timer(
            (1000ms/hz), bind(&RoboclawCore::timer_pwm_cb, this)); }
    }

    void RoboclawCore::create_pid_params()
    {
        for (int i = 0; i < mClawCnt; i++)  
        {
            for (int j = 1; j < 3; j++)
            {
                PID_PARAM_NAMES(i, j);
                this->declare_parameter<bool>(pid_position_enable, false);
                this->declare_parameter<bool>(pid_velocity_enable, false);
                this->declare_parameter<bool>(current_limit_enable, false);
                bool pos_enb, vel_enb, curr_enb;
                this->get_parameter(pid_position_enable, pos_enb);
                this->get_parameter(pid_velocity_enable, vel_enb);
                this->get_parameter(current_limit_enable, curr_enb);
                if (pos_enb)
                {
                    this->declare_parameter<float>(pid_pos_p, 0.0);
                    this->declare_parameter<float>(pid_pos_i, 0.0);
                    this->declare_parameter<float>(pid_pos_d, 0.0);
                    this->declare_parameter<float>(pid_pos_max_i, 0.0);
                    this->declare_parameter<int32_t>(pid_pos_deadzone, 0);
                    this->declare_parameter<int32_t>(pid_pos_max, numeric_limits<int32_t>::max());
                    this->declare_parameter<int32_t>(pid_pos_min, numeric_limits<int32_t>::min());
                    set_pos_pid_from_params(i, j);
                    create_pos_pid_callbacks(i, j);
                }
                if (vel_enb)
                {
                    this->declare_parameter<float>(pid_vel_p, 0.0);
                    this->declare_parameter<float>(pid_vel_i, 0.0);
                    this->declare_parameter<float>(pid_vel_d, 0.0);
                    this->declare_parameter<int32_t>(pid_vel_qpps, 0);
                    set_vel_pid_from_params(i, j);
                    create_vel_pid_callbacks(i, j);
                }
                if (curr_enb)
                {
                    this->declare_parameter<float>(current_limit_value, -1.0);
                    set_current_limit_from_params(i,j);
                    create_current_limit_callbacks(i,j);
                }
            }
        }
    }

    void RoboclawCore::create_pos_pid_callbacks(uint8_t node, uint8_t channel)
    {
        PID_PARAM_NAMES(node, channel);
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_pos_p, bind(&RoboclawCore::position_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_pos_i, bind(&RoboclawCore::position_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_pos_d, bind(&RoboclawCore::position_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_pos_max_i, bind(&RoboclawCore::position_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_pos_deadzone, bind(&RoboclawCore::position_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_pos_max, bind(&RoboclawCore::position_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_pos_min, bind(&RoboclawCore::position_pid_cb, 
                this, node, channel, std::placeholders::_1)));
    }

    void RoboclawCore::create_vel_pid_callbacks(uint8_t node, uint8_t channel)
    {
        PID_PARAM_NAMES(node, channel);
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_vel_p, bind(&RoboclawCore::velocity_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_vel_i, bind(&RoboclawCore::velocity_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_vel_d, bind(&RoboclawCore::velocity_pid_cb, 
                this, node, channel, std::placeholders::_1)));
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            pid_vel_qpps, bind(&RoboclawCore::velocity_pid_cb, 
                this, node, channel, std::placeholders::_1)));
    }

    void RoboclawCore::create_current_limit_callbacks(uint8_t node, uint8_t channel)
    {
        PID_PARAM_NAMES(node, channel);
        mParamCBHandle.emplace_back(mParamSub->add_parameter_callback(
            current_limit_value, bind(&RoboclawCore::current_limit_cb, 
                this, node, channel, std::placeholders::_1)));
    }
        
    void RoboclawCore::set_pos_pid_from_params(uint8_t node, uint8_t channel)
    {
        PID_PARAM_NAMES(node, channel);
        position_pid_t k;
        this->get_parameter(pid_pos_p, k.p);
        this->get_parameter(pid_pos_i, k.i);
        this->get_parameter(pid_pos_d, k.d);
        this->get_parameter(pid_pos_max_i, k.max_i);
        this->get_parameter(pid_pos_deadzone, k.deadzone);
        this->get_parameter(pid_pos_max, k.max_pos);
        this->get_parameter(pid_pos_min, k.min_pos);
        mRoboclaw->set_position_pid(driver::BASE_ADDRESS + node, channel, k);
    }
        
    void RoboclawCore::set_vel_pid_from_params(uint8_t node, uint8_t channel)
    {
        PID_PARAM_NAMES(node, channel);
        velocity_pid_t k;
        this->get_parameter(pid_vel_p, k.p);
        this->get_parameter(pid_vel_i, k.i);
        this->get_parameter(pid_vel_d, k.d);
        this->get_parameter(pid_vel_qpps, k.qpps);
        mRoboclaw->set_velocity_pid(driver::BASE_ADDRESS + node, channel, k);
    }
    
    void RoboclawCore::set_current_limit_from_params(uint8_t node, uint8_t channel)
    {
        PID_PARAM_NAMES(node, channel);
        float limit;
        this->get_parameter(current_limit_value, limit);
        if (limit > 0.0) { mRoboclaw->set_current_limit(driver::BASE_ADDRESS + node, channel, limit); }
    }
    
    void RoboclawCore::velocity_pid_cb(uint8_t node, uint8_t channel, const rclcpp::Parameter &p)
    {
        RCLCPP_INFO(
            this->get_logger(), 
            "Updating node %d channel %d velocity pid from parameter %s",
            node, channel, p.get_name().c_str());
        set_vel_pid_from_params(node, channel);
    }
        
    void RoboclawCore::position_pid_cb(uint8_t node, uint8_t channel, const rclcpp::Parameter &p)
    {
        RCLCPP_INFO(
            this->get_logger(), 
            "Updating node %d channel %d position pid from parameter %s",
            node, channel, p.get_name().c_str());
        set_pos_pid_from_params(node, channel);
    }    

    void RoboclawCore::current_limit_cb(uint8_t node, uint8_t channel, const rclcpp::Parameter &p)
    {
        RCLCPP_INFO(
            this->get_logger(), 
            "Updating node %d channel %d current limit from parameter %s",
            node, channel, p.get_name().c_str());
        set_current_limit_from_params(node, channel);
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

    void RoboclawCore::duty_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorDutySingleStamped &msg) 
    {
        // RCLCPP_INFO(this->get_logger(), "Received a single motor duty command");
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

    void RoboclawCore::velocity_callback(uint8_t idx, const roboclaw::msg::MotorVelocityStamped &msg) 
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

    void RoboclawCore::velocity_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorVelocitySingleStamped &msg) 
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

    void RoboclawCore::position_callback(uint8_t idx, const roboclaw::msg::MotorPositionStamped &msg) 
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

    void RoboclawCore::position_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorPositionSingleStamped &msg) 
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

    void RoboclawCore::get_posn_pid_cb(const shared_ptr<roboclaw::srv::GetPositionPid::Request> request,
        shared_ptr<roboclaw::srv::GetPositionPid::Response> response)
    {
        if(bad_inputs(request->node, request->channel)) 
        {
            response->success = false;
            response->error = "Bad node and/or channel spec";
            return;
        }
        mRoboclaw->read_position_pid(driver::BASE_ADDRESS + request->node, request->channel);
        position_pid_t pid;
        uint32_t cnt = 0;
        while ((cnt < mTimeoutMs ) and (!mRoboclaw->get_position_pid(driver::BASE_ADDRESS + request->node, request->channel, pid)))
        { 
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            cnt++;
        }
        if (cnt >= mTimeoutMs)
        {
            response->success = false;
            response->error = "Request timed out";
            return;
        }
        response->success = true;
        response->p = pid.p;
        response->i = pid.i;
        response->d = pid.d;
        response->max_i = pid.max_i;
        response->deadzone = pid.deadzone;
        response->max_pos = pid.max_pos;
        response->min_pos = pid.min_pos;
    }

    void RoboclawCore::get_vel_pid_cb(const shared_ptr<roboclaw::srv::GetVelocityPid::Request> request,
        shared_ptr<roboclaw::srv::GetVelocityPid::Response> response)
    {
        if(bad_inputs(request->node, request->channel)) 
        {
            response->success = false;
            response->error = "Bad node and/or channel spec";
            return;
        }
        mRoboclaw->read_velocity_pid(driver::BASE_ADDRESS + request->node, request->channel);
        velocity_pid_t pid;
        uint32_t cnt = 0;
        while ((cnt < mTimeoutMs ) and (!mRoboclaw->get_velocity_pid(driver::BASE_ADDRESS + request->node, request->channel, pid)))
        { 
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            cnt++;
        }
        if (cnt >= mTimeoutMs)
        {
            response->success = false;
            response->error = "Request timed out";
            return;
        }
        response->success = true;
        response->p = pid.p;
        response->i = pid.i;
        response->d = pid.d;
        response->qpps = pid.qpps;
    }

    void RoboclawCore::set_posn_pid_cb(const shared_ptr<roboclaw::srv::SetPositionPid::Request> request,
        shared_ptr<roboclaw::srv::SetPositionPid::Response> response)
    {
        if(bad_inputs(request->node, request->channel)) 
        {
            response->success = false;
            response->error = "Bad node and/or channel spec";
            return;
        }
        position_pid_t pid;
        pid.p = request->p;
        pid.i = request->i;
        pid.d = request->d;
        pid.max_i = request->max_i;
        pid.deadzone = request->deadzone;
        pid.max_pos = request->max_pos;
        pid.min_pos = request->min_pos;
        mRoboclaw->set_position_pid(driver::BASE_ADDRESS + request->node, request->channel, pid);

        response->success = true;
    }

    void RoboclawCore::set_vel_pid_cb(const shared_ptr<roboclaw::srv::SetVelocityPid::Request> request,
        shared_ptr<roboclaw::srv::SetVelocityPid::Response> response)
    {
        if(bad_inputs(request->node, request->channel)) 
        {
            response->success = false;
            response->error = "Bad node and/or channel spec";
            return;
        }
        velocity_pid_t pid;
        pid.p = request->p;
        pid.i = request->i;
        pid.d = request->d;
        pid.qpps = request->qpps;
        mRoboclaw->set_velocity_pid(driver::BASE_ADDRESS + request->node, request->channel, pid);

        response->success = true;
    }

    void RoboclawCore::reset_encoder_cb(const shared_ptr<roboclaw::srv::ResetEncoder::Request> request,
        shared_ptr<roboclaw::srv::ResetEncoder::Response> response)
    {
        if(bad_inputs(request->node, request->channel)) 
        {
            response->success = false;
            response->error = "Bad node and/or channel spec";
            return;
        }
        mRoboclaw->reset_encoder(driver::BASE_ADDRESS + request->node, request->channel, request->value);

        response->success = true;
        return;
    }

    void RoboclawCore::reset_motor_cb(const shared_ptr<roboclaw::srv::ResetMotor::Request> request,
        shared_ptr<roboclaw::srv::ResetMotor::Response> response)
    {
        if(bad_inputs(request->node, request->channel)) 
        {
            response->success = false;
            response->error = "Bad node and/or channel spec";
            return;
        }
        mRoboclaw->set_duty_single(driver::BASE_ADDRESS + request->node, request->channel, request->pwm_value);

        response->success = true;
        return;

    }
    
    void RoboclawCore::read_eeprom_cb(const shared_ptr<roboclaw::srv::ReadEeprom::Request> request,
        shared_ptr<roboclaw::srv::ReadEeprom::Response> response)
    {
        if(bad_inputs(request->node, 1)) 
        {
            response->success = false;
            return;
        }
        mRoboclaw->read_eeprom(driver::BASE_ADDRESS + request->node);

        response->success = true;
        return;

    }
    
    void RoboclawCore::write_eeprom_cb(const shared_ptr<roboclaw::srv::WriteEeprom::Request> request,
        shared_ptr<roboclaw::srv::WriteEeprom::Response> response)
    {
        if(bad_inputs(request->node, 1)) 
        {
            response->success = false;
            return;
        }

        return;

    }
    
    void RoboclawCore::get_current_limit_cb(const shared_ptr<roboclaw::srv::GetCurrentLimit::Request> request,
        shared_ptr<roboclaw::srv::GetCurrentLimit::Response> response)
    {
        if(bad_inputs(request->node, request->channel)) 
        {
            response->success = false;
            response->error = "Bad node and/or channel spec";
            return;
        }
        mRoboclaw->read_current_limit((driver::BASE_ADDRESS + request->node), request->channel);
        float limit;
        uint32_t cnt = 0;
        while ((cnt < mTimeoutMs ) and (!mRoboclaw->get_current_limit((driver::BASE_ADDRESS + request->node), request->channel, limit)))
        { 
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            cnt++;
        }
        if (cnt >= mTimeoutMs)
        {
            response->success = false;
            response->error = "Request timed out";
            return;
        }
        response->success = true;
        response->max_current = limit;
    }
    
    void RoboclawCore::set_current_limit_cb(const shared_ptr<roboclaw::srv::SetCurrentLimit::Request> request,
        shared_ptr<roboclaw::srv::SetCurrentLimit::Response> response)
    {
        if(bad_inputs(request->node, request->channel)) 
        {
            response->success = false;
            response->error = "Bad node and/or channel spec";
            return;
        }
        mRoboclaw->set_current_limit((driver::BASE_ADDRESS + request->node), request->channel, request->max_current);
        response->success = true;
    }

    void RoboclawCore::timer_encoder_cb() 
    {

        // Request data
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping position feedback");
            return;
        }
        for (int r = 0; r < mClawCnt; r++) 
        {
            mRoboclaw->read_encoders(driver::BASE_ADDRESS + r);
        }
    }
    
    void RoboclawCore::timer_velocity_cb() 
    {

        // Request data
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping velocity feedback");
            return;
        }
        for (int r = 0; r < mClawCnt; r++) 
        {
            mRoboclaw->read_velocity(driver::BASE_ADDRESS + r);
        }
    }

    void RoboclawCore::timer_encoder_err_cb() 
    {

        // Request data
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping position error feedback");
            return;
        }
        for (int r = 0; r < mClawCnt; r++) 
        {
            mRoboclaw->read_position_errors(driver::BASE_ADDRESS + r);
        }
    }
 
    void RoboclawCore::timer_status_cb() 
    {

        // Request data
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping status feedback");
            return;
        }
        for (int r = 0; r < mClawCnt; r++) 
        {
            mRoboclaw->read_status(driver::BASE_ADDRESS + r);
        }
    }

    void RoboclawCore::timer_volt_amp_cb() 
    {
        // Request data
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping current feedback");
            return;
        }
        for (int r = 0; r < mClawCnt; r++) 
        {
            mRoboclaw->read_volt_current(driver::BASE_ADDRESS + r);
        }
    }

    void RoboclawCore::timer_pwm_cb() 
    {
        // Request data
        if (mRoboclaw->is_queue_flooded())
        {
            RCLCPP_INFO(this->get_logger(), "Command queue flooded; skipping PWM feedback");
            return;
        }
        for (int r = 0; r < mClawCnt; r++) 
        {
            mRoboclaw->read_motor_pwm(driver::BASE_ADDRESS + r);
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
                pair<int16_t, int16_t> pwm;
                uint32_t status;
                if (mRoboclaw->get_encoders((driver::BASE_ADDRESS + r), encoders))
                {
                    auto msg_encs = roboclaw::msg::EncoderStepsStamped();
                    mRoboclaw->clear_encoders_ready(driver::BASE_ADDRESS + r);
                    msg_encs.header.stamp = this->get_clock()->now();
                    msg_encs.index = r;
                    msg_encs.mot1_enc_steps = encoders.first;
                    msg_encs.mot2_enc_steps = encoders.second;
                    mEncodersPub[r]->publish(msg_encs);
                }
                if (mRoboclaw->get_velocity((driver::BASE_ADDRESS + r), velocity))
                {
                    auto msg_vel = roboclaw::msg::EncoderVelocityStamped();
                    mRoboclaw->clear_velocities_ready(driver::BASE_ADDRESS + r);
                    msg_vel.index = r;
                    msg_vel.header.stamp = this->get_clock()->now();
                    msg_vel.mot1_enc_vel = velocity.first;
                    msg_vel.mot2_enc_vel = velocity.second;
                    mVelocityPub[r]->publish(msg_vel);
                }
                if (mRoboclaw->get_position_errors((driver::BASE_ADDRESS + r), errors))
                {
                    auto msg_errs = roboclaw::msg::EncoderStepsStamped();
                    mRoboclaw->clear_position_errors_ready(driver::BASE_ADDRESS + r);
                    msg_errs.index = r;
                    msg_errs.header.stamp = this->get_clock()->now();
                    msg_errs.mot1_enc_steps = errors.first;
                    msg_errs.mot2_enc_steps = errors.second;
                    mErrorsPub[r]->publish(msg_errs);
                }
                if (mRoboclaw->get_status((driver::BASE_ADDRESS + r), status))
                {
                    auto msg_stat = roboclaw::msg::StatusStamped();
                    mRoboclaw->clear_status_ready(driver::BASE_ADDRESS + r);
                    msg_stat.header.stamp               = this->get_clock()->now();
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
                    mStatusPub[r]->publish(msg_stat);
                }
                if (mRoboclaw->get_logic_voltage((driver::BASE_ADDRESS + r), logicV)
                    and mRoboclaw->get_motor_voltage((driver::BASE_ADDRESS + r), motorV)
                    and mRoboclaw->get_motor_current((driver::BASE_ADDRESS + r), currents)
                ) {
                    mRoboclaw->clear_logic_volt_ready(driver::BASE_ADDRESS + r);
                    mRoboclaw->clear_motor_volt_ready(driver::BASE_ADDRESS + r);
                    mRoboclaw->clear_motor_amps_ready(driver::BASE_ADDRESS + r);
                    auto msg_amps = roboclaw::msg::MotorVoltsAmpsStamped();

                    msg_amps.header.stamp = this->get_clock()->now();
                    msg_amps.index = r;
                    msg_amps.motor_volts = motorV;
                    msg_amps.logic_volts = logicV;
                    msg_amps.mot1_amps = currents.first;
                    msg_amps.mot2_amps = currents.second;
                    mVoltsAmpsPub[r]->publish(msg_amps);
                }
                if (mRoboclaw->get_motor_pwm((driver::BASE_ADDRESS + r), pwm))
                {
                    mRoboclaw->clear_motor_pwm_ready(driver::BASE_ADDRESS + r);
                    auto msg_pwm = roboclaw::msg::MotorPwmStamped();
                    msg_pwm.header.stamp = this->get_clock()->now();
                    msg_pwm.index = r;
                    msg_pwm.mot1_pwm = pwm.first;
                    msg_pwm.mot2_pwm = pwm.second;
                    mPwmPub[r]->publish(msg_pwm);
                }
            }
        }
    }
}
