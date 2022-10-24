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

#ifndef PROJECT_ROBOCLAW_ROSCORE_H
#define PROJECT_ROBOCLAW_ROSCORE_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"

#include "roboclaw_driver.h"

#include "roboclaw/msg/encoder_steps.hpp"
#include "roboclaw/msg/motor_duty_single.hpp"
#include "roboclaw/msg/motor_velocity.hpp"
#include "roboclaw/msg/motor_velocity_single.hpp"
#include "roboclaw/msg/motor_position.hpp"
#include "roboclaw/msg/motor_position_single.hpp"
#include "roboclaw/msg/motor_volts_amps.hpp"
#include "roboclaw/msg/encoder_velocity.hpp"
#include "roboclaw/msg/status.hpp"
#include "roboclaw/srv/get_position_pid.hpp"
#include "roboclaw/srv/set_position_pid.hpp"
#include "roboclaw/srv/get_velocity_pid.hpp"
#include "roboclaw/srv/set_velocity_pid.hpp"
#include "roboclaw/srv/reset_encoder.hpp"
#include "roboclaw/srv/reset_motor.hpp"

using namespace std::chrono_literals;
using namespace std;

namespace roboclaw {

    typedef tuple<int32_t, int32_t, int32_t>  position_limits_center_t;
    typedef tuple<position_limits_center_t, position_limits_center_t> node_posn_limits_t;

    class RoboclawCore : public rclcpp::Node 
    {
    public:
        RoboclawCore(string name);
        ~RoboclawCore();

    private:
        driver *mRoboclaw;
        uint8_t mClawCnt;

        unique_ptr<thread> mPubWorkerThread;
        bool mRunEnable;

        // position and other limits
        vector<node_posn_limits_t> mPosnLimits;
        vector<pair<uint8_t, uint8_t>> mCmdThrottleLimit;
        vector<pair<uint8_t, uint8_t>> mCmdThrottleCounter;
        uint32_t mTimeoutMs;

        // publishers
        vector<rclcpp::Publisher<roboclaw::msg::EncoderSteps>::SharedPtr> mEncodersPub;
        vector<rclcpp::Publisher<roboclaw::msg::EncoderSteps>::SharedPtr> mErrorsPub;
        vector<rclcpp::Publisher<roboclaw::msg::EncoderVelocity>::SharedPtr> mVelocityPub;
        vector<rclcpp::Publisher<roboclaw::msg::MotorVoltsAmps>::SharedPtr> mVoltsAmpsPub;
        vector<rclcpp::Publisher<roboclaw::msg::Status>::SharedPtr> mStatusPub;

        // subscribers
        vector<rclcpp::Subscription<roboclaw::msg::MotorDutySingle>::SharedPtr> mDutyCmdSingleSub;
        vector<rclcpp::Subscription<roboclaw::msg::MotorVelocity>::SharedPtr> mVelCmdSub;
        vector<rclcpp::Subscription<roboclaw::msg::MotorVelocitySingle>::SharedPtr> mVelCmdSingleSub;
        vector<rclcpp::Subscription<roboclaw::msg::MotorPosition>::SharedPtr> mPosCmdSub;
        vector<rclcpp::Subscription<roboclaw::msg::MotorPositionSingle>::SharedPtr> mPosCmdSingleSub;

        // services
        rclcpp::Service<roboclaw::srv::GetPositionPid>::SharedPtr   mGetPosPIDSrv;
        rclcpp::Service<roboclaw::srv::GetVelocityPid>::SharedPtr   mGetVelPIDSrv;
        rclcpp::Service<roboclaw::srv::SetPositionPid>::SharedPtr   mSetPosPIDSrv;
        rclcpp::Service<roboclaw::srv::SetVelocityPid>::SharedPtr   mSetVelPIDSrv;
        rclcpp::Service<roboclaw::srv::ResetEncoder>::SharedPtr     mResetEncoderSrv;
        rclcpp::Service<roboclaw::srv::ResetMotor>::SharedPtr       mResetMotorSrv;

        rclcpp::TimerBase::SharedPtr mEncoderTimer;
        rclcpp::TimerBase::SharedPtr mEncoderErrTimer;
        rclcpp::TimerBase::SharedPtr mVelocityTimer;
        rclcpp::TimerBase::SharedPtr mStatusTimer;
        rclcpp::TimerBase::SharedPtr mVoltAmpTimer;

        // subscriber callbacks
        void duty_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorDutySingle &msg);
        void velocity_callback(uint8_t idx, const roboclaw::msg::MotorVelocity &msg);
        void velocity_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorVelocitySingle &msg);
        void position_callback(uint8_t idx, const roboclaw::msg::MotorPosition &msg);
        void position_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorPositionSingle &msg);
        
        // service callbacks
        void get_posn_pid_cb(
            const shared_ptr<roboclaw::srv::GetPositionPid::Request> request,
            shared_ptr<roboclaw::srv::GetPositionPid::Response> response);
        void get_vel_pid_cb(
            const shared_ptr<roboclaw::srv::GetVelocityPid::Request> request,
            shared_ptr<roboclaw::srv::GetVelocityPid::Response> response);
        void set_posn_pid_cb(
            const shared_ptr<roboclaw::srv::SetPositionPid::Request> request,
            shared_ptr<roboclaw::srv::SetPositionPid::Response> response);
        void set_vel_pid_cb(
            const shared_ptr<roboclaw::srv::SetVelocityPid::Request> request,
            shared_ptr<roboclaw::srv::SetVelocityPid::Response> response);
        void reset_encoder_cb(
            const shared_ptr<roboclaw::srv::ResetEncoder::Request> request,
            shared_ptr<roboclaw::srv::ResetEncoder::Response> response);
        void reset_motor_cb(
            const shared_ptr<roboclaw::srv::ResetMotor::Request> request,
            shared_ptr<roboclaw::srv::ResetMotor::Response> response);

        // timer callback & thread workers
        void timer_encoder_cb();
        void timer_velocity_cb();
        void timer_encoder_err_cb();
        void timer_status_cb();
        void timer_volt_amp_cb();
        void pub_worker();

        // utility functions
        uint32_t get_max_posn(position_limits_center_t t) { return(get<0>(t)); }
        uint32_t get_min_posn(position_limits_center_t t) { return(get<1>(t)); }
        uint32_t get_center_posn(position_limits_center_t t) { return(get<2>(t)); }
        position_limits_center_t make_limits(uint32_t max, uint32_t min, uint32_t center) 
        {
            return make_tuple(max, min, center);
        }
        int32_t bound_input(int32_t val, int32_t max, int32_t min) { return(std::max(std::min(val, max), min)); }
        bool is_throttle_clear(uint8_t idx, uint8_t chan);
        bool bad_inputs(uint8_t idx, uint8_t chan);

    };


}

#endif //PROJECT_ROBOCLAW_ROSCORE_H
