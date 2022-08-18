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

using namespace std::chrono_literals;
using namespace std;

namespace roboclaw {

    class RoboclawCore : public rclcpp::Node 
    {
    public:
        RoboclawCore(string name);
        ~RoboclawCore();

    private:
        driver *mRoboclaw;
        uint8_t mClawCnt;

        unique_ptr<thread> mPubWorkerThread;
        unique_ptr<vector<atomic<bool>>> mDataArrived;
        bool mRunEnable;

        vector<rclcpp::Publisher<roboclaw::msg::EncoderSteps>::SharedPtr> mEncodersPub;
        vector<rclcpp::Publisher<roboclaw::msg::EncoderVelocity>::SharedPtr> mVelocityPub;
        vector<rclcpp::Publisher<roboclaw::msg::MotorVoltsAmps>::SharedPtr> mVoltsAmpsPub;

        vector<rclcpp::Subscription<roboclaw::msg::MotorDutySingle>::SharedPtr> mDutyCmdSingleSub;
        vector<rclcpp::Subscription<roboclaw::msg::MotorVelocity>::SharedPtr> mVelCmdSub;
        vector<rclcpp::Subscription<roboclaw::msg::MotorVelocitySingle>::SharedPtr> mVelCmdSingleSub;
        vector<rclcpp::Subscription<roboclaw::msg::MotorPosition>::SharedPtr> mPosCmdSub;
        vector<rclcpp::Subscription<roboclaw::msg::MotorPositionSingle>::SharedPtr> mPosCmdSingleSub;

        rclcpp::TimerBase::SharedPtr mPubTimer;

        void duty_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorDutySingle &msg);
        void velocity_callback(uint8_t idx, const roboclaw::msg::MotorVelocity &msg);
        void velocity_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorVelocitySingle &msg);
        void position_callback(uint8_t idx, const roboclaw::msg::MotorPosition &msg);
        void position_single_callback(uint8_t idx, uint8_t chan, const roboclaw::msg::MotorPositionSingle &msg);
        bool bad_inputs(uint8_t idx, uint8_t chan);
        void timer_callback();
        void pub_worker();

    };


}

#endif //PROJECT_ROBOCLAW_ROSCORE_H
