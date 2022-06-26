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

#include "rclcpp/rclcpp.hpp"

#include "roboclaw_driver.h"

#include "roboclaw/msg/encoder_steps.hpp"
#include "roboclaw/msg/motor_velocity.hpp"
#include "roboclaw/msg/motor_position.hpp"
#include "roboclaw/msg/motor_position_single.hpp"
#include "roboclaw/msg/motor_volts_amps.hpp"

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
        uint8_t mOpenCnt;

        rclcpp::Publisher<roboclaw::msg::EncoderSteps>::SharedPtr mEncodersPub;
        rclcpp::Publisher<roboclaw::msg::MotorVoltsAmps>::SharedPtr mVoltsAmpsPub;

        rclcpp::Subscription<roboclaw::msg::MotorVelocity>::SharedPtr mVelCmdSub;
        rclcpp::Subscription<roboclaw::msg::MotorPosition>::SharedPtr mPosCmdSub;
        rclcpp::Subscription<roboclaw::msg::MotorPositionSingle>::SharedPtr mPosCmdSingleSub;

        rclcpp::TimerBase::SharedPtr mPubTimer;

        // rclcpp::Time mLastVelCmd;
        // rclcpp::Time mLastPosCmd;

        void velocity_callback(const roboclaw::msg::MotorVelocity &msg);
        void position_callback(const roboclaw::msg::MotorPosition &msg);
        void position_single_callback(const roboclaw::msg::MotorPositionSingle &msg);
        void timer_callback();
    };


}

#endif //PROJECT_ROBOCLAW_ROSCORE_H
