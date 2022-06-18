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

#ifndef PROJECT_DIFFDRIVE_ROSCORE_H
#define PROJECT_DIFFDRIVE_ROSCORE_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "roboclaw/msg/encoder_steps.hpp"
#include "roboclaw/msg/motor_velocity.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/quaternion.hpp"

using namespace std;

namespace roboclaw {

    class DiffDriveCore : public rclcpp::Node 
    {
    public:
        DiffDriveCore(string name);
        ~DiffDriveCore() {};

    private:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomPub;
        rclcpp::Publisher<roboclaw::msg::MotorVelocity>::SharedPtr mMotorPub;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mTwistSub;
        rclcpp::Subscription<roboclaw::msg::EncoderSteps>::SharedPtr mEncSub;
        
        int mLastSteps1;
        int mLastSteps2;

        double mLastX;
        double mLastY;
        double mLastTheta;

        double mBaseWidth;
        double mStepsPerMeter;

        bool mOpenLoop;
        bool mSwapMotors;
        bool mInvertMotor1;
        bool mInvertMotor2;

        int8_t mTargetIndex;

        double mVarPosX;
        double mVarPosY;
        double mVarPosTheta;

        void twist_callback(const geometry_msgs::msg::Twist &msg);
        void encoder_callback(const roboclaw::msg::EncoderSteps &msg);
    };

}

#endif //PROJECT_DIFFDRIVE_ROSCORE_H
