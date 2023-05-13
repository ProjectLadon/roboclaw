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

#include "rclcpp/rclcpp.hpp"
#include "diffdrive_node.h"

#include <cmath>
#include <iostream>
#include <string>

// #include "tf/transform_datatypes.h"
// #include "tf/transform_broadcaster.h"

using namespace std;
using namespace std::chrono_literals;
//using std::placeholders::_1;

namespace roboclaw {

    DiffDriveCore::DiffDriveCore(string name) : Node(name)
    {
        // Declare the parameters
        this->declare_parameter<float>("base_width",       1.0f);
        this->declare_parameter<float>("steps_per_meter",  1000.0f);
        this->declare_parameter<bool>("open_loop",         false);
        this->declare_parameter<bool>("swap_motors",       true);
        this->declare_parameter<bool>("invert_motor_1",    false);
        this->declare_parameter<bool>("invert_motor_2",    false);
        this->declare_parameter<float>("var_pos_x",        0.01);
        this->declare_parameter<float>("var_pos_y",        0.01);
        this->declare_parameter<float>("var_pos_theta",    0.01);
        this->declare_parameter<int64_t>("target_index",   0);

        // fetch parameters
        this->get_parameter("base_width",       mBaseWidth);
        this->get_parameter("steps_per_meter",  mStepsPerMeter);
        this->get_parameter("open_loop",        mOpenLoop);
        this->get_parameter("swap_motors",      mSwapMotors);
        this->get_parameter("invert_motor_1",   mInvertMotor1);
        this->get_parameter("invert_motor_2",   mInvertMotor2);
        this->get_parameter("var_pos_x",        mVarPosX);
        this->get_parameter("var_pos_y",        mVarPosY);
        this->get_parameter("var_pos_theta",    mVarPosTheta);
        this->get_parameter("target_index",     mTargetIndex);

        // initialize internal state
        mLastX      = 0.0f;
        mLastSteps1 = 0;
        mLastSteps2 = 0;

        // create publishers and subscribers
        mOdomPub    = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        mMotorPub   = this->create_publisher<roboclaw::msg::MotorVelocity>("motor_cmd_vel", 10);
        mEncSub     = this->create_subscription<roboclaw::msg::EncoderSteps>(
                        "motor_enc", 10, bind(&DiffDriveCore::encoder_callback, this, std::placeholders::_1));
        mTwistSub   = this->create_subscription<geometry_msgs::msg::Twist>(
                        "cmd_vel", 10, bind(&DiffDriveCore::twist_callback, this, std::placeholders::_1));
    }

    void DiffDriveCore::twist_callback(const geometry_msgs::msg::Twist &msg) 
    {

        auto motor_vel = roboclaw::msg::MotorVelocity();
        motor_vel.index = mTargetIndex;
        motor_vel.mot1_vel_sps = 0;
        motor_vel.mot2_vel_sps = 0;

        // Linear
        motor_vel.mot1_vel_sps += (int) (mStepsPerMeter * msg.linear.x);
        motor_vel.mot2_vel_sps += (int) (mStepsPerMeter * msg.linear.x);

        if(msg.linear.y > 0)
        {
            motor_vel.mot2_vel_sps += (int) (mStepsPerMeter * msg.linear.y);
        }
        else if(msg.linear.y < 0)
        {
            motor_vel.mot1_vel_sps += (int) (mStepsPerMeter * msg.linear.y);
        }

        // Angular
        motor_vel.mot1_vel_sps += (int) -(mStepsPerMeter * msg.angular.z * mBaseWidth/2);
        motor_vel.mot2_vel_sps += (int) (mStepsPerMeter * msg.angular.z * mBaseWidth/2);

        if (mInvertMotor1)
        {
            motor_vel.mot1_vel_sps = -motor_vel.mot1_vel_sps;
            }

        if (mInvertMotor2)
        {
            motor_vel.mot2_vel_sps = -motor_vel.mot2_vel_sps;
        }

        if (mSwapMotors)
        {
            int tmp = motor_vel.mot1_vel_sps;
            motor_vel.mot1_vel_sps = motor_vel.mot2_vel_sps;
            motor_vel.mot2_vel_sps = tmp;
        }

        mMotorPub->publish(motor_vel);
    }

    void DiffDriveCore::encoder_callback(const roboclaw::msg::EncoderSteps &msg) {

        // TODO: reimplement this at some point -- not required for what we're doing now.

        // static tf::TransformBroadcaster br;

        // int delta_1 = msg.mot1_enc_steps - last_steps_1;
        // int delta_2 = msg.mot2_enc_steps - last_steps_2;

        // last_steps_1 = msg.mot1_enc_steps;
        // last_steps_2 = msg.mot2_enc_steps;

        // if (invert_motor_1)
        //     delta_1 = -delta_1;

        // if (invert_motor_2)
        //     delta_1 = -delta_2;

        // if (swap_motors){
        //     int tmp = delta_1;
        //     delta_1 = delta_2;
        //     delta_2 = tmp;
        // }

        // double u_w = ((delta_1 + delta_2) / mStepsPerMeter) / 2.0;
        // double u_p = ((delta_2 - delta_1) / mStepsPerMeter);

        // double delta_x = u_w * cos(last_theta);
        // double delta_y = u_w * sin(last_theta);
        // double delta_theta = u_p / mBaseWidth;

        // double cur_x = last_x + delta_x;
        // double cur_y = last_y + delta_y;
        // double cur_theta = last_theta + delta_theta;

        // nav_msgs::Odometry odom;

        // odom.header.frame_id = "odom";
        // odom.child_frame_id = "base_link";

        // // Time
        // odom.header.stamp = ros::Time::now();

        // // Position
        // odom.pose.pose.position.x = cur_x;
        // odom.pose.pose.position.y = cur_y;

        // // Velocity
        // odom.twist.twist.linear.x = cur_x - last_x;
        // odom.twist.twist.linear.y = cur_y - last_y;
        // odom.twist.twist.angular.z = cur_theta - last_theta;

        // tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, cur_theta);
        // odom.pose.pose.orientation.w = quaternion.w();
        // odom.pose.pose.orientation.x = quaternion.x();
        // odom.pose.pose.orientation.y = quaternion.y();
        // odom.pose.pose.orientation.z = quaternion.z();

        // // Pos_x Variance
        // odom.pose.covariance[0] = var_pos_x;

        // // Pos_y Variance
        // odom.pose.covariance[7] = var_pos_y;

        // // Theta_z Variance
        // odom.pose.covariance[35] = var_theta_z;

        // tf::Transform transform;
        // transform.setOrigin(tf::Vector3(last_x, last_y, 0.0));
        // transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, cur_theta));
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

        // odom_pub.publish(odom);

        // last_x = cur_x;
        // last_y = cur_y;
        // last_theta = cur_theta;

    }


}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<roboclaw::DiffDriveCore>("diff_drive"));
    rclcpp::shutdown();
    return 0;
}
