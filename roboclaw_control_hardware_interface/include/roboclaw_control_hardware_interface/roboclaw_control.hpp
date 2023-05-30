/**
 *
 * Copyright (c) 2022 Pierce Nichols.
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

#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "roboclaw_driver/roboclaw_driver.h"
#include "roboclaw_control_hardware_interface/visibility_control.hpp"

using namespace roboclaw;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace roboclaw 
{
    class RoboclawHardwareInterface : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(RoboclawHardwareInterface)

            ROBOCLAW_HARDWARE_INTERFACE_PUBLIC
            CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override; 
            
            ROBOCLAW_HARDWARE_INTERFACE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            
            ROBOCLAW_HARDWARE_INTERFACE_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            
            ROBOCLAW_HARDWARE_INTERFACE_PUBLIC
            return_type prepare_command_mode_switch(
                const std::vector<std::string> &start_interfaces,
                const std::vector<std::string> &stop_interfaces) override;
            
            ROBOCLAW_HARDWARE_INTERFACE_PUBLIC
            CallbackReturn on_activate(const rclcpp_lifecycle::State &previous) override;
            
            ROBOCLAW_HARDWARE_INTERFACE_PUBLIC
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous) override;
            
            ROBOCLAW_HARDWARE_INTERFACE_PUBLIC
            return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
            
            ROBOCLAW_HARDWARE_INTERFACE_PUBLIC
            return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;


        private:

            std::unique_ptr<driver> roboclaw_;
            static rclcpp::Logger logger_;

            // configuration
            std::vector<std::vector<std::pair<uint8_t, uint8_t>>> addresses_;
            std::vector<position_pid_t> position_pid_;
            std::vector<velocity_pid_t> velocity_pid_;
            std::vector<float> current_limit_;

            // commands
            std::vector<double> command_position_;
            std::vector<double> command_velocity_;
            std::vector<double> command_effort_;

            // per joint feedback
            std::vector<double> state_position_;
            std::vector<double> state_velocity_;
            std::vector<double> state_effort_;
            std::vector<double> position_error_;
            std::vector<double> velocity_error_;
            std::vector<double> motor_current_;

            // per node feedback
            std::vector<double> motor_voltage_;
            std::vector<double> logic_voltage_;
            std::vector<double> status_;

            enum ctrl_lvl_t : std::uint8_t 
            {
                NONE = 0,
                PWM = 1,
                VELOCITY = 2,
                POSITION = 3
            };

            std::vector<ctrl_lvl_t> control_levels_;
            bool bad_inputs(uint8_t idx, uint8_t chan);
            position_pid_t fetch_position_pid(const hardware_interface::ComponentInfo& joint);
            velocity_pid_t fetch_velocity_pid(const hardware_interface::ComponentInfo& joint);

    };
} //namespace roboclaw
