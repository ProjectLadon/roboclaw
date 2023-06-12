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
#include <map>
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
            return_type perform_command_mode_switch(
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

            struct feedback_interface_t
            {
                double              value;
                rclcpp::Duration    read_period;
                rclcpp::Time        last_read;
                feedback_interface_t(
                    const double &v, 
                    const rclcpp::Duration &rp
                ) : value(v), read_period(rp), last_read(rclcpp::Clock().now()) {}
            };

            // this is part of a thoroughly ugly hack to map the roboclaws'
            // per-board addressing scheme to the per-joint scheme of ros2_control
            struct board_interface_map_t
            {
                std::shared_ptr<feedback_interface_t> position;
                std::shared_ptr<feedback_interface_t> velocity;
                std::shared_ptr<feedback_interface_t> effort;
                std::shared_ptr<feedback_interface_t> position_err;
                std::shared_ptr<feedback_interface_t> velocity_err;
                std::shared_ptr<feedback_interface_t> motor_current;
            };
            typedef std::map<uint8_t, board_interface_map_t> joint_map_t;

            // pointers to the driver and the logging facility
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
            std::vector<std::shared_ptr<feedback_interface_t>> state_position_;
            std::vector<std::shared_ptr<feedback_interface_t>> state_velocity_;
            std::vector<std::shared_ptr<feedback_interface_t>> state_effort_;
            std::vector<std::shared_ptr<feedback_interface_t>> position_error_;
            std::vector<std::shared_ptr<feedback_interface_t>> velocity_error_;
            std::vector<std::shared_ptr<feedback_interface_t>> motor_current_;

            // per board feedback
            std::vector<feedback_interface_t> motor_voltage_;
            std::vector<feedback_interface_t> logic_voltage_;
            std::vector<feedback_interface_t> status_;

            // joint feedback interface to board mapping
            std::map<uint8_t, joint_map_t> board_interface_map_;

            enum ctrl_lvl_t : std::uint8_t 
            {
                NONE = 0,
                PWM = 1,
                VELOCITY = 2,
                POSITION = 3
            };

            std::vector<ctrl_lvl_t> control_level_;

            // utility functions
            bool bad_inputs(uint8_t idx, uint8_t chan);
            position_pid_t fetch_position_pid(const hardware_interface::ComponentInfo& joint);
            velocity_pid_t fetch_velocity_pid(const hardware_interface::ComponentInfo& joint);
            // The default period is a terasecond... which is about 32k yrs, i.e. effectively never)
            rclcpp::Duration get_duration(
                const hardware_interface::ComponentInfo& component, 
                const std::string &key, 
                const double default_period = 1e12);
            bool check_time(const rclcpp::Time &time, const feedback_interface_t *interface);

            // execute feedback reads
            void execute_fb_read(const rclcpp::Time &time, const uint8_t address, board_interface_map_t *ch1, board_interface_map_t *ch2);

            // data read callbacks
            void logic_voltage_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1);
            void motor_voltage_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1);
            void status_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1);
            void position_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1, feedback_interface_t *if2);
            void velocity_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1, feedback_interface_t *if2);
            void effort_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1, feedback_interface_t *if2);
            void position_err_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1, feedback_interface_t *if2);
            void velocity_err_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1, feedback_interface_t *if2);
            void motor_current_cb(uint8_t address, uint8_t channel, feedback_interface_t *if1, feedback_interface_t *if2);
    };
} //namespace roboclaw
