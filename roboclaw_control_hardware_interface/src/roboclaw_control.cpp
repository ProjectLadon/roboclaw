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

#include "roboclaw_control_hardware_interface/roboclaw_control.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace roboclaw
{
    CallbackReturn RoboclawHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) 
        {
            return CallbackReturn::ERROR;
        } 
  
        addresses_.resize(2);
        position_pid_.resize(info_.joints.size());
        velocity_pid_.resize(info_.joints.size());
        command_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        command_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        command_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        state_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        state_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        state_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        position_error_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_error_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        motor_current_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        motor_voltage_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
        logic_voltage_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
        status_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & sensor : info_.sensors) 
        {
            uint8_t idx = std::stoi(sensor.parameters.at("node_index"));
            if (!bad_inputs(idx, 1)) 
            {
                addresses_[0].emplace_back(std::make_pair(idx, 1));
            }
            else
            {
                addresses_[0].emplace_back(std::make_pair(0, 0));
            }
        }

        for (const hardware_interface::ComponentInfo & joint : info_.joints) 
        {
            uint8_t idx = std::stoi(joint.parameters.at("node_index"));
            uint8_t channel = std::stoi(joint.parameters.at("channel"));
            if (!bad_inputs(idx, channel)) 
            {
                addresses_[1].emplace_back(std::make_pair(idx, channel));
            }
            else
            {
                addresses_[1].emplace_back(std::make_pair(0, 0));
            }
            position_pid_.emplace_back(fetch_position_pid(joint));
            velocity_pid_.emplace_back(fetch_velocity_pid(joint));
            try { current_limit_.emplace_back(std::stof(joint.parameters.at("current_limit"))); }
            catch (const std::exception &e)
            { current_limit_.emplace_back(std::numeric_limits<float>::quiet_NaN()); }

        }

        roboclaw_ = std::make_unique<driver>(
            info_.hardware_parameters.at("serial_port"),
            std::stoi(info_.hardware_parameters.at("baudrate")),
            std::stoi(info_.hardware_parameters.at("timeout_ms")),
            logger_
        );

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            roboclaw_->set_velocity_pid(addresses_[1][i].first, addresses_[1][i].second, velocity_pid_[i]);
            roboclaw_->set_position_pid(addresses_[1][i].first, addresses_[1][i].second, position_pid_[i]);
            if (current_limit_[i] != std::numeric_limits<float>::quiet_NaN())
            {
                roboclaw_->set_current_limit(addresses_[1][i].first, addresses_[1][i].second, current_limit_[i]);
            }
        }
  
        return CallbackReturn::SUCCESS;
    } 

    CallbackReturn RoboclawHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            roboclaw_->set_duty_single(addresses_[1][i].first, addresses_[1][i].second, 0);
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RoboclawHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            roboclaw_->set_duty_single(addresses_[1][i].first, addresses_[1][i].second, 0);
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RoboclawHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;

        for (size_t i = 0; i < info_.sensors.size(); i++) 
        {
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "motor_voltage", &motor_voltage_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "logic_voltage", &logic_voltage_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "status", &status_[i]));
        }

        for (size_t i = 0; i < info_.joints.size(); i++) 
        {
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_position_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "position_error", &position_error_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "velocity_error", &velocity_error_[i]));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "motor_current", &motor_current_[i]));
        }

        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RoboclawHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++) 
        {
            interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &command_effort_[i]));
            interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &command_velocity_[i]));
            interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]));
        }

        return interfaces;
    }

    bool RoboclawHardwareInterface::bad_inputs(uint8_t idx, uint8_t chan)
    {
        if ((chan > 2) or (chan < 1))
        {
            RCLCPP_ERROR(logger_,  "Roboclaw channel must be either 1 or 2; is %d", chan);
            return true;
        }
        if (idx >= 8)
        {
            RCLCPP_ERROR(logger_,  "Only a maximum of 8 roboclaws can be connected; you called for %d", idx);
            return true;
        }
        return false;
    }
            
    position_pid_t RoboclawHardwareInterface::fetch_position_pid(const hardware_interface::ComponentInfo& joint)
    {
        position_pid_t retval = {}; // zero everything
        retval.max_pos = std::numeric_limits<int32_t>::max();
        retval.min_pos = std::numeric_limits<int32_t>::min();
        try { retval.deadzone = std::stoul(joint.parameters.at("position_pid_deadzone")); } catch (const std::exception &e) {}
        try { retval.max_pos = std::stol(joint.parameters.at("position_pid_max_pos")); } catch (const std::exception &e) {}
        try { retval.min_pos = std::stol(joint.parameters.at("position_pid_min_pos")); } catch (const std::exception &e) {}
        try { retval.p = std::stof(joint.parameters.at("position_pid_p")); } catch (const std::exception &e) {}
        try { retval.i = std::stof(joint.parameters.at("position_pid_i")); } catch (const std::exception &e) {}
        try { retval.d = std::stof(joint.parameters.at("position_pid_d")); } catch (const std::exception &e) {}
        try { retval.max_i = std::stof(joint.parameters.at("position_pid_max_i")); } catch (const std::exception &e) {}
        return retval;
    }

    velocity_pid_t RoboclawHardwareInterface::fetch_velocity_pid(const hardware_interface::ComponentInfo& joint)
    {
        velocity_pid_t retval = {}; // zero everything
        try { retval.qpps = std::stoul(joint.parameters.at("velocity_pid_qpps")); } catch (const std::exception &e) {}
        try { retval.p = std::stof(joint.parameters.at("velocity_pid_p")); } catch (const std::exception &e) {}
        try { retval.i = std::stof(joint.parameters.at("velocity_pid_i")); } catch (const std::exception &e) {}
        try { retval.d = std::stof(joint.parameters.at("velocity_pid_d")); } catch (const std::exception &e) {}
        return retval;
    }

    rclcpp::Logger RoboclawHardwareInterface::logger_ = rclcpp::get_logger("RoboclawHardwareInterface");

} // namespace roboclaw