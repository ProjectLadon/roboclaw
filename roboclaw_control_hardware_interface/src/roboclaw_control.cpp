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
            // Note that the default period is set so that the read of this value never fires
            // if the period is not explicitly set. 
            motor_voltage_.emplace_back(feedback_interface_t(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(sensor, "motor_voltage_period")));
            logic_voltage_.emplace_back(feedback_interface_t(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(sensor, "logic_voltage_period")));
            status_.emplace_back(feedback_interface_t(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(sensor, "status_period")));
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

            // Note that the default period is set so that the read of this value never fires
            // if the period is not explicitly set. 
            auto sp = std::make_shared<feedback_interface_t>(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(joint, "position_feedback_period"));
            state_position_.emplace_back(sp);
            board_interface_map_[idx][channel].position = sp;
            auto sv = std::make_shared<feedback_interface_t>(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(joint, "velocity_feedback_period"));
            state_velocity_.emplace_back(sv);
            board_interface_map_[idx][channel].velocity = sv;
            auto se = std::make_shared<feedback_interface_t>(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(joint, "effort_feedback_period"));
            state_effort_.emplace_back(se);
            board_interface_map_[idx][channel].effort = se;
            auto ve = std::make_shared<feedback_interface_t>(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(joint, "velocity_error_period"));
            velocity_error_.emplace_back(ve);
            board_interface_map_[idx][channel].velocity_err = ve;
            auto pe = std::make_shared<feedback_interface_t>(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(joint, "position_error_period"));
            position_error_.emplace_back(pe);
            board_interface_map_[idx][channel].position_err = pe;
            auto mc = std::make_shared<feedback_interface_t>(
                std::numeric_limits<double>::quiet_NaN(),
                get_duration(joint, "motor_current_period"));
            motor_current_.emplace_back(mc);
            board_interface_map_[idx][channel].motor_current = mc;
        }

        roboclaw_ = std::make_unique<driver>(
            info_.hardware_parameters.at("serial_port"),
            std::stoi(info_.hardware_parameters.at("baudrate")),
            std::stoi(info_.hardware_parameters.at("timeout_ms")),
            logger_
        );

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            // TODO: set this up so it only sets up the PID constants if they are present and otherwise ignores them
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
                info_.sensors[i].name, "motor_voltage", &motor_voltage_[i].value));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "logic_voltage", &logic_voltage_[i].value));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "status", &status_[i].value));
        }

        for (size_t i = 0; i < info_.joints.size(); i++) 
        {
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]->value));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]->value));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_position_[i]->value));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "position_error", &position_error_[i]->value));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "velocity_error", &velocity_error_[i]->value));
            interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "motor_current", &motor_current_[i]->value));
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

    return_type RoboclawHardwareInterface::prepare_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces)
    {
        for (std::string key : stop_interfaces) 
        {
            for (size_t i = 0; i < info_.joints.size(); i++) 
            {
                if (key.find(info_.joints[i].name) != std::string::npos) 
                {
                    control_level_[i] = ctrl_lvl_t::NONE;
                }
            }
        }

        for (std::string key : start_interfaces) 
        {
            for (size_t i = 0; i < info_.joints.size(); i++) 
            {
                switch (control_level_[i]) 
                {
                    case ctrl_lvl_t::NONE:
                    if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                        control_level_[i] = ctrl_lvl_t::PWM;
                    }

                    case ctrl_lvl_t::PWM:
                        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                            control_level_[i] = ctrl_lvl_t::VELOCITY;
                        }

                    case ctrl_lvl_t::VELOCITY:
                        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                            control_level_[i] = ctrl_lvl_t::POSITION;
                        }

                    case ctrl_lvl_t::POSITION:
                        break;
                }
            }
        }

        return return_type::OK;
    }
    
    return_type RoboclawHardwareInterface::perform_command_mode_switch(
        const std::vector<std::string> &, const std::vector<std::string> &)
    {
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            switch (control_level_[i])
            {
                case ctrl_lvl_t::NONE:
                    roboclaw_->set_position_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    roboclaw_->set_velocity_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    roboclaw_->set_duty_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    break;
                case ctrl_lvl_t::PWM:
                    roboclaw_->set_position_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    roboclaw_->set_velocity_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    roboclaw_->set_duty_single(addresses_[1][i].first, addresses_[1][i].second, (int16_t)command_effort_[i]);
                    break;
                case ctrl_lvl_t::VELOCITY:
                    roboclaw_->set_duty_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    roboclaw_->set_velocity_single(addresses_[1][i].first, addresses_[1][i].second, (int32_t)command_velocity_[i]);
                    break;
                case ctrl_lvl_t::POSITION:
                    roboclaw_->set_duty_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    roboclaw_->set_position_single(addresses_[1][i].first, addresses_[1][i].second, (int32_t)command_position_[i]);
                    break;
                default:
                    roboclaw_->set_position_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    roboclaw_->set_velocity_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    roboclaw_->set_duty_single(addresses_[1][i].first, addresses_[1][i].second, 0);
                    break;
            }
        }
        
        return return_type::OK;
    }

    return_type RoboclawHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &) 
    {
        // queue up the sensor reads
        for (size_t i = 0; i < info_.sensors.size(); i++)
        {
            if (check_time(time, &logic_voltage_[i])) 
            {
                roboclaw_->read_logic_voltage(
                    addresses_[0][i].first,
                    std::function<void(uint8_t, uint8_t)>(
                        std::bind(
                            &RoboclawHardwareInterface::logic_voltage_cb,
                            this, std::placeholders::_1, std::placeholders::_2,
                            &logic_voltage_[i]
                        )
                    )
                );
            }
            if (check_time(time, &motor_voltage_[i])) 
            {
                roboclaw_->read_motor_voltage(
                    addresses_[0][i].first,
                    std::function<void(uint8_t, uint8_t)>(
                        std::bind(
                            &RoboclawHardwareInterface::motor_voltage_cb,
                            this, std::placeholders::_1, std::placeholders::_2,
                            &motor_voltage_[i]
                        )
                    )
                );
            }
            if (check_time(time, &status_[i])) 
            {
                roboclaw_->read_status(
                    addresses_[0][i].first,
                    std::function<void(uint8_t, uint8_t)>(
                        std::bind(
                            &RoboclawHardwareInterface::status_cb,
                            this, std::placeholders::_1, std::placeholders::_2,
                            &status_[i]
                        )
                    )
                );
            }
        }
        // queue up the joint reads 
        for (auto &board : board_interface_map_)
        {
            if ((board.second.find(1) != board.second.end()) and (board.second.find(2) != board.second.end()))
            {
                execute_fb_read(time, board.first, &board.second[1], &board.second[2]);
            }
            else if (board.second.find(1) != board.second.end())
            {
                execute_fb_read(time, board.first, &board.second[1], nullptr);
            }
            else if (board.second.find(2) != board.second.end())
            {
                execute_fb_read(time, board.first, nullptr, &board.second[2]);
            }
            else 
            {
                return return_type::ERROR;
            }
        }

        return return_type::OK;
    }
    
    return_type RoboclawHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) 
    {
        for (size_t i = 0; i < info_.joints.size(); i++) 
        {
            switch (control_level_[i])
            {
                case ctrl_lvl_t::PWM:
                    roboclaw_->set_duty_single(
                        addresses_[0][i].first, addresses_[0][i].second, 
                        static_cast<int16_t>(command_effort_[i])
                    );
                    break;
                case ctrl_lvl_t::VELOCITY:
                    roboclaw_->set_velocity_single(
                        addresses_[0][i].first, addresses_[0][i].second, 
                        static_cast<int32_t>(command_velocity_[i])
                    );
                    break;
                case ctrl_lvl_t::POSITION:
                    roboclaw_->set_position_single(
                        addresses_[0][i].first, addresses_[0][i].second, 
                        static_cast<int32_t>(command_position_[i])
                    );
                    break;
                default:
                    roboclaw_->set_duty_single(
                        addresses_[0][i].first, addresses_[0][i].second, 0);
            }
        }
        
        return return_type::OK;
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

    rclcpp::Duration RoboclawHardwareInterface::get_duration(
        const hardware_interface::ComponentInfo& component, 
        const std::string &key, 
        const double default_period)
    {
        try { return rclcpp::Duration(std::chrono::nanoseconds((int64_t)RCL_S_TO_NS(std::stod(component.parameters.at(key))))); } 
        catch (const std::exception &e) { return rclcpp::Duration(std::chrono::nanoseconds((int64_t)RCL_S_TO_NS(default_period))); }
    }
    
    bool RoboclawHardwareInterface::check_time(const rclcpp::Time &time, const feedback_interface_t *interface)
    {
        if (interface == nullptr) return false;
        if ((time - interface->last_read) > interface->read_period) return true;
        return false;
    }
            
    void RoboclawHardwareInterface::execute_fb_read(
        const rclcpp::Time &time, 
        const uint8_t address, 
        board_interface_map_t *ch1, 
        board_interface_map_t *ch2)
    {
        // This mess is to handle the nullptr conditions
        if ((ch1 == nullptr) and (ch2 == nullptr)) return;
        auto if1 = ch1;
        auto if2 = ch2;
        if (if1 == nullptr) if1 = if2;
        if (if2 == nullptr) if2 = if1;

        if ((check_time(time, if1->position.get())) or  (check_time(time, if2->position.get()))) 
        {
            roboclaw_->read_encoders(
                address,
                std::function<void(uint8_t, uint8_t)>(
                    std::bind(
                        &RoboclawHardwareInterface::position_cb,
                        this, std::placeholders::_1, std::placeholders::_2,
                        (ch1 == nullptr) ? nullptr : ch1->position.get(), 
                        (ch2 == nullptr) ? nullptr : ch2->position.get()
                    )
                )
            );
        }

        if ((check_time(time, if1->velocity.get())) or  (check_time(time, if2->velocity.get()))) 
        {
            roboclaw_->read_velocity(
                address,
                std::function<void(uint8_t, uint8_t)>(
                    std::bind(
                        &RoboclawHardwareInterface::velocity_cb,
                        this, std::placeholders::_1, std::placeholders::_2,
                        (ch1 == nullptr) ? nullptr : ch1->velocity.get(), 
                        (ch2 == nullptr) ? nullptr : ch2->velocity.get()
                    )
                )
            );
        }

        if ((check_time(time, if1->effort.get())) or  (check_time(time, if2->effort.get()))) 
        {
            roboclaw_->read_motor_pwm(
                address,
                std::function<void(uint8_t, uint8_t)>(
                    std::bind(
                        &RoboclawHardwareInterface::effort_cb,
                        this, std::placeholders::_1, std::placeholders::_2,
                        (ch1 == nullptr) ? nullptr : ch1->effort.get(), 
                        (ch2 == nullptr) ? nullptr : ch2->effort.get()
                    )
                )
            );
        }

        if ((check_time(time, if1->position_err.get())) or  (check_time(time, if2->position_err.get()))) 
        {
            roboclaw_->read_position_errors(
                address,
                std::function<void(uint8_t, uint8_t)>(
                    std::bind(
                        &RoboclawHardwareInterface::position_err_cb,
                        this, std::placeholders::_1, std::placeholders::_2,
                        (ch1 == nullptr) ? nullptr : ch1->position_err.get(), 
                        (ch2 == nullptr) ? nullptr : ch2->position_err.get()
                    )
                )
            );
        }

        if ((check_time(time, if1->velocity_err.get())) or  (check_time(time, if2->velocity_err.get()))) 
        {
            roboclaw_->read_velocity_errors(
                address,
                std::function<void(uint8_t, uint8_t)>(
                    std::bind(
                        &RoboclawHardwareInterface::velocity_err_cb,
                        this, std::placeholders::_1, std::placeholders::_2,
                        (ch1 == nullptr) ? nullptr : ch1->velocity_err.get(), 
                        (ch2 == nullptr) ? nullptr : ch2->velocity_err.get()
                    )
                )
            );
        }

        if ((check_time(time, if1->motor_current.get())) or  (check_time(time, if2->motor_current.get()))) 
        {
            roboclaw_->read_motor_currents(
                address,
                std::function<void(uint8_t, uint8_t)>(
                    std::bind(
                        &RoboclawHardwareInterface::motor_current_cb,
                        this, std::placeholders::_1, std::placeholders::_2,
                        (ch1 == nullptr) ? nullptr : ch1->motor_current.get(), 
                        (ch2 == nullptr) ? nullptr : ch2->motor_current.get()
                    )
                )
            );
        }
    }

    void RoboclawHardwareInterface::logic_voltage_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1)
    {
        float result;
        if (roboclaw_->get_logic_voltage(address, result) and (if1 != nullptr))
        {  
            if1->value = (double)result;
            if1->last_read = rclcpp::Clock().now();
            roboclaw_->clear_logic_volt_ready(address);
        }
    }

    void RoboclawHardwareInterface::motor_voltage_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1)
    {
        float result;
        if (roboclaw_->get_motor_voltage(address, result) and (if1 != nullptr))
        {  
            if1->value = (double)result;
            if1->last_read = rclcpp::Clock().now();
            roboclaw_->clear_motor_volt_ready(address);
        }
    }

    void RoboclawHardwareInterface::status_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1)
    {
        uint32_t result;
        if (roboclaw_->get_status(address, result) and (if1 != nullptr))
        {  
            if1->value = (double)result;
            if1->last_read = rclcpp::Clock().now();
            roboclaw_->clear_status_ready(address);
        }
    }

    void RoboclawHardwareInterface::position_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1, 
        feedback_interface_t *if2)
    {
        std::pair<int, int> result;
        if (roboclaw_->get_encoders(address, result))
        {
            if (if1 != nullptr) 
            { 
                if1->value = (double)result.first;
                if1->last_read = rclcpp::Clock().now();
            }
            if (if2 != nullptr) 
            { 
                if2->value = (double)result.second;
                if2->last_read = rclcpp::Clock().now();
            }
            roboclaw_->clear_encoders_ready(address);
        }
    }

    void RoboclawHardwareInterface::velocity_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1, 
        feedback_interface_t *if2)
    {
        std::pair<int, int> result;
        if (roboclaw_->get_velocity(address, result))
        {
            if (if1 != nullptr) 
            { 
                if1->value = (double)result.first;
                if1->last_read = rclcpp::Clock().now();
            }
            if (if2 != nullptr) 
            { 
                if2->value = (double)result.second;
                if2->last_read = rclcpp::Clock().now();
            }
            roboclaw_->clear_velocities_ready(address);
        }
    }

    void RoboclawHardwareInterface::effort_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1, 
        feedback_interface_t *if2)
    {
        std::pair<int16_t, int16_t> result;
        if (roboclaw_->get_motor_pwm(address, result))
        {
            if (if1 != nullptr) 
            { 
                if1->value = (double)result.first;
                if1->last_read = rclcpp::Clock().now();
            }
            if (if2 != nullptr) 
            { 
                if2->value = (double)result.second;
                if2->last_read = rclcpp::Clock().now();
            }
            roboclaw_->clear_motor_pwm_ready(address);
        }
    }

    void RoboclawHardwareInterface::position_err_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1, 
        feedback_interface_t *if2)
    {
        std::pair<int, int> result;
        if (roboclaw_->get_position_errors(address, result))
        {
            if (if1 != nullptr) 
            { 
                if1->value = (double)result.first;
                if1->last_read = rclcpp::Clock().now();
            }
            if (if2 != nullptr) 
            { 
                if2->value = (double)result.second;
                if2->last_read = rclcpp::Clock().now();
            }
            roboclaw_->clear_position_errors_ready(address);
        }
    }

    void RoboclawHardwareInterface::velocity_err_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1, 
        feedback_interface_t *if2)
    {
        std::pair<int, int> result;
        if (roboclaw_->get_velocity_errors(address, result))
        {
            if (if1 != nullptr) 
            { 
                if1->value = (double)result.first;
                if1->last_read = rclcpp::Clock().now();
            }
            if (if2 != nullptr) 
            { 
                if2->value = (double)result.second;
                if2->last_read = rclcpp::Clock().now();
            }
            roboclaw_->clear_velocity_errors_ready(address);
        }
    }

    void RoboclawHardwareInterface::motor_current_cb(
        uint8_t address, uint8_t, 
        feedback_interface_t *if1, 
        feedback_interface_t *if2)
    {
        std::pair<float, float> result;
        if (roboclaw_->get_motor_current(address, result))
        {
            if (if1 != nullptr) 
            { 
                if1->value = (double)result.first;
                if1->last_read = rclcpp::Clock().now();
            }
            if (if2 != nullptr) 
            { 
                if2->value = (double)result.second;
                if2->last_read = rclcpp::Clock().now();
            }
            roboclaw_->clear_motor_amps_ready(address);
        }
    }
    

    rclcpp::Logger RoboclawHardwareInterface::logger_ = rclcpp::get_logger("RoboclawHardwareInterface");

} // namespace roboclaw