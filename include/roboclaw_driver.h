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

#ifndef PROJECT_ROBOCLAWDRIVER_H
#define PROJECT_ROBOCLAWDRIVER_H

#include <string>
#include <map>
#include <exception>
#include <thread>
#include <queue>
#include <atomic>
#include <memory>
#include <tuple>
#include <variant>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include <boost/thread/mutex.hpp>
#include "TimeoutSerial.h"

namespace roboclaw {

    enum EncoderCmds : std::uint8_t
    {
        ReadPosM1       = 16, // Read Encoder Count/Value for M1.
        ReadPosM2       = 17, // Read Encoder Count/Value for M2.
        ReadSpdM1       = 18, // Read M1 Speed in Encoder Counts Per Second.
        ReadSpdM2       = 19, // Read M2 Speed in Encoder Counts Per Second.
        ResetAll        = 20, // Resets Encoder Registers for M1 and M2(Quadrature only).
        ResetM1         = 22, // Set Encoder 1 Register(Quadrature only).
        ResetM2         = 23, // Set Encoder 2 Register(Quadrature only).
        ReadRawSpdM1    = 30, // Read Current M1 Raw Speed
        ReadRawSpdM2    = 31, // Read Current M2 Raw Speed
        ReadEncoderCnts = 78, // Read Encoders Counts
        ReadRawSpeeds   = 79, // Read Raw Motor Speeds
        ReadAvgSpeeds   = 108, // Read Motor Average Speeds
        ReadSpdErr      = 111, // Read Speed Errors
        ReadPosErr      = 114 // Read Position Errors
    };

    enum AdvMotorControlCmds : std::uint8_t
    {
        SetVelPIDM1             = 28, // Set Velocity PID Constants for M1.
        SetVelPIDM2             = 29, // Set Velocity PID Constants for M2.
        SetDutyM1               = 32, // Drive M1 With Signed Duty Cycle. (Encoders not required)
        SetDutyM2               = 33, // Drive M2 With Signed Duty Cycle. (Encoders not required)
        SetDutyM1M2             = 34, // Drive M1 / M2 With Signed Duty Cycle. (Encoders not required)
        SetSpdM1                = 35, // Drive M1 With Signed Speed.
        SetSpdM2                = 36, // Drive M2 With Signed Speed.
        SetSpdM1M2              = 37, // Drive M1 / M2 With Signed Speed.
        SetSpdAccM1             = 38, // Drive M1 With Signed Speed And Acceleration.
        SetSpdAccM2             = 39, // Drive M2 With Signed Speed And Acceleration.
        SetSpdAccM1M2           = 40, // Drive M1 / M2 With Signed Speed And Acceleration.
        SetSpdDistM1            = 41, // Drive M1 With Signed Speed And Distance. Buffered.
        SetSpdDistM2            = 42, // Drive M2 With Signed Speed And Distance. Buffered.
        SetSpdDistM1M2          = 43, // Drive M1 / M2 With Signed Speed And Distance. Buffered.
        SetSpdAccDistM1         = 44, // Drive M1 With Signed Speed, Acceleration and Distance. Buffered.
        SetSpdAccDistM2         = 45, // Drive M2 With Signed Speed, Acceleration and Distance. Buffered.
        SetSpdAccDistM1M2       = 46, // Drive M1 / M2 With Signed Speed, Acceleration and Distance. Buffered.
        ReadBufLen              = 47, // Read Buffer Length.
        SetSepSpdAccM1M2        = 50, // Drive M1 / M2 With Individual Signed Speed and Acceleration
        SetSepSpdAccDistM1M2    = 51, // Drive M1 / M2 With Individual Signed Speed, Accel and Distance
        SetDutyAccM1            = 52, // Drive M1 With Signed Duty and Accel. (Encoders not required)
        SetDutyAccM2            = 53, // Drive M2 With Signed Duty and Accel. (Encoders not required)
        SetDutyAccM1M2          = 54, // Drive M1 / M2 With Signed Duty and Accel. (Encoders not required)
        ReadVelPIDM1            = 55, // Read Motor 1 Velocity PID Constants
        ReadVelPIDM2            = 56, // Read Motor 2 Velocity PID Constants
        SetPosPIDM1             = 61, // Set Position PID Constants for M1.
        SetPosPIDM2             = 62, // Set Position PID Constants for M2.
        ReadPosPIDM1            = 63, // Read Position PID Constants for M1.
        ReadPosPIDM2            = 64, // Read Position PID Constants for M2.
        SetSpdAccDeccPosM1      = 65, // Drive M1 with Speed, Accel, Deccel and Position
        SetSpdAccDeccPosM2      = 66, // Drive M2 with Speed, Accel, Deccel and Position
        SetSpdAccDeccPosM1M2    = 67, // Drive M1 / M2 with Speed, Accel, Deccel and Position
        SetPosM1                = 119, // Drive M1 with Position.
        SetPosM2                = 120, // Drive M2 with Position.
        SetPosM1M2              = 121, // Drive M1/M2 with Position.
        SetPosSpdM1             = 122, // Drive M1 with Speed and Position.
        SetPosSpdM2             = 123, // Drive M2 with Speed and Position.
        SetPosSpdM1M2           = 124 // Drive M1/M2 with Speed and Postion.
    };

    enum CompatCmds : std::uint8_t
    {
        DrvFwdM1        = 0, // Drive Forward Motor 1
        DrvBackM1       = 1, // Drive Backwards Motor 1
        SetMainVoltMin  = 2, // Set Main Voltage Minimum
        SetMainVoltMax  = 3, // Set Main Voltage Maximum
        DrvFwdM2        = 4, // Drive Forward Motor 2
        DrvBackM2       = 5, // Drive Backwards Motor 2
        DrvM1           = 6, // Drive Motor 1 (7 Bit)
        DrvM2           = 7, // Drive Motor 2 (7 Bit)
        DrvFwdMixed     = 8, // Drive Forward Mixed Mode
        DrvBackMixed    = 9, // Drive Backwards Mixed Mode
        TurnRMixed      = 10, // Turn Right Mixed Mode
        TurnLMixed      = 11, // Turn Left Mixed Mode
        DrvFwdBack      = 12, // Drive Forward or Backward (7 bit)
        TurnLR          = 13 // Turn Left or Right (7 Bit)
    };

    enum StatusCmds : std::uint8_t
    {
        SetSerialTimeout        = 14, // Set Serial Timeout
        ReadSerialTimeout       = 15, // Read Serial Timeout
        ReadFirmwareVersion     = 21, // Read Firmware Version
        ReadMainVoltage         = 24, // Read Main Battery Voltage
        ReadLogicVoltage        = 25, // Read Logic Battery Voltage
        SetMinLogicVoltage      = 26, // Set Minimum Logic Voltage Level
        SetMaxLogicVoltage      = 27, // Set Maximum Logic Voltage Level
        ReadMotorPWMs           = 48, // Read Motor PWMs
        ReadMotorCurrents       = 49, // Read Motor Currents
        SetMainVoltage          = 57, // Set Main Battery Voltages
        SetLogicVoltage         = 58, // Set Logic Battery Voltages
        ReadMainVoltageSettings = 59, // Read Main Battery Voltage Settings
        ReadLogicVoltageSettings= 60, // Read Logic Battery Voltage Settings
        SetDefaultDutyAccM1     = 68, // Set default duty cycle acceleration for M1
        SetDefaultDutyAccM2     = 69,  // Set default duty cycle acceleration for M2
        SetDefaultSpeedM1       = 70, // Set Default Speed for M1
        SetDefaultSpeedM2       = 71, // Set Default Speed for M2
        ReadDefaultSpeed        = 72, // Read Default Speed Settings
        SetS3S4S5Modes          = 74, // Set S3,S4 and S5 Modes
        ReadS3S4S5Modes         = 75, // Read S3,S4 and S5 Modes
        SetAnalogRCDeadbamd     = 76, // Set DeadBand for RC/Analog controls
        ReadAnalogRCDeadbamd    = 77, // Read DeadBand for RC/Analog controls
        RestoreDefaults         = 80, // Restore Defaults
        ReadDefaultDutyAcc      = 81, // Read Default Duty Cycle Accelerations
        ReadTemp                = 82, // Read Temperature
        ReadTemp2               = 83, // Read Temperature 2
        ReadStatus              = 90, // Read Status
        ReadEncModes            = 91, // Read Encoder Modes
        SetEncModeM1            = 92, // Set Motor 1 Encoder Mode
        SetEncModeM2            = 93, // Set Motor 2 Encoder Mode
        WriteSettingsEEPROM     = 94, // Write Settings to EEPROM
        ReadSettingsEEPROM      = 95, // Read Settings from EEPROM
        SetStdConfig            = 98, // Set Standard Config Settings
        ReadStdConfig           = 99, // Read Standard Config Settings
        SetCtrlModes            = 100, // Set CTRL Modes
        ReadCtrlModes           = 101, // Read CTRL Modes
        SetCtrl1                = 102, // Set CTRL1
        SetCtrl2                = 103, // Set CTRL2
        ReadCtrls               = 104, // Read CTRLs
        SetAutoHomeDutySpdM1    = 105, // Set Auto Home Duty/Speed and Timeout M1
        SetAutoHomeDutySpdM2    = 106, // Set Auto Home Duty/Speed and Timeout M2
        ReadAutoHomeSettings    = 107, // Read Auto Home Settings
        SetSpeedErrLimits       = 109, // Set Speed Error Limits
        ReadSpeedErrLimits      = 110, // Read Speed Error Limits
        SetPosErrLimits         = 112, // Set Position Error Limits
        ReadPosErrLimits        = 113, // Read Position Error Limits
        SetBattVoltageOffsets   = 115, // Set Battery Voltage Offsets
        ReadBattVoltageOffsets  = 116, // Read Battery Voltage Offsets
        SetCurrBlankPercents    = 117, // Set Current Blanking Percentages
        ReadCurrBlankPercents   = 118, // Read Current Blanking Percentages
        SetMaxCurrM1            = 133, // Set M1 Maximum Current
        SetMaxCurrM2            = 134, // Set M2 Maximum Current
        ReadMaxCurrM1           = 135, // Read M1 Maximum Current
        ReadMaxCurrM2           = 136, // Read M2 Maximum Current
        SetPWMMode              = 148, // Set PWM Mode
        ReadPWMMode             = 149, // Read PWM Mode
        ReadUserEEPROM          = 252, // Read User EEPROM Memory Location
        WriteUserEEPROM         = 253, // Write User EEPROM Memory Location
    };

    typedef struct velocity_pid_t
    {
        uint32_t qpps;
        float p;
        float i;
        float d;
    } velocity_pid_t;

    typedef struct position_pid_t
    {
        uint32_t deadzone;
        int32_t max_pos;
        int32_t min_pos;
        float p;
        float i;
        float max_i;
        float d;
    } position_pid_t;

    typedef std::function<void(void)> cmd_func_t;
    typedef std::pair<uint8_t, cmd_func_t> cmd_t;

    class driver 
    {

    public:
        driver(
            const std::string port, 
            const uint32_t baudrate, 
            const uint32_t timeout_ms,
            rclcpp::Node *node
        );
        ~driver();

        void set_timeout_ms(const uint32_t to);

        void set_velocity(uint8_t address, std::pair<int32_t, int32_t> speed);
        void set_velocity_single(uint8_t address, uint8_t channel, int32_t speed);
        void set_duty(uint8_t address, std::pair<int16_t, int16_t> duty);
        void set_duty_single(uint8_t address, uint8_t channel, int16_t duty);
        void set_position(uint8_t address, std::pair<int32_t, int32_t> position);
        void set_position_single(uint8_t address, uint8_t channel, int32_t position);
        void reset_encoder(uint8_t address, uint8_t channel, int32_t value);
        void set_velocity_pid(uint8_t address, uint8_t channel, velocity_pid_t k);
        void set_position_pid(uint8_t address, uint8_t channel, position_pid_t k);
        void set_current_limit(uint8_t address, uint8_t channel, float limit);

        // TODO: change this mess to take some proper callbacks
        void write_eeprom(uint8_t address);
        void read_eeprom(uint8_t address);
        void read_version(uint8_t address);
        void read_encoders(uint8_t address);
        void read_velocity(uint8_t address);
        void read_logic_voltage(uint8_t address);
        void read_motor_voltage(uint8_t address);
        void read_motor_currents(uint8_t address);
        void read_volt_current(uint8_t address);
        void read_position_errors(uint8_t address);
        void read_status(uint8_t address);
        void read_motor_pwm(uint8_t address);
        void read_velocity_pid(uint8_t address, uint8_t channel);
        void read_position_pid(uint8_t address, uint8_t channel);
        void read_current_limit(uint8_t address, uint8_t channel);

        bool get_logic_voltage(uint8_t address, float &result);
        bool get_motor_voltage(uint8_t address, float &result);
        bool get_motor_current(uint8_t address, std::pair<float, float> &result);
        bool get_version(uint8_t address, std::string &result);
        bool get_encoders(uint8_t address, std::pair<int, int> &result);
        bool get_velocity(uint8_t address, std::pair<int, int> &result);
        bool get_position_errors(uint8_t address, std::pair<int, int> &result);
        bool get_status(uint8_t address, uint32_t &result);
        bool get_motor_pwm(uint8_t address, std::pair<int16_t, int16_t> &result);
        bool get_velocity_pid(uint8_t address, uint8_t channel, velocity_pid_t &result);
        bool get_position_pid(uint8_t address, uint8_t channel, position_pid_t &result);
        bool get_current_limit(uint8_t address, uint8_t channel, float &result);

        void clear_logic_volt_ready(uint8_t address) { logic_voltages_ready[address] = false; }
        void clear_motor_volt_ready(uint8_t address) { motor_voltages_ready[address] = false; }
        void clear_motor_amps_ready(uint8_t address) { motor_currents_ready[address] = false; }
        void clear_encoders_ready(uint8_t address) { encoders_ready[address] = false; }
        void clear_velocities_ready(uint8_t address) { velocities_ready[address] = false; }
        void clear_position_errors_ready(uint8_t address) { posn_err_ready[address] = false; }
        void clear_version_ready(uint8_t address) { versions_ready[address] = false; }
        void clear_status_ready(uint8_t address) { status_ready[address] = false; }
        void clear_motor_pwm_read(uint8_t address) { motor_pwm_ready[address] = false; }

        bool is_queue_flooded() { return (command_queue.size() > MAX_QUEUE_DEPTH); }

        const static uint8_t BASE_ADDRESS;
        const static uint32_t DEFAULT_BAUDRATE;
        const static float AMPS_SCALE;
        const static float VOLTS_SCALE;
        const static uint32_t DEFAULT_TIMEOUT_MS;
        const static uint8_t MAX_QUEUE_DEPTH;
        const static int32_t PID_CONST_MULT;

    private:

        bool run_enable;

        // typedef std::tuple<CommandType, uint8_t, int, int> cmd_t;
        std::unique_ptr<std::thread>    worker_thread;
        std::shared_ptr<TimeoutSerial>  serial;
        std::queue<cmd_t> command_queue;  

        boost::asio::io_service         io;
        boost::mutex                    serial_mutex;   // mutex to control access to the serial port
        boost::mutex                    queue_mutex;    // mutex to control access to the command queue
        boost::mutex                    data_mutex;     // mutex to control access to the returned data

        rclcpp::Node                    *log_node;
        
        std::map<uint8_t, float>                    logic_voltages;
        std::map<uint8_t, float>                    motor_voltages;
        std::map<uint8_t, std::pair<float, float>>  motor_currents;
        std::map<uint8_t, std::pair<int, int>>      encoders;
        std::map<uint8_t, std::pair<int, int>>      velocities;
        std::map<uint8_t, std::pair<int, int>>      posn_errors;
        std::map<uint8_t, std::string>              versions;
        std::map<uint8_t, uint32_t>                 status;
        std::map<uint8_t, std::pair<int16_t, int16_t>>                  motor_pwm;
        std::map<uint8_t, std::pair<velocity_pid_t, velocity_pid_t>>    velocity_pid_data;
        std::map<uint8_t, std::pair<position_pid_t, position_pid_t>>    position_pid_data;
        std::map<uint8_t, std::pair<float, float>>                      current_limit;

        std::map<uint8_t, std::atomic_bool>         logic_voltages_ready;
        std::map<uint8_t, std::atomic_bool>         motor_voltages_ready;
        std::map<uint8_t, std::atomic_bool>         motor_currents_ready;
        std::map<uint8_t, std::atomic_bool>         encoders_ready;
        std::map<uint8_t, std::atomic_bool>         velocities_ready;
        std::map<uint8_t, std::atomic_bool>         posn_err_ready;
        std::map<uint8_t, std::atomic_bool>         versions_ready;
        std::map<uint8_t, std::atomic_bool>         status_ready;
        std::map<uint8_t, std::atomic_bool>         motor_pwm_ready;
        std::map<uint8_t, std::pair<std::atomic_bool, std::atomic_bool>>  current_limit_ready;
        std::map<uint8_t, std::pair<std::atomic_bool, std::atomic_bool>>  velocity_pid_ready;
        std::map<uint8_t, std::pair<std::atomic_bool, std::atomic_bool>>  position_pid_ready;

        uint16_t crc;

        void worker();
        
        void exec_set_velocity(uint8_t address, std::pair<int32_t, int32_t> speed);
        void exec_set_velocity_single(uint8_t address, uint8_t channel, int32_t speed);
        void exec_set_duty(uint8_t address, std::pair<int16_t, int16_t> duty);
        void exec_set_duty_single(uint8_t address, uint8_t channel, int16_t duty);
        void exec_set_position(uint8_t address, std::pair<int32_t, int32_t> position);
        void exec_set_position_single(uint8_t address, uint8_t channel, int32_t position);
        void exec_reset_encoder(uint8_t address, uint8_t channel, int32_t value);
        void exec_set_velocity_pid(uint8_t address, uint8_t channel, velocity_pid_t pid);
        void exec_set_position_pid(uint8_t address, uint8_t channel, position_pid_t pid);
        void exec_set_current_limit(uint8_t address, uint8_t channel, float limit);

        void exec_write_eeprom(uint8_t address);
        void exec_read_eeprom(uint8_t address);
        void exec_read_version(uint8_t address);
        void exec_read_encoders(uint8_t address);
        void exec_read_velocity(uint8_t address);
        void exec_read_logic_voltage(uint8_t address);
        void exec_read_motor_voltage(uint8_t address);
        void exec_read_motor_currents(uint8_t address);
        void exec_read_volt_current(uint8_t address);
        void exec_read_position_errors(uint8_t address);
        void exec_read_status(uint8_t address);
        void exec_read_motor_pwm(uint8_t address);
        void exec_read_velocity_pid(uint8_t address, uint8_t channel);
        void exec_read_position_pid(uint8_t address, uint8_t channel);
        void exec_read_current_limit(uint8_t address, uint8_t channel);

        uint32_t decode_uint32(uint8_t *buf);
        int32_t decode_int32(uint8_t *buf);
        uint16_t decode_uint16(uint8_t *buf);
        int16_t decode_int16(uint8_t *buf);
        void encode_uint32(uint32_t val, uint8_t *buf);
        void encode_int32(int32_t val, uint8_t *buf);
        void encode_uint16(uint16_t val, uint8_t *buf);
        void encode_int16(int16_t val, uint8_t *buf);

        uint16_t crc16(uint8_t *packet, size_t nBytes);
        void crc16_reset();
        size_t txrx(uint8_t address, uint8_t command, uint8_t *tx_data, size_t tx_length,
                    uint8_t *rx_data, size_t rx_length, bool tx_crc = false, bool rx_crc = false);


    };

    class crc_exception : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    // trim from start (in place)
    static inline void ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
            return !std::isspace(ch);
        }));
    }

    // trim from end (in place)
    static inline void rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
            return !std::isspace(ch);
        }).base(), s.end());
    }

    // trim from both ends (in place)
    static inline void trim(std::string &s) {
        ltrim(s);
        rtrim(s);
    }
}
#endif //PROJECT_ROBOCLAWDRIVER_H
