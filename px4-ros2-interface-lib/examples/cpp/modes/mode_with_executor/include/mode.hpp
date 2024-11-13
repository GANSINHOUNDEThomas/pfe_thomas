/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <sensor_msgs/msg/joy.hpp>

// REQUIRED FOR DEFAULT MANUAL MODE
#include <px4_msgs/msg/manual_control_setpoint.hpp> // allow publishing manual joystick input
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/vehicle_global_position.hpp>


// FOR MY MANUAL MODE
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/control/peripheral_actuators.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <px4_ros2/control/setpoint_types/goto.hpp> // to use goto setpoint types

// #include <px4_ros2/utils/odometry_local_position.hpp>  // Include for OdometryLocalPosition
#include <cmath> // for fabsf()

// ajouté par moi (A VERIFIER )
#define NAVIGATION_STATE_MANUAL              0
#define NAVIGATION_STATE_ALTCTL              1
#define NAVIGATION_STATE_POSCTL              2
#define NAVIGATION_STATE_AUTO_MISSION        3
#define NAVIGATION_STATE_AUTO_LOITER         4
#define NAVIGATION_STATE_AUTO_RTL            5
#define NAVIGATION_STATE_ACRO                6
#define NAVIGATION_STATE_OFFBOARD            7
#define NAVIGATION_STATE_STAB                8
#define NAVIGATION_STATE_POSITION_SLOW       9
#define NAVIGATION_STATE_AUTO_TAKEOFF        10
#define NAVIGATION_STATE_AUTO_LAND           11
#define NAVIGATION_STATE_AUTO_FOLLOW_TARGET  12
#define NAVIGATION_STATE_AUTO_PRECLAND       13
#define NAVIGATION_STATE_ORBIT               14
#define NAVIGATION_STATE_AUTO_VTOL_TAKEOFF   15


// more define  // ajouté par moi
#define NAVIGATION_STATE_TAKEOFF 17
#define NAVIGATION_STATE_LAND 18
#define NAVIGATION_STATE_FOLLOW_TARGET 19
#define NAVIGATION_STATE_PRECLAND 20 
#define NAVIGATION_STATE_RTL 5

#define MAX_SPEED 5 // ici on suppose 5 /ms max  pour etre sur que le drone a toute la puissance qu'il lui faut


using namespace std::chrono_literals; // NOLINT

// // Global variable for joystick data shared between classes
sensor_msgs::msg::Joy::SharedPtr global_joy_data;


static const std::string kName = "Autonomous Executor";







// class FlightModeTest : public px4_ros2::ModeBase
// {
// public:
//     explicit FlightModeTest(rclcpp::Node &node)
//     : ModeBase(node, Settings{"My Manual Mode"}),
//     //   _manual_control_input(std::make_shared<px4_ros2::ManualControlInput>(*this)),
//       _rates_setpoint(std::make_shared<px4_ros2::RatesSetpointType>(*this)),
//       _attitude_setpoint(std::make_shared<px4_ros2::AttitudeSetpointType>(*this)),
//       _peripheral_actuator_controls(std::make_shared<px4_ros2::PeripheralActuatorControls>(*this))
//     {}

//     void onActivate() override
//     {
//         RCLCPP_INFO(node().get_logger(), "My Manual Mode Test activated.");
//     }

//     void onDeactivate() override
//     {
//         RCLCPP_INFO(node().get_logger(), "My Manual Mode Test deactivated.");
//     }

//     void updateSetpoint(float dt_s) override
//     {
//         if (!global_joy_data) {
//             RCLCPP_WARN(node().get_logger(), "No joystick data available.");
//             return;
//         }

//         if (global_joy_data->axes.size() < 4) {
//             RCLCPP_ERROR(node().get_logger(), "Joystick data does not contain enough axes.");
//             return;
//         }

//         // Map joystick axes to control inputs
//         float roll_input = global_joy_data->axes[2];
//         float pitch_input = global_joy_data->axes[3];
//         float yaw_input = global_joy_data->axes[0];
//         float throttle_input = global_joy_data->axes[1];

//         const float threshold = 0.5f;
//         const bool want_attitude = fabsf(roll_input) > threshold || fabsf(pitch_input) > threshold;

//         const float yaw_rate = px4_ros2::degToRad(yaw_input * 120.f);

//         if (want_attitude) {
//             _yaw += yaw_rate * dt_s;
//             const Eigen::Vector3f thrust_sp{0.f, 0.f, -throttle_input};
//             const Eigen::Quaternionf qd = px4_ros2::eulerRpyToQuaternion(
//                 px4_ros2::degToRad(roll_input * 55.f),
//                 px4_ros2::degToRad(-pitch_input * 55.f),
//                 _yaw
//             );
//             _attitude_setpoint->update(qd, thrust_sp, yaw_rate);
//         }

//         // Control a servo by passing through RC aux1 channel to 'Peripheral Actuator Set 1'
//         _peripheral_actuator_controls->set(0); // Assumes aux1 on button[4]
//     }

// private: 
//     // std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
//     std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
//     std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;
//     std::shared_ptr<px4_ros2::PeripheralActuatorControls> _peripheral_actuator_controls;
//     float _yaw{0.f};
// };

class FlightModeTest : public px4_ros2::ModeBase
{
public:
    explicit FlightModeTest(rclcpp::Node &node)
    : ModeBase(node, Settings{"My Manual Mode"}),
    //   _manual_control_input(std::make_shared<px4_ros2::ManualControlInput>(*this)),
      _rates_setpoint(std::make_shared<px4_ros2::RatesSetpointType>(*this)),
      _attitude_setpoint(std::make_shared<px4_ros2::AttitudeSetpointType>(*this)),
      _peripheral_actuator_controls(std::make_shared<px4_ros2::PeripheralActuatorControls>(*this))
    {}

    void onActivate() override
    {
        RCLCPP_INFO(node().get_logger(), "My Manual Mode Test activated.");
    }

    void onDeactivate() override
    {
        RCLCPP_INFO(node().get_logger(), "My Manual Mode Test deactivated.");
    }

    void updateSetpoint(float dt_s) override
    {
        if (!global_joy_data) {
            RCLCPP_WARN(node().get_logger(), "No joystick data available.");
            return;
        }

        if (global_joy_data->axes.size() < 4) {
            RCLCPP_ERROR(node().get_logger(), "Joystick data does not contain enough axes.");
            return;
        }

        // Map joystick axes to control inputs
        float roll_input = -global_joy_data->axes[2];
        float pitch_input = global_joy_data->axes[3];
        float yaw_input = -global_joy_data->axes[0]; 
        float throttle_input = global_joy_data->axes[1];

        const float threshold = 0.2f;
        const bool want_rates = fabsf(roll_input) > threshold || fabsf(pitch_input) > threshold;

        const float yaw_rate = px4_ros2::degToRad(yaw_input * 120.f);
        
        if (want_rates) {
            const Eigen::Vector3f thrust_sp{0.f, 0.f, -throttle_input};
            const Eigen::Vector3f rates_sp{
                px4_ros2::degToRad(roll_input * 50.f),
                px4_ros2::degToRad(-pitch_input * 50.f),
                yaw_rate
            };
            _rates_setpoint->update(rates_sp, thrust_sp);
        } else {
            _yaw += yaw_rate * dt_s;
            const Eigen::Vector3f thrust_sp{0.f, 0.f, -throttle_input};
            const Eigen::Quaternionf qd = px4_ros2::eulerRpyToQuaternion(
                px4_ros2::degToRad(roll_input * 55.f),
                px4_ros2::degToRad(-pitch_input * 55.f),
                _yaw
            );
            _attitude_setpoint->update(qd, thrust_sp, yaw_rate);
        }

        // Control a servo by passing through RC aux1 channel to 'Peripheral Actuator Set 1'
        _peripheral_actuator_controls->set(0); // Assumes aux1 on button[4]
    }

private:
    // std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
    std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
    std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;
    std::shared_ptr<px4_ros2::PeripheralActuatorControls> _peripheral_actuator_controls;
    float _yaw{0.f};
};


// // class FlightModeTest : public px4_ros2::ModeBase
// // {
// // public:
// //     explicit FlightModeTest(rclcpp::Node &node)
// //     : ModeBase(node, Settings{"My Manual Mode"}),
// //       _rates_setpoint(std::make_shared<px4_ros2::RatesSetpointType>(*this)),
// //       _peripheral_actuator_controls(std::make_shared<px4_ros2::PeripheralActuatorControls>(*this)),
// //       _vehicle_local_position(std::make_shared<px4_ros2::OdometryLocalPosition>(*this))
      
// //     {
// //         _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
// //     }

// //     void onActivate() override
// //     {
// //         RCLCPP_INFO(node().get_logger(), "My Manual Mode Test activated.");
// //         // Initialize altitude hold to the current altitude when activated
// //         _hold_altitude = _vehicle_local_position->positionNed().z();
// //     }

// //     void onDeactivate() override
// //     {
// //         RCLCPP_INFO(node().get_logger(), "My Manual Mode Test deactivated.");
// //     }

// //     void updateSetpoint(float dt_s) override
// //     {
// //         if (!global_joy_data) {
// //             RCLCPP_WARN(node().get_logger(), "No joystick data available.");
// //             return;
// //         }

// //         if (global_joy_data->axes.size() < 4) {
// //             RCLCPP_ERROR(node().get_logger(), "Joystick data does not contain enough axes.");
// //             return;
// //         }

// //         // // Map joystick axes to control inputs, normalize throttle to [0, 1]
// //         float roll_input = global_joy_data->axes[0];
// //         float pitch_input = global_joy_data->axes[1];
// //         float yaw_input = -global_joy_data->axes[2];
// //         float throttle_input = (global_joy_data->axes[3] + 1.0f) / 2.0f;  // Normalize to [0, 1]

// //         // const float current_altitude = _vehicle_local_position->positionNed().z();
// //         // const float altitude_error = _hold_altitude - current_altitude;
// //         // const float throttle_dead_zone = 0.05f;

// //         // if ( )

// //         // // Altitude hold logic
// //         // float altitude_hold_thrust = 0.5f;  // Baseline thrust for hover, adjust as necessary

// //         // // Adjust thrust to correct for altitude error if throttle is in the dead zone
// //         // if (fabsf(throttle_input - 0.5f) < throttle_dead_zone) {
// //         //     altitude_hold_thrust += altitude_error * _altitude_gain;  // P-gain for altitude control
// //         //     altitude_hold_thrust = std::clamp(altitude_hold_thrust, 0.0f, 1.0f);  // Limit thrust to [0, 1]
// //         // } else {
// //         //     // If throttle input is outside the dead zone, allow user to control altitude
// //         //     altitude_hold_thrust = throttle_input;
// //         //     _hold_altitude = current_altitude;  // Update hold altitude when user adjusts throttle
// //         // }

// //         // // Convert joystick inputs to rate setpoints
// //         // const Eigen::Vector3f rates_sp{
// //         //     px4_ros2::degToRad(roll_input * 200.f),   // Roll rate
// //         //     px4_ros2::degToRad(-pitch_input * 200.f), // Pitch rate
// //         //     px4_ros2::degToRad(yaw_input * 100.f)     // Yaw rate
// //         // };

// //         // // Update rates setpoint with computed altitude hold thrust
// //         // Eigen::Vector3f thrust_sp{0.0f, 0.0f, -altitude_hold_thrust};
// //         // _rates_setpoint->update(rates_sp, thrust_sp);

// //         // Optional logging for debugging
// //         // RCLCPP_INFO(node().get_logger(), "Position - Altitude: %.2f, Hold Altitude: %.2f, Altitude Error: %.2f, Thrust: %.2f",
// //         //             current_altitude, _hold_altitude, altitude_error, altitude_hold_thrust);

// //         // _peripheral_actuator_con/trols->set(1);
// //     }

// // private:
// //     std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
// //     std::shared_ptr<px4_ros2::PeripheralActuatorControls> _peripheral_actuator_controls;
// //     std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
// //     std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
// //     // float _hold_altitude{0.f};        // Altitude to hold when throttle is neutral
// //     // float _altitude_gain{0.3f};       // Proportional gain for altitude hold
// // };


class ModeExecutorTest : public px4_ros2::ModeExecutorBase
{
public:
  ModeExecutorTest(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
    _node(node) 
  {
    _joy_subscription = node.create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ModeExecutorTest::joyCallback, this, std::placeholders::_1));
    _last_button_states.resize(16, false);  // Initialize to match the number of buttons
    

    _manual_control_publisher = node.create_publisher<px4_msgs::msg::ManualControlSetpoint>(
      "/fmu/in/manual_control_input", 10); 

      
  }

  bool takeoffed = false;
  // State _previous_state = State::WaitUntilDisarmed;  // Initialize to a default or starting state


  void switchLeft()
  {
    _selected_mode = static_cast<State>((static_cast<int>(_selected_mode) - 1 + 10) % 10);  // 10 modes total
    RCLCPP_INFO(_node.get_logger(), "Switched to previous mode");
  }

  // Switch to the next mode (R1 button)
  void switchRight()
  {
    _selected_mode = static_cast<State>((static_cast<int>(_selected_mode) + 1) % 10);  // 16 modes total
    RCLCPP_INFO(_node.get_logger(), "Switched to next mode");
  }

  enum class State
  {
    waitReadyToArm,
    Arm,
    TakingOff,
    My_Manual_Mode,
    RTL,
    WaitUntilDisarmed,
    Land,
    Follow_Target,
    Hold,
    Manual,
    
  };
  
  State getCurrentMode() const {
    // Retrieve the current ModeBase::ModeID from PX4
    px4_ros2::ModeBase::ModeID my_mode_id = getCurrentNavState();

    // Map ModeBase::ModeID to State values
    switch (my_mode_id) {
        case NAVIGATION_STATE_MANUAL: return State::Manual;
        case NAVIGATION_STATE_ALTCTL: return State::waitReadyToArm;
        case NAVIGATION_STATE_POSCTL: return State::Hold;
        case NAVIGATION_STATE_AUTO_MISSION: return State::My_Manual_Mode;
        case NAVIGATION_STATE_AUTO_LOITER: return State::Hold;
        case NAVIGATION_STATE_RTL: return State::RTL;
        case NAVIGATION_STATE_TAKEOFF: return State::TakingOff;
        case NAVIGATION_STATE_LAND: return State::Land;
        case NAVIGATION_STATE_FOLLOW_TARGET: return State::Follow_Target;
        default:
            RCLCPP_WARN(_node.get_logger(), "Unknown mode ID: %d", static_cast<int>(my_mode_id));
            return State::Hold;  // Default to Hold if mode is unknown
    }
}


void onActivate() override
{
    _joy_check_timer = _node.create_wall_timer(
    100ms,  // Check every 100 milliseconds
    [this]() {
    if (_joy_data) {
        RCLCPP_INFO(_node.get_logger(), "Joy data received, proceeding with activation.");
        // Stop the timer once data is received
        _joy_check_timer->cancel();
        runState(State::waitReadyToArm, px4_ros2::Result::Success); 
    } else {
        RCLCPP_DEBUG(_node.get_logger(), "Waiting for joy data, Please launch joy_node or connect PS4 controller and try again...");
    }
    }
);
}
    
  


void onDeactivate(DeactivateReason reason) override
{
runState(State::RTL, px4_ros2::Result::Success); // just in case we lost control with the vehicle, i want it backt to the base
}

void runState(State state, px4_ros2::Result previous_result)
{
    // Log if the previous result was a failure, including the state that failed
    if (previous_result == px4_ros2::Result::Rejected) {
        if (state == State::Arm)
        {
          RCLCPP_ERROR(_node.get_logger(),"Arming Failed, Trying to run precheck condition... Wait");
          runState(State::waitReadyToArm, px4_ros2::Result::Success);
          
        };
        return;
    }

    // Log the current state being executed
    RCLCPP_INFO(
        _node.get_logger(),
        "Executing state: '%s' (%i)",
        stateToString(state).c_str(),
        static_cast<int>(state)
    );

    // Update _previous_state to the current state
    _previous_state = state;

    // Switch block to handle different states
    switch (state) {
        case State::waitReadyToArm:
            waitReadyToArm([this](px4_ros2::Result result) {
                logCompletion("Wait Ready to Arm", result);
            });
            break;

        case State::Arm:
            arm([this](px4_ros2::Result result) {
                logCompletion("Arm", result);
            });
            break;

        case State::Hold:
            scheduleMode(
                NAVIGATION_STATE_AUTO_LOITER,
                [this](px4_ros2::Result result) {
                    logCompletion("Hold", result);
                }
            );
            break;

        case State::TakingOff:
            takeoff([this](px4_ros2::Result result) {
                takeoffed = true;
                logCompletion("Taking Off", result);
                //runState(State::Hold, result);
            });
            // scheduleMode(
            //     NAVIGATION_STATE_TAKEOFF,
            //     [this](px4_ros2::Result result) {
            //         logCompletion("My Manual Mode", result);
            //     }
            // );
            break;

        case State::My_Manual_Mode:
            scheduleMode(
                ownedMode().id(),
                [this](px4_ros2::Result result) {
                    logCompletion("My Manual Mode", result);
                }
            );
            break;

        case State::Follow_Target:
            scheduleMode(
                NAVIGATION_STATE_AUTO_FOLLOW_TARGET,
                [this](px4_ros2::Result result) {
                    logCompletion("Follow Target", result);
                }
            );
            break;

        case State::Manual:
            // below i have to find the appropriate manual mode and replace it with ownedMode()
            // ownedMode().modeRequirements().manual_control = true;
            // scheduleMode(
            //     NAVIGATION_STATE_MANUAL,
            //     [this](px4_ros2::Result result) {
            //         logCompletion("Manual", result);
            //     }
            // );
            Manual_mode_PosCtl([this](px4_ros2::Result result) {
                logCompletion("Manual_mode_Position Control On", result);
                //runState(State::Hold, result);
            });
            
            break;

        case State::RTL:
            rtl([this](px4_ros2::Result result) {                
                logCompletion("Return to Launch (RTL)", result);
                runState(State::WaitUntilDisarmed, result);
            });
            takeoffed = false;
            break;

        case State::WaitUntilDisarmed:
            waitUntilDisarmed([this](px4_ros2::Result result) {
                logCompletion("Wait Until Disarmed", result);
                // runState(State::WaitUntilDisarmed, result);
            });
            break;

        case State::Land:
            land([this](px4_ros2::Result result) {                
                logCompletion("Land", result);
                runState(State::WaitUntilDisarmed, result);
            });
            takeoffed = false;
            break;
    }
}


// Handle joystick input for state switching
void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    _joy_data = msg;

    global_joy_data = msg;  // Copy the data to the global variable


    auto manual_setpoint = std::make_shared<px4_msgs::msg::ManualControlSetpoint>();

    // Use the correct field names
    // manual_setpoint->valid = true;
    manual_setpoint->roll = msg->axes[0];    // Roll (left/right)
    manual_setpoint->pitch = msg->axes[1];   // Pitch (forward/backward)
    manual_setpoint->yaw = msg->axes[2];     // Yaw rotation
    manual_setpoint->throttle = msg->axes[3]; // Throttle (up/down)

    if (!(msg->axes.empty()))
    {
        manual_setpoint->sticks_moving = true;
    }
    else {
        manual_setpoint->sticks_moving = false;
    }
    
    
    manual_setpoint->valid = true;
    manual_setpoint->timestamp = _node.get_clock()->now().nanoseconds() / 1000; // Timestamp in microseconds

    // Publish the manual control setpoint to PX4
    _manual_control_publisher->publish(*manual_setpoint); // Dereference for publishing



    // Iterate through buttons to detect state changes for debouncing
    for (size_t i = 0; i < _joy_data->buttons.size(); ++i) {
        bool current_state = (_joy_data->buttons[i] == 1);


        // Check if the button was just pressed (state change from unpressed to pressed)
        if (current_state && !_last_button_states[i]) {
            // Maintain the existing logic with added debouncing
            if (i == 2 && !isArmed()) {  // Button 3 (Square) for arming
                runState(State::Arm, px4_ros2::Result::Success);
            }

            if (isArmed()) {  
                // X button to Run current mode after takeoff
                if (takeoffed && i == 0) {
                    runState(_selected_mode, px4_ros2::Result::Success);
                }

                // Triangle button for my_manual mode 
                if (i == 3) {  // Button 2 (Triangle)
                    runState(State::My_Manual_Mode, px4_ros2::Result::Success);
                }

                // Shortcut for taking off (Square + X simultaneously)
                // if ((i == 2 && _joy_data->buttons[0] == 1) || (i == 0 && _joy_data->buttons[2] == 1)) {
                //     runState(State::TakingOff, px4_ros2::Result::Success);
                // }
                if ((_joy_data->buttons[0] == 1) && (_joy_data->buttons[2] == 1)) {
                    runState(State::TakingOff, px4_ros2::Result::Success);
                }

                // Circle button for landing
                if (i == 1 && isArmed()) {  // Button 1 (Circle)
                    runState(State::Land, px4_ros2::Result::Success);
                    takeoffed = false;
                }
            }

            // L1 button for switching left
            if (i == 9) {  // Button 4 (L1)
                switchLeft();
                logCurrentMode();
            }

            // R1 button for switching right
            if (i == 10) {  // Button 10 (R1)
                switchRight();
                logCurrentMode();
            }
            
        }

        // Update last button state for debouncing
        _last_button_states[i] = current_state;
    }
    // RCLCPP_WARN(_node.get_logger(), "ARMING STATE: '%d'.",isArmed());
}


void logCurrentMode()
{
    std::string mode_name;
    switch (_selected_mode) {
        case State::waitReadyToArm:
            mode_name = "Wait Ready to Arm";
            break;
        case State::Arm:
            mode_name = "Arm";
            break;
        case State::TakingOff:
            mode_name = "Taking Off";
            break;
        case State::My_Manual_Mode:
            mode_name = "My Manual Mode";
            break;
        case State::RTL:
            mode_name = "Return to Launch (RTL)";
            break;
        // case State::WaitUntilDisarmed:
        //     mode_name = "Wait Until Disarmed";
        //     break;
        case State::Land:
            mode_name = "Land";
            break;
        case State::Follow_Target:
            mode_name = "Follow Target";
            break;
        case State::Hold:
            mode_name = "Hold";
            break;
        case State::Manual:
            mode_name = "Manual";
            break;
        default:
            mode_name = "Unknown";
            break;
    }

    RCLCPP_INFO(_node.get_logger(), "Current mode switched to: %s (%i)", mode_name.c_str(), static_cast<int>(_selected_mode));
}
 

void logCompletion(const std::string &mode_name, px4_ros2::Result result)
{
    if (result == px4_ros2::Result::Success) {
        RCLCPP_DEBUG(_node.get_logger(), "Execution of '%s' completed successfully.", mode_name.c_str());
    } else {
      if (result == px4_ros2::Result::Deactivated)
      {
        RCLCPP_WARN(_node.get_logger(), "Execution of previous mode: '%s' INTERUPTED with result: %s.", mode_name.c_str(), resultToString(result));
      }
      else if (result == px4_ros2::Result::Rejected) {
        RCLCPP_ERROR(_node.get_logger(), "Execution of '%s' REQUIREMENTS NOT SATISFIED with result: %s.", mode_name.c_str(), resultToString(result));
      }    
    }
}


private:
  rclcpp::Node & _node;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscription;
  sensor_msgs::msg::Joy::SharedPtr _joy_data;  // Joystick data storage
  rclcpp::TimerBase::SharedPtr _joy_check_timer;
  rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr _manual_control_publisher;
  

  
  State _selected_mode;
  std::vector<bool> _last_button_states;

  State _previous_state= getCurrentMode(); // initialise to the vehicle state published by the vehicle status

  // Convert state to a string for logging purposes
    std::string stateToString(State state) const
    {
        switch (state) {
            case State::waitReadyToArm: return "Wait Ready to Arm";
            case State::Arm: return "Arm";
            case State::TakingOff: return "Taking Off";
            case State::My_Manual_Mode: return "My Manual Mode";
            case State::RTL: return "Return to Launch (RTL)";
            case State::WaitUntilDisarmed: return "Wait Until Disarmed";
            case State::Land: return "Land";
            case State::Follow_Target: return "Follow Target";
            case State::Hold: return "Hold";
            case State::Manual: return "Manual";
            default: return "Unknown";
        }
    }
  
};

