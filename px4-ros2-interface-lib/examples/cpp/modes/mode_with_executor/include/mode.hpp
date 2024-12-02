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
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>


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

int B_CROIX = 0;
int B_ROND = 1;
int B_CARRE = 2;
int B_TRIANGLE = 3;
int B_SHARE = 4;
int B_PS = 5;
int B_OPT = 6;
int B_JL = 7;
int B_JR = 8;
int B_L1 = 9;
int B_R1 = 10;
int B_H = 11;
int B_B = 12;
int B_L = 13;
int B_R = 14;
int B_CLIC = 15;

int J_LH = 0;
int J_LV = 1;
int J_L2 = 4;
int J_RH = 2;
int J_RV = 3;
int J_R2 = 5;
bool my_init_position = true;



Eigen::Vector3f current_position = {NAN,NAN,NAN};
float current_yaw = NAN;

class FlightModeTest : public px4_ros2::ModeBase
{
public:
    explicit FlightModeTest(rclcpp::Node &node)
    : ModeBase(node, Settings{"My Manual Mode"}),

      _trajectory_setpoint(std::make_shared<px4_ros2::TrajectorySetpointType>(*this)),
     _vehicle_local_position(std::make_shared<px4_ros2::OdometryLocalPosition>(*this))
     
    {
        // modeRequirements().manual_control = true; // ajouté par moi
    }

    void onActivate() override
    {
        RCLCPP_INFO(node().get_logger(), "My Manual Mode Test activated.");
        
        current_position = _vehicle_local_position->positionNed();
        current_yaw = _vehicle_local_position->heading();
        // const Eigen::Vector3f next_position = current_position;

    }

    void onDeactivate() override
    {
        RCLCPP_INFO(node().get_logger(), "My Manual Mode Test deactivated.");
        // my_init_position = true;
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



        // RCLCPP_INFO(node().get_logger(), "Position - X: %.2f, Y: %.2f, Z: %.2f, Heading: %.2f",
        //     _vehicle_local_position->positionNed().x(), _vehicle_local_position->positionNed().y(), _vehicle_local_position->positionNed().z(), current_yaw);


        float cs = cos(current_yaw);
        float ss = sin(current_yaw);
        RCLCPP_INFO(node().get_logger(), "cos: %.2f, sin: %.2f", cs, ss);

        // current_position = {
        //     static_cast<float>(current_position[0] + ((global_joy_data->axes[J_LH] * ss) + (global_joy_data->axes[J_LV] * cs)) / 10.0),
        //     static_cast<float>(current_position[1] + ((-global_joy_data->axes[J_LH] * cs) + (global_joy_data->axes[J_LV] * ss)) / 10.0),
        //     static_cast<float>(current_position[2]  - (global_joy_data->axes[J_RV] / 2.0))
        // };

        current_position = {
            static_cast<float>(current_position[0] + ((global_joy_data->axes[J_LH] * ss) + (global_joy_data->axes[J_LV] * cs)) / 20.0),
            static_cast<float>(current_position[1] + ((-global_joy_data->axes[J_LH] * cs) + (global_joy_data->axes[J_LV] * ss)) / 20.0),
            static_cast<float>(current_position[2]  - (global_joy_data->axes[J_RV] / 5.0))
        };
        RCLCPP_INFO(node().get_logger(), "Position - X: %.2f, Y: %.2f, Z: %.2f, Heading: %.2f",
            current_position[0], current_position[1], current_position[2], current_yaw);
        // current_yaw = static_cast<float>(current_yaw - global_joy_data->axes[J_RH] / 5.0);

        current_yaw = static_cast<float>(current_yaw - global_joy_data->axes[J_RH] / 7.0);
// 

        // _trajectory_setpoint->updatePosition(next_position);
        _trajectory_setpoint->updatePosition_and_yaw(current_position, current_yaw); // ajouté par moi
    }
private:
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position; 


};


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

            // scheduleMode(
            //     NAVIGATION_STATE_MANUAL,
            //     [this](px4_ros2::Result result) {
            //         logCompletion("Manual", result);
            //     }
            // );


            
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

    // Iterate through buttons to detect state changes for debouncing
    for (int i = 0; i < static_cast<int>(_joy_data->buttons.size()); ++i) {
        bool current_state = (_joy_data->buttons[i] == 1);


        // Check if the button was just pressed (state change from unpressed to pressed)
        if (current_state && !_last_button_states[i]) {
            // Maintain the existing logic with added debouncing
            if (i == B_CARRE && !isArmed()) {  // Button 3 (Square) for arming
                runState(State::Arm, px4_ros2::Result::Success);
            }

            if (isArmed()) {  
                // X button to Run current mode after takeoff
                if (takeoffed && i == B_CROIX) {
                    runState(_selected_mode, px4_ros2::Result::Success);
                }

                // Triangle button for my_manual mode 
                if (i == B_TRIANGLE) {  
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
                if (i == B_ROND && isArmed()) {  // Button 1 (Circle)
                    runState(State::Land, px4_ros2::Result::Success);
                    takeoffed = false;
                }
            }

            // L1 button for switching left
            if (i == B_L1) {  // Button 4 (L1)
                switchLeft();
                logCurrentMode();
            }

            // R1 button for switching right
            if (i == B_R1) {  // Button 10 (R1)
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

// void manual_mode_customized()
// {

// }


private:
  rclcpp::Node & _node;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscription;
  sensor_msgs::msg::Joy::SharedPtr _joy_data;  // Joystick data storage
  rclcpp::TimerBase::SharedPtr _joy_check_timer;
//   rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr _manual_control_publisher;
  

  
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

