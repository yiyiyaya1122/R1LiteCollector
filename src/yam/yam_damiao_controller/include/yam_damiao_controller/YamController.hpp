#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <array>
#include <vector>

#include "yam_arm_msg/msg/yam_cmd.hpp"
#include "yam_arm_msg/msg/yam_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "yam_damiao_controller/Damiao_6dof_node.hpp"
#include "mathematical_model/yam_fun.hpp"

namespace yam {

// State machine states
enum class ArmState {
    SOFT = 0,           // Disabled state
    INIT = 1,        // Homing procedure
    PROTECT = 2,        // Protection mode
    G_COMPENSATION = 3, // Gravity compensation
    END_CONTROL = 4,    // End effector control
    JOINT_CONTROL = 5, // Joint position control
    ZERO = 6,           // Zeroing/calibration
    PLANNING = 7        // Trajectory planning
};

class YamController : public rclcpp::Node {
public:
    YamController();
    void cleanup();

    void robot_init();

    // Command callback
    void cmdCallback(const yam_arm_msg::msg::YamCmd::SharedPtr msg);

    // State publishing
    void publishState();

    // State machine functions
    void transitionToState(ArmState new_state);
    void runStateMachine();
    void executeInit();
    void setRobotJoint();
    void executeJointControl();
    void executeEndEffectorControl();
    void executeGravityCompensation();

    void controlLoop();
    void checkError();

    void startHoming();

    // Utility functions
    void setdt(double dt) {
    dt_ = dt;
    }

    static void stopArm()
    {
        if (instance) {
            instance->interfaces_ptr_damiao_->Disable();
        }
    }

    static void signalHandler(int signum) {
        if (instance) {
            instance->interfaces_ptr_damiao_->Disable();
            std::this_thread::sleep_for(std::chrono::microseconds(2000));
        }
        std::exit(signum);
    }

    static YamController *instance;

private:
    // Motor interface
    std::shared_ptr<Damiao6dofInterfacesThread> interfaces_ptr_damiao_;
    std::shared_ptr<YamFun> yam_solver_ptr;

    // State machine state
    ArmState current_state_;

    // Target positions for control states
    std::vector<float> target_joint_positions_;
    std::vector<float> target_end_effector_pos_;
    float target_gripper_position_ = 0.0f;

    size_t arm_dof;
	size_t gripper_dof;

    // Control gains
    float arm_kp_1_3_;         // kp for arm joints
    float arm_kp_4_6_;
    float arm_kd_1_3_;
    float arm_kd_4_6_;
    float arm_velocity_gain_;         // kd for arm joints
    float gripper_position_gain_;     // kp for gripper
    float gripper_velocity_gain_;     // kd for gripper
    float go_home_kp_;                // kp for homing
    float go_home_kd_;                // kd for homing

    // Homing control
    bool homing_in_progress_ = true;
    rclcpp::Time homing_start_time_;
    
    // Smooth homing variables
    bool homing_ramp_initialized_ = false;
    float homing_duration_ = 3.0f;  // Total time for homing motion

    const float max_joint_delta = 0.7f; // 40 degree

    std::vector<float> joint_homes_;
    std::vector<float> joint_positions_;
    std::vector<float> joint_velocities_;
    std::vector<float> joint_torques_;
    std::vector<float> gripper_positions_;
    std::vector<float> gripper_homes_;
    std::vector<float> gripper_velocities_;
    std::vector<float> gripper_torques_;

    // Other members
    // JointArrayOTG<6> otg{0.005};  
    rclcpp::Time start_time_;
    // std::vector<LPF1st> filter;
    double dt_ = 0.001;  // control period

    // Publishers and subscribers
    rclcpp::Publisher<yam_arm_msg::msg::YamStatus>::SharedPtr robot_status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_state_publisher_;
    rclcpp::Subscription<yam_arm_msg::msg::YamCmd>::SharedPtr robot_cmd_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;  // Timer for control loop
    rclcpp::TimerBase::SharedPtr state_timer_;  // Timer for pub state

    rclcpp::Time previous_time_; // to store the last call time
};
}  // namespace yam