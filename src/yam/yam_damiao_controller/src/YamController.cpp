#include "yam_damiao_controller/YamController.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>

namespace yam {

	YamController::YamController() : Node("yam_controller_node"), current_state_(ArmState::INIT) {
		rclcpp::on_shutdown(std::bind(&YamController::cleanup, this));

		// Parameters
		std::string arm_control_type = this->declare_parameter("arm_control_type", "normal");
		std::string package_name = "yam_description";
		std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);

		std::string urdf_path = package_share_dir + "/urdf/" + "yam.urdf";
        yam_solver_ptr = std::make_shared<YamFun>(urdf_path);

		// Motor interface setup
		interfaces_ptr_damiao_ = 
		std::make_shared<Damiao6dofInterfacesThread>(this->declare_parameter("arm_can_id", "/dev/ttyACM0"));

		arm_dof = interfaces_ptr_damiao_->ARM_DOF;
		gripper_dof = interfaces_ptr_damiao_->GRIPPER_DOF;

        joint_homes_ = std::vector<float>(arm_dof, 0.0f);
        gripper_homes_ = std::vector<float>(gripper_dof, 0.0f);

        joint_positions_ = std::vector<float>(arm_dof, 0.0f);
        joint_velocities_= std::vector<float>(arm_dof, 0.0f);
        joint_torques_ = std::vector<float>(arm_dof, 0.0f);
        gripper_positions_ = std::vector<float>(gripper_dof, 0.0f);
        gripper_velocities_ = std::vector<float>(gripper_dof, 0.0f);
        gripper_torques_ = std::vector<float>(gripper_dof, 0.0f);

        target_joint_positions_ = std::vector<float>(arm_dof, 0.0f);
        target_end_effector_pos_ = std::vector<float>{0, 0, 0, 1, 0, 0, 0};    // xyz_wxyz

		// Gain parameters
		arm_kp_1_3_ = this->declare_parameter("arm_kp_1_3", 100.0f);
        arm_kp_4_6_ = this->declare_parameter("arm_kp_4_6", 50.0f);
        arm_kd_1_3_ = this->declare_parameter("arm_kd_1_3", 1.0f);
        arm_kd_4_6_ = this->declare_parameter("arm_kd_4_6", 1.0f);
		gripper_position_gain_ = this->declare_parameter("gripper_position_gain", 60.0f);
		gripper_velocity_gain_ = this->declare_parameter("gripper_velocity_gain", 1.0f);
		go_home_kp_ = this->declare_parameter("go_home_kp", 15.0f);
		go_home_kd_ = this->declare_parameter("go_home_kd", 0.5f);

		// Control setup
		double control_period = this->declare_parameter("arm_control_period", 0.005);
		setdt(control_period);

        previous_time_ = this->get_clock()->now();


        
		// Initialize motor interface
		interfaces_ptr_damiao_->Disable();
		robot_init();

		// Setup publishers/subscribers/timers
		if (arm_control_type == "normal") {
			robot_status_publisher_ = this->create_publisher<yam_arm_msg::msg::YamStatus>(
				this->declare_parameter("arm_pub_topic_name", "arm_status"), 1);

			state_timer_ = this->create_wall_timer(std::chrono::milliseconds(3), 
				std::bind(&YamController::publishState, this));
			RCLCPP_INFO(this->get_logger(), "Robot Status Pub created!");

			// Control timer at specified period
			transitionToState(ArmState::INIT);
			control_timer_ = this->create_wall_timer(
				std::chrono::duration<double>(dt_), 
				std::bind(&YamController::controlLoop, this));
			RCLCPP_INFO(this->get_logger(), "Robot ControlLoop created!");

			robot_cmd_subscriber_ = this->create_subscription<yam_arm_msg::msg::YamCmd>(
				this->declare_parameter("arm_sub_topic_name", "arm_cmd"), 1,
				std::bind(&YamController::cmdCallback, this, std::placeholders::_1));
			RCLCPP_INFO(this->get_logger(), "Robot Cmd Sub created!");

            // rviz_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            //     "/joint_states", 1);
            // RCLCPP_INFO(this->get_logger(), "Rviz Joint State Pub created!");
		}
		else {
			RCLCPP_ERROR(this->get_logger(), "Unknown arm control type: %s", arm_control_type.c_str());
		}
	}

	void YamController::robot_init(){
		interfaces_ptr_damiao_->Init();
	}

    void YamController::startHoming() {
        RCLCPP_INFO(this->get_logger(), "Homing started");
    }

	void YamController::transitionToState(ArmState new_state) {
	// Handle state transitions
	switch (new_state) {
		case ArmState::SOFT:
			interfaces_ptr_damiao_->Disable();
			RCLCPP_INFO(this->get_logger(), "Transitioning to SOFT state, restart to resume!!!");
			break;
			
		case ArmState::INIT:
			RCLCPP_INFO(this->get_logger(), "Transitioning to INIT state");
			startHoming();
			// Initialize homing procedure
		break;
			
        case ArmState::G_COMPENSATION:
			RCLCPP_INFO(this->get_logger(), "Transitioning to G_COMPENSATION state");
			break;            

		case ArmState::JOINT_CONTROL:
			RCLCPP_INFO(this->get_logger(), "Transitioning to JOINT_CONTROL state");
			break;
			
        case ArmState::END_CONTROL:
        	RCLCPP_INFO(this->get_logger(), "Transitioning to END_CONTROL state");
			break;
			
		// Add other state transitions as needed
		default:
			RCLCPP_WARN(this->get_logger(), "Transitioning to unimplemented state: %d", 
						static_cast<int>(new_state));
		}
		current_state_ = new_state;
	}

  void YamController::cmdCallback(const yam_arm_msg::msg::YamCmd::SharedPtr msg) {

    if (current_state_ != ArmState::SOFT && current_state_ != ArmState::INIT){
      // Map message modes to state machine states
        switch(msg->mode) {
            case 0:  // SOFT (disable motors) 
                transitionToState(ArmState::SOFT);
                break;
                
            // case 1:  // INIT
            //     transitionToState(ArmState::INIT);
            //     break;
                
            case 3:
                transitionToState(ArmState::G_COMPENSATION);
                break;

            case 4:  // END_CONTROL
                if (current_state_ != ArmState::END_CONTROL) {
                    transitionToState(ArmState::END_CONTROL);
                }
                for (int i = 0; i < 7; i++) {
                    target_end_effector_pos_[i] = msg->end_pose[i];
                }
                target_gripper_position_ = static_cast<float>(msg->gripper);
                break;
                
            case 5:  // JOINT_CONTROL
                if (current_state_ != ArmState::JOINT_CONTROL) {
                    transitionToState(ArmState::JOINT_CONTROL);
                }
                // Store target joint positions
                for (int i = 0; i < 6; i++) {
                    target_joint_positions_[i] = static_cast<float>(msg->joint_pos[i]);
                }
                target_gripper_position_ = static_cast<float>(msg->gripper);
                break;
                
            // case 6:  // ZERO (calibration)
            //     transitionToState(ArmState::ZERO);
            //     break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown control mode: %d", msg->mode);
        }
    }
  }

  void YamController::controlLoop() {
    // Run state machine
    runStateMachine();
  }

  void YamController::runStateMachine() {
      // Execute current state behavior
      switch (current_state_) {
        case ArmState::SOFT:
        break;
            
        case ArmState::INIT:
        executeInit();
        break;
            
        case ArmState::END_CONTROL:
        checkError();
        executeEndEffectorControl();
        break;
            
        case ArmState::JOINT_CONTROL:
        checkError();
        executeJointControl();
        break;

        case ArmState::G_COMPENSATION:
        checkError();
        executeGravityCompensation();
        break;

        //   case ArmState::ZERO:
        //     checkError();
        //     executeZeroing();
        //     break;
              
          // Add other state behaviors as needed
        default:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Unimplemented state behavior: %d", 
                                static_cast<int>(current_state_));
      }
    }

    void YamController::executeInit() {
        if (homing_in_progress_) {

            // Initialize homing ramp if not done yet
            if (!homing_ramp_initialized_) {
                homing_start_time_ = this->now();
                homing_ramp_initialized_ = true;
            }

            auto elapsed = this->now() - homing_start_time_;
            float elapsed_seconds = elapsed.seconds();
            
            if (elapsed_seconds > homing_duration_) {
                RCLCPP_INFO(this->get_logger(), "Homing complete");
                // Update robot status to get current positions
                interfaces_ptr_damiao_->updateYamStatus();
                auto current_joint_positions = interfaces_ptr_damiao_->getJointPositions();
                auto current_gripper_position = interfaces_ptr_damiao_->getGripperPositions();
                
                // Check if all joints are at home position (within tolerance)
                const float POSITION_TOLERANCE = 0.1f; // Adjust tolerance as needed
                bool all_joints_homed = true;
                
                for (size_t i = 0; i < current_joint_positions.size(); i++) {
                    if (fabs(current_joint_positions[i]) > POSITION_TOLERANCE) {
                        RCLCPP_ERROR(this->get_logger(),
                                    "Joint %zu not at home position: %.6f",
                                    i, (double)current_joint_positions[i]);
                        all_joints_homed = false;
                    }
                }
                
                for (size_t i = 0; i < current_gripper_position.size(); i++) {
                    if (fabs(current_gripper_position[i]) > POSITION_TOLERANCE) {
                        RCLCPP_ERROR(this->get_logger(), 
                                    "Gripper joint %zu not at home position", i);
                        all_joints_homed = false;
                    }
                }
                
                if (!all_joints_homed) {
                    RCLCPP_ERROR(this->get_logger(), "Homing failed - not all joints reached home position");
                    // Transition to error state or take appropriate action
                    transitionToState(ArmState::SOFT);
                    homing_ramp_initialized_ = false;
                    return;
                }
                
                homing_in_progress_ = false;
                RCLCPP_INFO(this->get_logger(), "Homing complete - all joints at home position");
            }
            else {

                interfaces_ptr_damiao_->setJointPosition(
                    joint_homes_,
                    std::vector<float>(arm_dof, 0.0f),
                    std::vector<float>(arm_dof, go_home_kp_),
                    std::vector<float>(arm_dof, go_home_kd_),
                    std::vector<float>(arm_dof, 0.0f)
                );
                
                interfaces_ptr_damiao_->setGripperPosition(
                    gripper_homes_,
                    std::vector<float>(gripper_dof, 0.0f),
                    std::vector<float>(gripper_dof, gripper_position_gain_),
                    std::vector<float>(gripper_dof, gripper_velocity_gain_),
                    std::vector<float>(gripper_dof, 0.0f)
                );
            }
        }
        else {
            // Init finished
            transitionToState(ArmState::JOINT_CONTROL);
            return;
        }
    }

    void YamController::setRobotJoint() {

        bool delta_too_big = false;

        // Iterate through the joint positions to check for excessive delta.
        // delta check for first 4 motors
        for (size_t i = 0; i < target_joint_positions_.size() - 2; ++i) {
            if (std::abs(target_joint_positions_[i] - joint_positions_[i]) > max_joint_delta) {
                delta_too_big = true;
                RCLCPP_ERROR(this->get_logger(), "Joint Delta Too Big!");
                break; // Exit the loop as soon as one large delta is found.
            }
        }

        // If a large delta was found, use the current joint positions to avoid a jump.
        if (delta_too_big) {
            target_joint_positions_ = joint_positions_;
        }

        Eigen::VectorXd current_q = Eigen::Map<Eigen::VectorXf>(
            joint_positions_.data(), joint_positions_.size()
        ).cast<double>();

        Eigen::VectorXf target_tau = yam_solver_ptr->gravityCompensation(current_q).cast<float>();

        std::vector<float> target_tau_gc(target_tau.data(), target_tau.data() + target_tau.size());

        interfaces_ptr_damiao_->setJointPosition(
            {target_joint_positions_[0], target_joint_positions_[1], target_joint_positions_[2],
            target_joint_positions_[3], target_joint_positions_[4], target_joint_positions_[5]},
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>{arm_kp_1_3_, arm_kp_1_3_, arm_kp_1_3_, arm_kp_4_6_, arm_kp_4_6_, arm_kp_4_6_},
            std::vector<float>{arm_kd_1_3_, arm_kd_1_3_, arm_kd_1_3_, arm_kd_4_6_, arm_kd_4_6_, arm_kd_4_6_},
            target_tau_gc
        );

        interfaces_ptr_damiao_->setGripperPosition(
            {target_gripper_position_},
            std::vector<float>(gripper_dof, 0.0f),
            std::vector<float>(gripper_dof, gripper_position_gain_),
            std::vector<float>(gripper_dof, gripper_velocity_gain_),
            std::vector<float>(gripper_dof, 0.0f)
        );


        // auto joint_cmd_msg = sensor_msgs::msg::JointState();

        // // timestamp
        // joint_cmd_msg.header.stamp = this->get_clock()->now();

        // // prepare sizes
        // size_t n = target_joint_positions_.size();
        // joint_cmd_msg.name.resize(n);
        // joint_cmd_msg.position.resize(n);
        // joint_cmd_msg.velocity.assign(n, 0.0); // zero velocities
        // joint_cmd_msg.effort.resize(n);

        // // fill positions (and efforts if available)
        // for (size_t i = 0; i < n; ++i) {
        //     // fallback joint names: "joint1", "joint2", ...
        //     joint_cmd_msg.name[i] = std::string("joint") + std::to_string(i + 1);

        //     // convert float -> double for message
        //     joint_cmd_msg.position[i] = static_cast<double>(target_joint_positions_[i]);

        //     // // efforts: use target_tau_gc if it has matching data, otherwise 0.0
        //     // if (i < target_tau_gc.size()) {
        //     //     joint_cmd_msg.effort[i] = static_cast<double>(target_tau_gc[i]);
        //     // } else {
        //     //     joint_cmd_msg.effort[i] = 0.0;
        //     // }
        // }

        // rviz_joint_state_publisher_->publish(joint_cmd_msg);

        // // Calculate time interval and log
        // rclcpp::Time current_time = this->get_clock()->now();
        // rclcpp::Duration time_interval = current_time - previous_time_;
        // previous_time_ = current_time;  // Update the previous time to the current one

        // // Log the time interval every time the function is called
        // RCLCPP_INFO(this->get_logger(), "setRobotJoint Time interval: %.7f seconds", time_interval.seconds());

    }

    void YamController::executeGravityCompensation() {
        Eigen::VectorXd current_q = Eigen::Map<Eigen::VectorXf>(
            joint_positions_.data(), joint_positions_.size()
        ).cast<double>();

        yam_solver_ptr->setCurrentConfig(current_q);

        Eigen::VectorXf current_tau = yam_solver_ptr->gravityCompensationCurrent().cast<float>();

        std::vector<float> target_tau_gc(current_tau.data(), current_tau.data() + current_tau.size());

        // std::cout << "target_tau_gc: [";
        // for (size_t i = 0; i < target_tau_gc.size(); ++i) {
        //     std::cout << target_tau_gc[i];
        //     if (i + 1 < target_tau_gc.size()) std::cout << ", ";
        // }
        // std::cout << "]" << std::endl;

        interfaces_ptr_damiao_->setJointPosition(
            {joint_positions_[0], joint_positions_[1], joint_positions_[2],
            joint_positions_[3], joint_positions_[4], joint_positions_[5]},
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>(arm_dof, 0.0f),
            std::vector<float>(arm_dof, 0.0f),
            target_tau_gc
        );

        interfaces_ptr_damiao_->setGripperPosition(
            {gripper_positions_},
            std::vector<float>(gripper_dof, 0.0f),
            std::vector<float>(gripper_dof, gripper_position_gain_),
            std::vector<float>(gripper_dof, gripper_velocity_gain_),
            std::vector<float>(gripper_dof, 0.0f)
        );
    }

    void YamController::executeJointControl() {
        setRobotJoint();
    }

    void YamController::executeEndEffectorControl() {
        std::vector<float> xyz_wxyz = target_end_effector_pos_;
        // Extract position and quaternion from the vector
        Eigen::Vector3d xyz(xyz_wxyz[0], xyz_wxyz[1], xyz_wxyz[2]);
        Eigen::Quaterniond quat(xyz_wxyz[3], xyz_wxyz[4], xyz_wxyz[5], xyz_wxyz[6]);

        Eigen::VectorXd current_q = Eigen::Map<Eigen::VectorXf>(
            joint_positions_.data(), joint_positions_.size()
        ).cast<double>();
        yam_solver_ptr->setCurrentConfig(current_q);

        auto start_time = std::chrono::high_resolution_clock::now();

        Eigen::VectorXd q;
        bool ik_success = yam_solver_ptr->computeIK(xyz, quat, q);

        // End timing and calculate duration
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        if (ik_success) {
            target_joint_positions_ = std::vector<float>(q.data(), q.data() + q.size());
        }
        else {
            RCLCPP_WARN(this->get_logger(), "IK computation failed");
            RCLCPP_INFO(
            this->get_logger(), 
            "IK computation time: %.4f milliseconds", 
            duration.count() / 1000.0
        );
        }

        setRobotJoint();
    }

	void YamController::publishState() {
        interfaces_ptr_damiao_->updateYamStatus();
        
        joint_positions_ = interfaces_ptr_damiao_->getJointPositions();
        joint_velocities_ = interfaces_ptr_damiao_->getJointVelocities();
        joint_torques_ = interfaces_ptr_damiao_->getJointCurrent();
        gripper_positions_ = interfaces_ptr_damiao_->getGripperPositions();
        gripper_velocities_ = interfaces_ptr_damiao_->getGripperVelocities();
        gripper_torques_ = interfaces_ptr_damiao_->getGripperCurrent();

        auto robot_status_msg = yam_arm_msg::msg::YamStatus();
        robot_status_msg.header.stamp = this->get_clock()->now();

        Eigen::VectorXd q(6);
        for (size_t i = 0; i < 6; ++i) {
            q(i) = static_cast<double>(joint_positions_[i]);
        }

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        yam_solver_ptr->computeEEFinNorm(q, position, orientation);

        // The message expects [x, y, z, w, qx, qy, qz].
        robot_status_msg.end_pose[0] = position.x();
        robot_status_msg.end_pose[1] = position.y();
        robot_status_msg.end_pose[2] = position.z();
        robot_status_msg.end_pose[3] = orientation.w();
        robot_status_msg.end_pose[4] = orientation.x();
        robot_status_msg.end_pose[5] = orientation.y();
        robot_status_msg.end_pose[6] = orientation.z();

        for (int i = 0; i < 6; i++) {
            robot_status_msg.joint_pos[i] = joint_positions_[i];
            robot_status_msg.joint_vel[i] = joint_velocities_[i];
            robot_status_msg.joint_cur[i] = joint_torques_[i];
        }
        robot_status_msg.joint_pos[6] = gripper_positions_[0];
        robot_status_msg.joint_vel[6] = gripper_velocities_[0];
        robot_status_msg.joint_cur[6] = gripper_torques_[0];

        robot_status_publisher_->publish(robot_status_msg);
	}

	void YamController::checkError() {
		bool robot_error = interfaces_ptr_damiao_->checkRobotErrors();
		std::string error_msg;

		if (robot_error) {
			error_msg = interfaces_ptr_damiao_->getErrorsDescription();
			RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
			transitionToState(ArmState::SOFT);
		}
	}

	void YamController::cleanup() {
		RCLCPP_INFO(this->get_logger(), "Shutting down, disabling motors");
		interfaces_ptr_damiao_->Disable();
	}
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<yam::YamController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
