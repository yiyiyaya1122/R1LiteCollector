//creat by zhoujie 2025-6-10 
//meidi damiao 6dof arm control node 
//anxis 4310  4330  4330 4310 4310 4310 grip
#pragma once
#include <mutex>
#include <vector>
#include <iostream>
#include <memory>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> // 包含变换和位姿处理

// #include "mathematical_model/ARX_R5_fun.hpp"
#include "drivers/motor_control.hpp"

namespace yam
{
    class Damiao6dofInterfacesThread
    {
    public:
        Damiao6dofInterfacesThread(const std::string &can_name = "/dev/ttyACM0");
        ~Damiao6dofInterfacesThread();
        void Init();
        void Enable();
        void Disable();
        void SetZeroMotors();
        void updateYamStatus();
        void updateArmStatus();
        void updateGripStatus();
        bool checkRobotErrors();
        std::string getErrorsDescription();
        void switchMotorControlMode(const damiao::Control_Mode mode);

        std::vector<float> getJointPositions();
        std::vector<float> getJointVelocities();
        std::vector<float> getJointCurrent();

        std::vector<float> getGripperPositions();
        std::vector<float> getGripperVelocities();
        std::vector<float> getGripperCurrent();

        Eigen::VectorXd getEndPose();

        MotorControl armControl;
        MotorControl gripperControl;

        const size_t ARM_DOF;
        const size_t GRIPPER_DOF; 
        
        void setJointPosition(const std::vector<float> target_pos,const std::vector<float> target_vel,const std::vector<float> P,const std::vector<float> D,const std::vector<float> tor);
        void setGripperPosition(const std::vector<float> target_pos,const std::vector<float> target_vel,const std::vector<float> P,const std::vector<float> D,const std::vector<float> tor);
        
    private:
        class {
        public:
            const double COS_2_7 = -0.904072;
            const float A = 5.0 / (1.0 - COS_2_7);
            const float C = 5.0 - A;
            const float B = M_PI/2 - 2.7;

            double theta2cmd(float theta) {
                return A * sin(-theta + B) + C;
            }

            double cmd2theta(float cmd) {
                double arg = (cmd - C) / A;
                // Clamp the argument to avoid domain errors
                if (arg > 1.0) arg = 1.0;
                if (arg < -1.0) arg = -1.0;
                
                return B - std::asin(arg);
            }
        } gripper_map;

        // void setEndPose(Eigen::Isometry3d input);

        // void setJointPositions(std::vector<double> positions);

        // void setCatch(double position);

        // void setCatchTorque(double torque);

        // void setCatchActionFast();
        // void setCatchActionSlow();
        // void arx_x(double arx1, double arx2, double arx3);

        // void setEndEffectorMass(double mass);
        // std::vector<int> getErrorCode();
    };
}

