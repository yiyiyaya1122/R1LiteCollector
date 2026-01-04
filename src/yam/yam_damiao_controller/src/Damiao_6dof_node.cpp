#include "yam_damiao_controller/Damiao_6dof_node.hpp"
namespace yam
{
    Damiao6dofInterfacesThread::Damiao6dofInterfacesThread(const std::string &can_name)
    : armControl(
        std::vector<damiao::Motor>{
            damiao::Motor(damiao::DM4340, 0x01, 0x11),
            damiao::Motor(damiao::DM4340, 0x02, 0x12),
            damiao::Motor(damiao::DM4340, 0x03, 0x13),
            damiao::Motor(damiao::DM4310, 0x04, 0x14),
            damiao::Motor(damiao::DM4310, 0x05, 0x15),
            damiao::Motor(damiao::DM4310, 0x06, 0x16)
        },
        can_name,
        std::vector<float>{-1.57,   0,      0,      -1.57,  -1.57,  -2},
        std::vector<float>{1.57,    3.65,   3.13,   1.57,   1.57,   2}
      ),
      gripperControl(
        std::vector<damiao::Motor>{
            damiao::Motor(damiao::DM4310, 0x07, 0x17)
        },
        can_name,
        std::vector<float>{-2.7},
        std::vector<float>{0}        
      ),
      ARM_DOF(armControl.get_motors_num()),
      GRIPPER_DOF(gripperControl.get_motors_num())
    {
        // RCLCPP_INFO(rclcpp::get_logger("Damiao6dofInterfacesThread init"));
    }

    Damiao6dofInterfacesThread::~Damiao6dofInterfacesThread() {
        armControl.DisableMotors(); 
        gripperControl.DisableMotors(); 
        std::cout << "Damiao6dofInterfacesThread destroy ok" << std::endl;
    }

    void Damiao6dofInterfacesThread::Init() {
        std::cout << "Start Init armControl" << std::endl;
        armControl.InitMotors();
        std::cout << "Start Init gripperControl" << std::endl;
        gripperControl.InitMotors();
    }

    void Damiao6dofInterfacesThread::Enable(){
        armControl.EnableMotors();
        gripperControl.EnableMotors();
    }

    void Damiao6dofInterfacesThread::Disable(){
        armControl.DisableMotors();
        gripperControl.DisableMotors();
    }

    void Damiao6dofInterfacesThread::SetZeroMotors(){
        // RCLCPP_INFO(rclcpp::get_logger("SetZeroMotors arm"));
        armControl.SetZeroMotors();
        gripperControl.SetZeroMotors();
    }

    void Damiao6dofInterfacesThread::updateYamStatus()
    {
        updateArmStatus();
        updateGripStatus();
    }

    void Damiao6dofInterfacesThread::updateArmStatus()
    {
        armControl.updateMotorState();
    }

    void Damiao6dofInterfacesThread::updateGripStatus()
    {
        gripperControl.updateMotorState();
    }

    bool Damiao6dofInterfacesThread::checkRobotErrors() {
        bool arm_error = armControl.CheckMotorErrors();
        bool gripper_error = gripperControl.CheckMotorErrors();
        return arm_error || gripper_error;
    }

    std::string Damiao6dofInterfacesThread::getErrorsDescription() {
        bool arm_error = armControl.CheckMotorErrors();
        bool gripper_error = gripperControl.CheckMotorErrors();
        std::string error_string;
        
        // Get arm error report
        std::string arm_report = armControl.CheckErrorsDescription();
        if (arm_error) {
            error_string += "=== Arm Errors ===\n";
            error_string += arm_report;
            if (arm_report.back() != '\n') error_string += '\n';  // Ensure newline
        }

        // Get gripper error report
        std::string gripper_report = gripperControl.CheckErrorsDescription();
        if (gripper_error) {
            error_string += "=== Gripper Errors ===\n";
            error_string += gripper_report;
            if (gripper_report.back() != '\n') error_string += '\n';  // Ensure newline
        }

        // Handle case where no errors exist
        if (error_string.empty()) {
            error_string = "No errors detected in arm or gripper";
        }

        return error_string;
    }

    std::vector<float> Damiao6dofInterfacesThread::getJointPositions(){
        return armControl.current_motor_pos;
    }
    std::vector<float> Damiao6dofInterfacesThread::getJointVelocities(){
        return armControl.current_motor_vel;
    }
    std::vector<float> Damiao6dofInterfacesThread::getJointCurrent(){
        return armControl.current_motor_tau;
    }
    

    // remap gripper position
    std::vector<float> Damiao6dofInterfacesThread::getGripperPositions() {
        std::vector<float> result;
        result.reserve(gripperControl.current_motor_pos.size());
        
        // Apply the mapping function to each element in the vector
        for (const auto& pos : gripperControl.current_motor_pos) {
            result.push_back(static_cast<float>(gripper_map.theta2cmd(pos)));
        }
        
        return result;
    }

    // remap gripper vel
    std::vector<float> Damiao6dofInterfacesThread::getGripperVelocities() {
        std::vector<float> result;
        result.reserve(gripperControl.current_motor_vel.size());
        
        for (const auto& vel : gripperControl.current_motor_vel) {
            result.push_back(-vel);
        }
        
        return result;
    }

    // remap gripper tor
    std::vector<float> Damiao6dofInterfacesThread::getGripperCurrent() {
        std::vector<float> result;
        result.reserve(gripperControl.current_motor_tau.size());
        
        for (const auto& tau : gripperControl.current_motor_tau) {
            result.push_back(-tau);
        }
        
        return result;
    }

    void Damiao6dofInterfacesThread::setJointPosition(const std::vector<float> target_pos,const std::vector<float> target_vel,const std::vector<float> P,const std::vector<float> D,const std::vector<float> tor){
        armControl.MitCtrl(target_pos,target_vel,P,D,tor);
    }

    // remap gripper position
    void Damiao6dofInterfacesThread::setGripperPosition(const std::vector<float> target_pos,const std::vector<float> target_vel,const std::vector<float> P,const std::vector<float> D,const std::vector<float> tor) {
        std::vector<float> result;
        result.reserve(target_pos.size());
        
        // Apply the mapping function to each element in the vector
        for (const auto& pos : target_pos) {
            // Clamp the position to [0, 5] range
            float clamped_pos = std::clamp(pos, 0.0f, 5.0f);
            result.push_back(static_cast<float>(gripper_map.cmd2theta(clamped_pos)));
        }
        gripperControl.MitCtrl(result,target_vel,P,D,tor);
        // gripperControl.PosCtrl(result,target_vel);
    }

    // Eigen::VectorXd Damiao6dofInterfacesThread::getEndPose(){
    //     auto target_pm=arm6dof.ARX_R5_FK(thetalist);
    //     double pm[16]{0};
    //     double pq[7]{ 0 };
    //     for (int i = 0; i < 4; i++) {
    //         for (int  j= 0; j < 4; j++)
    //         {
    //             pm[i+j] =target_pm(i,j);
    //         }
    //     }
    //     s_pm2pq(pm,pq);
    //     Eigen::VectorXd target_qp=Eigen::VectorXd::Zero(7);
    //     for (size_t i = 0; i < 7; i++)
    //     {
    //         target_qp(i)=pq[i];
    //     }
    //     return target_qp;
    // }
    
}


