#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace pin = pinocchio;

class YamFun {
public:
    YamFun(const std::string& urdf_path);
    // Forward kinematics - returns position and orientation as quaternion
    void computeEEFinNorm(const Eigen::VectorXd& q, 
                   Eigen::Vector3d& position, 
                   Eigen::Quaterniond& orientation);
    
    // Inverse kinematics - takes position and orientation as quaternion
    bool computeIK(const Eigen::Vector3d& target_pos,
                   const Eigen::Quaterniond& target_quat,
                   Eigen::VectorXd& result, 
                   int max_iter = 3e2,
                   double eps = 5e-3,
                   double dt = 1e-1);

    /// Compute gravity compensation torques for configuration q.
    /// Returns a vector of size model.nv (generalized torques).
    Eigen::VectorXd gravityCompensation(const Eigen::VectorXd& q);

    /// Convenience: gravity compensation at the stored current config.
    Eigen::VectorXd gravityCompensationCurrent() const;

    Eigen::VectorXd getCurrentConfig() const;
    void setCurrentConfig(const Eigen::VectorXd& q);
    void resetConfig();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointLimits() const;
    int getNumJoints() const;

private:
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex end_effector_id;
    Eigen::VectorXd q_init;
    Eigen::VectorXd q_current;
    // const Eigen::Quaterniond base2eef_quat{0.5, 0.5, 0.5, 0.5};   // w x y z in rviz
    // const Eigen::Matrix3d base2eef_mat = base2eef_quat.toRotationMatrix();
    // const Eigen::Vector3d eef_pos{0.110297, 0, 0.164001};    // x y z in rviz

    Eigen::Quaterniond base2eef_quat;   // w x y z in rviz
    Eigen::Matrix3d base2eef_mat;
    Eigen::Vector3d eef_pos;    // x y z in rviz
};
