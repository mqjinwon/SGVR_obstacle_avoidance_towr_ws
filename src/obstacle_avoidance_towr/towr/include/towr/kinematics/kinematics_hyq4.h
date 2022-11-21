#pragma once

#include <Eigen/Core>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>

// for forward kinematics
#include <kdl/chainfksolverpos_recursive.hpp>

// for jacobian solver
#include <kdl/chainjnttojacsolver.hpp>

namespace SGVR{

class HYQ4Kinematics {
    /**
     * @brief kinematics for the HYQ4 robot.
     * @details Assume robot base is at the origin of the world frame.
     * 
     */

public:
    using Ptr = std::shared_ptr<HYQ4Kinematics>;

    /**
     * @brief Construct a new HYQ4 Kinematics object
     * 
     * @param robot_description: robot urdf file
     */
    HYQ4Kinematics(const std::string& robot_description);

    /**
     * @brief Get Forward Kinematics position value
     * 
     * @param ee: end effector index [0, 3]
     * @param frame_id: frame index [0, 3]
     * @param joint_q: joint angles
     * @param length: length of the end effector
     */
    Eigen::Vector3d getFramePosition(const int ee, int frame_id, const Eigen::VectorXd& joint_q, const double length=0);

    Eigen::MatrixXd getLegFramePositionMatrix(const int ee, const Eigen::VectorXd joint_q);

    std::vector<Eigen::MatrixXd> getFramePositionMatrix(const Eigen::MatrixXd joint_q_mat);

    KDL::Jacobian MakeJacobian(const int& leg_idx, const int& link_idx, const Eigen::VectorXd& joint_q, const double length=0);

    void ChangeLinkLength(KDL::Chain& chain_, const int& segIdx,const KDL::Vector& changePos);

    double CheckReachAbility(const Eigen::Vector3d& target_point, const int& leg_idx, const int& link_idx, const double length=0, const double reach_ratio=0.8);

    const int GetNumOfEE() const { return ee_name_.size(); }
    const int GetNumOfLinks() const { return frame_name_.size(); }

private:
    KDL::Tree my_tree_;
    KDL::Chain my_chain_;
    KDL::Frame eeFrame_;

    std::vector<std::string> ee_name_;
    std::vector<std::string> frame_name_;
};

}// namespace SGVR