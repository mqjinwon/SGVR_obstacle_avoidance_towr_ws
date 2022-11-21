#pragma once

#include <iostream>
#include <memory>

#include <vector>
#include <Eigen/Dense>

#include <towr/terrain/grid_map.h> // for collision checking

//kinematics library
#include <towr/kinematics/kinematics_hyq4.h>
#include <towr/kinematics/inverse_kinematics_hyq4.h>

namespace SGVR{

class CollisionChecker{

public:
    struct CollisionSphere{
        using Ptr = std::shared_ptr<CollisionSphere>;

        Eigen::Vector3d center;
        double radius;
        double distFromParent; // distance from parent to me
        bool collisionState;

        double obsDist = 0;
        Eigen::Vector3d gradientDirection;

        CollisionSphere(){};

        CollisionSphere(const Eigen::Vector3d& center_, const double& radius_, const double& distFromParent_) : 
            center(center_), radius(radius_), distFromParent(distFromParent_), collisionState(false), gradientDirection(Eigen::Vector3d::Zero()) {};
    };

    using Ptr = std::shared_ptr<CollisionChecker>;

    using collisionSpheres = std::vector<CollisionSphere::Ptr>;
    using collisionLinks = std::vector<collisionSpheres>;

    ~CollisionChecker();

    CollisionChecker(SGVR::TowrMap::Ptr terrain);

    /**
     * @brief calculate forward matrix [frame, position], p[4,3] at hyq frame
     * @param leg_idx: leg index
     * @param ee_B : end effector position from base frame
     */
    void CalcJointPosMatrix(const int& leg_idx, const Eigen::Vector3d& ee_B);

    /**
     * @brief Create a Collision Spheres object
     * 
     * @param link_idx : link index
     * @param start_frame : start frame
     * @param end_frame : end frame
     * @param radius : radius of collision sphere
     */
    void CreateCollisionSpheres(const int &link_idx,
                                const Eigen::Vector3d &start_frame, 
                                const Eigen::Vector3d &end_frame);

    /**
     * @brief get cost & gradient using sdf
     * 
     * @param link_idx 
     * @param activation_dist : activation distance for collision avoidance, ref: Self-Collision Avoidance and Angular Momentum Compensation for a Biped Humanoid Robot
     * @return Eigen::Vector3d 
     */
    void CollisionChecking(const int &link_idx, double activation_dist = 0.0, double weight = 1.0, bool is_stance = true);

    ////////////
    // setter //
    ////////////
    void SetCollisionsphere(const CollisionSphere::Ptr collision_sphere, const int link_idx, const int sphere_idx){
        collision_links_[link_idx][sphere_idx] = collision_sphere;
    }

    void SetCollisionspheres(const collisionSpheres collision_spheres, const int link_idx){
        collision_links_[link_idx] = collision_spheres;
    }

    ////////////
    // getter //
    ////////////
    const CollisionSphere::Ptr GetCollisionsphere(const int link_idx, const int sphere_idx){
        return collision_links_[link_idx][sphere_idx];
    }

    const collisionSpheres GetCollisionspheres(const int link_idx){
        return collision_links_[link_idx];
    }

    const Eigen::MatrixXd& GetJointPosMatrix(){ return fk_mat_; }

    const Eigen::VectorXd GetJointAngles(){ return joint_q_; }

    const SGVR::HYQ4Kinematics::Ptr GetForwardKinematics(){ return fk_; }

    const double PenaltyFunction(const CollisionSphere::Ptr sphere, const double &activation_dist, const double &weight=1){
        if(sphere->obsDist < activation_dist){
            return weight / 3. * pow(activation_dist - sphere->obsDist, 3.);
        }
        else{
            return 0;
        }
    }
    const Eigen::Vector3d DerivativePenaltyFunction(const CollisionSphere::Ptr sphere, const double &activation_dist, const double &weight=1){
        if(sphere->obsDist < activation_dist){
            return -weight * pow(activation_dist - sphere->obsDist, 2.) * sphere->gradientDirection;
        }
        else{
            return Eigen::Vector3d(0,0,0);
        }
    }

    const int GetNumOfEE() const { return leg_num_; }
    const int GetNumOfLinks() const { return link_num_; }
    const double GetEERadius() const { return ee_radius_; }

private:

    int leg_num_;
    int link_num_;
    std::vector<double> collision_sphere_radius_; // FIXME: define collision sphere radius, can we change it automatically?

    int leg_idx_; // which leg is indicated
    Eigen::VectorXd joint_q_; // joint angles
    Eigen::MatrixXd fk_mat_;

    // kinematics
    SGVR::HYQ4Kinematics::Ptr fk_;
    xpp::InverseKinematicsHyq4 ik_;

    collisionLinks collision_links_;

    double ee_radius_ = 0.021;

    SGVR::TowrMap::Ptr terrain_;
    grid_map::SignedDistanceField sdf_;

};

}// namespace SGVR