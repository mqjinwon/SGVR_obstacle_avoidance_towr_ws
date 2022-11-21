#include <towr/collision/collision_checker.h>

namespace SGVR{

CollisionChecker::CollisionChecker(SGVR::TowrMap::Ptr terrain):
    terrain_(terrain), sdf_(terrain->getSDF())
{

    std::string hyq_path = PACKAGE_PATH;
    hyq_path.append("/../towr/include/towr/kinematics/hyq.urdf");

    fk_ = std::make_shared<SGVR::HYQ4Kinematics>(hyq_path);

    leg_num_ = fk_->GetNumOfEE();
    link_num_ = fk_->GetNumOfLinks();

    collision_links_.reserve(link_num_);

    for(int i = 0; i < link_num_; i++){
        collision_links_.push_back(collisionSpheres());
    }

    collision_sphere_radius_.reserve(link_num_);
    collision_sphere_radius_.push_back(0.03);
    collision_sphere_radius_.push_back(0.03);
    collision_sphere_radius_.push_back(0.03);
    collision_sphere_radius_.push_back(0.015);
}

CollisionChecker::~CollisionChecker(){
}

void CollisionChecker::CalcJointPosMatrix(const int& leg_idx, const Eigen::Vector3d& ee_B){
    leg_idx_ = leg_idx;
    joint_q_ =  ik_.GetJointAngles(leg_idx_, ee_B);
    fk_mat_ = fk_->getLegFramePositionMatrix(leg_idx_, joint_q_);
}

void CollisionChecker::CreateCollisionSpheres(const int &link_idx,
                                            const Eigen::Vector3d &start_frame, 
                                            const Eigen::Vector3d &end_frame){

    auto start_frame_ = start_frame;
    start_frame_[2] += ee_radius_;
    auto end_frame_ = end_frame;
    end_frame_[2] += ee_radius_;

    double radius = collision_sphere_radius_[link_idx];

    Eigen::Vector3d line_vector = end_frame_ - start_frame_;
    
    // normalize line vector & length to radius
    Eigen::Vector3d radius_vector = line_vector.normalized() * radius;

    Eigen::Vector3d radius_point = start_frame_;

    // create collision spheres
    collisionSpheres collision_spheres;
    CollisionSphere::Ptr collision_sphere;

    while(1){
        double obsDist = (end_frame_ - radius_point).norm();
        // last sphere
        if ( obsDist < 2*radius){
            // TODO: how to make last collision sphere?
            radius_point += radius_vector;
            collision_sphere = std::make_shared<CollisionSphere>(radius_point, radius, (radius_point - start_frame_).norm());
            // collision_sphere = CollisionSphere(radius_point + (end_frame_ - radius_point)/2.,
            //                                                     obsDist/2., (radius_point + (end_frame_ - radius_point)/2. - start_frame_).norm());
            collision_spheres.push_back(collision_sphere);
            break;
        }
        else{
            // move to center of sphere
            radius_point += radius_vector;

            collision_sphere = std::make_shared<CollisionSphere>(radius_point, radius, (radius_point - start_frame_).norm());
            collision_spheres.push_back(collision_sphere);

            // move to start point of next sphere
            radius_point += radius_vector;
        }
    }

    SetCollisionspheres(collision_spheres, link_idx);
}

void CollisionChecker::CollisionChecking(const int &link_idx, double activation_dist, double weight, bool is_stance){

    for (int sphere_idx = 0 ; sphere_idx <GetCollisionspheres(link_idx).size(); sphere_idx++){

        CollisionSphere::Ptr sphere = GetCollisionsphere(link_idx, sphere_idx);
        double radius = sphere->radius;

        double obsDist = activation_dist;
        Eigen::Vector3d gradientDirection(0,0,0);
        bool collisionFlag = false;

        grid_map::Position3 pos;
        pos.x() = sphere->center.x();
        pos.y() = sphere->center.y();
        pos.z() = sphere->center.z();

        double sdf_distance = sdf_.getInterpolatedDistanceAt(pos);
        obsDist = sdf_distance - radius;
        if(obsDist < activation_dist){
            gradientDirection = sdf_.getDistanceGradientAt(pos);
            collisionFlag = true;
        }
        else{
            gradientDirection.setZero();
            collisionFlag = false;
        }

        if(is_stance){
            gradientDirection.z() = 0;
            if(gradientDirection.norm() < 1e-6){
                gradientDirection.setZero();
                obsDist = 0;
                collisionFlag = false;
            }
        }

        // greatest norm value among sphere edge points
//        for(int i=0; i<5; i++){
//
//            pos.x() = sphere->center.x();
//            pos.y() = sphere->center.y();
//            pos.z() = sphere->center.z();
//
//            switch (i)
//            {
//            case 0:
//                pos.x() += radius;
//                break;
//            case 1:
//                pos.x() -= radius;
//                break;
//            case 2:
//                pos.y() += radius;
//                break;
//            case 3:
//                pos.y() -= radius;
//                break;
//            case 4:
//                pos.z() -= radius;
//                break;
//            default:
//                break;
//            }
//
//            double tmpDist = sdf_.getInterpolatedDistanceAt(pos);
//
//            // 거리가 멀면 true
//            if ( tmpDist < obsDist ){
//                obsDist = tmpDist;
//                gradientDirection = sdf_.getDistanceGradientAt(pos);
//                collisionFlag = true;
//            }
//        }

        // Ref:Self-Collision Avoidance and Angular Momentum Compensation for a Biped Humanoid Robot
        sphere->obsDist = obsDist;
        sphere->gradientDirection = gradientDirection;
        sphere->collisionState = collisionFlag;
    }
}

}