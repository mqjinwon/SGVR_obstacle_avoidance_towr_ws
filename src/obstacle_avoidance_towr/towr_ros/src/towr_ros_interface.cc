/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/towr_ros_interface.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>
#include <xpp_msgs/SphereObstacle.h>  // sphere obstacle message

#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

namespace towr {

TowrRosInterface::TowrRosInterface ()
{

  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                  &TowrRosInterface::UserCommandCallback, this);

  initial_state_pub_  = n.advertise<xpp_msgs::RobotStateCartesian>
                                          (xpp_msgs::robot_state_desired, 1);

  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);

  grid_map_pub_ = n.advertise<grid_map_msgs::GridMap>
                                    ("/sgvr/grid_map", 1);
  point_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>
                                    ("/sgvr/point_cloud", 1);

  solver_ = std::make_shared<ifopt::IpoptSolver>();

  visualization_dt_ = 0.01;

  gen = std::mt19937(std::random_device()());

  pre_terrain_id_ = -1;
}

BaseState
TowrRosInterface::GetGoalState(const TowrCommandMsg& msg) const
{
  BaseState goal;
  goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
  goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
  goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);

  return goal;
}

void
TowrRosInterface::UserCommandCallback(const TowrCommandMsg& msg)
{
  // robot model
  formulation_.model_ = RobotModel(static_cast<RobotModel::Robot>(msg.robot));
  auto robot_params_msg = BuildRobotParametersMsg(formulation_.model_);
  robot_parameters_pub_.publish(robot_params_msg);

  // terrain
  // if terrain id change, make new terrain
  if(msg.terrain != pre_terrain_id_){
    std::string elevationLayerName_ = "elevation";
    std::vector<std::string> layerNames_ = {elevationLayerName_};
    HeightMap::TerrainID terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
    formulation_.terrain_ = std::make_shared<SGVR::TowrMap>(terrain_id, layerNames_);

    double dxy   =  0.01; // 1cm resolution
    double x_min =  -1.0;
    double x_max =  4.0;
    double y_min =  -1.0;
    double y_max =  1.0;
    formulation_.terrain_->gridMapInit("world", 
                                      grid_map::Length(x_max - x_min, y_max - y_min), dxy, 
                                      grid_map::Position((x_min+x_max)/2.0, (y_min+y_max)/2.0));
    formulation_.terrain_->updateGridMap(elevationLayerName_);   //update grid map from height map information
    formulation_.terrain_->updateSDF(elevationLayerName_, 0.1);  //update sdf from grid map information, margin z=0.1
  }
  
  pre_terrain_id_ = msg.terrain; // save previous terrain id

  collision_checker_ = std::make_shared<SGVR::CollisionChecker>(formulation_.terrain_);

  int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
  formulation_.params_ = GetTowrParameters(n_ee, msg);
  formulation_.final_base_ = GetGoalState(msg);

  // if not the visualization mode
  if(!msg.replay_trajectory){
    ////////////////////////////
    ///// Publish grid map /////
    ////////////////////////////
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(formulation_.terrain_->getGridMap(), message);

    grid_map_pub_.publish(message);
    ros::spinOnce();

    ///////////////////////////////
    ///// Publish point cloud /////
    ///////////////////////////////
    pcl::PointCloud<pcl::PointXYZI> points;
    pcl::PointCloud<pcl::PointXYZI> publishPoints;
    auto sdf = formulation_.terrain_->getSDF();
    sdf.convertToPointCloud(points);
    
    int pointSize = points.size();
    std::uniform_int_distribution<int> dis(0, pointSize-1);

    int publishPointSize = sqrt(pointSize);

    for(int i=0; i<publishPointSize; i++)
      publishPoints.push_back(points.at(dis(gen)));

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(publishPoints, output);
    output.header.frame_id = "world";

    point_cloud_pub_.publish(output);
    ros::spinOnce();
  }

  SetTowrInitialState();

  // solver parameters
  SetIpoptParameters(msg);

  // visualization
  // PublishInitialState();

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  if (msg.optimize || msg.play_initialization) {
    nlp_ = ifopt::Problem();
    for (auto c : formulation_.GetVariableSets(solution))
      nlp_.AddVariableSet(c);
    for (auto c : formulation_.GetConstraints(solution))
      nlp_.AddConstraintSet(c);
    for (auto c : formulation_.GetCosts(solution))
      nlp_.AddCostSet(c);

    solver_->Solve(nlp_);
    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
  }

  std::cout.precision(2);
  nlp_.PrintCurrent(); // view variable-set, constraint violations, indices,...

  // playback using terminal commands
  if (msg.replay_trajectory || msg.play_initialization || msg.optimize) {
    int success = system(("rosbag play --topics "
        + xpp_msgs::robot_state_desired + " "
        + xpp_msgs::terrain_info + " "
        + xpp_msgs::ee_trajectory + std::to_string(0) + " "
        + xpp_msgs::ee_trajectory + std::to_string(1) + " "
        + xpp_msgs::ee_trajectory + std::to_string(2) + " "
        + xpp_msgs::ee_trajectory + std::to_string(3) + " "
        + xpp_msgs::base_trajectory + " "
        + xpp_msgs::obstacle_info + " "
        + " -r " + std::to_string(msg.replay_speed)
        + " --quiet " + bag_file).c_str());
  }

  if (msg.plot_trajectory) {
    int success = system(("killall rqt_bag; rqt_bag " + bag_file + "&").c_str());
  }

  // to publish entire trajectory (e.g. to send to controller)
  // xpp_msgs::RobotStateCartesianTrajectory xpp_msg = xpp::Convert::ToRos(GetTrajectory());
}

void
TowrRosInterface::PublishInitialState()
{
  int n_ee = formulation_.initial_ee_W_.size();
  xpp::RobotStateCartesian xpp(n_ee);
  xpp.base_.lin.p_ = formulation_.initial_base_.lin.p();
  xpp.base_.ang.q  = EulerConverter::GetQuaternionBaseToWorld(formulation_.initial_base_.ang.p());

  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
    xpp.ee_contact_.at(ee_xpp)   = true;
    xpp.ee_motion_.at(ee_xpp).p_ = formulation_.initial_ee_W_.at(ee_towr);
    xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
  }

  initial_state_pub_.publish(xpp::Convert::ToRos(xpp));

}

std::vector<TowrRosInterface::XppVec>
TowrRosInterface::GetIntermediateSolutions ()
{
  std::vector<XppVec> trajectories;

  for (int iter=0; iter<nlp_.GetIterationCount(); ++iter) {
    nlp_.SetOptVariables(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRosInterface::XppVec
TowrRosInterface::GetTrajectory () const
{
  XppVec trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  EulerConverter base_angular(solution.base_angular_);

  while (t<=T+1e-5) {
    int n_ee = solution.ee_motion_.size();
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp)  = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
      state.ee_forces_ .at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;
  }

  return trajectory;
}

xpp_msgs::RobotParameters
TowrRosInterface::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  int n_ee = nominal_B.size();
  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    Vector3d pos = nominal_B.at(ee_towr);
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
    params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
TowrRosInterface::SaveOptimizationAsRosbag (const std::string& bag_name,
                                   const xpp_msgs::RobotParameters& robot_params,
                                   const TowrCommandMsg user_command_msg,
                                   bool include_iterations)
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command+"_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }

  // save the final trajectory
  auto final_trajectory = GetTrajectory();
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  bag.close();
}

void
TowrRosInterface::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                 const XppVec& traj,
                                 const std::string& topic) const
{

  std::vector<nav_msgs::Path> total_ee_path(4);
  nav_msgs::Path total_base_path;
  std::vector<nav_msgs::Path> ee_path(4);
  std::vector<ros::Time> trajectory_timestamp(4);
  std::vector<bool>is_contact(4);

  for(int i=0; i<4; i++){
    ee_path[i].header.stamp=ros::Time(1e-6);
    ee_path[i].header.frame_id="/world";
    total_ee_path[i].header.stamp=ros::Time(1e-6);
    total_ee_path[i].header.frame_id="/world";
    total_base_path.header.stamp=ros::Time(1e-6);
    total_base_path.header.frame_id="/world";
    trajectory_timestamp[i]=ros::Time(1e-6);
    is_contact[i]=false;
  }

  geometry_msgs::PoseStamped this_pose_stamped;

  for (const auto state : traj) {

    auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

    for(int i=0; i<4; i++){
      if(is_contact[i]){
        trajectory_timestamp[i]=timestamp;
        ee_path[i].poses.clear();
        is_contact[i]=false;
      }
    }

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;

    xpp_msgs::SphereObstacle obstacle_msg;
    geometry_msgs::Vector3 obstacle_pos;
    obstacle_msg.frame_id = "/world";

    Eigen::VectorXd base_linear_motion = state.base_.lin.p_;
    total_base_path.header.stamp=timestamp;

    this_pose_stamped.pose.position.x = base_linear_motion(0);
    this_pose_stamped.pose.position.y = base_linear_motion(1);
    this_pose_stamped.pose.position.z = base_linear_motion(2);
    this_pose_stamped.pose.orientation.x = 0;
    this_pose_stamped.pose.orientation.y = 0;
    this_pose_stamped.pose.orientation.z = 0;
    this_pose_stamped.pose.orientation.w = 1;
    this_pose_stamped.header.stamp=timestamp;
    this_pose_stamped.header.frame_id="/world";

    total_base_path.poses.push_back(this_pose_stamped);

    for (int ee_=0; ee_< state.ee_motion_.GetEECount(); ee_++){

      auto ee_motion = state.ee_motion_.at(ee_);

      is_contact.at(ee_) = state.ee_contact_.at(ee_);

      Vector3d n = formulation_.terrain_->getHeightMap()->GetNormalizedBasis(HeightMap::Normal, ee_motion.p_.x(), ee_motion.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = formulation_.terrain_->getHeightMap()->GetFrictionCoeff();

      this_pose_stamped.pose.position.x = ee_motion.p_.x();
      this_pose_stamped.pose.position.y = ee_motion.p_.y();
      this_pose_stamped.pose.position.z = ee_motion.p_.z();
      this_pose_stamped.pose.orientation.x = 0;
			this_pose_stamped.pose.orientation.y = 0;
			this_pose_stamped.pose.orientation.z = 0;
			this_pose_stamped.pose.orientation.w = 1;
      this_pose_stamped.header.stamp=timestamp;
			this_pose_stamped.header.frame_id="/world";

      ee_path.at(ee_).poses.push_back(this_pose_stamped);
      total_ee_path.at(ee_).poses.push_back(this_pose_stamped);
      total_ee_path.at(ee_).header.stamp=timestamp;

      // calculate collision sphere positions
      Eigen::Vector3d base_W;
      base_W << state.base_.lin.p_;

      Eigen::Quaterniond base_W_q(state.base_.ang.q);
      Eigen::Matrix3d base_W_R = base_W_q.toRotationMatrix();
      Eigen::Matrix3d base_R_W = base_W_R.transpose();
      
      Eigen::Vector3d ee_W(ee_motion.p_.x(), ee_motion.p_.y(), ee_motion.p_.z());
      
      Eigen::Vector3d ee_B;
      ee_B = base_R_W * (ee_W - base_W);

      Eigen::MatrixXd fk_mat; // [4,3] = [frame, position]

      collision_checker_->CalcJointPosMatrix(ee_, ee_B);
      fk_mat = collision_checker_->GetJointPosMatrix();

      Eigen::Vector3d link_start;
      Eigen::Vector3d link_end;
      
      for (int frame_ = 3; frame_ > 1; frame_--)
      {
        link_start = fk_mat.row(frame_-1);
        link_end = fk_mat.row(frame_);
        link_start = base_W + base_W_R * link_start;
        link_end = base_W + base_W_R * link_end;

        collision_checker_->CreateCollisionSpheres(frame_,
                                                  link_start, 
                                                  link_end); 
                                                  
        collision_checker_->CollisionChecking(frame_); // set collision cost, gradient and collisionState at collisionSphere

        for(int sphere_idx = 0; sphere_idx < collision_checker_->GetCollisionspheres(frame_).size(); sphere_idx++){

          auto sphere = collision_checker_->GetCollisionsphere(frame_, sphere_idx);

          obstacle_msg.radius.push_back(sphere->radius);
          obstacle_pos.x = sphere->center.x();
          obstacle_pos.y = sphere->center.y();
          obstacle_pos.z = sphere->center.z();

          obstacle_msg.center.push_back(obstacle_pos);
          obstacle_msg.collisionState.push_back(sphere->collisionState);
        }
      }
    }

    // obstacles msg write & initialization
    obstacle_msg.stamp=timestamp;
    bag.write(xpp_msgs::obstacle_info, timestamp, obstacle_msg);
    obstacle_msg.radius.clear();
    obstacle_msg.center.clear();
    obstacle_msg.collisionState.clear();

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);

    for(int i=0; i<4; i++){
      bag.write( xpp_msgs::ee_trajectory + std::to_string(i), trajectory_timestamp[i], ee_path[i]);
    }

    bag.write(xpp_msgs::base_trajectory, total_base_path.header.stamp, total_base_path);
  }

  // lastly write total ee_path
  for(int i=0; i<4; i++){
    bag.write( xpp_msgs::ee_trajectory + std::to_string(i), total_ee_path.at(i).header.stamp, total_ee_path[i]);
  }

}

} /* namespace towr */

