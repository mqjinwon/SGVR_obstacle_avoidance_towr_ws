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

#include <cmath>
#include <iostream>
#include <fstream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/initialization/gait_generator.h>
#include <towr/models/robot_model.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>

#include <towr/collision/collision_checker.h>
#include <towr/costs/collision_cost.h>

#include <random>
#include <time.h>

using namespace std;
using namespace towr;

bool CheckBinaryBool(const unsigned int A, const unsigned int B){
  return (A & B) != 0;
}

void writeFile(const string& filePath, const vector<double>& data){
    cout << "filePath: " << filePath << endl;
    ofstream writeFile(filePath.data(), ios::app);

    // test from here
    if (writeFile.is_open()){
      for(int i = 0; i < data.size(); i++){
        writeFile << data[i];
        if(i != data.size()-1)
          writeFile << ", ";
      }

      writeFile << endl;
      writeFile.close();
    }
    else{
      cout << "Unable to open file" << endl;
    }
}

void printSolution(const SplineHolder& solution){
  using namespace std;
  cout << "\n====================\nHyq trajectory:\n====================\n";

  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

    cout << "Foot position x,y,z:          \t";
    cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Contact force x,y,z:          \t";
    cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
    string foot_in_contact = contact? "yes" : "no";
    cout << "Foot in contact:              \t" + foot_in_contact << endl;

    cout << endl;

    t += 0.2;
  }
}

void testFunction(const Parameters& param, 
                  const shared_ptr<ifopt::IpoptSolver> solver, 
                  const double& total_duration, 
                  const shared_ptr<SGVR::TowrMap> terrain_,
                  const string& filePath,
                  const double& rand_x,
                  const double& rand_y,
                  const double& rand_yaw){
  
  NlpFormulation formulation;

  formulation.params_ = param;
  
  formulation.model_ = towr::RobotModel(towr::RobotModel::Hyq);

  formulation.terrain_ = terrain_;

  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();

  double z_ground = 0.0;
  formulation.initial_ee_W_ =  nominal_stance_B;
  for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
                [&](Eigen::Vector3d& p){ p.z() = z_ground; } // feet at 0 height
  );

  formulation.initial_base_.lin.at(kPos).z() = - nominal_stance_B.front().z() + z_ground;
  formulation.final_base_.lin.at(towr::kPos).x() = 3.3 + rand_x;
  formulation.final_base_.lin.at(towr::kPos).y() = 0.0 + rand_y;
  formulation.final_base_.lin.at(towr::kPos).z() = -formulation.model_.kinematic_model_->GetNominalStanceInBase().front().z();
  formulation.final_base_.ang.at(towr::kPos).x() = 0.0;
  formulation.final_base_.ang.at(towr::kPos).y() = 0.0;
  formulation.final_base_.ang.at(towr::kPos).z() = rand_yaw;
  
  int n_ee = formulation.model_.kinematic_model_->GetNumberOfEndeffectors();
  auto gait_gen_ = towr::GaitGenerator::MakeGaitGenerator(n_ee);
  auto id_gait   = towr::GaitGenerator::Combos::C0;

  gait_gen_->SetCombo(id_gait);
  for (int ee=0; ee < n_ee; ++ee) {
      formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(total_duration, ee));
      formulation.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
  }

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts(solution))
    nlp.AddCostSet(c);

  int optimize_result = solver->Solve(nlp);

  ////////////////////////////////
  //////// get trajectory ////////
  ////////////////////////////////

  // for calculate collision
  SGVR::CollisionChecker::Ptr collision_checker_ = make_shared<SGVR::CollisionChecker>(formulation.terrain_);

  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();
  int link_num_ = 4; // 원래는 urdf에서 가져와야함...

  NodeSpline::Ptr base_linear_                = solution.base_linear_;
  EulerConverter base_angular_                = solution.base_angular_;
  vector<NodeSpline::Ptr> ee_motions_    = solution.ee_motion_;

  double total_cost = 0.0;
  int swing_link_collision_count = 0; bool link_collision_flag = false; // link에서만 충돌이 얼마나 났는 지
  int stance_link_collision_count = 0;
  int total_collision_count = 0; bool total_collision_flag = true; // 총 충돌이 얼마나 났는 지

  while (t <= T + 1e-5){
    // 1. IK for base pose
    Eigen::Vector3d base_W = base_linear_->GetPoint(t).p();
    EulerConverter::MatrixSXd R_wb = base_angular_.GetRotationMatrixBaseToWorld(t);
    EulerConverter::MatrixSXd R_bw = R_wb.transpose();

    for (int ee=0; ee < n_ee; ++ee) {
      link_collision_flag = false;
      total_collision_flag = false;

      int poly_id = ee_motions_.at(ee)->GetMiddleNodeId(t);

      bool swing_flag = !(solution.phase_durations_.at(ee)->IsContactPhase(t));

      Eigen::Vector3d ee_B; // ee position in base frame
      ee_B = R_bw * (ee_motions_.at(ee)->GetPoint(t).p() - base_W);
      
      // 2. calculate kinematics - using orocos-kdl-based-kinematics
      Eigen::MatrixXd fk_mat; // [4,3] = [frame, position]

      collision_checker_->CalcJointPosMatrix(ee, ee_B);
      fk_mat = collision_checker_->GetJointPosMatrix();
      
      Eigen::Vector3d link_start; link_start.setZero();
      Eigen::Vector3d link_end; link_end.setZero();

      for (int frame = link_num_-1; frame > 2; frame--)
      {
        double cost = 0.0;
        // 3. collision checker - make collision sphere & calculate collision cost and gradient
        link_start = fk_mat.row(frame-1);
        link_end = fk_mat.row(frame);
        link_start = base_W + R_wb * link_start;
        link_end = base_W + R_wb * link_end;

        collision_checker_->CreateCollisionSpheres(frame,
                                                    link_start, 
                                                    link_end); 

        collision_checker_->CollisionChecking(frame, 0.0, formulation.params_.swing_cost_weight_); // set collision cost, gradient and collisionState at collisionSphere

        // 4. cost function - this cost function is calculated by the end-effector gradients
        for(int sphere_idx = 0; sphere_idx < collision_checker_->GetCollisionspheres(frame).size(); sphere_idx++){
          auto sphere = collision_checker_->GetCollisionsphere(frame, sphere_idx);

          if (sphere->collisionState){
            cost += collision_checker_->PenaltyFunction(sphere, 0.0, 0.0);

            if(!total_collision_flag){
              total_collision_count++;
              total_collision_flag = true;
            }
            if((!link_collision_flag) && sphere_idx < collision_checker_->GetCollisionspheres(frame).size()-1){
              if(swing_flag) swing_link_collision_count++;
              else stance_link_collision_count++;
              link_collision_flag = true;
            }
          }
        }
        total_cost += cost;
      } // end of link loop
    } // end of ee loop
    
    t += formulation.params_.dt_obstacle_constraint_;
  } // end of dts
  
  double optimize_time = solver->GetTotalWallclockTime();
  bool success = ((optimize_result == 0 || optimize_result == 1) && (total_collision_count == 0));

  cout << "terrain_id: " << terrain_names.at(formulation.terrain_->getTerrainID()).c_str() << endl;

  cout << "collision mode: ";
  int type_size = log2(static_cast<double>(SGVR::CollisionCost::COST_TYPE::COST_COUNT));
  for(int i = 0; i < type_size; ++i)
  {
    if((formulation.params_.collision_avoidance_mode_ & (1 << i)) != 0)
    {
      cout << SGVR::collision_cost_name.at(static_cast<SGVR::CollisionCost::COST_TYPE>(1 << i)).c_str() << " + ";
    }
  }
  if(formulation.params_.collision_avoidance_mode_ == 0)
  {
    cout << SGVR::collision_cost_name.at(static_cast<SGVR::CollisionCost::COST_TYPE>(0)).c_str();
  }
  cout << endl;

  cout << "total success: " << success << endl;
  cout << "optimize success: " << optimize_result << endl;
  cout << "optimize time: " << optimize_time << endl;
  cout << "total cost : " << total_cost << endl;
  cout << "total collision count : " << total_collision_count << endl;
  cout << "swing link collision count : " << swing_link_collision_count << endl;
  cout << "stance link collision count: " << stance_link_collision_count << endl;
  cout << "final x: " << formulation.final_base_.lin.at(towr::kPos).x() << endl;
  cout << "final y: " << formulation.final_base_.lin.at(towr::kPos).y()  << endl;
  cout << "final yaw: " << formulation.final_base_.ang.at(towr::kPos).z()  << endl;
  cout << "collision dt: " << formulation.params_.dt_obstacle_constraint_ << endl;
  cout << "edge_threshold: " << formulation.params_.edge_threshold_dist_ << endl;
  cout << "===============================================================" << endl;

  vector<double> writeData;
  writeData.push_back(formulation.terrain_->getTerrainID());
  writeData.push_back(formulation.params_.collision_avoidance_mode_);
  writeData.push_back(success);
  writeData.push_back(optimize_result);
  writeData.push_back(total_cost);
  writeData.push_back(total_collision_count);
  writeData.push_back(swing_link_collision_count);
  writeData.push_back(stance_link_collision_count);
  writeData.push_back(optimize_time);
  writeData.push_back(formulation.final_base_.lin.at(towr::kPos).x());
  writeData.push_back(formulation.final_base_.lin.at(towr::kPos).y());
  writeData.push_back(formulation.final_base_.ang.at(towr::kPos).z());
  // writeData.push_back(formulation.params_.dt_obstacle_constraint_);
  writeData.push_back(formulation.params_.edge_threshold_dist_);

  writeFile(filePath, writeData);

  // nlp.PrintCurrent();
  // printSolution(solution);
}

int main()
{
  double time_duration = 6.0;
  Parameters param;

  mt19937 urbg(time(NULL));
  uniform_real_distribution<float> real_dist {-1.0f, 1.0f};

  param.use_obstacle_constraints_ = true;
  // param.stance_cost_weight_ = 1000;

  auto solver = make_shared<ifopt::IpoptSolver>();
//   solver->SetOption("tol", 1e-7);
  // solver->SetOption("compl_inf_tol", 1e-12); // objective function tolerance
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 60.0);
  solver->SetOption("print_level", 0);
  
  int test_num = 1000;

  int map_type = -1;
  while(!(map_type >=0 && map_type <=3)){
    cout << "0: RandomStairs" << endl;
    cout << "1: RandomBlocks" << endl;
    cout << "2: RandomHuddle" << endl;
    cout << "3: PerlinNoise" << endl;
    cout << "choose map type: ";
    cin >> map_type;
  }

  //choose which terrain use
  HeightMap::TerrainID height_id;
  std::string height_name = "";
  if(map_type == 0){
      height_id = HeightMap::TerrainID::RandomStairsID;
      height_name = "random_stairs";
  }
  else if(map_type == 1){
      height_id = HeightMap::TerrainID::RandomBlocksID;
      height_name = "random_blocks";
  }
  else if(map_type == 2){
      height_id = HeightMap::TerrainID::RandomHuddleID;
      height_name = "random_huddle";
  }
  else if(map_type == 3){
      height_id = HeightMap::TerrainID::PerlinNoiseMapID;
      height_name = "perlin_noise";
  }
  
  vector<unsigned int> coll_mode;
  vector<std::string> coll_name;

  coll_mode.push_back(SGVR::CollisionCost::COST_TYPE::NONE);
  coll_name.push_back("none");
  coll_mode.push_back(SGVR::CollisionCost::COST_TYPE::GRAD_STANCE);
  coll_name.push_back("grad_stance");
  coll_mode.push_back(SGVR::CollisionCost::COST_TYPE::GRAD_STANCE | SGVR::CollisionCost::COST_TYPE::GRAD_SWING);
  coll_name.push_back("grad_stance_grad_swing");

  if(height_id != HeightMap::TerrainID::PerlinNoiseMapID)
  {  
    coll_mode.push_back(SGVR::CollisionCost::COST_TYPE::HEURISTIC_STANCE);
    coll_name.push_back("heuristic_stance");
    coll_mode.push_back(SGVR::CollisionCost::COST_TYPE::HEURISTIC_STANCE | SGVR::CollisionCost::COST_TYPE::GRAD_SWING);
    coll_name.push_back("heuristic_stance_grad_swing");
  }

  std::vector<double> edge_threshold;
  edge_threshold.push_back(0.085);
  edge_threshold.push_back(0.15);

  for(int i=0; i<test_num; i++){
    cout << "test_num: " << i << endl;

    ////////////////////////////
    /////// make terrain ///////
    ////////////////////////////
    string elevationLayerName_ = "elevation";
    vector<string> layerNames_ = {elevationLayerName_};
    shared_ptr<SGVR::TowrMap> terrain_ = make_shared<SGVR::TowrMap>(height_id, layerNames_);

    double rand_x = real_dist(urbg) * 0.1;
    double rand_y = real_dist(urbg) * 0.1;
    double rand_yaw = real_dist(urbg) * 0.25;

    double dxy   =  0.01; // 1cm resolution
    double x_min =  -1.0;
    double x_max =  4.0;
    double y_min =  -1.0;
    double y_max =  1.0;
    terrain_->gridMapInit("world", 
                          grid_map::Length(x_max - x_min, y_max - y_min), dxy, 
                          grid_map::Position((x_min+x_max)/2.0, (y_min+y_max)/2.0));
    terrain_->updateGridMap(elevationLayerName_);   //update grid map from height map information
    terrain_->updateSDF(elevationLayerName_, 0.1);  //update sdf from grid map information, margin z=0.1

    for(int j = 0; j < coll_mode.size(); j++){
      auto mode = coll_mode[j];
      auto mode_name = coll_name[j];

      for(int k = 0; k < edge_threshold.size(); k++){
        auto edge_thresh = edge_threshold[k];
        param.edge_threshold_dist_ = edge_thresh;
        param.collision_avoidance_mode_ = mode;
        string file_path = PACKAGE_PATH;
        file_path = file_path + "/test/experiment/height_id_" + height_name + "_" + "coll_mode_" + mode_name + ".csv";
        testFunction(param, solver, time_duration, terrain_, file_path, rand_x, rand_y, rand_yaw);

        if( !CheckBinaryBool(mode, SGVR::CollisionCost::COST_TYPE::HEURISTIC_STANCE)){
          break;
        }
      }
    }
  }
} // end of main
