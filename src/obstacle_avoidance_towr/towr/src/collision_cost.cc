#include <towr/costs/collision_cost.h>

namespace SGVR {

CollisionCost::CollisionCost (const SGVR::TowrMap::Ptr& terrain, 
                                Parameters param, const EE& ee,
                                const SplineHolder& spline_holder, 
                                double weight)
  : CostTerm("collision_cost-" + std::to_string(ee))
{

  // Setting 
  terrain_        = terrain;
  base_linear_    = spline_holder.base_linear_;
  base_angular_   = EulerConverter(spline_holder.base_angular_);
  ee_motions_     = spline_holder.ee_motion_;
  ee_             = ee;
  weight_         = weight;

  // Collision checker
  collision_checker_  = std::make_shared<SGVR::CollisionChecker>(terrain_);
  leg_num_            = collision_checker_->GetNumOfEE();
  link_num_           = collision_checker_->GetNumOfLinks();

  // hyperparameters
  pseudo_inverse_ridge_factor_    = param.pseudo_inverse_ridge_factor_;
  use_sum_gradient_               = param.use_sum_gradient_;

  swing_cost_weight_              = param.swing_cost_weight_;
  stance_cost_weight_             = param.stance_cost_weight_;
  collision_avoidance_mode_       = param.collision_avoidance_mode_;
  heuristic_cost_weight_          = param.heuristic_cost_weight_;

  swing_activation_dist_          = param.swing_activation_dist_;
  stance_activation_dist_         = param.stance_activation_dist_;

  edge_threshold_dist_            = param.edge_threshold_dist_;

  // at distance edge_threshold_dist_, gaussain function value is 1e-10;
  edge_avoidance_sigma_           = sqrt((edge_threshold_dist_*edge_threshold_dist_)/(2*log(1/1e-10)));

  double T = param.GetTotalTime();
  double dt = param.dt_obstacle_constraint_;
  double t = 0.0;

  for (int i=0; i<floor(T/dt); ++i) {
    t += dt;
    dts_.push_back(t);
  }

  dts_.push_back(T); // also ensure constraints at very last node/time.
}

void
CollisionCost::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<NodesVariablesPhaseBased>(id::EEMotionNodes(ee_));
  base_ang_ = x->GetComponent<NodesVariables>(id::base_ang_nodes);
  base_lin_ = x->GetComponent<NodesVariables>(id::base_lin_nodes);

  pure_swing_node_ids_ = ee_motion_->GetIndicesOfNonConstantNodes();
  pure_stance_node_ids_ = ee_motion_->GetIndicesOfConstantNodes();
}

double
CollisionCost::GetCost () const
{
    /*
    frame 0: hipassembly
    frame 1: upperleg
    frame 2: lowerleg
    frame 3: foot
    */

    double total_cost = 0.0; // cost of collision avoidance

    // heuristic stance cost
    if(CheckBinaryBool(collision_avoidance_mode_, HEURISTIC_STANCE)){

        double cost=0;

        for (int id : pure_stance_node_ids_) {

            auto nodes = ee_motion_->GetNodes();
            Eigen::Vector3d ee_W = nodes.at(id).p();

            for(auto edges: terrain_->getHeightMap()->GetEdge()){
                cost += heuristic_cost_weight_ * edges.GaussainCost(ee_W.head(2), edge_avoidance_sigma_);
            }

            total_cost += cost;
        }
    }

    // our idea
    if(CheckBinaryBool(collision_avoidance_mode_, (GRAD_STANCE | GRAD_SWING) ) ){
        while (!cost_dt_pq_.empty()) {
            cost_dt_pq_.pop();
        }

        for (double t : dts_){
            double cost = 0;

            int poly_id = ee_motions_.at(ee_)->GetMiddleNodeId(t);

            bool swing_flag = false;
            
            for(auto swing_node: pure_swing_node_ids_){
                if(swing_node == poly_id){
                swing_flag = true;
                break;
                }
            }

            if(swing_flag){
                if(!CheckBinaryBool(collision_avoidance_mode_, GRAD_SWING)){
                    continue;
                }
            }
            else{
                if(!CheckBinaryBool(collision_avoidance_mode_, GRAD_STANCE)){
                    continue;
                }
            }

            // 1. IK for base pose
            Eigen::Vector3d base_W = base_linear_->GetPoint(t).p();
            EulerConverter::MatrixSXd R_wb = base_angular_.GetRotationMatrixBaseToWorld(t);
            EulerConverter::MatrixSXd R_bw = R_wb.transpose();
            
            Eigen::Vector3d ee_B; // ee position in base frame
            ee_B = R_bw * (ee_motions_.at(ee_)->GetPoint(t).p() - base_W);
            
            // 2. calculate kinematics - using orocos-kdl-based-kinematics
            Eigen::MatrixXd fk_mat; // [4,3] = [frame, position]

            collision_checker_->CalcJointPosMatrix(ee_, ee_B);
            fk_mat = collision_checker_->GetJointPosMatrix();
            
            Eigen::Vector3d link_start; link_start.setZero();
            Eigen::Vector3d link_end; link_end.setZero();

            for (int frame = link_num_-1; frame > 2; frame--)
            {
                // 3. collision checker - make collision sphere & calculate collision cost and gradient
                link_start = fk_mat.row(frame-1);
                link_end = fk_mat.row(frame);
                link_start = base_W + R_wb * link_start;
                link_end = base_W + R_wb * link_end;

                bool collision_flag = false; // only check collision sphere one by one -- chomp implementation

                collision_checker_->CreateCollisionSpheres(frame,
                                                            link_start, 
                                                            link_end); 

                if(swing_flag){
                    collision_checker_->CollisionChecking(frame, swing_activation_dist_, swing_cost_weight_, false);
                }
                else{
                    collision_checker_->CollisionChecking(frame, stance_activation_dist_, stance_cost_weight_, true); // set collision cost, gradient and collisionState at collisionSphere
                }

                // 4. cost function - this cost function is calculated by the end-effector gradients
                for(int sphere_idx = 0; sphere_idx < collision_checker_->GetCollisionspheres(frame).size(); sphere_idx++){

                    auto sphere = collision_checker_->GetCollisionsphere(frame, sphere_idx);
                    if (sphere->collisionState){
                        if(swing_flag){
                            cost += collision_checker_->PenaltyFunction(sphere, swing_activation_dist_, swing_cost_weight_);
                        }
                        else{
                            cost += collision_checker_->PenaltyFunction(sphere, stance_activation_dist_, stance_cost_weight_);
                        }

                        if(!use_sum_gradient_) collision_flag = true;
                    }
                    if (collision_flag) break;
                }
            }

            total_cost += cost;

            // pop and change cost
            if(cost > 1e-20){
                cost_dt_pq_.push({t, cost});
            }
            
        } // dt bracket
    } // coll_mode: chain_rule

    return weight_ * total_cost;
}

void
CollisionCost::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
    int n = jac.cols();
    Jacobian jac_model(1,n);

    // we focus on only motion nodes
    if (var_set == id::EEMotionNodes(ee_)) {

        // heuristic stance cost
        if(CheckBinaryBool(collision_avoidance_mode_, HEURISTIC_STANCE)){
            for (int id : pure_stance_node_ids_) {

                auto nodes = ee_motion_->GetNodes();

                Eigen::Vector3d ee_W = nodes.at(id).p();
                Eigen::Vector2d edge_gradient(0,0);

                for(auto edges: terrain_->getHeightMap()->GetEdge()){
                
                    edge_gradient += heuristic_cost_weight_ * edges.GaussainGrad(ee_W.head(2), edge_avoidance_sigma_);
                }

                jac_model.coeffRef(0, ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, X))) += weight_ * edge_gradient.x();
                jac_model.coeffRef(0, ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, Y))) += weight_ * edge_gradient.y();
            }
        }

        if(CheckBinaryBool(collision_avoidance_mode_, (GRAD_STANCE | GRAD_SWING) ) ){
            grad_dt_pq_ = cost_dt_pq_; // copy the queue

            while (!grad_dt_pq_.empty()) {
                double t = grad_dt_pq_.top().first; // high cost dt first

                // pass no collision at t
                if(grad_dt_pq_.top().second < 1e-12){
                    break;
                }
                int poly_id = ee_motions_.at(ee_)->GetMiddleNodeId(t);

                // for "chain rule" method
                double t_local = ee_motions_.at(ee_)->GetLocalTimes(t);

                bool swing_flag = false;
                
                for(auto swing_node: pure_swing_node_ids_){
                    if(swing_node == poly_id){
                        swing_flag = true;
                        break;
                    }
                }

                // check collision mode and poly id is matched
                if(swing_flag){
                    if(!CheckBinaryBool(collision_avoidance_mode_, GRAD_SWING)){
                        continue;
                    }
                }
                else{
                    if(!CheckBinaryBool(collision_avoidance_mode_, GRAD_STANCE)){
                        continue;
                    }
                }

                // 1. IK for base pose
                Eigen::Vector3d base_W = base_linear_->GetPoint(t).p();
                EulerConverter::MatrixSXd R_wb = base_angular_.GetRotationMatrixBaseToWorld(t);
                EulerConverter::MatrixSXd R_bw = R_wb.transpose();
                
                Eigen::Vector3d ee_B; // ee position in base frame
                ee_B = R_bw * (ee_motions_.at(ee_)->GetPoint(t).p() - base_W);
                
                // 2. calculate kinematics - using orocos-kdl-based-kinematics
                Eigen::MatrixXd fk_mat; // [4,3] = [frame, position]

                collision_checker_->CalcJointPosMatrix(ee_, ee_B);
                fk_mat = collision_checker_->GetJointPosMatrix();
                
                Eigen::Vector3d link_start; link_start.setZero();
                Eigen::Vector3d link_end; link_end.setZero();

                Eigen::Vector3d gradient_pos_result; gradient_pos_result.setZero();
                Eigen::Vector3d gradient_vel_result; gradient_vel_result.setZero();

                for (int frame = link_num_-1; frame > 2; frame--)
                {
                    // 3. collision checker - make collision sphere & calculate collision cost and gradient
                    link_start = fk_mat.row(frame-1);
                    link_end = fk_mat.row(frame);
                    link_start = base_W + R_wb * link_start;
                    link_end = base_W + R_wb * link_end;
                    
                    bool collision_flag = false; // only check collision sphere one by one -- chomp implementation
                    
                    collision_checker_->CreateCollisionSpheres(frame,
                                                                link_start, 
                                                                link_end);

                    if(swing_flag){
                        collision_checker_->CollisionChecking(frame, swing_activation_dist_, swing_cost_weight_, false);
                    }
                    else{
                        collision_checker_->CollisionChecking(frame, stance_activation_dist_, stance_cost_weight_, true); // set collision cost, gradient and collisionState at collisionSphere
                    }

                    Eigen::Matrix3d poly_pos_jac_mat; poly_pos_jac_mat.setZero();
                    Eigen::Matrix3d poly_vel_jac_mat; poly_vel_jac_mat.setZero();

                    poly_pos_jac_mat = ee_motions_.at(ee_)->GetSplineDerivative(poly_id, t_local, jac, kPos, kPos);

                    if(swing_flag){
                        poly_vel_jac_mat= ee_motions_.at(ee_)->GetSplineDerivative(poly_id, t_local, jac, kPos, kVel);
                    }
                    
                    // 4. cost function - this cost function is calculated by the end-effector difference
                    for(int sphere_idx = 0; sphere_idx < collision_checker_->GetCollisionspheres(frame).size(); sphere_idx++){
                            auto sphere = collision_checker_->GetCollisionsphere(frame, sphere_idx);

                        if (sphere->collisionState){

                            // sphere gradient 계산
                            Eigen::Vector3d sphere_gradient;
                            if(swing_flag){
                                sphere_gradient = collision_checker_->DerivativePenaltyFunction(sphere, swing_activation_dist_, swing_cost_weight_);
                            }
                            else{
                                sphere_gradient = collision_checker_->DerivativePenaltyFunction(sphere, stance_activation_dist_, stance_cost_weight_);
                            }

                            KDL::Jacobian sphere_jac = collision_checker_->GetForwardKinematics()->MakeJacobian(ee_, frame, 
                                                                                                                (collision_checker_->GetJointAngles()).head(frame),
                                                                                                                sphere->distFromParent);
                            // end-effector jacobian, pseudoJacobian
                            KDL::Jacobian ee_jac = collision_checker_->GetForwardKinematics()->MakeJacobian(ee_, link_num_-1, 
                                                                                                            collision_checker_->GetJointAngles().head(link_num_-1));
                            Eigen::MatrixXd sphere_jac_vel_mat = sphere_jac.data.topRows(3);

                            Eigen::MatrixXd ee_jac_vel_mat = ee_jac.data.topRows(3);
                            Eigen::MatrixXd ee_pseudo_jac_vel_mat = ee_jac_vel_mat.transpose() 
                                                                    * (ee_jac_vel_mat * ee_jac_vel_mat.transpose() + Eigen::MatrixXd::Identity(3, 3) * pseudo_inverse_ridge_factor_).inverse();
                            gradient_pos_result += sphere_gradient.transpose() * R_wb * sphere_jac_vel_mat * ee_pseudo_jac_vel_mat * R_bw * poly_pos_jac_mat;
                            
                            if(swing_flag){
                                gradient_vel_result += sphere_gradient.transpose() * R_wb * sphere_jac_vel_mat * ee_pseudo_jac_vel_mat * R_bw * poly_vel_jac_mat;
                            }
                            
                            if(!use_sum_gradient_) collision_flag = true;
                        }
                        if (collision_flag) break;
                    }
                }

                jac_model.coeffRef(0, ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(poly_id, kPos, X))) += weight_ * gradient_pos_result.x();
                jac_model.coeffRef(0, ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(poly_id, kPos, Y))) += weight_ * gradient_pos_result.y();
                jac_model.coeffRef(0, ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(poly_id, kPos, Z))) += weight_ * gradient_pos_result.z();

                if(swing_flag){
                    jac_model.coeffRef(0, ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(poly_id, kVel, X))) += weight_ * gradient_vel_result.x();
                    jac_model.coeffRef(0, ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(poly_id, kVel, Y))) += weight_ * gradient_vel_result.y();
                }

                grad_dt_pq_.pop(); // flush the queue

            }// dt bracket
        }
    } // end of if (var_set == id::EEMotionNodes(ee_)) 

    jac = jac_model;

}// FillJacobianBlock function bracket

} // namespace SGVR