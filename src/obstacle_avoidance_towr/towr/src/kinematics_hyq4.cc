#include <towr/kinematics/kinematics_hyq4.h>

namespace SGVR{

    HYQ4Kinematics::HYQ4Kinematics(const std::string& robot_description){
        // Initialize the robot model
        ee_name_.reserve(4);
        ee_name_.push_back("lf_");
        ee_name_.push_back("rf_");
        ee_name_.push_back("lh_");
        ee_name_.push_back("rh_");

        frame_name_.reserve(4);
        frame_name_.push_back("hipassembly");
        frame_name_.push_back("upperleg");
        frame_name_.push_back("lowerleg");
        frame_name_.push_back("foot");

        if (!kdl_parser::treeFromFile(robot_description, my_tree_)){
            std::cout << "Failed to construct kdl tree" << std::endl;
            exit(1);
        }
    }

    Eigen::Vector3d HYQ4Kinematics::getFramePosition(const int ee, int frame_id, const Eigen::VectorXd& joint_q, const double length){
        /**
        ==================
        from base to foot
        ================== 
        getNrOfJoints: 3
        getNrOfSegments: 5
        segmentName: trunk
        jointType: None
        segmentName: lf_hipassembly
        jointType: RotAxis
        segmentName: lf_upperleg
        jointType: RotAxis
        segmentName: lf_lowerleg
        jointType: RotAxis
        segmentName: lf_foot
        jointType: None
        **/

        my_tree_.getChain("trunk", ee_name_[ee] + frame_name_[frame_id], my_chain_); // get chain from tree

        // need to change my_chain length
        if(!(abs(length) < 1e-8)){
            KDL::Vector frame_length = my_chain_.getSegment(frame_id).getFrameToTip().p;
            frame_length.Normalize();

            ChangeLinkLength(my_chain_, frame_id, frame_length * length); // change link length
        }

        KDL::JntArray jointAngles = KDL::JntArray(my_chain_.getNrOfJoints()); // define joint angles array

        // set joint angles
        for (int i=0; i<joint_q.size(); i++){
            jointAngles(i) = joint_q(i);
        }

        KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(my_chain_); // define forward kinematics solver
        
        // solve forward kinematics
        assert(FKSolver.JntToCart(jointAngles, eeFrame_) >= 0);

        // return position
        Eigen::Vector3d eePos;
        eePos << eeFrame_.p.x(), eeFrame_.p.y(), eeFrame_.p.z();

        return eePos;
    }

    Eigen::MatrixXd HYQ4Kinematics::getLegFramePositionMatrix(const int ee, const Eigen::VectorXd joint_q){

        Eigen::MatrixXd legFrameMatrix(frame_name_.size(), 3); // [4,3] = [frame, position]

        // each frame forward kinematics - hipassembly, upperleg, lowerleg, foot
        for(int frame_=0; frame_<frame_name_.size(); frame_++)
        {
            legFrameMatrix.row(frame_)= this->getFramePosition(ee, frame_, joint_q);
        }

        return legFrameMatrix;
    }

    std::vector<Eigen::MatrixXd> HYQ4Kinematics::getFramePositionMatrix(const Eigen::MatrixXd joint_q_mat){
        std::vector<Eigen::MatrixXd> fk_mat; // [4,4,3] = [leg, frame, position]

        // each legs forward kinematics
        for (int ee_ = 0; ee_ < ee_name_.size(); ee_++)
        {
            Eigen::MatrixXd tmpMatrix(frame_name_.size(), 3); // [4,3] = [frame, position]

            // each frame forward kinematics - hipassembly, upperleg, lowerleg, foot
            for(int frame_=0; frame_<frame_name_.size(); frame_++)
            {
            tmpMatrix.row(frame_)= this->getFramePosition(ee_, frame_, joint_q_mat.row(ee_));
            }
            fk_mat.push_back(tmpMatrix);
        }

        return fk_mat;
    }

    KDL::Jacobian HYQ4Kinematics::MakeJacobian(const int& leg_idx, const int& link_idx, const Eigen::VectorXd& joint_q, const double length){
        my_tree_.getChain("trunk", ee_name_[leg_idx] + frame_name_[link_idx], my_chain_); // get chain from tree

        // need to change my_chain length
        if(!(abs(length) < 1e-8)){
            KDL::Vector frame_length = my_chain_.getSegment(link_idx).getFrameToTip().p;
            frame_length.Normalize();

            ChangeLinkLength(my_chain_, link_idx, frame_length * length); // change link length
        }

        // jacobian solver
        KDL::ChainJntToJacSolver jacSolver(my_chain_);

        KDL::Jacobian jac_(my_chain_.getNrOfJoints());
        KDL::JntArray jointAngles = KDL::JntArray(my_chain_.getNrOfJoints()); // define joint angles array

        // set joint angles
        for (int i=0; i<joint_q.size(); i++){
            jointAngles(i) = joint_q(i);
        }

        assert(jacSolver.JntToJac(jointAngles, jac_) >= 0);

        return jac_;
    }

    void HYQ4Kinematics::ChangeLinkLength(KDL::Chain& chain_, const int& segIdx,const KDL::Vector& changePos){
        // get old information
        KDL::Segment oldSeg = chain_.getSegment(segIdx);
        KDL::Frame oldTip = oldSeg.getFrameToTip();
        auto oldJoint = oldSeg.getJoint();
        oldTip.p = changePos;

        KDL::Joint newJoint;

        // change segment
        if (oldJoint.getType() != KDL::Joint::JointType::None){
            newJoint = KDL::Joint(oldJoint.getName(), oldTip.p, oldJoint.JointAxis(), oldJoint.getType()); // make new joint
        }
        else{
            newJoint = KDL::Joint(oldJoint.getName(), KDL::Joint::None);
        }

        KDL::Segment newSeg(oldSeg.getName(), newJoint, oldTip, oldSeg.getInertia()); // make new segment
        chain_.segments.at(segIdx) = newSeg;
    }

    double HYQ4Kinematics::CheckReachAbility(const Eigen::Vector3d& target_point, const int& leg_idx, const int& link_idx, const double length, const double reach_ratio){
        Eigen::Vector3d base2hip_;
        if (leg_idx == 0){
            base2hip_ = Eigen::Vector3d(0.3735, 0.207, 0.0);
        }
        else if (leg_idx == 1){
            base2hip_ = Eigen::Vector3d(0.3735, -0.207, 0.0);
        }
        else if (leg_idx == 2){
            base2hip_ = Eigen::Vector3d(-0.3735, 0.207, 0.0);
        }
        else if (leg_idx == 3){
            base2hip_ = Eigen::Vector3d(-0.3735, -0.207, 0.0);
        }
        
        double target_length = (target_point - base2hip_).norm();

        my_tree_.getChain("trunk", ee_name_[leg_idx] + frame_name_[link_idx], my_chain_); // get chain from tree

        double ref_length = 0;
        for(int i=0; i<link_idx; i++){
            KDL::Vector frame_length = my_chain_.getSegment(i).getFrameToTip().p;
            ref_length += frame_length.Norm();
        }
        ref_length += length;

        ref_length *= reach_ratio;

        if (target_length > ref_length){
            return ref_length/target_length;
        }
        else{
            return 1; // ok;
        }
    }
}