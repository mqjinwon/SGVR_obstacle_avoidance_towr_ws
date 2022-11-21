#pragma once

#include <memory>
#include <string>
#include <cmath>

#include <ifopt/cost_term.h>

#include <towr/variables/variable_names.h>
#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>
#include <towr/variables/euler_converter.h>

// Collision library
#include <towr/collision/collision_checker.h>

// Mapping library
#include <towr/terrain/grid_map.h>

// Parameter library
#include <towr/parameters.h>

// for selective dt
#include <queue>
#include <functional>

namespace SGVR {
  using namespace towr;
  using namespace std; // for std::queue

/**
 * @brief  Assigns a cost to node values.
 *
 * @ingroup Costs
 */
class CollisionCost : public ifopt::CostTerm {
public:
  enum COST_TYPE {
    NONE = 0b0000,
    GRAD_STANCE = 0b0001,
    GRAD_SWING = 0b0010,
    HEURISTIC_STANCE = 0b0100,
    COST_COUNT = 0b1000
  };

  using VecTimes = std::vector<double>;
  using EE = uint;

  CollisionCost (const SGVR::TowrMap::Ptr& terrain, 
                      Parameters param, const EE& ee,
                      const SplineHolder& spline_holder, 
                      double weight);

  virtual ~CollisionCost () = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;

  double GetCost () const override;

  bool CheckBinaryBool(const unsigned int A, const unsigned int B) const{
    return (A & B) != 0;
  }

private:
  void FillJacobianBlock(std::string var_set, Jacobian&) const override;

  VecTimes dts_;

  NodeSpline::Ptr base_linear_;     ///< the linear position of the base.
  EulerConverter base_angular_; ///< the orientation of the base.
  std::vector<NodeSpline::Ptr> ee_motions_;       ///< the linear position of the endeffectors.

  EE ee_;

  NodesVariablesPhaseBased::Ptr ee_motion_;
  NodesVariables::Ptr base_ang_;
  NodesVariables::Ptr base_lin_;
  std::vector<int> pure_stance_node_ids_;
  std::vector<int> pure_swing_node_ids_;

  double weight_;

  // parameters
  uint8_t leg_num_;
  uint8_t link_num_;
  
  SGVR::TowrMap::Ptr terrain_; ///< gradient information at every position (x,y).

  // collision checking
  SGVR::CollisionChecker::Ptr collision_checker_;

  double pseudo_inverse_ridge_factor_;
  bool use_sum_gradient_;
  unsigned int collision_avoidance_mode_;
  double swing_cost_weight_;
  double stance_cost_weight_;
  double swing_activation_dist_;
  double stance_activation_dist_;
  double heuristic_cost_weight_;
  double edge_threshold_dist_;
  double edge_avoidance_sigma_;

  using TimeCost = pair<double, double>;
  // pair: <dt time, cost> cost를 기준으로 내림차순 정렬
  class cost_less{
  public:
    bool operator()(TimeCost& lhs, TimeCost& rhs) const{
      return lhs.second < rhs.second;
    }
  };

  mutable priority_queue<TimeCost, vector<TimeCost>, cost_less> cost_dt_pq_;
  mutable priority_queue<TimeCost, vector<TimeCost>, cost_less> grad_dt_pq_;
};

const static std::map<CollisionCost::COST_TYPE, std::string> collision_cost_name =
{
  {CollisionCost::NONE,                "None"},
  {CollisionCost::GRAD_STANCE,         "Grad_stance" },
  {CollisionCost::GRAD_SWING,          "Grad_swing" },
  {CollisionCost::HEURISTIC_STANCE,    "Heuristic_stance"    }
};

} /* namespace sgvr */
