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

#include <towr/variables/node_spline.h>


#include <iostream>

namespace towr {

NodeSpline::NodeSpline(NodeSubjectPtr const node_variables,
                       const VecTimes& polynomial_durations)
    :   Spline(polynomial_durations, node_variables->GetDim()),
        NodesObserver(node_variables)
{
  UpdateNodes();
  jac_wrt_nodes_structure_ = Jacobian(node_variables->GetDim(), node_variables->GetRows());
}

void
NodeSpline::UpdateNodes ()
{
  for (int i=0; i<cubic_polys_.size(); ++i) {
    auto nodes = node_values_->GetBoundaryNodes(i);
    cubic_polys_.at(i).SetNodes(nodes.front(), nodes.back());
  }

  UpdatePolynomialCoeff();
}

int 
NodeSpline::GetNodeId(double t_global){
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, GetPolyDurations());

  return id;
}

int 
NodeSpline::GetMiddleNodeId(double t_global){
  int id;
  id = GetMiddleSegmentID(t_global, GetPolyDurations());

  return id;
}

double 
NodeSpline::GetLocalTimes(double t_global){
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, GetPolyDurations());
  return t_local;
}

int
NodeSpline::GetNodeVariablesCount() const
{
  return node_values_->GetRows();
}

NodeSpline::Jacobian
NodeSpline::GetJacobianWrtNodes (double t_global, Dx dxdt) const
{
  int id; double t_local;
  std::tie(id, t_local) = GetLocalTime(t_global, GetPolyDurations());

  return GetJacobianWrtNodes(id, t_local, dxdt);
}

NodeSpline::Jacobian
NodeSpline::GetJacobianWrtNodes (int id, double t_local, Dx dxdt) const
{
  Jacobian jac = jac_wrt_nodes_structure_;
  FillJacobianWrtNodes(id, t_local, dxdt, jac, false);

  // needed to avoid Eigen::assert failure "wrong storage order" triggered
  // in dynamic_constraint.cc
  jac.makeCompressed();

  return jac;
}

void
NodeSpline::FillJacobianWrtNodes (int poly_id, double t_local, Dx dxdt,
                                  Jacobian& jac, bool fill_with_zeros) const
{
  for (int idx=0; idx<jac.cols(); ++idx) {
    for (auto nvi : node_values_->GetNodeValuesInfo(idx)) {
      for (auto side : {NodesVariables::Side::Start, NodesVariables::Side::End}) { // every jacobian is affected by two nodes
        int node = node_values_->GetNodeId(poly_id, side);

        if (node == nvi.id_) {
          double val = 0.0;

          if (side == NodesVariables::Side::Start)
            val = cubic_polys_.at(poly_id).GetDerivativeWrtStartNode(dxdt, nvi.deriv_, t_local);
          else if (side == NodesVariables::Side::End)
            val = cubic_polys_.at(poly_id).GetDerivativeWrtEndNode(dxdt, nvi.deriv_, t_local);
          else
            assert(false); // this shouldn't happen

          // if only want structure
          if (fill_with_zeros)
            val = 0.0;

          jac.coeffRef(nvi.dim_, idx) += val;
        }
      }
    }
  }
}

Eigen::Matrix3d
NodeSpline::GetSplineDerivative (int poly_id, double t_local, Jacobian& jac, Dx dxdt, Dx node_derivative) const
{
  Eigen::Matrix3d poly_jac_mat; poly_jac_mat.setZero();

  for (int idx=0; idx<jac.cols(); ++idx) { // 변수들 쫙 돌아보면서
    for (auto nvi : node_values_->GetNodeValuesInfo(idx)) { // 그때의 정보를 받는다.
      for (auto side : {NodesVariables::Side::Start, NodesVariables::Side::End}) { // every jacobian is affected by two nodes
        if (poly_id == nvi.id_) {
          if(node_derivative == nvi.deriv_){
            double val = 0.0;

            if (side == NodesVariables::Side::Start)
              // dxdt: t, t'..., nvi.deriv_: pos or vel node
              val = cubic_polys_.at(poly_id).GetDerivativeWrtStartNode(dxdt, nvi.deriv_, t_local);
            else if (side == NodesVariables::Side::End)
              val = cubic_polys_.at(poly_id).GetDerivativeWrtEndNode(dxdt, nvi.deriv_, t_local);
            else
              assert(false); // this shouldn't happen

            poly_jac_mat(nvi.dim_,nvi.dim_) += val;
          }
          
        }
      }
    }
  }
  return poly_jac_mat;
}

} /* namespace towr */
