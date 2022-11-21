#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>
#include <typeinfo>
#include <math.h>
#include <iostream>

namespace SGVR{

class Edge{
public:

    Edge(){};

    Edge(const Eigen::Vector2d& center, const Eigen::Vector2d& direction, const double& length):
        center_(center), direction_(direction), length_(length){}

    double Distance(const Eigen::Vector2d& point) const{
        double point_dist = (point - center_).norm();
        double edge_dist = abs((point-center_).dot(direction_));
        double height = sqrt(point_dist*point_dist - edge_dist*edge_dist);
        return height;
    }

    double GaussainCost(const Eigen::Vector2d point, const double& sigma) const{
        double edge_dist = abs((point-center_).dot(direction_));

        if(edge_dist > length_)
            return 0;
        else
            return exp(-Distance(point)*Distance(point)/(2*sigma*sigma));
    }

    Eigen::Vector2d GaussainGrad(const Eigen::Vector2d& point, const double& sigma) const{
        double alpha = -GaussainCost(point, sigma) * Distance(point)/ (sigma*sigma);

        //get gradient of Distance
        Eigen::Vector2d grad_dist;

        double point_dist = (point - center_).norm();
        double edge_dist = abs((point-center_).dot(direction_));
        double Distance = sqrt(point_dist*point_dist - edge_dist*edge_dist);

        grad_dist.x() = -((center_.x()-point.x())*(1+direction_.x()*direction_.x()) + direction_.x()*direction_.y() * (center_.y()-point.y()) )  / Distance;
        grad_dist.y() = -((center_.y()-point.y())*(1-direction_.y()*direction_.y()) - direction_.x()*direction_.y() * (center_.x()-point.x()) )  / Distance;
        
        return alpha * grad_dist;
    }

private:
    Eigen::Vector2d center_;
    Eigen::Vector2d direction_;
    double length_;
};

} //end namespace SGVR