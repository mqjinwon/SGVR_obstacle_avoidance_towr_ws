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

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_

#include <towr/terrain/height_map.h>
#include <towr/terrain/PerlinNoise.h>
#include <random>

namespace towr {

/**
 * @addtogroup Terrains
 * @{
 */

/**
 * @brief Sample terrain of even height.
 */
class FlatGround : public HeightMap {
public:
  FlatGround(double height = 0.0);
  double GetHeight(double x, double y)  const override { return height_; };

private:
  double height_; // [m]
};

/**
 * @brief Sample terrain with a step in height in x-direction.
 */
class Block : public HeightMap {
public:
  double GetHeight(double x, double y)  const override;
  double GetHeightDerivWrtX(double x, double y) const override;

private:
  double block_start = 0.7;
  double length_     = 3.5;
  double height_     = 0.5; // [m]

  double eps_ = 0.03; // approximate as slope
  const double slope_ = height_/eps_;
};

/**
 * @brief Sample terrain with a two-steps in height in x-direction.
 */
class Stairs : public HeightMap {
public:
  Stairs(int num_stairs=2);
  double GetHeight(double x, double y) const override;

private:
  std::vector<double> x_center_;
  std::vector<double> y_center_;
  std::vector<double> x_min_;
  std::vector<double> x_max_;
  std::vector<double> y_min_;
  std::vector<double> y_max_;
  std::vector<double> half_length_;
  std::vector<double> theta_;
  std::vector<double> height_;
  int num_stairs_;
};

/**
 * @brief Sample terrain with parabola-modeled gap in x-direction.
 */
class Gap : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;
  double GetHeightDerivWrtXX(double x, double y) const override;

private:
  const double gap_start_ = 1.0;
  const double w = 0.5;
  const double h = 1.5;

  const double slope_ = h/w;
  const double dx = w/2.0;
  const double xc = gap_start_ + dx; // gap center
  const double gap_end_x = gap_start_ + w;

  // generated with matlab
  // see matlab/gap_height_map.m
  // coefficients of 2nd order polynomial
  // h = a*x^2 + b*x + c
  const double a = (4*h)/(w*w);
  const double b = -(8*h*xc)/(w*w);
  const double c = -(h*(w - 2*xc)*(w + 2*xc))/(w*w);
};

/**
 * @brief Sample terrain with an increasing and then decreasing slope in x-direction.
 */
class Slope : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;

private:
  const double slope_start_ = 1.0;
  const double up_length_   = 1.0;
  const double down_length_ = 1.0;
  const double height_center = 0.7;

  const double x_down_start_ = slope_start_+up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope_ = height_center/up_length_;
};

class OneStair : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

private:
  double x_start_ = 0.4;
  double length_  = 0.1;
  double y_start_ = 0.5; // distance to start of slope from center at z=0
  double height_ = 0.15;
};

class OneGap : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

private:
  double x_start_ = 0.4;
  double length_  = 0.05;
  double y_start_ = 0.5; // distance to start of slope from center at z=0
  double height_ = 0.15;
};

class SmallBlocks : public HeightMap {
public:
  SmallBlocks(int num_blocks = 5);
  SmallBlocks(std::vector<double> x_start, std::vector<double> length, std::vector<double> y_start);
  double GetHeight(double x, double y) const override;

private:
  std::vector<double> x_center_;
  std::vector<double> y_center_;
  std::vector<double> length_;
  int num_blocks_;
  double height_ = 0.2;
};

class LargeBlocks : public HeightMap {
public:
  LargeBlocks(int num_blocks = 5);
  LargeBlocks(std::vector<double> x_start, std::vector<double> length, std::vector<double> y_start);
  double GetHeight(double x, double y) const override;

private:
  std::vector<double> x_center_;
  std::vector<double> y_center_;
  std::vector<double> length_;
  int num_blocks_;
  double height_ = 0.2;
};

class RandomStairs : public HeightMap {
public:
  RandomStairs();
  double GetHeight(double x, double y) const override;

private:
  std::vector<double> x_center_;
  std::vector<double> y_center_;
  std::vector<double> x_min_;
  std::vector<double> x_max_;
  std::vector<double> y_min_;
  std::vector<double> y_max_;
  std::vector<double> half_length_;
  std::vector<double> theta_;
  std::vector<double> height_;
  int num_stairs_;

  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
};

class RandomBlocks : public HeightMap {
public:
  RandomBlocks(int num_blocks = 8);
  double GetHeight(double x, double y) const override;

private:
  std::vector<double> x_center_;
  std::vector<double> y_center_;
  std::vector<double> length_;
  std::vector<double> height_;
  int num_blocks_;

  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
};

class RandomHuddle : public HeightMap {
public:
  RandomHuddle(int num_huddle = 3);
  double GetHeight(double x, double y) const override;

private:
  std::vector<double> x_center_;
  std::vector<double> y_center_;
  std::vector<double> x_length_;
  std::vector<double> y_length_;
  std::vector<double> z_height_;
  int num_huddle_;

  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
};

class PerlinNoiseMap : public HeightMap {
public:
  PerlinNoiseMap();
  
  double GetHeight(double x, double y) const override;

private:
  double x_length_;
  double y_length_;
  double weight_;
  double z_;

  int random_seed_;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_ */
