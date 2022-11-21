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

#include <towr/terrain/examples/height_map_examples.h>

namespace towr {


FlatGround::FlatGround(double height)
{
  height_ = height;
}

double
Block::GetHeight (double x, double y) const
{
  double h = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    h = slope_*(x-block_start);

  if (block_start+eps_ <= x && x <= block_start+length_)
    h = height_;

  return h;
}

double
Block::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    dhdx = slope_;

  return dhdx;
}

Stairs::Stairs(int num_stairs):
num_stairs_(num_stairs)
{
  x_center_.reserve(num_stairs_);
  y_center_.reserve(num_stairs_);
  x_min_.reserve(num_stairs_);
  x_max_.reserve(num_stairs_);
  y_min_.reserve(num_stairs_);
  y_max_.reserve(num_stairs_);
  half_length_.reserve(num_stairs_);
  theta_.reserve(num_stairs_);
  height_.reserve(num_stairs_);

  for(int i=0; i<num_stairs_; i++){
    x_center_[i] = 0.7*i + 0.8;
    y_center_[i] = 0.0;
    half_length_[i] = 1;
    theta_[i] = M_PI_2;
    height_[i] = 0.15;

    x_min_[i] = x_center_[i] - 10e-5;
    x_max_[i] = x_center_[i] + 10e-5;
    y_min_[i] = y_center_[i] - half_length_[i];
    y_max_[i] = y_center_[i] + half_length_[i];

    edge_list_.emplace_back(Eigen::Vector2d(x_center_[i], y_center_[i]),
                            Eigen::Vector2d(0,1),
                            half_length_[i]);

  }
}


// STAIRS
double
Stairs::GetHeight (double x, double y) const
{
  double h = 0.0;

  for (int i=0; i<num_stairs_; i++) {
    // y_axis parallel
    double line_x_value = 0.0;
    if( fmod(theta_[i], M_PI_2) < 1e-5){
      line_x_value = x_center_[i];
    }
    else{
      line_x_value = ( y - (y_center_[i] - tan(theta_[i]) * x_center_[i])) / (tan(theta_[i]) + 1e-10);
    }

    if(x > line_x_value){
      h += height_[i];
    }
  }

  return h;
}


// GAP
double
Gap::GetHeight (double x, double y) const
{
  double h = 0.0;

  // modelled as parabola
  if (gap_start_ <= x && x <= gap_end_x)
    h = a*x*x + b*x + c;

  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dhdx = 2*a*x + b;

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dzdxx = 2*a;

  return dzdxx;
}


// SLOPE
double
Slope::GetHeight (double x, double y) const
{
  double z = 0.0;
  if (x >= slope_start_)
    z = slope_*(x-slope_start_);

  // going back down
  if (x >= x_down_start_) {
    z = height_center - slope_*(x-x_down_start_);
  }

  // back on flat ground
  if (x >= x_flat_start_)
    z = 0.0;

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  if (x >= x_down_start_)
    dzdx = -slope_;

  if (x >= x_flat_start_)
    dzdx = 0.0;

  return dzdx;
}

// OneStair - for test
double
OneStair::GetHeight (double x, double y) const
{

  if (x_start_ <= x){
    return height_;
  }
  else
    return 0.0;

}

double
OneGap::GetHeight (double x, double y) const
{

  if (x_start_ <= x && x <= x_start_+ length_){
    return height_;
  }
  else
    return 0.0;
}

SmallBlocks::SmallBlocks(int num_blocks):
num_blocks_(num_blocks),
x_center_(num_blocks),
length_(num_blocks),
y_center_(num_blocks)
{
  // nothing to do
  for(int i=0; i<num_blocks_; i++){
    x_center_[i] = 0.2*i + 0.5;
    y_center_[i] = pow(-1, i) * 0.3;
    length_[i] = 0.1;
  }
}

SmallBlocks::SmallBlocks (std::vector<double> x_start, std::vector<double> length, std::vector<double> y_start):
x_center_(x_start),
length_(length),
y_center_(y_start)
{
  // nothing to do
}

double SmallBlocks::GetHeight (double x, double y) const
{
  double h = 0.0;

  for (int i=0; i<num_blocks_; i++) {
    if (x_center_[i]-length_[i]/2.0 <= x && x <= x_center_[i]+length_[i]/2.0)
      if(y_center_[i]-length_[i]/2.0 <= y && y <= y_center_[i]+length_[i]/2.0)
        h = height_;
  }

  return h;
}

LargeBlocks::LargeBlocks(int num_blocks):
num_blocks_(num_blocks),
x_center_(num_blocks),
length_(num_blocks),
y_center_(num_blocks)
{
  // nothing to do
  for(int i=0; i<num_blocks_; i++){
    x_center_[i] = 0.2*i + 0.7;
    y_center_[i] = pow(-1, i) * 0.3;
    length_[i] = 0.25;
  }
}

LargeBlocks::LargeBlocks (std::vector<double> x_start, std::vector<double> length, std::vector<double> y_start):
x_center_(x_start),
length_(length),
y_center_(y_start)
{
  // nothing to do
}

double LargeBlocks::GetHeight (double x, double y) const
{
  double h = 0.0;

  for (int i=0; i<num_blocks_; i++) {
    if (x_center_[i]-length_[i]/2.0 <= x && x <= x_center_[i]+length_[i]/2.0)
      if(y_center_[i]-length_[i]/2.0 <= y && y <= y_center_[i]+length_[i]/2.0)
        h = height_;
  }

  return h;

}

RandomStairs::RandomStairs():
gen(std::random_device()()),
dis(0, 1)
{
  num_stairs_ = int((dis(gen)*2) + 2);
  x_center_.reserve(num_stairs_);
  y_center_.reserve(num_stairs_);
  x_min_.reserve(num_stairs_);
  x_max_.reserve(num_stairs_);
  y_min_.reserve(num_stairs_);
  y_max_.reserve(num_stairs_);
  half_length_.reserve(num_stairs_);
  theta_.reserve(num_stairs_);
  height_.reserve(num_stairs_);

  for(int i=0; i<num_stairs_; i++){
    x_center_[i] = 0.5*i + 0.5;
    y_center_[i] = 0.0;
    half_length_[i] = 1;
    theta_[i] = ((dis(gen)-0.5) * M_PI/9.0) + M_PI_2; // [-pi/18, pi/18]
    height_[i] = 0.05 * dis(gen) + 0.1;

    double x_half_length = half_length_[i] * cos(theta_[i]);
    double y_half_length = half_length_[i] * sin(theta_[i]);
    x_min_[i] = x_center_[i] - x_half_length;
    x_max_[i] = x_center_[i] + x_half_length;
    y_min_[i] = y_center_[i] - y_half_length;
    y_max_[i] = y_center_[i] + y_half_length;

    edge_list_.emplace_back(Eigen::Vector2d(x_center_[i], y_center_[i]),
                            Eigen::Vector2d(cos(theta_[i]),sin(theta_[i])),
                            half_length_[i]);
  }
}

double RandomStairs::GetHeight(double x, double y) const
{
  double h = 0.0;

  for (int i=0; i<num_stairs_; i++) {
    // y_axis parallel
    double line_x_value = 0.0;
    if( fmod(theta_[i], M_PI_2) < 1e-5){
      line_x_value = x_center_[i];
    }
    else{
      line_x_value = ( y - (y_center_[i] - tan(theta_[i]) * x_center_[i])) / (tan(theta_[i]) + 1e-10);
    }

    if(x > line_x_value){
      h += height_[i];
    }
  }

  return h;
}

RandomBlocks::RandomBlocks(int num_blocks):
num_blocks_(num_blocks),
x_center_(num_blocks),
length_(num_blocks),
y_center_(num_blocks),
height_(num_blocks),
gen(std::random_device()()),
dis(0, 1)
{
  for(int i=0; i<num_blocks_; i++){
    x_center_[i] = dis(gen) + 0.6;
    y_center_[i] = pow(-1, i) * (dis(gen) + 0.1);
    length_[i] = 0.1 * dis(gen) + 0.15;
    height_[i] = 0.15 * dis(gen) + 0.05;

    double half_length_ = length_[i]/2.0;

    double x_min_ = x_center_[i] - half_length_;
    double x_max_ = x_center_[i] + half_length_;
    double y_min_ = y_center_[i] - half_length_;
    double y_max_ = y_center_[i] + half_length_;

    // parallel to y_axis
    edge_list_.emplace_back(Eigen::Vector2d(x_min_, y_center_[i]),
                            Eigen::Vector2d(0,1),
                            half_length_);

    // parallel to x_axis
    edge_list_.emplace_back(Eigen::Vector2d(x_center_[i], y_min_),
                            Eigen::Vector2d(1,0),
                            half_length_);

    // parallel to y_axis
    edge_list_.emplace_back(Eigen::Vector2d(x_max_, y_center_[i]),
                            Eigen::Vector2d(0,1),
                            half_length_);

    // parallel to x_axis
    edge_list_.emplace_back(Eigen::Vector2d(x_center_[i], y_max_),
                            Eigen::Vector2d(1,0),
                            half_length_);
  }

}
double RandomBlocks::GetHeight (double x, double y) const
{
  double h = 0.0;

  for (int i=0; i<num_blocks_; i++) {
    if (x_center_[i]-length_[i]/2.0 <= x && x <= x_center_[i]+length_[i]/2.0)
      if(y_center_[i]-length_[i]/2.0 <= y && y <= y_center_[i]+length_[i]/2.0)
        h = height_[i];
  }

  return h;

}

RandomHuddle::RandomHuddle(int num_huddle):
num_huddle_(num_huddle),
x_center_(num_huddle),
y_center_(num_huddle),
x_length_(num_huddle),
y_length_(num_huddle),
z_height_(num_huddle),
gen(std::random_device()()),
dis(0, 1)
{
  double huddle_start_x = 0.6;

  
  bool rl_flag = dis(gen) > 0.5;

  int rl_sign = rl_flag ? 1 : -1;

  for(int i=0; i<num_huddle_; i++){
    x_length_[i] = 0.4 + dis(gen) * 0.1;
    y_length_[i] = 0.6 + dis(gen) * 0.5;
    z_height_[i] = 0.1 + dis(gen) * 0.05;

    x_center_[i] = huddle_start_x + x_length_[i]/2.0;

    //true: 오른쪽에서 시작
    y_center_[i] = rl_sign * (-1 + y_length_[i]/2.0);
    
    // parallel to y_axis
    edge_list_.emplace_back(Eigen::Vector2d(huddle_start_x, y_center_[i]),
                            Eigen::Vector2d(0,1),
                            y_length_[i]/2.0);

    // parallel to x_axis
    edge_list_.emplace_back(Eigen::Vector2d(x_center_[i], 
                            y_center_[i] + rl_sign * y_length_[i]/2.0),
                            Eigen::Vector2d(1,0),
                            x_length_[i]/2.0);

    // parallel to y_axis
    edge_list_.emplace_back(Eigen::Vector2d(huddle_start_x + x_length_[i], y_center_[i]),
                            Eigen::Vector2d(0,1),
                            y_length_[i]/2.0);

    huddle_start_x += x_length_[i] + 0.2 + dis(gen) * 0.1;

    rl_sign *= -1;
  }

}
double RandomHuddle::GetHeight (double x, double y) const
{
  double h = 0.0;

  for (int i=0; i<num_huddle_; i++) {
    if (x_center_[i]-x_length_[i]/2.0 <= x && x <= x_center_[i]+x_length_[i]/2.0)
      if(y_center_[i]-y_length_[i]/2.0 <= y && y <= y_center_[i]+y_length_[i]/2.0)
        h = z_height_[i];
  }

  return h;
}

PerlinNoiseMap::PerlinNoiseMap():
  x_length_(5),
  y_length_(10),
  weight_(20), // 얼마나 흔들리게 할 것인지...
  z_(0.25),
  gen(std::random_device()()),
  dis(0, 1000)
{
  random_seed_ = dis(gen);
}

double PerlinNoiseMap::GetHeight (double x, double y) const
{
  PerlinNoise pn_(random_seed_);
  double h = 0.0;

  double x_ratio = x/x_length_;
  double y_ratio = y/y_length_;

  // start is flat
  if (x > 0.5 && x < 1.5){
    h = pn_.noise(weight_ * x_ratio, weight_ * y_ratio, 0.8);
    h = h * z_;
    // h = floor(h/0.025) * 0.025; // randomly 2.5cm steps
  }
    
  return h;
}

} /* namespace towr */

