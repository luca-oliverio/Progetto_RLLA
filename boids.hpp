#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <vector>

namespace bd {

struct boid
{
  double x, y;
};

struct boid_vel
{
  double v_x, v_y;
  boid_vel(double a = 0.0, double b = 0.0);
  boid_vel& operator+=(const boid_vel& other);
};

boid cm(const std::vector<boid>& b);

class Movement
{
  std::vector<boid> b;
  std::vector<boid_vel> velocities;
  int n_b;
  double d, d_s, s, a, c;
  static constexpr double max_speed     = 10.0;
  static constexpr double screen_width  = 1600.0;
  static constexpr double screen_height = 900.0;

 public:
  Movement(const std::vector<boid>& b_, int n_b_, double d_, double d_s_,
           double s_, double a_, double c_);

  void update(int frame);

 private:
  double velocity(const boid_vel& boid) const;
  double diff_pos(const boid& i, const boid& j) const;
  boid_vel rule1(const boid& i, const boid& j) const;
  boid_vel rule2(const boid_vel& i, const boid_vel& j) const;
  boid_vel rule3(const boid& i, const boid& center_mass) const;
  void check_bord(boid& i) const;
  void limit_velocity(boid_vel& v) const;
  void print_stats(int frame, const std::vector<boid>& b,
                   const std::vector<boid_vel>& b_vel);
};

} // namespace bd

#endif // MOVEMENT_HPP
