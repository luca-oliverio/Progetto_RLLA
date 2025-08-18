#ifndef BOIDS_LOGIC_HPP
#define BOIDS_LOGIC_HPP

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace bd {
// costanti che useremo nel codice
constexpr double max_speed     = 6.0;
constexpr double screen_width  = 1600.0;
constexpr double screen_height = 900.0;

// Posizione di un boid
struct Boid
{
  double x;
  double y;
};

// Velocità di un boid
struct Boid_vel
{
  double v_x;
  double v_y;
  Boid_vel(double a = 0.0, double b = 0.0);
  Boid_vel& operator+=(const Boid_vel& other);
};

struct Boid_Complete
{
  Boid b;
  Boid_vel b_v;
  Boid_Complete(const Boid& c, const Boid_vel& d);
};

Boid cm(const std::vector<Boid>& b);

// classe con i metodi che definiscono i movimenti dei boids
class Movement
{
  std::vector<Boid_Complete> boids;
  std::vector<Boid_vel> velocities;
  size_t n_b;
  double d;
  double d_s;
  double s;
  double a;
  double c;

  sf::Vector2f mouse_pos;
  bool mouse_pressed          = false;
  double mouse_force_strength = 0.3;
  bool mouse_force_active     = true;
  double mouse_force_radius   = 60.0;

 public:
  Movement(const std::vector<Boid_Complete>& b_, double d_, double d_s_,
           double s_, double a_, double c_);

  static double velocity(const Boid_vel& v);
  static double diff_pos(const Boid& i, const Boid& j);
  bool is_neighbor(const Boid& i, const Boid& j) const;

  // regole del moto
  Boid_vel rule1(const Boid& i, const Boid& j) const;
  Boid_vel rule2(const Boid_vel& i, const Boid_vel& mean_vel) const;
  Boid_vel rule3(const Boid& i, const Boid& center_mass) const;

  static void check_bord(Boid& i);
  static void limit_velocity(Boid_vel& v);

  void set_mouse_force(const sf::Vector2f& pos, bool pressed,
                       bool switch_mouse_force);
  bool is_mouse_force_active() const;
  double get_mouse_force_radius() const;

  // metodo principale
  void update(int frame);

  const std::vector<Boid>& get_boids() const;
  const std::vector<Boid_vel>& get_velocities() const;

  void print_stats(int frame) const;
};

} // namespace bd

#endif