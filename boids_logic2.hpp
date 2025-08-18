#ifndef BOIDS_LOGIC_HPP
#define BOIDS_LOGIC_HPP

#include <SFML/Graphics.hpp>
#include <array>
#include <vector>

namespace bd {
using Position = std::array<double, 2>;
using Velocity = std::array<double, 2>;
void add_inplace(std::array<double, 2>& i, const std::array<double, 2>& other);
struct Boid
{
  Position pos;
  Velocity vel;
  explicit Boid(double x_ = 0, double y_ = 0, double v_x_ = 0, double v_y_ = 0);
  Boid& operator+=(const Boid& other);
};

Boid cm(const std::vector<Boid>& b);

// classe con i metodi che definiscono i movimenti dei boids
class Movement
{
  std::vector<Boid> boids;
  size_t n_b;
  double d;
  double d_s;
  double s;
  double a;
  double c;

  static constexpr double max_speed     = 360.0;
  static constexpr double screen_width  = 1600.0;
  static constexpr double screen_height = 900.0;

  sf::Vector2f mouse_pos;
  bool mouse_pressed                           = false;
  static constexpr double mouse_force_strength = 0.3;
  bool mouse_force_active                      = true;
  static constexpr double mouse_force_radius   = 60.0;

 public:
  Movement(const std::vector<Boid>& b_, double d_, double d_s_, double s_,
           double a_, double c_);
  std::vector<Position>& get_boids() const;
  std::vector<Velocity>& get_velocities() const;
  double get_speed(const Velocity& vel) const;
  double diff_pos(const Position& pos_i, const Position& pos_j) const;
  bool is_neighbor(const Position& pos_i, const Position& pos_j) const;

  // regole del moto
  Velocity rule1(const Position& pos_i, const Position& pos_j) const;
  Velocity rule2(const Velocity& i, const Velocity& mean_vel) const;
  Velocity rule3(const Position& i, const Position& center_mass) const;

  void check_bord(Position& i);
  void limit_velocity(Velocity& v);

  void set_mouse_force(const sf::Vector2f& pos, bool pressed,
                       bool switch_mouse_force);
  bool is_mouse_force_active() const;
  double get_mouse_force_radius() const;

  // metodo principale
  void update(int frame);

  void print_stats(int frame) const;
};

} // namespace bd
#endif