#ifndef BOIDS_LOGIC_HPP
#define BOIDS_LOGIC_HPP

#include <SFML/Graphics.hpp>
#include <array>
#include <cassert>
#include <iostream>
#include <vector>

namespace bd {

using Position = std::array<double, 2>;
using Velocity = std::array<double, 2>;

inline void add_inplace(std::array<double, 2>& i,
                        const std::array<double, 2>& other)
{
  i[0] += other[0];
  i[1] += other[1];
};

struct Boid
{
  Position pos;
  Velocity vel;
  explicit Boid(double x_ = 0, double y_ = 0, double v_x_ = 0, double v_y_ = 0);
};

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
  // vedere se aggiungere inline e confrontare dove mettere constexpr e dove
  // solo const
  sf::Vector2f mouse_pos;
  bool mouse_pressed                                  = false;
  bool mouse_force_active                             = true;
  inline static constexpr int mouse_force_radius      = 80;
  inline static constexpr double mouse_force_strength = 60;

  inline static double time_accum = 0.0; // accumulatore del tempo per le stats
  inline static constexpr double stats_interval =
      1.0; // ogni quanti secondi stampare

 public:
  inline static constexpr int max_speed     = 700;
  inline static constexpr int screen_width  = 1600;
  inline static constexpr int screen_height = 900;

  inline void push_back(const Boid& bo)
  {
    boids.push_back(bo);
    ++n_b;
    assert(n_b == boids.size());
  }

  Movement(const std::vector<Boid>& b_ = {}, double d_ = 0, double d_s_ = 0,
           double s_ = 0, double a_ = 0, double c_ = 0);
  std::vector<Position> get_positions() const;
  std::vector<Velocity> get_velocities() const;
  double get_speed(const Velocity& vel) const;
  double diff_pos2(const Position& pos_i, const Position& pos_j) const;
  bool is_neighbor(const Position& pos_i, const Position& pos_j) const;

  // regole del moto
  Velocity rule1(const Position& pos_i, const Position& pos_j) const;
  Velocity rule2(const Velocity& i, const Velocity& mean_vel) const;
  Velocity rule3(const Position& i, const Position& center_mass) const;

  void check_bord(Position& i);
  void limit_velocity(Velocity& v);

  void set_mouse_force(const sf::Vector2f& pos, bool pressed,
                       bool switch_mouse_force);
  inline bool is_mouse_force_active() const
  {
    return mouse_force_active;
  }
  inline double get_mouse_force_radius() const
  {
    return mouse_force_radius;
  }

  // metodo principale
  void update(int frame, double dt);

  void print_stats(int frame) const;
};

} // namespace bd
#endif