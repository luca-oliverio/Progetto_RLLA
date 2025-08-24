#ifndef BOIDS_LOGIC_HPP
#define BOIDS_LOGIC_HPP

#include <SFML/Graphics.hpp>
#include <array>
#include <vector>

namespace bd {
using Position = std::array<double, 2>;
using Velocity = std::array<double, 2>;

inline void add_inplace(std::array<double, 2>& i,
                        const std::array<double, 2>& other)
{
  i[0] += other[0];
  i[1] += other[1];
}

struct Boid
{
  Position pos;
  Velocity vel;
  explicit Boid(double x_ = 0, double y_ = 0, double v_x_ = 0, double v_y_ = 0);
}; // il primo elemento degli array riguarda le coordinate sulle x, il secondo
   // sulle y

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

  sf::Vector2f mouse_pos;
  inline static bool mouse_pressed             = false;
  inline static bool mouse_force_active        = false;
  static constexpr int mouse_force_radius      = 80;
  static constexpr double mouse_force_strength = 40;

  inline static double time_accum        = 0.0;
  static constexpr double stats_interval = 1.0; // ogni quanti secondi stampare

 public:
  static constexpr int max_speed     = 700;
  static constexpr int screen_width  = 1600;
  static constexpr int screen_height = 900;
  static constexpr int edge        = 30;

  explicit Movement(const std::vector<Boid>& b_ = {}, double d_ = 0,
                    double d_s_ = 0, double s_ = 0, double a_ = 0,
                    double c_ = 0);

  void push_back_(const Boid& bo);
  void remove_();

  const std::vector<Boid>& get_boids() const;
  double get_speed(const Velocity& vel) const;
  double diff_pos2(const Position& pos_i, const Position& pos_j) const;
  bool is_neighbor(const Position& pos_i, const Position& pos_j) const;

  // regole del moto
  Velocity rule1(const Position& pos_i, const Position& pos_j) const;
  Velocity rule2(const Velocity& vel_i, const Velocity& mean_vel) const;
  Velocity rule3(const Position& vel_i, const Position& center_mass) const;

  void check_sides(Position& i);
  void limit_velocity(Velocity& v);

  void set_mouse_force(const sf::Vector2f& pos, bool pressed,
                       bool switch_mouse_force);
  inline bool is_mouse_force_active() const
  {
    return mouse_force_active;
  }

  void apply_neighbor_rules(size_t i, Velocity& v_i);
  void apply_mouse_force(const Boid& self, Velocity& v_i);
  void update_pos_vel(std::vector<Velocity>& vel_tot, double dt);

  void time_stats(const int frame, const double dt);

  // metodo principale
  void update(int frame, double dt);

  void print_stats(int frame) const;

  static void draw_mouse(const sf::Vector2i& mouse_position,
                         const bool is_mouse_pressed, sf::RenderWindow& window);
  void draw_boids(const Position& p, const Velocity& v,
                  sf::RenderWindow& window) const;
};

} // namespace bd
#endif