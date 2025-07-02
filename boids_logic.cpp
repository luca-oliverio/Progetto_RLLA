#include "boids_logic.hpp"

namespace bd {

Boid_vel::Boid_vel(double a, double b)
    : v_x(a)
    , v_y(b)
{}

Boid_vel& Boid_vel::operator+=(const Boid_vel& other)
{
  v_x += other.v_x;
  v_y += other.v_y;
  return *this;
}

Boid_Complete::Boid_Complete(const Boid& c, const Boid_vel& d)
    : b(c)
    , b_v(d)
{}

Boid cm(const std::vector<Boid>& b)
{
  Boid total = std::accumulate(b.begin(), b.end(), Boid{0, 0},
                               [](const Boid& acc, const Boid& bo) {
                                 return Boid{acc.x + bo.x, acc.y + bo.y};
                               });
  int n      = static_cast<int>(b.size());
  return {total.x / n, total.y / n};
}

Movement::Movement(const std::vector<Boid_Complete>& b_, double d_, double d_s_,
                   double s_, double a_, double c_)
    : boids(b_)
    , n_b(static_cast<int>(b_.size()))
    , d(d_)
    , d_s(d_s_)
    , s(s_)
    , a(a_)
    , c(c_)
{
  velocities.reserve(n_b);
  for (const auto& bo : boids)
    velocities.push_back(bo.b_v);
}

double Movement::velocity(const Boid_vel& v)
{
  return std::sqrt(v.v_x * v.v_x + v.v_y * v.v_y);
}

double Movement::diff_pos(const Boid& i, const Boid& j)
{
  return (i.x - j.x) * (i.x - j.x) + (i.y - j.y) * (i.y - j.y);
}

bool Movement::is_neighbor(const Boid& i, const Boid& j) const
{
  return diff_pos(i, j) < d * d;
}

// Separazione: allontana se troppo vicini
Boid_vel Movement::rule1(const Boid& i, const Boid& j) const
{
  if (diff_pos(i, j) < d_s * d_s) {
    return {-s * (j.x - i.x), -s * (j.y - i.y)};
  }
  return {0.0, 0.0};
}

// Allineamento: avvicina alla velocità media dei vicini
Boid_vel Movement::rule2(const Boid_vel& i, const Boid_vel& avg_velocity) const
{
  return (n_b <= 1) ? Boid_vel{0.0, 0.0}
                    : Boid_vel{a * (avg_velocity.v_x - i.v_x),
                               a * (avg_velocity.v_y - i.v_y)};
}

// Coesione: avvicina al centro dei vicini
Boid_vel Movement::rule3(const Boid& i, const Boid& center_mass) const
{
  return {c * (center_mass.x - i.x), c * (center_mass.y - i.y)};
}

// Controlla i bordi e riposiziona se esce dallo schermo
void Movement::check_bord(Boid& i)
{
  if (i.x >= screen_width)
    i.x -= screen_width;
  if (i.x < 0)
    i.x += screen_width;
  if (i.y >= screen_height)
    i.y -= screen_height;
  if (i.y < 0)
    i.y += screen_height;
}

// Limita la velocità massima di un boid
void Movement::limit_velocity(Boid_vel& v)
{
  double speed = velocity(v);
  if (speed > max_speed) {
    double scale = max_speed / speed;
    v.v_x *= scale;
    v.v_y *= scale;
  }
}

void Movement::set_mouse_force(const sf::Vector2f& pos, bool pressed,
                               bool toggle_mouse_force)
{
  mouse_pos     = pos;
  mouse_pressed = pressed;
  if (toggle_mouse_force)
    mouse_force_active = !mouse_force_active;
}

bool Movement::is_mouse_force_active() const
{
  return mouse_force_active;
}

double Movement::get_mouse_force_radius() const
{
  return mouse_force_radius;
}

// Aggiorna la posizione dei boid ad ogni frame
void Movement::update(int frame)
{
  if (n_b <= 1) {
    if (n_b == 1) {
      limit_velocity(velocities[0]);
      boids[0].b.x += velocities[0].v_x;
      boids[0].b.y += velocities[0].v_y;
      check_bord(boids[0].b);
    }
    return;
  }

  std::vector<Boid_vel> vel_tot = velocities;

  for (int i = 0; i < n_b; ++i) {
    Boid& self = boids[i].b;
    Boid center_mass{0, 0};
    Boid_vel avg_velocity{0, 0};
    int neighbor_count = 0;

    for (int j = 0; j < n_b; ++j) {
      if (i == j)
        continue;

      const Boid& other = boids[j].b;
      if (is_neighbor(self, other)) {
        center_mass.x += other.x;
        center_mass.y += other.y;
        avg_velocity += velocities[j];
        neighbor_count++;
        vel_tot[i] += rule1(self, other); // Separazione
      }
    }

    if (neighbor_count > 0) {
      center_mass.x /= neighbor_count;
      center_mass.y /= neighbor_count;
      avg_velocity.v_x /= neighbor_count;
      avg_velocity.v_y /= neighbor_count;

      vel_tot[i] += rule2(velocities[i], avg_velocity); // Allineamento
      vel_tot[i] += rule3(self, center_mass);           // Coesione

      if (mouse_force_active) {
        double dx      = mouse_pos.x - self.x;
        double dy      = mouse_pos.y - self.y;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < mouse_force_radius * mouse_force_radius) {
          double dist = std::sqrt(dist_sq + 1e-6);
          double fx   = dx / dist;
          double fy   = dy / dist;
          double force =
              mouse_pressed ? -mouse_force_strength : mouse_force_strength;

          vel_tot[i].v_x += force * fx;
          vel_tot[i].v_y += force * fy;
        }
      }
    }

    limit_velocity(vel_tot[i]);
  }

  for (int i = 0; i < n_b; ++i) {
    boids[i].b.x += vel_tot[i].v_x;
    boids[i].b.y += vel_tot[i].v_y;
    velocities[i] = vel_tot[i];
    check_bord(boids[i].b);
  }

  print_stats(frame);
}

const std::vector<Boid>& Movement::get_boids() const
{
  static std::vector<Boid> boid_positions;
  boid_positions.clear();
  for (const auto& bc : boids)
    boid_positions.push_back(bc.b);
  return boid_positions;
}

const std::vector<Boid_vel>& Movement::get_velocities() const
{
  return velocities;
}

// Stampa alcune statistiche (velocità media, distanza media, deviazione
// standard)
void Movement::print_stats(int frame) const
{
  if (n_b < 2)
    return;

  std::vector<double> speeds(n_b);
  double total_speed = 0.0;

  for (int i = 0; i < n_b; ++i) {
    double v  = velocity(velocities[i]);
    speeds[i] = v;
    total_speed += v;
  }

  double mean_speed    = total_speed / n_b;
  double speed_std_dev = 0.0;
  for (double s : speeds)
    speed_std_dev += (s - mean_speed) * (s - mean_speed);
  speed_std_dev = std::sqrt(speed_std_dev / n_b);

  std::vector<double> distances;
  double total_distance = 0.0;

  for (int i = 0; i < n_b; ++i) {
    for (int j = i + 1; j < n_b; ++j) {
      double dx   = boids[i].b.x - boids[j].b.x;
      double dy   = boids[i].b.y - boids[j].b.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      distances.push_back(dist);
      total_distance += dist;
    }
  }

  int num_pairs        = static_cast<int>(distances.size());
  double mean_distance = (num_pairs > 0) ? total_distance / num_pairs : 0.0;

  double dist_std_dev = 0.0;
  for (double d : distances)
    dist_std_dev += (d - mean_distance) * (d - mean_distance);
  if (num_pairs > 0)
    dist_std_dev = std::sqrt(dist_std_dev / num_pairs);

  std::cout << "Frame " << frame << " | Vel. media: " << mean_speed
            << " | Dev. std. vel.: " << speed_std_dev
            << " | Dist. media: " << mean_distance
            << " | Dev. std. dist.: " << dist_std_dev << '\n';
}

} // namespace bd
