#include "boids_logic.hpp"
#include <cmath>

namespace bd {

Boid::Boid(double x_, double y_, double v_x_, double v_y_)
    : pos{x_, y_}
    , vel{v_x_, v_y_}
{}

Movement::Movement(const std::vector<Boid>& b_, double d_, double d_s_,
                   double s_, double a_, double c_)
    : boids(b_)
    , n_b(b_.size())
    , d(d_)
    , d_s(d_s_)
    , s(s_)
    , a(a_)
    , c(c_)
{}

std::vector<Position> Movement::get_positions() const
{
  std::vector<Position> boids_positions;
  boids_positions.reserve(n_b);
  for (const auto& bc : boids)
    boids_positions.push_back(bc.pos);
  return boids_positions;
}

std::vector<Velocity> Movement::get_velocities() const
{
  std::vector<Position> boids_velocities;
  boids_velocities.reserve(n_b);
  for (const auto& bc : boids)
    boids_velocities.push_back(bc.vel);
  return boids_velocities;
}

double Movement::get_speed(const Velocity& vel) const
{
  return std::sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
}

double Movement::diff_pos2(const Position& pos_i, const Position& pos_j) const
{
  return (pos_i[0] - pos_j[0]) * (pos_i[0] - pos_j[0])
       + (pos_i[1] - pos_j[1]) * (pos_i[1] - pos_j[1]);
}

bool Movement::is_neighbor(const Position& pos_i, const Position& pos_j) const
{
  return (diff_pos2(pos_i, pos_j) < d * d);
}
// il primo elemento dell'array è la x, il secondo è la y
// Separazione: allontana se troppo vicini
Velocity Movement::rule1(const Position& pos_i, const Position& pos_j) const
{
  if (diff_pos2(pos_i, pos_j) < d_s * d_s) {
    return {-s * (pos_j[0] - pos_i[0]), -s * (pos_j[1] - pos_i[1])};
  }
  return {0., 0.};
}

// Allineamento: avvicina alla velocità media dei vicini
Velocity Movement::rule2(const Velocity& i, const Velocity& mean_vel) const
{
  return (n_b <= 1)
           ? Velocity{0., 0.}
           : Velocity{a * (mean_vel[0] - i[0]), a * (mean_vel[1] - i[1])};
}

// Coesione: avvicina al centro dei vicini
Velocity Movement::rule3(const Position& i, const Position& center_mass) const
{
  return {c * (center_mass[0] - i[0]), c * (center_mass[1] - i[1])};
}

// effetto pacman
void Movement::check_bord(Position& i)
{
  if (i[0] >= screen_width)
    i[0] -= screen_width;
  if (i[0] < 0)
    i[0] += screen_width;
  if (i[1] >= screen_height)
    i[1] -= screen_height;
  if (i[1] < 0)
    i[1] += screen_height;
}

void Movement::limit_velocity(Velocity& v)
{
  double speed = get_speed(v);
  if (speed > max_speed) {
    double scale = max_speed / speed;
    v[0] *= scale;
    v[1] *= scale;
  }
}

// interazione puntatore
void Movement::set_mouse_force(const sf::Vector2f& pos, bool pressed,
                               bool switch_mouse_force)
{
  mouse_pos     = pos;
  mouse_pressed = pressed;
  if (switch_mouse_force)
    mouse_force_active = !mouse_force_active;
}

// Aggiorna la posizione e la velocità dei boid ad ogni frame
void Movement::update(int frame, double dt)
{
  assert(frame >= 0);
  if (n_b < 1)
    return;
  // creiamo un vettore che viene inizializzato alle velocità dei boids
  std::vector<Velocity> vel_tot = get_velocities();

  for (size_t i = 0; i < n_b; ++i) {
    Boid& self = boids[i];
    Position center_mass{};
    Velocity mean_vel{};
    int neighbor_count = 0;

    for (size_t j = 0; j < n_b; ++j) {
      if (i == j)
        continue;
      const Boid& other = boids[j];
      if (is_neighbor(boids[i].pos, boids[j].pos)) {
        center_mass[0] +=
            other.pos[0]; // calcolo centro di massa con i boids vicini
        center_mass[1] += other.pos[1];
        mean_vel[0] +=
            other.vel[0]; // calcolo velocità media con i boids vicini
        mean_vel[1] += other.vel[1];
        neighbor_count++;
        add_inplace(vel_tot[i], (rule1(self.pos, other.pos)));
      }
    }

    if (neighbor_count > 0) {
      center_mass[0] /= neighbor_count;
      center_mass[1] /= neighbor_count;
      mean_vel[0] /= neighbor_count;
      mean_vel[1] /= neighbor_count;

      add_inplace(vel_tot[i], (rule2(boids[i].vel, mean_vel)));
      add_inplace(vel_tot[i], (rule3(self.pos, center_mass)));
    }
    if (mouse_force_active) {
      double dx      = mouse_pos.x - self.pos[0];
      double dy      = mouse_pos.y - self.pos[1];
      double dist_sq = dx * dx + dy * dy;

      if (dist_sq < mouse_force_radius * mouse_force_radius) {
        double dist = std::sqrt(dist_sq + 1e-6);
        double fx   = dx / dist;
        double fy   = dy / dist;
        double force =
            mouse_pressed ? -mouse_force_strength : mouse_force_strength;

        vel_tot[i][0] += force * fx;
        vel_tot[i][1] += force * fy;
      }
    }

    limit_velocity(vel_tot[i]);
  }
  for (size_t i = 0; i < n_b; ++i) {
    boids[i].pos[0] += (vel_tot[i][0]) * dt;
    boids[i].pos[1] += (vel_tot[i][1]) * dt;
    boids[i].vel = vel_tot[i];
    check_bord(boids[i].pos);
  }
  time_accum += dt;
  if (time_accum >= stats_interval) {
    print_stats(frame);
    time_accum -= (stats_interval);
  }
}
// Stampa alcune statistiche (velocità media, distanza media, deviazione
// standard)
void Movement::print_stats(int frame) const
{
  if (n_b < 2)
    return;

  std::vector<double> speeds(n_b);
  double total_speed = 0.0;

  for (size_t i = 0; i < n_b; ++i) {
    double v  = get_speed(boids[i].vel);
    speeds[i] = v;
    total_speed += v;
  }
  double n_b2          = static_cast<double>(n_b);
  double mean_speed    = total_speed / n_b2;
  double speed_std_dev = 0.0;
  for (double sp : speeds)
    speed_std_dev += (sp - mean_speed) * (sp - mean_speed);
  speed_std_dev = std::sqrt(speed_std_dev / n_b2);

  std::vector<double> distances;
  double total_distance = 0.0;

  for (size_t i = 0; i < n_b; ++i) {
    for (size_t j = i + 1; j < n_b; ++j) {
      double dx   = boids[i].pos[0] - boids[j].pos[0];
      double dy   = boids[i].pos[1] - boids[j].pos[1];
      double dist = std::sqrt(dx * dx + dy * dy);
      distances.push_back(dist);
      total_distance += dist;
    }
  }

  int num_pairs        = static_cast<int>(distances.size());
  double mean_distance = (num_pairs > 0) ? total_distance / num_pairs : 0.0;

  double dist_std_dev = 0.0;
  for (double dist : distances)
    dist_std_dev += (dist - mean_distance) * (dist - mean_distance);
  if (num_pairs > 0)
    dist_std_dev = std::sqrt(dist_std_dev / num_pairs);

  std::cout << "Frame " << frame << " | Vel. media: " << mean_speed
            << " | Dev. std. vel.: " << speed_std_dev
            << " | Dist. media: " << mean_distance
            << " | Dev. std. dist.: " << dist_std_dev << " | N_b: " << n_b
            << '\n';
}
} // namespace bd