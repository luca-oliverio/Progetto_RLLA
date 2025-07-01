#ifndef BOIDSLOGIC_CPP
#define BOIDSLOGIC_CPP

#include "boids_logic.hpp"

namespace bd {

boid_vel::boid_vel(double a, double b)
    : v_x(a)
    , v_y(b)
{}

boid_vel& boid_vel::operator+=(const boid_vel& other)
{
  v_x += other.v_x;
  v_y += other.v_y;
  return *this;
}

boid cm(const std::vector<boid>& b)
{
  boid total = std::accumulate(b.begin(), b.end(), boid{0, 0},
                               [](const boid& acc, const boid& bo) {
                                 return boid{acc.x + bo.x, acc.y + bo.y};
                               });
  int n      = b.size();
  return {total.x / n, total.y / n};
}

Movement::Movement(const std::vector<boid>& b_, int n_b_, double d_, double d_s_,
                   double s_, double a_, double c_)
    : b(b_)
    , n_b(n_b_)
    , d(d_)
    , d_s(d_s_)
    , s(s_)
    , a(a_)
    , c(c_)
{
  velocities.resize(n_b_, boid_vel(0, 0));
}

double Movement::velocity(const boid_vel& boid) const
{
  return std::sqrt(boid.v_x * boid.v_x + boid.v_y * boid.v_y);
}

double Movement::diff_pos(const boid& i, const boid& j) const
{
  return (i.x - j.x) * (i.x - j.x) + (i.y - j.y) * (i.y - j.y);
}

boid_vel Movement::rule1(const boid& i, const boid& j) const
{
  if (diff_pos(i, j) < d_s * d_s) {
    return {-s * (j.x - i.x), -s * (j.y - i.y)};
  }
  return {0., 0.};
}

boid_vel Movement::rule2(const boid_vel& i, const boid_vel& j) const
{
  if (n_b <= 1)
    return {0., 0.};
  return {(a / (n_b - 1)) * (j.v_x - i.v_x),
          (a / (n_b - 1)) * (j.v_y - i.v_y)};
}

boid_vel Movement::rule3(const boid& i, const boid& center_mass) const
{
  return {c * (center_mass.x - i.x), c * (center_mass.y - i.y)};
}

void Movement::check_bord(boid& i) const
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

void Movement::limit_velocity(boid_vel& v) const
{
  double speed = velocity(v);
  if (speed > max_speed) {
    v.v_x = (v.v_x / speed) * max_speed;
    v.v_y = (v.v_y / speed) * max_speed;
  }
}

void Movement::update(int frame)
{
  if (n_b <= 1) {
    if (n_b == 1) {
      limit_velocity(velocities[0]);
      b[0].x += velocities[0].v_x;
      b[0].y += velocities[0].v_y;
      check_bord(b[0]);
    }
    return;
  }

  std::vector<boid_vel> vel_tot(n_b, boid_vel{0.0, 0.0});
  boid center = cm(b);
  double const sum_x = center.x * n_b;
  double const sum_y = center.y * n_b;
  for (int i = 0; i < n_b; ++i) {
    boid cm_excl = {(sum_x - b[i].x) / (n_b - 1),
                    (sum_y - b[i].y) / (n_b - 1)};
    for (int j = 0; j < n_b; ++j) {
      if (i == j)
        continue;
      vel_tot[i] += rule1(b[i], b[j]);
      boid_vel align = rule2(velocities[i], velocities[j]);
      vel_tot[i] += align;
    }

    vel_tot[i] += rule3(b[i], cm_excl);
    limit_velocity(vel_tot[i]);
  }

  for (int i = 0; i < n_b; ++i) {
    b[i].x += vel_tot[i].v_x;
    b[i].y += vel_tot[i].v_y;

    velocities[i] = vel_tot[i];

    check_bord(b[i]);
  }
  print_stats(frame, b, velocities);
}

const std::vector<boid>& Movement::get_boids() const
{
  return b;
}

const std::vector<boid_vel>& Movement::get_velocities() const
{
  return velocities;
}

void Movement::add_velocity(size_t index, double vx, double vy)
{
  if (index < velocities.size()) {
    velocities[index].v_x += vx;
    velocities[index].v_y += vy;
  }
}

void Movement::print_stats(int frame, const std::vector<boid>& b,
                           const std::vector<boid_vel>& b_vel)
{
  const int n = b.size();
  if (n < 2)
    return;

  double total_speed = 0.0;
  std::vector<double> speeds;
  for (int i = 0; i < n; ++i) {
    double v = std::sqrt(b_vel[i].v_x * b_vel[i].v_x + b_vel[i].v_y * b_vel[i].v_y);
    speeds.push_back(v);
    total_speed += v;
  }
  double mean_speed = total_speed / n;

  double speed_std_dev = 0.0;
  for (double s : speeds) {
    speed_std_dev += (s - mean_speed) * (s - mean_speed);
  }
  speed_std_dev = std::sqrt(speed_std_dev / n);

  double total_distance = 0.0;
  std::vector<double> distances;
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      double dx = b[i].x - b[j].x;
      double dy = b[i].y - b[j].y;
      double dist = std::sqrt(dx * dx + dy * dy);
      distances.push_back(dist);
      total_distance += dist;
    }
  }

  int num_pairs = distances.size();
  double mean_distance = (num_pairs > 0) ? total_distance / num_pairs : 0.0;

  double dist_std_dev = 0.0;
  for (double d : distances) {
    dist_std_dev += (d - mean_distance) * (d - mean_distance);
  }
  if (num_pairs > 0) {
    dist_std_dev = std::sqrt(dist_std_dev / num_pairs);
  }

  std::cout << "Frame " << frame << " | Vel. media: " << mean_speed
            << " | Dev. std. vel.: " << speed_std_dev
            << " | Dist. media: " << mean_distance
            << " | Dev. std. dist.: " << dist_std_dev << '\n';
}

} // namespace bd
#endif