#ifndef BoidSLOGIC_HPP
#define BoidSLOGIC_HPP

#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>
#include <SFML/Graphics.hpp>

namespace bd {

struct Boid {
  double x, y;
};

struct Boid_vel {
  double v_x, v_y;
  Boid_vel(double a = 0.0, double b = 0.0)
      : v_x(a), v_y(b) {}

  Boid_vel& operator+=(const Boid_vel& other) {
    v_x += other.v_x;
    v_y += other.v_y;
    return *this;
  }
};

struct Boid_Complete {
  Boid b;
  Boid_vel b_v;
  Boid_Complete(Boid c, Boid_vel d)
      : b(c), b_v(d) {}
};

Boid cm(const std::vector<Boid>& b) {
  Boid total = std::accumulate(b.begin(), b.end(), Boid{0, 0},
                               [](const Boid& acc, const Boid& bo) {
                                 return Boid{acc.x + bo.x, acc.y + bo.y};
                               });
  int n = b.size();
  return {total.x / n, total.y / n};
}

class Movement {
  std::vector<Boid> b;
  std::vector<Boid_vel> velocities;
  std::vector<Boid_Complete> boids;
  int n_b;
  double d, d_s, s, a, c;
  static constexpr double max_speed = 6.0;
  static constexpr double screen_width = 1600.0;
  static constexpr double screen_height = 900.0;

 public:
  Movement(const std::vector<Boid_Complete>& b_, double d_, double d_s_,
           double s_, double a_, double c_)
      : boids(b_), n_b(b_.size()), d(d_), d_s(d_s_), s(s_), a(a_), c(c_) {
    for (const auto& bo : boids) {
      b.push_back(bo.b);
      velocities.push_back(bo.b_v);
    }
  }

  double velocity(const Boid_vel& Boid) const {
    return std::sqrt(Boid.v_x * Boid.v_x + Boid.v_y * Boid.v_y);
  }

  double diff_pos(const Boid& i, const Boid& j) const {
    return (i.x - j.x) * (i.x - j.x) + (i.y - j.y) * (i.y - j.y);
  }
  bool is_neighbor(const Boid& i, const Boid& j) const {
    return diff_pos(i, j) < d * d;
}
const std::vector<Boid_vel>& get_velocities() const {
    return velocities;
}
  Boid_vel rule1(const Boid& i, const Boid& j) const {
    if (diff_pos(i, j) < d_s * d_s) {
      return {-s * (j.x - i.x), -s * (j.y - i.y)};
    }
    return {0., 0.};
  }

  Boid_vel rule2(const Boid_vel& i, const Boid_vel& avg_velocity) const {
    if (n_b <= 1) return {0., 0.};
    return {a * (avg_velocity.v_x - i.v_x), a * (avg_velocity.v_y - i.v_y)};
}

  Boid_vel rule3(const Boid& i, const Boid& center_mass) const {
    return {c * (center_mass.x - i.x), c * (center_mass.y - i.y)};
  }

  void check_bord(Boid& i) const {
    if (i.x >= screen_width)
      i.x -= screen_width;
    if (i.x < 0)
      i.x += screen_width;
    if (i.y >= screen_height)
      i.y -= screen_height;
    if (i.y < 0)
      i.y += screen_height;
  }

  void limit_velocity(Boid_vel& v) const {
    double speed = velocity(v);
    if (speed > max_speed) {
      v.v_x = (v.v_x / speed) * max_speed;
      v.v_y = (v.v_y / speed) * max_speed;
    }
  }
void update(int frame) {
    if (n_b <= 1) {
        if (n_b == 1) {
            limit_velocity(velocities[0]);
            b[0].x += velocities[0].v_x;
            b[0].y += velocities[0].v_y;
            check_bord(b[0]);
        }
        return;
    }

    std::vector<Boid_vel> vel_tot = velocities;
    
    for (int i = 0; i < n_b; ++i) {
        // Calcola centro di massa e velocità media solo dei vicini
        Boid center_mass = {0, 0};
        Boid_vel avg_velocity = {0, 0};
        int neighbor_count = 0;
        
        for (int j = 0; j < n_b; ++j) {
            if (i == j) continue;
            if (is_neighbor(b[i], b[j])) {
                center_mass.x += b[j].x;
                center_mass.y += b[j].y;
                avg_velocity.v_x += velocities[j].v_x;
                avg_velocity.v_y += velocities[j].v_y;
                neighbor_count++;
                
                vel_tot[i] += rule1(b[i], b[j]);
            }
        }
        
        if (neighbor_count > 0) {
            center_mass.x /= neighbor_count;
            center_mass.y /= neighbor_count;
            avg_velocity.v_x /= neighbor_count;
            avg_velocity.v_y /= neighbor_count;
            
            vel_tot[i] += rule2(velocities[i], avg_velocity);
            vel_tot[i] += rule3(b[i], center_mass);
        }
        
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

  const std::vector<Boid>& get_boids() const {
    return b;
  }

  void print_stats(int frame, const std::vector<Boid>& b,
                   const std::vector<Boid_vel>& b_vel) {
    const int n = b.size();
    if (n < 2)
      return;

    double total_speed = 0.0;
    std::vector<double> speeds;
    for (int i = 0; i < n; ++i) {
      double v = std::sqrt(b_vel[i].v_x * b_vel[i].v_x +
                           b_vel[i].v_y * b_vel[i].v_y);
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
};

} // namespace bd

#endif