#include <cmath>
#include <vector>
namespace bd { // namespace bd
struct boids
{
  double x, y; // parametri della posizione e velocità
};
struct boids_vel
{
  double v_x, v_y;
  boids_vel(double a, double b)
  {
    v_x = a;
    v_y = b;
  }
};

boids cm(std::vector<boids> b)
{
  double sum_x = 0;
  double sum_y = 0;
  int n        = b.size();
  for (const auto& boid : b) {
    sum_x += boid.x;
    sum_y += boid.y;
  }
  return boids{sum_x / n, sum_y / n};
};
// creo una classe di movimento
class Movement
{
  std::vector<boids> b;
  int n_b;
  double d, d_s, s, a, c;

 public:
  Movement(std::vector<boids> b_, int n_b_, double d_, double d_s_, double s_,
           double a_, double c_)
  {
    b   = b_;
    n_b = n_b_;
    d   = d_;
    d_s = d_s_;
    s   = s_;
    a   = a_;
    c   = c_;
  };
  double velocity(boids_vel boid)
  { // calcolo il modulo della velocità per ogni boid
    return (sqrt(boid.v_x * boid.v_x + boid.v_y * boid.v_y));
  }
  double diff_pos(boids i, boids j)
  { // calcolo la differenza tra le posizioni dei boid i e j
    return (sqrt((i.x - j.x) * (i.x - j.x) + (i.y - j.y) * (i.y - j.y)));
  }
  boids_vel rule1(boids i, boids j)
  {
    if (diff_pos(i, j) < d_s) {
      return boids_vel(-s * (j.x - i.x), -s * (j.y - i.y));
    } else
      return boids_vel(0., 0.);
  };
  boids_vel rule2(boids_vel i, boids_vel j)
  { // a minore di 1, VERIFICA
    return boids_vel((a / (n_b - 1)) * (j.v_x - i.v_x),
                     (a / (n_b - 1)) * (j.v_y - i.v_y));
  };
  boids_vel rule3(boids i, boids j)
  {
    j   = cm(b); // voglio togliere il boid i dal calcolo del centro di massa
    j.x = (j.x * n_b - i.x) / (n_b - 1);
    j.y = (j.y * n_b - i.y) / (n_b - 1);
    return boids_vel(c * (j.x - i.x), c * (j.y - i.y));
  };
};
} // namespace bd