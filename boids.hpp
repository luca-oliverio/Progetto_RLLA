#include <vector>
namespace bd { // namespace bd
struct boids
{
  double x, y, v_x, v_y; // parametri della posizione e velocità
};
// creo una classe di movimento
class Movement
{
  boids b;
  int n_b;
  double d, d_s, s, a, c;

 public:
  Movement(boids b_, int n_b_, double d_, double d_s_, double s_, double a_,
           double c_)
  {
    b_   = b;
    n_b_ = n_b;
    d_   = d;
    d_s_ = d_s;
    s_   = s;
    a_   = a;
    c_   = c;
  };
};
} // namespace bd
