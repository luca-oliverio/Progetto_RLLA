#include "boids_logic.hpp"
#include "boids_visualization.hpp"
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

int main()
{
  int n_b;
  double d, d_s, s, a, c;

  std::cout << "Inserisci numero boids: ";
  std::cin >> n_b;
  std::cout << "Inserisci distanza allineamento (d): ";
  std::cin >> d;
  std::cout << "Inserisci distanza separazione (d_s): ";
  std::cin >> d_s;
  std::cout << "Inserisci peso separazione (s): ";
  std::cin >> s;
  std::cout << "Inserisci peso allineamento (a): ";
  std::cin >> a;
  std::cout << "Inserisci peso coesione (c): ";
  std::cin >> c;

  std::srand(static_cast<unsigned int>(std::time(nullptr)));

  std::vector<bd::boid> boids(n_b);
  for (auto& b : boids) {
    b.x = std::rand() % 1600;
    b.y = std::rand() % 900;
  }

  bd::Movement simulation(boids, n_b, d, d_s, s, a, c);

  bd::BoidVisualization visualization(simulation);
  visualization.run();

  return 0;
}
