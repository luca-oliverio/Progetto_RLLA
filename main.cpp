#include "boids_logic.hpp"
#include <SFML/Graphics.hpp>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

int main()
{
  std::srand(std::time(0)); // Inizializza il generatore di numeri casuali

  int num_boids;
  double d, d_s, s, a, c;

  std::cout << "Inserisci il numero di boids: ";
  std::cin >> num_boids;

  std::cout
      << "Inserisci la distanza massima per il centro di separazione (d): ";
  std::cin >> d;

  std::cout << "Inserisci la distanza di sicurezza tra i boids (d_s): ";
  std::cin >> d_s;

  std::cout << "Inserisci il coefficiente di separazione (s): ";
  std::cin >> s;

  std::cout << "Inserisci il coefficiente di allineamento (a): ";
  std::cin >> a;

  std::cout << "Inserisci il coefficiente di coesione (c): ";
  std::cin >> c;

  std::vector<bd::Boid_Complete> iniziali;
  for (int i = 0; i < num_boids; ++i) {
    double x  = rand() % 1600;
    double y  = rand() % 900;
    double vx = ((rand() % 200) - 100) / 10.0;
    double vy = ((rand() % 200) - 100) / 10.0;
    iniziali.emplace_back(bd::Boid{x, y}, bd::Boid_vel{vx, vy});
  }

  bd::Movement mov(iniziali, d, d_s, s, a, c);

  sf::RenderWindow window(sf::VideoMode(1600, 900), "Boids Simulation");
  window.setFramerateLimit(60);

  int frame = 0;
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    mov.update(frame);

    window.clear(sf::Color::Black);

    const auto& boids      = mov.get_boids();
    const auto& velocities = mov.get_velocities();

    for (size_t i = 0; i < boids.size(); ++i) {
      sf::CircleShape shape(3.f);

      // Calcola l'intensità del colore in base alla velocità (0-1)
      double speed = std::sqrt(velocities[i].v_x * velocities[i].v_x
                               + velocities[i].v_y * velocities[i].v_y);
      double normalized_speed =
          std::min(speed / 6.0, 1.0); // Assumendo max_speed = 6

      // Gradiente da bianco (velocità 0) a rosso (velocità massima)
      sf::Uint8 red   = 255;
      sf::Uint8 green = 255 * (1.0 - normalized_speed);
      sf::Uint8 blue  = 255 * (1.0 - normalized_speed);

      shape.setFillColor(sf::Color(red, green, blue));
      shape.setPosition(boids[i].x, boids[i].y);
      window.draw(shape);
    }
    window.display();
    ++frame;
  }

  return 0;
}
