#include "boids_logic.hpp"
#include <iostream>
#include <random>

int main()
{
  try {
    std::random_device r;
    std::default_random_engine eng{r()};
    std::uniform_real_distribution<double> dist(-1, 1);
    // Input parametri boids
    size_t n_b;
    double d, d_s, s, a, c;
    std::cout << "Inserisci il numero di boids: ";
    std::cin >> n_b;
    if (std::cin.fail()) {
      throw std::invalid_argument("numero di boids non valido");
    }

    std::cout << "Inserisci la distanza di interazione (d): ";
    std::cin >> d;
    if (std::cin.fail() || d <= 0) {
      throw std::invalid_argument(
          "La distanza di interazione deve essere positiva");
    }

    std::cout << "Inserisci la distanza di separazione (d_s): ";
    std::cin >> d_s;
    if (std::cin.fail() || d_s <= 0) {
      throw std::invalid_argument(
          "La distanza di separazione deve essere positiva");
    }
    if (d_s > d) {
      throw std::invalid_argument("La distanza di separazione non può essere "
                                  "maggiore della distanza di interazione");
    }

    std::cout << "Inserisci il coefficiente di separazione (s): ";
    std::cin >> s;
    if (std::cin.fail() || s < 0) {
      throw std::invalid_argument(
          "Il coefficiente di separazione deve essere positivo");
    }

    std::cout << "Inserisci il coefficiente di allineamento (a): ";
    std::cin >> a;
    if (std::cin.fail() || a < 0 || a > 1) {
      throw std::invalid_argument("Il coefficiente di allineamento deve essere "
                                  "un numero compreso tra 0 e 1");
    }

    std::cout << "Inserisci il coefficiente di coesione (c): ";
    std::cin >> c;
    if (std::cin.fail() || c < 0 || c > 1) {
      throw std::invalid_argument(
          "Il coefficiente di coesione deve essere compreso tra 0 e 1");
    }
    // Inizializzazione boids con posizioni e velocità casuali
    std::vector<bd::Boid> initials;

    auto random_boid = [&dist, &eng]() {
      double x  = std::fabs(dist(eng) * bd::Movement::screen_width);
      double y  = std::fabs(dist(eng) * bd::Movement::screen_height);
      double vx = dist(eng) * (bd::Movement::max_speed / std::sqrt(2));
      double vy = dist(eng) * (bd::Movement::max_speed / std::sqrt(2));
      return bd::Boid{x, y, vx, vy};
    };

    for (size_t i = 0; i < n_b; ++i) {
      initials.emplace_back(random_boid());
    }

    bd::Movement mov(initials, d, d_s, s, a, c);
    sf::RenderWindow window(
        sf::VideoMode(bd::Movement::screen_width, bd::Movement::screen_height),
        "Boids Simulation");
    const int FPS = 90;
    window.setFramerateLimit(FPS);

    sf::Vector2i mouse_position;
    bool is_mouse_pressed = false;
    const double dt       = (1. / FPS);

    for (int frame = 0; window.isOpen(); ++frame) {
      sf::Event event;
      bool switch_mouse_force = false;

      // Aggiornamento schermo e boids
      while (window.pollEvent(event)) {
        switch (event.type) {
        case sf::Event::Closed:
          window.close();
          break;

        case sf::Event::MouseMoved:
          mouse_position = sf::Mouse::getPosition(window);
          break;

        case sf::Event::MouseButtonPressed:
          if (event.mouseButton.button == sf::Mouse::Left)
            is_mouse_pressed = true;
          else if (event.mouseButton.button == sf::Mouse::Right)
            switch_mouse_force = true;
          break;

        case sf::Event::MouseButtonReleased:
          if (event.mouseButton.button == sf::Mouse::Left)
            is_mouse_pressed = false;
          break;

        default:
          break;
        }
      }

      mov.set_mouse_force(sf::Vector2f(static_cast<float>(mouse_position.x),
                                       static_cast<float>(mouse_position.y)),
                          is_mouse_pressed, switch_mouse_force);

      // generatore di boids
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
        mov.push_back_(random_boid());
      }
      // rimuove i boids
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::R)) {
        mov.remove_();
      }

      mov.update(frame, dt);

      window.clear(sf::Color::Black);

      // Verifica se il mouse è visivamente dentro la finestra
      const bool mouse_in_window =
          mouse_position.x >= bd::Movement::edge
          && mouse_position.x
                 <= (bd::Movement::screen_width - bd::Movement::edge)
          && mouse_position.y >= bd::Movement::edge
          && mouse_position.y
                 <= (bd::Movement::screen_height - bd::Movement::edge);

      // Disegna il raggio della forza del mouse se attiva
      if (mov.is_mouse_force_active() && mouse_in_window) {
        mov.draw_mouse(mouse_position, is_mouse_pressed, window);
      }

      // Disegna ogni boid con colore in base alla velocità
      for (const bd::Boid& b : mov.get_boids()) {
        mov.draw_boids(b.pos, b.vel, window);
      }

      window.display();
    }
    return 0;
  }
  // controllo validità parametri
  catch (const std::invalid_argument& e) {
    std::cerr << "Parametro non valido: " << e.what() << '\n';
    std::cerr << "Programma terminato.\n";
    return EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "Errore: " << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Errore sconosciuto\n";
    return EXIT_FAILURE;
  }
}