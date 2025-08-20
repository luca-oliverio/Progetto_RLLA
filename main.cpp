#include "boids_logic.hpp"
#include <random>

int main()
{
  try {
    std::random_device r;
    std::default_random_engine eng{r()};
    std::uniform_real_distribution<double> dist(-1, 1);
    int n_b;
    double d, d_s, s, a, c;
    // Input parametri boids
    std::cout << "Inserisci il numero di boids: ";
    std::cin >> n_b;
    if (std::cin.fail() || n_b < 0) {
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
          "Il coefficiente di coesione deve essere un numero positivo");
    }
    // Inizializzazione boids con posizioni e velocità casuali
    std::vector<bd::Boid> initials;
    initials.reserve(static_cast<size_t>(n_b));
    for (int i = 0; i < n_b; ++i) {
      const double x  = std::fabs(dist(eng) * (bd::Movement::screen_width));
      const double y  = std::fabs(dist(eng) * (bd::Movement::screen_height));
      const double vx = dist(eng) * (bd::Movement::max_speed / sqrt(2));
      const double vy = dist(eng) * (bd::Movement::max_speed / sqrt(2));
      initials.emplace_back(bd::Boid{x, y, vx, vy});
    }
    bd::Movement mov(initials, d, d_s, s, a, c);
    sf::RenderWindow window(sf::VideoMode(1600, 900), "Boids Simulation");
    const int FPS = 90;
    window.setFramerateLimit(FPS);

    sf::Vector2i mouse_position;
    bool is_mouse_pressed = false;
    int frame             = 0;
    double dt             = (1. / FPS);

    while (window.isOpen()) {
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
      // generatore di boid
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
        const double x_  = std::fabs(dist(eng) * (bd::Movement::screen_width));
        const double y_  = std::fabs(dist(eng) * (bd::Movement::screen_height));
        const double vx_ = dist(eng) * (bd::Movement::max_speed / sqrt(2));
        const double vy_ = dist(eng) * (bd::Movement::max_speed / sqrt(2));
        mov.push_back(bd::Boid{x_, y_, vx_, vy_});
      }

      mov.update(frame, dt);

      window.clear(sf::Color::Black);

      // Verifica se il mouse è visivamente dentro la finestra
      static constexpr int margin = 30;
      const bool mouse_in_window =
          mouse_position.x >= margin
          && mouse_position.x <= (bd::Movement::screen_width - margin)
          && mouse_position.y >= margin
          && mouse_position.y <= (bd::Movement::screen_height - margin);

      // Disegna il raggio della forza del mouse se attiva
      if (mov.is_mouse_force_active() && mouse_in_window) {
        const float radius = static_cast<float>(mov.get_mouse_force_radius());
        sf::CircleShape circle(radius);
        circle.setOrigin(radius, radius);
        circle.setPosition(static_cast<float>(mouse_position.x),
                           static_cast<float>(mouse_position.y));

        circle.setFillColor(is_mouse_pressed
                                ? sf::Color(255, 0, 0, 20) 
                                : sf::Color(0, 255, 0, 20));

        circle.setOutlineThickness(3.f);
        circle.setOutlineColor(is_mouse_pressed
                                   ? sf::Color(255, 0, 0, 40) 
                                   : sf::Color(0, 255, 0, 40));

        window.draw(circle);
      }

      // Disegna ogni boid con colore in base alla velocità
      const std::vector<bd::Position>& positions  = mov.get_positions();
      const std::vector<bd::Velocity>& velocities = mov.get_velocities();

      for (std::size_t i = 0; i < positions.size(); ++i) {
        const bd::Position& p = positions[i];
        const bd::Position& v = velocities[i];

        sf::CircleShape shape(3.f);

        const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1]);
        const double normalized_speed =
            std::min(speed / bd::Movement::max_speed, 1.);

        const sf::Uint8 red = 255;
        const sf::Uint8 green =
            static_cast<sf::Uint8>(255 * (1. - normalized_speed));
        const sf::Uint8 blue = green;

        shape.setFillColor(sf::Color(red, green, blue));
        shape.setPosition(static_cast<float>(p[0]), static_cast<float>(p[1]));
        window.draw(shape);
      }

      window.display();
      ++frame;
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