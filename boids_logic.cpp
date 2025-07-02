class Movement {
  std::vector<Boid> b;
  std::vector<Boid_vel> velocities;
  std::vector<Boid_Complete> boids;
  int n_b;
  double d, d_s, s, a, c;
  static constexpr double max_speed = 4.0;
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

  // Modifica della regola 1 per applicarla solo ai boids vicini
  Boid_vel rule1(const Boid& i, const Boid& j) const {
    if (diff_pos(i, j) < d * d) {  // Usa la distanza d
      return {-s * (j.x - i.x), -s * (j.y - i.y)};
    }
    return {0., 0.};
  }

  Boid_vel rule2(const Boid_vel& i, const Boid_vel& j) const {
    if (n_b <= 1)
      return {0., 0.};
    else return {(a / (n_b - 1)) * (j.v_x - i.v_x),
            (a / (n_b - 1)) * (j.v_y - i.v_y)};
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
    Boid center = cm(b);
    double sum_x = center.x * n_b;
    double sum_y = center.y * n_b;

    for (int i = 0; i < n_b; ++i) {
        Boid cm_excl = {(sum_x - b[i].x) / (n_b - 1), (sum_y - b[i].y) / (n_b - 1)};
        for (int j = 0; j < n_b; ++j) {
            if (i == j) continue;

            // Aggiungi la regola 1 solo se i boids sono vicini (distanza < d)
            if (diff_pos(b[i], b[j]) < d * d) {
                vel_tot[i] += rule1(b[i], b[j]);
            }
            
            vel_tot[i] += rule2(velocities[i], velocities[j]);
        }
        vel_tot[i] += rule3(b[i], cm_excl);
        limit_velocity(vel_tot[i]);
    }

    for (int i = 0; i < n_b; ++i) {
        b[i].x += vel_tot[i].v_x;  // Usa la nuova velocità
        b[i].y += vel_tot[i].v_y;
        velocities[i] = vel_tot[i]; // Aggiorna la velocità
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
int main() {
  std::srand(std::time(0)); // Inizializza il generatore di numeri casuali
  
  int num_boids;
  double d, d_s, s, a, c;

  std::cout << "Inserisci il numero di boids: ";
  std::cin >> num_boids;

  std::cout << "Inserisci la distanza massima per il centro di separazione (d): ";
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
    double x = rand() % 1600;
    double y = rand() % 900;
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

    // Disegna ogni boid come un cerchio bianco
    for (const auto& boid : mov.get_boids()) {
      sf::CircleShape shape(3.f);  // Crea un cerchio con raggio 3
      shape.setFillColor(sf::Color::White);  // Colore bianco per il boid
      shape.setPosition(boid.x, boid.y);  // Posiziona il boid
      window.draw(shape);
    }

    window.display();  // Visualizza la scena
    ++frame;  // Incrementa il numero del frame
  }

  return 0;
}
