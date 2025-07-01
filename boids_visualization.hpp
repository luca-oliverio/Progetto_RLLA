#ifndef BOIDS_VISUALIZATION_HPP
#define BOIDS_VISUALIZATION_HPP

#include <SFML/Graphics.hpp>
#include "boids_logic.hpp"

namespace bd {

class BoidVisualization {
private:
    sf::RenderWindow window;
    sf::RenderWindow stats_window;
    std::vector<sf::CircleShape> boid_shapes;
    sf::VertexArray mouse_repulsion_area;
    bool mouse_repulsion_active = false;
    sf::Vector2f mouse_position;

    Movement& simulation;

    static constexpr float boid_radius = 5.f;
    static constexpr float repulsion_radius = 50.f;
    static constexpr float max_speed = 10.0f;
    static constexpr unsigned int screen_width = 1600;
    static constexpr unsigned int screen_height = 900;

public:
    BoidVisualization(Movement& sim);
    void run();

private:
    void handle_events();
    void update_simulation(int frame);
    void render();
    void render_boids();
    void render_mouse_repulsion();
    void render_stats();
};

} // namespace bd

#endif // BOIDS_VISUALIZATION_HPP
