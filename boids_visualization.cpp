#ifndef BOIDS_VISUALIZATION_CPP
#define BOIDS_VISUALIZATION_CPP

#include "boids_visualization.hpp"

namespace bd {

BoidVisualization::BoidVisualization(Movement& sim)
    : simulation(sim),
      window(sf::VideoMode(screen_width, screen_height), "Boids Simulation"),
      stats_window(sf::VideoMode(400, 300), "Velocity Distribution"),
      mouse_repulsion_area(sf::TrianglesFan)
{
    const auto& boids = sim.get_boids();
    boid_shapes.reserve(boids.size());

    for (const auto& bo : boids) {
        sf::CircleShape shape(boid_radius, 3);
        shape.setPosition(static_cast<float>(bo.x), static_cast<float>(bo.y));
        shape.setFillColor(sf::Color::Green);
        shape.setOutlineColor(sf::Color::Black);
        shape.setOutlineThickness(1.f);
        boid_shapes.push_back(shape);
    }

    mouse_repulsion_area.append(sf::Vertex(mouse_position, sf::Color(255, 0, 0, 100)));
    for (int i = 0; i <= 360; i += 10) {
        float angle = static_cast<float>(i) * 3.14159265f / 180.f;
        mouse_repulsion_area.append(sf::Vertex(
            mouse_position + sf::Vector2f(std::cos(angle) * repulsion_radius, std::sin(angle) * repulsion_radius),
            sf::Color(255, 0, 0, 50)));
    }
}

void BoidVisualization::run() {
    sf::Clock clock;
    int frame_count = 0;

    while (window.isOpen()) {
        handle_events();
        update_simulation(frame_count++);
        render();
        sf::sleep(sf::milliseconds(16));
    }
}

void BoidVisualization::handle_events() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window.close();

        if (event.type == sf::Event::MouseMoved) {
            mouse_position = sf::Vector2f(static_cast<float>(event.mouseMove.x), static_cast<float>(event.mouseMove.y));
            mouse_repulsion_active = true;
        }
    }
}

void BoidVisualization::update_simulation(int frame) {
    if (mouse_repulsion_active) {
        auto& boids = simulation.get_boids();
        for (size_t i = 0; i < boids.size(); ++i) {
            double dx = boids[i].x - mouse_position.x;
            double dy = boids[i].y - mouse_position.y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < repulsion_radius * repulsion_radius) {
                double dist = std::sqrt(dist_sq);
                double factor = 5.0 * (1.0 - dist / repulsion_radius);
                simulation.add_velocity(i, factor * dx / dist, factor * dy / dist);
            }
        }
    }

    simulation.update(frame);
}

void BoidVisualization::render() {
    window.clear(sf::Color::White);
    render_boids();
    render_mouse_repulsion();
    window.display();

    stats_window.clear(sf::Color::White);
    render_stats();
    stats_window.display();
}

void BoidVisualization::render_boids() {
    const auto& boids = simulation.get_boids();
    const auto& velocities = simulation.get_velocities();

    for (size_t i = 0; i < boids.size(); ++i) {
        boid_shapes[i].setPosition(static_cast<float>(boids[i].x), static_cast<float>(boids[i].y));

        if (std::abs(velocities[i].v_x) > 0.001 || std::abs(velocities[i].v_y) > 0.001) {
            float angle = std::atan2(velocities[i].v_y, velocities[i].v_x) * 180.f / 3.14159265f;
            boid_shapes[i].setRotation(angle + 90.f);
        }

        window.draw(boid_shapes[i]);
    }
}

void BoidVisualization::render_mouse_repulsion() {
    if (mouse_repulsion_active) {
        mouse_repulsion_area[0].position = mouse_position;
        for (size_t i = 1; i < mouse_repulsion_area.getVertexCount(); ++i) {
            float angle = static_cast<float>(i - 1) * 10 * 3.14159265f / 180.f;
            mouse_repulsion_area[i].position = mouse_position + sf::Vector2f(std::cos(angle) * repulsion_radius, std::sin(angle) * repulsion_radius);
        }
        window.draw(mouse_repulsion_area);
    }
}

void BoidVisualization::render_stats() {
    const auto& velocities = simulation.get_velocities();

    double mean = 0.0;
    std::vector<double> speeds;
    speeds.reserve(velocities.size());

    for (const auto& vel : velocities) {
        double speed = std::sqrt(vel.v_x * vel.v_x + vel.v_y * vel.v_y);
        speeds.push_back(speed);
        mean += speed;
    }
    mean /= speeds.size();

    double std_dev = 0.0;
    for (double speed : speeds) {
        std_dev += (speed - mean) * (speed - mean);
    }
    std_dev = std::sqrt(std_dev / speeds.size());

    sf::VertexArray axes(sf::Lines, 4);
    axes[0] = sf::Vertex(sf::Vector2f(50, 250), sf::Color::Black);
    axes[1] = sf::Vertex(sf::Vector2f(350, 250), sf::Color::Black);
    axes[2] = sf::Vertex(sf::Vector2f(200, 50), sf::Color::Black);
    axes[3] = sf::Vertex(sf::Vector2f(200, 250), sf::Color::Black);
    stats_window.draw(axes);

    if (std_dev > 0.001) {
        sf::VertexArray curve(sf::LineStrip);
        for (int x = 0; x <= 400; x += 2) {
            double x_val = max_speed * (x - 200) / 200.0;
            double y_val = 200 * std::exp(-0.5 * std::pow((x_val - mean) / std_dev, 2));

            curve.append(sf::Vertex(sf::Vector2f(static_cast<float>(x), static_cast<float>(250 - y_val)), sf::Color::Blue));
        }
        stats_window.draw(curve);
    }

    // sf::Font font;
    // if (font.loadFromFile("arial.ttf")) {
    //     sf::Text mean_text("Mean: " + std::to_string(mean).substr(0, 4), font, 12);
    //     mean_text.setPosition(10, 10);
    //     mean_text.setFillColor(sf::Color::Black);
    //     stats_window.draw(mean_text);

    //     sf::Text std_text("Std Dev: " + std::to_string(std_dev).substr(0, 4), font, 12);
    //     std_text.setPosition(10, 30);
    //     std_text.setFillColor(sf::Color::Black);
    //     stats_window.draw(std_text);
    // }
}

} // namespace bd

#endif