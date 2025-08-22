#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids_logic.hpp"
#include "doctest.h"
#include <cmath>

TEST_CASE("add() function")
{
  bd::Velocity v1{1.0, 2.0};
  bd::Velocity v2{3.0, -1.0};
  bd::add_inplace(v1, v2);
  CHECK(v1[0] == doctest::Approx(4.0));
  CHECK(v1[1] == doctest::Approx(1.0));
}

TEST_CASE("Test push_back_ and erase_")
{
  bd::Movement mov({}, 100., 20., 1.5, 0.04, 0.3);

  SUBCASE("push_back_ adds a boid")
  {
    bd::Boid b{1.0, 2.0, 0.5, 0.5};
    mov.push_back_(b);

    auto positions = mov.get_positions();
    CHECK(positions.size() == 1);
    CHECK(positions[0][0] == doctest::Approx(1.0));
    CHECK(positions[0][1] == doctest::Approx(2.0));
  }

  SUBCASE("erase_ removes the last boid")
  {
    bd::Boid b1{0.0, 0.0, 1.0, 0.0};
    bd::Boid b2{5.0, 5.0, 0.0, 1.0};

    mov.push_back_(b1);
    mov.push_back_(b2);
    CHECK(mov.get_positions().size() == 2);

    mov.erase_();
    auto positions = mov.get_positions();
    CHECK(positions.size() == 1);
    CHECK(positions[0][0] == doctest::Approx(0.0));
    CHECK(positions[0][1] == doctest::Approx(0.0));
  }
}

TEST_CASE("Test velocity")
{
  bd::Movement mov{};
  SUBCASE("Normal test")
  {
    bd::Velocity v{3., 4.};
    CHECK(mov.get_speed(v) == doctest::Approx(5.0));
  }

  SUBCASE("Test get_speed with zero velocity")
  {
    bd::Velocity v{0.0, 0.0};
    CHECK(mov.get_speed(v) == doctest::Approx(0.0));
  }
  SUBCASE("Negative values")
  {
    bd::Velocity v{-3., -4.};
    CHECK(mov.get_speed(v) == doctest::Approx(5.0));
  }
}

TEST_CASE("Test diff_pos2")
{
  bd::Movement mov{};
  SUBCASE("Different positions")
  {
    bd::Position a{0.0, 0.0};
    bd::Position b{3.0, 4.0};
    CHECK(mov.diff_pos2(a, b) == doctest::Approx(25.0));
  }
  SUBCASE("Same position")
  {
    bd::Position p1{0.0, 0.0};
    bd::Position p2{0.0, 0.0};
    CHECK(mov.diff_pos2(p1, p2) == doctest::Approx(0.0));
  }
  SUBCASE("Test diff_pos2 with negative coordinates")
  {
    bd::Position a{-3.0, -4.0};
    bd::Position b{0.0, 0.0};
    CHECK(mov.diff_pos2(a, b) == doctest::Approx(25.0));
  }
}

TEST_CASE("Velocity limiting")
{
  bd::Movement mov{};
  SUBCASE("Velocity limiting with high speed")
  {
    bd::Velocity v{800., 800.};
    mov.limit_velocity(v);
    double expected_mag = mov.get_speed(v);
    CHECK(expected_mag <= bd::Movement::max_speed + 1e-6);
  }
  SUBCASE("Velocity limiting with low speed")
  {
    bd::Velocity v{1.0, 1.0};
    mov.limit_velocity(v);
    double expected_mag = mov.get_speed(v);
    CHECK(expected_mag <= bd::Movement::max_speed + 1e-6);
  }
  SUBCASE("Velocity limiting with high speed and negative value")
  {
    bd::Velocity v{-800., 800.};
    mov.limit_velocity(v);
    double expected_mag = mov.get_speed(v);
    CHECK(expected_mag <= bd::Movement::max_speed + 1e-6);
  }
}

TEST_CASE("Test is_neighbor correct")
{
  bd::Movement mov({}, 100., 20., 1.5, 0.04, 0.3);
  bd::Position a{0., 0.};
  bd::Position b{5.5, 8.};
  CHECK(mov.is_neighbor(a, b));
  b = {110.0, 0.0};
  CHECK_FALSE(mov.is_neighbor(a, b));
}

TEST_CASE("Test rule1 active separation only if too close")
{
  bd::Movement mov({}, 100., 20., 1.5, 0.04, 0.3);
  bd::Position a{0.0, 0.0};
  bd::Position b{30., 40.};

  auto res = mov.rule1(a, b);
  CHECK(res[0] == doctest::Approx(0.0));
  CHECK(res[1] == doctest::Approx(0.0));

  b   = {2.0, 2.0};
  res = mov.rule1(a, b);
  CHECK(res[0] < 0.0);
  CHECK(res[1] < 0.0);
}

TEST_CASE("Test rule2 alignment")
{
  std::vector<bd::Boid> boids = {bd::Boid(0.0, 0.0, 1.0, 2.0),
                                 bd::Boid(1.0, 1.0, 3.0, 4.0)};
  bd::Movement mov(boids, 100., 20.0, 1.5, 0.04, 0.3);
  bd::Velocity self{1.0, 2.0};
  bd::Velocity mean{3.0, 4.0};
  auto res = mov.rule2(self, mean);
  CHECK(res[0] == doctest::Approx(0.08));
  CHECK(res[1] == doctest::Approx(0.08));
}

TEST_CASE("Test rule3 cohesion")
{
  bd::Movement mov({}, 100., 20., 1.5, 0.04, 0.3);
  bd::Position self{1.0, 2.0};
  bd::Position center{3.0, 4.0};
  auto res = mov.rule3(self, center);
  CHECK(res[0] == doctest::Approx(0.6));
  CHECK(res[1] == doctest::Approx(0.6));
}

TEST_CASE("Test apply_neighbor_rules")
{
  SUBCASE("Test apply_neighbor_rules with no neighbors")
  {
    std::vector<bd::Boid> boids = {bd::Boid(0.0, 0.0, 1.0, 1.0)};
    bd::Movement mov(boids, 100., 20., 1.5, 0.04, 0.3);

    bd::Velocity v{1.0, 1.0};
    mov.apply_neighbor_rules(0, v);

    // Nessun cambiamento perché non ci sono vicini
    CHECK(v[0] == doctest::Approx(1.0));
    CHECK(v[1] == doctest::Approx(1.0));
  }
  SUBCASE("Test apply_neighbor_rules with one neighbor")
  {
    std::vector<bd::Boid> boids = {bd::Boid(0.0, 0.0, 1.0, 1.0),
                                   bd::Boid(2.0, 0.0, 1.0, 0.0)};
    bd::Movement mov(boids, 100., 20., 1.5, 0.04, 0.3);

    bd::Velocity v = boids[0].vel;
    mov.apply_neighbor_rules(0, v);

    // Deve aver applicato almeno rule2 o rule3
    CHECK((v[0] != 1.0 || v[1] != 1.0));
  }
}

TEST_CASE("Test apply_mouse_force")
{
  bd::Boid b{770., 450., 0., 0.};
  bd::Movement mov({b}, 100., 20., 1.5, 0.04, 0.3);

  SUBCASE("Test apply_mouse_force attractive")
  {
    mov.set_mouse_force({800, 450}, false, false);
    bd::Velocity v = b.vel;
    mov.apply_mouse_force(b, v);

    CHECK(v[0] > 0.0); // dovrebbe spingere verso destra
  }
  SUBCASE("Test apply_mouse_force repulsive")
  {
    mov.set_mouse_force({800, 450}, true, false);
    bd::Velocity v = b.vel;
    mov.apply_mouse_force(b, v);

    CHECK(v[0] < 0.0); // dovrebbe spingere verso sinistra
  }
  SUBCASE("Test apply_mouse_force turned off")
  {
    mov.set_mouse_force({800, 450}, true, true);
    bd::Velocity v = b.vel;
    mov.apply_mouse_force(b, v);

    CHECK(v[0] == doctest::Approx(0.)); // dovrebbe spingere verso sinistra
  }
}

TEST_CASE("Test update_pos_vel basic movement")
{
  std::vector<bd::Boid> boids = {bd::Boid(0.0, 0.0, 200.0, 200.0)};
  bd::Movement mov(boids, 100., 20., 1.5, 0.04, 0.3);

  std::vector<bd::Velocity> vel_tot = {{300.0, 250.0}};
  mov.update_pos_vel(vel_tot, 0.01);

  auto pos = mov.get_positions()[0];
  CHECK(pos[0] == doctest::Approx(3.0));
  CHECK(pos[1] == doctest::Approx(2.5));

  auto vel = mov.get_velocities()[0];
  CHECK(vel[0] == doctest::Approx(300.0));
  CHECK(vel[1] == doctest::Approx(250.0));
}

TEST_CASE("Test Update")
{
  SUBCASE("Test update with one boid")
  {
    bd::Boid b{100., 100., 155., 0.};
    bd::Movement mov({b}, 100., 20., 1.5, 0.04, 0.3);
    mov.update(0, 0.017);
    auto pos = mov.get_positions()[0];
    CHECK(pos[0] > 100.0);
  }
  SUBCASE("Update multiple boids and check max speed")
  {
    std::vector<bd::Boid> boids = {bd::Boid(0., 0., 900., 0.),
                                   bd::Boid(5., 5., 0., 900.)};
    bd::Movement mov(boids, 100., 20., 1.5, 0.04, 0.3);

    mov.update(0, 0.017);

    for (auto& pos : mov.get_positions()) {
      CHECK(pos[0] >= 0.);
      CHECK(pos[1] >= 0.);
    }

    for (auto& v : mov.get_velocities()) {
      CHECK(mov.get_speed(v) <= bd::Movement::max_speed + 1e-6);
    }
  }
}