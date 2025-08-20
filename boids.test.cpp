#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids_logic.hpp"
#include "doctest.h"
#include <cmath>

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

TEST_CASE("Test velocity")
{
  bd::Movement mov{};
  SUBCASE("Normal test")
  {
    bd::Velocity v{3.0, 4.0};
    CHECK(mov.get_speed(v) == doctest::Approx(5.0));
  }

  SUBCASE("Test get_speed with zero velocity")
  {
    bd::Velocity v{0.0, 0.0};
    CHECK(mov.get_speed(v) == doctest::Approx(0.0));
  }
}

TEST_CASE("Velocity limiting")
{
  bd::Movement mov{};
  SUBCASE("Velocity limiting with high speed")
  {
    bd::Velocity v{800.0, 800.0};
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
}

TEST_CASE("Test is_neighbor correct")
{
  bd::Movement mov({}, 10.0, 5.0, 0.1, 0.1, 0.1);
  bd::Position a{0.0, 0.0};
  bd::Position b{5.5, 8.0};
  CHECK(mov.is_neighbor(a, b));
  b = {11.0, 0.0};
  CHECK_FALSE(mov.is_neighbor(a, b));
}

TEST_CASE("Test rule1 active separation only if too close")
{
  bd::Movement mov({}, 10.0, 5.0, 0.1, 0.1, 0.1);
  bd::Position a{0.0, 0.0};
  bd::Position b{3.0, 4.0};

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
  bd::Movement mov(boids, 10.0, 5.0, 0.1, 0.1, 0.1);
  bd::Velocity self{1.0, 2.0};
  bd::Velocity mean{3.0, 4.0};
  auto res = mov.rule2(self, mean);
  CHECK(res[0] == doctest::Approx(0.2));
  CHECK(res[1] == doctest::Approx(0.2));
}

TEST_CASE("Test rule3 cohesion")
{
  bd::Movement mov({}, 10.0, 5.0, 0.1, 0.1, 0.1);
  bd::Position self{1.0, 2.0};
  bd::Position center{3.0, 4.0};
  auto res = mov.rule3(self, center);
  CHECK(res[0] == doctest::Approx(0.2));
  CHECK(res[1] == doctest::Approx(0.2));
}

TEST_CASE("Test Update")
{
  SUBCASE("Test update with one boid")
  {
    bd::Boid b{100., 100., 155., 0.};
    bd::Movement mov({b}, 100.0, 20.0, 1.5, 0.05, 0.3);
    mov.update(0, 60);
    auto pos = mov.get_positions()[0];
    CHECK(pos[0] > 100.0);
  }
  SUBCASE("Update multiple boids and check max speed")
  {
    std::vector<bd::Boid> boids = {bd::Boid(0., 0., 10., 0.),
                                   bd::Boid(5., 5., 0., 10.)};
    bd::Movement mov(boids, 10.0, 5.0, 0.1, 0.1, 0.1);

    mov.update(0, 1/60 );

    for (auto& pos : mov.get_positions()) {
      CHECK(pos[0] >= 0.0);
      CHECK(pos[1] >= 0.0);
    }

    for (auto& v : mov.get_velocities()) {
      CHECK(mov.get_speed(v) <= bd::Movement::max_speed + 1e-6);
    }
  }
}

TEST_CASE("add() function")
{
  bd::Velocity v1{1.0, 2.0};
  bd::Velocity v2{3.0, -1.0};
  bd::add_inplace(v1, v2);
  CHECK(v1[0] == doctest::Approx(4.0));
  CHECK(v1[1] == doctest::Approx(1.0));
}
