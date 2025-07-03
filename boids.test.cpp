#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids_logic.hpp"
#include "doctest.h"
#include <cmath>

TEST_CASE("Test diff_pos")
{
    bd::Boid a{0.0, 0.0};
    bd::Boid b{3.0, 4.0};
    CHECK(bd::Movement::diff_pos(a, b) == doctest::Approx(25.0));
}

TEST_CASE("Test velocity calcolo corretto")
{
    bd::Boid_vel v{3.0, 4.0};
    CHECK(bd::Movement::velocity(v) == doctest::Approx(5.0));
}

TEST_CASE("Velocity limiting") {
    bd::Boid_vel v{6.0, 8.0}; // modulo = 10
    bd::Movement m({}, 0, 0, 0, 0, 0);
    m.limit_velocity(v);
    double expected_mag = m.velocity(v);
    CHECK(expected_mag <= bd::max_speed + 1e-6);
}

TEST_CASE("Test is_neighbor corretto")
{
    bd::Movement mov({}, 10.0, 5.0, 0.1, 0.1, 0.1);
    bd::Boid a{0.0, 0.0};
    bd::Boid b{5.5, 8.0}; // distanza minore di 10
    CHECK(mov.is_neighbor(a, b));
    b = {11.0, 0.0};
    CHECK_FALSE(mov.is_neighbor(a, b));
}

TEST_CASE("Test rule1 separazione attiva solo se troppo vicini")
{
    bd::Movement mov({}, 10.0, 5.0, 0.1, 0.1, 0.1);
    bd::Boid a{0.0, 0.0};
    bd::Boid b{3.0, 4.0}; // distanza 5 -> al limite

    auto res = mov.rule1(a, b);
    CHECK(res.v_x == doctest::Approx(0.0));
    CHECK(res.v_y == doctest::Approx(0.0));

    b = {2.0, 2.0}; // distanza < sqrt(5)
    res = mov.rule1(a, b);
    CHECK(res.v_x < 0.0); // Deve spingere allontanandosi
    CHECK(res.v_y < 0.0);
}

TEST_CASE("Test rule2 allineamento")
{
std::vector<bd::Boid_Complete> boids = {
  bd::Boid_Complete({0.0, 0.0}, {1.0, 2.0}),
  bd::Boid_Complete({1.0, 1.0}, {3.0, 4.0})
};
bd::Movement mov(boids, 10.0, 5.0, 0.1, 0.1, 0.1);
bd::Boid_vel self{1.0, 2.0};
bd::Boid_vel mean{3.0, 4.0};
auto res = mov.rule2(self, mean);
CHECK(res.v_x == doctest::Approx(0.2));
CHECK(res.v_y == doctest::Approx(0.2));
}

TEST_CASE("Test rule3 coesione")
{
    bd::Movement mov({}, 10.0, 5.0, 0.1, 0.1, 0.1);
    bd::Boid self{1.0, 2.0};
    bd::Boid center{3.0, 4.0};
    auto res = mov.rule3(self, center);
    CHECK(res.v_x == doctest::Approx(0.2));
    CHECK(res.v_y == doctest::Approx(0.2));
}

TEST_CASE("Test cm centro di massa")
{
    std::vector<bd::Boid> boids = { {0, 0}, {2, 2} };
    auto center = bd::cm(boids);
    CHECK(center.x == doctest::Approx(1.0));
    CHECK(center.y == doctest::Approx(1.0));
}

TEST_CASE("Test update con un solo boid")
{
    bd::Boid_Complete b{{100, 100}, {5, 0}};
    bd::Movement mov({b}, 10.0, 5.0, 0.1, 0.1, 0.1);

    mov.update(0);
    auto pos = mov.get_boids()[0];
    CHECK(pos.x > 100.0);
}

TEST_CASE("Boid_vel operator+=") {
    bd::Boid_vel v1(1.0, 2.0);
    bd::Boid_vel v2(3.0, -1.0);
    v1 += v2;
    CHECK(v1.v_x == doctest::Approx(4.0));
    CHECK(v1.v_y == doctest::Approx(1.0));
}

