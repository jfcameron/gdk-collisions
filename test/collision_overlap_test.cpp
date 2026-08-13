// © Joseph Cameron - All Rights Reserved

#include <jfc/catch.hpp>

#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_narrow_phase.h>
#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_heightfield_data.h>
#include <gdk/collisions/impl_mesh_data.h>
#include <gdk/collisions/impl_shape.h>
#include <gdk/collisions/overlap.h>

#include <utility>
#include <vector>

using namespace gdk::collisions;

namespace {
    constexpr floating_point_type TIME_MARGIN = 1e-5f;
    constexpr floating_point_type NORMAL_MARGIN = 1e-4f;

    const impl_collision_policy policy = {};

    struct body final {
        shape_type shape;
        shape_kinematics kinematics;
    };

    body make_sphere(const vector3_type &aPosition,
        const vector3_type &aVelocity = vector3_type::zero) {
        return body{sphere_shape{0.5f}, {aPosition, aVelocity}};
    }

    body make_capsule(const vector3_type &aPosition,
        const vector3_type &aVelocity = vector3_type::zero,
        const floating_point_type aHalfHeight = 0.5f,
        const quaternion_type &aOrientation = quaternion_type::identity) {
        return body{capsule_shape{0.5f, aHalfHeight}, {aPosition, aVelocity, aOrientation}};
    }

    [[nodiscard]] quaternion_type quarter_turn_about_z() {
        return quaternion_type(vector3_type{0, 0, 3.14159265f / 2.0f});
    }

    body make_triangle(const vector3_type &aPosition,
        const vector3_type &aVelocity = vector3_type::zero) {
        return body{triangle_shape{{-1, 0, -1}, {1, 0, -1}, {0, 0, 1}}, {aPosition, aVelocity}};
    }

    [[nodiscard]] std::shared_ptr<const impl_mesh_data> unit_quad() {
        static const auto quad = std::dynamic_pointer_cast<const impl_mesh_data>(
            impl_mesh_data::make({{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, {0, 1, 2, 0, 2, 3}));
        return quad;
    }

    body make_mesh(const vector3_type &aPosition,
        const vector3_type &aVelocity = vector3_type::zero,
        const quaternion_type &aOrientation = quaternion_type::identity) {
        return body{mesh_shape{unit_quad()}, {aPosition, aVelocity, aOrientation}};
    }

    template <typename sampler_type>
    [[nodiscard]] heightfield_data_ptr_type make_heightfield_data(const std::size_t aColumns,
        const std::size_t aRows, const sampler_type &aSampler) {
        std::vector<floating_point_type> heights(aColumns * aRows);

        for (std::size_t row = 0; row < aRows; ++row)
            for (std::size_t column = 0; column < aColumns; ++column)
                heights[row * aColumns + column] = aSampler(column, row);

        return impl_heightfield_data::make(aColumns, aRows, std::move(heights));
    }

    template <typename sampler_type>
    body make_heightfield(const std::size_t aColumns, const std::size_t aRows, const sampler_type &aSampler) {
        return body{heightfield_shape{std::dynamic_pointer_cast<const impl_heightfield_data>(
            make_heightfield_data(aColumns, aRows, aSampler))},
            {vector3_type::zero, vector3_type::zero}};
    }

    body make_obb(const vector3_type &aPosition,
        const vector3_type &aVelocity = vector3_type::zero,
        const quaternion_type &aOrientation = quaternion_type::identity,
        const vector3_type &aHalfExtents = {0.5f, 0.5f, 0.5f}) {
        return body{obb_shape{aHalfExtents}, {aPosition, aVelocity, aOrientation}};
    }

    body make_box(const vector3_type &aPosition,
        const vector3_type &aVelocity = vector3_type::zero) {
        return body{box_shape{{0.5f, 0.5f, 0.5f}}, {aPosition, aVelocity}};
    }

    [[nodiscard]] std::optional<overlap> overlap_of(const body &aSubject, const body &aOther,
        const delta_time_type aDeltaTime) {
        return narrow_phase_overlap(aSubject.shape, aSubject.kinematics,
            aOther.shape, aOther.kinematics, aDeltaTime, policy);
    }

    void require_vector_near(const vector3_type &aActual, const vector3_type &aExpected,
        const floating_point_type aMargin) {
        REQUIRE(aActual.x == Approx(aExpected.x).margin(aMargin));
        REQUIRE(aActual.y == Approx(aExpected.y).margin(aMargin));
        REQUIRE(aActual.z == Approx(aExpected.z).margin(aMargin));
    }
}

TEST_CASE("gdk::overlap sphere vs sphere", "[gdk::collision]")
{
    SECTION("head on approach reports the time of impact")
    {
        const auto moving = make_sphere({-2, 0, 0}, {1, 0, 0});
        const auto stationary = make_sphere({0, 0, 0});

        const auto result = overlap_of(moving, stationary, 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(1.0f).margin(TIME_MARGIN));
        REQUIRE(result->exit_time == Approx(2.0f).margin(TIME_MARGIN));
    }

    SECTION("stationary and already overlapping reports immediate contact")
    {
        const auto a = make_sphere({0, 0, 0});
        const auto b = make_sphere({0.5f, 0, 0});

        const auto result = overlap_of(a, b, 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(0.0f).margin(TIME_MARGIN));
        REQUIRE(result->exit_time == Approx(2.0f).margin(TIME_MARGIN));
    }

    SECTION("stationary and separated reports no overlap")
    {
        const auto a = make_sphere({0, 0, 0});
        const auto b = make_sphere({3, 0, 0});

        REQUIRE_FALSE(overlap_of(a, b, 2.0f).has_value());
    }

    SECTION("receding reports no overlap")
    {
        const auto moving = make_sphere({-2, 0, 0}, {-1, 0, 0});
        const auto stationary = make_sphere({0, 0, 0});

        REQUIRE_FALSE(overlap_of(moving, stationary, 2.0f).has_value());
    }

    SECTION("passing alongside reports no overlap")
    {
        const auto moving = make_sphere({-2, 2, 0}, {1, 0, 0});
        const auto stationary = make_sphere({0, 0, 0});

        REQUIRE_FALSE(overlap_of(moving, stationary, 2.0f).has_value());
    }
}

TEST_CASE("gdk::overlap box vs box", "[gdk::collision]")
{
    SECTION("head on approach reports the time of impact and the contact axis")
    {
        const auto moving = make_box({-2, 0, 0}, {1, 0, 0});
        const auto stationary = make_box({0, 0, 0});

        const auto result = overlap_of(moving, stationary, 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(1.0f).margin(TIME_MARGIN));
        require_vector_near(result->contact_normal, {-1, 0, 0}, NORMAL_MARGIN);
    }

    SECTION("stationary and already overlapping reports immediate contact on the shallowest axis")
    {
        const auto a = make_box({0, 0, 0});
        const auto b = make_box({0.5f, 0, 0});

        const auto result = overlap_of(a, b, 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(0.0f).margin(TIME_MARGIN));
        require_vector_near(result->contact_normal, {-1, 0, 0}, NORMAL_MARGIN);
        REQUIRE(result->penetration == Approx(0.5f).margin(NORMAL_MARGIN));
    }

    SECTION("stationary and separated reports no overlap")
    {
        const auto a = make_box({0, 0, 0});
        const auto b = make_box({3, 0, 0});

        REQUIRE_FALSE(overlap_of(a, b, 2.0f).has_value());
    }
}

TEST_CASE("gdk::overlap sphere vs box", "[gdk::collision]")
{
    SECTION("head on approach against a face")
    {
        const auto sphere = make_sphere({-2, 0, 0}, {1, 0, 0});
        const auto box = make_box({0, 0, 0});

        const auto result = overlap_of(sphere, box, 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(1.0f).margin(TIME_MARGIN));
        require_vector_near(result->contact_normal, {-1, 0, 0}, NORMAL_MARGIN);
        require_vector_near(result->contact_point, {-0.5f, 0, 0}, NORMAL_MARGIN);
    }

    SECTION("diagonal approach against an edge")
    {
        const auto sphere = make_sphere({-2, -2, 0}, {1, 1, 0});
        const auto box = make_box({0, 0, 0});

        const auto result = overlap_of(sphere, box, 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(1.14644661f).margin(TIME_MARGIN));
        require_vector_near(result->contact_normal, {-0.70710678f, -0.70710678f, 0}, NORMAL_MARGIN);
        require_vector_near(result->contact_point, {-0.5f, -0.5f, 0}, NORMAL_MARGIN);
    }

    SECTION("stationary and separated reports no overlap")
    {
        const auto sphere = make_sphere({3, 0, 0});
        const auto box = make_box({0, 0, 0});

        REQUIRE_FALSE(overlap_of(sphere, box, 2.0f).has_value());
    }

    SECTION("stationary and shallowly overlapping reports contact")
    {
        const auto sphere = make_sphere({0.75f, 0, 0});
        const auto box = make_box({0, 0, 0});

        const auto result = overlap_of(sphere, box, 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(0.0f).margin(TIME_MARGIN));
        REQUIRE(result->penetration == Approx(0.25f).margin(NORMAL_MARGIN));
        require_vector_near(result->contact_normal, {1, 0, 0}, NORMAL_MARGIN);
    }
}

TEST_CASE("gdk::overlap sphere deeply inside box is detected", "[gdk::collision]")
{
    const auto sphere = make_sphere({0, 0, 0});
    const auto box = make_box({0, 0, 0});

    REQUIRE(overlap_of(sphere, box, 2.0f).has_value());
}

TEST_CASE("gdk::overlap exit_time is clamped to the timestep", "[gdk::collision]")
{
    const auto moving = make_box({-2, 0, 0}, {1, 0, 0});
    const auto stationary = make_box({0, 0, 0});

    const auto result = overlap_of(moving, stationary, 2.0f);

    REQUIRE(result.has_value());
    REQUIRE(result->exit_time <= Approx(2.0f).margin(TIME_MARGIN));
}

TEST_CASE("gdk::overlap capsule is a superset of sphere", "[gdk::collision]")
{
    SECTION("degenerate capsule reproduces the sphere head on result")
    {
        const auto viaCapsule = overlap_of(make_capsule({-2, 0, 0}, {1, 0, 0}, 0.0f),
            make_capsule({0, 0, 0}, vector3_type::zero, 0.0f), 2.0f);

        REQUIRE(viaCapsule.has_value());
        REQUIRE(viaCapsule->entry_time == Approx(1.0f).margin(1e-3f));
        require_vector_near(viaCapsule->contact_normal, {-1, 0, 0}, 1e-3f);
    }

    SECTION("degenerate capsule reproduces the sphere miss")
    {
        REQUIRE_FALSE(overlap_of(make_capsule({-2, 2, 0}, {1, 0, 0}, 0.0f),
            make_capsule({0, 0, 0}, vector3_type::zero, 0.0f), 2.0f).has_value());
    }
}

TEST_CASE("gdk::overlap capsule vs capsule", "[gdk::collision]")
{
    SECTION("side by side approach contacts at the radii sum")
    {
        const auto result = overlap_of(make_capsule({-2, 0, 0}, {1, 0, 0}),
            make_capsule({0, 0, 0}), 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(1.0f).margin(1e-2f));
        require_vector_near(result->contact_normal, {-1, 0, 0}, 1e-2f);
    }

    SECTION("passing well clear on Y still misses despite overlapping X")
    {
        REQUIRE_FALSE(overlap_of(make_capsule({-2, 4, 0}, {1, 0, 0}),
            make_capsule({0, 0, 0}), 2.0f).has_value());
    }

    SECTION("stacked vertically, the caps are what touch")
    {
        const auto result = overlap_of(make_capsule({0, -3, 0}, {0, 1, 0}),
            make_capsule({0, 0, 0}), 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(1.0f).margin(1e-2f));
        require_vector_near(result->contact_normal, {0, -1, 0}, 1e-2f);
    }

    SECTION("already touching reports immediate contact")
    {
        const auto result = overlap_of(make_capsule({0, -2, 0}, {0, 1, 0}),
            make_capsule({0, 0, 0}), 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(0.0f).margin(1e-3f));
    }
}

TEST_CASE("gdk::overlap capsule vs box", "[gdk::collision]")
{
    SECTION("head on approach against a face")
    {
        const auto result = overlap_of(make_capsule({-2, 0, 0}, {1, 0, 0}), make_box({0, 0, 0}), 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(1.0f).margin(1e-2f));
        require_vector_near(result->contact_normal, {-1, 0, 0}, 1e-2f);
    }

    SECTION("box as the subject reports the opposite normal")
    {
        const auto result = overlap_of(make_box({-2, 0, 0}, {1, 0, 0}), make_capsule({0, 0, 0}), 2.0f);

        REQUIRE(result.has_value());
        require_vector_near(result->contact_normal, {-1, 0, 0}, 1e-2f);
    }

    SECTION("clear of the box reports no overlap")
    {
        REQUIRE_FALSE(overlap_of(make_capsule({-4, 0, 0}), make_box({0, 0, 0}), 2.0f).has_value());
    }
}

TEST_CASE("gdk::overlap a capsule's segment follows its orientation", "[gdk::collision]")
{
    SECTION("upright capsule is narrow on X, so a distant approach along X misses")
    {
        REQUIRE_FALSE(overlap_of(make_sphere({1.2f, 0, 0}), make_capsule({0, 0, 0}), 0.0f).has_value());
    }

    SECTION("the same capsule laid on its side reaches far enough to touch")
    {
        const auto result = overlap_of(make_sphere({1.2f, 0, 0}),
            make_capsule({0, 0, 0}, vector3_type::zero, 0.5f, quarter_turn_about_z()), 0.0f);

        REQUIRE(result.has_value());
    }

    SECTION("and is correspondingly short on Y once rotated")
    {
        REQUIRE(overlap_of(make_sphere({0, 1.2f, 0}), make_capsule({0, 0, 0}), 0.0f).has_value());
        REQUIRE_FALSE(overlap_of(make_sphere({0, 1.2f, 0}),
            make_capsule({0, 0, 0}, vector3_type::zero, 0.5f, quarter_turn_about_z()),
            0.0f).has_value());
    }
}

TEST_CASE("gdk::overlap an oriented box at identity matches an axis aligned one", "[gdk::collision]")
{
    SECTION("head on approach reports the same time of impact")
    {
        const auto viaObb = overlap_of(make_obb({-2, 0, 0}, {1, 0, 0}), make_obb({0, 0, 0}), 2.0f);

        REQUIRE(viaObb.has_value());
        REQUIRE(viaObb->entry_time == Approx(1.0f).margin(1e-2f));
        require_vector_near(viaObb->contact_normal, {-1, 0, 0}, 1e-2f);
    }

    SECTION("separated pair reports no overlap")
    {
        REQUIRE_FALSE(overlap_of(make_obb({3, 0, 0}), make_obb({0, 0, 0}), 2.0f).has_value());
    }

    SECTION("mixed axis aligned and oriented agree at identity")
    {
        const auto mixed = overlap_of(make_box({-2, 0, 0}, {1, 0, 0}), make_obb({0, 0, 0}), 2.0f);

        REQUIRE(mixed.has_value());
        REQUIRE(mixed->entry_time == Approx(1.0f).margin(1e-2f));
    }
}

TEST_CASE("gdk::overlap a rotated box occupies a different volume", "[gdk::collision]")
{
    const auto fortyFiveAboutZ = quaternion_type(
        vector3_type{0, 0, 3.14159265f / 4.0f});

    SECTION("unrotated box does not reach the probe")
    {
        REQUIRE_FALSE(overlap_of(make_sphere({1.15f, 0, 0}), make_obb({0, 0, 0}), 0.0f).has_value());
    }

    SECTION("rotated box does")
    {
        REQUIRE(overlap_of(make_sphere({1.15f, 0, 0}),
            make_obb({0, 0, 0}, vector3_type::zero, fortyFiveAboutZ), 0.0f).has_value());
    }

    SECTION("two rotated boxes meet corner to corner")
    {
        const auto result = overlap_of(make_obb({-2, 0, 0}, {1, 0, 0}, fortyFiveAboutZ),
            make_obb({0, 0, 0}, vector3_type::zero, fortyFiveAboutZ), 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(0.586f).margin(2e-2f));
        require_vector_near(result->contact_normal, {-0.7071f, -0.7071f, 0}, 1e-2f);
    }

    SECTION("the reported translation actually separates the boxes")
    {
        const auto overlapping = make_obb({-1.2f, 0, 0}, vector3_type::zero, fortyFiveAboutZ);
        const auto other = make_obb({0, 0, 0}, vector3_type::zero, fortyFiveAboutZ);

        const auto before = overlap_of(overlapping, other, 0.0f);
        REQUIRE(before.has_value());
        REQUIRE(before->penetration > 0.0f);

        const auto separated = make_obb(
            overlapping.kinematics.position + before->contact_normal * before->penetration,
            vector3_type::zero, fortyFiveAboutZ);

        const auto after = overlap_of(separated, other, 0.0f);
        REQUIRE_FALSE(after.has_value());
    }

    SECTION("rotated boxes closer than their diagonal reach start overlapped")
    {
        const auto result = overlap_of(make_obb({-1.2f, 0, 0}, {1, 0, 0}, fortyFiveAboutZ),
            make_obb({0, 0, 0}, vector3_type::zero, fortyFiveAboutZ), 2.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(0.0f).margin(1e-3f));
        REQUIRE(result->penetration == Approx(0.1515f).margin(2e-3f));
        require_vector_near(result->contact_normal, {-0.7071f, -0.7071f, 0}, 1e-2f);
    }
}

TEST_CASE("gdk::overlap every shape pair reports symmetrically", "[gdk::collision]")
{
    const std::vector<std::pair<const char *, shape_type>> shapes = {
        {"sphere",  sphere_shape{0.5f}},
        {"box",     box_shape{{0.5f, 0.5f, 0.5f}}},
        {"capsule", capsule_shape{0.5f, 0.5f}},
        {"obb",     obb_shape{{0.5f, 0.5f, 0.5f}}},
        {"plane",   plane_shape{}},
        {"triangle", triangle_shape{{-1, 0, -1}, {1, 0, -1}, {0, 0, 1}}},
        {"mesh",    mesh_shape{unit_quad()}},
        {"heightfield", heightfield_shape{std::dynamic_pointer_cast<const impl_heightfield_data>(
            make_heightfield_data(3, 3, [](std::size_t, std::size_t) { return 0.0f; }))}},
    };

    for (const auto &first : shapes) {
        for (const auto &second : shapes) {
            if (std::holds_alternative<plane_shape>(first.second) &&
                std::holds_alternative<plane_shape>(second.second)) continue;

            if (std::holds_alternative<triangle_shape>(first.second) &&
                std::holds_alternative<triangle_shape>(second.second)) continue;

            INFO(first.first << " vs " << second.first);

            const body a{first.second, {{-0.6f, 0, 0}, vector3_type::zero}};
            const body b{second.second, {{0, 0, 0}, vector3_type::zero}};

            const auto forward = overlap_of(a, b, 0.0f);
            const auto reverse = overlap_of(b, a, 0.0f);

            REQUIRE(forward.has_value() == reverse.has_value());
            if (!forward.has_value()) continue;

            REQUIRE(forward->penetration == Approx(reverse->penetration).margin(1e-3f));

            const auto cancelled = forward->contact_normal + reverse->contact_normal;
            REQUIRE(cancelled.length() == Approx(0.0f).margin(1e-3f));
        }
    }
}

TEST_CASE("gdk::overlap a buried body is pushed out, not further in", "[gdk::collision]")
{
    const auto buried = make_sphere({0.1f, 0, 0});
    const auto box = make_box({0, 0, 0});

    const auto result = overlap_of(buried, box, 1.0f);
    REQUIRE(result.has_value());
    REQUIRE(result->penetration > 0.0f);

    const auto freed = make_sphere(buried.kinematics.position
        + result->contact_normal * result->penetration);
    const auto after = overlap_of(freed, box, 1.0f);

    REQUIRE_FALSE(after.has_value());
}

TEST_CASE("gdk::overlap a moving body inside geometry is still detected", "[gdk::collision]")
{
    for (const auto velocity : {vector3_type{0, 0, 0}, vector3_type{1, 0, 0},
        vector3_type{0.001f, 0, 0}}) {
        INFO("velocity " << velocity.x);
        const auto result = overlap_of(make_sphere({0, 0, 0}, velocity), make_box({0, 0, 0}), 1.0f);

        REQUIRE(result.has_value());
        REQUIRE(result->penetration > 0.0f);
    }
}

TEST_CASE("gdk::overlap every shape meets a triangle's face", "[gdk::collision]")
{
    SECTION("sphere") {
        const auto result = overlap_of(make_sphere({0, 5, 0}, {0, -1, 0}), make_triangle({0, 0, 0}), 10.0f);
        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(4.5f).margin(1e-2f));   
        require_vector_near(result->contact_normal, {0, 1, 0}, 1e-2f);
    }

    SECTION("capsule") {
        const auto result = overlap_of(make_capsule({0, 5, 0}, {0, -1, 0}), make_triangle({0, 0, 0}), 10.0f);
        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(4.0f).margin(1e-2f));   
        require_vector_near(result->contact_normal, {0, 1, 0}, 1e-2f);
    }

    SECTION("axis aligned box") {
        const auto result = overlap_of(make_box({0, 5, 0}, {0, -1, 0}), make_triangle({0, 0, 0}), 10.0f);
        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(4.5f).margin(1e-2f));
        require_vector_near(result->contact_normal, {0, 1, 0}, 1e-2f);
    }

    SECTION("oriented box") {
        const auto result = overlap_of(make_obb({0, 5, 0}, {0, -1, 0}), make_triangle({0, 0, 0}), 10.0f);
        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(4.5f).margin(1e-2f));
        require_vector_near(result->contact_normal, {0, 1, 0}, 1e-2f);
    }

    SECTION("a body passing beside the triangle misses it") {
        REQUIRE_FALSE(overlap_of(make_sphere({5, 5, 0}, {0, -1, 0}),
            make_triangle({0, 0, 0}), 10.0f).has_value());
    }
}

TEST_CASE("gdk::overlap triangle against triangle is refused, not silently missed", "[gdk::collision]")
{
    REQUIRE_THROWS_AS(overlap_of(make_triangle({0, 0, 0}), make_triangle({0, 0, 0}), 1.0f),
        exception);
}

TEST_CASE("gdk::overlap every shape meets a mesh's surface", "[gdk::collision]")
{
    const auto mesh = make_mesh({0, 0, 0});

    const auto check = [&](const char *aLabel, const body &aBody,
        const floating_point_type aExpectedTime) {
        INFO(aLabel);
        const auto result = overlap_of(aBody, mesh, 1.0f);
        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(aExpectedTime).margin(TIME_MARGIN));
        require_vector_near(result->contact_normal, {0, 1, 0}, NORMAL_MARGIN);
    };

    check("sphere", make_sphere({0, 5, 0}, {0, -10, 0}), 0.45f);
    check("box", body{box_shape{{0.5f, 0.5f, 0.5f}}, {{0, 5, 0}, {0, -10, 0}}}, 0.45f);
    check("obb", body{obb_shape{{0.5f, 0.5f, 0.5f}}, {{0, 5, 0}, {0, -10, 0}}}, 0.45f);
    check("capsule", make_capsule({0, 5, 0}, {0, -10, 0}, 1.0f), 0.35f);
}

TEST_CASE("gdk::overlap a mesh is bounded by its triangles", "[gdk::collision]")
{
    REQUIRE_FALSE(overlap_of(make_sphere({20, 5, 0}, {0, -10, 0}), make_mesh({0, 0, 0}), 1.0f).has_value());
    REQUIRE_FALSE(overlap_of(make_sphere({2, 5, 0}, {0, -10, 0}), make_mesh({0, 0, 0}), 1.0f).has_value());
}

TEST_CASE("gdk::overlap a mesh carries its collider's transform", "[gdk::collision]")
{
    const auto result = overlap_of(make_sphere({-5, 0, 0}, {10, 0, 0}),
        make_mesh({0, 0, 0}, vector3_type::zero, quarter_turn_about_z()), 1.0f);

    REQUIRE(result.has_value());
    REQUIRE(result->entry_time == Approx(0.45f).margin(TIME_MARGIN));
    require_vector_near(result->contact_normal, {-1, 0, 0}, NORMAL_MARGIN);
}

TEST_CASE("gdk::overlap a moving mesh finds a stationary body", "[gdk::collision]")
{
    const auto falling = overlap_of(make_sphere({0, 5, 0}, {0, -10, 0}), make_mesh({0, 0, 0}), 1.0f);
    const auto rising = overlap_of(make_sphere({0, 5, 0}), make_mesh({0, 0, 0}, {0, 10, 0}), 1.0f);

    REQUIRE(falling.has_value());
    REQUIRE(rising.has_value());
    REQUIRE(rising->entry_time == Approx(falling->entry_time).margin(TIME_MARGIN));
    REQUIRE(rising->entry_time == Approx(0.45f).margin(TIME_MARGIN));
}

TEST_CASE("gdk::overlap mesh against mesh reports nothing rather than throwing", "[gdk::collision]")
{
    REQUIRE_NOTHROW(overlap_of(make_mesh({0, 0, 0}), make_mesh({0, 0, 0}), 1.0f));
    REQUIRE_FALSE(overlap_of(make_mesh({0, 0, 0}), make_mesh({0, 0, 0}), 1.0f).has_value());
    REQUIRE_FALSE(overlap_of(make_mesh({0, 0, 0}, {0, -10, 0}), make_mesh({0, 5, 0}), 1.0f).has_value());
}

TEST_CASE("gdk::overlap a mesh with no geometry overlaps nothing", "[gdk::collision]")
{
    const body empty{mesh_shape{}, {{0, 0, 0}, vector3_type::zero}};
    REQUIRE_FALSE(overlap_of(make_sphere({0, 5, 0}, {0, -10, 0}), empty, 1.0f).has_value());
    REQUIRE_FALSE(overlap_of(empty, make_sphere({0, 0, 0}), 1.0f).has_value());
}

TEST_CASE("gdk::overlap a mesh's hierarchy narrows the triangles tested", "[gdk::collision]")
{
    std::vector<vector3_type> vertices;
    std::vector<std::uint32_t> indices;
    constexpr int N = 200;

    for (int z = 0; z <= N; ++z)
        for (int x = 0; x <= N; ++x) vertices.push_back({x - N / 2.0f, 0, z - N / 2.0f});

    for (int z = 0; z < N; ++z)
        for (int x = 0; x < N; ++x) {
            const auto a = static_cast<std::uint32_t>(z * (N + 1) + x);
            indices.insert(indices.end(), {a, a + 1, a + N + 2, a, a + N + 2, a + N + 1});
        }

    const auto pMesh = std::dynamic_pointer_cast<const impl_mesh_data>(
        impl_mesh_data::make(std::move(vertices), std::move(indices)));

    REQUIRE(pMesh->triangle_count() == 80000);

    std::vector<std::uint32_t> candidates;
    pMesh->query({{-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f}}, candidates);

    REQUIRE(candidates.size() < 100);

    const body grid{mesh_shape{pMesh}, {{0, 0, 0}, vector3_type::zero}};
    const auto result = overlap_of(make_sphere({0, 5, 0}, {0, -10, 0}), grid, 1.0f);
    REQUIRE(result.has_value());
    REQUIRE(result->entry_time == Approx(0.45f).margin(TIME_MARGIN));
}

TEST_CASE("gdk::overlap a glancing approach is not missed", "[gdk::collision]")
{
    const body wall{box_shape{{5, 200, 200}}, {{-5, 0, 0}, vector3_type::zero}};

    for (int degrees = 0; degrees <= 75; degrees += 5) {
        INFO(degrees << " degrees off the surface normal");

        const auto radians = degrees * 3.14159265f / 180.0f;
        const vector3_type direction{-std::cos(radians), 0, std::sin(radians)};

        REQUIRE(overlap_of(make_sphere({5, 0, 0}, direction * 20.0f), wall, 1.0f).has_value());
    }
}

TEST_CASE("gdk::overlap a genuine near miss is still a miss", "[gdk::collision]")
{
    const body wall{box_shape{{5, 5, 5}}, {{-5, 0, 0}, vector3_type::zero}};

    REQUIRE_FALSE(overlap_of(make_sphere({2, 0, -10}, {0, 0, 20}), wall, 1.0f).has_value());
    REQUIRE_FALSE(overlap_of(make_sphere({4, 0, -10}, {-1, 0, 20}), wall, 1.0f).has_value());
}

TEST_CASE("gdk::overlap a flat seam between triangles is flagged, a real edge is not", "[gdk::collision]")
{
    SECTION("two coplanar triangles flag the diagonal they share")
    {
        const auto pQuad = std::dynamic_pointer_cast<const impl_mesh_data>(
            impl_mesh_data::make({{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, {0, 1, 2, 0, 2, 3}));

        REQUIRE(pQuad->internal_edges(0) == 0b100);
        REQUIRE(pQuad->internal_edges(1) == 0b001);
    }

    SECTION("a convex ridge does not flag the edge along its crest")
    {
        const auto pRidge = std::dynamic_pointer_cast<const impl_mesh_data>(impl_mesh_data::make(
            {{-1, 0, -1}, {-1, 0, 1}, {0, 1, -1}, {0, 1, 1}, {1, 0, -1}, {1, 0, 1}},
            {0, 1, 2, 1, 3, 2, 2, 3, 4, 3, 5, 4}));

        std::size_t flagged = 0;
        for (std::uint32_t t = 0; t < pRidge->triangle_count(); ++t)
            for (int e = 0; e < 3; ++e) if (pRidge->internal_edges(t) & (1u << e)) ++flagged;

        REQUIRE(flagged == 4);
    }

    SECTION("vertices repeated at the same position are still joined")
    {
        const auto pSplit = std::dynamic_pointer_cast<const impl_mesh_data>(impl_mesh_data::make(
            {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1},
             {-1, 0, -1}, {1, 0, 1}, {-1, 0, 1}},          
            {0, 1, 2, 3, 4, 5}));

        REQUIRE(pSplit->internal_edges(0) == 0b100);
        REQUIRE(pSplit->internal_edges(1) == 0b001);
    }
}

TEST_CASE("gdk::overlap a seam contact reports the surface's normal", "[gdk::collision]")
{
    const auto pQuad = std::dynamic_pointer_cast<const impl_mesh_data>(
        impl_mesh_data::make({{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, {0, 1, 2, 0, 2, 3}));

    const body floor{mesh_shape{pQuad}, {{0, 0, 0}, vector3_type::zero}};
    const auto result = overlap_of(make_sphere({0.4f, 0.45f, -0.4f}), floor, 1.0f / 60.0f);

    REQUIRE(result.has_value());
    require_vector_near(result->contact_normal, {0, 1, 0}, NORMAL_MARGIN);
}

TEST_CASE("gdk::overlap correcting a seam normal never removes the contact", "[gdk::collision]")
{
    const auto pRamp = std::dynamic_pointer_cast<const impl_mesh_data>(
        impl_mesh_data::make({{-1, -1, -1}, {1, 1, -1}, {1, 1, 1}, {-1, -1, 1}}, {0, 1, 2, 0, 2, 3}));

    const body ramp{mesh_shape{pRamp}, {{0, 0, 0}, vector3_type::zero}};
    const auto result = overlap_of(make_sphere({0, 5, 0}, {0, -10, 0}), ramp, 1.0f);

    REQUIRE(result.has_value());

    const auto expected = vector3_type{-1, 1, 0}.normal();
    require_vector_near(result->contact_normal, expected, 1e-2f);
}

TEST_CASE("gdk::overlap every shape meets a heightfield's surface", "[gdk::collision]")
{
    const auto flat = make_heightfield(5, 5, [](std::size_t, std::size_t) { return 0.0f; });

    const auto check = [&](const char *aLabel, const body &aBody,
        const floating_point_type aExpectedTime) {
        INFO(aLabel);
        const auto result = overlap_of(aBody, flat, 1.0f);
        REQUIRE(result.has_value());
        REQUIRE(result->entry_time == Approx(aExpectedTime).margin(TIME_MARGIN));
        require_vector_near(result->contact_normal, {0, 1, 0}, NORMAL_MARGIN);
    };

    check("sphere", make_sphere({0, 5, 0}, {0, -10, 0}), 0.45f);
    check("box", body{box_shape{{0.5f, 0.5f, 0.5f}}, {{0, 5, 0}, {0, -10, 0}}}, 0.45f);
    check("obb", body{obb_shape{{0.5f, 0.5f, 0.5f}}, {{0, 5, 0}, {0, -10, 0}}}, 0.45f);
    check("capsule", make_capsule({0, 5, 0}, {0, -10, 0}, 1.0f), 0.35f);
}

TEST_CASE("gdk::overlap a heightfield is bounded by its grid", "[gdk::collision]")
{
    const auto flat = make_heightfield(5, 5, [](std::size_t, std::size_t) { return 0.0f; });

    REQUIRE(overlap_of(make_sphere({1.5f, 5, 0}, {0, -10, 0}), flat, 1.0f).has_value());
    REQUIRE_FALSE(overlap_of(make_sphere({20, 5, 0}, {0, -10, 0}), flat, 1.0f).has_value());
    REQUIRE_FALSE(overlap_of(make_sphere({0, 5, 20}, {0, -10, 0}), flat, 1.0f).has_value());
}

TEST_CASE("gdk::overlap a heightfield's samples shape its surface", "[gdk::collision]")
{
    const auto ramp = make_heightfield(5, 5,
        [](const std::size_t aColumn, const std::size_t) { return static_cast<float>(aColumn); });

    const auto result = overlap_of(make_sphere({0, 8, 0}, {0, -10, 0}), ramp, 1.0f);

    REQUIRE(result.has_value());
    require_vector_near(result->contact_normal, vector3_type{-1, 1, 0}.normal(), 1e-2f);
}

TEST_CASE("gdk::overlap a heightfield flags flat seams but not a ridge", "[gdk::collision]")
{
    const auto count_flags = [](const heightfield_data_ptr_type &aField) {
        const auto pField = std::dynamic_pointer_cast<const impl_heightfield_data>(aField);
        std::size_t flagged = 0;
        for (std::uint32_t t = 0; t < pField->triangle_count(); ++t)
            for (int e = 0; e < 3; ++e) if (pField->internal_edges(t) & (1u << e)) ++flagged;
        return flagged;
    };

    const auto flat = make_heightfield_data(5, 5, [](std::size_t, std::size_t) { return 0.0f; });
    const auto ridged = make_heightfield_data(5, 5,
        [](const std::size_t aColumn, const std::size_t) { return aColumn == 2 ? 3.0f : 0.0f; });

    REQUIRE(count_flags(ridged) < count_flags(flat));
}

TEST_CASE("gdk::overlap heightfields do not collide with each other or with meshes", "[gdk::collision]")
{
    const auto flat = make_heightfield(5, 5, [](std::size_t, std::size_t) { return 0.0f; });
    const auto other = make_heightfield(5, 5, [](std::size_t, std::size_t) { return 0.0f; });

    REQUIRE_NOTHROW(overlap_of(flat, other, 1.0f));
    REQUIRE_FALSE(overlap_of(flat, other, 1.0f).has_value());
    REQUIRE_FALSE(overlap_of(flat, make_mesh({0, 0, 0}), 1.0f).has_value());
    REQUIRE_FALSE(overlap_of(make_mesh({0, 0, 0}), flat, 1.0f).has_value());
}

TEST_CASE("gdk::overlap malformed heightfield dimensions are refused at build time", "[gdk::collision]")
{
    const std::vector<floating_point_type> nine(9, 0.0f);

    REQUIRE_THROWS_AS(impl_heightfield_data::make(1, 3, nine), exception);      
    REQUIRE_THROWS_AS(impl_heightfield_data::make(3, 1, nine), exception);      
    REQUIRE_THROWS_AS(impl_heightfield_data::make(3, 4, nine), exception);      
    REQUIRE_THROWS_AS(impl_heightfield_data::make(3, 3, nine, {0, 1}), exception);  
    REQUIRE_NOTHROW(impl_heightfield_data::make(3, 3, nine));
}

TEST_CASE("gdk::overlap geometry can be read back through the public interface", "[gdk::collision]")
{
    SECTION("a mesh reports the triangles it was built from")
    {
        const auto pMesh = impl_mesh_data::make(
            {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, {0, 1, 2, 0, 2, 3});

        REQUIRE(pMesh->triangle_count() == 2);

        const auto first = pMesh->triangle(0);
        require_vector_near(first.a, {-1, 0, -1}, 1e-6f);
        require_vector_near(first.b, {1, 0, -1}, 1e-6f);
        require_vector_near(first.c, {1, 0, 1}, 1e-6f);

        const auto second = pMesh->triangle(1);
        require_vector_near(second.a, {-1, 0, -1}, 1e-6f);
        require_vector_near(second.b, {1, 0, 1}, 1e-6f);
        require_vector_near(second.c, {-1, 0, 1}, 1e-6f);
    }

    SECTION("terrain reports its sample spacing")
    {
        const auto pTerrain = impl_heightfield_data::make(3, 4,
            std::vector<floating_point_type>(12, 0.0f), {2.0f, 0.5f});

        REQUIRE(pTerrain->columns() == 3);
        REQUIRE(pTerrain->rows() == 4);
        REQUIRE(pTerrain->cell_size().x == Approx(2.0f));
        REQUIRE(pTerrain->cell_size().y == Approx(0.5f));
    }

    SECTION("terrain reports the heights it was built from")
    {
        const auto pTerrain = impl_heightfield_data::make(3, 3,
            {0, 1, 2, 3, 4, 5, 6, 7, 8});

        REQUIRE(pTerrain->height(0, 0) == Approx(0.0f));
        REQUIRE(pTerrain->height(2, 0) == Approx(2.0f));
        REQUIRE(pTerrain->height(0, 2) == Approx(6.0f));
        REQUIRE(pTerrain->height(2, 2) == Approx(8.0f));
    }
}

TEST_CASE("gdk::overlap malformed mesh geometry is refused at build time", "[gdk::collision]")
{
    REQUIRE_THROWS_AS(impl_mesh_data::make({{0, 0, 0}, {1, 0, 0}, {0, 0, 1}}, {0, 1}), exception);
    REQUIRE_THROWS_AS(impl_mesh_data::make({{0, 0, 0}, {1, 0, 0}, {0, 0, 1}}, {0, 1, 7}), exception);
    REQUIRE_NOTHROW(impl_mesh_data::make({{0, 0, 0}, {1, 0, 0}, {0, 0, 1}}, {0, 1, 2}));
}

TEST_CASE("gdk::collisions::narrow_phase_penetration answers without any motion", "[gdk::collision]")
{
    const impl_collision_policy policy;

    SECTION("a stationary sphere overlapping a box reports its depth") {
        const auto subject = make_sphere({0.8f, 0, 0});
        const auto other = make_box({0, 0, 0});

        const auto result = narrow_phase_penetration(subject.shape, subject.kinematics,
            other.shape, other.kinematics, policy);

        REQUIRE(result.has_value());
        REQUIRE(result->penetration == Approx(0.2f).margin(1e-4f));
        REQUIRE(result->contact_normal.x == Approx(1.0f).margin(1e-4f));
        REQUIRE(result->entry_time == Approx(0.0f));
    }

    SECTION("the swept query still answers for a stationary sphere and box") {
        const auto subject = make_sphere({0.8f, 0, 0});
        const auto other = make_box({0, 0, 0});

        REQUIRE(narrow_phase_overlap(subject.shape, subject.kinematics,
            other.shape, other.kinematics, 0.0f, policy).has_value());
    }

    SECTION("a separated pair reports nothing") {
        const auto subject = make_sphere({1.2f, 0, 0});   
        const auto other = make_box({0, 0, 0});

        REQUIRE_FALSE(narrow_phase_penetration(subject.shape, subject.kinematics,
            other.shape, other.kinematics, policy).has_value());
    }

    SECTION("a capsule across a rotated box's edge reports a depth") {
        quaternion_type rotation;
        rotation.set_from_euler({0.3f, 0.6f, 0.4f});

        const auto subject = make_capsule({0.6f, 0.3f, 0}, vector3_type::zero);
        const auto other = make_obb({0, 0, 0}, vector3_type::zero, rotation);

        const auto result = narrow_phase_penetration(subject.shape, subject.kinematics,
            other.shape, other.kinematics, policy);

        REQUIRE(result.has_value());
        REQUIRE(result->penetration > 0.0f);
        REQUIRE(result->contact_normal.length() == Approx(1.0f).margin(1e-3f));
    }
}
