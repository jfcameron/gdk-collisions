// © Joseph Cameron - MIT License

#include "scene_fixture.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

using namespace gdk::collisions;
using namespace gdk::collisions::scene_testing;

namespace {
    [[nodiscard]] vector3_type turn_by(const quaternion_type &aQ,
        const vector3_type &aV) {
        const vector3_type axis{aQ.x, aQ.y, aQ.z};
        const vector3_type t{
            2.0f * (axis.y * aV.z - axis.z * aV.y),
            2.0f * (axis.z * aV.x - axis.x * aV.z),
            2.0f * (axis.x * aV.y - axis.y * aV.x)};
        return vector3_type{
            aV.x + aQ.w * t.x + (axis.y * t.z - axis.z * t.y),
            aV.y + aQ.w * t.y + (axis.z * t.x - axis.x * t.z),
            aV.z + aQ.w * t.z + (axis.x * t.y - axis.y * t.x)};
    }

    [[nodiscard]] floating_point_type penetration_into_box(
        const vector3_type &aCapsuleCentre, const quaternion_type &aCapsuleRotation,
        const floating_point_type aRadius, const floating_point_type aHalfHeight,
        const vector3_type &aBoxCentre, const quaternion_type &aBoxRotation,
        const vector3_type &aHalfExtents) {
        const auto axis = turn_by(aCapsuleRotation, vector3_type{0, aHalfHeight, 0});
        const auto a = aCapsuleCentre - axis;
        const auto b = aCapsuleCentre + axis;

        quaternion_type inverse;
        inverse.w = aBoxRotation.w;
        inverse.x = -aBoxRotation.x;
        inverse.y = -aBoxRotation.y;
        inverse.z = -aBoxRotation.z;

        auto nearest = std::numeric_limits<floating_point_type>::max();
        constexpr int SAMPLES = 2000;
        for (int i = 0; i <= SAMPLES; ++i) {
            const auto local = turn_by(inverse,
                (a + (b - a) * (static_cast<floating_point_type>(i) / SAMPLES)) - aBoxCentre);

            const auto clamp = [](const floating_point_type aValue,
                const floating_point_type aLimit) {
                return aValue < -aLimit ? -aLimit : (aValue > aLimit ? aLimit : aValue);
            };
            const vector3_type onBox{clamp(local.x, aHalfExtents.x),
                clamp(local.y, aHalfExtents.y), clamp(local.z, aHalfExtents.z)};

            nearest = std::min(nearest, (local - onBox).length());
        }
        return aRadius - nearest;
    }

    [[nodiscard]] floating_point_type slide_penetration(
        const quaternion_type &aBoxRotation, const vector3_type &aStart,
        const vector3_type &aVelocity) {
        fixture f;

        const vector3_type halfExtents{0.5f, 0.5f, 0.5f};
        const vector3_type boxCentre = vector3_type::zero;

        matrix4x4_type boxTransform;
        boxTransform.set_rotation(aBoxRotation);
        const auto pBox = f.scene->make_static_obb_collider(boxTransform, halfExtents);

        const auto pCapsule = f.scene->make_capsule_collider(
            response_handlers::slide_projecting());
        pCapsule->set_position(aStart);

        auto worst = std::numeric_limits<floating_point_type>::lowest();
        for (int frame = 0; frame < 240; ++frame) {
            pCapsule->add_velocity(aVelocity);
            f.scene->update(1.0f / 60.0f);

            worst = std::max(worst, penetration_into_box(pCapsule->transform().translation(),
                pCapsule->rotation(), 0.5f, 0.5f, boxCentre, aBoxRotation, halfExtents));
        }

        REQUIRE(pBox->half_extents().x == Approx(0.5f));   
        return worst;
    }
}

namespace {
    [[nodiscard]] std::vector<floating_point_type> obb_slide_gaps(
        const quaternion_type &aFrame, const vector3_type &aStart,
        const vector3_type &aVelocity, const int aFrames) {
        fixture f;

        const vector3_type halfExtents{0.5f, 0.5f, 0.5f};
        matrix4x4_type boxTransform;
        boxTransform.set_rotation(aFrame);
        const auto pBox = f.scene->make_static_obb_collider(boxTransform, halfExtents);

        const auto pCapsule = f.scene->make_capsule_collider(
            response_handlers::slide_projecting());
        pCapsule->set_position(turn_by(aFrame, aStart));
        pCapsule->set_rotation(aFrame);

        std::vector<floating_point_type> gaps;
        for (int frame = 0; frame < aFrames; ++frame) {
            pCapsule->add_velocity(turn_by(aFrame, aVelocity));
            f.scene->update(1.0f / 60.0f);

            gaps.push_back(-penetration_into_box(pCapsule->transform().translation(),
                pCapsule->rotation(), 0.5f, 0.5f, vector3_type::zero, aFrame, halfExtents));
        }

        REQUIRE(pBox->half_extents().x == Approx(0.5f));   
        return gaps;
    }
}

namespace {
    struct held_slide_extremes final {
        floating_point_type deepest_penetration;
        floating_point_type furthest_separation;
    };

    [[nodiscard]] held_slide_extremes held_slide(const quaternion_type &aBoxRotation,
        const vector3_type &aStartDirection, const vector3_type &aOrbitAxis,
        const floating_point_type aPressRadians, const int aFrames) {
        fixture f;

        const vector3_type halfExtents{0.5f, 0.5f, 0.5f};
        matrix4x4_type boxTransform;
        boxTransform.set_rotation(aBoxRotation);
        const auto pBox = f.scene->make_static_obb_collider(boxTransform, halfExtents);

        const auto pCapsule = f.scene->make_capsule_collider(
            response_handlers::slide_projecting());
        pCapsule->set_position(aStartDirection * 1.3f);

        const auto cross = [](const vector3_type &a, const vector3_type &b) {
            return vector3_type{
                a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        };

        held_slide_extremes extremes{0, 0};
        bool contacted = false;
        const auto press = std::sin(aPressRadians);
        const auto along = std::cos(aPressRadians);

        for (int frame = 0; frame < aFrames; ++frame) {
            auto toCentre = pCapsule->transform().translation() * -1.0f;
            const auto distance = toCentre.length();
            if (distance > 1e-4f) toCentre = toCentre * (1.0f / distance);

            auto tangent = cross(toCentre, aOrbitAxis);
            const auto tangentLength = tangent.length();
            if (tangentLength > 1e-4f) tangent = tangent * (1.0f / tangentLength);

            pCapsule->add_velocity((toCentre * press + tangent * along) * 10.0f);   
            f.scene->update(1.0f / 60.0f);

            const auto gap = -penetration_into_box(pCapsule->transform().translation(),
                pCapsule->rotation(), 0.5f, 0.5f, vector3_type::zero, aBoxRotation,
                halfExtents);

            if (gap < 0.05f) contacted = true;
            if (contacted) {
                extremes.deepest_penetration = std::min(extremes.deepest_penetration, gap);
                extremes.furthest_separation = std::max(extremes.furthest_separation, gap);
            }
        }

        REQUIRE(pBox->half_extents().x == Approx(0.5f));   
        return extremes;
    }
}

TEST_CASE("gdk::collider add_rotation accumulates identically in both directions",
    "[gdk::collision]")
{
    fixture f;

    const auto z_angle_of = [](const collider &aBody) {
        const auto turned = aBody.rotation() * vector3_type{1, 0, 0};
        return std::atan2(turned.y, turned.x);
    };

    const auto accumulate = [&f](const floating_point_type aStep, const int aTimes) {
        const auto pBody = f.scene->make_capsule_collider();

        quaternion_type turn;
        turn.set_from_euler({0, 0, aStep});

        for (int i = 0; i < aTimes; ++i) pBody->add_rotation(turn);
        return pBody;
    };

    SECTION("fifty steps of 0.02 reach 1.0 radian, whichever way they go")
    {
        const auto pPositive = accumulate(+0.02f, 50);
        const auto pNegative = accumulate(-0.02f, 50);

        REQUIRE(z_angle_of(*pPositive) == Approx(+1.0f).margin(1e-3f));
        REQUIRE(z_angle_of(*pNegative) == Approx(-1.0f).margin(1e-3f));
        REQUIRE(z_angle_of(*pPositive) == Approx(-z_angle_of(*pNegative)).margin(1e-4f));
    }

    SECTION("every single step moves the body, in both directions")
    {
        for (const auto step : {+0.02f, -0.02f}) {
            const auto pBody = f.scene->make_capsule_collider();

            quaternion_type turn;
            turn.set_from_euler({0, 0, step});

            auto previous = z_angle_of(*pBody);

            for (int i = 0; i < 50; ++i) {
                pBody->add_rotation(turn);

                const auto current = z_angle_of(*pBody);
                REQUIRE(current != Approx(previous).margin(1e-5f));
                REQUIRE((current - previous) == Approx(step).margin(1e-4f));
                previous = current;
            }
        }
    }

    SECTION("the transform stays consistent with the rotation as they accumulate")
    {
        for (const auto step : {+0.02f, -0.02f}) {
            const auto pBody = accumulate(step, 50);

            const vector3_type probe{0.3f, 0.5f, -0.8f};
            const auto viaQuaternion = pBody->rotation() * probe;

            const vector4_type probe4{probe.x, probe.y, probe.z, 1.0f};
            const auto viaMatrix = pBody->transform() * probe4;

            REQUIRE(viaQuaternion.x == Approx(viaMatrix.x).margin(1e-4f));
            REQUIRE(viaQuaternion.y == Approx(viaMatrix.y).margin(1e-4f));
            REQUIRE(viaQuaternion.z == Approx(viaMatrix.z).margin(1e-4f));
        }
    }

    SECTION("the transform stays orthonormal as rotations accumulate")
    {
        for (const auto step : {+0.02f, -0.02f}) {
            const auto scale = accumulate(step, 500)->transform().scale();

            REQUIRE(scale.x == Approx(1.0f).margin(1e-3f));
            REQUIRE(scale.y == Approx(1.0f).margin(1e-3f));
            REQUIRE(scale.z == Approx(1.0f).margin(1e-3f));
        }
    }

    SECTION("update() never alters a collider's orientation")
    {
        const auto pBox = f.scene->make_aabb_collider();
        pBox->set_position({0, 0, 0});
        pBox->set_inverse_overlap_weight(0);

        const auto pCapsule = f.scene->make_capsule_collider();
        pCapsule->set_position({0, 1.5f, 0});

        quaternion_type turn;
        turn.set_from_euler({0, 0, 0.02f});

        for (int frame = 0; frame < 120; ++frame) {
            pCapsule->add_velocity({0, -4.0f, 0});
            pCapsule->add_rotation(turn);

            const auto before = z_angle_of(*pCapsule);
            f.update(1.0f / 60.0f);
            REQUIRE(z_angle_of(*pCapsule) == Approx(before).margin(1e-6f));
        }
    }
}

TEST_CASE("gdk::collider can be oriented after creation", "[gdk::collision]")
{
    fixture f;

    SECTION("add_rotation composes in world space, matching add_velocity")
    {
        const auto pBody = f.scene->make_capsule_collider();

        quaternion_type quarter;
        quarter.set_from_euler({0, 0, 3.14159265f * 0.5f});

        pBody->add_rotation(quarter);
        pBody->add_rotation(quarter);

        const auto up = pBody->rotation();
        const vector3_type localY{0, 1, 0};
        const auto turned = up * localY;

        REQUIRE(turned.y == Approx(-1.0f).margin(1e-4f));
    }

    SECTION("a transform's rotation survives set_transform")
    {
        const auto pBody = f.scene->make_obb_collider();

        quaternion_type turn;
        turn.set_from_euler({0.3f, -0.7f, 1.1f});

        matrix4x4_type transform;
        transform.set_rotation(turn);

        std::dynamic_pointer_cast<impl_collider>(pBody)->set_transform(transform);

        const vector3_type probe{0.3f, 0.5f, -0.8f};
        const auto expected = turn * probe;
        const auto actual = pBody->rotation() * probe;

        REQUIRE(actual.x == Approx(expected.x).margin(1e-4f));
        REQUIRE(actual.y == Approx(expected.y).margin(1e-4f));
        REQUIRE(actual.z == Approx(expected.z).margin(1e-4f));
    }

    SECTION("a rotated static puts its geometry where the transform says")
    {
        quaternion_type quarter;
        quarter.set_from_euler({0, 0, 3.14159265f * 0.5f});

        matrix4x4_type transform;
        transform.set_rotation(quarter);

        const auto pRotated = f.scene->make_static_compound_collider(transform,
            [](compound_collider &aBuild) { aBuild.add_sphere({2.0f, 0, 0}, 0.5f); });

        const auto pProbe = f.scene->make_sphere_collider();
        pProbe->set_position({0, 2.0f, 0});

        f.update(1.0f / 60.0f);

        REQUIRE(f.count(event_type::enter) == 1);
        static_cast<void>(pRotated);
    }

    SECTION("a turned capsule collides along its new axis")
    {
        const auto pGround = f.scene->make_static_plane_collider(matrix4x4_type::identity);

        const auto pBody = f.scene->make_capsule_collider();
        pBody->set_radius(0.25f);
        pBody->set_half_height(1.5f);
        pBody->set_position({0, 4, 0});

        const auto pWall = f.scene->make_static_sphere_collider(
            [] { matrix4x4_type t; t.set_translation({1.4f, 4, 0}); return t; }(), 0.5f);

        pBody->add_velocity({0, 0, 0});
        f.update(1.0f / 60.0f);
        REQUIRE(f.count(event_type::enter) == 0);

        quaternion_type quarter;
        quarter.set_from_euler({0, 0, 3.14159265f * 0.5f});
        pBody->add_rotation(quarter);

        f.update(1.0f / 60.0f);
        REQUIRE(f.count(event_type::enter) == 1);

        static_cast<void>(pGround);
        static_cast<void>(pWall);
    }
}

TEST_CASE("gdk::scene keeps a rotating capsule out of the surface it is pressed against",
    "[gdk::collision]")
{
    fixture f;

    const auto pBox = f.scene->make_aabb_collider();
    pBox->set_position({0, 0, 0});

    pBox->set_inverse_overlap_weight(0);

    const auto pCapsule = f.scene->make_capsule_collider();
    pCapsule->set_radius(0.5f);
    pCapsule->set_half_height(0.5f);
    pCapsule->set_position({0, 1.5f, 0});   

    const auto lowest_point_of = [&pCapsule] {
        const auto centre = pCapsule->transform().translation();
        const auto axis = pCapsule->rotation() * vector3_type{0, 0.5f, 0};
        return std::min(centre.y + axis.y, centre.y - axis.y) - 0.5f;
    };

    REQUIRE(lowest_point_of() == Approx(0.5f).margin(1e-4f));

    quaternion_type turn;
    turn.set_from_euler({0, 0, 0.02f});   

    constexpr float NEAR_PARALLEL_LOW = 1.396f;    
    constexpr float NEAR_PARALLEL_HIGH = 1.745f;   

    auto deepestAway = 0.0f;
    auto deepestNearParallel = 0.0f;
    auto highest = lowest_point_of();

    for (int frame = 0; frame < 120; ++frame) {
        pCapsule->add_velocity({0, -4.0f, 0});
        pCapsule->add_rotation(turn);
        f.update(1.0f / 60.0f);

        const auto angle = (frame + 1) * 0.02f;
        const auto depth = 0.5f - lowest_point_of();

        if (angle > NEAR_PARALLEL_LOW && angle < NEAR_PARALLEL_HIGH)
            deepestNearParallel = std::max(deepestNearParallel, depth);
        else
            deepestAway = std::max(deepestAway, depth);

        highest = std::max(highest, lowest_point_of());
    }

    REQUIRE(deepestAway < 1e-4f);

    REQUIRE(deepestNearParallel < 0.02f);

    REQUIRE(highest < 0.5f + 1e-2f);

    REQUIRE(pBox->transform().translation().y == Approx(0.0f).margin(1e-5f));

    const auto finalAxis = pCapsule->rotation() * vector3_type{0, 0.5f, 0};
    REQUIRE(std::abs(finalAxis.y) < 0.45f);   
    REQUIRE(lowest_point_of() == Approx(0.5f).margin(1e-2f));
}

TEST_CASE("gdk::collider turning into a neighbour does not displace an immovable one",
    "[gdk::collision]")
{
    const auto settle = [](const floating_point_type aWeight) {
        fixture f;

        const auto pBox = f.scene->make_aabb_collider();
        pBox->set_position({0, 0, 0});
        pBox->set_inverse_overlap_weight(aWeight);

        const auto pCapsule = f.scene->make_capsule_collider();
        pCapsule->set_position({0, 1.6f, 0});

        quaternion_type turn;
        turn.set_from_euler({0, 0, 0.02f});

        for (int frame = 0; frame < 120; ++frame) {
            pCapsule->add_velocity({0, -4.0f, 0});
            pCapsule->add_rotation(turn);
            f.update(1.0f / 60.0f);
        }

        return pBox->transform().translation().y;
    };

    REQUIRE(settle(1.0f) < -0.05f);

    REQUIRE(settle(0.0f) == Approx(0.0f).margin(1e-5f));
}

TEST_CASE("gdk::scene rotates a compound's parts with it", "[gdk::collision]")
{
    const auto arm_reaches = [](const bool aTurned, const vector3_type &aWhere) {
        fixture f;
        matrix4x4_type transform;
        if (aTurned) transform.set_rotation(quaternion_type(
            vector3_type{0, 0, 3.14159265f / 2.0f}));

        const auto pWall = f.scene->make_static_compound_collider(transform, [](compound_collider &aBuild) {
            aBuild.add_box({0, 2.0f, 0}, {0.5f, 0.5f, 0.5f});
        });

        const auto pProbe = f.scene->make_sphere_collider();
        pProbe->set_position(aWhere);

        f.update(0.01f);
        return !f.collisions.empty();
    };

    SECTION("unrotated, the arm is where it was placed")
    {
        REQUIRE(arm_reaches(false, {0, 2, 0}));
        REQUIRE_FALSE(arm_reaches(false, {2, 0, 0}));
    }

    SECTION("a quarter turn about Z carries it onto the negative X axis")
    {
        REQUIRE(arm_reaches(true, {-2, 0, 0}));
        REQUIRE_FALSE(arm_reaches(true, {2, 0, 0}));
        REQUIRE_FALSE(arm_reaches(true, {0, 2, 0}));
    }
}

TEST_CASE("gdk::scene pushes a body out when its rotation drives it in", "[gdk::collision]")
{
    fixture f;
    matrix4x4_type identity;
    const auto pWall = f.scene->make_static_aabb_collider(identity, {5, 5, 0.5f});

    const auto pCapsule = f.scene->make_capsule_collider();
    pCapsule->set_radius(0.4f);
    pCapsule->set_half_height(1.0f);
    pCapsule->set_position({0, 0, -3});

    for (int frame = 0; frame < 60; ++frame) {
        pCapsule->add_velocity({0, 0, 4});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(pCapsule->transform().translation().z == Approx(-0.9f).margin(1e-3f));

    const auto pImplementation = std::dynamic_pointer_cast<impl_capsule_collider>(pCapsule);
    REQUIRE(pImplementation != nullptr);

    for (int frame = 0; frame < 30; ++frame) {
        pImplementation->set_rotation(quaternion_type::from_euler(
            {(frame + 1) / 30.0f * 3.14159265f / 2.0f, 0, 0}));
        f.update(1.0f / 60.0f);
    }

    const auto z = pCapsule->transform().translation().z;
    REQUIRE(z <= Approx(-1.9f).margin(1e-2f));
    REQUIRE(z > -4.0f);
}

TEST_CASE("gdk::scene a capsule sliding on an unrotated box stays outside it", "[gdk::collision]")
{
    REQUIRE(slide_penetration(quaternion_type::identity, {-3, 0, 0}, {3, 0, 0}) < 1e-3f);
}

TEST_CASE("gdk::scene a capsule sliding on a rotated obb stays outside it", "[gdk::collision]")
{
    quaternion_type rotation;
    rotation.set_from_euler({0.3f, 0.6f, 0.4f});   

    REQUIRE(slide_penetration(rotation, {-3, 0, 0}, {3, 0, 0}) < 1e-3f);
}

TEST_CASE("gdk::scene a capsule slid shallowly across a rotated obb stays outside it",
    "[gdk::collision]")
{
    quaternion_type rotation;
    rotation.set_from_euler({0.3f, 0.6f, 0.4f});

    REQUIRE(slide_penetration(rotation, {-2, 1.2f, 0}, {3, -0.4f, 0}) < 1e-3f);
}

TEST_CASE("gdk::scene slides a capsule along an obb the same way in any frame",
    "[gdk::collision]")
{
    quaternion_type rotated;
    rotated.set_from_euler({0.3f, 0.6f, 0.4f});

    const auto plain = obb_slide_gaps(quaternion_type::identity, {-2, 0.9f, 0}, {3, -0.6f, 0}, 240);
    const auto spun = obb_slide_gaps(rotated, {-2, 0.9f, 0}, {3, -0.6f, 0}, 240);

    REQUIRE(plain.size() == spun.size());

    auto worst = floating_point_type{0};
    for (std::size_t i = 0; i < plain.size(); ++i)
        worst = std::max(worst, std::abs(plain[i] - spun[i]));

    REQUIRE(worst < 0.01f);
}

TEST_CASE("gdk::scene a capsule held against a rotated obb never sinks into it",
    "[gdk::collision]")
{
    quaternion_type rotation;
    rotation.set_from_euler({0.3f, 0.6f, 0.4f});   

    const vector3_type starts[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const vector3_type axes[] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}};

    for (const auto &start : starts) for (const auto &axis : axes) {
        if (std::abs(start.dot_product(axis)) > 0.99f) continue;   
        for (const auto degrees : {5.0f, 15.0f, 30.0f, 60.0f}) {
            const auto extremes = held_slide(rotation, start, axis,
                degrees * 3.14159265f / 180.0f, 240);

            REQUIRE(extremes.deepest_penetration > -1e-3f);
        }
    }
}

TEST_CASE("gdk::scene a capsule held against an obb behaves the same at any rotation",
    "[gdk::collision]")
{
    const vector3_type start{0.577f, 0.577f, 0.577f};
    const vector3_type axis{0, 1, 0};
    constexpr auto PRESS = 30.0f * 3.14159265f / 180.0f;

    const auto control = held_slide(quaternion_type::identity, start, axis, PRESS, 240);
    REQUIRE(control.deepest_penetration > -1e-3f);

    const vector3_type eulers[] = {
        {0.05f, 0, 0},              
        {0.2f, 0.1f, 0},
        {0.3f, 0.6f, 0.4f},         
        {0, 0, 0.7853982f},         
        {1.1f, 0.7f, 0.3f},
        {0.9f, 1.3f, 2.1f}};

    for (const auto &euler : eulers) {
        quaternion_type rotation;
        rotation.set_from_euler(euler);

        const auto extremes = held_slide(rotation, start, axis, PRESS, 240);

        REQUIRE(extremes.deepest_penetration > -1e-3f);
        REQUIRE(extremes.furthest_separation < control.furthest_separation + 0.25f);
    }
}

TEST_CASE("gdk::collider transform() and rotation() describe the same box", "[gdk::collision]")
{
    quaternion_type rotation;
    rotation.set_from_euler({0.3f, 0.6f, 0.4f});   

    matrix4x4_type transform;
    transform.set_translation({-7, 0, 0});
    transform.set_rotation(rotation);

    fixture f;
    const vector3_type halfExtents{0.9f, 0.3f, 0.5f};
    const auto pBox = f.scene->make_static_obb_collider(transform, halfExtents);

    const auto position = pBox->transform().translation();
    const auto matrix = pBox->transform();

    for (int corner = 0; corner < 8; ++corner) {
        const vector3_type local{
            (corner & 1) ? halfExtents.x : -halfExtents.x,
            (corner & 2) ? halfExtents.y : -halfExtents.y,
            (corner & 4) ? halfExtents.z : -halfExtents.z};

        const auto tested = turn_by(pBox->rotation(), local) + position;

        const vector3_type drawn{
            matrix.get(0, 0) * local.x + matrix.get(1, 0) * local.y + matrix.get(2, 0) * local.z + matrix.get(3, 0),
            matrix.get(0, 1) * local.x + matrix.get(1, 1) * local.y + matrix.get(2, 1) * local.z + matrix.get(3, 1),
            matrix.get(0, 2) * local.x + matrix.get(1, 2) * local.y + matrix.get(2, 2) * local.z + matrix.get(3, 2)};

        REQUIRE((drawn - tested).length() < 1e-5f);
    }

    const auto recovered = pBox->rotation();
    REQUIRE(recovered.w == Approx(rotation.w).margin(1e-5f));
    REQUIRE(recovered.x == Approx(rotation.x).margin(1e-5f));
    REQUIRE(recovered.y == Approx(rotation.y).margin(1e-5f));
    REQUIRE(recovered.z == Approx(rotation.z).margin(1e-5f));
}

TEST_CASE("gdk::collider angular velocity is per-frame input, like velocity", "[gdk::collision]")
{
    fixture f;
    const auto pBody = f.scene->make_obb_collider();
    pBody->set_half_extents({1.0f, 0.25f, 0.25f});

    SECTION("the scene integrates the orientation from it and then clears it")
    {
        constexpr auto QUARTER = 1.5707963f;
        pBody->set_angular_velocity({0, QUARTER, 0});

        f.update(1.0f / 60.0f);

        REQUIRE(pBody->angular_velocity().length() == Approx(0.0f).margin(1e-6f));

        const auto turned = pBody->rotation();
        REQUIRE(turned.y == Approx(std::sin(QUARTER / 60.0f * 0.5f)).margin(1e-5f));
    }

    SECTION("a held rate integrates to the angle it should over a second")
    {
        constexpr auto QUARTER = 1.5707963f;

        for (int frame = 0; frame < 60; ++frame) {
            pBody->add_angular_velocity({0, QUARTER, 0});
            f.update(1.0f / 60.0f);
        }

        const auto turnedX = turn_by(pBody->rotation(), vector3_type{1, 0, 0});
        REQUIRE(turnedX.x == Approx(0.0f).margin(2e-2f));
        REQUIRE(turnedX.z == Approx(-1.0f).margin(2e-2f));
    }

    SECTION("add accumulates within a frame, as add_velocity does")
    {
        pBody->add_angular_velocity({0, 1.0f, 0});
        pBody->add_angular_velocity({0, 2.0f, 0});
        REQUIRE(pBody->angular_velocity().y == Approx(3.0f));
    }
}

TEST_CASE("gdk::collider set_rotation normalises what it is given", "[gdk::collision]")
{
    fixture f;
    const auto pBody = f.scene->make_obb_collider();
    pBody->set_half_extents({1.0f, 0.5f, 0.25f});

    quaternion_type wanted;
    wanted.set_from_euler({0.3f, 0.6f, 0.4f});

    SECTION("a scaled quaternion is stored as the rotation it represents, at unit length")
    {
        quaternion_type overlong;
        overlong.w = wanted.w * 3.0f;
        overlong.x = wanted.x * 3.0f;
        overlong.y = wanted.y * 3.0f;
        overlong.z = wanted.z * 3.0f;

        pBody->set_rotation(overlong);

        const auto stored = pBody->rotation();
        const auto length = std::sqrt(stored.w * stored.w + stored.x * stored.x
            + stored.y * stored.y + stored.z * stored.z);

        REQUIRE(length == Approx(1.0f).margin(1e-5f));
        REQUIRE(stored.w == Approx(wanted.w).margin(1e-5f));
        REQUIRE(stored.x == Approx(wanted.x).margin(1e-5f));
        REQUIRE(stored.y == Approx(wanted.y).margin(1e-5f));
        REQUIRE(stored.z == Approx(wanted.z).margin(1e-5f));

        const auto scale = pBody->transform().scale();
        REQUIRE(scale.x == Approx(1.0f).margin(1e-4f));
        REQUIRE(scale.y == Approx(1.0f).margin(1e-4f));
        REQUIRE(scale.z == Approx(1.0f).margin(1e-4f));
    }

    SECTION("drift accumulated by a caller integrating its own orientation is absorbed")
    {
        quaternion_type drifting = quaternion_type::identity;
        quaternion_type step;
        step.set_from_euler({0, 0.02f, 0});

        for (int frame = 0; frame < 500; ++frame) {
            drifting = step * drifting;          
            pBody->set_rotation(drifting);
        }

        const auto scale = pBody->transform().scale();
        REQUIRE(scale.x == Approx(1.0f).margin(1e-3f));
        REQUIRE(scale.y == Approx(1.0f).margin(1e-3f));
        REQUIRE(scale.z == Approx(1.0f).margin(1e-3f));
    }

    SECTION("a zero-length quaternion gives identity rather than NaN")
    {
        quaternion_type zero;
        zero.w = 0; zero.x = 0; zero.y = 0; zero.z = 0;

        pBody->set_rotation(zero);

        const auto stored = pBody->rotation();
        REQUIRE(std::isfinite(stored.w));
        REQUIRE(stored.w == Approx(1.0f).margin(1e-5f));

        const auto scale = pBody->transform().scale();
        REQUIRE(std::isfinite(scale.x));
        REQUIRE(scale.x == Approx(1.0f).margin(1e-4f));
    }
}
