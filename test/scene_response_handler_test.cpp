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
    [[nodiscard]] mesh_data_ptr_type open_box_mesh() {
        std::vector<vector3_type> vertices;
        std::vector<std::uint32_t> indices;

        const auto quad = [&](const vector3_type &a, const vector3_type &b,
            const vector3_type &c, const vector3_type &d) {
            const auto base = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {a, b, c, d});
            indices.insert(indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
        };

        quad({-1.5f, -1, -1.5f}, {1.5f, -1, -1.5f}, {1.5f, -1, 1.5f}, {-1.5f, -1, 1.5f});
        quad({-1.5f, -1, -1.5f}, {-1.5f, 1, -1.5f}, {1.5f, 1, -1.5f}, {1.5f, -1, -1.5f});
        quad({-1.5f, -1, -1.5f}, {-1.5f, -1, 1.5f}, {-1.5f, 1, 1.5f}, {-1.5f, 1, -1.5f});
        quad({1.5f, -1, -1.5f}, {1.5f, 1, -1.5f}, {1.5f, 1, 1.5f}, {1.5f, -1, 1.5f});

        return make_mesh_data(std::move(vertices), std::move(indices));
    }

    [[nodiscard]] floating_point_type worst_step_when_settled(
        const vector3_type &aDrive, const floating_point_type aSpin,
        const response_handler &aHandler) {
        fixture f;

        const auto pMesh = f.scene->make_static_mesh_collider(matrix4x4_type::identity,
            open_box_mesh());

        const auto pCapsule = f.scene->make_capsule_collider(aHandler);
        pCapsule->set_position({0, 0.2f, 0});

        std::vector<vector3_type> path;

        for (int frame = 0; frame < 400; ++frame) {
            pCapsule->add_velocity(aDrive);

            if (aSpin != 0) {
                quaternion_type turn;
                turn.set_from_euler({0, 0, aSpin});
                pCapsule->add_rotation(turn);
            }

            f.update(1.0f / 60.0f);
            path.push_back(pCapsule->transform().translation());
        }

        floating_point_type worst = 0;
        for (std::size_t i = path.size() - 100; i + 1 < path.size(); ++i)
            worst = std::max(worst, (path[i + 1] - path[i]).length());

        static_cast<void>(pMesh);
        return worst;
    }
}

namespace {
    [[nodiscard]] mesh_data_ptr_type descending_valley(const floating_point_type aHalfWidth,
        const floating_point_type aWallHeight) {
        std::vector<vector3_type> vertices;
        std::vector<std::uint32_t> indices;

        const auto quad = [&](const vector3_type &a, const vector3_type &b,
            const vector3_type &c, const vector3_type &d) {
            const auto base = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {a, b, c, d});
            indices.insert(indices.end(), {base, base + 1u, base + 2u, base, base + 2u, base + 3u});
        };

        for (int i = 0; i < 40; ++i) {
            const auto z0 = static_cast<floating_point_type>(i);
            const auto z1 = static_cast<floating_point_type>(i + 1);
            const auto y0 = -0.25f * z0;
            const auto y1 = -0.25f * z1;

            quad({-aHalfWidth, y0 + aWallHeight, z0}, {0, y0, z0}, {0, y1, z1},
                {-aHalfWidth, y1 + aWallHeight, z1});
            quad({0, y0, z0}, {aHalfWidth, y0 + aWallHeight, z0},
                {aHalfWidth, y1 + aWallHeight, z1}, {0, y1, z1});
        }

        return make_mesh_data(std::move(vertices), std::move(indices));
    }

    [[nodiscard]] floating_point_type valley_travel(const response_handler &aHandler) {
        fixture f;

        const auto pValley = f.scene->make_static_mesh_collider(matrix4x4_type::identity,
            descending_valley(1.0f, 4.0f));   

        const auto pBody = f.scene->make_sphere_collider(aHandler);
        pBody->set_position({0, 1.0f, 2.0f});

        for (int frame = 0; frame < 400; ++frame) {
            pBody->add_velocity({0, -6.0f, 0});
            f.update(1.0f / 60.0f);
        }

        static_cast<void>(pValley);
        return pBody->transform().translation().z - 2.0f;
    }
}

namespace {
    vector3_type slide(const vector3_type &aVelocity,
        const contact_context &aContact, const bool aPreserveSpeed) {
        const auto &aCollisionNormal = aContact.normal;
        const auto &aClipPlanes = aContact.clip_planes;
        const auto slideThreshold = aContact.min_swept_speed;

        const auto &velocity = aVelocity;
        const auto vn = velocity.dot_product(aCollisionNormal);

        if (vn >= -slideThreshold) return velocity;

        auto slideVelocity = velocity - aCollisionNormal * vn;

        for (std::size_t first = 0; first + 1 < aClipPlanes.size(); ++first) {
            const auto &plane = aClipPlanes[first];

            if (slideVelocity.dot_product(plane) >= 0) continue;

            const auto facing = plane.dot_product(aCollisionNormal);

            if (facing <= -aContact.clip_plane_parallel_cosine) return vector3_type::zero;

            if (facing >= aContact.clip_plane_parallel_cosine) continue;

            auto along = plane.cross_product(aCollisionNormal);
            const auto length = along.length();

            along = along * (1.0f / length);
            slideVelocity = along * slideVelocity.dot_product(along);

            for (std::size_t other = 0; other + 1 < aClipPlanes.size(); ++other)
                if (other != first && slideVelocity.dot_product(aClipPlanes[other]) < 0)
                    return vector3_type::zero;

            break;
        }

        if (!aPreserveSpeed) return slideVelocity;

        return slideVelocity.normal() * velocity.length();
    }

    [[nodiscard]] vector3_type ask_slide(const collider &aSubject,
        const vector3_type &aVelocity, const vector3_type &aContactNormal,
        const clip_planes_type &aClipPlanes) {
        const scene_policy policy;
        const overlap result;

        const contact_context context{aSubject, result, aContactNormal, 1.0f / 60.0f, aClipPlanes, 0,
            policy.MIN_SWEPT_SPEED, policy.NORMALIZATION_THRESHOLD, policy.CLIP_PLANE_PARALLEL_COSINE};

        return slide(aVelocity, context, true);
    }
}

namespace {
    [[nodiscard]] response_handler ride_platforms() {
        return [](collider &aThis, const contact_context &aContact) {
            const auto velocity = aThis.velocity();
            auto delta = ::slide(velocity, aContact, false) - velocity;

            if (aContact.normal.y > 0.5f) {
                const auto arm = aContact.result.contact_point
                    - aContact.other.transform().translation();
                const auto omega = aContact.other.angular_velocity();

                const vector3_type spin{
                    omega.y * arm.z - omega.z * arm.y,
                    omega.z * arm.x - omega.x * arm.z,
                    omega.x * arm.y - omega.y * arm.x};

                delta += aContact.other.velocity() + spin;
            }

            return delta;
        };
    }
}

TEST_CASE("gdk::response_handler can slide along a crease rather than wedging in it",
    "[gdk::collision]")
{
    const auto wedging = valley_travel([](collider &aThis, const contact_context &aContact) {
        const auto velocity = aThis.velocity();
        const auto into = velocity.dot_product(aContact.normal);
        if (into >= 0) return vector3_type::zero;
        return (velocity - aContact.normal * into) - velocity;
    });

    REQUIRE(wedging < 0.5f);
    REQUIRE(valley_travel(response_handlers::slide_preserving_speed()) > 5.0f);
}

TEST_CASE("gdk::scene settles a capsule driven into a mesh", "[gdk::collision]")
{
    SECTION("against one surface, turning")
    {
        REQUIRE(worst_step_when_settled({0, -4, 0}, 0.02f, response_handlers::slide_projecting()) < 0.03f);
    }

    SECTION("into a three-surface corner, not turning")
    {
        REQUIRE(worst_step_when_settled({-4, -4, -4}, 0.0f, response_handlers::slide_projecting())
            == Approx(0.0f).margin(1e-5f));
    }

    SECTION("into a three-surface corner, turning")
    {
        REQUIRE(worst_step_when_settled({-4, -4, -4}, 0.02f, response_handlers::slide_projecting()) < 0.03f);
    }
}

TEST_CASE("gdk::collisions::response_handlers collide_and_slide keeps its speed in a corner",
    "[gdk::collision]")
{
    const auto rescaling = worst_step_when_settled({-4, -4, -4}, 0.02f,
        response_handlers::slide_preserving_speed());
    const auto projecting = worst_step_when_settled({-4, -4, -4}, 0.02f, response_handlers::slide_projecting());

    REQUIRE(projecting < 0.03f);
    REQUIRE(rescaling > projecting * 4);
}

TEST_CASE("gdk::scene a response handler is given the scene's own tolerances",
    "[gdk::collision]")
{
    fixture f;

    const scene_policy policy;   

    struct { bool called; floating_point_type minSweptSpeed;
        floating_point_type normalizationThreshold;
        std::size_t clipPlaneCount; vector3_type lastPlane;
        vector3_type normal; delta_time_type remainingTime; } seen{};

    const auto pGround = f.scene->make_static_plane_collider(matrix4x4_type());

    const auto pBall = f.scene->make_sphere_collider(
        [&seen](collider &, const contact_context &aContact) {
            seen.called = true;
            seen.minSweptSpeed = aContact.min_swept_speed;
            seen.normalizationThreshold = aContact.normalization_threshold;
            seen.clipPlaneCount = aContact.clip_planes.size();
            if (!aContact.clip_planes.empty()) seen.lastPlane = aContact.clip_planes.back();
            seen.normal = aContact.normal;
            seen.remainingTime = aContact.remaining_time;
            return vector3_type::zero;
        });
    pBall->set_position({0, 1.0f, 0});

    for (int frame = 0; frame < 30 && !seen.called; ++frame) {
        pBall->add_velocity({0, -3, 0});
        f.update(1.0f / 60.0f);
    }

    REQUIRE(seen.called);                                    
    REQUIRE(seen.minSweptSpeed == Approx(policy.MIN_SWEPT_SPEED));
    REQUIRE(seen.normalizationThreshold == Approx(policy.NORMALIZATION_THRESHOLD));
    REQUIRE(seen.clipPlaneCount >= 1);
    REQUIRE(seen.lastPlane.x == Approx(seen.normal.x).margin(1e-5f));
    REQUIRE(seen.lastPlane.y == Approx(seen.normal.y).margin(1e-5f));
    REQUIRE(seen.lastPlane.z == Approx(seen.normal.z).margin(1e-5f));
    REQUIRE(seen.normal.y == Approx(1.0f).margin(1e-4f));
    REQUIRE(seen.remainingTime >= 0.0f);
    REQUIRE(seen.remainingTime <= Approx(1.0f / 60.0f).margin(1e-6f));
}

TEST_CASE("gdk::collisions::response_handlers collide_and_slide tells a vice from a flat surface",
    "[gdk::collision]")
{
    fixture f;
    const auto pSubject = f.scene->make_sphere_collider();

    const vector3_type contactNormal{0, 1, 0};

    constexpr float TILT = 5e-4f;

    SECTION("two surfaces facing the same way are one surface, and it slides")
    {
        pSubject->add_velocity({0, -1, -1});
        const auto before = pSubject->velocity();

        vector3_type nearlySame{0, 1, TILT};
        nearlySame = nearlySame.normal();

        const auto after = ask_slide(*pSubject, before, contactNormal, {nearlySame, contactNormal});

        REQUIRE(after.length() > 1e-4f);
        REQUIRE(after.y == Approx(0.0f).margin(1e-3f));
        REQUIRE(after.length() == Approx(before.length()).margin(1e-3f));
    }

    SECTION("two surfaces facing each other are a vice, and it stops")
    {
        pSubject->set_velocity({0, -1, -1});
        const auto before = pSubject->velocity();

        vector3_type opposed{0, -1, TILT};
        opposed = opposed.normal();

        const auto after = ask_slide(*pSubject, before, contactNormal, {opposed, contactNormal});

        REQUIRE(after.length() < 1e-4f);
    }
}

TEST_CASE("gdk::scene the stock response handlers are the two velocity-ownership pairings",
    "[gdk::collision]")
{
    const auto drive = [](const response_handler &aHandler) {
        fixture f;
        const auto pWall = add_static_box_at_origin(f);

        const auto pSphere = f.scene->make_sphere_collider(aHandler);
        pSphere->set_position({-1.2f, 0, 0});

        for (int frame = 0; frame < 30; ++frame) {
            pSphere->add_velocity({1.0f, 0.25f, 0});
            f.update(1.0f / 60.0f);
        }

        REQUIRE(pWall->half_extents().x == Approx(0.5f));   
        return pSphere->transform().translation();
    };

    SECTION("the three give three different answers to the same contact")
    {
        const auto stopped = drive(response_handlers::null_opt);
        const auto projected = drive(response_handlers::slide_projecting());
        const auto preserved = drive(response_handlers::slide_preserving_speed());

        REQUIRE(stopped.x <= Approx(-1.0f).margin(1e-2f));
        REQUIRE(projected.x <= Approx(-1.0f).margin(1e-2f));
        REQUIRE(preserved.x <= Approx(-1.0f).margin(1e-2f));
        REQUIRE(projected.y > stopped.y + 0.02f);
        REQUIRE(preserved.y > projected.y * 2);
    }

    SECTION("the factories default to the speed-preserving slide")
    {
        const auto defaulted = drive(response_handlers::slide_preserving_speed());

        fixture f;
        const auto pWall = add_static_box_at_origin(f);
        const auto pSphere = f.scene->make_sphere_collider();   
        pSphere->set_position({-1.2f, 0, 0});

        for (int frame = 0; frame < 30; ++frame) {
            pSphere->add_velocity({1.0f, 0.25f, 0});
            f.update(1.0f / 60.0f);
        }

        REQUIRE(pSphere->transform().translation().y == Approx(defaulted.y).margin(1e-4f));
        REQUIRE(pWall->half_extents().x == Approx(0.5f));
    }
}

TEST_CASE("gdk::scene a handler can express behaviour the stock ones do not",
    "[gdk::collision]")
{
    const auto drive = [](const response_handler &aHandler) {
        fixture f;
        const auto pWall = add_static_box_at_origin(f);

        const auto pSphere = f.scene->make_sphere_collider(aHandler);
        pSphere->set_position({-1.2f, 0, 0});

        auto mostLeftward = floating_point_type{0};

        for (int frame = 0; frame < 20; ++frame) {
            pSphere->add_velocity({1.0f, 0, 0});
            f.update(1.0f / 60.0f);
            mostLeftward = std::min(mostLeftward, pSphere->resolved_velocity().x);
        }

        REQUIRE(pWall->half_extents().x == Approx(0.5f));   
        return mostLeftward;
    };

    SECTION("a bounce, which none of the three modes can do")
    {
        const response_handler bounce = [](collider &aThis, const contact_context &aContact) {
            const auto velocity = aThis.velocity();
            const auto into = velocity.dot_product(aContact.normal);
            if (into >= 0) return vector3_type::zero;

            const auto reflected = velocity - aContact.normal * (2.0f * into);
            return reflected - velocity;   
        };

        const auto slid = drive(response_handlers::slide_preserving_speed());
        const auto bounced = drive(bounce);

        REQUIRE(slid == Approx(0.0f).margin(1e-3f));
        REQUIRE(bounced == Approx(-1.0f).margin(1e-2f));
    }

    SECTION("a handler can build on the library's slide instead of reimplementing it")
    {
        const response_handler halfSpeedSlide = [](collider &aThis,
            const contact_context &aContact) {
            const auto velocity = aThis.velocity();
            const auto slid = ::slide(velocity, aContact, true);

            return slid * 0.5f - velocity;   
        };

        fixture f;
        const auto pWall = add_static_box_at_origin(f);
        const auto pSphere = f.scene->make_sphere_collider(halfSpeedSlide);
        pSphere->set_position({-1.2f, 0, 0});

        for (int frame = 0; frame < 20; ++frame) {
            pSphere->add_velocity({1.0f, 0.5f, 0});
            f.update(1.0f / 60.0f);
        }

        const auto position = pSphere->transform().translation();

        REQUIRE(position.x <= Approx(-1.0f).margin(1e-2f));   
        REQUIRE(position.y > 0.02f);                          
        REQUIRE(pWall->half_extents().x == Approx(0.5f));
    }
}

TEST_CASE("gdk::scene a handler can ride a moving platform", "[gdk::collision]")
{
    SECTION("a translating platform, which worked before angular velocity existed")
    {
        fixture f;
        const auto pPlatform = f.scene->make_aabb_collider();
        pPlatform->set_kinematic(true);
        pPlatform->set_position({0, 0, 0});

        const auto pRider = f.scene->make_sphere_collider(ride_platforms());
        pRider->set_position({0, 1.0f, 0});
        const auto start = pRider->transform().translation();

        for (int frame = 0; frame < 120; ++frame) {
            pPlatform->add_velocity({2.0f, 0, 0});
            pRider->add_velocity({0, -1.0f, 0});
            f.update(1.0f / 60.0f);
        }

        const auto carried = pRider->transform().translation().x - start.x;
        REQUIRE(pPlatform->transform().translation().x == Approx(4.0f).margin(1e-3f));
        REQUIRE(carried == Approx(4.0f).margin(0.05f));           
        REQUIRE(pRider->transform().translation().y == Approx(1.0f).margin(1e-2f));   
    }

    SECTION("a rotating platform, which needs the angular velocity")
    {
        fixture f;

        const auto pPlatform = f.scene->make_obb_collider();
        pPlatform->set_half_extents({4.0f, 0.25f, 1.0f});
        pPlatform->set_kinematic(true);
        pPlatform->set_position({0, 0, 0});

        const auto pRider = f.scene->make_sphere_collider(ride_platforms());
        pRider->set_position({3.0f, 0.75f, 0});
        const auto start = pRider->transform().translation();

        constexpr auto RATE = 1.0f;   
        for (int frame = 0; frame < 60; ++frame) {
            pPlatform->add_angular_velocity({0, RATE, 0});
            pRider->add_velocity({0, -1.0f, 0});
            f.update(1.0f / 60.0f);
        }

        const auto end = pRider->transform().translation();

        REQUIRE(std::abs(end.z) > 1.5f);                       
        REQUIRE(end.x < start.x - 0.8f);                       
        REQUIRE(end.y == Approx(0.75f).margin(0.2f));          

        const auto radius = std::sqrt(end.x * end.x + end.z * end.z);
        REQUIRE(radius == Approx(3.0f).margin(0.6f));
    }
}

TEST_CASE("gdk::response_handlers keep a slide out of every surface already met", "[gdk::collision]")
{
    fixture f;

    const auto pBody = f.scene->make_sphere_collider();
    pBody->set_velocity({0, -1, 0});

    const auto first = vector3_type{1, 1, 0}.normal();
    const auto current = vector3_type{-0.5f, 1, 0}.normal();
    const overlap hit{};

    const auto slid = [&](const clip_planes_type &aPlanes, const response_handler &aHandler) {
        const contact_context context{*pBody, hit, current, 1.0f, aPlanes, 1, 1e-3f, 1e-6f, 0.9999995f};

        return pBody->velocity() + aHandler(*pBody, context);
    };

    for (const auto &planes : {clip_planes_type{first}, clip_planes_type{first, current}, clip_planes_type{current, first}})
        for (const auto *pHandler : {&response_handlers::slide_projecting(), &response_handlers::slide_preserving_speed()}) {
            const auto velocity = slid(planes, *pHandler);

            REQUIRE(velocity.dot_product(first) >= -1e-4f);
            REQUIRE(velocity.dot_product(current) >= -1e-4f);
        }
}
