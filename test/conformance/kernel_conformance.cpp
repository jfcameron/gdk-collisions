// © Joseph Cameron - MIT License

#include <jfc/catch.hpp>

#include "conformance.h"

#include <gdk/collisions/aabb_collider.h>
#include <gdk/collisions/kernels/scene_kernels.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/sphere_collider.h>

#include <algorithm>
#include <cstdint>
#include <span>
#include <vector>

using namespace gdk::collisions;
using gdk::collisions::conformance::finds_contacts;
using gdk::collisions::conformance::implementation;
using gdk::collisions::conformance::make_kernels;

namespace {
    constexpr delta_time_type STEP = 1.0f / 60.0f;

    struct body final {
        collider_id_type id = 0;
        vector3_type position = vector3_type::zero;
        vector3_type velocity = vector3_type::zero;
        shape_kind kind = shape_kind::sphere;
        vector3_type size = {0.5f, 0, 0};
        bool trigger = false;
        body_motion motion = body_motion::dynamic;
        collision_layers_type on = DEFAULT_COLLISION_LAYER;
        collision_layers_type collides_with = ALL_COLLISION_LAYERS;
    };

    std::uint32_t add(body_table &aTable, const body &aBody) {
        const auto half = aBody.kind == shape_kind::sphere
            ? vector3_type{aBody.size.x, aBody.size.x, aBody.size.x} : aBody.size;

        aTable.id.push_back(aBody.id);
        aTable.position.push_back(aBody.position);
        aTable.velocity.push_back(aBody.velocity);
        aTable.rotation.push_back(quaternion_type::identity);
        aTable.extent.push_back(bounds{aBody.position - half, aBody.position + half});
        aTable.on.push_back(aBody.on);
        aTable.collides_with.push_back(aBody.collides_with);
        aTable.motion.push_back(aBody.motion);
        aTable.trigger.push_back(aBody.trigger ? 1 : 0);
        aTable.live.push_back(1);
        aTable.first_part.push_back(static_cast<std::uint32_t>(aTable.parts.size()));
        aTable.part_count.push_back(1);
        aTable.parts.push_back(part_record{aBody.kind, aBody.size});

        return static_cast<std::uint32_t>(aTable.size() - 1);
    }

    struct world final {
        body_table statics;
        body_table movers;
        scene_kernels_ptr_type kernels = make_kernels();

        [[nodiscard]] scene_tables tables() const { return scene_tables{statics, movers, contact_rules{}}; }

        void ready(const delta_time_type aDeltaTime = STEP) {
            std::vector<std::uint32_t> rows(statics.size());
            for (std::uint32_t row = 0; row < rows.size(); ++row) rows[row] = row;

            kernels->statics_changed(tables(), rows, {});
            kernels->movers_changed(tables(), aDeltaTime);
        }

        [[nodiscard]] std::vector<candidate_pair> pairs_of(std::vector<std::uint32_t> aSubjects,
            const delta_time_type aTime = STEP, const bool aKinematicCandidates = true) {
            const std::vector<delta_time_type> times(aSubjects.size(), aTime);

            std::vector<candidate_pair> pairs;
            kernels->gather_pairs(tables(), pair_request{aSubjects, times, aKinematicCandidates}, pairs);
            return pairs;
        }

        [[nodiscard]] std::vector<pair_hit> hits_of(const std::vector<pair_test> &aTests) {
            std::vector<pair_hit> hits;
            kernels->test_pairs(tables(), aTests, hits);
            return hits;
        }
    };

    [[nodiscard]] bool has_pair(const std::vector<candidate_pair> &aPairs, const std::uint32_t aSubject,
        const body_ref &aOther) {
        return std::any_of(aPairs.begin(), aPairs.end(), [&](const candidate_pair &aPair) {
            return aPair.subject == aSubject && aPair.other == aOther;
        });
    }

    [[nodiscard]] bool skipped() {
        if (finds_contacts()) return false;

        WARN(implementation() << " finds nothing by design");
        return true;
    }
}

TEST_CASE("conformance, gather_pairs finds every pair whose bodies could meet", "[conformance]") {
    INFO("implementation " << implementation());
    if (skipped()) return;

    world w;
    const auto subject = add(w.movers, {.id = 1});
    const auto near = add(w.movers, {.id = 2, .position = {0.8f, 0, 0}});
    const auto beneath = add(w.statics, {.id = 3, .position = {0, -0.9f, 0}, .motion = body_motion::fixed});
    const auto ahead = add(w.movers, {.id = 4, .position = {3, 0, 0}});
    w.movers.velocity[subject] = {120, 0, 0};
    w.movers.extent[subject].max.x += 120 * STEP;
    w.ready();

    const auto pairs = w.pairs_of({subject});

    CHECK(has_pair(pairs, subject, {near, false}));
    CHECK(has_pair(pairs, subject, {beneath, true}));
    CHECK(has_pair(pairs, subject, {ahead, false}));
    CHECK_FALSE(has_pair(pairs, subject, {subject, false}));
}

TEST_CASE("conformance, gather_pairs leaves out bodies whose layers do not meet, and kinematics when asked",
    "[conformance]") {
    INFO("implementation " << implementation());

    world w;
    const auto subject = add(w.movers, {.id = 1, .collides_with = 1});
    const auto elsewhere = add(w.movers, {.id = 2, .position = {0.5f, 0, 0}, .on = 2});
    const auto deaf = add(w.movers, {.id = 3, .position = {-0.5f, 0, 0}, .collides_with = 2});
    const auto lift = add(w.movers, {.id = 4, .position = {0, 0.5f, 0}, .motion = body_motion::kinematic});
    w.ready();

    const auto pairs = w.pairs_of({subject}, STEP, false);

    CHECK_FALSE(has_pair(pairs, subject, {elsewhere, false}));
    CHECK_FALSE(has_pair(pairs, subject, {deaf, false}));
    CHECK_FALSE(has_pair(pairs, subject, {lift, false}));
    CHECK_FALSE(has_pair(pairs, subject, {subject, false}));

    if (finds_contacts()) CHECK(has_pair(w.pairs_of({subject}), subject, {lift, false}));
}

TEST_CASE("conformance, gather_pairs orders pairs by subject, then colliders, statics, and id", "[conformance]") {
    INFO("implementation " << implementation());

    world w;
    const auto first = add(w.movers, {.id = 10});
    const auto second = add(w.movers, {.id = 11, .position = {0, 3, 0}});
    add(w.movers, {.id = 5, .position = {0.4f, 0, 0}, .trigger = true});
    add(w.movers, {.id = 3, .position = {-0.4f, 0, 0}});
    add(w.movers, {.id = 2, .position = {0, 0.4f, 0}});
    add(w.statics, {.id = 9, .position = {0, -0.4f, 0}, .motion = body_motion::fixed});
    add(w.statics, {.id = 1, .position = {0, 0, 0.4f}, .trigger = true, .motion = body_motion::fixed});
    add(w.statics, {.id = 7, .position = {0, 3.4f, 0}, .motion = body_motion::fixed});
    w.ready();

    const auto pairs = w.pairs_of({second, first});

    const auto table_of = [&](const body_ref &aBody) -> const body_table & { return aBody.fixed ? w.statics : w.movers; };

    const auto key = [&](const candidate_pair &aPair) {
        const auto &table = table_of(aPair.other);
        return std::tuple{aPair.subject == first, table.trigger[aPair.other.row] != 0, !aPair.other.fixed,
            table.id[aPair.other.row]};
    };

    CHECK(std::is_sorted(pairs.begin(), pairs.end(),
        [&](const candidate_pair &aFirst, const candidate_pair &aSecond) { return key(aFirst) < key(aSecond); }));

    if (finds_contacts()) CHECK(pairs.size() >= 6);
}

TEST_CASE("conformance, test_pairs reports a closing contact, in test order", "[conformance]") {
    INFO("implementation " << implementation());
    if (skipped()) return;

    world w;
    const auto subject = add(w.movers, {.id = 1, .velocity = {10, 0, 0}});
    const auto wall = add(w.movers, {.id = 2, .position = {1.5f, 0, 0}});
    const auto far = add(w.movers, {.id = 3, .position = {50, 0, 0}});
    w.ready(1);

    const auto hits = w.hits_of({{subject, {far, false}, 1, true}, {subject, {wall, false}, 1, true},
        {subject, {wall, false}, 0.01f, true}, {subject, {wall, false}, 1, true}});

    REQUIRE(hits.size() == 2);
    CHECK(hits[0].test == 1);
    CHECK(hits[1].test == 3);
    CHECK(hits[0].hit.entry_time == Approx(0.05f).margin(1e-3f));
    CHECK(hits[0].hit.contact_normal.x == Approx(-1).margin(1e-3f));
}

TEST_CASE("conformance, test_pairs holds a standing other still and ignores bodies moving apart", "[conformance]") {
    INFO("implementation " << implementation());
    if (skipped()) return;

    world w;
    const auto subject = add(w.movers, {.id = 1, .velocity = {10, 0, 0}});
    const auto fleeing = add(w.movers, {.id = 2, .position = {1.5f, 0, 0}, .velocity = {20, 0, 0}});
    const auto parting = add(w.movers, {.id = 3, .position = {-1, 0, 0}, .velocity = {-5, 0, 0}});
    w.ready(1);

    CHECK(w.hits_of({{subject, {fleeing, false}, 1, true}}).empty());
    CHECK(w.hits_of({{subject, {fleeing, false}, 1, false}}).size() == 1);
    CHECK(w.hits_of({{subject, {parting, false}, 1, true}}).empty());
}

TEST_CASE("conformance, statics come and go", "[conformance]") {
    INFO("implementation " << implementation());
    if (skipped()) return;

    world w;
    const auto subject = add(w.movers, {.id = 1});
    const auto floor = add(w.statics, {.id = 2, .position = {0, -0.9f, 0}, .motion = body_motion::fixed});
    w.ready();

    REQUIRE(has_pair(w.pairs_of({subject}), subject, {floor, true}));

    w.statics.live[floor] = 0;
    const std::uint32_t removed[1] = {floor};
    w.kernels->statics_changed(w.tables(), {}, removed);

    CHECK_FALSE(has_pair(w.pairs_of({subject}), subject, {floor, true}));

    w.statics.live[floor] = 1;
    w.statics.position[floor] = {0, -0.9f, 4};
    w.statics.extent[floor] = bounds{{-0.5f, -1.4f, 3.5f}, {0.5f, -0.4f, 4.5f}};
    w.kernels->statics_changed(w.tables(), removed, {});

    CHECK_FALSE(has_pair(w.pairs_of({subject}), subject, {floor, true}));
}

TEST_CASE("conformance, cast_ray strikes the nearest collider, never a trigger", "[conformance]") {
    INFO("implementation " << implementation());

    world w;
    add(w.statics, {.id = 1, .position = {3, 0, 0}, .trigger = true, .motion = body_motion::fixed});
    const auto wall = add(w.statics, {.id = 2, .position = {5, 0, 0}, .motion = body_motion::fixed});
    const auto runner = add(w.movers, {.id = 3, .position = {8, 0, 0}});
    w.ready();

    ray_query query;
    query.direction = {1, 0, 0};
    query.max_distance = 100;

    const auto hit = w.kernels->cast_ray(w.tables(), query);

    if (!finds_contacts()) {
        CHECK_FALSE(hit);
        return;
    }

    REQUIRE(hit);
    CHECK(hit->body == body_ref{wall, true});
    CHECK(hit->distance == Approx(4.5f).margin(1e-3f));
    CHECK(hit->normal.x == Approx(-1).margin(1e-3f));

    const collider_id_type skipping[1] = {2};
    query.ignore = skipping;

    const auto past = w.kernels->cast_ray(w.tables(), query);
    REQUIRE(past);
    CHECK(past->body == body_ref{runner, false});

    query.statics_only = true;
    CHECK_FALSE(w.kernels->cast_ray(w.tables(), query));

    query.statics_only = false;
    query.ignore = {};
    query.layers = 2;
    CHECK_FALSE(w.kernels->cast_ray(w.tables(), query));
}

TEST_CASE("conformance, meets_statics answers for upright cylinders and parallelepipeds", "[conformance]") {
    INFO("implementation " << implementation());

    world w;
    add(w.statics, {.id = 1, .kind = shape_kind::aabb, .size = {1, 1, 1}, .motion = body_motion::fixed});
    add(w.statics, {.id = 2, .position = {10, 0, 0}, .trigger = true, .motion = body_motion::fixed});
    w.ready();

    const upright_cylinder inside{{0, -0.5f, 0}, 1, 0.5f};
    const upright_cylinder onTrigger{{10, -0.5f, 0}, 1, 0.5f};
    const parallelepiped box{{0.5f, 0.5f, 0.5f}, {vector3_type{1, 0, 0}, vector3_type{0, 1, 0}, vector3_type{0, 0, 1}}};
    const parallelepiped far{{20, 0, 0}, {vector3_type{1, 0, 0}, vector3_type{0, 1, 0}, vector3_type{0, 0, 1}}};

    const collider_id_type none[1] = {999};
    const collider_id_type cube[1] = {1};

    CHECK_FALSE(w.kernels->meets_statics(w.tables(), onTrigger, none));
    CHECK_FALSE(w.kernels->meets_statics(w.tables(), far, none));
    CHECK_FALSE(w.kernels->meets_statics(w.tables(), inside, cube));

    if (!finds_contacts()) return;

    CHECK(w.kernels->meets_statics(w.tables(), inside, none));
    CHECK(w.kernels->meets_statics(w.tables(), box, none));
}

TEST_CASE("conformance, a scene on these kernels holds a falling ball on the ground", "[conformance]") {
    INFO("implementation " << implementation());

    std::size_t entered = 0;

    const auto pScene = make_scene([&entered](const collision_event &aEvent) {
        if (aEvent.type == event_type::enter) ++entered;
    }, nullptr, scene_policy{}, make_kernels());

    const auto pGround = pScene->make_static_aabb_collider(matrix4x4_type::identity, {5, 0.5f, 5});

    const auto pBall = pScene->make_sphere_collider();
    pBall->set_position({0, 2, 0});

    for (int frame = 0; frame < 120; ++frame) {
        pBall->add_velocity({0, -6, 0});
        pScene->update(STEP);
        pScene->process_events();
    }

    if (!finds_contacts()) {
        CHECK(pBall->transform().translation().y < -5);
        CHECK(entered == 0);
        return;
    }

    CHECK(pBall->transform().translation().y == Approx(1.0f).margin(1e-2f));
    CHECK(entered == 1);
}
