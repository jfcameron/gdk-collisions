// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_collision_profile.h>
#include <gdk/collisions/impl_scene.h>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace gdk::collisions;

namespace {
    [[nodiscard]] std::optional<vector3_type> resolve_overlap(const impl_collider_ptr_type &aSubject,
        const impl_collider_ptr_type &aOther, const overlap &aOverlap) {
        const auto normal = aOverlap.contact_normal;
        if (normal.is_effectively_zero()) return std::nullopt;

        const auto subjectShare = aSubject->displacement_share();
        const auto otherShare = aOther->displacement_share();
        const auto totalShare = subjectShare + otherShare;

        if (totalShare > std::numeric_limits<floating_point_type>::epsilon() && aOverlap.penetration > 0.0f) {
            const auto translation = normal * aOverlap.penetration;

            if (subjectShare > 0.0f)
                aSubject->set_position(aSubject->position() + translation * (subjectShare / totalShare));

            if (otherShare > 0.0f)
                aOther->set_position(aOther->position() - translation * (otherShare / totalShare));
        }

        return normal;
    }
}

void impl_scene::body_advancement::release() {
    blocking_hit.reset();
    trigger_hits.clear();
    resting_contacts.clear();
    clip_planes.clear();
}

void impl_scene::update_step(const delta_time_type aDeltaTime) {
    gather_movers(aDeltaTime);

    m_Kernels->movers_changed(tables(), aDeltaTime);

    kinematic_pass(aDeltaTime);
    advance_dynamics(aDeltaTime);
    sweep_triggers(aDeltaTime);
    finish_step();
}

void impl_scene::gather_movers(const delta_time_type aDeltaTime) {
    clear_table(m_Movers);
    m_MoverOwners.clear();

    std::vector<impl_collider_ptr_type> kinematics;

    std::erase_if(m_DynamicColliders, [&](const impl_collider_weak_ptr_type &aWeak) {
        auto pBody = aWeak.lock();
        if (!pBody) return true;

        (pBody->is_kinematic() ? kinematics : m_MoverOwners).push_back(std::move(pBody));
        return false;
    });

    m_DynamicCount = static_cast<std::uint32_t>(m_MoverOwners.size());
    m_KinematicCount = static_cast<std::uint32_t>(kinematics.size());
    m_MoverOwners.insert(m_MoverOwners.end(), kinematics.begin(), kinematics.end());

    std::erase_if(m_DynamicTriggers, [&](const impl_collider_weak_ptr_type &aWeak) {
        auto pBody = aWeak.lock();
        if (!pBody) return true;

        m_MoverOwners.push_back(std::move(pBody));
        return false;
    });

    for (auto &pBody : m_MoverOwners) pBody->integrate_angular_velocity(aDeltaTime);

    const auto firstTrigger = m_DynamicCount + m_KinematicCount;

    for (std::uint32_t row = 0; row < m_MoverOwners.size(); ++row) {
        const auto &pBody = *m_MoverOwners[row];
        const auto trigger = row >= firstTrigger;
        const auto motion = (trigger ? pBody.is_kinematic() : row >= m_DynamicCount)
            ? body_motion::kinematic : body_motion::dynamic;

        append_row(m_Movers, pBody, motion, trigger);
    }

    m_Dirty.assign(m_MoverOwners.size(), 0);
    m_DirtyRows.clear();
}

void impl_scene::refresh_row(const std::uint32_t aRow) {
    const auto &pBody = *m_MoverOwners[aRow];

    m_Movers.position[aRow] = pBody.position();
    m_Movers.velocity[aRow] = pBody.velocity();
    m_Movers.rotation[aRow] = pBody.rotation();
    m_Movers.extent[aRow] = pBody.world_bounds();
}

void impl_scene::mark_dirty(const body_ref &aBody) {
    if (aBody.fixed || m_Dirty[aBody.row]) return;

    m_Dirty[aBody.row] = 1;
    m_DirtyRows.push_back(aBody.row);
}

void impl_scene::refresh_dirty() {
    for (const auto row : m_DirtyRows) {
        refresh_row(row);
        m_Dirty[row] = 0;
    }

    m_DirtyRows.clear();
}

void impl_scene::gather_and_test(const pair_request &aRequest, const bool aOthersMove) {
    m_Pairs.clear();
    m_Tests.clear();
    m_Hits.clear();

    m_Kernels->gather_pairs(tables(), aRequest, m_Pairs);

    std::size_t subject = 0;

    for (const auto &pair : m_Pairs) {
        while (aRequest.subjects[subject] != pair.subject) ++subject;

        const auto otherMoves = aOthersMove
            && (pair.other.fixed || m_Movers.motion[pair.other.row] != body_motion::kinematic);

        m_Tests.push_back(pair_test{pair.subject, pair.other, aRequest.times[subject], otherMoves});
    }

    m_Statistics.pairs_tested += m_Tests.size();

    if (!m_Tests.empty()) m_Kernels->test_pairs(tables(), m_Tests, m_Hits);
}

void impl_scene::kinematic_pass(const delta_time_type aDeltaTime) {
    if (m_KinematicCount == 0) return;

    GDK_COLLISION_PROFILE_BEGIN(kinematic_pass_ns);

    m_Subjects.clear();
    m_Times.clear();

    for (auto row = m_DynamicCount; row < m_DynamicCount + m_KinematicCount; ++row) {
        m_Subjects.push_back(row);
        m_Times.push_back(aDeltaTime);
    }

    gather_and_test(pair_request{m_Subjects, m_Times, false}, true);

    for (const auto &hit : m_Hits) {
        const auto &test = m_Tests[hit.test];
        const auto &pKinematic = m_MoverOwners[test.subject];
        const auto pOther = owner_of(test.other);
        if (!pOther) continue;

        const auto trigger = (test.other.fixed ? m_Statics : m_Movers).trigger[test.other.row];

        if (trigger) m_Events.record_trigger(pKinematic, pOther);
        else m_Events.record_collision(pKinematic, pOther, hit.hit.contact_normal, hit.hit.contact_point,
            hit.hit.penetration);
    }

    for (auto row = m_DynamicCount; row < m_DynamicCount + m_KinematicCount; ++row) {
        const auto &pKinematic = m_MoverOwners[row];
        pKinematic->set_position(pKinematic->position() + pKinematic->velocity() * aDeltaTime);
        refresh_row(row);
    }

    GDK_COLLISION_PROFILE_END(kinematic_pass_ns);
}

void impl_scene::advance_dynamics(const delta_time_type aDeltaTime) {
    GDK_COLLISION_PROFILE_BEGIN(dynamic_loop_ns);

    m_Advancement.resize(m_DynamicCount);

    for (auto &state : m_Advancement) {
        state.remaining_time = aDeltaTime;
        state.active = aDeltaTime > 0;
        state.release();
    }

    for (unsigned short int iteration = 0; iteration < m_Policy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS; ++iteration) {
        refresh_dirty();

        m_Subjects.clear();
        m_Times.clear();

        for (std::uint32_t row = 0; row < m_DynamicCount; ++row) {
            auto &state = m_Advancement[row];
            if (!state.active) continue;

            state.earliest_blocking_toi = state.remaining_time;
            state.blocking_hit.reset();
            state.trigger_hits.clear();
            state.resting_contacts.clear();

            m_Subjects.push_back(row);
            m_Times.push_back(state.remaining_time);
        }

        if (m_Subjects.empty()) break;

        gather_and_test(pair_request{m_Subjects, m_Times, true}, true);

        for (const auto &found : m_Hits) {
            const auto &test = m_Tests[found.test];
            const auto &hit = found.hit;
            auto &state = m_Advancement[test.subject];

            const auto &otherTable = test.other.fixed ? m_Statics : m_Movers;

            if (otherTable.trigger[test.other.row]) {
                if (hit.entry_time <= state.earliest_blocking_toi + m_Policy.AXIS_ENTRY_EPSILON)
                    state.trigger_hits.push_back(test.other);

                continue;
            }

            const auto velocity = m_Movers.velocity[test.subject];
            const auto otherVelocity = test.other_moves ? otherTable.velocity[test.other.row] : vector3_type::zero;

            const bool ownClosing = velocity.dot_product(hit.contact_normal) < -m_Policy.MIN_SWEPT_SPEED;
            const bool closing = (velocity - otherVelocity).dot_product(hit.contact_normal) <= -m_Policy.MIN_SWEPT_SPEED;

            if (!ownClosing || !closing) {
                state.resting_contacts.emplace_back(hit, test.other);
                continue;
            }

            if (hit.entry_time < state.earliest_blocking_toi) {
                state.earliest_blocking_toi = hit.entry_time;
                state.blocking_hit = hit;
                state.blocking_body = test.other;
            }
        }

        resolve_and_respond(iteration);
    }

    for (auto &state : m_Advancement) state.release();

    GDK_COLLISION_PROFILE_END(dynamic_loop_ns);
}

std::optional<vector3_type> impl_scene::resolve_once(const std::uint32_t aRow, const body_ref &aOther,
    const overlap &aOverlap) {
    const auto &pSubject = m_MoverOwners[aRow];
    const auto pOther = owner_of(aOther);

    if (pOther && m_ResolvedPairs.insert(pSubject->id(), pOther->id())) {
        mark_dirty(aOther);
        return resolve_overlap(pSubject, pOther, aOverlap);
    }

    if (aOverlap.contact_normal.is_effectively_zero()) return std::nullopt;

    return aOverlap.contact_normal;
}

void impl_scene::resolve_and_respond(const unsigned short int aIteration) {
    GDK_COLLISION_PROFILE_BEGIN(resolve_respond_ns);

    m_ResolvedPairs.clear();

    for (std::uint32_t row = 0; row < m_DynamicCount; ++row) {
        auto &state = m_Advancement[row];
        if (!state.active) continue;

        const auto &pBody = m_MoverOwners[row];

        mark_dirty(body_ref{row, false});

        const bool advanced = state.earliest_blocking_toi > m_Policy.TIME_OF_IMPACT_EPSILON;
        if (advanced) {
            pBody->set_position(pBody->position() + pBody->velocity() * state.earliest_blocking_toi);
            state.remaining_time -= state.earliest_blocking_toi;
        }

        for (const auto &each : state.trigger_hits)
            if (const auto pTrigger = owner_of(each)) m_Events.record_trigger(pBody, pTrigger);

        for (const auto &[hit, other] : state.resting_contacts) {
            const auto restingNormal = resolve_once(row, other, hit);

            if (const auto pOther = owner_of(other))
                m_Events.record_collision(pBody, pOther, restingNormal ? *restingNormal : hit.contact_normal,
                    hit.contact_point, hit.penetration);
        }

        if (!state.blocking_hit) {
            state.active = false;
            continue;
        }

        const auto pBlocker = owner_of(state.blocking_body);

        if (!pBlocker) {
            state.active = false;
            continue;
        }

        auto velocityChange = vector3_type::zero;

        const auto oCollisionNormal = resolve_once(row, state.blocking_body, *state.blocking_hit);

        if (oCollisionNormal) {
            const auto alreadyKnown = std::any_of(state.clip_planes.begin(), state.clip_planes.end(),
                [&](const vector3_type &aPlane) {
                    return aPlane.dot_product(*oCollisionNormal) >= m_Policy.CLIP_PLANE_PARALLEL_COSINE;
                });

            if (!alreadyKnown) state.clip_planes.push_back(*oCollisionNormal);

            const contact_context context{*pBlocker, *state.blocking_hit, *oCollisionNormal,
                state.remaining_time, state.clip_planes, aIteration, m_Policy.MIN_SWEPT_SPEED,
                m_Policy.NORMALIZATION_THRESHOLD, m_Policy.CLIP_PLANE_PARALLEL_COSINE};

            const auto velocityBefore = pBody->velocity();

            pBody->set_velocity(velocityBefore + pBody->respond_to_collision_resolution(context));

            velocityChange = pBody->velocity() - velocityBefore;
        }

        m_Events.record_collision(pBody, pBlocker,
            oCollisionNormal ? *oCollisionNormal : state.blocking_hit->contact_normal,
            state.blocking_hit->contact_point, state.blocking_hit->penetration);

        if (state.remaining_time <= 0 || pBody->velocity().is_effectively_zero()
            || (!advanced && velocityChange.is_effectively_zero()))
            state.active = false;
    }

    GDK_COLLISION_PROFILE_END(resolve_respond_ns);
}

void impl_scene::sweep_triggers(const delta_time_type aDeltaTime) {
    GDK_COLLISION_PROFILE_BEGIN(trigger_loop_ns);

    refresh_dirty();

    for (auto row = m_DynamicCount + m_KinematicCount; row < m_MoverOwners.size(); ++row) {
        const auto &pTrigger = m_MoverOwners[row];
        auto remainingTime = aDeltaTime;

        for (unsigned short int iteration = 0;
            remainingTime > 0 && iteration < m_Policy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS; ++iteration) {
            auto earliest = remainingTime;
            m_TriggerHits.clear();

            m_Subjects.assign(1, row);
            m_Times.assign(1, remainingTime);

            gather_and_test(pair_request{m_Subjects, m_Times, true}, false);

            for (const auto &found : m_Hits) {
                const auto entry = found.hit.entry_time;
                const auto &other = m_Tests[found.test].other;

                if (entry < earliest - m_Policy.AXIS_ENTRY_EPSILON) {
                    earliest = entry;
                    m_TriggerHits.assign(1, other);
                }
                else if (std::abs(entry - earliest) <= m_Policy.AXIS_ENTRY_EPSILON) m_TriggerHits.push_back(other);
            }

            const auto advance = m_TriggerHits.empty() || earliest <= m_Policy.TIME_OF_IMPACT_EPSILON
                ? remainingTime : earliest;

            pTrigger->set_position(pTrigger->position() + pTrigger->velocity() * advance);
            remainingTime -= advance;
            refresh_row(row);

            if (m_TriggerHits.empty()) continue;

            m_Tests.clear();
            m_Hits.clear();

            for (const auto &other : m_TriggerHits) m_Tests.push_back(pair_test{row, other, 0, false});

            m_Kernels->test_pairs(tables(), m_Tests, m_Hits);

            for (const auto &found : m_Hits)
                if (found.hit.entry_time <= m_Policy.TIME_OF_IMPACT_EPSILON)
                    if (const auto pOther = owner_of(m_Tests[found.test].other)) m_Events.record_trigger(pTrigger, pOther);
        }
    }

    GDK_COLLISION_PROFILE_END(trigger_loop_ns);
}

void impl_scene::finish_step() {
    clear_table(m_Movers);
    m_MoverOwners.clear();

    ++m_Statistics.steps;
}

void impl_scene::clear_velocities() {
    const auto clear = [](const std::vector<impl_collider_weak_ptr_type> &aBodies) {
        for (const auto &weak : aBodies)
            if (const auto pBody = weak.lock()) {
                pBody->clear_velocity();
                pBody->clear_angular_velocity();
            }
    };

    clear(m_DynamicColliders);
    clear(m_DynamicTriggers);
}
