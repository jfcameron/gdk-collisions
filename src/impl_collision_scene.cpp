// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/types.h>
#include <gdk/collisions/impl_axis_aligned_box.h>
#include <gdk/collisions/impl_broadphase_grid.h>
#include <gdk/collisions/impl_capsule_collider.h>
#include <gdk/collisions/impl_compound_collider.h>
#include <gdk/collisions/impl_heightfield_collider.h>
#include <gdk/collisions/impl_mesh_collider.h>
#include <gdk/collisions/impl_plane_collider.h>
#include <gdk/collisions/impl_obb_collider.h>
#include <gdk/collisions/impl_collision_event_tracker.h>
#include <gdk/collisions/impl_collision_profile.h>
#include <gdk/collisions/impl_collision_scene.h>
#include <gdk/collisions/impl_dynamic_broadphase_grid.h>
#include <gdk/collisions/impl_narrow_phase.h>
#include <gdk/collisions/impl_sphere_collider.h>
#include <gdk/collisions/overlap.h>

#include <algorithm>
#include <limits>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace gdk::collisions;

namespace {
    /// \brief lock every weak_ptr in aContainer, dropping those that have expired.
    template <typename weak_ptr_container_type>
    auto lock_and_prune(weak_ptr_container_type &aContainer) {
        using weak_ptr_type = typename weak_ptr_container_type::value_type;
        using element_type = typename weak_ptr_type::element_type;
        using shared_ptr_container_type = std::vector<std::shared_ptr<element_type>>;

        shared_ptr_container_type result;
        result.reserve(aContainer.size());

        auto it = aContainer.begin();
        while (it != aContainer.end()) {
            if (auto sp = it->lock()) {
                result.push_back(std::move(sp));
                ++it;
            }
            else it = aContainer.erase(it);
        }
        return result;
    }

    template <typename container_type>
    inline void clear_velocity_in_collection(const container_type &aCollection) {
        for (auto &pDynamic : aCollection) {
            pDynamic->clear_velocity();
            pDynamic->clear_angular_velocity();
        }
    }

    /// \brief narrow phase, dispatched by visiting both colliders' shapes.
    [[nodiscard]] inline std::optional<overlap> overlap_between(const impl_collider_ptr_type &aSubject,
        const impl_collider_ptr_type &aOther, const delta_time_type aDeltaTime,
        const impl_collision_policy &aPolicy) {
        return narrow_phase_overlap_parts(
            aSubject->parts(), aSubject->sweep_kinematics(),
            aOther->parts(), aOther->sweep_kinematics(),
            aDeltaTime, aPolicy);
    }

    /// \brief push two overlapping colliders apart, returning the contact normal.
    [[nodiscard]] inline std::optional<vector3_type> resolve_overlap(const impl_collider_ptr_type &aSubject,
        const impl_collider_ptr_type &aOther, const overlap &aOverlap) {
        const auto normal = aOverlap.contact_normal;
        if (normal.is_effectively_zero()) return std::nullopt;

        const auto subjectShare = aSubject->displacement_share();
        const auto otherShare = aOther->displacement_share();
        const auto totalShare = subjectShare + otherShare;

        if (totalShare > std::numeric_limits<floating_point_type>::epsilon() &&
            aOverlap.penetration > 0.0f) {
            const auto translation = normal * aOverlap.penetration;

            if (subjectShare > 0.0f)
                aSubject->set_position(aSubject->position() + translation * (subjectShare / totalShare));

            if (otherShare > 0.0f)
                aOther->set_position(aOther->position() - translation * (otherShare / totalShare));
        }

        return normal;
    }

    /// \brief the buffers one parallel detection chunk works in.
    struct detect_scratch final {
        collider_id_set seen;
        std::vector<impl_broadphase_grid::neighbour> neighbours;
    };

    /// \brief the collider pairs already separated during the current advancement iteration.
    class resolved_pair_set final {
    public:
        /// \brief forget everything, in constant time
        void clear() {
            ++m_Generation;

            if (m_Generation == 0) {
                std::fill(m_Stamps.begin(), m_Stamps.end(), std::uint32_t{0});
                m_Generation = 1;
            }

            m_Count = 0;
        }

        /// \brief record the unordered pair {aFirst, aSecond}. \return true if it was not present.
        bool insert(const collider_id_type aFirst, const collider_id_type aSecond) {
            if (m_Stamps.empty()) grow(64);

            const auto low = aFirst < aSecond ? aFirst : aSecond;
            const auto high = aFirst < aSecond ? aSecond : aFirst;
            const auto key = (static_cast<std::uint64_t>(low) << 32) ^ static_cast<std::uint64_t>(high);

            if ((m_Count + 1) * 2 > m_Stamps.size()) grow(m_Stamps.size() * 2);

            return insert_key(key);
        }

    private:
        [[nodiscard]] static std::size_t hash_of(const std::uint64_t aKey) {
            auto x = aKey + 0x9e3779b97f4a7c15ull;
            x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
            x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
            return static_cast<std::size_t>(x ^ (x >> 31));
        }

        bool insert_key(const std::uint64_t aKey) {
            const auto mask = m_Stamps.size() - 1;
            auto index = hash_of(aKey) & mask;

            for (;;) {
                if (m_Stamps[index] != m_Generation) {
                    m_Stamps[index] = m_Generation;
                    m_Keys[index] = aKey;
                    ++m_Count;
                    return true;
                }

                if (m_Keys[index] == aKey) return false;

                index = (index + 1) & mask;
            }
        }

        /// \brief resize, carrying the current iteration's entries across.
        void grow(const std::size_t aCapacity) {
            std::vector<std::uint64_t> live;
            live.reserve(m_Count);

            for (std::size_t i = 0; i < m_Stamps.size(); ++i)
                if (m_Stamps[i] == m_Generation) live.push_back(m_Keys[i]);

            m_Keys.assign(aCapacity, 0);
            m_Stamps.assign(aCapacity, 0);
            m_Generation = 1;
            m_Count = 0;

            for (const auto key : live) insert_key(key);
        }

        std::vector<std::uint64_t> m_Keys;
        std::vector<std::uint32_t> m_Stamps;
        std::uint32_t m_Generation = 1;
        std::size_t m_Count = 0;
    };

    /// \brief one dynamic body's state across the advancement iterations of a single step.
    struct body_advancement final {
        /// \brief the part of the step this body has still to travel
        delta_time_type remaining_time = 0;

        /// \brief false once this body can do nothing further this step
        bool active = false;

        floating_point_type earliest_blocking_toi = 0;
        std::optional<overlap> blocking_hit;
        impl_collider_ptr_type blocking_collider;

        std::vector<impl_broadphase_grid::neighbour> trigger_hits;

        /// \brief contacts that exist but are not closing: touching, or being left behind.
        /// Separated and reported, but they never halt the body and are never offered to a handler.
        std::vector<std::pair<overlap, impl_collider_ptr_type>> resting_contacts;

        /// \brief the surfaces this body has been redirected along so far during the step.
        clip_planes_type clip_planes;

        /// \brief drop the handles, keeping the vectors' capacity for the next step.
        void release() {
            blocking_hit.reset();
            blocking_collider.reset();
            trigger_hits.clear();
            resting_contacts.clear();
            clip_planes.clear();
        }
    };

    /// \brief the concrete scene. Defined here rather than in the header so that none of this
    /// implementation's state is visible to consumers, who see only impl_collision_scene::make.
    class scene_implementation final : public impl_collision_scene {
    public:
        scene_implementation(observer_type aCollisionObserver,
            trigger_overlap_observer_type aTriggerOverlapObserver, impl_collision_policy aPolicy,
            task_dispatcher_type aDispatcher)
        : m_Events(std::move(aCollisionObserver), std::move(aTriggerOverlapObserver))
        , m_Dispatcher(std::move(aDispatcher))
        , m_DynamicGrid(aPolicy)
        , m_StaticGrid(aPolicy)
        , m_Policy(aPolicy)
        {}

        virtual ~scene_implementation() override = default;

        virtual void do_update(const delta_time_type aDeltaTime) override;
        virtual void do_process_events() override;

        [[nodiscard]] virtual std::optional<raycast_hit> do_raycast(const vector3_type &aOrigin,
            const vector3_type &aDirection,
            const floating_point_type aMaxDistance) const override;

        [[nodiscard]] virtual std::vector<contact> do_contacts() const override;
        [[nodiscard]] virtual std::vector<contact> do_contacts_for(const collider &aCollider) const override;

        virtual box_collider_ptr_type do_make_axis_aligned_box_collider(const response_handler &) override;
        virtual box_collider_ptr_type do_make_axis_aligned_box_trigger() override;
        virtual const_box_collider_ptr_type do_make_static_axis_aligned_box_collider(const matrix4x4_type &, const vector3_type &) override;
        virtual const_box_collider_ptr_type do_make_static_axis_aligned_box_trigger(const matrix4x4_type &, const vector3_type &) override;

        virtual const_plane_collider_ptr_type do_make_static_plane_collider(const matrix4x4_type &) override;

        virtual mesh_collider_ptr_type do_make_mesh_collider(const response_handler &) override;
        virtual mesh_collider_ptr_type do_make_mesh_trigger() override;
        virtual const_mesh_collider_ptr_type do_make_static_mesh_collider(const matrix4x4_type &, const mesh_data_ptr_type &) override;
        virtual const_mesh_collider_ptr_type do_make_static_mesh_trigger(const matrix4x4_type &, const mesh_data_ptr_type &) override;

        virtual heightfield_collider_ptr_type do_make_heightfield_collider(const response_handler &) override;
        virtual const_heightfield_collider_ptr_type do_make_static_heightfield_collider(const matrix4x4_type &, const heightfield_data_ptr_type &) override;

        virtual compound_collider_ptr_type do_make_compound_collider(const response_handler &) override;
        virtual const_compound_collider_ptr_type do_make_static_compound_collider(const matrix4x4_type &,
            const std::function<void(compound_collider &)> &) override;

        virtual obb_collider_ptr_type do_make_obb_collider(const response_handler &) override;
        virtual const_obb_collider_ptr_type do_make_static_obb_collider(const matrix4x4_type &, const vector3_type &) override;

        virtual capsule_collider_ptr_type do_make_capsule_collider(const response_handler &) override;
        virtual const_capsule_collider_ptr_type do_make_static_capsule_collider(const matrix4x4_type &, const floating_point_type,
            const floating_point_type) override;

        virtual sphere_collider_ptr_type do_make_sphere_collider(const response_handler &) override;
        virtual sphere_collider_ptr_type do_make_sphere_trigger() override;
        virtual const_sphere_collider_ptr_type do_make_static_sphere_collider(const matrix4x4_type &, const floating_point_type) override;
        virtual const_sphere_collider_ptr_type do_make_static_sphere_trigger(const matrix4x4_type &, const floating_point_type) override;

    protected:
        void update_step(const delta_time_type aDeltaTime);

        /// \brief ids are handed out per scene rather than from a global counter, so that two
        /// scenes in the same process cannot perturb each other's ordering.
        [[nodiscard]] collider_id_type next_collider_id() { return m_NextColliderId++; }

        /// \brief queue an immovable body for the persistent grid.
        void register_static(const impl_collider_ptr_type &aBody, const impl_broadphase_grid::body_kind aKind) {
            m_PendingStatics.push_back({aBody, aKind});
        }

        /// \brief move anything queued into the persistent grid.
        void flush_pending_statics() {
            for (auto &pending : m_PendingStatics)
                if (const auto pBody = pending.first.lock())
                    m_StaticGrid.insert(pBody, pBody->broad_phase_bounds(), pending.second, true);

            m_PendingStatics.clear();
        }

        std::vector<impl_collider_weak_ptr_type> m_StaticColliders;
        std::vector<impl_collider_weak_ptr_type> m_StaticTriggers;
        std::vector<impl_collider_weak_ptr_type> m_DynamicColliders;
        std::vector<impl_collider_weak_ptr_type> m_DynamicTriggers;

        impl_collision_event_tracker m_Events;

        std::vector<body_advancement> m_Advancement;

        std::vector<detect_scratch> m_DetectScratch;

        std::vector<std::pair<std::uint64_t, std::uint32_t>> m_DetectOrder;

        task_dispatcher_type m_Dispatcher;

        resolved_pair_set m_ResolvedPairs;

        impl_dynamic_broadphase_grid m_DynamicGrid;

        std::vector<std::pair<impl_collider_weak_ptr_type, impl_broadphase_grid::body_kind>> m_PendingStatics;

        impl_broadphase_grid m_StaticGrid;

        const impl_collision_policy m_Policy;

        collider_id_type m_NextColliderId = 0;
    };
}

scene_ptr_type impl_collision_scene::make(observer_type aCollisionObserver,
    trigger_overlap_observer_type aTriggerObserver, impl_collision_policy aPolicy,
    task_dispatcher_type aDispatcher) {
    return scene_ptr_type(new scene_implementation(std::move(aCollisionObserver),
        std::move(aTriggerObserver), aPolicy, std::move(aDispatcher)));
}

box_collider_ptr_type scene_implementation::do_make_axis_aligned_box_collider(
    const response_handler &aCollisionResponseHandler) {
    auto pAABox = std::make_shared<impl_axis_aligned_box_collider>(m_Policy, aCollisionResponseHandler, 1, next_collider_id());
    m_DynamicColliders.push_back(pAABox);
    return pAABox;
}

const_box_collider_ptr_type scene_implementation::do_make_static_axis_aligned_box_collider(
    const matrix4x4_type &aTransform, const vector3_type &aHalfExtents) {
    auto pStaticAABox = std::make_shared<impl_axis_aligned_box_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pStaticAABox->set_transform(aTransform);
    pStaticAABox->set_half_extents(aHalfExtents);
    m_StaticColliders.push_back(pStaticAABox);
    register_static(pStaticAABox, impl_broadphase_grid::body_kind::collider);
    return pStaticAABox;
}

box_collider_ptr_type scene_implementation::do_make_axis_aligned_box_trigger() {
    auto pAABox = std::make_shared<impl_axis_aligned_box_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    m_DynamicTriggers.push_back(pAABox);
    return pAABox;
}

const_box_collider_ptr_type scene_implementation::do_make_static_axis_aligned_box_trigger(
    const matrix4x4_type &aTransform, const vector3_type &aHalfExtents) {
    auto pStaticAABox = std::make_shared<impl_axis_aligned_box_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pStaticAABox->set_transform(aTransform);
    pStaticAABox->set_half_extents(aHalfExtents);
    m_StaticTriggers.push_back(pStaticAABox);
    register_static(pStaticAABox, impl_broadphase_grid::body_kind::trigger);
    return pStaticAABox;
}

const_plane_collider_ptr_type scene_implementation::do_make_static_plane_collider(
    const matrix4x4_type &aTransform) {
    auto pPlane = std::make_shared<impl_plane_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pPlane->set_transform(aTransform);
    m_StaticColliders.push_back(pPlane);
    register_static(pPlane, impl_broadphase_grid::body_kind::collider);
    return pPlane;
}

mesh_collider_ptr_type scene_implementation::do_make_mesh_collider(
    const response_handler &aCollisionResponseHandler) {
    auto pMesh = std::make_shared<impl_mesh_collider>(m_Policy, aCollisionResponseHandler, 1, next_collider_id());
    m_DynamicColliders.push_back(pMesh);
    return pMesh;
}

mesh_collider_ptr_type scene_implementation::do_make_mesh_trigger() {
    auto pMesh = std::make_shared<impl_mesh_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    m_DynamicTriggers.push_back(pMesh);
    return pMesh;
}

const_mesh_collider_ptr_type scene_implementation::do_make_static_mesh_collider(
    const matrix4x4_type &aTransform, const mesh_data_ptr_type &aMesh) {
    auto pMesh = std::make_shared<impl_mesh_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pMesh->set_transform(aTransform);
    pMesh->set_mesh(aMesh);
    m_StaticColliders.push_back(pMesh);
    register_static(pMesh, impl_broadphase_grid::body_kind::collider);
    return pMesh;
}

const_mesh_collider_ptr_type scene_implementation::do_make_static_mesh_trigger(
    const matrix4x4_type &aTransform, const mesh_data_ptr_type &aMesh) {
    auto pMesh = std::make_shared<impl_mesh_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pMesh->set_transform(aTransform);
    pMesh->set_mesh(aMesh);
    m_StaticTriggers.push_back(pMesh);
    register_static(pMesh, impl_broadphase_grid::body_kind::trigger);
    return pMesh;
}

heightfield_collider_ptr_type scene_implementation::do_make_heightfield_collider(
    const response_handler &aCollisionResponseHandler) {
    auto pTerrain = std::make_shared<impl_heightfield_collider>(m_Policy, aCollisionResponseHandler, 1,
        next_collider_id());
    m_DynamicColliders.push_back(pTerrain);
    return pTerrain;
}

const_heightfield_collider_ptr_type scene_implementation::do_make_static_heightfield_collider(
    const matrix4x4_type &aTransform, const heightfield_data_ptr_type &aHeightfield) {
    auto pTerrain = std::make_shared<impl_heightfield_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pTerrain->set_transform(aTransform);
    pTerrain->set_heightfield(aHeightfield);
    m_StaticColliders.push_back(pTerrain);
    register_static(pTerrain, impl_broadphase_grid::body_kind::collider);
    return pTerrain;
}

compound_collider_ptr_type scene_implementation::do_make_compound_collider(
    const response_handler &aCollisionResponseHandler) {
    auto pCompound = std::make_shared<impl_compound_collider>(m_Policy, aCollisionResponseHandler, 1, next_collider_id());
    m_DynamicColliders.push_back(pCompound);
    return pCompound;
}

const_compound_collider_ptr_type scene_implementation::do_make_static_compound_collider(
    const matrix4x4_type &aTransform, const std::function<void(compound_collider &)> &aBuild) {
    auto pCompound = std::make_shared<impl_compound_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pCompound->set_transform(aTransform);

    if (aBuild) aBuild(*pCompound);
    m_StaticColliders.push_back(pCompound);
    register_static(pCompound, impl_broadphase_grid::body_kind::collider);
    return pCompound;
}

obb_collider_ptr_type scene_implementation::do_make_obb_collider(
    const response_handler &aCollisionResponseHandler) {
    auto pBox = std::make_shared<impl_obb_collider>(m_Policy, aCollisionResponseHandler, 1, next_collider_id());
    m_DynamicColliders.push_back(pBox);
    return pBox;
}

const_obb_collider_ptr_type scene_implementation::do_make_static_obb_collider(
    const matrix4x4_type &aTransform, const vector3_type &aHalfExtents) {
    auto pBox = std::make_shared<impl_obb_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pBox->set_transform(aTransform);
    pBox->set_half_extents(aHalfExtents);
    m_StaticColliders.push_back(pBox);
    register_static(pBox, impl_broadphase_grid::body_kind::collider);
    return pBox;
}

capsule_collider_ptr_type scene_implementation::do_make_capsule_collider(
    const response_handler &aCollisionResponseHandler) {
    auto pCapsule = std::make_shared<impl_capsule_collider>(m_Policy, aCollisionResponseHandler, 1, next_collider_id());
    m_DynamicColliders.push_back(pCapsule);
    return pCapsule;
}

const_capsule_collider_ptr_type scene_implementation::do_make_static_capsule_collider(
    const matrix4x4_type &aTransform, const floating_point_type aRadius,
    const floating_point_type aHalfHeight) {
    auto pCapsule = std::make_shared<impl_capsule_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pCapsule->set_transform(aTransform);
    pCapsule->set_radius(aRadius);
    pCapsule->set_half_height(aHalfHeight);
    m_StaticColliders.push_back(pCapsule);
    register_static(pCapsule, impl_broadphase_grid::body_kind::collider);
    return pCapsule;
}

sphere_collider_ptr_type scene_implementation::do_make_sphere_collider(
    const response_handler &aCollisionResponseHandler) {
    auto pSphere = std::make_shared<impl_sphere_collider>(m_Policy, aCollisionResponseHandler, 1, next_collider_id());
    m_DynamicColliders.push_back(pSphere);
    return pSphere;
}

const_sphere_collider_ptr_type scene_implementation::do_make_static_sphere_collider(
    const matrix4x4_type &aTransform, const floating_point_type aRadius) {
    auto pStaticSphere = std::make_shared<impl_sphere_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pStaticSphere->set_transform(aTransform);
    pStaticSphere->set_radius(aRadius);
    m_StaticColliders.push_back(pStaticSphere);
    register_static(pStaticSphere, impl_broadphase_grid::body_kind::collider);
    return pStaticSphere;
}

sphere_collider_ptr_type scene_implementation::do_make_sphere_trigger() {
    auto pSphere = std::make_shared<impl_sphere_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    m_DynamicTriggers.push_back(pSphere);
    return pSphere;
}

const_sphere_collider_ptr_type scene_implementation::do_make_static_sphere_trigger(
    const matrix4x4_type &aTransform, const floating_point_type aRadius) {
    auto pStaticSphere = std::make_shared<impl_sphere_collider>(m_Policy,
        response_handlers::null_opt, 0, next_collider_id());
    pStaticSphere->set_transform(aTransform);
    pStaticSphere->set_radius(aRadius);
    m_StaticTriggers.push_back(pStaticSphere);
    register_static(pStaticSphere, impl_broadphase_grid::body_kind::trigger);
    return pStaticSphere;
}

void scene_implementation::update_step(const delta_time_type aDeltaTime) {
    using body_kind = impl_broadphase_grid::body_kind;
    using body_ref = impl_broadphase_grid::neighbour;

    const auto gather = [&](detect_scratch &aScratch, const impl_collider &aSubject,
        const delta_time_type aTime, std::vector<body_ref> &aOut) {
        GDK_COLLISION_PROFILE_BEGIN(gather_query_ms);
        auto &seen = aScratch.seen;
        seen.clear();
        aOut.clear();

        const auto bounds = aSubject.broad_phase_swept_bounds(aTime);

        m_StaticGrid.gather_overlapping_excluding(bounds, &aSubject, seen, aOut);
        m_DynamicGrid.gather_overlapping_excluding(bounds, &aSubject, seen, aOut);
        GDK_COLLISION_PROFILE_END(gather_query_ms);

        GDK_COLLISION_PROFILE_BEGIN(gather_sort_ms);
        impl_broadphase_grid::sort(aOut);
        GDK_COLLISION_PROFILE_END(gather_sort_ms);
    };

    flush_pending_statics();

    m_StaticGrid.compact_expired();

    if (m_DetectScratch.empty()) m_DetectScratch.resize(1);

    auto dynamicColliders = lock_and_prune(m_DynamicColliders);
    const auto dynamicTriggers = lock_and_prune(m_DynamicTriggers);

    const auto firstNonKinematic = std::stable_partition(dynamicColliders.begin(), dynamicColliders.end(),
        [](const impl_collider_ptr_type &aBody) { return aBody->is_kinematic(); });

    const std::vector<impl_collider_ptr_type> kinematicColliders(dynamicColliders.begin(), firstNonKinematic);
    dynamicColliders.erase(dynamicColliders.begin(), firstNonKinematic);

    for (auto &pCollider : dynamicColliders) pCollider->integrate_angular_velocity(aDeltaTime);
    for (auto &pKinematic : kinematicColliders) pKinematic->integrate_angular_velocity(aDeltaTime);
    for (auto &pTrigger : dynamicTriggers) pTrigger->integrate_angular_velocity(aDeltaTime);

    GDK_COLLISION_PROFILE_BEGIN(broadphase_rebuild_ms);

    m_DynamicGrid.clear();
    m_DynamicGrid.reserve(dynamicColliders.size() + kinematicColliders.size() + dynamicTriggers.size());

    for (auto &pCollider : dynamicColliders)
        m_DynamicGrid.add(pCollider, pCollider->broad_phase_swept_bounds(aDeltaTime), body_kind::collider);

    for (auto &pKinematic : kinematicColliders)
        m_DynamicGrid.add(pKinematic, pKinematic->broad_phase_swept_bounds(aDeltaTime), body_kind::collider);

    for (auto &pTrigger : dynamicTriggers)
        m_DynamicGrid.add(pTrigger, pTrigger->broad_phase_swept_bounds(aDeltaTime), body_kind::trigger);

    m_DynamicGrid.build(m_Dispatcher);

    GDK_COLLISION_PROFILE_END(broadphase_rebuild_ms);
    GDK_COLLISION_PROFILE_BEGIN(kinematic_pass_ms);

    for (auto &pKinematic : kinematicColliders) {
        std::vector<body_ref> neighbours;
        gather(m_DetectScratch[0], *pKinematic, aDeltaTime, neighbours);

        for (const auto &neighbour : neighbours) {
            const auto oHit = narrow_phase_overlap_parts(
                pKinematic->parts(),
                {pKinematic->position(), pKinematic->velocity(), pKinematic->rotation()},
                neighbour.ptr->parts(), neighbour.ptr->sweep_kinematics(), aDeltaTime, m_Policy);

            if (!oHit) continue;

            if (neighbour.kind == body_kind::collider)
                m_Events.record_collision(pKinematic, neighbour.ptr, oHit->contact_normal,
                    oHit->contact_point, oHit->penetration);
            else m_Events.record_trigger(pKinematic, neighbour.ptr);
        }

        pKinematic->set_position(pKinematic->position() + pKinematic->velocity() * aDeltaTime);
    }

    GDK_COLLISION_PROFILE_END(kinematic_pass_ms);
    GDK_COLLISION_PROFILE_BEGIN(dynamic_loop_ms);

    m_Advancement.resize(dynamicColliders.size());

    const auto chunkSize = std::max<std::size_t>(m_Policy.PARALLEL_DETECT_CHUNK_SIZE, 1);
    const auto chunkCount = m_Dispatcher
        ? (dynamicColliders.size() + chunkSize - 1) / chunkSize
        : std::size_t{1};

    if (m_DetectScratch.size() < chunkCount) m_DetectScratch.resize(chunkCount);

    m_DetectOrder.clear();
    m_DetectOrder.reserve(dynamicColliders.size());

    for (std::uint32_t i = 0; i < dynamicColliders.size(); ++i)
        m_DetectOrder.emplace_back(m_Policy.SPATIAL_DETECT_ORDER
            ? broadphase_morton_code(m_Policy, dynamicColliders[i]->position())
            : i, i);

    if (m_Policy.SPATIAL_DETECT_ORDER) std::sort(m_DetectOrder.begin(), m_DetectOrder.end());

    for (std::size_t i = 0; i < dynamicColliders.size(); ++i) {
        auto &state = m_Advancement[i];
        state.remaining_time = aDeltaTime;
        state.active = aDeltaTime > 0;
        state.release();   
    }

    const auto detect_chunk = [&](const std::size_t aChunk) {
        auto &scratch = m_DetectScratch[aChunk];

        const auto begin = m_Dispatcher ? aChunk * chunkSize : std::size_t{0};
        const auto end = m_Dispatcher
            ? std::min(begin + chunkSize, m_DetectOrder.size())
            : m_DetectOrder.size();

        for (std::size_t slot = begin; slot < end; ++slot) {
            const auto i = m_DetectOrder[slot].second;

            auto &state = m_Advancement[i];
            if (!state.active) continue;

            const auto &pDynamicCollider = dynamicColliders[i];

            state.earliest_blocking_toi = state.remaining_time;
            state.blocking_hit.reset();
            state.blocking_collider.reset();
            state.trigger_hits.clear();
            state.resting_contacts.clear();

            gather(scratch, *pDynamicCollider, state.remaining_time, scratch.neighbours);

            GDK_COLLISION_PROFILE_BEGIN(narrow_phase_ms);

            for (auto &neighbour : scratch.neighbours) {
                if (auto ov = overlap_between(pDynamicCollider, neighbour.ptr, state.remaining_time, m_Policy)) {
                    if (neighbour.kind == body_kind::collider) {
                        if (!is_closing(*ov, pDynamicCollider->sweep_kinematics(),
                            neighbour.ptr->sweep_kinematics(), m_Policy)) {
                            state.resting_contacts.emplace_back(*ov, neighbour.ptr);
                            continue;
                        }

                        if (ov->entry_time < state.earliest_blocking_toi) {
                            state.earliest_blocking_toi = ov->entry_time;
                            state.blocking_hit = ov;
                            state.blocking_collider = neighbour.ptr;
                        }
                    }
                    else if (ov->entry_time <= state.earliest_blocking_toi + m_Policy.AXIS_ENTRY_EPSILON) {
                        state.trigger_hits.push_back(neighbour);
                    }
                }
            }

            GDK_COLLISION_PROFILE_END(narrow_phase_ms);
        }
    };

    for (unsigned short int step = 0; step < m_Policy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS; ++step) {
        bool anyActive = false;
        for (const auto &state : m_Advancement) if (state.active) { anyActive = true; break; }
        if (!anyActive) break;

        if (m_Dispatcher) m_Dispatcher(chunkCount, detect_chunk);
        else detect_chunk(0);

        GDK_COLLISION_PROFILE_BEGIN(resolve_respond_ms);

        m_ResolvedPairs.clear();

        const auto resolve_once = [&](const impl_collider_ptr_type &aSubject,
            const impl_collider_ptr_type &aOther,
            const overlap &aOverlap) -> std::optional<vector3_type> {
            if (m_ResolvedPairs.insert(aSubject->id(), aOther->id()))
                return resolve_overlap(aSubject, aOther, aOverlap);

            if (aOverlap.contact_normal.is_effectively_zero()) return std::nullopt;
            return aOverlap.contact_normal;
        };

        for (std::size_t i = 0; i < dynamicColliders.size(); ++i) {
            auto &state = m_Advancement[i];
            if (!state.active) continue;

            const auto &pDynamicCollider = dynamicColliders[i];

            const bool advanced = state.earliest_blocking_toi > m_Policy.TIME_OF_IMPACT_EPSILON;
            if (advanced) {
                pDynamicCollider->set_position(pDynamicCollider->position()
                    + pDynamicCollider->velocity() * state.earliest_blocking_toi);
                state.remaining_time -= state.earliest_blocking_toi;
            }

            for (const auto &tr : state.trigger_hits)
                m_Events.record_trigger(pDynamicCollider, tr.ptr);

            for (const auto &[hit, pOther] : state.resting_contacts) {
                const auto restingNormal = resolve_once(pDynamicCollider, pOther, hit);
                m_Events.record_collision(pDynamicCollider, pOther,
                    restingNormal ? *restingNormal : hit.contact_normal, hit.contact_point,
                    hit.penetration);
            }

            if (state.blocking_hit) {
                auto velocityChange = vector3_type::zero;

                const auto oCollisionNormal = resolve_once(pDynamicCollider, state.blocking_collider,
                    *state.blocking_hit);

                if (oCollisionNormal) {
                    const auto alreadyKnown = [&] {
                        for (const auto &plane : state.clip_planes)
                            if (plane.dot_product(*oCollisionNormal) > m_Policy.MIN_SWEPT_SPEED_SQUARED) return true;
                        return false;
                    }();

                    if (!alreadyKnown) state.clip_planes.push_back(*oCollisionNormal);

                    const contact_context context{*state.blocking_collider, *state.blocking_hit,
                        *oCollisionNormal, state.remaining_time, state.clip_planes, step,
                        m_Policy.MIN_SWEPT_SPEED, m_Policy.NORMALIZATION_THRESHOLD,
                        m_Policy.CLIP_PLANE_PARALLEL_COSINE};

                    const auto velocityBefore = pDynamicCollider->velocity();

                    const auto responseDelta = pDynamicCollider->respond_to_collision_resolution(context);

                    pDynamicCollider->set_velocity(pDynamicCollider->velocity() + responseDelta);

                    velocityChange = pDynamicCollider->velocity() - velocityBefore;
                }

                m_Events.record_collision(pDynamicCollider, state.blocking_collider,
                    oCollisionNormal ? *oCollisionNormal : state.blocking_hit->contact_normal,
                    state.blocking_hit->contact_point, state.blocking_hit->penetration);

                if (state.remaining_time <= 0 || pDynamicCollider->velocity().is_effectively_zero())
                    state.active = false;

                else if (!advanced && velocityChange.is_effectively_zero()) state.active = false;
            }
            else state.active = false;

            if (state.remaining_time <= 0) state.active = false;
        }

        GDK_COLLISION_PROFILE_END(resolve_respond_ms);
    }

    for (auto &state : m_Advancement) state.release();

    GDK_COLLISION_PROFILE_END(dynamic_loop_ms);
    GDK_COLLISION_PROFILE_BEGIN(trigger_loop_ms);

    for (auto &pDynamicTrigger : dynamicTriggers) {
        float remainingTime = aDeltaTime;
        int stepCount = 0;

        while (remainingTime > 0 && stepCount++ < m_Policy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS) {
            float earliestTOI = remainingTime;
            std::vector<body_ref> triggerHits;
            std::vector<body_ref> neighbours;

            gather(m_DetectScratch[0], *pDynamicTrigger, remainingTime, neighbours);

            for (const auto &neighbour : neighbours) {
                if (auto ov = overlap_between(pDynamicTrigger, neighbour.ptr, remainingTime, m_Policy)) {
                    if (ov->entry_time < earliestTOI - m_Policy.AXIS_ENTRY_EPSILON) {
                        earliestTOI = ov->entry_time;
                        triggerHits.clear();
                        triggerHits.push_back(neighbour);
                    }
                    else if (std::abs(ov->entry_time - earliestTOI) <= m_Policy.AXIS_ENTRY_EPSILON)
                        triggerHits.push_back(neighbour);
                }
            }

            float advance = 0.0f;

            if (!triggerHits.empty()) {
                if (earliestTOI <= m_Policy.TIME_OF_IMPACT_EPSILON) advance = remainingTime;
                else advance = earliestTOI;
            }
            else advance = remainingTime;

            pDynamicTrigger->set_position(pDynamicTrigger->position() + pDynamicTrigger->velocity() * advance);
            remainingTime -= advance;

            for (const auto &hit : triggerHits) {
                if (const auto confirm = overlap_between(pDynamicTrigger, hit.ptr, 0.0f, m_Policy)) {
                    if (confirm->entry_time <= m_Policy.TIME_OF_IMPACT_EPSILON)
                        m_Events.record_trigger(pDynamicTrigger, hit.ptr);
                }
            }
        }
    }

    GDK_COLLISION_PROFILE_END(trigger_loop_ms);
    GDK_COLLISION_PROFILE_BEGIN(events_ms);

    m_Events.flush();

    GDK_COLLISION_PROFILE_END(events_ms);

    clear_velocity_in_collection(dynamicColliders);
    clear_velocity_in_collection(kinematicColliders);
    clear_velocity_in_collection(dynamicTriggers);

    m_DynamicGrid.clear();
    for (auto &scratch : m_DetectScratch) scratch.neighbours.clear();
}

namespace {
    /// \brief turn the tracker's weakly-held records into what a caller is handed
    /// \param aOnly if set, keep only contacts involving this collider.
    [[nodiscard]] std::vector<contact> collect_contacts(
        const impl_collision_event_tracker &aEvents, const collider *aOnly) {
        std::vector<contact> result;

        const auto &recorded = aEvents.last_step_collisions();
        result.reserve(recorded.size());

        for (const auto &entry : recorded) {
            auto a = entry.a.lock();
            auto b = entry.b.lock();
            if (!a || !b) continue;

            if (aOnly && a.get() != aOnly && b.get() != aOnly) continue;

            contact reported;
            reported.a = std::move(a);
            reported.b = std::move(b);
            reported.normal = entry.normal;
            reported.point = entry.point;
            reported.penetration = entry.penetration;

            result.push_back(std::move(reported));
        }

        return result;
    }
}

std::vector<contact> scene_implementation::do_contacts() const {
    return collect_contacts(m_Events, nullptr);
}

std::vector<contact> scene_implementation::do_contacts_for(const collider &aCollider) const {
    return collect_contacts(m_Events, &aCollider);
}

std::optional<raycast_hit> scene_implementation::do_raycast(const vector3_type &aOrigin,
    const vector3_type &aDirection, const floating_point_type aMaxDistance) const {
    const auto direction = aDirection.normal();
    if (direction.is_effectively_zero() || aMaxDistance <= 0.0f) return std::nullopt;

    static const std::vector<collider_part> ray{collider_part{sphere_shape{0.0f}}};
    const shape_kinematics rayKinematics{aOrigin, direction, quaternion_type::identity};

    std::optional<raycast_hit> nearest;

    const auto consider = [&](const impl_collider_ptr_type &pCollider) {
        const auto result = narrow_phase_overlap_parts(ray, rayKinematics,
            pCollider->parts(),
            {pCollider->position(), pCollider->velocity(), pCollider->rotation()},
            aMaxDistance, m_Policy);

        if (!result) return;
        if (nearest && result->entry_time >= nearest->distance) return;

        raycast_hit hit;
        hit.collider = pCollider;
        hit.distance = result->entry_time;
        hit.point = result->contact_point;
        hit.normal = result->contact_normal;
        nearest = hit;
    };

    for (const auto &pending : m_PendingStatics)
        if (pending.second == impl_broadphase_grid::body_kind::collider)
            if (const auto pBody = pending.first.lock()) consider(pBody);

    impl_collider::broadphase_bounds rayBounds;
    const auto tip = aOrigin + direction * aMaxDistance;
    rayBounds.min = vector3_type::min(aOrigin, tip);
    rayBounds.max = vector3_type::max(aOrigin, tip);

    collider_id_set seen;
    std::vector<impl_broadphase_grid::neighbour> candidates;
    m_StaticGrid.gather_overlapping(rayBounds, seen, candidates);

    for (const auto &candidate : candidates)
        if (candidate.kind == impl_broadphase_grid::body_kind::collider) consider(candidate.ptr);

    for (const auto &weak : m_DynamicColliders)
        if (const auto pCollider = weak.lock()) consider(pCollider);

    return nearest;
}

void scene_implementation::do_process_events() {
    m_Events.dispatch();
}

void scene_implementation::do_update(const delta_time_type aDeltaTime) {
    GDK_COLLISION_PROFILE_BEGIN(total_ms);

    const auto subDeltaTime = aDeltaTime / m_Policy.STEPS_PER_UPDATE;
    for (unsigned short int step(0); step < m_Policy.STEPS_PER_UPDATE; ++step) update_step(subDeltaTime);

    GDK_COLLISION_PROFILE_END(total_ms);
    GDK_COLLISION_PROFILE_COUNT_STEP();
}
