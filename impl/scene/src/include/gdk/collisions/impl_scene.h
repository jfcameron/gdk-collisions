// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_SCENE_H
#define GDK_COLLISIONS_IMPL_SCENE_H

#include <gdk/collisions/kernels/scene_kernels.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_collider_id_pool.h>
#include <gdk/collisions/impl_collision_event_tracker.h>
#include <gdk/collisions/impl_resolved_pair_set.h>
#include <gdk/collisions/impl_scene_policy.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gdk::collisions {
    /// \brief the scene make_scene makes: conservative advancement, its spatial work done by its kernels
    class impl_scene final : public scene {
    public:
        impl_scene(collision_observer_type aCollisionObserver, trigger_observer_type aTriggerObserver,
            scene_policy aPolicy, scene_kernels_ptr_type aKernels);

        virtual ~impl_scene() override = default;

    protected:
        virtual void do_update(const delta_time_type aDeltaTime) override;
        virtual void do_process_events() override;
        [[nodiscard]] virtual statistics do_take_statistics() override;

        [[nodiscard]] virtual std::optional<raycast_hit> do_raycast(const vector3_type &aOrigin,
            const vector3_type &aDirection, const floating_point_type aMaxDistance, const collider *pIgnore,
            bool aStaticOnly, collision_layers_type aLayers) const override;

        [[nodiscard]] virtual bool do_overlaps_static(const upright_cylinder &aVolume) const override;
        [[nodiscard]] virtual bool do_overlaps_static(const parallelepiped &aVolume) const override;

        [[nodiscard]] virtual std::vector<contact> do_contacts() const override;
        [[nodiscard]] virtual std::vector<contact> do_contacts_for(const collider &aCollider) const override;

        [[nodiscard]] virtual aabb_collider_ptr_type do_make_aabb_collider(const response_handler &) override;
        [[nodiscard]] virtual aabb_collider_ptr_type do_make_aabb_trigger() override;
        [[nodiscard]] virtual const_aabb_collider_ptr_type do_make_static_aabb_collider(const matrix4x4_type &,
            const vector3_type &) override;
        [[nodiscard]] virtual const_aabb_collider_ptr_type do_make_static_aabb_trigger(const matrix4x4_type &,
            const vector3_type &) override;

        [[nodiscard]] virtual const_plane_collider_ptr_type do_make_static_plane_collider(const matrix4x4_type &) override;

        [[nodiscard]] virtual mesh_collider_ptr_type do_make_mesh_collider(const response_handler &) override;
        [[nodiscard]] virtual mesh_collider_ptr_type do_make_mesh_trigger() override;
        [[nodiscard]] virtual const_mesh_collider_ptr_type do_make_static_mesh_collider(const matrix4x4_type &,
            const mesh_data_ptr_type &) override;
        [[nodiscard]] virtual const_mesh_collider_ptr_type do_make_static_mesh_trigger(const matrix4x4_type &,
            const mesh_data_ptr_type &) override;

        [[nodiscard]] virtual heightfield_collider_ptr_type do_make_heightfield_collider(const response_handler &) override;
        [[nodiscard]] virtual const_heightfield_collider_ptr_type do_make_static_heightfield_collider(const matrix4x4_type &,
            const heightfield_data_ptr_type &) override;

        [[nodiscard]] virtual compound_collider_ptr_type do_make_compound_collider(const response_handler &) override;
        [[nodiscard]] virtual const_compound_collider_ptr_type do_make_static_compound_collider(const matrix4x4_type &,
            const std::function<void(compound_collider &)> &) override;

        [[nodiscard]] virtual obb_collider_ptr_type do_make_obb_collider(const response_handler &) override;
        [[nodiscard]] virtual const_obb_collider_ptr_type do_make_static_obb_collider(const matrix4x4_type &,
            const vector3_type &) override;

        [[nodiscard]] virtual capsule_collider_ptr_type do_make_capsule_collider(const response_handler &) override;
        [[nodiscard]] virtual const_capsule_collider_ptr_type do_make_static_capsule_collider(const matrix4x4_type &,
            const floating_point_type, const floating_point_type) override;

        [[nodiscard]] virtual sphere_collider_ptr_type do_make_sphere_collider(const response_handler &) override;
        [[nodiscard]] virtual sphere_collider_ptr_type do_make_sphere_trigger() override;
        [[nodiscard]] virtual const_sphere_collider_ptr_type do_make_static_sphere_collider(const matrix4x4_type &,
            const floating_point_type) override;
        [[nodiscard]] virtual const_sphere_collider_ptr_type do_make_static_sphere_trigger(const matrix4x4_type &,
            const floating_point_type) override;

    private:
        static constexpr std::uint32_t NO_ROW = ~std::uint32_t{0};

        /// \brief one dynamic collider's progress through the current step's advancement
        struct body_advancement final {
            delta_time_type remaining_time = 0;

            bool active = false;

            floating_point_type earliest_blocking_toi = 0;
            std::optional<overlap> blocking_hit;
            body_ref blocking_body;

            std::vector<body_ref> trigger_hits;
            std::vector<std::pair<overlap, body_ref>> resting_contacts;

            clip_planes_type clip_planes;

            void release();
        };

        /// \brief shared surfaces, counted by the parts that refer to them
        template <typename pointer_type>
        struct surface_uses final {
            std::unordered_map<const void *, std::uint32_t> index;
            std::vector<std::uint32_t> count;
            std::vector<std::uint32_t> free;

            [[nodiscard]] std::uint32_t take(std::vector<pointer_type> &aTable, const pointer_type &aSurface);
            void give_back(std::vector<pointer_type> &aTable, const std::uint32_t aIndex);
        };

        template <typename impl_type>
        [[nodiscard]] std::shared_ptr<impl_type> make_body(const response_handler &aHandler,
            const floating_point_type aInverseOverlapWeight);

        template <typename impl_type>
        [[nodiscard]] std::shared_ptr<impl_type> make_dynamic(const response_handler &aHandler);

        template <typename impl_type>
        [[nodiscard]] std::shared_ptr<impl_type> make_trigger();

        template <typename impl_type, typename configure_type>
        [[nodiscard]] std::shared_ptr<impl_type> make_static(const matrix4x4_type &aTransform, const bool aTrigger,
            configure_type &&aConfigure);

        [[nodiscard]] scene_tables tables() const;

        void register_static(const impl_collider_ptr_type &aBody, const bool aTrigger);
        void forget_dead_statics();

        /// \brief append aBody as a row, with a surface entry of the table's own for each surface it refers to
        static void append_row(body_table &aTable, const impl_collider &aBody, const body_motion aMotion,
            const bool aTrigger);

        static void clear_table(body_table &aTable);

        void fill_static_row(const std::uint32_t aRow, const impl_collider &aBody, const bool aTrigger);
        void release_static_surfaces(const std::uint32_t aRow);
        void refresh_row(const std::uint32_t aRow);
        void mark_dirty(const body_ref &aBody);
        void refresh_dirty();

        [[nodiscard]] impl_collider_ptr_type owner_of(const body_ref &aBody) const;

        void update_step(const delta_time_type aDeltaTime);

        void gather_movers(const delta_time_type aDeltaTime);
        void kinematic_pass(const delta_time_type aDeltaTime);
        void advance_dynamics(const delta_time_type aDeltaTime);
        void resolve_and_respond(const unsigned short int aIteration);
        void sweep_triggers(const delta_time_type aDeltaTime);
        void finish_step();

        void gather_and_test(const pair_request &aRequest, const bool aOthersMove);

        [[nodiscard]] std::optional<vector3_type> resolve_once(const std::uint32_t aRow, const body_ref &aOther,
            const overlap &aOverlap);

        void clear_velocities();

        const scene_policy m_Policy;
        const contact_rules m_Rules;

        scene_kernels_ptr_type m_Kernels;

        std::shared_ptr<impl_collider_id_pool> m_Ids = std::make_shared<impl_collider_id_pool>();

        impl_collision_event_tracker m_Events;

        std::vector<impl_collider_weak_ptr_type> m_DynamicColliders;
        std::vector<impl_collider_weak_ptr_type> m_DynamicTriggers;

        surface_uses<mesh_data_ptr_type> m_MeshUses;
        surface_uses<heightfield_data_ptr_type> m_HeightfieldUses;

        body_table m_Statics;
        std::vector<impl_collider_weak_ptr_type> m_StaticOwners;
        std::vector<std::uint32_t> m_StaticRowOfId;
        std::vector<std::uint32_t> m_StaticPartCapacity;
        std::vector<std::uint32_t> m_FreeStaticRows;

        body_table m_Movers;
        std::vector<impl_collider_ptr_type> m_MoverOwners;
        std::uint32_t m_DynamicCount = 0;
        std::uint32_t m_KinematicCount = 0;

        std::vector<std::uint8_t> m_Dirty;
        std::vector<std::uint32_t> m_DirtyRows;


        std::vector<body_advancement> m_Advancement;

        std::vector<std::uint32_t> m_Subjects;
        std::vector<delta_time_type> m_Times;
        std::vector<candidate_pair> m_Pairs;
        std::vector<pair_test> m_Tests;
        std::vector<pair_hit> m_Hits;

        impl_resolved_pair_set m_ResolvedPairs;

        std::vector<body_ref> m_TriggerHits;

        statistics m_Statistics;
    };

    template <typename impl_type>
    std::shared_ptr<impl_type> impl_scene::make_body(const response_handler &aHandler,
        const floating_point_type aInverseOverlapWeight) {
        const auto id = m_Ids->take();

        std::unique_ptr<impl_type> pBody;

        try { pBody = std::make_unique<impl_type>(m_Policy, aHandler, aInverseOverlapWeight, id); }
        catch (...) {
            m_Ids->give_back(id);
            throw;
        }

        return std::shared_ptr<impl_type>(pBody.release(), [pIds = m_Ids](impl_type *const pDead) noexcept {
            const auto deadId = pDead->id();
            delete pDead;
            pIds->give_back(deadId);
        });
    }

    template <typename impl_type>
    std::shared_ptr<impl_type> impl_scene::make_dynamic(const response_handler &aHandler) {
        auto pBody = make_body<impl_type>(aHandler, 1);
        m_DynamicColliders.push_back(pBody);
        return pBody;
    }

    template <typename impl_type>
    std::shared_ptr<impl_type> impl_scene::make_trigger() {
        auto pBody = make_body<impl_type>(response_handlers::null_opt, 0);
        m_DynamicTriggers.push_back(pBody);
        return pBody;
    }

    template <typename impl_type, typename configure_type>
    std::shared_ptr<impl_type> impl_scene::make_static(const matrix4x4_type &aTransform, const bool aTrigger,
        configure_type &&aConfigure) {
        auto pBody = make_body<impl_type>(response_handlers::null_opt, 0);
        pBody->set_transform(aTransform);
        aConfigure(*pBody);
        register_static(pBody, aTrigger);
        return pBody;
    }
}

#endif
