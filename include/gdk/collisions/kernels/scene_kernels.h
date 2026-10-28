// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_KERNELS_SCENE_KERNELS_H
#define GDK_COLLISIONS_KERNELS_SCENE_KERNELS_H

#include <gdk/collisions/kernels/tables.h>
#include <gdk/collisions/parallelepiped.h>
#include <gdk/collisions/upright_cylinder.h>

#include <optional>
#include <span>
#include <vector>

namespace gdk::collisions {
    /// \brief the spatial work of a scene: which bodies could meet, whether they do, and what rays and volumes reach.
    ///
    /// A scene owns the tables and its kernels own whatever structures they build over them. A kernels object
    /// serves one scene. Write one to put that work somewhere else, a GPU for instance, and hold a std kernels
    /// object to hand on whatever it does not do itself.
    /// \note one call at a time, except that cast_ray and meets_statics may run on several threads at once
    /// between the others; an implementation may use threads of its own within a call
    class scene_kernels {
    public:
        virtual ~scene_kernels() = default;

        /// \brief rows of the statics table were filled or freed. A freed row may be filled again later.
        virtual void statics_changed(const scene_tables &aTables, std::span<const std::uint32_t> aAdded,
            std::span<const std::uint32_t> aRemoved) = 0;

        /// \brief the movers table was filled again, for a step of aDeltaTime
        virtual void movers_changed(const scene_tables &aTables, const delta_time_type aDeltaTime) = 0;

        /// \brief every pair a subject could meet within its time: no subject with itself, and only bodies
        /// that each collide with a layer the other is on. Pairs come subject by subject, in the request's order;
        /// within a subject colliders come before triggers, statics before movers, then by ascending id.
        /// Extra pairs are allowed, a missing one is not.
        virtual void gather_pairs(const scene_tables &aTables, const pair_request &aRequest,
            std::vector<candidate_pair> &aOut) = 0;

        /// \brief the tests that meet, in test order. A test meets when, within its time, the shapes penetrate
        /// deeper than penetration_depth_epsilon or close faster than min_swept_speed.
        virtual void test_pairs(const scene_tables &aTables, std::span<const pair_test> aTests,
            std::vector<pair_hit> &aOut) = 0;

        /// \brief the nearest collider, never a trigger, along a ray
        [[nodiscard]] virtual std::optional<ray_answer> cast_ray(const scene_tables &aTables,
            const ray_query &aQuery) const = 0;

        /// \brief whether any static collider, never a trigger nor one of aIgnore, reaches into the volume
        [[nodiscard]] virtual bool meets_statics(const scene_tables &aTables, const upright_cylinder &aVolume,
            std::span<const collider_id_type> aIgnore) const = 0;

        /// \overload
        [[nodiscard]] virtual bool meets_statics(const scene_tables &aTables, const parallelepiped &aVolume,
            std::span<const collider_id_type> aIgnore) const = 0;
    };
}

#endif
