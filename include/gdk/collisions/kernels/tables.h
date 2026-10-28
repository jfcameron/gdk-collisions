// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_KERNELS_TABLES_H
#define GDK_COLLISIONS_KERNELS_TABLES_H

#include <gdk/collisions/overlap.h>
#include <gdk/collisions/types.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace gdk::collisions {
    enum class shape_kind : std::uint8_t {
        sphere,
        aabb,
        obb,
        capsule,
        plane,
        mesh,
        heightfield,
    };

    inline constexpr std::uint32_t NO_SURFACE = ~std::uint32_t{0};

    /// \brief one piece of a collider's geometry, placed in the collider's frame
    struct part_record final {
        shape_kind kind = shape_kind::sphere;

        /// \brief sphere: x is the radius. aabb and obb: the half extents. capsule: x the radius, y the half height
        vector3_type size = vector3_type::zero;

        vector3_type offset = vector3_type::zero;
        quaternion_type turn = quaternion_type::identity;

        /// \brief mesh: an index into its table's surfaces.meshes. heightfield: into its surfaces.heightfields
        std::uint32_t surface = NO_SURFACE;
    };

    struct bounds final {
        vector3_type min = vector3_type::zero;
        vector3_type max = vector3_type::zero;
    };

    /// \brief the shared, unchanging geometry parts refer to
    struct surface_table final {
        std::vector<mesh_data_ptr_type> meshes;
        std::vector<heightfield_data_ptr_type> heightfields;
    };

    enum class body_motion : std::uint8_t {
        dynamic,
        kinematic,
        fixed,
    };

    /// \brief bodies, one row each; a row's parts are parts[first_part, first_part + part_count)
    struct body_table final {
        std::vector<collider_id_type> id;
        std::vector<vector3_type> position;
        std::vector<vector3_type> velocity;
        std::vector<quaternion_type> rotation;

        /// \brief where the body stands now, in the world
        std::vector<bounds> extent;

        std::vector<collision_layers_type> on;
        std::vector<collision_layers_type> collides_with;
        std::vector<body_motion> motion;
        std::vector<std::uint8_t> trigger;

        /// \brief 0 for a row that was freed and may be filled again
        std::vector<std::uint8_t> live;

        std::vector<std::uint32_t> first_part;
        std::vector<std::uint32_t> part_count;
        std::vector<part_record> parts;

        surface_table surfaces;

        [[nodiscard]] std::size_t size() const { return id.size(); }
    };

    /// \brief the scene's tolerances, which every kernel's answers follow
    struct contact_rules final {
        floating_point_type min_swept_speed = 1e-3f;
        floating_point_type penetration_depth_epsilon = 1e-6f;
        floating_point_type normalization_threshold = 1e-6f;
        floating_point_type axis_entry_epsilon = 1e-5f;
        delta_time_type time_of_impact_epsilon = 1e-7f;
    };

    /// \brief everything a kernel reads: the scene keeps the tables, a kernel never changes them
    struct scene_tables final {
        const body_table &statics;
        const body_table &movers;
        contact_rules rules;
    };

    /// \brief a row of the statics table when fixed, else of the movers table
    struct body_ref final {
        std::uint32_t row = 0;
        bool fixed = false;

        bool operator==(const body_ref &) const = default;
    };

    struct pair_request final {
        /// \brief rows of the movers table
        std::span<const std::uint32_t> subjects;

        /// \brief how far ahead each subject looks, one per subject
        std::span<const delta_time_type> times;

        /// \brief whether kinematic movers can be another's candidate
        bool kinematic_candidates = true;
    };

    struct candidate_pair final {
        std::uint32_t subject = 0;
        body_ref other;
    };

    struct pair_test final {
        /// \brief a row of the movers table, moving at its velocity
        std::uint32_t subject = 0;

        body_ref other;

        /// \brief how far ahead to look
        delta_time_type time = 0;

        /// \brief false to hold the other still where it stands
        bool other_moves = true;
    };

    struct pair_hit final {
        /// \brief which test, as an index into the tests given
        std::uint32_t test = 0;

        /// \brief the normal points from the other toward the subject
        overlap hit;
    };

    struct ray_query final {
        vector3_type origin = vector3_type::zero;

        /// \brief unit length
        vector3_type direction = vector3_type::zero;

        floating_point_type max_distance = 0;

        /// \brief a collider is struck only when it is on one of these
        collision_layers_type layers = ALL_COLLISION_LAYERS;

        /// \brief colliders the ray passes through
        std::span<const collider_id_type> ignore;

        bool statics_only = false;
    };

    struct ray_answer final {
        body_ref body;
        floating_point_type distance = 0;
        vector3_type point = vector3_type::zero;

        /// \brief facing back toward the ray's origin
        vector3_type normal = vector3_type::zero;
    };
}

#endif
