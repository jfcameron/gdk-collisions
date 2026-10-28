// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_SHAPE_GEOMETRY_H
#define GDK_COLLISIONS_IMPL_SHAPE_GEOMETRY_H

#include <gdk/collisions/impl_std_tuning.h>
#include <gdk/collisions/impl_shape.h>

namespace gdk::collisions {
    /// \brief closest points, frames and placements shared by contact generation and scene queries
    namespace geometry {
        struct box_bounds final {
            vector3_type min;
            vector3_type max;
        };

        struct segment final {
            vector3_type a;
            vector3_type b;
        };

        /// \brief the mutually closest pair of points between two pieces of geometry
        struct closest_points final {
            vector3_type on_first;
            vector3_type on_second;

            /// \brief set when on_first lies inside the other shape rather than outside it
            bool inside = false;
        };

        struct triangle_points final {
            vector3_type a;
            vector3_type b;
            vector3_type c;
        };

        struct box_frame final {
            vector3_type centre;
            vector3_type half_extents;
            vector3_type axis[3];
        };

        struct plane_frame final {
            vector3_type point;
            vector3_type normal;
        };

        [[nodiscard]] box_bounds bounds_of(const box_shape &aBox, const vector3_type &aPosition);

        /// \brief the interior segment of a shape at a position. Non-capsules degenerate to a point.
        [[nodiscard]] segment segment_of(const shape_type &aShape, const shape_kinematics &aKinematics);

        [[nodiscard]] floating_point_type clamp01(const floating_point_type aValue);

        [[nodiscard]] closest_points closest_points_between_segments(const segment &aFirst,
            const segment &aSecond, const impl_std_tuning &aPolicy);

        [[nodiscard]] triangle_points world_triangle(const triangle_shape &aTriangle,
            const shape_kinematics &aKinematics);

        [[nodiscard]] vector3_type closest_point_on_triangle(const triangle_points &aTriangle,
            const vector3_type &aPoint);

        [[nodiscard]] closest_points closest_points_segment_triangle(const segment &aSegment,
            const triangle_points &aTriangle, const impl_std_tuning &aPolicy);

        [[nodiscard]] vector3_type closest_point_on_box(const box_bounds &aBounds, const vector3_type &aPoint);

        /// \brief inside is set when the segment's nearest point lies within the box
        [[nodiscard]] closest_points closest_points_segment_box(const segment &aSegment,
            const box_bounds &aBounds, const impl_std_tuning &aPolicy);

        [[nodiscard]] box_frame frame_of(const vector3_type &aHalfExtents,
            const vector3_type &aCentre, const quaternion_type &aOrientation);

        /// \brief half the box's width along aAxis
        [[nodiscard]] floating_point_type projected_radius(const box_frame &aBox, const vector3_type &aAxis);

        /// \brief a plane collider's surface: its position, and its local +Y turned into the world
        [[nodiscard]] plane_frame plane_of(const shape_kinematics &aKinematics);
    }
}

#endif
