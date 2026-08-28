// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_RIGID_TRANSFORM_H
#define GDK_COLLISIONS_RIGID_TRANSFORM_H

#include <gdk/collisions/exception.h>
#include <gdk/collisions/types.h>

#include <cmath>
#include <string>

namespace gdk::collisions {
    enum class transform_defect {
        none,
        scaled,    //!< a basis vector is not unit length
        sheared,   //!< two basis vectors are not perpendicular
        reflected  //!< orthonormal but flipped handedness 
    };

    //! how far a basis may stray from orthonormal and still count as rigid
    inline constexpr floating_point_type RIGID_TRANSFORM_TOLERANCE =
        static_cast<floating_point_type>(1e-3);

    /// \brief classify aTransform's upper-left 3x3
    [[nodiscard]] inline transform_defect find_transform_defect(const matrix4x4_type &aTransform,
        const floating_point_type aTolerance = RIGID_TRANSFORM_TOLERANCE) {
        const vector3_type basis[3] = {
            {aTransform.get(0, 0), aTransform.get(0, 1), aTransform.get(0, 2)},
            {aTransform.get(1, 0), aTransform.get(1, 1), aTransform.get(1, 2)},
            {aTransform.get(2, 0), aTransform.get(2, 1), aTransform.get(2, 2)}};

        const auto dot = [](const vector3_type &aLeft, const vector3_type &aRight) {
            return aLeft.x * aRight.x + aLeft.y * aRight.y + aLeft.z * aRight.z;
        };

        for (const auto &column : basis)
            if (std::abs(std::sqrt(dot(column, column)) - 1) > aTolerance)
                return transform_defect::scaled;

        if (std::abs(dot(basis[0], basis[1])) > aTolerance
            || std::abs(dot(basis[0], basis[2])) > aTolerance
            || std::abs(dot(basis[1], basis[2])) > aTolerance)
            return transform_defect::sheared;

        const vector3_type cross{
            basis[1].y * basis[2].z - basis[1].z * basis[2].y,
            basis[1].z * basis[2].x - basis[1].x * basis[2].z,
            basis[1].x * basis[2].y - basis[1].y * basis[2].x};

        if (dot(basis[0], cross) < 0) return transform_defect::reflected;

        return transform_defect::none;
    }

    //! rotation and translation only, no scale, shear or reflection
    [[nodiscard]] inline bool is_rigid_transform(const matrix4x4_type &aTransform,
        const floating_point_type aTolerance = RIGID_TRANSFORM_TOLERANCE) {
        return find_transform_defect(aTransform, aTolerance) == transform_defect::none;
    }

    /// \brief throw unless aTransform is rigid
    /// \exception exception naming aCaller and what was wrong with the transform
    inline void require_rigid_transform(const matrix4x4_type &aTransform, const char *aCaller) {
        const auto defect = find_transform_defect(aTransform);

        if (defect == transform_defect::none) return;

        const auto describe = [defect]() -> const char * {
            switch (defect) {
                case transform_defect::scaled: return "it carries a scale";
                case transform_defect::sheared: return "it carries a shear";
                case transform_defect::reflected: return "it flips handedness";
                default: return "it is not rigid";
            }
        };

        throw exception(std::string("gdk::collision: ").append(aCaller)
            .append(" needs a rigid transform, rotation and translation only, but ")
            .append(describe())
            .append(". A static collider holds its geometry in its own units and the transform"
                " only places it, so a scale would be discarded rather than applied. Scale the"
                " geometry instead: the radius, half extents, or the vertices of the mesh."));
    }
}

#endif
