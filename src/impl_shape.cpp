// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_shape.h>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace gdk::collisions;

namespace {
    [[nodiscard]] vector3_type rotated_bounds_extent(const impl_mesh_data::bounds &aBounds,
        const quaternion_type &aOrientation) {
        auto low = vector3_type::zero;
        auto high = vector3_type::zero;

        for (int corner = 0; corner < 8; ++corner) {
            const auto rotated = aOrientation * vector3_type{
                (corner & 1) ? aBounds.max.x : aBounds.min.x,
                (corner & 2) ? aBounds.max.y : aBounds.min.y,
                (corner & 4) ? aBounds.max.z : aBounds.min.z};

            low = corner ? vector3_type::min(low, rotated) : rotated;
            high = corner ? vector3_type::max(high, rotated) : rotated;
        }

        return vector3_type{
            std::max(std::abs(low.x), std::abs(high.x)),
            std::max(std::abs(low.y), std::abs(high.y)),
            std::max(std::abs(low.z), std::abs(high.z)),
        };
    }
}

shape_type gdk::collisions::shape_of(const part_record &aPart, const impl_mesh_data *const aMesh,
    const impl_heightfield_data *const aHeightfield) {
    switch (aPart.kind) {
        case shape_kind::sphere: return sphere_shape{aPart.size.x};
        case shape_kind::aabb: return box_shape{aPart.size};
        case shape_kind::obb: return obb_shape{aPart.size};
        case shape_kind::capsule: return capsule_shape{aPart.size.x, aPart.size.y};
        case shape_kind::plane: return plane_shape{};
        case shape_kind::mesh: return mesh_shape{aMesh};
        case shape_kind::heightfield: return heightfield_shape{aHeightfield};
    }

    return plane_shape{};
}

collider_part gdk::collisions::decoded(const part_record &aPart, const impl_mesh_data *const aMesh,
    const impl_heightfield_data *const aHeightfield) {
    return collider_part{shape_of(aPart, aMesh, aHeightfield), aPart.offset, aPart.turn};
}

shape_offsets gdk::collisions::shape_bounds(const shape_type &aShape, const quaternion_type &aOrientation) {
    const auto turned = [&](const impl_mesh_data::bounds &aBounds) {
        shape_offsets out{vector3_type::zero, vector3_type::zero};

        for (int corner = 0; corner < 8; ++corner) {
            const auto rotated = aOrientation * vector3_type{
                (corner & 1) ? aBounds.max.x : aBounds.min.x,
                (corner & 2) ? aBounds.max.y : aBounds.min.y,
                (corner & 4) ? aBounds.max.z : aBounds.min.z};

            out.low = corner ? vector3_type::min(out.low, rotated) : rotated;
            out.high = corner ? vector3_type::max(out.high, rotated) : rotated;
        }

        return out;
    };

    if (const auto *const pMesh = std::get_if<mesh_shape>(&aShape)) {
        if (!pMesh->data || pMesh->data->triangle_count() == 0) return {vector3_type::zero, vector3_type::zero};

        return turned(pMesh->data->root_bounds());
    }

    if (const auto *const pHeightfield = std::get_if<heightfield_shape>(&aShape)) {
        if (!pHeightfield->data) return {vector3_type::zero, vector3_type::zero};

        return turned(pHeightfield->data->root_bounds());
    }

    if (const auto *const pTriangle = std::get_if<triangle_shape>(&aShape)) {
        const auto a = aOrientation * pTriangle->a, b = aOrientation * pTriangle->b, c = aOrientation * pTriangle->c;

        return {vector3_type::min(a, vector3_type::min(b, c)), vector3_type::max(a, vector3_type::max(b, c))};
    }

    const auto extents = shape_extents(aShape, aOrientation);

    return {extents * -1.0f, extents};
}

vector3_type gdk::collisions::shape_extents(const shape_type &aShape,
    const quaternion_type &aOrientation) {
    return std::visit(overloaded{
        [](const sphere_shape &aSphere) {
            return vector3_type{aSphere.radius, aSphere.radius, aSphere.radius};
        },
        [](const box_shape &aBox) { return aBox.half_extents; },
        [](const plane_shape &) {
            const auto unbounded = std::numeric_limits<floating_point_type>::infinity();
            return vector3_type{unbounded, unbounded, unbounded};
        },
        [&](const triangle_shape &aTriangle) {
            const auto a = aOrientation * aTriangle.a;
            const auto b = aOrientation * aTriangle.b;
            const auto c = aOrientation * aTriangle.c;
            const auto low = vector3_type::min(a, vector3_type::min(b, c));
            const auto high = vector3_type::max(a, vector3_type::max(b, c));
            return vector3_type{
                std::max(std::abs(low.x), std::abs(high.x)),
                std::max(std::abs(low.y), std::abs(high.y)),
                std::max(std::abs(low.z), std::abs(high.z)),
            };
        },
        [&](const heightfield_shape &aHeightfield) {
            if (!aHeightfield.data) return vector3_type::zero;

            const auto local = aHeightfield.data->root_bounds();
            return rotated_bounds_extent(local, aOrientation);
        },
        [&](const mesh_shape &aMesh) {
            if (!aMesh.data || aMesh.data->triangle_count() == 0) return vector3_type::zero;

            const auto local = aMesh.data->root_bounds();

            return rotated_bounds_extent(local, aOrientation);
        },
        [&](const obb_shape &aBox) {
            const auto x = aOrientation * vector3_type{aBox.half_extents.x, 0, 0};
            const auto y = aOrientation * vector3_type{0, aBox.half_extents.y, 0};
            const auto z = aOrientation * vector3_type{0, 0, aBox.half_extents.z};
            return vector3_type{
                std::abs(x.x) + std::abs(y.x) + std::abs(z.x),
                std::abs(x.y) + std::abs(y.y) + std::abs(z.y),
                std::abs(x.z) + std::abs(y.z) + std::abs(z.z),
            };
        },
        [&](const capsule_shape &aCapsule) {
            const auto half = aOrientation * vector3_type{0, aCapsule.half_height, 0};
            return vector3_type{
                std::abs(half.x) + aCapsule.radius,
                std::abs(half.y) + aCapsule.radius,
                std::abs(half.z) + aCapsule.radius,
            };
        },
    }, aShape);
}
