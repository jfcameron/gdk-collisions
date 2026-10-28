// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_shape_geometry.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <variant>

using namespace gdk::collisions;
using namespace gdk::collisions::geometry;

namespace {
    [[nodiscard]] vector3_type nearest_face_point(const box_bounds &aBounds,
        const vector3_type &aPoint) {
        const auto toLow = aPoint - aBounds.min;
        const auto toHigh = aBounds.max - aPoint;

        auto axis = std::size_t{0};
        auto depth = std::numeric_limits<floating_point_type>::infinity();
        auto atMax = false;

        for (std::size_t i = 0; i < 3; ++i) {
            if (toLow[i] < depth) { depth = toLow[i]; axis = i; atMax = false; }
            if (toHigh[i] < depth) { depth = toHigh[i]; axis = i; atMax = true; }
        }

        auto face = aPoint;
        face[axis] = atMax ? aBounds.max[axis] : aBounds.min[axis];
        return face;
    }

}

namespace gdk::collisions::geometry {
    box_bounds bounds_of(const box_shape &aBox, const vector3_type &aPosition) {
        return box_bounds{aPosition - aBox.half_extents, aPosition + aBox.half_extents};
    }

    segment segment_of(const shape_type &aShape, const shape_kinematics &aKinematics) {
        const auto &position = aKinematics.position;
        return std::visit(overloaded{
            [&](const sphere_shape &) { return segment{position, position}; },
            [&](const box_shape &) { return segment{position, position}; },
            [&](const obb_shape &) { return segment{position, position}; },
            [&](const plane_shape &) { return segment{position, position}; },
            [&](const triangle_shape &) { return segment{position, position}; },
            [&](const mesh_shape &) { return segment{position, position}; },
            [&](const heightfield_shape &) { return segment{position, position}; },
            [&](const capsule_shape &aCapsule) {
                const auto offset = aKinematics.orientation * vector3_type{0, aCapsule.half_height, 0};
                return segment{position - offset, position + offset};
            },
        }, aShape);
    }

    floating_point_type clamp01(const floating_point_type aValue) {
        return std::max(0.0f, std::min(1.0f, aValue));
    }

    closest_points closest_points_between_segments(const segment &aFirst,
        const segment &aSecond, const impl_std_tuning &aPolicy) {
        const auto d1 = aFirst.b - aFirst.a;
        const auto d2 = aSecond.b - aSecond.a;
        const auto r = aFirst.a - aSecond.a;

        const auto a = d1.dot_product(d1);
        const auto e = d2.dot_product(d2);
        const auto f = d2.dot_product(r);
        const auto epsilon = aPolicy.NORMALIZATION_THRESHOLD;

        floating_point_type s = 0;
        floating_point_type t = 0;

        if (a <= epsilon && e <= epsilon) {
        }
        else if (a <= epsilon) {
            t = clamp01(f / e);
        }
        else {
            const auto c = d1.dot_product(r);
            if (e <= epsilon) {
                s = clamp01(-c / a);
            }
            else {
                const auto b = d1.dot_product(d2);
                const auto denominator = a * e - b * b;

                s = denominator != 0.0f ? clamp01((b * f - c * e) / denominator) : 0.0f;
                t = (b * s + f) / e;

                if (t < 0.0f) { t = 0.0f; s = clamp01(-c / a); }
                else if (t > 1.0f) { t = 1.0f; s = clamp01((b - c) / a); }
            }
        }

        return closest_points{aFirst.a + d1 * s, aSecond.a + d2 * t};
    }

    triangle_points world_triangle(const triangle_shape &aTriangle,
        const shape_kinematics &aKinematics) {
        return triangle_points{
            aKinematics.position + aKinematics.orientation * aTriangle.a,
            aKinematics.position + aKinematics.orientation * aTriangle.b,
            aKinematics.position + aKinematics.orientation * aTriangle.c};
    }

    vector3_type closest_point_on_triangle(const triangle_points &aTriangle,
        const vector3_type &aPoint) {
        const auto ab = aTriangle.b - aTriangle.a;
        const auto ac = aTriangle.c - aTriangle.a;
        const auto ap = aPoint - aTriangle.a;

        const auto d1 = ab.dot_product(ap);
        const auto d2 = ac.dot_product(ap);
        if (d1 <= 0.0f && d2 <= 0.0f) return aTriangle.a;

        const auto bp = aPoint - aTriangle.b;
        const auto d3 = ab.dot_product(bp);
        const auto d4 = ac.dot_product(bp);
        if (d3 >= 0.0f && d4 <= d3) return aTriangle.b;

        const auto vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) return aTriangle.a + ab * (d1 / (d1 - d3));

        const auto cp = aPoint - aTriangle.c;
        const auto d5 = ab.dot_product(cp);
        const auto d6 = ac.dot_product(cp);
        if (d6 >= 0.0f && d5 <= d6) return aTriangle.c;

        const auto vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) return aTriangle.a + ac * (d2 / (d2 - d6));

        const auto va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            return aTriangle.b + (aTriangle.c - aTriangle.b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

        const auto denominator = 1.0f / (va + vb + vc);
        return aTriangle.a + ab * (vb * denominator) + ac * (vc * denominator);
    }

    closest_points closest_points_segment_triangle(const segment &aSegment,
        const triangle_points &aTriangle, const impl_std_tuning &aPolicy) {
        closest_points best;
        auto bestDistanceSquared = std::numeric_limits<floating_point_type>::infinity();

        const auto consider = [&](const vector3_type &aOnSegment,
            const vector3_type &aOnTriangle) {
            const auto distanceSquared = (aOnSegment - aOnTriangle).length_squared();
            if (distanceSquared >= bestDistanceSquared) return;
            bestDistanceSquared = distanceSquared;
            best = closest_points{aOnSegment, aOnTriangle, false};
        };

        consider(aSegment.a, closest_point_on_triangle(aTriangle, aSegment.a));
        consider(aSegment.b, closest_point_on_triangle(aTriangle, aSegment.b));

        const segment edges[3] = {
            segment{aTriangle.a, aTriangle.b},
            segment{aTriangle.b, aTriangle.c},
            segment{aTriangle.c, aTriangle.a}};

        for (const auto &edge : edges) {
            const auto pair = closest_points_between_segments(aSegment, edge, aPolicy);
            consider(pair.on_first, pair.on_second);
        }

        return best;
    }

    vector3_type closest_point_on_box(const box_bounds &aBounds,
        const vector3_type &aPoint) {
        return vector3_type{
            std::max(aBounds.min.x, std::min(aPoint.x, aBounds.max.x)),
            std::max(aBounds.min.y, std::min(aPoint.y, aBounds.max.y)),
            std::max(aBounds.min.z, std::min(aPoint.z, aBounds.max.z)),
        };
    }

    closest_points closest_points_segment_box(const segment &aSegment,
        const box_bounds &aBounds, const impl_std_tuning &aPolicy) {
        const auto direction = aSegment.b - aSegment.a;

        const auto point_at = [&aSegment, &direction](const floating_point_type aT) {
            return aSegment.a + direction * aT;
        };

        std::array<floating_point_type, 2 + 6> candidates{};
        std::size_t candidateCount = 0;
        candidates[candidateCount++] = 0;
        candidates[candidateCount++] = 1;

        const auto add_crossing = [&](const floating_point_type aStart,
            const floating_point_type aDelta, const floating_point_type aPlane) {
            if (aDelta == 0) return;                      
            const auto t = (aPlane - aStart) / aDelta;
            if (t > 0 && t < 1) candidates[candidateCount++] = t;
        };

        add_crossing(aSegment.a.x, direction.x, aBounds.min.x);
        add_crossing(aSegment.a.x, direction.x, aBounds.max.x);
        add_crossing(aSegment.a.y, direction.y, aBounds.min.y);
        add_crossing(aSegment.a.y, direction.y, aBounds.max.y);
        add_crossing(aSegment.a.z, direction.z, aBounds.min.z);
        add_crossing(aSegment.a.z, direction.z, aBounds.max.z);

        for (std::size_t i = 1; i < candidateCount; ++i)
            for (auto j = i; j > 0 && candidates[j] < candidates[j - 1]; --j) std::swap(candidates[j], candidates[j - 1]);

        auto bestT = floating_point_type{0};
        auto bestDistanceSquared = std::numeric_limits<floating_point_type>::max();

        const auto consider = [&](const floating_point_type aT) {
            const auto onSegment = point_at(aT);
            const auto squared = (onSegment - closest_point_on_box(aBounds, onSegment)).length_squared();
            if (squared < bestDistanceSquared) { bestDistanceSquared = squared; bestT = aT; }
        };

        for (std::size_t i = 0; i < candidateCount; ++i) consider(candidates[i]);

        for (std::size_t i = 0; i + 1 < candidateCount; ++i) {
            const auto low = candidates[i];
            const auto high = candidates[i + 1];
            if (high <= low) continue;

            const auto middle = point_at((low + high) * 0.5f);

            auto quadraticA = floating_point_type{0};
            auto quadraticB = floating_point_type{0};

            const auto accumulate = [&](const floating_point_type aStart,
                const floating_point_type aDelta, const floating_point_type aMid,
                const floating_point_type aMin, const floating_point_type aMax) {
                if (aMid >= aMin && aMid <= aMax) return;
                const auto face = aMid < aMin ? aMin : aMax;
                quadraticA += aDelta * aDelta;
                quadraticB += 2.0f * aDelta * (aStart - face);
            };

            accumulate(aSegment.a.x, direction.x, middle.x, aBounds.min.x, aBounds.max.x);
            accumulate(aSegment.a.y, direction.y, middle.y, aBounds.min.y, aBounds.max.y);
            accumulate(aSegment.a.z, direction.z, middle.z, aBounds.min.z, aBounds.max.z);

            if (quadraticA <= aPolicy.NORMALIZATION_THRESHOLD) continue;

            const auto vertex = -quadraticB / (2.0f * quadraticA);
            if (vertex > low && vertex < high) consider(vertex);
        }

        const auto onSegment = point_at(bestT);
        const auto onBox = closest_point_on_box(aBounds, onSegment);

        if (onBox.x == onSegment.x && onBox.y == onSegment.y && onBox.z == onSegment.z)
            return closest_points{onSegment, nearest_face_point(aBounds, onSegment), true};

        return closest_points{onSegment, onBox, false};
    }

    box_frame frame_of(const vector3_type &aHalfExtents,
        const vector3_type &aCentre, const quaternion_type &aOrientation) {
        box_frame frame;
        frame.centre = aCentre;
        frame.half_extents = aHalfExtents;
        frame.axis[0] = aOrientation * vector3_type{1, 0, 0};
        frame.axis[1] = aOrientation * vector3_type{0, 1, 0};
        frame.axis[2] = aOrientation * vector3_type{0, 0, 1};
        return frame;
    }

    floating_point_type projected_radius(const box_frame &aBox,
        const vector3_type &aAxis) {
        return std::abs(aAxis.dot_product(aBox.axis[0])) * aBox.half_extents.x
             + std::abs(aAxis.dot_product(aBox.axis[1])) * aBox.half_extents.y
             + std::abs(aAxis.dot_product(aBox.axis[2])) * aBox.half_extents.z;
    }

    plane_frame plane_of(const shape_kinematics &aKinematics) {
        return plane_frame{aKinematics.position,
            aKinematics.orientation * vector3_type{0, 1, 0}};
    }
}
