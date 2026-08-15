// © Joseph Cameron - All Rights Reserved

#include <gdk/collision_exception.h>
#include <gdk/impl_collision_profile.h>
#include <gdk/impl_narrow_phase.h>

#include <variant>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace gdk;

namespace {
    /// \brief axis aligned bounds of a shape sitting at a position
    struct bounds final {
        collision_vector3_type min;
        collision_vector3_type max;
    };

    [[nodiscard]] bounds bounds_of(const box_shape &aBox, const collision_vector3_type &aPosition) {
        return bounds{aPosition - aBox.half_extents, aPosition + aBox.half_extents};
    }

    struct segment final {
        collision_vector3_type a;
        collision_vector3_type b;
    };

    /// \brief the mutually closest pair of points between two pieces of geometry
    struct closest_points final {
        collision_vector3_type on_first;
        collision_vector3_type on_second;

        /// \brief set when on_first lies *inside* the other shape rather than outside it, which
        bool inside = false;
    };

    /// \brief how far apart two shapes are, and where
    struct separation final {
        collision_floating_point_type distance;
        closest_points points;
    };

    /// \brief the interior segment of a shape at a position. Non-capsules degenerate to a point.
    [[nodiscard]] segment segment_of(const collision_shape_type &aShape, const shape_kinematics &aKinematics) {
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
                const auto offset = rotate(aKinematics.orientation,
                    collision_vector3_type{0, aCapsule.half_height, 0});
                return segment{position - offset, position + offset};
            },
        }, aShape);
    }

    [[nodiscard]] collision_floating_point_type clamp01(const collision_floating_point_type aValue) {
        return std::max(0.0f, std::min(1.0f, aValue));
    }

    /// \brief closest points between two segments 
    [[nodiscard]] closest_points closest_points_between_segments(const segment &aFirst,
        const segment &aSecond, const impl_collision_policy &aPolicy) {
        const auto d1 = aFirst.b - aFirst.a;
        const auto d2 = aSecond.b - aSecond.a;
        const auto r = aFirst.a - aSecond.a;

        const auto a = d1.dot_product(d1);
        const auto e = d2.dot_product(d2);
        const auto f = d2.dot_product(r);
        const auto epsilon = aPolicy.NORMALIZATION_THRESHOLD;

        collision_floating_point_type s = 0;
        collision_floating_point_type t = 0;

        if (a <= epsilon && e <= epsilon) {
            // both degenerate: point vs point
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

    struct triangle_points final {
        collision_vector3_type a;
        collision_vector3_type b;
        collision_vector3_type c;
    };

    [[nodiscard]] triangle_points world_triangle(const triangle_shape &aTriangle,
        const shape_kinematics &aKinematics) {
        return triangle_points{
            aKinematics.position + rotate(aKinematics.orientation, aTriangle.a),
            aKinematics.position + rotate(aKinematics.orientation, aTriangle.b),
            aKinematics.position + rotate(aKinematics.orientation, aTriangle.c)};
    }

    /// \brief closest point on a triangle to a point 
    [[nodiscard]] std::optional<collision_vector3_type> corrected_internal_edge_normal(
        const std::uint8_t aInternalEdges, const triangle_points &aTriangle,
        const collision_vector3_type &aContactPoint, const collision_vector3_type &aContactNormal,
        const impl_collision_policy &aPolicy) {
        if (!aInternalEdges) return std::nullopt;

        const auto ab = aTriangle.b - aTriangle.a;
        const auto ac = aTriangle.c - aTriangle.a;

        auto faceNormal = ab.cross_product(ac);
        if (faceNormal.length() <= aPolicy.NORMALIZATION_THRESHOLD) return std::nullopt;
        faceNormal = faceNormal.normal();

        const auto alignment = aContactNormal.dot_product(faceNormal);
        if (std::abs(alignment) >= 1.0f - aPolicy.FACE_CONTACT_COSINE_EPSILON) return std::nullopt;

        const auto ap = aContactPoint - aTriangle.a;

        const auto d00 = ab.dot_product(ab);
        const auto d01 = ab.dot_product(ac);
        const auto d11 = ac.dot_product(ac);
        const auto d20 = ap.dot_product(ab);
        const auto d21 = ap.dot_product(ac);

        const auto denominator = d00 * d11 - d01 * d01;
        if (std::abs(denominator) <= aPolicy.NORMALIZATION_THRESHOLD) return std::nullopt;   // degenerate

        const auto v = (d11 * d20 - d01 * d21) / denominator;
        const auto w = (d00 * d21 - d01 * d20) / denominator;
        const auto u = 1.0f - v - w;

        const bool on[3] = {
            w <= aPolicy.BARYCENTRIC_EDGE_EPSILON,     // edge a->b is opposite c
            u <= aPolicy.BARYCENTRIC_EDGE_EPSILON,     // edge b->c is opposite a
            v <= aPolicy.BARYCENTRIC_EDGE_EPSILON,     // edge c->a is opposite b
        };

        auto touched = false;

        for (int e = 0; e < 3; ++e) {
            if (!on[e]) continue;
            if (!(aInternalEdges & (1u << e))) return std::nullopt;
            touched = true;
        }

        if (!touched) return std::nullopt;

        return alignment >= 0 ? faceNormal : faceNormal * -1.0f;
    }

    [[nodiscard]] collision_vector3_type closest_point_on_triangle(const triangle_points &aTriangle,
        const collision_vector3_type &aPoint) {
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

    /// \brief closest points between a segment and a triangle.
    [[nodiscard]] closest_points closest_points_segment_triangle(const segment &aSegment,
        const triangle_points &aTriangle, const impl_collision_policy &aPolicy) {
        closest_points best;
        auto bestDistanceSquared = std::numeric_limits<collision_floating_point_type>::infinity();

        const auto consider = [&](const collision_vector3_type &aOnSegment,
            const collision_vector3_type &aOnTriangle) {
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

    [[nodiscard]] collision_vector3_type closest_point_on_box(const bounds &aBounds,
        const collision_vector3_type &aPoint) {
        return collision_vector3_type{
            std::max(aBounds.min.x, std::min(aPoint.x, aBounds.max.x)),
            std::max(aBounds.min.y, std::min(aPoint.y, aBounds.max.y)),
            std::max(aBounds.min.z, std::min(aPoint.z, aBounds.max.z)),
        };
    }

    /// \brief the nearest point on a box's *surface* to a point inside it.
    [[nodiscard]] collision_vector3_type nearest_face_point(const bounds &aBounds,
        const collision_vector3_type &aPoint) {
        const auto toLow = aPoint - aBounds.min;
        const auto toHigh = aBounds.max - aPoint;

        auto axis = std::size_t{0};
        auto depth = std::numeric_limits<collision_floating_point_type>::infinity();
        auto atMax = false;

        for (std::size_t i = 0; i < 3; ++i) {
            if (toLow[i] < depth) { depth = toLow[i]; axis = i; atMax = false; }
            if (toHigh[i] < depth) { depth = toHigh[i]; axis = i; atMax = true; }
        }

        auto face = aPoint;
        face[axis] = atMax ? aBounds.max[axis] : aBounds.min[axis];
        return face;
    }

    [[nodiscard]] closest_points closest_points_segment_box(const segment &aSegment,
        const bounds &aBounds, const impl_collision_policy &aPolicy) {
        const auto direction = aSegment.b - aSegment.a;

        const auto point_at = [&aSegment, &direction](const collision_floating_point_type aT) {
            return aSegment.a + direction * aT;
        };

        std::array<collision_floating_point_type, 8> candidates{};
        std::size_t candidateCount = 0;
        candidates[candidateCount++] = 0;
        candidates[candidateCount++] = 1;

        const auto add_crossing = [&](const collision_floating_point_type aStart,
            const collision_floating_point_type aDelta, const collision_floating_point_type aPlane) {
            if (aDelta == 0) return;                      // parallel to this slab: never crosses it
            const auto t = (aPlane - aStart) / aDelta;
            if (t > 0 && t < 1) candidates[candidateCount++] = t;
        };

        add_crossing(aSegment.a.x, direction.x, aBounds.min.x);
        add_crossing(aSegment.a.x, direction.x, aBounds.max.x);
        add_crossing(aSegment.a.y, direction.y, aBounds.min.y);
        add_crossing(aSegment.a.y, direction.y, aBounds.max.y);
        add_crossing(aSegment.a.z, direction.z, aBounds.min.z);
        add_crossing(aSegment.a.z, direction.z, aBounds.max.z);

        std::sort(candidates.begin(), candidates.begin() + candidateCount);

        auto bestT = collision_floating_point_type{0};
        auto bestDistanceSquared = std::numeric_limits<collision_floating_point_type>::max();

        const auto consider = [&](const collision_floating_point_type aT) {
            const auto onSegment = point_at(aT);
            const auto squared = (onSegment - closest_point_on_box(aBounds, onSegment)).length_squared();
            if (squared < bestDistanceSquared) { bestDistanceSquared = squared; bestT = aT; }
        };

        for (std::size_t i = 0; i < candidateCount; ++i) consider(candidates[i]);

        for (std::size_t i = 0; i + 1 < candidateCount; ++i) {
            const auto low = candidates[i];
            const auto high = candidates[i + 1];
            if (high <= low) continue;

            // The clamp pattern is constant across the interval, so reading it at the midpoint reads
            // it for the whole interval.
            const auto middle = point_at((low + high) * 0.5f);

            auto quadraticA = collision_floating_point_type{0};
            auto quadraticB = collision_floating_point_type{0};

            const auto accumulate = [&](const collision_floating_point_type aStart,
                const collision_floating_point_type aDelta, const collision_floating_point_type aMid,
                const collision_floating_point_type aMin, const collision_floating_point_type aMax) {
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

        if ((onBox - onSegment).length_squared() <= aPolicy.NORMALIZATION_THRESHOLD)
            return closest_points{onSegment, nearest_face_point(aBounds, onSegment), true};

        return closest_points{onSegment, onBox, false};
    }

    [[nodiscard]] separation separation_at(
        const collision_shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const collision_shape_type &aOtherShape, const shape_kinematics &aOther,
        const impl_collision_policy &aPolicy) {
        const auto subjectSegment = segment_of(aSubjectShape, aSubject);

        if (std::holds_alternative<obb_shape>(aOtherShape)) {
            const auto inverse = aOther.orientation.inverse_unit();
            const segment localSegment{
                rotate(inverse, subjectSegment.a - aOther.position),
                rotate(inverse, subjectSegment.b - aOther.position)};

            const auto &extents = std::get<obb_shape>(aOtherShape).half_extents;
            const bounds localBox{extents * -1.0f, extents};

            const auto local = closest_points_segment_box(localSegment, localBox, aPolicy);
            const closest_points world{
                rotate(aOther.orientation, local.on_first) + aOther.position,
                rotate(aOther.orientation, local.on_second) + aOther.position,
                local.inside};

            const auto distance = (world.on_first - world.on_second).length();
            return separation{world.inside ? -distance : distance, world};
        }

        if (std::holds_alternative<triangle_shape>(aOtherShape)) {
            const auto points = closest_points_segment_triangle(subjectSegment,
                world_triangle(std::get<triangle_shape>(aOtherShape), aOther), aPolicy);
            return separation{(points.on_first - points.on_second).length(), points};
        }

        const auto points = std::holds_alternative<box_shape>(aOtherShape)
            ? closest_points_segment_box(subjectSegment,
                bounds_of(std::get<box_shape>(aOtherShape), aOther.position), aPolicy)
            : closest_points_between_segments(subjectSegment, segment_of(aOtherShape, aOther), aPolicy);

        const auto distance = (points.on_first - points.on_second).length();
        return separation{points.inside ? -distance : distance, points};
    }

    [[nodiscard]] collision_floating_point_type radius_of(const collision_shape_type &aShape) {
        return std::visit(overloaded{
            [](const sphere_shape &aSphere) { return aSphere.radius; },
            [](const box_shape &) { return collision_floating_point_type{0}; },
            [](const obb_shape &) { return collision_floating_point_type{0}; },
            [](const triangle_shape &) { return collision_floating_point_type{0}; },
            [](const mesh_shape &) { return collision_floating_point_type{0}; },
            [](const heightfield_shape &) { return collision_floating_point_type{0}; },
            [](const plane_shape &) { return collision_floating_point_type{0}; },
            [](const capsule_shape &aCapsule) { return aCapsule.radius; },
        }, aShape);
    }

    /// \brief generic swept test for shapes whose separation is a distance query.
    [[nodiscard]] std::optional<overlap> swept_by_separation(
        const collision_shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const collision_shape_type &aOtherShape, const shape_kinematics &aOther,
        const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto contactRadius = radius_of(aSubjectShape) + radius_of(aOtherShape);
        const auto relativeVelocity = aSubject.velocity - aOther.velocity;
        const auto relativeSpeed = relativeVelocity.length();

        const auto sample_at = [&](const collision_delta_time_type aTime) {
            return separation_at(
                aSubjectShape, {aSubject.position + aSubject.velocity * aTime, aSubject.velocity, aSubject.orientation},
                aOtherShape, {aOther.position + aOther.velocity * aTime, aOther.velocity, aOther.orientation},
                aPolicy);
        };

        const auto contact_at = [&](const collision_delta_time_type aTime, const separation &aSample) {
            const auto delta = aSample.points.inside
                ? aSample.points.on_second - aSample.points.on_first
                : aSample.points.on_first - aSample.points.on_second;

            const auto length = delta.length();
            const auto normal = length > aPolicy.NORMALIZATION_THRESHOLD
                ? delta * (1.0f / length)
                : (relativeSpeed > aPolicy.NORMALIZATION_THRESHOLD
                    ? relativeVelocity.normal() * -1.0f
                    : collision_vector3_type{0, 1, 0});

            overlap result;
            result.entry_time = aTime;
            result.exit_time = aDeltaTime;
            result.penetration = std::max(0.0f, contactRadius - aSample.distance);
            result.contact_normal = normal;
            result.contact_point = aSample.points.on_second;
            return result;
        };

        collision_delta_time_type time = 0;
        auto previousGap = std::numeric_limits<collision_floating_point_type>::infinity();

        for (unsigned short int step = 0; step <= aPolicy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS; ++step) {
            GDK_COLLISION_PROFILE_COUNT(advancement_iterations, 1);

            const auto sample = sample_at(time);
            const auto gap = sample.distance - contactRadius;

            if (gap <= aPolicy.PENETRATION_DEPTH_EPSILON) return contact_at(time, sample);

            if (relativeSpeed <= aPolicy.NORMALIZATION_THRESHOLD) return std::nullopt;

            if (gap >= previousGap) return std::nullopt;
            previousGap = gap;

            const auto next = time + gap / relativeSpeed;
            if (next > aDeltaTime) return std::nullopt;

            if (step == aPolicy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS) return contact_at(next, sample_at(next));

            time = next;
        }

        return std::nullopt;
    }

    struct box_frame final {
        collision_vector3_type centre;
        collision_vector3_type half_extents;
        collision_vector3_type axis[3];
    };

    [[nodiscard]] box_frame frame_of(const collision_vector3_type &aHalfExtents,
        const collision_vector3_type &aCentre, const collision_quaternion_type &aOrientation) {
        box_frame frame;
        frame.centre = aCentre;
        frame.half_extents = aHalfExtents;
        frame.axis[0] = rotate(aOrientation, collision_vector3_type{1, 0, 0});
        frame.axis[1] = rotate(aOrientation, collision_vector3_type{0, 1, 0});
        frame.axis[2] = rotate(aOrientation, collision_vector3_type{0, 0, 1});
        return frame;
    }

    [[nodiscard]] collision_floating_point_type projected_radius(const box_frame &aBox,
        const collision_vector3_type &aAxis) {
        return std::abs(aAxis.dot_product(aBox.axis[0])) * aBox.half_extents.x
             + std::abs(aAxis.dot_product(aBox.axis[1])) * aBox.half_extents.y
             + std::abs(aAxis.dot_product(aBox.axis[2])) * aBox.half_extents.z;
    }

    struct contact_sample final {
        collision_floating_point_type gap; ///< negative when interpenetrating
        collision_vector3_type normal;     ///< from the other body toward the subject
        collision_vector3_type point;
    };

    [[nodiscard]] contact_sample separating_axis(const box_frame &aSubject, const box_frame &aOther,
        const impl_collision_policy &aPolicy) {
        const auto delta = aSubject.centre - aOther.centre;

        contact_sample best;
        best.gap = -std::numeric_limits<collision_floating_point_type>::infinity();
        best.normal = collision_vector3_type::zero;

        const auto consider = [&](collision_vector3_type aAxis) {
            const auto lengthSquared = aAxis.length_squared();
            if (lengthSquared < aPolicy.NORMALIZATION_THRESHOLD) return true;

            aAxis = aAxis * (1.0f / std::sqrt(lengthSquared));

            const auto centreDistance = delta.dot_product(aAxis);
            const auto gap = std::abs(centreDistance)
                - (projected_radius(aSubject, aAxis) + projected_radius(aOther, aAxis));

            if (gap > best.gap + aPolicy.PENETRATION_DEPTH_EPSILON) {
                best.gap = gap;
                best.normal = centreDistance < 0.0f ? aAxis * -1.0f : aAxis;
            }
            return gap <= 0.0f;
        };

        for (int i = 0; i < 3; ++i) if (!consider(aSubject.axis[i])) return best;
        for (int i = 0; i < 3; ++i) if (!consider(aOther.axis[i])) return best;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (!consider(aSubject.axis[i].cross_product(aOther.axis[j]))) return best;

        return best;
    }

    template <typename sample_at_type>
    [[nodiscard]] std::optional<overlap> conservative_sweep(sample_at_type aSampleAt,
        const collision_floating_point_type aRelativeSpeed,
        const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto contact_at = [&](const collision_delta_time_type aTime, const contact_sample &aSample) {
            overlap result;
            result.entry_time = aTime;
            result.exit_time = aDeltaTime;
            result.penetration = std::max(0.0f, -aSample.gap);
            result.contact_normal = aSample.normal;
            result.contact_point = aSample.point;
            return result;
        };

        collision_delta_time_type time = 0;
        auto previousGap = std::numeric_limits<collision_floating_point_type>::infinity();

        for (unsigned short int step = 0; step <= aPolicy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS; ++step) {
            GDK_COLLISION_PROFILE_COUNT(advancement_iterations, 1);

            const auto sample = aSampleAt(time);

            if (sample.gap <= aPolicy.PENETRATION_DEPTH_EPSILON) return contact_at(time, sample);

            if (aRelativeSpeed <= aPolicy.NORMALIZATION_THRESHOLD) return std::nullopt;

            if (sample.gap >= previousGap) return std::nullopt;
            previousGap = sample.gap;

            const auto next = time + sample.gap / aRelativeSpeed;
            if (next > aDeltaTime) return std::nullopt;

            if (step == aPolicy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS) return contact_at(next, aSampleAt(next));

            time = next;
        }

        return std::nullopt;
    }

    [[nodiscard]] contact_sample separating_axis_triangle(const box_frame &aBox,
        const triangle_points &aTriangle, const impl_collision_policy &aPolicy) {
        contact_sample best;
        best.gap = -std::numeric_limits<collision_floating_point_type>::infinity();
        best.normal = collision_vector3_type::zero;

        const auto consider = [&](collision_vector3_type aAxis) {
            const auto lengthSquared = aAxis.length_squared();
            if (lengthSquared < aPolicy.NORMALIZATION_THRESHOLD) return true;

            aAxis = aAxis * (1.0f / std::sqrt(lengthSquared));

            const auto pa = aTriangle.a.dot_product(aAxis);
            const auto pb = aTriangle.b.dot_product(aAxis);
            const auto pc = aTriangle.c.dot_product(aAxis);
            const auto triangleLow = std::min({pa, pb, pc});
            const auto triangleHigh = std::max({pa, pb, pc});

            const auto centre = aBox.centre.dot_product(aAxis);
            const auto radius = projected_radius(aBox, aAxis);

            const auto below = triangleLow - (centre + radius);
            const auto above = (centre - radius) - triangleHigh;
            const auto gap = std::max(below, above);
            const auto normal = above > below ? aAxis : aAxis * -1.0f;

            if (gap > best.gap + aPolicy.PENETRATION_DEPTH_EPSILON) {
                best.gap = gap;
                best.normal = normal;
            }
            return gap <= 0.0f;
        };

        const collision_vector3_type edges[3] = {
            aTriangle.b - aTriangle.a,
            aTriangle.c - aTriangle.b,
            aTriangle.a - aTriangle.c};

        if (!consider(edges[0].cross_product(edges[1]))) return best;   
        for (int i = 0; i < 3; ++i) if (!consider(aBox.axis[i])) return best;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (!consider(aBox.axis[i].cross_product(edges[j]))) return best;

        return best;
    }

    [[nodiscard]] std::optional<overlap> swept_box_triangle(
        const collision_vector3_type &aBoxExtents, const collision_quaternion_type &aBoxOrientation,
        const shape_kinematics &aBox, const triangle_shape &aTriangleShape,
        const shape_kinematics &aTriangle, const collision_delta_time_type aDeltaTime,
        const impl_collision_policy &aPolicy) {
        const auto relativeSpeed = (aBox.velocity - aTriangle.velocity).length();

        return conservative_sweep([&](const collision_delta_time_type aTime) {
            const auto box = frame_of(aBoxExtents, aBox.position + aBox.velocity * aTime, aBoxOrientation);
            const auto triangle = world_triangle(aTriangleShape,
                {aTriangle.position + aTriangle.velocity * aTime, aTriangle.velocity, aTriangle.orientation});

            auto sample = separating_axis_triangle(box, triangle, aPolicy);
            sample.point = closest_point_on_triangle(triangle, box.centre);
            return sample;
        }, relativeSpeed, aDeltaTime, aPolicy);
    }

    [[nodiscard]] std::optional<overlap> swept_box_pair(
        const collision_vector3_type &aSubjectExtents, const collision_quaternion_type &aSubjectOrientation,
        const shape_kinematics &aSubject,
        const collision_vector3_type &aOtherExtents, const collision_quaternion_type &aOtherOrientation,
        const shape_kinematics &aOther,
        const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto relativeSpeed = (aSubject.velocity - aOther.velocity).length();

        return conservative_sweep([&](const collision_delta_time_type aTime) {
            const auto subject = frame_of(aSubjectExtents,
                aSubject.position + aSubject.velocity * aTime, aSubjectOrientation);
            const auto other = frame_of(aOtherExtents,
                aOther.position + aOther.velocity * aTime, aOtherOrientation);

            auto sample = separating_axis(subject, other, aPolicy);
            sample.point = other.centre + sample.normal * projected_radius(other, sample.normal);
            return sample;
        }, relativeSpeed, aDeltaTime, aPolicy);
    }

    struct plane_frame final {
        collision_vector3_type point;
        collision_vector3_type normal;
    };

    [[nodiscard]] plane_frame plane_of(const shape_kinematics &aKinematics) {
        return plane_frame{aKinematics.position,
            rotate(aKinematics.orientation, collision_vector3_type{0, 1, 0})};
    }

    [[nodiscard]] collision_floating_point_type gap_to_plane(const collision_shape_type &aShape,
        const shape_kinematics &aKinematics, const plane_frame &aPlane) {
        const auto centreHeight = (aKinematics.position - aPlane.point).dot_product(aPlane.normal);

        return std::visit(overloaded{
            [&](const sphere_shape &aSphere) { return centreHeight - aSphere.radius; },
            [&](const box_shape &aBox) {
                return centreHeight - projected_radius(
                    frame_of(aBox.half_extents, aKinematics.position, collision_quaternion_type::identity),
                    aPlane.normal);
            },
            [&](const obb_shape &aBox) {
                return centreHeight - projected_radius(
                    frame_of(aBox.half_extents, aKinematics.position, aKinematics.orientation),
                    aPlane.normal);
            },
            [&](const capsule_shape &aCapsule) {
                const auto offset = rotate(aKinematics.orientation,
                    collision_vector3_type{0, aCapsule.half_height, 0}).dot_product(aPlane.normal);
                return centreHeight - std::abs(offset) - aCapsule.radius;
            },
            [&](const triangle_shape &aTriangle) {
                const auto world = world_triangle(aTriangle, aKinematics);
                return std::min({
                    (world.a - aPlane.point).dot_product(aPlane.normal),
                    (world.b - aPlane.point).dot_product(aPlane.normal),
                    (world.c - aPlane.point).dot_product(aPlane.normal)});
            },
            [&](const heightfield_shape &) {
                return std::numeric_limits<collision_floating_point_type>::max();
            },
            [&](const mesh_shape &) {
                return std::numeric_limits<collision_floating_point_type>::max();
            },
            [&](const plane_shape &) -> collision_floating_point_type {
                throw collision_exception("gdk::collision: half-space against half-space is not a "
                    "supported pair -- two infinite planes have no bounded overlap to report");
            },
        }, aShape);
    }

    [[nodiscard]] std::optional<overlap> vs_plane(const collision_shape_type &aSubjectShape,
        const shape_kinematics &aSubject, const shape_kinematics &aPlaneKinematics,
        const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto plane = plane_of(aPlaneKinematics);
        const auto gap = gap_to_plane(aSubjectShape, aSubject, plane);

        const auto contact_at = [&](const collision_delta_time_type aTime,
            const collision_floating_point_type aGap) {
            overlap result;
            result.entry_time = aTime;
            result.exit_time = aDeltaTime;
            result.penetration = std::max(0.0f, -aGap);
            result.contact_normal = plane.normal;
            const auto centre = aSubject.position + aSubject.velocity * aTime;
            result.contact_point = centre - plane.normal * (centre - plane.point).dot_product(plane.normal);
            return result;
        };

        if (gap <= aPolicy.PENETRATION_DEPTH_EPSILON) return contact_at(0.0f, gap);

        const auto rate = (aSubject.velocity - aPlaneKinematics.velocity).dot_product(plane.normal);
        if (rate >= -aPolicy.MIN_SWEPT_SPEED) return std::nullopt;

        const auto time = gap / -rate;
        if (time > aDeltaTime) return std::nullopt;

        return contact_at(time, 0.0f);
    }

    std::optional<overlap> sphere_vs_sphere(const sphere_shape &aSubjectShape, const shape_kinematics &aSubject,
        const sphere_shape &aOtherShape, const shape_kinematics &aOther,
        const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto p = aSubject.position - aOther.position;
        const auto v = aSubject.velocity - aOther.velocity;
        const auto r = aSubjectShape.radius + aOtherShape.radius;
        const auto a = v.dot_product(v);

        if (a < aPolicy.MIN_SWEPT_SPEED_SQUARED) {
            const auto distSquared = p.dot_product(p);
            if (distSquared > r * r) return std::nullopt;

            const auto distance = std::sqrt(distSquared);
            const auto normal = distance > aPolicy.NORMALIZATION_THRESHOLD
                ? p * (1.0f / distance)
                : collision_vector3_type{1.0f, 0.0f, 0.0f};

            overlap result;
            result.entry_time = 0.0f;
            result.exit_time = aDeltaTime;
            result.penetration = r - distance;
            result.contact_normal = normal;
            result.contact_point = aOther.position + normal * aOtherShape.radius;
            return result;
        }

        const auto b = 2.0f * p.dot_product(v);
        const auto c = p.dot_product(p) - r * r;

        const auto discriminant = b * b - 4 * a * c;
        if (discriminant < -aPolicy.PENETRATION_DEPTH_EPSILON) return std::nullopt;

        const auto sqrtDisc = std::sqrt(discriminant);
        const auto t1 = (-b - sqrtDisc) / (2 * a);
        const auto t2 = (-b + sqrtDisc) / (2 * a);

        const auto entryTime = std::max(0.0f, t1);
        const auto exitTime = std::min(aDeltaTime, t2);

        if (entryTime > exitTime || exitTime < 0.0f || entryTime > aDeltaTime) return std::nullopt;

        const auto separationAtImpact = p + v * entryTime;
        const auto distanceAtImpact = separationAtImpact.length();
        const auto normal = distanceAtImpact > aPolicy.NORMALIZATION_THRESHOLD
            ? separationAtImpact * (1.0f / distanceAtImpact)
            : (v.length_squared() > aPolicy.MIN_SWEPT_SPEED_SQUARED
                ? v.normal() * -1.0f
                : collision_vector3_type{1.0f, 0.0f, 0.0f});

        overlap result;
        result.entry_time = entryTime;
        result.exit_time = exitTime;
        result.penetration = std::max(0.0f, r - distanceAtImpact);
        result.contact_normal = normal;
        result.contact_point = (aOther.position + aOther.velocity * entryTime) + normal * aOtherShape.radius;
        return result;
    }


    std::optional<overlap> box_vs_box(const box_shape &aSubjectShape, const shape_kinematics &aSubject,
        const box_shape &aOtherShape, const shape_kinematics &aOther,
        const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto thisBroadPhaseBounds = bounds_of(aSubjectShape, aSubject.position);
        const auto otherBroadPhaseBounds = bounds_of(aOtherShape, aOther.position);
        const auto relativeVelocity = aSubject.velocity - aOther.velocity;

        const auto contact_at = [&](const std::vector<overlap::contact_axis> &aAxes,
            const collision_delta_time_type aToi) {
            const auto boundsA = bounds_of(aSubjectShape, aSubject.position + aSubject.velocity * aToi);
            const auto boundsB = bounds_of(aOtherShape, aOther.position + aOther.velocity * aToi);

            struct contact final {
                collision_vector3_type normal = collision_vector3_type::zero;
                collision_floating_point_type penetration = std::numeric_limits<collision_floating_point_type>::infinity();
                collision_vector3_type point = collision_vector3_type::zero;
            } best;

            for (const auto axis : aAxes) {
                const auto i = static_cast<std::size_t>(axis);
                const auto penetration = std::min(boundsA.max[i] - boundsB.min[i],
                    boundsB.max[i] - boundsA.min[i]);

                if (penetration >= best.penetration) continue;

                const auto vi = relativeVelocity[i];
                const auto sign = std::abs(vi) >= aPolicy.MIN_SWEPT_SPEED
                    ? (vi > 0.0f ? -1.0f : 1.0f)
                    : ((boundsA.min[i] + boundsA.max[i]) < (boundsB.min[i] + boundsB.max[i]) ? -1.0f : 1.0f);

                auto normal = collision_vector3_type::zero;
                normal[i] = sign;

                best.penetration = penetration;
                best.normal = normal;
            }

            const auto centre = aSubject.position + aSubject.velocity * aToi;
            best.point = collision_vector3_type{
                std::max(boundsB.min.x, std::min(centre.x, boundsB.max.x)),
                std::max(boundsB.min.y, std::min(centre.y, boundsB.max.y)),
                std::max(boundsB.min.z, std::min(centre.z, boundsB.max.z)),
            };
            return best;
        };

        auto anyMoving = false;
        auto entryMax = -std::numeric_limits<collision_floating_point_type>::infinity();
        auto exitMin = std::numeric_limits<collision_floating_point_type>::infinity();
        auto minPenetration = std::numeric_limits<collision_floating_point_type>::infinity();
        auto staticAxis = overlap::contact_axis::X;
        std::vector<overlap::contact_axis> toiAxes;

        for (std::size_t i(0); i < 3; ++i) {
            const auto vi = relativeVelocity[i];

            if (std::abs(vi) < aPolicy.MIN_SWEPT_SPEED) {
                if (thisBroadPhaseBounds.max[i] < otherBroadPhaseBounds.min[i] || 
                    thisBroadPhaseBounds.min[i] > otherBroadPhaseBounds.max[i])
                    return std::nullopt;

                const auto overlap =
                    std::min(thisBroadPhaseBounds.max[i], otherBroadPhaseBounds.max[i]) -
                    std::max(thisBroadPhaseBounds.min[i], otherBroadPhaseBounds.min[i]);
                if (overlap < minPenetration) {
                    minPenetration = overlap;
                    staticAxis = static_cast<overlap::contact_axis>(i);
                }
                continue;
            }

            anyMoving = true;

            const collision_floating_point_type invVi = 1.0f / vi;
            const collision_floating_point_type t1 = (otherBroadPhaseBounds.min[i] - thisBroadPhaseBounds.max[i]) * invVi;
            const collision_floating_point_type t2 = (otherBroadPhaseBounds.max[i] - thisBroadPhaseBounds.min[i]) * invVi;

            const collision_floating_point_type axisEntry = std::min(t1, t2);
            const collision_floating_point_type axisExit  = std::max(t1, t2);

            if (axisEntry > entryMax + aPolicy.AXIS_ENTRY_EPSILON) {
                entryMax = axisEntry;
                toiAxes.clear();
                toiAxes.push_back(static_cast<overlap::contact_axis>(i));
            } else if (std::abs(axisEntry - entryMax) <= aPolicy.AXIS_ENTRY_EPSILON) {
                toiAxes.push_back(static_cast<overlap::contact_axis>(i));
            }

            exitMin = std::min(exitMin, axisExit);

            if (entryMax > exitMin) return std::nullopt;
        }

        if (!anyMoving) {
            if (!std::isfinite(minPenetration)) return std::nullopt;

            const auto contact = contact_at({staticAxis}, 0.0f);
            overlap result;
            result.entry_time = 0.0f;
            result.exit_time = aDeltaTime;
            result.penetration = contact.penetration;
            result.contact_normal = contact.normal;
            result.contact_point = contact.point;
            return result;
        }

        if (exitMin < 0.0f) return std::nullopt;

        {
            std::vector<overlap::contact_axis> valid;
            valid.reserve(toiAxes.size());
            for (auto axis : toiAxes) {
                const auto i = static_cast<std::size_t>(axis);
                const auto vi = relativeVelocity[i];
                const collision_floating_point_type invVi = 1.0f / vi;
                const collision_floating_point_type t1 = (otherBroadPhaseBounds.min[i] - thisBroadPhaseBounds.max[i]) * invVi;
                const collision_floating_point_type t2 = (otherBroadPhaseBounds.max[i] - thisBroadPhaseBounds.min[i]) * invVi;
                const collision_floating_point_type axisEntry = std::min(t1, t2);
                const collision_floating_point_type axisExit  = std::max(t1, t2);

                if (axisEntry <= exitMin + aPolicy.AXIS_ENTRY_EPSILON && 
                    axisExit >= entryMax - aPolicy.AXIS_ENTRY_EPSILON) 
                    valid.push_back(axis);
            }
            toiAxes.swap(valid);
        }

        if (toiAxes.empty()) return std::nullopt;

        const auto toi = std::min(aDeltaTime, std::max(entryMax, 0.0f));

        if (toi > aDeltaTime) return std::nullopt;

        const auto contact = toi <= aPolicy.TIME_OF_IMPACT_EPSILON
            ? contact_at({overlap::contact_axis::X, overlap::contact_axis::Y, overlap::contact_axis::Z}, toi)
            : contact_at(toiAxes, toi);
        overlap result;
        result.entry_time = toi;
        result.exit_time = std::min(aDeltaTime, exitMin);
        result.penetration = contact.penetration;
        result.contact_normal = contact.normal;
        result.contact_point = contact.point;
        return result;
    }
}

namespace {
    [[nodiscard]] collision_vector3_type rotated_bounds_extent(const impl_mesh_data::bounds &aBounds,
        const collision_quaternion_type &aOrientation) {
        auto low = collision_vector3_type::zero;
        auto high = collision_vector3_type::zero;

        for (int corner = 0; corner < 8; ++corner) {
            const auto rotated = rotate(aOrientation, collision_vector3_type{
                (corner & 1) ? aBounds.max.x : aBounds.min.x,
                (corner & 2) ? aBounds.max.y : aBounds.min.y,
                (corner & 4) ? aBounds.max.z : aBounds.min.z});

            low = corner ? collision_vector3_type::min(low, rotated) : rotated;
            high = corner ? collision_vector3_type::max(high, rotated) : rotated;
        }

        return collision_vector3_type{
            std::max(std::abs(low.x), std::abs(high.x)),
            std::max(std::abs(low.y), std::abs(high.y)),
            std::max(std::abs(low.z), std::abs(high.z)),
        };
    }
}

collision_vector3_type gdk::shape_extents(const collision_shape_type &aShape,
    const collision_quaternion_type &aOrientation) {
    return std::visit(overloaded{
        [](const sphere_shape &aSphere) {
            return collision_vector3_type{aSphere.radius, aSphere.radius, aSphere.radius};
        },
        [](const box_shape &aBox) { return aBox.half_extents; },
        [](const plane_shape &) {
            const auto unbounded = std::numeric_limits<collision_floating_point_type>::infinity();
            return collision_vector3_type{unbounded, unbounded, unbounded};
        },
        [&](const triangle_shape &aTriangle) {
            const auto a = rotate(aOrientation, aTriangle.a);
            const auto b = rotate(aOrientation, aTriangle.b);
            const auto c = rotate(aOrientation, aTriangle.c);
            const auto low = collision_vector3_type::min(a, collision_vector3_type::min(b, c));
            const auto high = collision_vector3_type::max(a, collision_vector3_type::max(b, c));
            return collision_vector3_type{
                std::max(std::abs(low.x), std::abs(high.x)),
                std::max(std::abs(low.y), std::abs(high.y)),
                std::max(std::abs(low.z), std::abs(high.z)),
            };
        },
        [&](const heightfield_shape &aHeightfield) {
            if (!aHeightfield.data) return collision_vector3_type::zero;

            const auto local = aHeightfield.data->root_bounds();
            return rotated_bounds_extent(local, aOrientation);
        },
        [&](const mesh_shape &aMesh) {
            if (!aMesh.data || aMesh.data->triangle_count() == 0) return collision_vector3_type::zero;

            const auto local = aMesh.data->root_bounds();

            return rotated_bounds_extent(local, aOrientation);
        },
        [&](const obb_shape &aBox) {
            const auto frame = frame_of(aBox.half_extents, collision_vector3_type::zero, aOrientation);
            return collision_vector3_type{
                projected_radius(frame, collision_vector3_type{1, 0, 0}),
                projected_radius(frame, collision_vector3_type{0, 1, 0}),
                projected_radius(frame, collision_vector3_type{0, 0, 1}),
            };
        },
        [&](const capsule_shape &aCapsule) {
            const auto half = rotate(aOrientation, collision_vector3_type{0, aCapsule.half_height, 0});
            return collision_vector3_type{
                std::abs(half.x) + aCapsule.radius,
                std::abs(half.y) + aCapsule.radius,
                std::abs(half.z) + aCapsule.radius,
            };
        },
    }, aShape);
}

namespace {
    [[nodiscard]] bool is_closing_contact(const overlap &aOverlap, const shape_kinematics &aSubject,
        const shape_kinematics &aOther, const impl_collision_policy &aPolicy) {
        return aOverlap.penetration > aPolicy.PENETRATION_DEPTH_EPSILON
            || is_closing(aOverlap, aSubject, aOther, aPolicy);
    }

    struct narrow_phase final {
        const collision_shape_type &subject_shape;
        const shape_kinematics &subject;
        const collision_shape_type &other_shape;
        const shape_kinematics &other;
        collision_delta_time_type deltaTime;
        const impl_collision_policy &policy;

        [[nodiscard]] std::optional<overlap> flipped() const {
            auto result = vs_plane(other_shape, other, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }

        template <typename source_type>
        [[nodiscard]] std::optional<overlap> against_triangles(const collision_shape_type &aSubjectShape,
            const shape_kinematics &aSubject, const source_type &aMesh,
            const shape_kinematics &aMeshKinematics) const {
            if (!aMesh.data || aMesh.data->triangle_count() == 0) return std::nullopt;

            const auto extents = shape_extents(aSubjectShape, aSubject.orientation);
            const auto sweep = (aSubject.velocity - aMeshKinematics.velocity) * deltaTime;

            std::vector<std::uint32_t> candidates;

            GDK_COLLISION_PROFILE_COUNT(mesh_queries, 1);
            GDK_COLLISION_PROFILE_BEGIN(bvh_query_ms);

            if (std::isinf(extents.x) || std::isinf(extents.y) || std::isinf(extents.z)) {
                aMesh.data->all(candidates);
            }
            else {
                impl_mesh_data::bounds world;
                world.min = collision_vector3_type::min(aSubject.position - extents,
                    aSubject.position - extents + sweep);
                world.max = collision_vector3_type::max(aSubject.position + extents,
                    aSubject.position + extents + sweep);

                const auto toLocal = aMeshKinematics.orientation.inverse_unit();

                impl_mesh_data::bounds local;
                for (int corner = 0; corner < 8; ++corner) {
                    const auto rotated = rotate(toLocal, collision_vector3_type{
                        (corner & 1) ? world.max.x : world.min.x,
                        (corner & 2) ? world.max.y : world.min.y,
                        (corner & 4) ? world.max.z : world.min.z} - aMeshKinematics.position);

                    local.min = corner ? collision_vector3_type::min(local.min, rotated) : rotated;
                    local.max = corner ? collision_vector3_type::max(local.max, rotated) : rotated;
                }

                aMesh.data->query(local, candidates);
            }

            GDK_COLLISION_PROFILE_END(bvh_query_ms);
            GDK_COLLISION_PROFILE_COUNT(mesh_candidates, candidates.size());
            GDK_COLLISION_PROFILE_BEGIN(triangle_test_ms);

            std::optional<overlap> nearest;

            for (const auto candidate : candidates) {
                GDK_COLLISION_PROFILE_COUNT(triangle_tests, 1);

                triangle_shape triangle;
                aMesh.data->triangle(candidate, triangle.a, triangle.b, triangle.c);

                const collision_shape_type triangleShape{triangle};

                auto result = std::visit(narrow_phase{aSubjectShape, aSubject, triangleShape,
                    aMeshKinematics, deltaTime, policy}, aSubjectShape, triangleShape);

                if (!result) continue;

                if (const auto corrected = corrected_internal_edge_normal(
                    aMesh.data->internal_edges(candidate), world_triangle(triangle, aMeshKinematics),
                    result->contact_point, result->contact_normal, policy))
                    result->contact_normal = *corrected;

                if (!is_closing_contact(*result, aSubject, aMeshKinematics, policy)) continue;

                if (!nearest
                    || result->entry_time < nearest->entry_time - policy.AXIS_ENTRY_EPSILON
                    || (result->entry_time <= nearest->entry_time + policy.AXIS_ENTRY_EPSILON
                        && result->penetration > nearest->penetration))
                    nearest = result;
            }

            GDK_COLLISION_PROFILE_END(triangle_test_ms);

            return nearest;
        }

        std::optional<overlap> operator()(const sphere_shape &aSubject, const sphere_shape &aOther) const {
            return sphere_vs_sphere(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const sphere_shape &aSubject, const box_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const box_shape &aSubject, const sphere_shape &aOther) const {
            auto result = swept_by_separation(aOther, other, aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }

        std::optional<overlap> operator()(const box_shape &aSubject, const box_shape &aOther) const {
            return box_vs_box(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const capsule_shape &aSubject, const capsule_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const capsule_shape &aSubject, const sphere_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const sphere_shape &aSubject, const capsule_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const capsule_shape &aSubject, const box_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const sphere_shape &aSubject, const obb_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const capsule_shape &aSubject, const obb_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const obb_shape &aSubject, const sphere_shape &aOther) const {
            auto result = swept_by_separation(aOther, other, aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }

        std::optional<overlap> operator()(const obb_shape &aSubject, const capsule_shape &aOther) const {
            auto result = swept_by_separation(aOther, other, aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }

        std::optional<overlap> operator()(const obb_shape &aSubject, const obb_shape &aOther) const {
            return swept_box_pair(aSubject.half_extents, subject.orientation, subject,
                aOther.half_extents, other.orientation, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const obb_shape &aSubject, const box_shape &aOther) const {
            return swept_box_pair(aSubject.half_extents, subject.orientation, subject,
                aOther.half_extents, collision_quaternion_type::identity, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const box_shape &aSubject, const obb_shape &aOther) const {
            return swept_box_pair(aSubject.half_extents, collision_quaternion_type::identity, subject,
                aOther.half_extents, other.orientation, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const sphere_shape &, const plane_shape &) const {
            return vs_plane(subject_shape, subject, other, deltaTime, policy);
        }
        std::optional<overlap> operator()(const box_shape &, const plane_shape &) const {
            return vs_plane(subject_shape, subject, other, deltaTime, policy);
        }
        std::optional<overlap> operator()(const capsule_shape &, const plane_shape &) const {
            return vs_plane(subject_shape, subject, other, deltaTime, policy);
        }
        std::optional<overlap> operator()(const obb_shape &, const plane_shape &) const {
            return vs_plane(subject_shape, subject, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const plane_shape &, const sphere_shape &) const { return flipped(); }
        std::optional<overlap> operator()(const plane_shape &, const box_shape &) const { return flipped(); }
        std::optional<overlap> operator()(const plane_shape &, const capsule_shape &) const { return flipped(); }
        std::optional<overlap> operator()(const plane_shape &, const obb_shape &) const { return flipped(); }

        std::optional<overlap> operator()(const sphere_shape &aSubject, const triangle_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }
        std::optional<overlap> operator()(const capsule_shape &aSubject, const triangle_shape &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }
        std::optional<overlap> operator()(const box_shape &aSubject, const triangle_shape &aOther) const {
            return swept_box_triangle(aSubject.half_extents, collision_quaternion_type::identity, subject,
                aOther, other, deltaTime, policy);
        }
        std::optional<overlap> operator()(const obb_shape &aSubject, const triangle_shape &aOther) const {
            return swept_box_triangle(aSubject.half_extents, subject.orientation, subject,
                aOther, other, deltaTime, policy);
        }
        std::optional<overlap> operator()(const plane_shape &, const triangle_shape &) const {
            return flipped();
        }

        std::optional<overlap> operator()(const triangle_shape &aSubject, const sphere_shape &aOther) const {
            auto result = swept_by_separation(aOther, other, aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }
        std::optional<overlap> operator()(const triangle_shape &aSubject, const capsule_shape &aOther) const {
            auto result = swept_by_separation(aOther, other, aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }
        std::optional<overlap> operator()(const triangle_shape &aSubject, const box_shape &aOther) const {
            auto result = swept_box_triangle(aOther.half_extents, collision_quaternion_type::identity, other,
                aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }
        std::optional<overlap> operator()(const triangle_shape &aSubject, const obb_shape &aOther) const {
            auto result = swept_box_triangle(aOther.half_extents, other.orientation, other,
                aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }
        std::optional<overlap> operator()(const triangle_shape &, const plane_shape &) const {
            return vs_plane(subject_shape, subject, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const sphere_shape &, const mesh_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }
        std::optional<overlap> operator()(const box_shape &, const mesh_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }
        std::optional<overlap> operator()(const capsule_shape &, const mesh_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }
        std::optional<overlap> operator()(const obb_shape &, const mesh_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }
        std::optional<overlap> operator()(const plane_shape &, const mesh_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }

        std::optional<overlap> operator()(const sphere_shape &, const heightfield_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }
        std::optional<overlap> operator()(const box_shape &, const heightfield_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }
        std::optional<overlap> operator()(const capsule_shape &, const heightfield_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }
        std::optional<overlap> operator()(const obb_shape &, const heightfield_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }
        std::optional<overlap> operator()(const plane_shape &, const heightfield_shape &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }

        template <typename source_type>
        [[nodiscard]] std::optional<overlap> surface_as_subject(const source_type &aSubject) const {
            auto result = against_triangles(other_shape, other, aSubject, subject);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }

        std::optional<overlap> operator()(const heightfield_shape &aSubject, const sphere_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const heightfield_shape &aSubject, const box_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const heightfield_shape &aSubject, const capsule_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const heightfield_shape &aSubject, const obb_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const heightfield_shape &aSubject, const plane_shape &) const {
            return surface_as_subject(aSubject);
        }

        std::optional<overlap> operator()(const heightfield_shape &, const heightfield_shape &) const { return std::nullopt; }
        std::optional<overlap> operator()(const heightfield_shape &, const mesh_shape &) const { return std::nullopt; }
        std::optional<overlap> operator()(const mesh_shape &, const heightfield_shape &) const { return std::nullopt; }
        std::optional<overlap> operator()(const heightfield_shape &, const triangle_shape &) const { return std::nullopt; }
        std::optional<overlap> operator()(const triangle_shape &, const heightfield_shape &) const { return std::nullopt; }

        std::optional<overlap> operator()(const triangle_shape &, const mesh_shape &) const {
            return std::nullopt;
        }

        std::optional<overlap> operator()(const mesh_shape &aSubject, const sphere_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const mesh_shape &aSubject, const box_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const mesh_shape &aSubject, const capsule_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const mesh_shape &aSubject, const obb_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const mesh_shape &aSubject, const plane_shape &) const {
            return surface_as_subject(aSubject);
        }
        std::optional<overlap> operator()(const mesh_shape &, const triangle_shape &) const {
            return std::nullopt;   
        }

        std::optional<overlap> operator()(const mesh_shape &, const mesh_shape &) const {
            return std::nullopt;
        }

        std::optional<overlap> operator()(const triangle_shape &, const triangle_shape &) const {
            throw collision_exception("gdk::collision: triangle against triangle was reached, which "
                "should be impossible -- the mesh pair is intercepted before decomposition");
        }

        std::optional<overlap> operator()(const plane_shape &, const plane_shape &) const {
            throw collision_exception("gdk::collision: half-space against half-space is not a "
                "supported pair -- two infinite planes have no bounded overlap to report");
        }

        std::optional<overlap> operator()(const box_shape &aSubject, const capsule_shape &aOther) const {
            auto result = swept_by_separation(aOther, other, aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }
    };
}

bool gdk::is_closing(const overlap &aOverlap, const shape_kinematics &aSubject,
    const shape_kinematics &aOther, const impl_collision_policy &aPolicy) {
    const auto relativeVelocity = aSubject.velocity - aOther.velocity;
    return relativeVelocity.dot_product(aOverlap.contact_normal) <= -aPolicy.MIN_SWEPT_SPEED;
}

std::optional<overlap> gdk::narrow_phase_overlap(
    const collision_shape_type &aSubjectShape, const shape_kinematics &aSubject,
    const collision_shape_type &aOtherShape, const shape_kinematics &aOther,
    const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
    auto result = std::visit(narrow_phase{aSubjectShape, aSubject, aOtherShape, aOther, aDeltaTime, aPolicy},
        aSubjectShape, aOtherShape);

    if (result && !is_closing_contact(*result, aSubject, aOther, aPolicy)) return std::nullopt;

    return result;
}

std::optional<overlap> gdk::narrow_phase_penetration(
    const collision_shape_type &aSubjectShape, const shape_kinematics &aSubject,
    const collision_shape_type &aOtherShape, const shape_kinematics &aOther,
    const impl_collision_policy &aPolicy) {
    const auto subjectSupported = std::holds_alternative<sphere_shape>(aSubjectShape)
        || std::holds_alternative<capsule_shape>(aSubjectShape);
    const auto otherSupported = std::holds_alternative<box_shape>(aOtherShape)
        || std::holds_alternative<obb_shape>(aOtherShape)
        || std::holds_alternative<sphere_shape>(aOtherShape)
        || std::holds_alternative<capsule_shape>(aOtherShape);
    if (!subjectSupported || !otherSupported) return std::nullopt;

    const auto contactRadius = radius_of(aSubjectShape) + radius_of(aOtherShape);
    const auto sample = separation_at(aSubjectShape, aSubject, aOtherShape, aOther, aPolicy);

    const auto gap = sample.distance - contactRadius;
    if (gap >= 0) return std::nullopt;                      

    const auto delta = sample.points.inside
        ? sample.points.on_second - sample.points.on_first
        : sample.points.on_first - sample.points.on_second;

    const auto length = delta.length();
    if (length <= aPolicy.NORMALIZATION_THRESHOLD) return std::nullopt; 

    overlap result;
    result.entry_time = 0;
    result.penetration = -gap;
    result.contact_normal = delta * (collision_floating_point_type{1} / length);
    return result;
}

std::optional<overlap> gdk::narrow_phase_overlap_parts(
    const std::vector<collider_part> &aSubjectParts, const shape_kinematics &aSubject,
    const std::vector<collider_part> &aOtherParts, const shape_kinematics &aOther,
    const collision_delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
    const auto world_of = [](const collider_part &aPart, const shape_kinematics &aCollider) {
        shape_kinematics kinematics;
        kinematics.position = aCollider.position + rotate(aCollider.orientation, aPart.position);
        kinematics.velocity = aCollider.velocity;
        kinematics.orientation = aCollider.orientation * aPart.rotation;
        return kinematics;
    };

    std::optional<overlap> earliest;

    for (const auto &subjectPart : aSubjectParts) {
        const auto subject = world_of(subjectPart, aSubject);

        for (const auto &otherPart : aOtherParts) {
            const auto other = world_of(otherPart, aOther);

            const auto result = narrow_phase_overlap(subjectPart.shape, subject,
                otherPart.shape, other, aDeltaTime, aPolicy);


            if (result && (!earliest || result->entry_time < earliest->entry_time))
                earliest = result;
        }
    }

    return earliest;
}
