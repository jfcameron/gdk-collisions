// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_collision_profile.h>
#include <gdk/collisions/impl_narrow_phase.h>

#include <variant>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <span>

using namespace gdk::collisions;

namespace {
    constexpr gdk::collisions::floating_point_type ROUNDING_UNITS = 8;

    /// \brief axis aligned bounds of a shape sitting at a position
    struct bounds final {
        vector3_type min;
        vector3_type max;
    };

    [[nodiscard]] bounds bounds_of(const box_shape &aBox, const vector3_type &aPosition) {
        return bounds{aPosition - aBox.half_extents, aPosition + aBox.half_extents};
    }

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

    /// \brief how far apart two shapes are, and where
    struct separation final {
        floating_point_type distance;
        closest_points points;
    };

    /// \brief the interior segment of a shape at a position. Non-capsules degenerate to a point.
    [[nodiscard]] segment segment_of(const shape_type &aShape, const shape_kinematics &aKinematics) {
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

    [[nodiscard]] floating_point_type clamp01(const floating_point_type aValue) {
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

    struct triangle_points final {
        vector3_type a;
        vector3_type b;
        vector3_type c;
    };

    [[nodiscard]] triangle_points world_triangle(const triangle_shape &aTriangle,
        const shape_kinematics &aKinematics) {
        return triangle_points{
            aKinematics.position + aKinematics.orientation * aTriangle.a,
            aKinematics.position + aKinematics.orientation * aTriangle.b,
            aKinematics.position + aKinematics.orientation * aTriangle.c};
    }

    /// \brief closest point on a triangle to a point 
    /// \brief what to make of a contact that landed on a triangle's internal edge
    struct internal_edge_verdict final {
        /// \brief the subject is off this triangle's rim rather than over it
        bool doubtful = false;

        //! a replacement normal
        std::optional<vector3_type> normal;
    };

    [[nodiscard]] internal_edge_verdict corrected_internal_edge_normal(
        const std::uint8_t aInternalEdges, const triangle_points &aTriangle,
        const vector3_type &aContactPoint, const vector3_type &aContactNormal,
        const impl_collision_policy &aPolicy) {
        if (!aInternalEdges) return {};

        const auto ab = aTriangle.b - aTriangle.a;
        const auto ac = aTriangle.c - aTriangle.a;

        auto faceNormal = ab.cross_product(ac);
        if (faceNormal.length() <= aPolicy.NORMALIZATION_THRESHOLD) return {};
        faceNormal = faceNormal.normal();

        const auto alignment = aContactNormal.dot_product(faceNormal);
        if (std::abs(alignment) >= 1.0f - aPolicy.FACE_CONTACT_COSINE_EPSILON) return {};

        const auto ap = aContactPoint - aTriangle.a;

        const auto d00 = ab.dot_product(ab);
        const auto d01 = ab.dot_product(ac);
        const auto d11 = ac.dot_product(ac);
        const auto d20 = ap.dot_product(ab);
        const auto d21 = ap.dot_product(ac);

        const auto denominator = d00 * d11 - d01 * d01;
        if (std::abs(denominator) <= aPolicy.NORMALIZATION_THRESHOLD) return {};

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
            if (!(aInternalEdges & (1u << e))) return {};
            touched = true;
        }

        if (!touched) return {};

        const auto snapped = alignment >= 0 ? faceNormal : faceNormal * -1.0f;

        return {std::abs(alignment) <= aPolicy.INTERNAL_EDGE_GHOST_COSINE, snapped};
    }

    [[nodiscard]] vector3_type closest_point_on_triangle(const triangle_points &aTriangle,
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

    /// \brief closest points between a segment and a triangle.
    [[nodiscard]] closest_points closest_points_segment_triangle(const segment &aSegment,
        const triangle_points &aTriangle, const impl_collision_policy &aPolicy) {
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

    [[nodiscard]] vector3_type closest_point_on_box(const bounds &aBounds,
        const vector3_type &aPoint) {
        return vector3_type{
            std::max(aBounds.min.x, std::min(aPoint.x, aBounds.max.x)),
            std::max(aBounds.min.y, std::min(aPoint.y, aBounds.max.y)),
            std::max(aBounds.min.z, std::min(aPoint.z, aBounds.max.z)),
        };
    }

    [[nodiscard]] vector3_type nearest_face_point(const bounds &aBounds,
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

    [[nodiscard]] closest_points closest_points_segment_box(const segment &aSegment,
        const bounds &aBounds, const impl_collision_policy &aPolicy) {
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

        candidateCount = std::min(candidateCount, candidates.size());

        std::sort(candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(candidateCount));

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

    [[nodiscard]] separation separation_at(
        const shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const shape_type &aOtherShape, const shape_kinematics &aOther,
        const impl_collision_policy &aPolicy) {
        const auto subjectSegment = segment_of(aSubjectShape, aSubject);

        if (std::holds_alternative<obb_shape>(aOtherShape)) {
            const auto inverse = aOther.orientation.inverse_unit();
            const segment localSegment{
                inverse * (subjectSegment.a - aOther.position),
                inverse * (subjectSegment.b - aOther.position)};

            const auto &extents = std::get<obb_shape>(aOtherShape).half_extents;
            const bounds localBox{extents * -1.0f, extents};

            const auto local = closest_points_segment_box(localSegment, localBox, aPolicy);
            const closest_points world{
                aOther.orientation * local.on_first + aOther.position,
                aOther.orientation * local.on_second + aOther.position,
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

    [[nodiscard]] floating_point_type radius_of(const shape_type &aShape) {
        return std::visit(overloaded{
            [](const sphere_shape &aSphere) { return aSphere.radius; },
            [](const box_shape &) { return floating_point_type{0}; },
            [](const obb_shape &) { return floating_point_type{0}; },
            [](const triangle_shape &) { return floating_point_type{0}; },
            [](const mesh_shape &) { return floating_point_type{0}; },
            [](const heightfield_shape &) { return floating_point_type{0}; },
            [](const plane_shape &) { return floating_point_type{0}; },
            [](const capsule_shape &aCapsule) { return aCapsule.radius; },
        }, aShape);
    }

    /// \brief generic swept test for shapes whose separation is a distance query.
    [[nodiscard]] std::optional<overlap> swept_by_separation(
        const shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const shape_type &aOtherShape, const shape_kinematics &aOther,
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto contactRadius = radius_of(aSubjectShape) + radius_of(aOtherShape);
        const auto relativeVelocity = aSubject.velocity - aOther.velocity;
        const auto relativeSpeed = relativeVelocity.length();

        const auto sample_at = [&](const delta_time_type aTime) {
            return separation_at(
                aSubjectShape, {aSubject.position + aSubject.velocity * aTime, aSubject.velocity, aSubject.orientation},
                aOtherShape, {aOther.position + aOther.velocity * aTime, aOther.velocity, aOther.orientation},
                aPolicy);
        };

        const auto rounding_of = [](const separation &aSample) {
            const auto &a = aSample.points.on_first, &b = aSample.points.on_second;

            const auto scale = std::max({std::abs(a.x), std::abs(a.y), std::abs(a.z), std::abs(b.x), std::abs(b.y), std::abs(b.z)});

            return ROUNDING_UNITS * std::numeric_limits<floating_point_type>::epsilon() * scale;
        };

        std::optional<vector3_type> lastReadable;

        const auto readable_of = [&](const separation &aSample) -> std::optional<vector3_type> {
            const auto delta = aSample.points.inside
                ? aSample.points.on_second - aSample.points.on_first
                : aSample.points.on_first - aSample.points.on_second;

            const auto length = delta.length();

            if (length > aPolicy.NORMALIZATION_THRESHOLD + rounding_of(aSample)) return delta * (1.0f / length);

            return std::nullopt;
        };

        const auto contact_at = [&](const delta_time_type aTime, const separation &aSample) {
            const auto normal = readable_of(aSample).value_or(lastReadable.value_or(
                relativeSpeed > aPolicy.NORMALIZATION_THRESHOLD
                    ? relativeVelocity.normal() * -1.0f
                    : vector3_type{0, 1, 0}));

            overlap result;
            result.entry_time = aTime;
            result.exit_time = aDeltaTime;
            result.penetration = std::max(0.0f, contactRadius - aSample.distance);
            result.contact_normal = normal;
            result.contact_point = aSample.points.on_second;
            return result;
        };

        delta_time_type time = 0;
        auto previousGap = std::numeric_limits<floating_point_type>::infinity();

        for (unsigned short int step = 0; step <= aPolicy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS; ++step) {
            GDK_COLLISION_PROFILE_COUNT(advancement_iterations, 1);

            const auto sample = sample_at(time);
            const auto gap = sample.distance - contactRadius;

            if (gap <= aPolicy.PENETRATION_DEPTH_EPSILON + rounding_of(sample)) return contact_at(time, sample);

            if (const auto readable = readable_of(sample)) lastReadable = readable;

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
        vector3_type centre;
        vector3_type half_extents;
        vector3_type axis[3];
    };

    [[nodiscard]] box_frame frame_of(const vector3_type &aHalfExtents,
        const vector3_type &aCentre, const quaternion_type &aOrientation) {
        box_frame frame;
        frame.centre = aCentre;
        frame.half_extents = aHalfExtents;
        frame.axis[0] = aOrientation * vector3_type{1, 0, 0};
        frame.axis[1] = aOrientation * vector3_type{0, 1, 0};
        frame.axis[2] = aOrientation * vector3_type{0, 0, 1};
        return frame;
    }

    [[nodiscard]] floating_point_type projected_radius(const box_frame &aBox,
        const vector3_type &aAxis) {
        return std::abs(aAxis.dot_product(aBox.axis[0])) * aBox.half_extents.x
             + std::abs(aAxis.dot_product(aBox.axis[1])) * aBox.half_extents.y
             + std::abs(aAxis.dot_product(aBox.axis[2])) * aBox.half_extents.z;
    }

    struct contact_sample final {
        floating_point_type gap; ///< negative when interpenetrating
        vector3_type normal;     ///< from the other body toward the subject
        vector3_type point;
    };

    [[nodiscard]] contact_sample separating_axis(const box_frame &aSubject, const box_frame &aOther,
        const impl_collision_policy &aPolicy) {
        const auto delta = aSubject.centre - aOther.centre;

        contact_sample best;
        best.gap = -std::numeric_limits<floating_point_type>::infinity();
        best.normal = vector3_type::zero;

        const auto consider = [&](vector3_type aAxis) {
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
        const floating_point_type aRelativeSpeed,
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto contact_at = [&](const delta_time_type aTime, const contact_sample &aSample) {
            overlap result;
            result.entry_time = aTime;
            result.exit_time = aDeltaTime;
            result.penetration = std::max(0.0f, -aSample.gap);
            result.contact_normal = aSample.normal;
            result.contact_point = aSample.point;
            return result;
        };

        delta_time_type time = 0;
        auto previousGap = std::numeric_limits<floating_point_type>::infinity();

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
        best.gap = -std::numeric_limits<floating_point_type>::infinity();
        best.normal = vector3_type::zero;

        const auto consider = [&](vector3_type aAxis) {
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

        const vector3_type edges[3] = {
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
        const vector3_type &aBoxExtents, const quaternion_type &aBoxOrientation,
        const shape_kinematics &aBox, const triangle_shape &aTriangleShape,
        const shape_kinematics &aTriangle, const delta_time_type aDeltaTime,
        const impl_collision_policy &aPolicy) {
        const auto relativeSpeed = (aBox.velocity - aTriangle.velocity).length();

        return conservative_sweep([&](const delta_time_type aTime) {
            const auto box = frame_of(aBoxExtents, aBox.position + aBox.velocity * aTime, aBoxOrientation);
            const auto triangle = world_triangle(aTriangleShape,
                {aTriangle.position + aTriangle.velocity * aTime, aTriangle.velocity, aTriangle.orientation});

            auto sample = separating_axis_triangle(box, triangle, aPolicy);
            sample.point = closest_point_on_triangle(triangle, box.centre);
            return sample;
        }, relativeSpeed, aDeltaTime, aPolicy);
    }

    [[nodiscard]] std::optional<overlap> swept_box_pair(
        const vector3_type &aSubjectExtents, const quaternion_type &aSubjectOrientation,
        const shape_kinematics &aSubject,
        const vector3_type &aOtherExtents, const quaternion_type &aOtherOrientation,
        const shape_kinematics &aOther,
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto relativeSpeed = (aSubject.velocity - aOther.velocity).length();

        return conservative_sweep([&](const delta_time_type aTime) {
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
        vector3_type point;
        vector3_type normal;
    };

    [[nodiscard]] plane_frame plane_of(const shape_kinematics &aKinematics) {
        return plane_frame{aKinematics.position,
            aKinematics.orientation * vector3_type{0, 1, 0}};
    }

    [[nodiscard]] floating_point_type gap_to_plane(const shape_type &aShape,
        const shape_kinematics &aKinematics, const plane_frame &aPlane) {
        const auto centreHeight = (aKinematics.position - aPlane.point).dot_product(aPlane.normal);

        return std::visit(overloaded{
            [&](const sphere_shape &aSphere) { return centreHeight - aSphere.radius; },
            [&](const box_shape &aBox) {
                return centreHeight - projected_radius(
                    frame_of(aBox.half_extents, aKinematics.position, quaternion_type::identity),
                    aPlane.normal);
            },
            [&](const obb_shape &aBox) {
                return centreHeight - projected_radius(
                    frame_of(aBox.half_extents, aKinematics.position, aKinematics.orientation),
                    aPlane.normal);
            },
            [&](const capsule_shape &aCapsule) {
                const auto offset = (aKinematics.orientation * vector3_type{0, aCapsule.half_height, 0}).dot_product(aPlane.normal);
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
                return std::numeric_limits<floating_point_type>::max();
            },
            [&](const mesh_shape &) {
                return std::numeric_limits<floating_point_type>::max();
            },
            [&](const plane_shape &) -> floating_point_type {
                throw exception("gdk::collision: half-space against half-space is not a "
                    "supported pair -- two infinite planes have no bounded overlap to report");
            },
        }, aShape);
    }

    [[nodiscard]] std::optional<overlap> vs_plane(const shape_type &aSubjectShape,
        const shape_kinematics &aSubject, const shape_kinematics &aPlaneKinematics,
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto plane = plane_of(aPlaneKinematics);
        const auto gap = gap_to_plane(aSubjectShape, aSubject, plane);

        const auto contact_at = [&](const delta_time_type aTime,
            const floating_point_type aGap) {
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
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
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
                : vector3_type{1.0f, 0.0f, 0.0f};

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
                : vector3_type{1.0f, 0.0f, 0.0f});

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
        const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
        const auto thisBroadPhaseBounds = bounds_of(aSubjectShape, aSubject.position);
        const auto otherBroadPhaseBounds = bounds_of(aOtherShape, aOther.position);
        const auto relativeVelocity = aSubject.velocity - aOther.velocity;

        const auto contact_at = [&](const std::vector<overlap::contact_axis> &aAxes,
            const delta_time_type aToi) {
            const auto boundsA = bounds_of(aSubjectShape, aSubject.position + aSubject.velocity * aToi);
            const auto boundsB = bounds_of(aOtherShape, aOther.position + aOther.velocity * aToi);

            struct contact final {
                vector3_type normal = vector3_type::zero;
                floating_point_type penetration = std::numeric_limits<floating_point_type>::infinity();
                vector3_type point = vector3_type::zero;
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

                auto normal = vector3_type::zero;
                normal[i] = sign;

                best.penetration = penetration;
                best.normal = normal;
            }

            const auto centre = aSubject.position + aSubject.velocity * aToi;
            best.point = vector3_type{
                std::max(boundsB.min.x, std::min(centre.x, boundsB.max.x)),
                std::max(boundsB.min.y, std::min(centre.y, boundsB.max.y)),
                std::max(boundsB.min.z, std::min(centre.z, boundsB.max.z)),
            };
            return best;
        };

        auto anyMoving = false;
        auto entryMax = -std::numeric_limits<floating_point_type>::infinity();
        auto exitMin = std::numeric_limits<floating_point_type>::infinity();
        auto minPenetration = std::numeric_limits<floating_point_type>::infinity();
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

            const floating_point_type invVi = 1.0f / vi;
            const floating_point_type t1 = (otherBroadPhaseBounds.min[i] - thisBroadPhaseBounds.max[i]) * invVi;
            const floating_point_type t2 = (otherBroadPhaseBounds.max[i] - thisBroadPhaseBounds.min[i]) * invVi;

            const floating_point_type axisEntry = std::min(t1, t2);
            const floating_point_type axisExit  = std::max(t1, t2);

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
                const floating_point_type invVi = 1.0f / vi;
                const floating_point_type t1 = (otherBroadPhaseBounds.min[i] - thisBroadPhaseBounds.max[i]) * invVi;
                const floating_point_type t2 = (otherBroadPhaseBounds.max[i] - thisBroadPhaseBounds.min[i]) * invVi;
                const floating_point_type axisEntry = std::min(t1, t2);
                const floating_point_type axisExit  = std::max(t1, t2);

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
            const auto frame = frame_of(aBox.half_extents, vector3_type::zero, aOrientation);
            return vector3_type{
                projected_radius(frame, vector3_type{1, 0, 0}),
                projected_radius(frame, vector3_type{0, 1, 0}),
                projected_radius(frame, vector3_type{0, 0, 1}),
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

namespace {
    [[nodiscard]] bool is_closing_contact(const overlap &aOverlap, const shape_kinematics &aSubject,
        const shape_kinematics &aOther, const impl_collision_policy &aPolicy) {
        return aOverlap.penetration > aPolicy.PENETRATION_DEPTH_EPSILON
            || is_closing(aOverlap, aSubject, aOther, aPolicy);
    }

    struct narrow_phase final {
        const shape_type &subject_shape;
        const shape_kinematics &subject;
        const shape_type &other_shape;
        const shape_kinematics &other;
        delta_time_type deltaTime;
        const impl_collision_policy &policy;

        [[nodiscard]] std::optional<overlap> flipped() const {
            auto result = vs_plane(other_shape, other, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }

        template <typename source_type>
        [[nodiscard]] std::optional<overlap> against_triangles(const shape_type &aSubjectShape,
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
                world.min = vector3_type::min(aSubject.position - extents,
                    aSubject.position - extents + sweep);
                world.max = vector3_type::max(aSubject.position + extents,
                    aSubject.position + extents + sweep);

                const auto toLocal = aMeshKinematics.orientation.inverse_unit();

                impl_mesh_data::bounds local;
                for (int corner = 0; corner < 8; ++corner) {
                    const auto rotated = toLocal * (vector3_type{
                        (corner & 1) ? world.max.x : world.min.x,
                        (corner & 2) ? world.max.y : world.min.y,
                        (corner & 4) ? world.max.z : world.min.z} - aMeshKinematics.position);

                    local.min = corner ? vector3_type::min(local.min, rotated) : rotated;
                    local.max = corner ? vector3_type::max(local.max, rotated) : rotated;
                }

                aMesh.data->query(local, candidates);
            }

            GDK_COLLISION_PROFILE_END(bvh_query_ms);
            GDK_COLLISION_PROFILE_COUNT(mesh_candidates, candidates.size());
            GDK_COLLISION_PROFILE_BEGIN(triangle_test_ms);

            std::optional<overlap> nearest;

            std::optional<overlap> doubtful;

            for (const auto candidate : candidates) {
                GDK_COLLISION_PROFILE_COUNT(triangle_tests, 1);

                triangle_shape triangle;
                aMesh.data->triangle(candidate, triangle.a, triangle.b, triangle.c);

                const shape_type triangleShape{triangle};

                auto result = std::visit(narrow_phase{aSubjectShape, aSubject, triangleShape,
                    aMeshKinematics, deltaTime, policy}, aSubjectShape, triangleShape);

                if (!result) continue;

                const auto verdict = corrected_internal_edge_normal(
                    aMesh.data->internal_edges(candidate),
                    world_triangle(triangle, aMeshKinematics),
                    result->contact_point, result->contact_normal, policy);

                if (verdict.normal) result->contact_normal = *verdict.normal;

                if (!is_closing_contact(*result, aSubject, aMeshKinematics, policy)) continue;

                auto &into = verdict.doubtful ? doubtful : nearest;

                if (!into
                    || result->entry_time < into->entry_time - policy.AXIS_ENTRY_EPSILON
                    || (result->entry_time <= into->entry_time + policy.AXIS_ENTRY_EPSILON
                        && result->penetration > into->penetration))
                    into = result;
            }

            GDK_COLLISION_PROFILE_END(triangle_test_ms);

            return nearest ? nearest : doubtful;
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
                aOther.half_extents, quaternion_type::identity, other, deltaTime, policy);
        }

        std::optional<overlap> operator()(const box_shape &aSubject, const obb_shape &aOther) const {
            return swept_box_pair(aSubject.half_extents, quaternion_type::identity, subject,
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
            return swept_box_triangle(aSubject.half_extents, quaternion_type::identity, subject,
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
            auto result = swept_box_triangle(aOther.half_extents, quaternion_type::identity, other,
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
            throw exception("gdk::collision: triangle against triangle was reached, which "
                "should be impossible -- the mesh pair is intercepted before decomposition");
        }

        std::optional<overlap> operator()(const plane_shape &, const plane_shape &) const {
            throw exception("gdk::collision: half-space against half-space is not a "
                "supported pair -- two infinite planes have no bounded overlap to report");
        }

        std::optional<overlap> operator()(const box_shape &aSubject, const capsule_shape &aOther) const {
            auto result = swept_by_separation(aOther, other, aSubject, subject, deltaTime, policy);
            if (result) result->contact_normal = result->contact_normal * -1.0f;
            return result;
        }
    };
}

bool gdk::collisions::is_closing(const overlap &aOverlap, const shape_kinematics &aSubject,
    const shape_kinematics &aOther, const impl_collision_policy &aPolicy) {
    const auto relativeVelocity = aSubject.velocity - aOther.velocity;
    return relativeVelocity.dot_product(aOverlap.contact_normal) <= -aPolicy.MIN_SWEPT_SPEED;
}

std::optional<overlap> gdk::collisions::narrow_phase_overlap(
    const shape_type &aSubjectShape, const shape_kinematics &aSubject,
    const shape_type &aOtherShape, const shape_kinematics &aOther,
    const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
    auto result = std::visit(narrow_phase{aSubjectShape, aSubject, aOtherShape, aOther, aDeltaTime, aPolicy},
        aSubjectShape, aOtherShape);

    if (result && !is_closing_contact(*result, aSubject, aOther, aPolicy)) return std::nullopt;

    return result;
}

std::optional<overlap> gdk::collisions::narrow_phase_penetration(
    const shape_type &aSubjectShape, const shape_kinematics &aSubject,
    const shape_type &aOtherShape, const shape_kinematics &aOther,
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
    result.contact_normal = delta * (floating_point_type{1} / length);
    return result;
}

std::optional<overlap> gdk::collisions::narrow_phase_overlap_parts(
    std::span<const collider_part> aSubjectParts, const shape_kinematics &aSubject,
    std::span<const collider_part> aOtherParts, const shape_kinematics &aOther,
    const delta_time_type aDeltaTime, const impl_collision_policy &aPolicy) {
    const auto world_of = [](const collider_part &aPart, const shape_kinematics &aCollider) {
        shape_kinematics kinematics;
        kinematics.position = aCollider.position + aCollider.orientation * aPart.position;
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

namespace {
    /// \name A ray, tested exactly
    ///
    /// @{

    //! how far outside a triangle's edges a ray may still meet it
    constexpr floating_point_type TRIANGLE_EDGE_SLACK = 1e-5f;

    struct ray_hit final {
        floating_point_type distance;
        vector3_type normal;
    };

    [[nodiscard]] std::optional<overlap> overlap_of(const ray_hit &aHit, const vector3_type &aOrigin,
        const vector3_type &aDirection, const floating_point_type aMaxDistance) {
        overlap out;

        out.entry_time = aHit.distance;
        out.exit_time = aMaxDistance;
        out.contact_normal = aHit.normal;
        out.contact_point = aOrigin + aDirection * aHit.distance;

        return out;
    }

    //! an axis aligned box
    [[nodiscard]] std::optional<ray_hit> ray_box(const vector3_type &aOrigin, const vector3_type &aDirection,
        const floating_point_type aMax, const vector3_type &aMin, const vector3_type &aMaxCorner) {
        const bool inside = aOrigin.x >= aMin.x && aOrigin.x <= aMaxCorner.x && aOrigin.y >= aMin.y && aOrigin.y <= aMaxCorner.y
            && aOrigin.z >= aMin.z && aOrigin.z <= aMaxCorner.z;

        if (inside) {
            std::size_t axis = 0;
            floating_point_type depth = std::numeric_limits<floating_point_type>::infinity(), sign = -1.0f;

            for (std::size_t i = 0; i < 3; ++i) {
                if (aOrigin[i] - aMin[i] < depth) { depth = aOrigin[i] - aMin[i]; axis = i; sign = -1.0f; }
                if (aMaxCorner[i] - aOrigin[i] < depth) { depth = aMaxCorner[i] - aOrigin[i]; axis = i; sign = 1.0f; }
            }

            vector3_type normal = vector3_type::zero;

            normal[axis] = sign;

            return ray_hit{0, normal};
        }

        floating_point_type near = 0, far = aMax;
        int axis = -1;
        floating_point_type sign = 0;

        for (int i = 0; i < 3; ++i) {
            const auto o = aOrigin[static_cast<std::size_t>(i)], d = aDirection[static_cast<std::size_t>(i)];
            const auto low = aMin[static_cast<std::size_t>(i)], high = aMaxCorner[static_cast<std::size_t>(i)];

            if (std::abs(d) < 1e-12f) {
                if (o < low || o > high) return std::nullopt;

                continue;
            }

            auto t0 = (low - o) / d, t1 = (high - o) / d;
            floating_point_type entering = -1.0f;

            if (t0 > t1) {
                std::swap(t0, t1);
                entering = 1.0f;
            }

            if (t0 > near) {
                near = t0;
                axis = i;
                sign = entering;
            }

            far = std::min(far, t1);

            if (near > far) return std::nullopt;
        }

        if (axis < 0) return std::nullopt;

        vector3_type normal = vector3_type::zero;

        normal[static_cast<std::size_t>(axis)] = sign;

        return ray_hit{near, normal};
    }

    [[nodiscard]] std::optional<ray_hit> ray_sphere(const vector3_type &aOrigin, const vector3_type &aDirection,
        const floating_point_type aMax, const vector3_type &aCentre, const floating_point_type aRadius) {
        if (aRadius <= 0) return std::nullopt;

        const auto offset = aOrigin - aCentre;
        const auto c = offset.length_squared() - aRadius * aRadius;

        if (c <= 0) {
            const auto length = offset.length();

            return ray_hit{0, length > 1e-12f ? offset * (1.0f / length) : aDirection * -1.0f};
        }

        const auto b = offset.dot_product(aDirection);

        if (b > 0) return std::nullopt;

        const auto discriminant = b * b - c;

        if (discriminant < 0) return std::nullopt;

        const auto t = -b - std::sqrt(discriminant);

        if (t > aMax) return std::nullopt;

        return ray_hit{std::max(0.0f, t), (aOrigin + aDirection * t - aCentre).normal()};
    }

    [[nodiscard]] std::optional<ray_hit> ray_capsule(const vector3_type &aOrigin, const vector3_type &aDirection,
        const floating_point_type aMax, const vector3_type &aA, const vector3_type &aB, const floating_point_type aRadius) {
        if (aRadius <= 0) return std::nullopt;

        const auto axis = aB - aA;
        const auto lengthSquared = axis.length_squared();

        {
            const auto t = lengthSquared > 1e-12f ? clamp01((aOrigin - aA).dot_product(axis) / lengthSquared) : 0.0f;
            const auto spine = aA + axis * t;
            const auto offset = aOrigin - spine;

            if (offset.length_squared() <= aRadius * aRadius) {
                const auto length = offset.length();

                return ray_hit{0, length > 1e-12f ? offset * (1.0f / length) : aDirection * -1.0f};
            }
        }

        std::optional<ray_hit> best;

        const auto keep = [&](const std::optional<ray_hit> &aHit) {
            if (aHit && (!best || aHit->distance < best->distance)) best = aHit;
        };

        keep(ray_sphere(aOrigin, aDirection, aMax, aA, aRadius));
        keep(ray_sphere(aOrigin, aDirection, aMax, aB, aRadius));

        if (lengthSquared > 1e-12f) {
            //! **the cylinder between**, in the plane across its axis
            const auto unit = axis * (1.0f / std::sqrt(lengthSquared));
            const auto relative = aOrigin - aA;
            const auto d = aDirection - unit * aDirection.dot_product(unit);
            const auto m = relative - unit * relative.dot_product(unit);
            const auto a = d.length_squared();

            if (a > 1e-12f) {
                const auto b = m.dot_product(d);
                const auto c = m.length_squared() - aRadius * aRadius;
                const auto discriminant = b * b - a * c;

                if (discriminant >= 0) {
                    const auto t = (-b - std::sqrt(discriminant)) / a;

                    if (t >= 0 && t <= aMax) {
                        const auto at = aOrigin + aDirection * t;
                        const auto along = (at - aA).dot_product(unit);

                        if (along >= 0 && along * along <= lengthSquared)
                            keep(ray_hit{t, (at - (aA + unit * along)).normal()});
                    }
                }
            }
        }

        return best;
    }

    [[nodiscard]] std::optional<ray_hit> ray_triangle(const vector3_type &aOrigin, const vector3_type &aDirection,
        const floating_point_type aMax, const vector3_type &aA, const vector3_type &aB, const vector3_type &aC) {
        const auto edge1 = aB - aA, edge2 = aC - aA;

        if (aDirection.x == 0 && aDirection.z == 0 && aDirection.y != 0) {
            const auto area = edge1.x * edge2.z - edge1.z * edge2.x;

            if (std::abs(area) < 1e-12f) return std::nullopt;

            const auto inverse = 1.0f / area;
            const auto sx = aOrigin.x - aA.x, sz = aOrigin.z - aA.z;
            const auto u = (sx * edge2.z - sz * edge2.x) * inverse;

            if (u < -TRIANGLE_EDGE_SLACK || u > 1.0f + TRIANGLE_EDGE_SLACK) return std::nullopt;

            const auto v = (sz * edge1.x - sx * edge1.z) * inverse;

            if (v < -TRIANGLE_EDGE_SLACK || u + v > 1.0f + TRIANGLE_EDGE_SLACK) return std::nullopt;

            const auto t = (aA.y + edge1.y * u + edge2.y * v - aOrigin.y) / aDirection.y;

            if (t < 0 || t > aMax) return std::nullopt;

            auto normal = edge1.cross_product(edge2).normal();

            if (normal.dot_product(aDirection) > 0) normal = normal * -1.0f;

            return ray_hit{t, normal};
        }

        const auto p = aDirection.cross_product(edge2);
        const auto determinant = edge1.dot_product(p);

        if (std::abs(determinant) < 1e-12f) return std::nullopt;

        const auto inverse = 1.0f / determinant;
        const auto s = aOrigin - aA;
        const auto u = s.dot_product(p) * inverse;

        if (u < -TRIANGLE_EDGE_SLACK || u > 1.0f + TRIANGLE_EDGE_SLACK) return std::nullopt;

        const auto q = s.cross_product(edge1);
        const auto v = aDirection.dot_product(q) * inverse;

        if (v < -TRIANGLE_EDGE_SLACK || u + v > 1.0f + TRIANGLE_EDGE_SLACK) return std::nullopt;

        const auto t = edge2.dot_product(q) * inverse;

        if (t < 0 || t > aMax) return std::nullopt;

        auto normal = edge1.cross_product(edge2).normal();

        if (normal.dot_product(aDirection) > 0) normal = normal * -1.0f;

        return ray_hit{t, normal};
    }

    template <typename data_type>
    [[nodiscard]] std::optional<ray_hit> ray_triangles(const vector3_type &aOrigin, const vector3_type &aDirection,
        const floating_point_type aMax, const data_type &aData, const shape_kinematics &aKinematics) {
        if (aData.triangle_count() == 0) return std::nullopt;

        const bool turned = !(aKinematics.orientation == quaternion_type::identity);

        const auto toLocal = aKinematics.orientation.inverse_unit();
        const auto origin = turned ? toLocal * (aOrigin - aKinematics.position) : aOrigin - aKinematics.position;
        const auto direction = turned ? toLocal * aDirection : aDirection;
        const auto tip = origin + direction * aMax;

        typename data_type::bounds reach;

        reach.min = vector3_type::min(origin, tip);
        reach.max = vector3_type::max(origin, tip);

        thread_local std::vector<std::uint32_t> candidates;

        candidates.clear();

        aData.query(reach, candidates);

        std::optional<ray_hit> best;

        for (const auto candidate : candidates) {
            vector3_type a, b, c;

            aData.triangle(candidate, a, b, c);

            const auto hit = ray_triangle(origin, direction, best ? best->distance : aMax, a, b, c);

            if (hit && (!best || hit->distance < best->distance)) best = hit;
        }

        if (best && turned) best->normal = aKinematics.orientation * best->normal;

        return best;
    }

    [[nodiscard]] std::optional<ray_hit> ray_shape(const vector3_type &aOrigin, const vector3_type &aDirection,
        const floating_point_type aMax, const shape_type &aShape, const shape_kinematics &aKinematics) {
        return std::visit(overloaded{
            [&](const sphere_shape &aSphere) { return ray_sphere(aOrigin, aDirection, aMax, aKinematics.position, aSphere.radius); },
            [&](const box_shape &aBox) {
                const auto box = bounds_of(aBox, aKinematics.position);

                return ray_box(aOrigin, aDirection, aMax, box.min, box.max);
            },
            [&](const obb_shape &aBox) -> std::optional<ray_hit> {
                const auto toLocal = aKinematics.orientation.inverse_unit();
                auto hit = ray_box(toLocal * (aOrigin - aKinematics.position), toLocal * aDirection, aMax,
                    aBox.half_extents * -1.0f, aBox.half_extents);

                if (hit) hit->normal = aKinematics.orientation * hit->normal;

                return hit;
            },
            [&](const capsule_shape &aCapsule) {
                const auto spine = segment_of(aCapsule, aKinematics);

                return ray_capsule(aOrigin, aDirection, aMax, spine.a, spine.b, aCapsule.radius);
            },
            [&](const plane_shape &) -> std::optional<ray_hit> {
                const auto plane = plane_of(aKinematics);
                const auto height = (aOrigin - plane.point).dot_product(plane.normal);

                if (height <= 0) return ray_hit{0, plane.normal};

                const auto rate = aDirection.dot_product(plane.normal);

                if (rate >= 0) return std::nullopt;

                const auto t = height / -rate;

                if (t > aMax) return std::nullopt;

                return ray_hit{t, plane.normal};
            },
            [&](const triangle_shape &aTriangle) {
                const auto corners = world_triangle(aTriangle, aKinematics);

                return ray_triangle(aOrigin, aDirection, aMax, corners.a, corners.b, corners.c);
            },
            [&](const mesh_shape &aMesh) -> std::optional<ray_hit> {
                if (!aMesh.data) return std::nullopt;

                return ray_triangles(aOrigin, aDirection, aMax, *aMesh.data, aKinematics);
            },
            [&](const heightfield_shape &aField) -> std::optional<ray_hit> {
                if (!aField.data) return std::nullopt;

                return ray_triangles(aOrigin, aDirection, aMax, *aField.data, aKinematics);
            },
        }, aShape);
    }
    /// @}
}

std::optional<overlap> gdk::collisions::narrow_phase_ray_parts(const vector3_type &aOrigin, const vector3_type &aDirection,
    const floating_point_type aMaxDistance, std::span<const collider_part> aParts, const shape_kinematics &aCollider) {
    std::optional<ray_hit> best;

    for (const auto &part : aParts) {
        shape_kinematics placed;

        placed.position = aCollider.position + aCollider.orientation * part.position;
        placed.velocity = aCollider.velocity;
        placed.orientation = aCollider.orientation * part.rotation;

        const auto hit = ray_shape(aOrigin, aDirection, best ? best->distance : aMaxDistance, part.shape, placed);

        if (hit && (!best || hit->distance < best->distance)) best = hit;
    }

    if (!best) return std::nullopt;

    return overlap_of(*best, aOrigin, aDirection, aMaxDistance);
}

namespace {
    /// \name An upright cylinder against each kind of shape
    /// @{

    constexpr floating_point_type TOUCH = 1e-4f;

    struct flat_point final {
        floating_point_type x;
        floating_point_type z;
    };

    struct cylinder_frame final {
        floating_point_type x;
        floating_point_type z;
        floating_point_type low;
        floating_point_type high;
        floating_point_type radius;

        explicit cylinder_frame(const upright_cylinder &aCylinder)
        : x(aCylinder.base.x)
        , z(aCylinder.base.z)
        , low(aCylinder.base.y)
        , high(aCylinder.base.y + aCylinder.height)
        , radius(aCylinder.radius)
        {}

        [[nodiscard]] vector3_type nearest_to(const vector3_type &aPoint) const {
            const auto dx = aPoint.x - x, dz = aPoint.z - z;
            const auto across = std::sqrt(dx * dx + dz * dz);
            const auto scale = across > radius ? radius / across : 1.0f;

            return {x + dx * scale, std::max(low, std::min(high, aPoint.y)), z + dz * scale};
        }
    };

    [[nodiscard]] floating_point_type axis_to_segment(const cylinder_frame &aCylinder, const flat_point aA, const flat_point aB) {
        const auto ex = aB.x - aA.x, ez = aB.z - aA.z;
        const auto length = ex * ex + ez * ez;

        const auto t = length > 0 ? std::max(0.0f, std::min(1.0f, ((aCylinder.x - aA.x) * ex + (aCylinder.z - aA.z) * ez) / length))
            : 0.0f;

        const auto px = aA.x + ex * t - aCylinder.x, pz = aA.z + ez * t - aCylinder.z;

        return std::sqrt(px * px + pz * pz);
    }

    [[nodiscard]] bool triangle_in(const cylinder_frame &aCylinder, const vector3_type &aA, const vector3_type &aB,
        const vector3_type &aC) {
        std::array<vector3_type, 5> polygon{aA, aB, aC}, clipped{};
        std::size_t count = 3;

        const auto clip = [&](const floating_point_type aHeight, const bool aKeepAbove) {
            std::size_t kept = 0;

            const auto inside = [&](const vector3_type &aPoint) { return aKeepAbove ? aPoint.y > aHeight : aPoint.y < aHeight; };

            for (std::size_t i = 0; i < count; ++i) {
                const auto &from = polygon[i];
                const auto &to = polygon[(i + 1) % count];

                if (inside(from)) clipped[kept++] = from;

                if (inside(from) != inside(to)) {
                    const auto t = (aHeight - from.y) / (to.y - from.y);

                    clipped[kept++] = from + (to - from) * t;
                }
            }

            polygon = clipped;
            count = kept;
        };

        clip(aCylinder.low + TOUCH, true);

        if (count) clip(aCylinder.high - TOUCH, false);

        if (!count) return false;

        bool positive = false, negative = false;

        for (std::size_t i = 0; i < count; ++i) {
            const auto &from = polygon[i];
            const auto &to = polygon[(i + 1) % count];

            const auto turn = (to.x - from.x) * (aCylinder.z - from.z) - (to.z - from.z) * (aCylinder.x - from.x);

            if (turn > 0) positive = true;
            if (turn < 0) negative = true;
        }

        floating_point_type area = 0;

        for (std::size_t i = 0; i < count; ++i) {
            const auto &from = polygon[i];
            const auto &to = polygon[(i + 1) % count];

            area += from.x * to.z - to.x * from.z;
        }

        if (std::abs(area) > TOUCH && !(positive && negative)) return true;

        for (std::size_t i = 0; i < count; ++i) {
            const auto &from = polygon[i];
            const auto &to = polygon[(i + 1) % count];

            if (axis_to_segment(aCylinder, {from.x, from.z}, {to.x, to.z}) < aCylinder.radius - TOUCH) return true;
        }

        return false;
    }

    [[nodiscard]] bool box_in(const cylinder_frame &aCylinder, const vector3_type &aMin, const vector3_type &aMax) {
        if (aMin.y >= aCylinder.high - TOUCH || aMax.y <= aCylinder.low + TOUCH) return false;

        const auto dx = std::max({aMin.x - aCylinder.x, 0.0f, aCylinder.x - aMax.x});
        const auto dz = std::max({aMin.z - aCylinder.z, 0.0f, aCylinder.z - aMax.z});

        return dx * dx + dz * dz < (aCylinder.radius - TOUCH) * (aCylinder.radius - TOUCH);
    }

    [[nodiscard]] floating_point_type distance_to(const cylinder_frame &aCylinder, const vector3_type &aPoint) {
        return (aPoint - aCylinder.nearest_to(aPoint)).length();
    }

    [[nodiscard]] bool obb_in(const cylinder_frame &aCylinder, const obb_shape &aBox, const shape_kinematics &aKinematics) {
        const auto toLocal = aKinematics.orientation.inverse_unit();
        const auto middle = toLocal * (vector3_type{aCylinder.x, (aCylinder.low + aCylinder.high) * 0.5f, aCylinder.z}
            - aKinematics.position);

        const auto &h = aBox.half_extents;

        if (std::abs(middle.x) < h.x && std::abs(middle.y) < h.y && std::abs(middle.z) < h.z) return true;

        const auto corner = [&](const int aX, const int aY, const int aZ) {
            return aKinematics.position + aKinematics.orientation * vector3_type{aX ? h.x : -h.x, aY ? h.y : -h.y, aZ ? h.z : -h.z};
        };

        static constexpr int FACES[6][4][3] = {
            {{0, 0, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}}, {{1, 0, 0}, {1, 1, 0}, {1, 1, 1}, {1, 0, 1}},
            {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}}, {{0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}},
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}}, {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}};

        for (const auto &face : FACES) {
            const auto a = corner(face[0][0], face[0][1], face[0][2]), b = corner(face[1][0], face[1][1], face[1][2]);
            const auto c = corner(face[2][0], face[2][1], face[2][2]), d = corner(face[3][0], face[3][1], face[3][2]);

            if (triangle_in(aCylinder, a, b, c) || triangle_in(aCylinder, a, c, d)) return true;
        }

        return false;
    }

    [[nodiscard]] bool capsule_in(const cylinder_frame &aCylinder, const vector3_type &aA, const vector3_type &aB,
        const floating_point_type aRadius) {
        const auto along = [&](const floating_point_type t) { return distance_to(aCylinder, aA + (aB - aA) * t); };

        floating_point_type low = 0, high = 1;

        for (int each = 0; each < 48; ++each) {
            const auto first = low + (high - low) / 3, second = high - (high - low) / 3;

            if (along(first) < along(second)) high = second; else low = first;
        }

        return std::min({along(0), along(1), along((low + high) * 0.5f)}) < aRadius - TOUCH;
    }

    [[nodiscard]] bool plane_in(const cylinder_frame &aCylinder, const shape_kinematics &aKinematics) {
        const auto plane = plane_of(aKinematics);

        const auto height_of = [&](const floating_point_type aY) {
            return (vector3_type{aCylinder.x, aY, aCylinder.z} - plane.point).dot_product(plane.normal);
        };

        const auto sideways = std::sqrt(plane.normal.x * plane.normal.x + plane.normal.z * plane.normal.z);

        return std::min(height_of(aCylinder.low), height_of(aCylinder.high)) - aCylinder.radius * sideways < -TOUCH;
    }

    template <typename data_type>
    [[nodiscard]] bool triangles_in(const cylinder_frame &aCylinder, const data_type &aData, const shape_kinematics &aKinematics) {
        if (aData.triangle_count() == 0) return false;

        const auto toLocal = aKinematics.orientation.inverse_unit();

        typename data_type::bounds reach;

        for (int corner = 0; corner < 8; ++corner) {
            const auto local = toLocal * (vector3_type{corner & 1 ? aCylinder.x + aCylinder.radius : aCylinder.x - aCylinder.radius,
                corner & 2 ? aCylinder.high : aCylinder.low,
                corner & 4 ? aCylinder.z + aCylinder.radius : aCylinder.z - aCylinder.radius} - aKinematics.position);

            reach.min = corner ? vector3_type::min(reach.min, local) : local;
            reach.max = corner ? vector3_type::max(reach.max, local) : local;
        }

        thread_local std::vector<std::uint32_t> candidates;

        candidates.clear();

        aData.query(reach, candidates);

        for (const auto candidate : candidates) {
            vector3_type a, b, c;

            aData.triangle(candidate, a, b, c);

            if (triangle_in(aCylinder, aKinematics.position + aKinematics.orientation * a,
                aKinematics.position + aKinematics.orientation * b, aKinematics.position + aKinematics.orientation * c))
                return true;
        }

        return false;
    }

    [[nodiscard]] bool shape_in(const cylinder_frame &aCylinder, const shape_type &aShape, const shape_kinematics &aKinematics) {
        return std::visit(overloaded{
            [&](const sphere_shape &aSphere) { return distance_to(aCylinder, aKinematics.position) < aSphere.radius - TOUCH; },
            [&](const box_shape &aBox) {
                const auto box = bounds_of(aBox, aKinematics.position);

                return box_in(aCylinder, box.min, box.max);
            },
            [&](const obb_shape &aBox) { return obb_in(aCylinder, aBox, aKinematics); },
            [&](const capsule_shape &aCapsule) {
                const auto spine = segment_of(aCapsule, aKinematics);

                return capsule_in(aCylinder, spine.a, spine.b, aCapsule.radius);
            },
            [&](const plane_shape &) { return plane_in(aCylinder, aKinematics); },
            [&](const triangle_shape &aTriangle) {
                const auto corners = world_triangle(aTriangle, aKinematics);

                return triangle_in(aCylinder, corners.a, corners.b, corners.c);
            },
            [&](const mesh_shape &aMesh) { return aMesh.data && triangles_in(aCylinder, *aMesh.data, aKinematics); },
            [&](const heightfield_shape &aField) { return aField.data && triangles_in(aCylinder, *aField.data, aKinematics); },
        }, aShape);
    }
    /// @}
}

bool gdk::collisions::narrow_phase_cylinder_parts(const upright_cylinder &aCylinder, std::span<const collider_part> aParts,
    const shape_kinematics &aCollider) {
    if (aCylinder.radius <= 0 || aCylinder.height <= 0) return false;

    const cylinder_frame cylinder(aCylinder);

    for (const auto &part : aParts) {
        shape_kinematics placed;

        placed.position = aCollider.position + aCollider.orientation * part.position;
        placed.velocity = aCollider.velocity;
        placed.orientation = aCollider.orientation * part.rotation;

        if (shape_in(cylinder, part.shape, placed)) return true;
    }

    return false;
}

namespace {
    /// \name a parallelepiped against each kind of shape
    /// @{

    constexpr floating_point_type CUBE_TOUCH = 1e-5f;

    struct parallelepiped_frame final {
        vector3_type corner;
        std::array<vector3_type, 3> edges;

        std::array<vector3_type, 3> inverse;

        bool flat = false;

        vector3_type low, high;

        explicit parallelepiped_frame(const parallelepiped &aVolume)
        : corner(aVolume.corner)
        , edges(aVolume.edges) {
            const auto c0 = edges[1].cross_product(edges[2]);
            const auto c1 = edges[2].cross_product(edges[0]);
            const auto c2 = edges[0].cross_product(edges[1]);
            const auto determinant = edges[0].dot_product(c0);

            flat = std::abs(determinant) <= 1e-12f;

            if (!flat) inverse = {c0 * (1.0f / determinant), c1 * (1.0f / determinant), c2 * (1.0f / determinant)};

            low = high = corner;

            for (int each = 1; each < 8; ++each) {
                const auto at = world(vector3_type{each & 1 ? 1.0f : 0.0f, each & 2 ? 1.0f : 0.0f, each & 4 ? 1.0f : 0.0f});

                low = vector3_type::min(low, at);
                high = vector3_type::max(high, at);
            }
        }

        [[nodiscard]] vector3_type local(const vector3_type &aWorld) const {
            const auto offset = aWorld - corner;

            return {inverse[0].dot_product(offset), inverse[1].dot_product(offset), inverse[2].dot_product(offset)};
        }

        [[nodiscard]] vector3_type world(const vector3_type &aLocal) const {
            return corner + edges[0] * aLocal.x + edges[1] * aLocal.y + edges[2] * aLocal.z;
        }

        [[nodiscard]] bool inside(const vector3_type &aWorld) const {
            const auto at = local(aWorld);

            return at.x > CUBE_TOUCH && at.x < 1 - CUBE_TOUCH && at.y > CUBE_TOUCH && at.y < 1 - CUBE_TOUCH
                && at.z > CUBE_TOUCH && at.z < 1 - CUBE_TOUCH;
        }
    };

    [[nodiscard]] bool triangle_in_cube(const vector3_type &aA, const vector3_type &aB, const vector3_type &aC) {
        constexpr floating_point_type HALF = 0.5f - CUBE_TOUCH;

        const vector3_type middle{0.5f, 0.5f, 0.5f};
        const auto a = aA - middle, b = aB - middle, c = aC - middle;

        const auto apart = [&](const vector3_type &aAxis) {
            if (aAxis.x * aAxis.x + aAxis.y * aAxis.y + aAxis.z * aAxis.z < 1e-20f) return false;

            const auto pa = a.dot_product(aAxis), pb = b.dot_product(aAxis), pc = c.dot_product(aAxis);
            const auto reach = HALF * (std::abs(aAxis.x) + std::abs(aAxis.y) + std::abs(aAxis.z));

            return std::min({pa, pb, pc}) >= reach || std::max({pa, pb, pc}) <= -reach;
        };

        const std::array<vector3_type, 3> faces{vector3_type{1, 0, 0}, vector3_type{0, 1, 0}, vector3_type{0, 0, 1}};

        for (const auto &face : faces) if (apart(face)) return false;

        const std::array<vector3_type, 3> sides{b - a, c - b, a - c};

        if (apart(sides[0].cross_product(sides[1]))) return false;

        for (const auto &face : faces)
            for (const auto &side : sides)
                if (apart(face.cross_product(side))) return false;

        return true;
    }

    [[nodiscard]] bool triangle_in(const parallelepiped_frame &aVolume, const vector3_type &aA, const vector3_type &aB,
        const vector3_type &aC) {
        return triangle_in_cube(aVolume.local(aA), aVolume.local(aB), aVolume.local(aC));
    }

    template <typename corner_type>
    [[nodiscard]] bool box_faces_in(const parallelepiped_frame &aVolume, const corner_type &aCorner) {
        static constexpr int FACES[6][4][3] = {
            {{0, 0, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}}, {{1, 0, 0}, {1, 1, 0}, {1, 1, 1}, {1, 0, 1}},
            {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}}, {{0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}},
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}}, {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}};

        for (const auto &face : FACES) {
            const auto a = aCorner(face[0][0], face[0][1], face[0][2]), b = aCorner(face[1][0], face[1][1], face[1][2]);
            const auto c = aCorner(face[2][0], face[2][1], face[2][2]), d = aCorner(face[3][0], face[3][1], face[3][2]);

            if (triangle_in(aVolume, a, b, c) || triangle_in(aVolume, a, c, d)) return true;
        }

        return false;
    }

    template <typename other_type>
    [[nodiscard]] floating_point_type least_distance(const parallelepiped_frame &aVolume, const other_type &aOther,
        const vector3_type &aAlong) {
        const auto bound = aVolume.edges[0].dot_product(aVolume.edges[0]) + aVolume.edges[1].dot_product(aVolume.edges[1])
            + aVolume.edges[2].dot_product(aVolume.edges[2]) + aAlong.dot_product(aAlong);
        const auto step = bound > 0 ? 1.0f / bound : 1.0f;

        vector3_type x{0.5f, 0.5f, 0.5f};
        floating_point_type t = 0.5f;

        const auto clamp = [](const floating_point_type aValue) { return std::max(0.0f, std::min(1.0f, aValue)); };

        for (int each = 0; each < 200; ++each) {
            const auto gap = aVolume.world(x) - aOther(t);

            x = {clamp(x.x - step * gap.dot_product(aVolume.edges[0])), clamp(x.y - step * gap.dot_product(aVolume.edges[1])),
                clamp(x.z - step * gap.dot_product(aVolume.edges[2]))};
            t = clamp(t + step * gap.dot_product(aAlong));
        }

        return (aVolume.world(x) - aOther(t)).length();
    }

    [[nodiscard]] bool plane_in(const parallelepiped_frame &aVolume, const shape_kinematics &aKinematics) {
        const auto plane = plane_of(aKinematics);

        for (int each = 0; each < 8; ++each) {
            const auto at = aVolume.world({each & 1 ? 1.0f : 0.0f, each & 2 ? 1.0f : 0.0f, each & 4 ? 1.0f : 0.0f});

            if ((at - plane.point).dot_product(plane.normal) < -1e-4f) return true;
        }

        return false;
    }

    template <typename data_type>
    [[nodiscard]] bool triangles_in(const parallelepiped_frame &aVolume, const data_type &aData,
        const shape_kinematics &aKinematics) {
        if (aData.triangle_count() == 0) return false;

        const auto toLocal = aKinematics.orientation.inverse_unit();

        typename data_type::bounds reach;

        for (int corner = 0; corner < 8; ++corner) {
            const auto local = toLocal * (vector3_type{corner & 1 ? aVolume.high.x : aVolume.low.x,
                corner & 2 ? aVolume.high.y : aVolume.low.y, corner & 4 ? aVolume.high.z : aVolume.low.z} - aKinematics.position);

            reach.min = corner ? vector3_type::min(reach.min, local) : local;
            reach.max = corner ? vector3_type::max(reach.max, local) : local;
        }

        thread_local std::vector<std::uint32_t> candidates;

        candidates.clear();

        aData.query(reach, candidates);

        for (const auto candidate : candidates) {
            vector3_type a, b, c;

            aData.triangle(candidate, a, b, c);

            if (triangle_in(aVolume, aKinematics.position + aKinematics.orientation * a,
                aKinematics.position + aKinematics.orientation * b, aKinematics.position + aKinematics.orientation * c))
                return true;
        }

        return false;
    }

    [[nodiscard]] bool shape_in(const parallelepiped_frame &aVolume, const shape_type &aShape,
        const shape_kinematics &aKinematics) {
        return std::visit(overloaded{
            [&](const sphere_shape &aSphere) {
                if (aVolume.inside(aKinematics.position)) return true;

                return least_distance(aVolume, [&](floating_point_type) { return aKinematics.position; }, vector3_type::zero)
                    < aSphere.radius - 1e-4f;
            },
            [&](const box_shape &aBox) {
                const auto box = bounds_of(aBox, aKinematics.position);

                const auto middle = aVolume.world({0.5f, 0.5f, 0.5f});

                if (middle.x > box.min.x && middle.x < box.max.x && middle.y > box.min.y && middle.y < box.max.y
                    && middle.z > box.min.z && middle.z < box.max.z) return true;

                return box_faces_in(aVolume, [&](const int aX, const int aY, const int aZ) {
                    return vector3_type{aX ? box.max.x : box.min.x, aY ? box.max.y : box.min.y, aZ ? box.max.z : box.min.z};
                });
            },
            [&](const obb_shape &aBox) {
                const auto &h = aBox.half_extents;
                const auto middle = aKinematics.orientation.inverse_unit() * (aVolume.world({0.5f, 0.5f, 0.5f}) - aKinematics.position);

                if (std::abs(middle.x) < h.x && std::abs(middle.y) < h.y && std::abs(middle.z) < h.z) return true;

                return box_faces_in(aVolume, [&](const int aX, const int aY, const int aZ) {
                    return aKinematics.position + aKinematics.orientation * vector3_type{aX ? h.x : -h.x, aY ? h.y : -h.y,
                        aZ ? h.z : -h.z};
                });
            },
            [&](const capsule_shape &aCapsule) {
                const auto spine = segment_of(aCapsule, aKinematics);

                if (aVolume.inside(spine.a) || aVolume.inside(spine.b)) return true;

                return least_distance(aVolume, [&](const floating_point_type aT) { return spine.a + (spine.b - spine.a) * aT; },
                    spine.b - spine.a) < aCapsule.radius - 1e-4f;
            },
            [&](const plane_shape &) { return plane_in(aVolume, aKinematics); },
            [&](const triangle_shape &aTriangle) {
                const auto corners = world_triangle(aTriangle, aKinematics);

                return triangle_in(aVolume, corners.a, corners.b, corners.c);
            },
            [&](const mesh_shape &aMesh) { return aMesh.data && triangles_in(aVolume, *aMesh.data, aKinematics); },
            [&](const heightfield_shape &aField) { return aField.data && triangles_in(aVolume, *aField.data, aKinematics); },
        }, aShape);
    }
    /// @}
}

bool gdk::collisions::narrow_phase_parallelepiped_parts(const parallelepiped &aVolume,
    std::span<const collider_part> aParts, const shape_kinematics &aCollider) {
    const parallelepiped_frame volume(aVolume);

    if (volume.flat) return false;

    for (const auto &part : aParts) {
        shape_kinematics placed;

        placed.position = aCollider.position + aCollider.orientation * part.position;
        placed.velocity = aCollider.velocity;
        placed.orientation = aCollider.orientation * part.rotation;

        if (shape_in(volume, part.shape, placed)) return true;
    }

    return false;
}

gdk::collisions::vector3_type gdk::collisions::parallelepiped_low(const parallelepiped &aVolume) {
    return parallelepiped_frame(aVolume).low;
}

gdk::collisions::vector3_type gdk::collisions::parallelepiped_high(const parallelepiped &aVolume) {
    return parallelepiped_frame(aVolume).high;
}

