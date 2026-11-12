// © Joseph Cameron - MIT License

#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_collision_profile.h>
#include <gdk/collisions/impl_narrow_phase.h>
#include <gdk/collisions/impl_shape_geometry.h>

#include <variant>

#include <algorithm>
#include <array>
#include <concepts>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <span>

using namespace gdk::collisions;
using namespace gdk::collisions::geometry;

namespace {
    constexpr gdk::collisions::floating_point_type ROUNDING_UNITS = 8;

    /// \brief how far apart two shapes are, and where
    struct separation final {
        floating_point_type distance;
        closest_points points;
    };

    /// \brief what to make of a contact that landed on a triangle's internal edge
    struct internal_edge_verdict final {
        /// \brief the subject is off this triangle's rim rather than over it
        bool doubtful = false;

        //! a replacement normal
        std::optional<vector3_type> normal;
    };

    [[nodiscard]] internal_edge_verdict corrected_internal_edge_normal(const std::uint8_t aInternalEdges,
                                                                       const triangle_points &aTriangle,
                                                                       const vector3_type &aContactPoint,
                                                                       const vector3_type &aContactNormal,
                                                                       const impl_std_tuning &aPolicy) {
        if (!aInternalEdges)
            return {};

        const auto precise = [](const vector3_type &aPoint) {
            return gdk::math::vector3<double>{aPoint.x, aPoint.y, aPoint.z};
        };
        const auto ab = precise(aTriangle.b) - precise(aTriangle.a);
        const auto ac = precise(aTriangle.c) - precise(aTriangle.a);
        const auto crossed = ab.cross_product(ac);
        const auto square = crossed.length_squared();
        if (square == 0)
            return {};
        const auto unit = crossed * (1 / std::sqrt(square));
        const vector3_type faceNormal{static_cast<floating_point_type>(unit.x),
                                      static_cast<floating_point_type>(unit.y),
                                      static_cast<floating_point_type>(unit.z)};

        const auto alignment = aContactNormal.dot_product(faceNormal);
        if (std::abs(alignment) >= 1.0f - aPolicy.FACE_CONTACT_COSINE_EPSILON)
            return {};

        const auto ap = precise(aContactPoint) - precise(aTriangle.a);
        // Cross products retain the small area lost by the Gram determinant on thin triangles.
        const auto v = ap.cross_product(ac).dot_product(crossed) / square;
        const auto w = ab.cross_product(ap).dot_product(crossed) / square;
        const auto u = 1.0f - v - w;

        const bool on[3] = {
            w <= aPolicy.BARYCENTRIC_EDGE_EPSILON, // edge a->b is opposite c
            u <= aPolicy.BARYCENTRIC_EDGE_EPSILON, // edge b->c is opposite a
            v <= aPolicy.BARYCENTRIC_EDGE_EPSILON, // edge c->a is opposite b
        };

        auto touched = false;

        for (int e = 0; e < 3; ++e) {
            if (!on[e])
                continue;
            if (!(aInternalEdges & (1u << e)))
                return {};
            touched = true;
        }

        if (!touched)
            return {};

        const auto snapped = alignment >= 0 ? faceNormal : faceNormal * -1.0f;

        return {std::abs(alignment) <= aPolicy.INTERNAL_EDGE_GHOST_COSINE, snapped};
    }

    [[nodiscard]] separation separation_at(
        const shape_type &aSubjectShape, const shape_kinematics &aSubject,
        const shape_type &aOtherShape, const shape_kinematics &aOther,
        const impl_std_tuning &aPolicy) {
        const auto subjectSegment = segment_of(aSubjectShape, aSubject);

        if (std::holds_alternative<obb_shape>(aOtherShape)) {
            const auto inverse = aOther.orientation.inverse_unit();
            const segment localSegment{
                inverse * (subjectSegment.a - aOther.position),
                inverse * (subjectSegment.b - aOther.position)};

            const auto &extents = std::get<obb_shape>(aOtherShape).half_extents;
            const box_bounds localBox{extents * -1.0f, extents};

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
        const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy) {
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

            const auto readable = readable_of(sample);
            if (readable) lastReadable = readable;

            const auto closing = readable ? -relativeVelocity.dot_product(*readable) : relativeSpeed;

            const auto touching = gap <= aPolicy.PENETRATION_DEPTH_EPSILON + rounding_of(sample);

            if (touching && (closing < aPolicy.MIN_SWEPT_SPEED || gap <= closing * aPolicy.TIME_OF_IMPACT_EPSILON))
                return contact_at(time, sample);

            if (relativeSpeed <= aPolicy.NORMALIZATION_THRESHOLD) return std::nullopt;

            if (closing < aPolicy.MIN_SWEPT_SPEED) return std::nullopt;

            if (gap >= previousGap) return touching ? std::optional(contact_at(time, sample)) : std::nullopt;
            previousGap = gap;

            const auto next = time + gap / closing;
            if (next > aDeltaTime) return std::nullopt;

            if (step == aPolicy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS) return contact_at(next, sample_at(next));

            time = next;
        }

        return std::nullopt;
    }

    struct projected_intervals final {
        floating_point_type subjectLow, subjectHigh, otherLow, otherHigh;
    };

    // With fixed orientations, every projected interval moves linearly. A contact exists only
    // where all axes' time intervals overlap; crossing one triangle's extended plane is insufficient.
    template <typename project_type>
    [[nodiscard]] std::optional<overlap> linear_sweep(std::span<const vector3_type> aAxes,
        const project_type aProject, const vector3_type &aRelativeVelocity,
        const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy) {
        delta_time_type entry = 0, exit = aDeltaTime;
        auto largestGap = -std::numeric_limits<floating_point_type>::infinity();
        auto overlapNormal = vector3_type::zero;
        auto entryNormal = vector3_type::zero;
        bool touching = true;

        for (auto axis : aAxes) {
            const auto square = axis.length_squared();
            if (square < aPolicy.NORMALIZATION_THRESHOLD) continue;
            axis = axis * (1.0f / std::sqrt(square));
            const auto ranges = aProject(axis);
            const auto below = ranges.otherLow - ranges.subjectHigh;
            const auto above = ranges.subjectLow - ranges.otherHigh;
            const auto gap = std::max(below, above);
            if (gap > largestGap + aPolicy.PENETRATION_DEPTH_EPSILON) {
                largestGap = gap;
                overlapNormal = above > below ? axis : axis * -1.0f;
            }
            if (gap > aPolicy.PENETRATION_DEPTH_EPSILON) touching = false;

            const auto speed = aRelativeVelocity.dot_product(axis);
            if (speed == 0) {
                if (gap > aPolicy.PENETRATION_DEPTH_EPSILON) return std::nullopt;
                continue;
            }
            auto first = (below - aPolicy.PENETRATION_DEPTH_EPSILON) / speed;
            auto last = (ranges.otherHigh - ranges.subjectLow + aPolicy.PENETRATION_DEPTH_EPSILON) / speed;
            if (first > last) std::swap(first, last);
            if (first > entry) {
                if (first > entry + aPolicy.AXIS_ENTRY_EPSILON || entryNormal.is_effectively_zero())
                    entryNormal = speed > 0 ? axis * -1.0f : axis;
                entry = first;
            }
            exit = std::min(exit, last);
            if (entry > exit) return std::nullopt;
        }

        overlap result;
        result.entry_time = touching ? 0 : entry;
        result.exit_time = exit;
        result.penetration = touching ? std::max(0.0f, -largestGap) : 0;
        result.contact_normal = touching ? overlapNormal : entryNormal;
        return result;
    }

    [[nodiscard]] std::optional<overlap> swept_box_triangle(
        const vector3_type &aBoxExtents, const quaternion_type &aBoxOrientation,
        const shape_kinematics &aBox, const triangle_shape &aTriangleShape,
        const shape_kinematics &aTriangle, const delta_time_type aDeltaTime,
        const impl_std_tuning &aPolicy) {
        const auto box = frame_of(aBoxExtents, aBox.position, aBoxOrientation);
        const auto triangle = world_triangle(aTriangleShape, aTriangle);
        const std::array<vector3_type, 3> edges{
            triangle.b - triangle.a, triangle.c - triangle.b, triangle.a - triangle.c};
        std::array<vector3_type, 13> axes;
        axes[0] = edges[0].cross_product(edges[1]);
        std::size_t index = 1;
        for (const auto axis : box.axis) axes[index++] = axis;
        for (const auto axis : box.axis)
            for (const auto edge : edges) axes[index++] = axis.cross_product(edge);
        auto result = linear_sweep(axes, [&](const vector3_type &aAxis) {
            const auto centre = box.centre.dot_product(aAxis);
            const auto radius = projected_radius(box, aAxis);
            const auto a = triangle.a.dot_product(aAxis), b = triangle.b.dot_product(aAxis),
                c = triangle.c.dot_product(aAxis);
            return projected_intervals{centre - radius, centre + radius, std::min({a, b, c}), std::max({a, b, c})};
        }, aBox.velocity - aTriangle.velocity, aDeltaTime, aPolicy);
        if (result) {
            const shape_kinematics moved{aTriangle.position + aTriangle.velocity * result->entry_time,
                aTriangle.velocity, aTriangle.orientation};
            result->contact_point = closest_point_on_triangle(world_triangle(aTriangleShape, moved),
                aBox.position + aBox.velocity * result->entry_time);
        }
        return result;
    }

    [[nodiscard]] std::optional<overlap> swept_box_pair(
        const vector3_type &aSubjectExtents, const quaternion_type &aSubjectOrientation,
        const shape_kinematics &aSubject,
        const vector3_type &aOtherExtents, const quaternion_type &aOtherOrientation,
        const shape_kinematics &aOther, const delta_time_type aDeltaTime,
        const impl_std_tuning &aPolicy) {
        const auto subject = frame_of(aSubjectExtents, aSubject.position, aSubjectOrientation);
        const auto other = frame_of(aOtherExtents, aOther.position, aOtherOrientation);
        std::array<vector3_type, 15> axes;
        std::size_t index = 0;
        for (const auto axis : subject.axis) axes[index++] = axis;
        for (const auto axis : other.axis) axes[index++] = axis;
        for (const auto first : subject.axis)
            for (const auto second : other.axis) axes[index++] = first.cross_product(second);
        auto result = linear_sweep(axes, [&](const vector3_type &aAxis) {
            const auto first = subject.centre.dot_product(aAxis), second = other.centre.dot_product(aAxis);
            const auto firstRadius = projected_radius(subject, aAxis), secondRadius = projected_radius(other, aAxis);
            return projected_intervals{first - firstRadius, first + firstRadius, second - secondRadius, second + secondRadius};
        }, aSubject.velocity - aOther.velocity, aDeltaTime, aPolicy);
        if (result) result->contact_point = other.centre + aOther.velocity * result->entry_time
            + result->contact_normal * projected_radius(other, result->contact_normal);
        return result;
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
        const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy) {
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
        const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy) {
        const auto p = aSubject.position - aOther.position;
        const auto v = aSubject.velocity - aOther.velocity;
        const auto r = aSubjectShape.radius + aOtherShape.radius;
        const auto a = v.dot_product(v);

        if (a < aPolicy.MIN_SWEPT_SPEED * aPolicy.MIN_SWEPT_SPEED) {
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
            : (v.length_squared() > aPolicy.MIN_SWEPT_SPEED * aPolicy.MIN_SWEPT_SPEED
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


    struct axis_list final {
        std::array<std::size_t, 3> axes{};
        std::size_t count = 0;

        void push(const std::size_t aAxis) { axes[count++] = aAxis; }

        [[nodiscard]] std::span<const std::size_t> view() const { return {axes.data(), count}; }
    };

    std::optional<overlap> box_vs_box(const box_shape &aSubjectShape, const shape_kinematics &aSubject,
        const box_shape &aOtherShape, const shape_kinematics &aOther,
        const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy) {
        const auto thisBroadPhaseBounds = bounds_of(aSubjectShape, aSubject.position);
        const auto otherBroadPhaseBounds = bounds_of(aOtherShape, aOther.position);
        const auto relativeVelocity = aSubject.velocity - aOther.velocity;

        const auto contact_at = [&](const std::span<const std::size_t> aAxes, const delta_time_type aToi) {
            const auto boundsA = bounds_of(aSubjectShape, aSubject.position + aSubject.velocity * aToi);
            const auto boundsB = bounds_of(aOtherShape, aOther.position + aOther.velocity * aToi);

            struct contact final {
                vector3_type normal = vector3_type::zero;
                floating_point_type penetration = std::numeric_limits<floating_point_type>::infinity();
                vector3_type point = vector3_type::zero;
            } best;

            for (const auto i : aAxes) {
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
        auto staticAxis = std::size_t{0};
        axis_list toiAxes;

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
                    staticAxis = i;
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
                toiAxes.count = 0;
                toiAxes.push(i);
            } else if (std::abs(axisEntry - entryMax) <= aPolicy.AXIS_ENTRY_EPSILON) {
                toiAxes.push(i);
            }

            exitMin = std::min(exitMin, axisExit);

            if (entryMax > exitMin) return std::nullopt;
        }

        if (!anyMoving) {
            if (!std::isfinite(minPenetration)) return std::nullopt;

            const std::size_t only[1] = {staticAxis};
            const auto contact = contact_at(only, 0.0f);
            overlap result;
            result.entry_time = 0.0f;
            result.exit_time = aDeltaTime;
            result.penetration = contact.penetration;
            result.contact_normal = contact.normal;
            result.contact_point = contact.point;
            return result;
        }

        if (exitMin < 0.0f || entryMax > aDeltaTime) return std::nullopt;

        {
            axis_list valid;
            for (const auto i : toiAxes.view()) {
                const auto vi = relativeVelocity[i];
                const floating_point_type invVi = 1.0f / vi;
                const floating_point_type t1 = (otherBroadPhaseBounds.min[i] - thisBroadPhaseBounds.max[i]) * invVi;
                const floating_point_type t2 = (otherBroadPhaseBounds.max[i] - thisBroadPhaseBounds.min[i]) * invVi;
                const floating_point_type axisEntry = std::min(t1, t2);
                const floating_point_type axisExit  = std::max(t1, t2);

                if (axisEntry <= exitMin + aPolicy.AXIS_ENTRY_EPSILON && 
                    axisExit >= entryMax - aPolicy.AXIS_ENTRY_EPSILON) 
                    valid.push(i);
            }
            toiAxes = valid;
        }

        if (toiAxes.count == 0) return std::nullopt;

        const auto toi = std::max(entryMax, 0.0f);

        static constexpr std::size_t EVERY_AXIS[3] = {0, 1, 2};

        const auto contact = contact_at(toi <= aPolicy.TIME_OF_IMPACT_EPSILON
            ? std::span<const std::size_t>(EVERY_AXIS) : toiAxes.view(), toi);
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
    [[nodiscard]] bool is_closing_contact(const overlap &aOverlap, const shape_kinematics &aSubject,
        const shape_kinematics &aOther, const impl_std_tuning &aPolicy) {
        return aOverlap.penetration > aPolicy.PENETRATION_DEPTH_EPSILON
            || is_closing(aOverlap, aSubject, aOther, aPolicy);
    }

    template <typename shape> concept rounded_shape = std::same_as<shape, sphere_shape> || std::same_as<shape, capsule_shape>;
    template <typename shape> concept box_like_shape = std::same_as<shape, box_shape> || std::same_as<shape, obb_shape>;
    template <typename shape> concept solid_shape = rounded_shape<shape> || box_like_shape<shape>;
    template <typename shape> concept surface_shape = std::same_as<shape, mesh_shape> || std::same_as<shape, heightfield_shape>;

    template <box_like_shape shape>
    [[nodiscard]] quaternion_type orientation_of(const shape_kinematics &aKinematics) {
        return std::same_as<shape, obb_shape> ? aKinematics.orientation : quaternion_type::identity;
    }

    struct narrow_phase final {
        const shape_type &subject_shape;
        const shape_kinematics &subject;
        const shape_type &other_shape;
        const shape_kinematics &other;
        delta_time_type deltaTime;
        const impl_std_tuning &policy;

        template <typename source_type>
        [[nodiscard]] std::optional<overlap> against_triangles(const shape_type &aSubjectShape,
            const shape_kinematics &aSubject, const source_type &aMesh,
            const shape_kinematics &aMeshKinematics) const {
            if (!aMesh.data || aMesh.data->triangle_count() == 0) return std::nullopt;

            const auto extents = shape_extents(aSubjectShape, aSubject.orientation);
            const auto sweep = (aSubject.velocity - aMeshKinematics.velocity) * deltaTime;

            thread_local std::vector<std::uint32_t> candidates;

            candidates.clear();

            GDK_COLLISION_PROFILE_COUNT(mesh_queries, 1);
            GDK_COLLISION_PROFILE_BEGIN(bvh_query_ns);

            if (std::holds_alternative<plane_shape>(aSubjectShape)) {
                const auto toLocal = aMeshKinematics.orientation.inverse_unit();
                const auto normal = toLocal * (aSubject.orientation * vector3_type{0, 1, 0});

                aMesh.data->query_below(toLocal * (aSubject.position - aMeshKinematics.position), normal,
                    std::max(0.0f, (toLocal * sweep).dot_product(normal)) + policy.PENETRATION_DEPTH_EPSILON, candidates);
            }
            else if (std::isinf(extents.x) || std::isinf(extents.y) || std::isinf(extents.z)) {
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

            GDK_COLLISION_PROFILE_END(bvh_query_ns);
            GDK_COLLISION_PROFILE_COUNT(mesh_candidates, candidates.size());
            GDK_COLLISION_PROFILE_BEGIN(triangle_test_ns);

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

            GDK_COLLISION_PROFILE_END(triangle_test_ns);

            return nearest ? nearest : doubtful;
        }

        [[nodiscard]] std::optional<overlap> contact(const sphere_shape &aSubject, const sphere_shape &aOther) const {
            return sphere_vs_sphere(aSubject, subject, aOther, other, deltaTime, policy);
        }

        template <rounded_shape subject_type, typename other_type>
        requires (solid_shape<other_type> || std::same_as<other_type, triangle_shape>)
            && (!std::same_as<subject_type, sphere_shape> || !std::same_as<other_type, sphere_shape>)
        [[nodiscard]] std::optional<overlap> contact(const subject_type &aSubject, const other_type &aOther) const {
            return swept_by_separation(aSubject, subject, aOther, other, deltaTime, policy);
        }

        [[nodiscard]] std::optional<overlap> contact(const box_shape &aSubject, const box_shape &aOther) const {
            return box_vs_box(aSubject, subject, aOther, other, deltaTime, policy);
        }

        template <box_like_shape subject_type, box_like_shape other_type>
        requires (!std::same_as<subject_type, box_shape> || !std::same_as<other_type, box_shape>)
        [[nodiscard]] std::optional<overlap> contact(const subject_type &aSubject, const other_type &aOther) const {
            return swept_box_pair(aSubject.half_extents, orientation_of<subject_type>(subject), subject,
                aOther.half_extents, orientation_of<other_type>(other), other, deltaTime, policy);
        }

        template <box_like_shape subject_type>
        [[nodiscard]] std::optional<overlap> contact(const subject_type &aSubject, const triangle_shape &aOther) const {
            return swept_box_triangle(aSubject.half_extents, orientation_of<subject_type>(subject), subject,
                aOther, other, deltaTime, policy);
        }

        template <typename subject_type>
        requires solid_shape<subject_type> || std::same_as<subject_type, triangle_shape>
        [[nodiscard]] std::optional<overlap> contact(const subject_type &, const plane_shape &) const {
            return vs_plane(subject_shape, subject, other, deltaTime, policy);
        }

        template <typename subject_type, surface_shape other_type>
        requires solid_shape<subject_type> || std::same_as<subject_type, plane_shape>
        [[nodiscard]] std::optional<overlap> contact(const subject_type &, const other_type &aOther) const {
            return against_triangles(subject_shape, subject, aOther, other);
        }

        template <typename subject_type, typename other_type>
        [[nodiscard]] std::optional<overlap> operator()(const subject_type &aSubject, const other_type &aOther) const {
            if constexpr (requires { contact(aSubject, aOther); }) return contact(aSubject, aOther);
            else if constexpr (requires { contact(aOther, aSubject); }) {
                auto result = narrow_phase{other_shape, other, subject_shape, subject, deltaTime, policy}.contact(aOther, aSubject);
                if (result) result->contact_normal = result->contact_normal * -1.0f;
                return result;
            }
            else if constexpr (std::same_as<subject_type, plane_shape> && std::same_as<other_type, plane_shape>)
                throw exception("gdk::collision: half-space against half-space is not a "
                    "supported pair -- two infinite planes have no bounded overlap to report");
            else if constexpr (std::same_as<subject_type, triangle_shape> && std::same_as<other_type, triangle_shape>)
                throw exception("gdk::collision: triangle against triangle was reached, which "
                    "should be impossible -- the mesh pair is intercepted before decomposition");
            else return std::nullopt;
        }
    };
}

bool gdk::collisions::is_closing(const overlap &aOverlap, const shape_kinematics &aSubject,
    const shape_kinematics &aOther, const impl_std_tuning &aPolicy) {
    const auto relativeVelocity = aSubject.velocity - aOther.velocity;
    return relativeVelocity.dot_product(aOverlap.contact_normal) <= -aPolicy.MIN_SWEPT_SPEED;
}

std::optional<overlap> gdk::collisions::narrow_phase_overlap(
    const shape_type &aSubjectShape, const shape_kinematics &aSubject,
    const shape_type &aOtherShape, const shape_kinematics &aOther,
    const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy) {
    auto result = std::visit(narrow_phase{aSubjectShape, aSubject, aOtherShape, aOther, aDeltaTime, aPolicy},
        aSubjectShape, aOtherShape);

    if (result && !is_closing_contact(*result, aSubject, aOther, aPolicy)) return std::nullopt;

    return result;
}

std::optional<overlap> gdk::collisions::narrow_phase_overlap_parts(
    std::span<const collider_part> aSubjectParts, const shape_kinematics &aSubject,
    std::span<const collider_part> aOtherParts, const shape_kinematics &aOther,
    const delta_time_type aDeltaTime, const impl_std_tuning &aPolicy) {
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
