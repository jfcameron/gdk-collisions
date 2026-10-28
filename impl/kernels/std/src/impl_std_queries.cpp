// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_std_queries.h>
#include <gdk/collisions/impl_shape_geometry.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <variant>
#include <vector>

using namespace gdk::collisions;
using namespace gdk::collisions::geometry;

namespace {

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

        const auto closest = offset - aDirection * b;
        const auto discriminant = aRadius * aRadius - closest.length_squared();

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
            const auto unit = axis * (1.0f / std::sqrt(lengthSquared));
            const auto relative = aOrigin - aA;
            const auto d = aDirection - unit * aDirection.dot_product(unit);
            const auto m = relative - unit * relative.dot_product(unit);
            const auto a = d.length_squared();

            if (a > 1e-12f) {
                const auto nearest = -m.dot_product(d) / a;
                const auto closest = m + d * nearest;
                const auto discriminant = (aRadius * aRadius - closest.length_squared()) / a;

                if (discriminant >= 0) {
                    const auto t = nearest - std::sqrt(discriminant);

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
}

std::optional<overlap> gdk::collisions::ray_against_parts(const vector3_type &aOrigin, const vector3_type &aDirection,
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
}

bool gdk::collisions::cylinder_meets_parts(const upright_cylinder &aCylinder, std::span<const collider_part> aParts,
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

    [[nodiscard]] std::array<triangle_points, 12> faces_of(const parallelepiped_frame &aVolume) {
        static constexpr int FACES[6][4][3] = {
            {{0, 0, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}}, {{1, 0, 0}, {1, 1, 0}, {1, 1, 1}, {1, 0, 1}},
            {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}}, {{0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}},
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}}, {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}};

        const auto corner = [&](const int (&aCorner)[3]) {
            return aVolume.world({static_cast<floating_point_type>(aCorner[0]), static_cast<floating_point_type>(aCorner[1]),
                static_cast<floating_point_type>(aCorner[2])});
        };

        std::array<triangle_points, 12> out;

        for (std::size_t face = 0; face < 6; ++face) {
            const auto a = corner(FACES[face][0]), b = corner(FACES[face][1]);
            const auto c = corner(FACES[face][2]), d = corner(FACES[face][3]);

            out[face * 2] = {a, b, c};
            out[face * 2 + 1] = {a, c, d};
        }

        return out;
    }

    [[nodiscard]] bool segment_crosses(const parallelepiped_frame &aVolume, const vector3_type &aA, const vector3_type &aB) {
        const auto from = aVolume.local(aA), to = aVolume.local(aB);

        floating_point_type enter = 0, leave = 1;

        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto start = from[axis], delta = to[axis] - start;

            if (std::abs(delta) < 1e-12f) {
                if (start <= CUBE_TOUCH || start >= 1 - CUBE_TOUCH) return false;

                continue;
            }

            auto low = (CUBE_TOUCH - start) / delta, high = (1 - CUBE_TOUCH - start) / delta;

            if (low > high) std::swap(low, high);

            enter = std::max(enter, low);
            leave = std::min(leave, high);

            if (enter >= leave) return false;
        }

        return true;
    }

    [[nodiscard]] floating_point_type distance_to_faces(const parallelepiped_frame &aVolume, const vector3_type &aA,
        const vector3_type &aB) {
        static const impl_std_tuning POLICY{};

        auto least = std::numeric_limits<floating_point_type>::infinity();

        for (const auto &face : faces_of(aVolume)) {
            const auto points = closest_points_segment_triangle(segment{aA, aB}, face, POLICY);

            least = std::min(least, (points.on_first - points.on_second).length());
        }

        return least;
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

                return distance_to_faces(aVolume, aKinematics.position, aKinematics.position) < aSphere.radius - 1e-4f;
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

                if (segment_crosses(aVolume, spine.a, spine.b)) return true;

                return distance_to_faces(aVolume, spine.a, spine.b) < aCapsule.radius - 1e-4f;
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
}

bool gdk::collisions::parallelepiped_meets_parts(const parallelepiped &aVolume,
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
