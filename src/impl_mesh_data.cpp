// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_mesh_data.h>

#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <numeric>
#include <utility>

using namespace gdk::collisions;

namespace {
    constexpr std::uint32_t TRIANGLES_PER_LEAF = 4;

    [[nodiscard]] bool overlaps(const impl_mesh_data::bounds &aA, const impl_mesh_data::bounds &aB) {
        return aA.min.x <= aB.max.x && aA.max.x >= aB.min.x
            && aA.min.y <= aB.max.y && aA.max.y >= aB.min.y
            && aA.min.z <= aB.max.z && aA.max.z >= aB.min.z;
    }
}

mesh_data_ptr_type impl_mesh_data::make(std::vector<vector3_type> aVertices,
    std::vector<std::uint32_t> aIndices, const floating_point_type aCoplanarTolerance) {
    if (aIndices.size() % 3 != 0)
        throw exception("gdk::collision: mesh index count is not a multiple of three");

    for (const auto index : aIndices)
        if (index >= aVertices.size())
            throw exception("gdk::collision: mesh index is out of range of its vertices");

    return mesh_data_ptr_type(new impl_mesh_data(std::move(aVertices), std::move(aIndices),
        aCoplanarTolerance));
}

impl_mesh_data::impl_mesh_data(std::vector<vector3_type> aVertices,
    std::vector<std::uint32_t> aIndices, const floating_point_type aCoplanarTolerance)
: m_Vertices(std::move(aVertices))
, m_Indices(std::move(aIndices))
{
    const auto count = static_cast<std::uint32_t>(triangle_count());
    if (count == 0) return;

    m_Order.resize(count);
    std::iota(m_Order.begin(), m_Order.end(), 0u);

    std::vector<vector3_type> centroids(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        vector3_type a, b, c;
        triangle(i, a, b, c);
        centroids[i] = (a + b + c) / 3.0f;
    }

    m_Nodes.reserve(2 * count);
    build(0, count, centroids);

    flag_internal_edges(aCoplanarTolerance);
}

void impl_mesh_data::flag_internal_edges(const floating_point_type aCoplanarTolerance) {
    const auto count = static_cast<std::uint32_t>(triangle_count());
    m_InternalEdges.assign(count, 0);

    std::map<std::array<floating_point_type, 3>, std::uint32_t> welded;
    std::vector<std::uint32_t> canonical(m_Vertices.size());

    for (std::size_t i = 0; i < m_Vertices.size(); ++i) {
        const std::array<floating_point_type, 3> key{
            m_Vertices[i].x, m_Vertices[i].y, m_Vertices[i].z};

        canonical[i] = welded.emplace(key, static_cast<std::uint32_t>(welded.size())).first->second;
    }

    std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edges;

    const auto edge_key = [&](const std::uint32_t aTriangle, const int aEdge) {
        const auto base = static_cast<std::size_t>(aTriangle) * 3;
        const auto first = canonical[m_Indices[base + aEdge]];
        const auto second = canonical[m_Indices[base + (aEdge + 1) % 3]];
        return std::make_pair(std::min(first, second), std::max(first, second));
    };

    for (std::uint32_t t = 0; t < count; ++t)
        for (int e = 0; e < 3; ++e) edges[edge_key(t, e)].push_back(t);

    const auto normal_of = [this](const std::uint32_t aTriangle) {
        vector3_type a, b, c;
        triangle(aTriangle, a, b, c);
        return (b - a).cross_product(c - a).normal();
    };

    for (std::uint32_t t = 0; t < count; ++t) {
        const auto normal = normal_of(t);

        for (int e = 0; e < 3; ++e) {
            const auto &sharing = edges[edge_key(t, e)];

            if (sharing.size() != 2) continue;

            const auto neighbour = sharing[0] == t ? sharing[1] : sharing[0];

            if (normal.dot_product(normal_of(neighbour)) >= 1.0f - aCoplanarTolerance)
                m_InternalEdges[t] |= static_cast<std::uint8_t>(1u << e);
        }
    }
}

std::uint8_t impl_mesh_data::internal_edges(const std::uint32_t aTriangle) const {
    return m_InternalEdges[aTriangle];
}

std::size_t impl_mesh_data::triangle_count() const {
    return m_Indices.size() / 3;
}

void impl_mesh_data::triangle(const std::uint32_t aTriangle, vector3_type &aA,
    vector3_type &aB, vector3_type &aC) const {
    const auto base = static_cast<std::size_t>(aTriangle) * 3;
    aA = m_Vertices[m_Indices[base + 0]];
    aB = m_Vertices[m_Indices[base + 1]];
    aC = m_Vertices[m_Indices[base + 2]];
}

mesh_triangle impl_mesh_data::triangle(const std::size_t aTriangle) const {
    mesh_triangle out;
    triangle(static_cast<std::uint32_t>(aTriangle), out.a, out.b, out.c);
    return out;
}

impl_mesh_data::bounds impl_mesh_data::bounds_of(const std::uint32_t aTriangle) const {
    vector3_type a, b, c;
    triangle(aTriangle, a, b, c);
    return bounds{
        vector3_type::min(a, vector3_type::min(b, c)),
        vector3_type::max(a, vector3_type::max(b, c))};
}

const impl_mesh_data::bounds &impl_mesh_data::root_bounds() const {
    static const bounds empty{
        vector3_type{std::numeric_limits<floating_point_type>::infinity(),
            std::numeric_limits<floating_point_type>::infinity(),
            std::numeric_limits<floating_point_type>::infinity()},
        vector3_type{-std::numeric_limits<floating_point_type>::infinity(),
            -std::numeric_limits<floating_point_type>::infinity(),
            -std::numeric_limits<floating_point_type>::infinity()}};

    return m_Nodes.empty() ? empty : m_Nodes.front().box;
}

std::uint32_t impl_mesh_data::build(const std::uint32_t aFirst, const std::uint32_t aCount,
    const std::vector<vector3_type> &aCentroids) {
    const auto self = static_cast<std::uint32_t>(m_Nodes.size());
    m_Nodes.push_back(node{});

    auto box = bounds_of(m_Order[aFirst]);
    for (std::uint32_t i = 1; i < aCount; ++i) {
        const auto b = bounds_of(m_Order[aFirst + i]);
        box.min = vector3_type::min(box.min, b.min);
        box.max = vector3_type::max(box.max, b.max);
    }
    m_Nodes[self].box = box;

    if (aCount <= TRIANGLES_PER_LEAF) {
        std::sort(m_Order.begin() + aFirst, m_Order.begin() + aFirst + aCount);

        m_Nodes[self].first = aFirst;
        m_Nodes[self].count = aCount;
        return self;
    }

    const auto extent = box.max - box.min;
    const int axis = extent.x > extent.y
        ? (extent.x > extent.z ? 0 : 2)
        : (extent.y > extent.z ? 1 : 2);

    const auto component = [axis](const vector3_type &aVector) {
        return axis == 0 ? aVector.x : (axis == 1 ? aVector.y : aVector.z);
    };

    const auto begin = m_Order.begin() + aFirst;
    const auto middle = begin + aCount / 2;
    std::nth_element(begin, middle, begin + aCount,
        [&](const std::uint32_t aLeft, const std::uint32_t aRight) {
            const auto left = component(aCentroids[aLeft]);
            const auto right = component(aCentroids[aRight]);
            return left != right ? left < right : aLeft < aRight;
        });

    const auto half = aCount / 2;
    build(aFirst, half, aCentroids);
    m_Nodes[self].right = build(aFirst + half, aCount - half, aCentroids);
    return self;
}

void impl_mesh_data::query(const bounds &aQuery, std::vector<std::uint32_t> &aOut) const {
    if (m_Nodes.empty()) return;

    constexpr std::size_t MAX_DEPTH = 64;
    std::uint32_t stack[MAX_DEPTH];
    std::size_t depth = 0;

    stack[depth++] = 0;

    while (depth > 0) {
        const auto index = stack[--depth];

        const auto &current = m_Nodes[index];
        if (!overlaps(current.box, aQuery)) continue;

        if (current.count > 0) {
            for (std::uint32_t i = 0; i < current.count; ++i) {
                const auto triangle = m_Order[current.first + i];

                vector3_type a;
                vector3_type b;
                vector3_type c;
                this->triangle(triangle, a, b, c);

                bounds box;
                box.min = vector3_type::min(a, vector3_type::min(b, c));
                box.max = vector3_type::max(a, vector3_type::max(b, c));

                if (overlaps(box, aQuery)) aOut.push_back(triangle);
            }

            continue;
        }

        stack[depth++] = current.right;
        stack[depth++] = index + 1;
    }
}

void impl_mesh_data::all(std::vector<std::uint32_t> &aOut) const {
    aOut.insert(aOut.end(), m_Order.begin(), m_Order.end());
}
