// © Joseph Cameron - All Rights Reserved

#include <iterator>
#include <gdk/collisions/impl_broadphase_grid.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <utility>

using namespace gdk::collisions;

std::size_t broadphase_cell_hasher::operator()(const broadphase_cell_key &aKey) const noexcept {
    // GOLDEN_RATIO_64BIT: 2^64 / φ, used in many hash combiners to scatter values.
    static constexpr std::uint64_t GOLDEN_RATIO_64BIT(0x9e3779b97f4a7c15ull);
    // SPLITMIX_MUL1/2: avalanche multipliers from SplitMix64 (public domain).
    static constexpr std::uint64_t SPLITMIX_MUL1_64BIT(0xbf58476d1ce4e5b9ull);
    static constexpr std::uint64_t SPLITMIX_MUL2_64BIT(0x94d049bb133111ebull);
    auto h = GOLDEN_RATIO_64BIT;
    auto mix = [&](int v) {
        std::uint64_t x = static_cast<std::uint64_t>(static_cast<std::uint32_t>(v));
        x += GOLDEN_RATIO_64BIT;
        x = (x ^ (x >> 30)) * SPLITMIX_MUL1_64BIT;
        x = (x ^ (x >> 27)) * SPLITMIX_MUL2_64BIT;
        x ^= (x >> 31);
        h ^= x + GOLDEN_RATIO_64BIT + (h << 6) + (h >> 2);
    };
    mix(aKey.x); mix(aKey.y); mix(aKey.z);
    return static_cast<std::size_t>(h);
}

broadphase_cell_key gdk::collisions::broadphase_cell_coord(const impl_collision_policy &aPolicy,
    const vector3_type &aValue) {
    return broadphase_cell_key{
        static_cast<int>(std::floor(aValue.x / aPolicy.BROAD_PHASE_CELL_SIZE)),
        static_cast<int>(std::floor(aValue.y / aPolicy.BROAD_PHASE_CELL_SIZE)),
        static_cast<int>(std::floor(aValue.z / aPolicy.BROAD_PHASE_CELL_SIZE))
    };
}

std::uint64_t gdk::collisions::broadphase_morton_code(const impl_collision_policy &aPolicy,
    const vector3_type &aValue) {
    const auto cell = broadphase_cell_coord(aPolicy, aValue);

    // Spread 21 bits out to every third bit, so three of them interleave into 63.
    const auto spread = [](std::uint64_t aBits) {
        aBits &= 0x1fffffull;
        aBits = (aBits | (aBits << 32)) & 0x1f00000000ffffull;
        aBits = (aBits | (aBits << 16)) & 0x1f0000ff0000ffull;
        aBits = (aBits | (aBits <<  8)) & 0x100f00f00f00f00full;
        aBits = (aBits | (aBits <<  4)) & 0x10c30c30c30c30c3ull;
        aBits = (aBits | (aBits <<  2)) & 0x1249249249249249ull;
        return aBits;
    };

    const auto bias = [](const int aValue) {
        return static_cast<std::uint64_t>(static_cast<std::int64_t>(aValue) + (1 << 20));
    };

    return spread(bias(cell.x)) | (spread(bias(cell.y)) << 1) | (spread(bias(cell.z)) << 2);
}

impl_broadphase_grid::impl_broadphase_grid(const impl_collision_policy &aPolicy)
: m_Policy(aPolicy)
{}

void impl_broadphase_grid::clear() {
    m_PreviousCellCount = m_Cells.size();
    m_Cells.clear();
    m_Unbinned.clear();
}

void impl_broadphase_grid::reserve(const std::size_t aBodyCount) {
    m_Cells.reserve(std::max(aBodyCount, m_PreviousCellCount));
}

void impl_broadphase_grid::insert_unbounded(const impl_collider_ptr_type &aBody, const body_kind aKind,
    const bool aIsStatic) {
    m_Unbinned.push_back(entry{aBody, aBody->id(), aKind, aIsStatic});
}

void impl_broadphase_grid::sort(std::vector<neighbour> &aNeighbours) {
    std::sort(aNeighbours.begin(), aNeighbours.end(),
        [](const neighbour &a, const neighbour &b) {
            if (a.kind != b.kind) return a.kind < b.kind;
            if (a.is_static != b.is_static) return a.is_static;
            return a.ptr->id() < b.ptr->id();
        });
}

void impl_broadphase_grid::take_live(const std::vector<entry> &aEntries, const impl_collider *aExclude,
    collider_id_set &aSeen, std::vector<neighbour> &aOut) const {
    for (const auto &entry : aEntries) {
        auto pLive = entry.ptr.lock();

        if (!pLive) {
            m_HasExpiredEntries.store(true, std::memory_order_relaxed);
            continue;
        }

        if (pLive.get() == aExclude) continue;
        if (!aSeen.insert(entry.id)) continue;

        aOut.push_back(neighbour{std::move(pLive), entry.kind, entry.is_static});
    }
}

void impl_broadphase_grid::compact_expired() {
    if (!m_HasExpiredEntries.exchange(false, std::memory_order_relaxed)) return;

    const auto drop_dead = [](std::vector<entry> &aEntries) {
        aEntries.erase(std::remove_if(aEntries.begin(), aEntries.end(),
            [](const entry &aEntry) { return aEntry.ptr.expired(); }), aEntries.end());
    };

    drop_dead(m_Unbinned);

    for (auto it = m_Cells.begin(); it != m_Cells.end();) {
        drop_dead(it->second);
        it = it->second.empty() ? m_Cells.erase(it) : std::next(it);
    }
}

void impl_broadphase_grid::insert(const impl_collider_ptr_type &aBody, const impl_collider::broadphase_bounds &aBounds,
    const body_kind aKind, const bool aIsStatic) {
    if (!std::isfinite(aBounds.min.x) || !std::isfinite(aBounds.min.y) || !std::isfinite(aBounds.min.z) ||
        !std::isfinite(aBounds.max.x) || !std::isfinite(aBounds.max.y) || !std::isfinite(aBounds.max.z)) {
        insert_unbounded(aBody, aKind, aIsStatic);
        return;
    }

    const auto max = cell_coord(aBounds.max);
    const auto min = cell_coord(aBounds.min);

    const auto span = static_cast<std::uint64_t>(max.x - min.x + 1)
        * static_cast<std::uint64_t>(max.y - min.y + 1)
        * static_cast<std::uint64_t>(max.z - min.z + 1);

    if (span > m_Policy.BROAD_PHASE_MAX_CELLS_PER_BODY) {
        insert_unbounded(aBody, aKind, aIsStatic);
        return;
    }

    for (int z = min.z; z <= max.z; ++z)
    for (int y = min.y; y <= max.y; ++y)
    for (int x = min.x; x <= max.x; ++x) {
        m_Cells[cell_key{x, y, z}].push_back(entry{aBody, aBody->id(), aKind, aIsStatic});
    }
}

void impl_broadphase_grid::gather_neighbours(const impl_collider &aCollider,
    const delta_time_type aDeltaTime, collider_id_set &aSeen,
    std::vector<neighbour> &aOutNeighbours) const {
    gather_overlapping_excluding(aCollider.broad_phase_swept_bounds(aDeltaTime), &aCollider,
        aSeen, aOutNeighbours);
}

void impl_broadphase_grid::gather_overlapping(const impl_collider::broadphase_bounds &aBounds,
    collider_id_set &aSeen, std::vector<neighbour> &aOutNeighbours) const {
    gather_overlapping_excluding(aBounds, nullptr, aSeen, aOutNeighbours);
}

void impl_broadphase_grid::gather_overlapping_excluding(const impl_collider::broadphase_bounds &aBounds,
    const impl_collider *aExclude, collider_id_set &aSeen,
    std::vector<neighbour> &aOutNeighbours) const {
    // Unbinned bodies are near everything by definition, so they join every result.
    take_live(m_Unbinned, aExclude, aSeen, aOutNeighbours);

    const auto max = cell_coord(aBounds.max);
    const auto min = cell_coord(aBounds.min);

    for (int z = min.z; z <= max.z; ++z)
    for (int y = min.y; y <= max.y; ++y)
    for (int x = min.x; x <= max.x; ++x) {
        const auto it = m_Cells.find(cell_key{x, y, z});
        if (it == m_Cells.end()) continue;
        take_live(it->second, aExclude, aSeen, aOutNeighbours);
    }
}
