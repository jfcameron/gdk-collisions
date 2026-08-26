// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/impl_dynamic_broadphase_grid.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

using namespace gdk::collisions;

namespace {
    [[nodiscard]] std::size_t next_power_of_two(const std::size_t aValue) {
        std::size_t capacity = 64;
        while (capacity < aValue) capacity <<= 1;
        return capacity;
    }
}

impl_dynamic_broadphase_grid::impl_dynamic_broadphase_grid(const impl_collision_policy &aPolicy)
: m_Policy(aPolicy)
{}

void impl_dynamic_broadphase_grid::next_generation() {
    ++m_Generation;

    if (m_Generation == 0) {
        grow_table(m_Slots.size(), m_Chunks.empty() ? 1 : m_Chunks.size());
        ++m_Generation;
    }
}

void impl_dynamic_broadphase_grid::clear() {
    m_Bodies.clear();
    m_Unbinned.clear();
    m_Entries.clear();

    for (auto &chunk : m_Chunks) {
        chunk.occupied.clear();
        chunk.entries.clear();
    }

    next_generation();
}

void impl_dynamic_broadphase_grid::reserve(const std::size_t aBodyCount) {
    m_Bodies.reserve(aBodyCount);
}

void impl_dynamic_broadphase_grid::add(const impl_collider_ptr_type &aBody,
    const impl_collider::broadphase_bounds &aBounds, const body_kind aKind) {
    body_record record;
    record.ptr = aBody;
    record.id = aBody->id();
    record.kind = aKind;
    record.min = cell_key{0, 0, 0};
    record.max = cell_key{0, 0, 0};
    record.unbinned = false;

    const bool finite = std::isfinite(aBounds.min.x) && std::isfinite(aBounds.min.y)
        && std::isfinite(aBounds.min.z) && std::isfinite(aBounds.max.x)
        && std::isfinite(aBounds.max.y) && std::isfinite(aBounds.max.z);

    if (finite) {
        record.min = cell_coord(aBounds.min);
        record.max = cell_coord(aBounds.max);

        const auto span = static_cast<std::uint64_t>(record.max.x - record.min.x + 1)
            * static_cast<std::uint64_t>(record.max.y - record.min.y + 1)
            * static_cast<std::uint64_t>(record.max.z - record.min.z + 1);

        record.unbinned = span > m_Policy.BROAD_PHASE_MAX_CELLS_PER_BODY;
    }
    else record.unbinned = true;

    if (record.unbinned) m_Unbinned.push_back(static_cast<std::uint32_t>(m_Bodies.size()));

    m_Bodies.push_back(std::move(record));
}

void impl_dynamic_broadphase_grid::grow_table(const std::size_t aCapacity, const std::size_t aChunkCount) {
    const auto perChunk = std::max<std::size_t>(next_power_of_two(aCapacity / aChunkCount), 64);

    m_SlotsPerChunk = perChunk;

    m_Slots.assign(perChunk * aChunkCount, slot{cell_key{0, 0, 0}, 0, 0, 0});
    m_Generation = 0;
}

std::size_t impl_dynamic_broadphase_grid::claim_slot(build_chunk &aChunk, const std::size_t aChunkIndex,
    const cell_key &aKey, const std::size_t aHash) {
    const auto mask = m_SlotsPerChunk - 1;
    const auto base = aChunkIndex * m_SlotsPerChunk;

    auto offset = (aHash / m_Chunks.size()) & mask;

    for (;;) {
        auto &candidate = m_Slots[base + offset];

        if (candidate.generation != m_Generation) {
            candidate.generation = m_Generation;
            candidate.key = aKey;
            candidate.count = 0;
            candidate.cursor = 0;
            aChunk.occupied.push_back(static_cast<std::uint32_t>(base + offset));
            return base + offset;
        }

        if (candidate.key == aKey) return base + offset;

        offset = (offset + 1) & mask;
    }
}

std::size_t impl_dynamic_broadphase_grid::find_slot(const cell_key &aKey) const {
    const auto hash = broadphase_cell_hasher{}(aKey);
    const auto mask = m_SlotsPerChunk - 1;
    const auto base = chunk_of(hash) * m_SlotsPerChunk;

    auto offset = (hash / m_Chunks.size()) & mask;

    for (;;) {
        const auto &candidate = m_Slots[base + offset];
        if (candidate.generation != m_Generation) return m_Slots.size();
        if (candidate.key == aKey) return base + offset;
        offset = (offset + 1) & mask;
    }
}

void impl_dynamic_broadphase_grid::count_cells(const std::size_t aChunkIndex) {
    auto &chunk = m_Chunks[aChunkIndex];

    chunk.occupied.clear();
    chunk.entries.clear();
    chunk.overflowed = false;

    const auto occupancyLimit = m_SlotsPerChunk / 2;

    const bool partitioned = m_Chunks.size() > 1;

    for (std::uint32_t body = 0; body < m_Bodies.size(); ++body) {
        const auto &record = m_Bodies[body];
        if (record.unbinned) continue;

        for (int z = record.min.z; z <= record.max.z; ++z)
        for (int y = record.min.y; y <= record.max.y; ++y)
        for (int x = record.min.x; x <= record.max.x; ++x) {
            const cell_key key{x, y, z};
            const auto hash = broadphase_cell_hasher{}(key);

            if (partitioned && chunk_of(hash) != aChunkIndex) continue;

            const auto slot = claim_slot(chunk, aChunkIndex, key, hash);
            ++m_Slots[slot].count;
            chunk.entries.emplace_back(static_cast<std::uint32_t>(slot), body);

            if (chunk.occupied.size() > occupancyLimit) {
                chunk.overflowed = true;
                return;
            }
        }
    }
}

void impl_dynamic_broadphase_grid::build(const task_dispatcher_type &aDispatcher) {
    m_Entries.clear();

    const std::size_t chunkCount = aDispatcher ? 8 : 1;
    if (m_Chunks.size() != chunkCount) {
        m_Chunks.assign(chunkCount, build_chunk{});
        m_Slots.clear();
    }

    if (m_Slots.empty())
        grow_table(next_power_of_two(std::max<std::size_t>(m_PreviousCellCount, 1) * 2), chunkCount);

    for (;;) {
        next_generation();

        if (aDispatcher) aDispatcher(chunkCount, [this](const std::size_t aChunk) { count_cells(aChunk); });
        else count_cells(0);

        bool overflowed = false;
        for (const auto &chunk : m_Chunks) if (chunk.overflowed) { overflowed = true; break; }
        if (!overflowed) break;

        grow_table(m_Slots.size() * 2, chunkCount);
    }

    std::uint32_t offset = 0;
    std::size_t occupiedTotal = 0;

    for (auto &chunk : m_Chunks) {
        chunk.base = offset;

        std::uint32_t local = 0;
        for (const auto slot : chunk.occupied) local += m_Slots[slot].count;

        offset += local;
        occupiedTotal += chunk.occupied.size();
    }

    m_PreviousCellCount = occupiedTotal;
    m_Entries.resize(offset);

    if (aDispatcher) aDispatcher(chunkCount, [this](const std::size_t aChunk) { scatter(aChunk); });
    else scatter(0);
}

void impl_dynamic_broadphase_grid::scatter(const std::size_t aChunkIndex) {
    auto &chunk = m_Chunks[aChunkIndex];

    auto running = chunk.base;
    for (const auto slot : chunk.occupied) {
        auto &occupied = m_Slots[slot];
        occupied.cursor = running;
        running += occupied.count;
    }

    for (const auto &entry : chunk.entries) m_Entries[m_Slots[entry.first].cursor++] = entry.second;
}

void impl_dynamic_broadphase_grid::gather_neighbours(const impl_collider &aCollider,
    const delta_time_type aDeltaTime, collider_id_set &aSeen,
    std::vector<neighbour> &aOutNeighbours) const {
    gather_overlapping_excluding(aCollider.broad_phase_swept_bounds(aDeltaTime), &aCollider,
        aSeen, aOutNeighbours);
}

void impl_dynamic_broadphase_grid::gather_overlapping_excluding(
    const impl_collider::broadphase_bounds &aBounds, const impl_collider *aExclude,
    collider_id_set &aSeen, std::vector<neighbour> &aOutNeighbours) const {
    const auto append = [&](const std::uint32_t aBody) {
        const auto &record = m_Bodies[aBody];
        if (record.ptr.get() == aExclude) return;
        if (!aSeen.insert(record.id)) return;

        aOutNeighbours.push_back(neighbour{record.ptr, record.kind, false});
    };

    for (const auto body : m_Unbinned) append(body);

    if (m_Slots.empty()) return;

    if (!std::isfinite(aBounds.min.x) || !std::isfinite(aBounds.min.y) || !std::isfinite(aBounds.min.z) ||
        !std::isfinite(aBounds.max.x) || !std::isfinite(aBounds.max.y) || !std::isfinite(aBounds.max.z))
        return;

    const auto min = cell_coord(aBounds.min);
    const auto max = cell_coord(aBounds.max);

    for (int z = min.z; z <= max.z; ++z)
    for (int y = min.y; y <= max.y; ++y)
    for (int x = min.x; x <= max.x; ++x) {
        const auto index = find_slot(cell_key{x, y, z});
        if (index == m_Slots.size()) continue;

        const auto &occupied = m_Slots[index];
        for (auto entry = occupied.cursor - occupied.count; entry < occupied.cursor; ++entry)
            append(m_Entries[entry]);
    }
}
