// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_DYNAMIC_BROADPHASE_GRID_H
#define GDK_COLLISIONS_IMPL_DYNAMIC_BROADPHASE_GRID_H

#include <gdk/collisions/impl_broadphase_grid.h>
#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_collision_types.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <unordered_set>
#include <vector>

namespace gdk::collisions {
    /// \brief broadphase structure for dynamics. similar to impl_broadphase_grid but this one is expected to destroy and rebuild its data much more frequently
    class impl_dynamic_broadphase_grid final {
    public:
        using body_kind = impl_broadphase_grid::body_kind;
        using neighbour = impl_broadphase_grid::neighbour;

        explicit impl_dynamic_broadphase_grid(const impl_collision_policy &aPolicy);

        /// \brief drop every body, retaining all allocated capacity
        void clear();

        /// \brief hint the number of bodies about to be added
        void reserve(const std::size_t aBodyCount);

        /// \brief record a body and the cells its bounds cover. Nothing is binned until build().
        void add(const impl_collider_ptr_type &aBody, const impl_collider::broadphase_bounds &aBounds,
            const body_kind aKind);

        /// \brief bin every added body. Call once, after the last add() and before any query.
        /// \param aDispatcher optional reference to the worker thread interface
        void build(const task_dispatcher_type &aDispatcher = {});

        /// \brief append the unique bodies sharing a cell with aCollider's swept bounds.
        void gather_neighbours(const impl_collider &aCollider, const delta_time_type aDeltaTime,
            collider_id_set &aSeen, std::vector<neighbour> &aOutNeighbours) const;

        void gather_overlapping_excluding(const impl_collider::broadphase_bounds &aBounds,
            const impl_collider *aExclude, collider_id_set &aSeen,
            std::vector<neighbour> &aOutNeighbours) const;

        /// \brief get how many distinct cells the last build occupied
        [[nodiscard]] std::size_t occupied_cell_count() const {
            std::size_t total = 0;
            for (const auto &chunk : m_Chunks) total += chunk.occupied.size();
            return total;
        }

    private:
        using cell_key = broadphase_cell_key;

        struct body_record final {
            impl_collider_ptr_type ptr;
            collider_id_type id;
            body_kind kind;

            cell_key min;
            cell_key max;

            /// too large to bin: a neighbour of every query instead. \see impl_collision_policy
            bool unbinned;
        };

        /// \brief one cell of the open-addressed table.
        struct slot final {
            cell_key key;
            std::uint32_t generation;
            std::uint32_t count;
            std::uint32_t cursor;
        };

        [[nodiscard]] cell_key cell_coord(const vector3_type &aValue) const {
            return broadphase_cell_coord(m_Policy, aValue);
        }

        /// \brief everything one chunk of the build owns. Slices are disjoint, so chunks share
        /// nothing writable and need no synchronisation with each other.
        struct build_chunk final {
            /// \brief slots this chunk touched 
            std::vector<std::uint32_t> occupied;

            /// \brief (slot, body) for every cell insert this chunk handled
            std::vector<std::pair<std::uint32_t, std::uint32_t>> entries;

            /// \brief where this chunk's run of m_Entries begins
            std::uint32_t base = 0;

            /// \brief set when this chunk's slice filled past half
            bool overflowed = false;
        };

        /// \brief which chunk's slice a hash belongs to, and where in that slice it starts.
        [[nodiscard]] std::size_t chunk_of(const std::size_t aHash) const {
            return aHash & (m_Chunks.size() - 1);
        }

        /// \brief index of aKey's slot within aChunk's slice
        [[nodiscard]] std::size_t claim_slot(build_chunk &aChunk, const std::size_t aChunkIndex,
            const cell_key &aKey, const std::size_t aHash);

        /// \brief index of aKey's slot or m_Slots.size() if the key is not present
        [[nodiscard]] std::size_t find_slot(const cell_key &aKey) const;

        /// \brief get the number of cells for one chunk's slice
        void count_cells(const std::size_t aChunkIndex);

        void grow_table(const std::size_t aCapacity, const std::size_t aChunkCount);

        /// \brief prefix sum and scatter for one chunk
        void scatter(const std::size_t aChunkIndex);

        /// \brief invalidate every slot in the table
        void next_generation();

        std::vector<body_record> m_Bodies;

        std::vector<std::uint32_t> m_Unbinned;

        std::vector<slot> m_Slots;

        std::vector<build_chunk> m_Chunks;

        std::size_t m_SlotsPerChunk = 0;

        std::vector<std::uint32_t> m_Entries;

        std::uint32_t m_Generation = 0;

        std::size_t m_PreviousCellCount = 0;

        impl_collision_policy m_Policy;
    };
}

#endif
