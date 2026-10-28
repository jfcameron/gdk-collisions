// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_MOVER_GRID_H
#define GDK_COLLISIONS_IMPL_MOVER_GRID_H

#include <gdk/collisions/kernels/tables.h>
#include <gdk/collisions/std_kernels.h>
#include <gdk/collisions/impl_static_grid.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <unordered_set>
#include <vector>

namespace gdk::collisions {
    /// \brief a uniform spatial hash over the rows of the movers table, rebuilt every step
    class impl_mover_grid final {
    public:
        explicit impl_mover_grid(const std_kernels_policy &aPolicy);

        /// \brief drop every body, retaining all allocated capacity
        void clear();

        /// \brief hint the number of bodies about to be added
        void reserve(const std::size_t aBodyCount);

        /// \brief record a row's swept bounds; aStanding is where it starts. Nothing is binned until build().
        void add(const std::uint32_t aRow, const bounds &aSwept, const bounds &aStanding);

        /// \brief bin every added body. Call once, after the last add() and before any query.
        /// \param aDispatcher optional reference to the worker thread interface
        void build(const task_dispatcher_type &aDispatcher = {});

        /// \brief append, once each, the rows whose reach meets aBounds, all but aExclude
        void gather(const bounds &aBounds, const std::uint32_t aExclude, impl_row_set &aSeen,
            std::vector<std::uint32_t> &aOut) const;

        /// \brief get how many distinct cells the last build occupied
        [[nodiscard]] std::size_t occupied_cell_count() const {
            std::size_t total = 0;
            for (const auto &chunk : m_Chunks) total += chunk.occupied.size();
            return total;
        }

    private:
        using cell_key = broadphase_cell_key;

        struct body_record final {
            std::uint32_t row;

            cell_key min;
            cell_key max;

            bounds reach;

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
            return broadphase_cell_coord(m_CellSize, aValue);
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

        floating_point_type m_CellSize;
        std::size_t m_MaxCellsPerBody;
    };
}

#endif
