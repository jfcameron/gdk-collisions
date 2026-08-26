// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_BROADPHASE_GRID_H
#define GDK_COLLISIONS_IMPL_BROADPHASE_GRID_H

#include <gdk/collisions/impl_collider.h>
#include <gdk/collisions/impl_collision_policy.h>
#include <gdk/collisions/impl_collision_types.h>

#include <cstddef>
#include <unordered_map>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <vector>

namespace gdk::collisions {
    /// \brief integer coordinates of one broadphase cell.
    struct broadphase_cell_key final {
        int x;
        int y;
        int z;

        bool operator==(const broadphase_cell_key &aOther) const noexcept {
            return x == aOther.x && y == aOther.y && z == aOther.z;
        }

    };

    struct broadphase_cell_hasher final {
        std::size_t operator()(const broadphase_cell_key &aKey) const noexcept;
    };

    /// \brief the cell a world-space point falls in
    [[nodiscard]] broadphase_cell_key broadphase_cell_coord(const impl_collision_policy &aPolicy,
        const vector3_type &aValue);

    /// \brief a Z-order (Morton) code for a point's cell for sorting & spatial order.
    [[nodiscard]] std::uint64_t broadphase_morton_code(const impl_collision_policy &aPolicy,
        const vector3_type &aValue);

    /// \brief the set of colliders a single query has already reported.
    class collider_id_set final {
    public:
        void clear() {
            ++m_Generation;

            if (m_Generation == 0) {
                std::fill(m_Stamps.begin(), m_Stamps.end(), std::uint32_t{0});
                m_Generation = 1;
            }
        }

        /// \brief record aId. \return true if it was not already present
        bool insert(const collider_id_type aId) {
            if (aId >= m_Stamps.size()) m_Stamps.resize(aId + 1, 0);

            auto &stamp = m_Stamps[static_cast<std::size_t>(aId)];
            if (stamp == m_Generation) return false;

            stamp = m_Generation;
            return true;
        }

    private:
        std::vector<std::uint32_t> m_Stamps;
        std::uint32_t m_Generation = 1;
    };

    /// \brief uniform spatial hash used to reject collider pairs before narrow phase.
    class impl_broadphase_grid final {
    public:
        enum class body_kind {
            collider,
            trigger,
        };

        /// \brief a live collider returned by a query.
        struct neighbour final {
            impl_collider_ptr_type ptr;
            body_kind kind;
            bool is_static;
        };

        static void sort(std::vector<neighbour> &aNeighbours);

        explicit impl_broadphase_grid(const impl_collision_policy &aPolicy);

        void clear();

        /// \brief preallocate to a specific known size in order to avoid resizing
        void reserve(const std::size_t aBodyCount);

        /// \brief register a body in every cell its bounds touch.
        void insert(const impl_collider_ptr_type &aBody, const impl_collider::broadphase_bounds &aBounds,
            const body_kind aKind, const bool aIsStatic);

        /// \brief register a body of unbounded extent
        void insert_unbounded(const impl_collider_ptr_type &aBody, const body_kind aKind, const bool aIsStatic);

        /// \brief get the other live colliders sharing a cell with aCollider's swept bounds
        void gather_neighbours(const impl_collider &aCollider, const delta_time_type aDeltaTime,
            collider_id_set &aSeen, std::vector<neighbour> &aOutNeighbours) const;

        /// \brief get the live colliders sharing a cell with eg a raycast
        void gather_overlapping(const impl_collider::broadphase_bounds &aBounds,
            collider_id_set &aSeen, std::vector<neighbour> &aOutNeighbours) const;

        /// \brief gather_overlapping but excluding one collider
        void gather_overlapping_excluding(const impl_collider::broadphase_bounds &aBounds,
            const impl_collider *aExclude, collider_id_set &aSeen,
            std::vector<neighbour> &aOutNeighbours) const;

        /// \brief drop the entries that queries have found expired since the last call
        void compact_expired();

    private:
        using cell_key = broadphase_cell_key;
        using cell_hasher = broadphase_cell_hasher;

        [[nodiscard]] cell_key cell_coord(const vector3_type &aValue) const {
            return broadphase_cell_coord(m_Policy, aValue);
        }

        struct entry final {
            impl_collider_weak_ptr_type ptr;
            collider_id_type id;
            body_kind kind;
            bool is_static;
        };

        void take_live(const std::vector<entry> &aEntries, const impl_collider *aExclude,
            collider_id_set &aSeen, std::vector<neighbour> &aOut) const;

        std::unordered_map<cell_key, std::vector<entry>, cell_hasher> m_Cells;

        std::vector<entry> m_Unbinned;

        std::size_t m_PreviousCellCount = 0;

        /// \brief set by a query that skipped a dead entry
        ///
        /// Atomic because concurrent queries may set it and mutable because they are const. 
        mutable std::atomic<bool> m_HasExpiredEntries{false};

        impl_collision_policy m_Policy;
    };
}

#endif
