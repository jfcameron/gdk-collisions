// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_RESOLVED_PAIR_SET_H
#define GDK_COLLISIONS_IMPL_RESOLVED_PAIR_SET_H

#include <gdk/collisions/impl_collision_types.h>

#include <jfc/hash.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gdk::collisions {
    /// \brief the unordered collider pairs already resolved this iteration; clearing is constant time
    class impl_resolved_pair_set final {
    public:
        void clear() {
            ++m_Generation;

            if (m_Generation == 0) {
                std::fill(m_Stamps.begin(), m_Stamps.end(), std::uint32_t{0});
                m_Generation = 1;
            }

            m_Count = 0;
        }

        /// \return true if the pair was not already present
        bool insert(const collider_id_type aFirst, const collider_id_type aSecond) {
            if (m_Stamps.empty()) grow(64);

            const auto low = std::min(aFirst, aSecond);
            const auto high = std::max(aFirst, aSecond);
            const key_type key{low, high};

            if ((m_Count + 1) * 2 > m_Stamps.size()) grow(m_Stamps.size() * 2);

            return insert_key(key);
        }

    private:
        struct key_type final {
            collider_id_type low;
            collider_id_type high;

            bool operator==(const key_type &) const = default;
        };

        [[nodiscard]] static std::size_t hash_of(const key_type &aKey) {
            return static_cast<std::size_t>(jfc::hash::mix64(aKey.low * 0x9e3779b97f4a7c15ull ^ aKey.high));
        }

        bool insert_key(const key_type &aKey) {
            const auto mask = m_Stamps.size() - 1;
            auto index = hash_of(aKey) & mask;

            for (;;) {
                if (m_Stamps[index] != m_Generation) {
                    m_Stamps[index] = m_Generation;
                    m_Keys[index] = aKey;
                    ++m_Count;
                    return true;
                }

                if (m_Keys[index] == aKey) return false;

                index = (index + 1) & mask;
            }
        }

        void grow(const std::size_t aCapacity) {
            std::vector<key_type> live;
            live.reserve(m_Count);

            for (std::size_t i = 0; i < m_Stamps.size(); ++i)
                if (m_Stamps[i] == m_Generation) live.push_back(m_Keys[i]);

            m_Keys.assign(aCapacity, key_type{0, 0});
            m_Stamps.assign(aCapacity, 0);
            m_Generation = 1;
            m_Count = 0;

            for (const auto &key : live) insert_key(key);
        }

        std::vector<key_type> m_Keys;
        std::vector<std::uint32_t> m_Stamps;
        std::uint32_t m_Generation = 1;
        std::size_t m_Count = 0;
    };
}

#endif
