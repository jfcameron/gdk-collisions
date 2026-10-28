// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_COLLIDER_ID_POOL_H
#define GDK_COLLISIONS_IMPL_COLLIDER_ID_POOL_H

#include <gdk/collisions/impl_collision_types.h>

#include <cstddef>
#include <mutex>
#include <vector>

namespace gdk::collisions {
    /// \brief hands out collider ids and takes them back, reusing an id only after an update has forgotten it.
    /// \note give_back is safe from any thread; everything else belongs to the scene's thread.
    class impl_collider_id_pool final {
    public:
        [[nodiscard]] collider_id_type take() {
            std::lock_guard lock(m_Mutex);

            if (m_Free.empty()) return m_Next++;

            const auto id = m_Free.back();
            m_Free.pop_back();
            return id;
        }

        void give_back(const collider_id_type aId) noexcept {
            std::lock_guard lock(m_Mutex);

            try { m_Released.push_back(aId); }
            catch (...) {}
        }

        /// \brief the ids released since the last call, now set aside until an update has passed
        [[nodiscard]] std::vector<collider_id_type> collect() {
            std::lock_guard lock(m_Mutex);

            auto released = std::move(m_Released);
            m_Released.clear();

            m_Collected.insert(m_Collected.end(), released.begin(), released.end());

            return released;
        }

        /// \brief the ids released and not yet collected
        [[nodiscard]] std::vector<collider_id_type> released() const {
            std::lock_guard lock(m_Mutex);

            return m_Released;
        }

        /// \brief the ids collected so far become reusable once this update has finished
        void begin_update() {
            std::lock_guard lock(m_Mutex);

            m_Retiring.insert(m_Retiring.end(), m_Collected.begin(), m_Collected.end());
            m_Collected.clear();
        }

        void end_update() {
            std::lock_guard lock(m_Mutex);

            m_Free.insert(m_Free.end(), m_Retiring.begin(), m_Retiring.end());
            m_Retiring.clear();
        }

    private:
        mutable std::mutex m_Mutex;

        collider_id_type m_Next = 0;

        std::vector<collider_id_type> m_Free;
        std::vector<collider_id_type> m_Released;
        std::vector<collider_id_type> m_Collected;
        std::vector<collider_id_type> m_Retiring;
    };
}

#endif
