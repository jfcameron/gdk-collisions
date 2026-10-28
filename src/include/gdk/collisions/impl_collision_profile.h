// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_COLLISION_PROFILE_H
#define GDK_COLLISIONS_IMPL_COLLISION_PROFILE_H

/// \file
/// \brief profiling timers for each phase in the collision pipeline

#ifdef GDK_COLLISION_PROFILE

#include <atomic>
#include <chrono>
#include <cstdint>

namespace gdk::collisions {
    namespace profile {
        using counter_type = std::atomic<std::uint64_t>;

        /// \brief nanoseconds and counts accumulated in each phase since the last reset.
        /// \note atomic, because parallel detect chunks add to it from every worker
        struct timings final {
            counter_type broadphase_rebuild_ns{0};
            counter_type kinematic_pass_ns{0};
            counter_type dynamic_loop_ns{0};
            counter_type trigger_loop_ns{0};
            counter_type events_ns{0};
            counter_type total_ns{0};

            counter_type gather_query_ns{0};
            counter_type gather_sort_ns{0};
            counter_type narrow_phase_ns{0};

            counter_type resolve_respond_ns{0};

            counter_type bvh_query_ns{0};
            counter_type triangle_test_ns{0};

            counter_type mesh_queries{0};
            counter_type mesh_candidates{0};
            counter_type triangle_tests{0};
            counter_type advancement_iterations{0};

            counter_type steps{0};
        };

        [[nodiscard]] inline timings &accumulated() {
            static timings value;
            return value;
        }

        inline void reset() {
            auto &t = accumulated();

            for (auto *pCounter : {&t.broadphase_rebuild_ns, &t.kinematic_pass_ns, &t.dynamic_loop_ns, &t.trigger_loop_ns,
                &t.events_ns, &t.total_ns, &t.gather_query_ns, &t.gather_sort_ns, &t.narrow_phase_ns, &t.resolve_respond_ns,
                &t.bvh_query_ns, &t.triangle_test_ns, &t.mesh_queries, &t.mesh_candidates, &t.triangle_tests,
                &t.advancement_iterations, &t.steps})
                pCounter->store(0, std::memory_order_relaxed);
        }

        [[nodiscard]] inline double milliseconds(const counter_type &aNanoseconds) {
            return static_cast<double>(aNanoseconds.load(std::memory_order_relaxed)) / 1e6;
        }
    }
}

#define GDK_COLLISION_PROFILE_BEGIN(FIELD) \
    const auto gdkCollisionProfileStart_##FIELD = ::std::chrono::steady_clock::now()

#define GDK_COLLISION_PROFILE_END(FIELD) \
    ::gdk::collisions::profile::accumulated().FIELD.fetch_add(static_cast<::std::uint64_t>( \
        ::std::chrono::duration_cast<::std::chrono::nanoseconds>( \
        ::std::chrono::steady_clock::now() - gdkCollisionProfileStart_##FIELD).count()), ::std::memory_order_relaxed)

#define GDK_COLLISION_PROFILE_COUNT_STEP() \
    ::gdk::collisions::profile::accumulated().steps.fetch_add(1, ::std::memory_order_relaxed)

#define GDK_COLLISION_PROFILE_COUNT(FIELD, AMOUNT) \
    ::gdk::collisions::profile::accumulated().FIELD.fetch_add(static_cast<::std::uint64_t>(AMOUNT), \
        ::std::memory_order_relaxed)

#else

#define GDK_COLLISION_PROFILE_BEGIN(FIELD) ((void)0)
#define GDK_COLLISION_PROFILE_END(FIELD) ((void)0)
#define GDK_COLLISION_PROFILE_COUNT_STEP() ((void)0)
#define GDK_COLLISION_PROFILE_COUNT(FIELD, AMOUNT) ((void)0)

#endif

#endif
