// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_IMPL_COLLISION_PROFILE_H
#define GDK_COLLISIONS_IMPL_COLLISION_PROFILE_H

/// \file
/// \brief profiling timers for each phase in the collision pipeline

#ifdef GDK_COLLISION_PROFILE

#include <chrono>

namespace gdk::collisions {
    namespace profile {
        /// \brief milliseconds accumulated in each phase since the last reset.
        struct timings final {
            double broadphase_rebuild_ms = 0;
            double kinematic_pass_ms = 0;
            double dynamic_loop_ms = 0;
            double trigger_loop_ms = 0;
            double events_ms = 0;
            double total_ms = 0;

            double gather_query_ms = 0;
            double gather_sort_ms = 0;
            double narrow_phase_ms = 0;

            double resolve_respond_ms = 0;

            double bvh_query_ms = 0;
            double triangle_test_ms = 0;

            unsigned long long mesh_queries = 0;
            unsigned long long mesh_candidates = 0;
            unsigned long long triangle_tests = 0;
            unsigned long long advancement_iterations = 0;

            unsigned long long steps = 0;
        };

        [[nodiscard]] inline timings &accumulated() {
            static timings value;
            return value;
        }

        inline void reset() { accumulated() = timings{}; }
    }
}

#define GDK_COLLISION_PROFILE_BEGIN(FIELD) \
    const auto gdkCollisionProfileStart_##FIELD = ::std::chrono::steady_clock::now()

#define GDK_COLLISION_PROFILE_END(FIELD) \
    ::gdk::collisions::profile::accumulated().FIELD += ::std::chrono::duration<double, ::std::milli>( \
        ::std::chrono::steady_clock::now() - gdkCollisionProfileStart_##FIELD).count()

#define GDK_COLLISION_PROFILE_COUNT_STEP() ++::gdk::collisions::profile::accumulated().steps

#define GDK_COLLISION_PROFILE_COUNT(FIELD, AMOUNT) \
    ::gdk::collisions::profile::accumulated().FIELD += (AMOUNT)

#else

#define GDK_COLLISION_PROFILE_BEGIN(FIELD) ((void)0)
#define GDK_COLLISION_PROFILE_END(FIELD) ((void)0)
#define GDK_COLLISION_PROFILE_COUNT_STEP() ((void)0)
#define GDK_COLLISION_PROFILE_COUNT(FIELD, AMOUNT) ((void)0)

#endif

#endif
