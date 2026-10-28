// © Joseph Cameron - MIT License

#include <gdk/collisions/exception.h>
#include <gdk/collisions/std_kernels.h>
#include <gdk/collisions/impl_collision_profile.h>
#include <gdk/collisions/impl_mover_grid.h>
#include <gdk/collisions/impl_narrow_phase.h>
#include <gdk/collisions/impl_shape.h>
#include <gdk/collisions/impl_static_grid.h>
#include <gdk/collisions/impl_std_queries.h>
#include <gdk/collisions/impl_std_tuning.h>

#include <jfc/dispatch.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

using namespace gdk::collisions;

namespace {
    [[nodiscard]] bool meets(const body_table &aFirst, const std::uint32_t aFirstRow, const body_table &aSecond,
        const std::uint32_t aSecondRow) {
        return (aFirst.on[aFirstRow] & aSecond.collides_with[aSecondRow]) != 0
            && (aSecond.on[aSecondRow] & aFirst.collides_with[aFirstRow]) != 0;
    }

    [[nodiscard]] bounds swept(const body_table &aTable, const std::uint32_t aRow, const delta_time_type aTime) {
        const auto &standing = aTable.extent[aRow];
        const auto sweep = aTable.velocity[aRow] * aTime;

        return bounds{vector3_type::min(standing.min, standing.min + sweep),
            vector3_type::max(standing.max, standing.max + sweep)};
    }

    [[nodiscard]] shape_kinematics kinematics_of(const body_table &aTable, const std::uint32_t aRow, const bool aMoves) {
        return shape_kinematics{aTable.position[aRow], aMoves ? aTable.velocity[aRow] : vector3_type::zero,
            aTable.rotation[aRow]};
    }

    void decode_parts(const body_table &aTable, const std::uint32_t aRow, std::vector<collider_part> &aOut) {
        aOut.clear();

        const auto first = aTable.first_part[aRow];

        for (std::uint32_t i = 0; i < aTable.part_count[aRow]; ++i) {
            const auto &part = aTable.parts[first + i];

            const impl_mesh_data *pMesh = nullptr;
            const impl_heightfield_data *pHeightfield = nullptr;

            if (part.surface != NO_SURFACE) {
                if (part.kind == shape_kind::mesh)
                    pMesh = static_cast<const impl_mesh_data *>(aTable.surfaces.meshes[part.surface].get());
                else if (part.kind == shape_kind::heightfield)
                    pHeightfield = static_cast<const impl_heightfield_data *>(
                        aTable.surfaces.heightfields[part.surface].get());
            }

            aOut.push_back(decoded(part, pMesh, pHeightfield));
        }
    }

    [[nodiscard]] const body_table &table_of(const scene_tables &aTables, const body_ref &aBody) {
        return aBody.fixed ? aTables.statics : aTables.movers;
    }

    void require_valid_std_policy(const std_kernels_policy &aPolicy) {
        const auto refuse = [](const char *aName, const char *aRule) {
            throw exception(std::string("gdk::collision: std_kernels_policy::").append(aName).append(" must be ")
                .append(aRule));
        };

        if (!std::isfinite(aPolicy.BROAD_PHASE_CELL_SIZE) || !(aPolicy.BROAD_PHASE_CELL_SIZE > 0))
            refuse("BROAD_PHASE_CELL_SIZE", "finite and positive");

        if (aPolicy.BROAD_PHASE_MAX_CELLS_PER_BODY < 1) refuse("BROAD_PHASE_MAX_CELLS_PER_BODY", "at least 1");
        if (aPolicy.PARALLEL_CHUNK_SIZE < 1) refuse("PARALLEL_CHUNK_SIZE", "at least 1");
        if (aPolicy.SWEEP_MAX_STEPS < 1) refuse("SWEEP_MAX_STEPS", "at least 1");

        for (const auto &[value, name] : {std::pair{aPolicy.BARYCENTRIC_EDGE_EPSILON, "BARYCENTRIC_EDGE_EPSILON"},
            std::pair{aPolicy.FACE_CONTACT_COSINE_EPSILON, "FACE_CONTACT_COSINE_EPSILON"},
            std::pair{aPolicy.INTERNAL_EDGE_GHOST_COSINE, "INTERNAL_EDGE_GHOST_COSINE"}})
            if (!std::isfinite(value) || value < 0) refuse(name, "finite and not negative");
    }

    class impl_std_kernels final : public scene_kernels {
    public:
        impl_std_kernels(const std_kernels_policy &aPolicy, task_dispatcher_type aDispatcher)
        : m_Policy(aPolicy)
        , m_Dispatcher(std::move(aDispatcher))
        , m_Statics(aPolicy)
        , m_Movers(aPolicy)
        {}

        virtual void statics_changed(const scene_tables &aTables, std::span<const std::uint32_t> aAdded,
            std::span<const std::uint32_t> aRemoved) override {
            for (const auto row : aRemoved) m_Statics.remove(row);
            for (const auto row : aAdded) m_Statics.insert(row, aTables.statics.extent[row]);
        }

        virtual void movers_changed(const scene_tables &aTables, const delta_time_type aDeltaTime) override {
            GDK_COLLISION_PROFILE_BEGIN(broadphase_rebuild_ns);

            const auto &movers = aTables.movers;
            const auto count = static_cast<std::uint32_t>(movers.size());

            m_Movers.clear();
            m_Movers.reserve(count);

            for (std::uint32_t row = 0; row < count; ++row)
                m_Movers.add(row, swept(movers, row, aDeltaTime), movers.extent[row]);

            m_Movers.build(dispatcher_for(count));

            GDK_COLLISION_PROFILE_END(broadphase_rebuild_ns);
        }

        virtual void gather_pairs(const scene_tables &aTables, const pair_request &aRequest,
            std::vector<candidate_pair> &aOut) override {
            GDK_COLLISION_PROFILE_BEGIN(gather_query_ns);

            const auto &movers = aTables.movers;
            const auto &statics = aTables.statics;

            in_chunks(aRequest.subjects.size(), [&](const std::size_t aBegin, const std::size_t aEnd, scratch &aScratch) {
                aScratch.pairs.clear();

                for (auto k = aBegin; k < aEnd; ++k) {
                    const auto subject = aRequest.subjects[k];
                    const auto query = swept(movers, subject, aRequest.times[k]);

                    auto &others = aScratch.others;
                    others.clear();
                    aScratch.statics_seen.clear();
                    aScratch.movers_seen.clear();
                    aScratch.rows.clear();

                    m_Statics.visit_overlapping(query, aScratch.statics_seen, [&](const std::uint32_t aRow) {
                        if (statics.live[aRow] && meets(movers, subject, statics, aRow)) others.push_back({aRow, true});

                        return false;
                    });

                    m_Movers.gather(query, subject, aScratch.movers_seen, aScratch.rows);

                    for (const auto row : aScratch.rows) {
                        if (!aRequest.kinematic_candidates && movers.motion[row] == body_motion::kinematic) continue;
                        if (meets(movers, subject, movers, row)) others.push_back({row, false});
                    }

                    std::sort(others.begin(), others.end(), [&](const body_ref &aFirst, const body_ref &aSecond) {
                        const auto &first = table_of(aTables, aFirst);
                        const auto &second = table_of(aTables, aSecond);

                        if (first.trigger[aFirst.row] != second.trigger[aSecond.row])
                            return first.trigger[aFirst.row] < second.trigger[aSecond.row];

                        if (aFirst.fixed != aSecond.fixed) return aFirst.fixed;

                        return first.id[aFirst.row] < second.id[aSecond.row];
                    });

                    for (const auto &other : others) aScratch.pairs.push_back({subject, other});
                }
            });

            for (std::size_t chunk = 0; chunk < m_Chunks; ++chunk)
                aOut.insert(aOut.end(), m_Scratch[chunk].pairs.begin(), m_Scratch[chunk].pairs.end());

            GDK_COLLISION_PROFILE_END(gather_query_ns);
        }

        virtual void test_pairs(const scene_tables &aTables, std::span<const pair_test> aTests,
            std::vector<pair_hit> &aOut) override {
            GDK_COLLISION_PROFILE_BEGIN(narrow_phase_ns);

            const auto tuning = tuning_of(m_Policy, aTables.rules);
            const auto &movers = aTables.movers;

            in_chunks(aTests.size(), [&](const std::size_t aBegin, const std::size_t aEnd, scratch &aScratch) {
                aScratch.hits.clear();

                auto decodedSubject = ~std::uint32_t{0};

                for (auto i = aBegin; i < aEnd; ++i) {
                    const auto &test = aTests[i];

                    if (test.subject != decodedSubject) {
                        decode_parts(movers, test.subject, aScratch.subject_parts);
                        decodedSubject = test.subject;
                    }

                    const auto &other = table_of(aTables, test.other);

                    decode_parts(other, test.other.row, aScratch.other_parts);

                    if (const auto hit = narrow_phase_overlap_parts(aScratch.subject_parts,
                        kinematics_of(movers, test.subject, true), aScratch.other_parts,
                        kinematics_of(other, test.other.row, test.other_moves), test.time, tuning))
                        aScratch.hits.push_back({static_cast<std::uint32_t>(i), *hit});
                }
            });

            for (std::size_t chunk = 0; chunk < m_Chunks; ++chunk)
                aOut.insert(aOut.end(), m_Scratch[chunk].hits.begin(), m_Scratch[chunk].hits.end());

            GDK_COLLISION_PROFILE_END(narrow_phase_ns);
        }

        [[nodiscard]] virtual std::optional<ray_answer> cast_ray(const scene_tables &aTables,
            const ray_query &aQuery) const override {
            std::optional<ray_answer> nearest;

            thread_local std::vector<collider_part> parts;
            thread_local impl_row_set seen;

            const auto consider = [&](const body_table &aTable, const std::uint32_t aRow, const bool aFixed) {
                if (!aTable.live[aRow] || aTable.trigger[aRow]) return;
                if ((aTable.on[aRow] & aQuery.layers) == 0) return;
                if (std::find(aQuery.ignore.begin(), aQuery.ignore.end(), aTable.id[aRow]) != aQuery.ignore.end()) return;

                decode_parts(aTable, aRow, parts);

                const auto result = ray_against_parts(aQuery.origin, aQuery.direction,
                    nearest ? nearest->distance : aQuery.max_distance, parts, kinematics_of(aTable, aRow, true));

                if (!result || (nearest && result->entry_time >= nearest->distance)) return;

                nearest = ray_answer{{aRow, aFixed}, result->entry_time, result->contact_point, result->contact_normal};
            };

            seen.clear();

            m_Statics.visit_along_ray(aQuery.origin, aQuery.direction, aQuery.max_distance, seen,
                [&](const std::uint32_t aRow) {
                    consider(aTables.statics, aRow, true);

                    return nearest ? nearest->distance : aQuery.max_distance;
                });

            if (!aQuery.statics_only)
                for (std::uint32_t row = 0; row < aTables.movers.size(); ++row) consider(aTables.movers, row, false);

            return nearest;
        }

        [[nodiscard]] virtual bool meets_statics(const scene_tables &aTables, const upright_cylinder &aVolume,
            std::span<const collider_id_type> aIgnore) const override {
            if (aVolume.radius <= 0 || aVolume.height <= 0) return false;

            const bounds reach{{aVolume.base.x - aVolume.radius, aVolume.base.y, aVolume.base.z - aVolume.radius},
                {aVolume.base.x + aVolume.radius, aVolume.base.y + aVolume.height, aVolume.base.z + aVolume.radius}};

            return any_static(aTables, reach, aIgnore,
                [&](const std::vector<collider_part> &aParts, const shape_kinematics &aBody) {
                    return cylinder_meets_parts(aVolume, aParts, aBody);
                });
        }

        [[nodiscard]] virtual bool meets_statics(const scene_tables &aTables, const parallelepiped &aVolume,
            std::span<const collider_id_type> aIgnore) const override {
            const bounds reach{parallelepiped_low(aVolume), parallelepiped_high(aVolume)};

            return any_static(aTables, reach, aIgnore,
                [&](const std::vector<collider_part> &aParts, const shape_kinematics &aBody) {
                    return parallelepiped_meets_parts(aVolume, aParts, aBody);
                });
        }

    private:
        struct scratch final {
            impl_row_set statics_seen;
            impl_row_set movers_seen;
            std::vector<std::uint32_t> rows;
            std::vector<body_ref> others;
            std::vector<candidate_pair> pairs;
            std::vector<pair_hit> hits;
            std::vector<collider_part> subject_parts;
            std::vector<collider_part> other_parts;
        };

        [[nodiscard]] const task_dispatcher_type &dispatcher_for(const std::size_t aWork) const {
            static const task_dispatcher_type SERIAL{};

            return m_Dispatcher && aWork >= m_Policy.PARALLEL_MIN_WORK ? m_Dispatcher : SERIAL;
        }

        template <typename work_type>
        void in_chunks(const std::size_t aCount, work_type &&aWork) {
            const auto &dispatcher = dispatcher_for(aCount);
            const auto size = m_Policy.PARALLEL_CHUNK_SIZE;

            m_Chunks = dispatcher ? std::max<std::size_t>((aCount + size - 1) / size, 1) : 1;

            if (m_Scratch.size() < m_Chunks) m_Scratch.resize(m_Chunks);

            jfc::dispatch::run(dispatcher, m_Chunks, [&](const std::size_t aChunk) {
                const auto begin = dispatcher ? aChunk * size : 0;
                const auto end = dispatcher ? std::min(begin + size, aCount) : aCount;

                aWork(begin, end, m_Scratch[aChunk]);
            });
        }

        template <typename meets_type>
        [[nodiscard]] bool any_static(const scene_tables &aTables, const bounds &aReach,
            std::span<const collider_id_type> aIgnore, const meets_type &aMeets) const {
            thread_local std::vector<collider_part> parts;
            thread_local impl_row_set seen;

            seen.clear();

            const auto &statics = aTables.statics;
            auto found = false;

            m_Statics.visit_overlapping(aReach, seen, [&](const std::uint32_t aRow) {
                if (!statics.live[aRow] || statics.trigger[aRow]) return false;
                if (std::find(aIgnore.begin(), aIgnore.end(), statics.id[aRow]) != aIgnore.end()) return false;

                decode_parts(statics, aRow, parts);

                found = aMeets(parts, kinematics_of(statics, aRow, true));

                return found;
            });

            return found;
        }

        std_kernels_policy m_Policy;
        task_dispatcher_type m_Dispatcher;
        impl_static_grid m_Statics;
        impl_mover_grid m_Movers;
        std::vector<scratch> m_Scratch;
        std::size_t m_Chunks = 1;
    };
}

scene_kernels_ptr_type gdk::collisions::make_std_kernels(std_kernels_policy aPolicy, task_dispatcher_type aDispatcher) {
    require_valid_std_policy(aPolicy);

    return std::make_shared<impl_std_kernels>(aPolicy, std::move(aDispatcher));
}
