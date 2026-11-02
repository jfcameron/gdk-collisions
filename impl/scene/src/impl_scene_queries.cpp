// © Joseph Cameron - MIT License

#include <gdk/collisions/impl_scene.h>

#include <utility>

using namespace gdk::collisions;

namespace {
    [[nodiscard]] std::vector<contact> collect_contacts(const impl_collision_event_tracker &aEvents,
        const collider *aOnly) {
        std::vector<contact> result;

        const auto &recorded = aEvents.last_step_collisions();

        const auto *const pOnly = aOnly ? dynamic_cast<const impl_collider *>(aOnly) : nullptr;

        if (!aOnly) result.reserve(recorded.size());

        for (const auto &entry : recorded) {
            if (pOnly && entry.key.low != pOnly->id() && entry.key.high != pOnly->id()) continue;

            auto a = entry.a.lock();
            auto b = entry.b.lock();
            if (!a || !b) continue;

            if (aOnly && a.get() != aOnly && b.get() != aOnly) continue;

            contact reported;
            reported.a = std::move(a);
            reported.b = std::move(b);
            reported.normal = entry.normal;
            reported.point = entry.point;
            reported.penetration = entry.penetration;

            result.push_back(std::move(reported));
        }

        return result;
    }
}

std::vector<contact> impl_scene::do_contacts() const {
    return collect_contacts(m_Events, nullptr);
}

std::vector<contact> impl_scene::do_contacts_for(const collider &aCollider) const {
    return collect_contacts(m_Events, &aCollider);
}

impl_scene::query_movers impl_scene::movers_for_query(const bool aStaticOnly) const {
    query_movers movers;

    if (!aStaticOnly)
        for (const auto &weak : m_DynamicColliders)
            if (auto pBody = weak.lock()) {
                append_row(movers.table, *pBody,
                    pBody->is_kinematic() ? body_motion::kinematic : body_motion::dynamic, false);
                movers.owners.push_back(std::move(pBody));
            }

    return movers;
}

std::vector<collider_id_type> impl_scene::ignored_by_query(const collider *const pIgnore) const {
    auto ignore = m_Ids->released();

    if (const auto *const pSkipped = dynamic_cast<const impl_collider *>(pIgnore)) ignore.push_back(pSkipped->id());

    return ignore;
}

const_collider_ptr_type impl_scene::collider_answering(const body_ref &aBody, const query_movers &aMovers) const {
    if (aBody.fixed) return m_StaticOwners[aBody.row].lock();

    return aMovers.owners[aBody.row];
}

std::optional<raycast_hit> impl_scene::do_raycast(const vector3_type &aOrigin, const vector3_type &aDirection,
    const floating_point_type aMaxDistance, const collider *const pIgnore, const bool aStaticOnly,
    const collision_layers_type aLayers) const {
    const auto direction = aDirection.normal();
    if (direction.is_effectively_zero() || aMaxDistance <= 0.0f) return std::nullopt;

    const auto movers = movers_for_query(aStaticOnly);
    const auto ignore = ignored_by_query(pIgnore);

    ray_query query;
    query.origin = aOrigin;
    query.direction = direction;
    query.max_distance = aMaxDistance;
    query.layers = aLayers;
    query.ignore = ignore;
    query.statics_only = aStaticOnly;

    const auto answer = m_Kernels->cast_ray(scene_tables{m_Statics, movers.table, m_Rules}, query);

    if (!answer) return std::nullopt;

    raycast_hit hit;
    hit.collider = collider_answering(answer->body, movers);
    hit.distance = answer->distance;
    hit.point = answer->point;
    hit.normal = answer->normal;

    if (!hit.collider) return std::nullopt;

    return hit;
}

std::optional<sweep_hit> impl_scene::do_sweep(const capsule &aShape, const vector3_type &aDirection,
    const floating_point_type aMaxDistance, const collider *const pIgnore, const bool aStaticOnly,
    const collision_layers_type aLayers) const {
    const auto direction = aDirection.normal();
    if (direction.is_effectively_zero() || aMaxDistance <= 0.0f) return std::nullopt;
    if (aShape.radius <= 0.0f || aShape.half_height < 0.0f) return std::nullopt;

    const auto movers = movers_for_query(aStaticOnly);
    const auto ignore = ignored_by_query(pIgnore);

    auto shape = aShape;
    shape.rotation = aShape.rotation.normalized();

    sweep_query query;
    query.direction = direction;
    query.max_distance = aMaxDistance;
    query.layers = aLayers;
    query.ignore = ignore;
    query.statics_only = aStaticOnly;

    const auto answer = m_Kernels->cast_shape(scene_tables{m_Statics, movers.table, m_Rules}, shape, query);

    if (!answer) return std::nullopt;

    sweep_hit hit;
    hit.collider = collider_answering(answer->body, movers);
    hit.distance = answer->distance;
    hit.point = answer->point;
    hit.normal = answer->normal;
    hit.penetration = answer->penetration;

    if (!hit.collider) return std::nullopt;

    return hit;
}

bool impl_scene::do_overlaps_static(const upright_cylinder &aVolume) const {
    const body_table movers;

    return m_Kernels->meets_statics(scene_tables{m_Statics, movers, m_Rules}, aVolume, m_Ids->released());
}

bool impl_scene::do_overlaps_static(const parallelepiped &aVolume) const {
    const body_table movers;

    return m_Kernels->meets_statics(scene_tables{m_Statics, movers, m_Rules}, aVolume, m_Ids->released());
}
