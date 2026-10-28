// © Joseph Cameron - MIT License

#include <gdk/collisions/std_kernels.h>
#include <gdk/collisions/impl_aabb_collider.h>
#include <gdk/collisions/impl_capsule_collider.h>
#include <gdk/collisions/impl_collision_profile.h>
#include <gdk/collisions/impl_compound_collider.h>
#include <gdk/collisions/impl_heightfield_collider.h>
#include <gdk/collisions/impl_mesh_collider.h>
#include <gdk/collisions/impl_obb_collider.h>
#include <gdk/collisions/impl_plane_collider.h>
#include <gdk/collisions/impl_scene.h>
#include <gdk/collisions/impl_sphere_collider.h>

#include <algorithm>
#include <utility>

using namespace gdk::collisions;

namespace {
    [[nodiscard]] contact_rules rules_of(const scene_policy &aPolicy) {
        return contact_rules{aPolicy.MIN_SWEPT_SPEED, aPolicy.PENETRATION_DEPTH_EPSILON, aPolicy.NORMALIZATION_THRESHOLD,
            aPolicy.AXIS_ENTRY_EPSILON, aPolicy.TIME_OF_IMPACT_EPSILON};
    }
}

scene_ptr_type gdk::collisions::make_scene(collision_observer_type aCollisionObserver,
    trigger_observer_type aTriggerObserver, scene_policy aPolicy, scene_kernels_ptr_type aKernels) {
    require_valid_policy(aPolicy);

    return std::make_shared<impl_scene>(std::move(aCollisionObserver), std::move(aTriggerObserver), aPolicy,
        aKernels ? std::move(aKernels) : make_std_kernels());
}

impl_scene::impl_scene(collision_observer_type aCollisionObserver, trigger_observer_type aTriggerObserver,
    scene_policy aPolicy, scene_kernels_ptr_type aKernels)
: m_Policy(aPolicy)
, m_Rules(rules_of(aPolicy))
, m_Kernels(std::move(aKernels))
, m_Events(std::move(aCollisionObserver), std::move(aTriggerObserver))
{}

scene::statistics impl_scene::do_take_statistics() {
    return std::exchange(m_Statistics, statistics{});
}

void impl_scene::do_process_events() {
    m_Events.dispatch();
}

scene_tables impl_scene::tables() const {
    return scene_tables{m_Statics, m_Movers, m_Rules};
}

template <typename pointer_type>
std::uint32_t impl_scene::surface_uses<pointer_type>::take(std::vector<pointer_type> &aTable, const pointer_type &aSurface) {
    if (const auto it = index.find(aSurface.get()); it != index.end()) {
        ++count[it->second];
        return it->second;
    }

    std::uint32_t slot;

    if (!free.empty()) {
        slot = free.back();
        free.pop_back();
        aTable[slot] = aSurface;
        count[slot] = 1;
    }
    else {
        slot = static_cast<std::uint32_t>(aTable.size());
        aTable.push_back(aSurface);
        count.push_back(1);
    }

    index.emplace(aSurface.get(), slot);
    return slot;
}

template <typename pointer_type>
void impl_scene::surface_uses<pointer_type>::give_back(std::vector<pointer_type> &aTable, const std::uint32_t aIndex) {
    if (--count[aIndex] > 0) return;

    index.erase(aTable[aIndex].get());
    aTable[aIndex] = nullptr;
    free.push_back(aIndex);
}

void impl_scene::append_row(body_table &aTable, const impl_collider &aBody, const body_motion aMotion,
    const bool aTrigger) {
    aTable.id.push_back(aBody.id());
    aTable.position.push_back(aBody.position());
    aTable.velocity.push_back(aBody.velocity());
    aTable.rotation.push_back(aBody.rotation());
    aTable.extent.push_back(aBody.world_bounds());
    aTable.on.push_back(aBody.collision_layers());
    aTable.collides_with.push_back(aBody.collides_with());
    aTable.motion.push_back(aMotion);
    aTable.trigger.push_back(aTrigger ? 1 : 0);
    aTable.live.push_back(1);
    aTable.first_part.push_back(static_cast<std::uint32_t>(aTable.parts.size()));
    aTable.part_count.push_back(static_cast<std::uint32_t>(aBody.parts().size()));

    const auto pMesh = aBody.surface_mesh();
    const auto pHeightfield = aBody.surface_heightfield();

    for (auto part : aBody.parts()) {
        part.surface = NO_SURFACE;

        if (part.kind == shape_kind::mesh && pMesh) {
            part.surface = static_cast<std::uint32_t>(aTable.surfaces.meshes.size());
            aTable.surfaces.meshes.push_back(pMesh);
        }
        else if (part.kind == shape_kind::heightfield && pHeightfield) {
            part.surface = static_cast<std::uint32_t>(aTable.surfaces.heightfields.size());
            aTable.surfaces.heightfields.push_back(pHeightfield);
        }

        aTable.parts.push_back(part);
    }
}

void impl_scene::clear_table(body_table &aTable) {
    aTable.id.clear();
    aTable.position.clear();
    aTable.velocity.clear();
    aTable.rotation.clear();
    aTable.extent.clear();
    aTable.on.clear();
    aTable.collides_with.clear();
    aTable.motion.clear();
    aTable.trigger.clear();
    aTable.live.clear();
    aTable.first_part.clear();
    aTable.part_count.clear();
    aTable.parts.clear();
    aTable.surfaces.meshes.clear();
    aTable.surfaces.heightfields.clear();
}

void impl_scene::fill_static_row(const std::uint32_t aRow, const impl_collider &aBody, const bool aTrigger) {
    body_table scratch;
    append_row(scratch, aBody, body_motion::fixed, aTrigger);

    for (auto &part : scratch.parts) {
        if (part.surface == NO_SURFACE) continue;

        if (part.kind == shape_kind::mesh)
            part.surface = m_MeshUses.take(m_Statics.surfaces.meshes, scratch.surfaces.meshes[part.surface]);
        else if (part.kind == shape_kind::heightfield)
            part.surface = m_HeightfieldUses.take(m_Statics.surfaces.heightfields,
                scratch.surfaces.heightfields[part.surface]);
    }

    if (aRow == m_Statics.size()) {
        m_Statics.id.push_back(0);
        m_Statics.position.emplace_back();
        m_Statics.velocity.emplace_back();
        m_Statics.rotation.emplace_back();
        m_Statics.extent.emplace_back();
        m_Statics.on.push_back(0);
        m_Statics.collides_with.push_back(0);
        m_Statics.motion.push_back(body_motion::fixed);
        m_Statics.trigger.push_back(0);
        m_Statics.live.push_back(0);
        m_Statics.first_part.push_back(static_cast<std::uint32_t>(m_Statics.parts.size()));
        m_Statics.part_count.push_back(0);
        m_StaticPartCapacity.push_back(0);
    }

    m_Statics.id[aRow] = scratch.id[0];
    m_Statics.position[aRow] = scratch.position[0];
    m_Statics.velocity[aRow] = scratch.velocity[0];
    m_Statics.rotation[aRow] = scratch.rotation[0];
    m_Statics.extent[aRow] = scratch.extent[0];
    m_Statics.on[aRow] = scratch.on[0];
    m_Statics.collides_with[aRow] = scratch.collides_with[0];
    m_Statics.trigger[aRow] = scratch.trigger[0];
    m_Statics.live[aRow] = 1;
    m_Statics.part_count[aRow] = scratch.part_count[0];

    if (scratch.part_count[0] > m_StaticPartCapacity[aRow]) {
        m_Statics.first_part[aRow] = static_cast<std::uint32_t>(m_Statics.parts.size());
        m_StaticPartCapacity[aRow] = scratch.part_count[0];
        m_Statics.parts.insert(m_Statics.parts.end(), scratch.parts.begin(), scratch.parts.end());
    }
    else std::copy(scratch.parts.begin(), scratch.parts.end(), m_Statics.parts.begin() + m_Statics.first_part[aRow]);
}

void impl_scene::release_static_surfaces(const std::uint32_t aRow) {
    const auto first = m_Statics.first_part[aRow];

    for (std::uint32_t i = 0; i < m_Statics.part_count[aRow]; ++i) {
        const auto &part = m_Statics.parts[first + i];
        if (part.surface == NO_SURFACE) continue;

        if (part.kind == shape_kind::mesh) m_MeshUses.give_back(m_Statics.surfaces.meshes, part.surface);
        else if (part.kind == shape_kind::heightfield)
            m_HeightfieldUses.give_back(m_Statics.surfaces.heightfields, part.surface);
    }
}

void impl_scene::register_static(const impl_collider_ptr_type &aBody, const bool aTrigger) {
    std::uint32_t row;

    if (!m_FreeStaticRows.empty()) {
        row = m_FreeStaticRows.back();
        m_FreeStaticRows.pop_back();
    }
    else row = static_cast<std::uint32_t>(m_Statics.size());

    fill_static_row(row, *aBody, aTrigger);

    if (row >= m_StaticOwners.size()) m_StaticOwners.resize(static_cast<std::size_t>(row) + 1);
    m_StaticOwners[row] = aBody;

    const auto id = static_cast<std::size_t>(aBody->id());
    if (id >= m_StaticRowOfId.size()) m_StaticRowOfId.resize(id + 1, NO_ROW);
    m_StaticRowOfId[id] = row;

    const std::uint32_t added[1] = {row};
    m_Kernels->statics_changed(tables(), added, {});
}

void impl_scene::forget_dead_statics() {
    std::vector<std::uint32_t> removed;

    for (const auto id : m_Ids->collect()) {
        if (id >= m_StaticRowOfId.size() || m_StaticRowOfId[id] == NO_ROW) continue;

        const auto row = m_StaticRowOfId[id];
        m_StaticRowOfId[id] = NO_ROW;

        release_static_surfaces(row);
        m_Statics.live[row] = 0;
        m_StaticOwners[row].reset();
        m_FreeStaticRows.push_back(row);

        removed.push_back(row);
    }

    if (!removed.empty()) m_Kernels->statics_changed(tables(), {}, removed);
}

impl_collider_ptr_type impl_scene::owner_of(const body_ref &aBody) const {
    return aBody.fixed ? m_StaticOwners[aBody.row].lock() : m_MoverOwners[aBody.row];
}

void impl_scene::do_update(const delta_time_type aDeltaTime) {
    GDK_COLLISION_PROFILE_BEGIN(total_ns);

    forget_dead_statics();

    m_Ids->begin_update();

    const auto subDeltaTime = aDeltaTime / m_Policy.STEPS_PER_UPDATE;
    for (unsigned short int step(0); step < m_Policy.STEPS_PER_UPDATE; ++step) update_step(subDeltaTime);

    GDK_COLLISION_PROFILE_BEGIN(events_ns);

    m_Events.flush();

    GDK_COLLISION_PROFILE_END(events_ns);

    clear_velocities();

    m_Ids->end_update();

    GDK_COLLISION_PROFILE_END(total_ns);
    GDK_COLLISION_PROFILE_COUNT_STEP();
}

aabb_collider_ptr_type impl_scene::do_make_aabb_collider(const response_handler &aHandler) {
    return make_dynamic<impl_aabb_collider>(aHandler);
}

aabb_collider_ptr_type impl_scene::do_make_aabb_trigger() {
    return make_trigger<impl_aabb_collider>();
}

const_aabb_collider_ptr_type impl_scene::do_make_static_aabb_collider(
    const matrix4x4_type &aTransform, const vector3_type &aHalfExtents) {
    return make_static<impl_aabb_collider>(aTransform, false,
        [&](auto &aBox) { aBox.set_half_extents(aHalfExtents); });
}

const_aabb_collider_ptr_type impl_scene::do_make_static_aabb_trigger(
    const matrix4x4_type &aTransform, const vector3_type &aHalfExtents) {
    return make_static<impl_aabb_collider>(aTransform, true,
        [&](auto &aBox) { aBox.set_half_extents(aHalfExtents); });
}

const_plane_collider_ptr_type impl_scene::do_make_static_plane_collider(const matrix4x4_type &aTransform) {
    return make_static<impl_plane_collider>(aTransform, false, [](auto &) {});
}

mesh_collider_ptr_type impl_scene::do_make_mesh_collider(const response_handler &aHandler) {
    return make_dynamic<impl_mesh_collider>(aHandler);
}

mesh_collider_ptr_type impl_scene::do_make_mesh_trigger() {
    return make_trigger<impl_mesh_collider>();
}

const_mesh_collider_ptr_type impl_scene::do_make_static_mesh_collider(
    const matrix4x4_type &aTransform, const mesh_data_ptr_type &aMesh) {
    return make_static<impl_mesh_collider>(aTransform, false, [&](auto &aBody) { aBody.set_mesh(aMesh); });
}

const_mesh_collider_ptr_type impl_scene::do_make_static_mesh_trigger(
    const matrix4x4_type &aTransform, const mesh_data_ptr_type &aMesh) {
    return make_static<impl_mesh_collider>(aTransform, true, [&](auto &aBody) { aBody.set_mesh(aMesh); });
}

heightfield_collider_ptr_type impl_scene::do_make_heightfield_collider(const response_handler &aHandler) {
    return make_dynamic<impl_heightfield_collider>(aHandler);
}

const_heightfield_collider_ptr_type impl_scene::do_make_static_heightfield_collider(
    const matrix4x4_type &aTransform, const heightfield_data_ptr_type &aHeightfield) {
    return make_static<impl_heightfield_collider>(aTransform, false,
        [&](auto &aBody) { aBody.set_heightfield(aHeightfield); });
}

compound_collider_ptr_type impl_scene::do_make_compound_collider(const response_handler &aHandler) {
    return make_dynamic<impl_compound_collider>(aHandler);
}

const_compound_collider_ptr_type impl_scene::do_make_static_compound_collider(
    const matrix4x4_type &aTransform, const std::function<void(compound_collider &)> &aBuild) {
    return make_static<impl_compound_collider>(aTransform, false,
        [&](auto &aBody) { if (aBuild) aBuild(aBody); });
}

obb_collider_ptr_type impl_scene::do_make_obb_collider(const response_handler &aHandler) {
    return make_dynamic<impl_obb_collider>(aHandler);
}

const_obb_collider_ptr_type impl_scene::do_make_static_obb_collider(
    const matrix4x4_type &aTransform, const vector3_type &aHalfExtents) {
    return make_static<impl_obb_collider>(aTransform, false,
        [&](auto &aBox) { aBox.set_half_extents(aHalfExtents); });
}

capsule_collider_ptr_type impl_scene::do_make_capsule_collider(const response_handler &aHandler) {
    return make_dynamic<impl_capsule_collider>(aHandler);
}

const_capsule_collider_ptr_type impl_scene::do_make_static_capsule_collider(
    const matrix4x4_type &aTransform, const floating_point_type aRadius, const floating_point_type aHalfHeight) {
    return make_static<impl_capsule_collider>(aTransform, false, [&](auto &aCapsule) {
        aCapsule.set_radius(aRadius);
        aCapsule.set_half_height(aHalfHeight);
    });
}

sphere_collider_ptr_type impl_scene::do_make_sphere_collider(const response_handler &aHandler) {
    return make_dynamic<impl_sphere_collider>(aHandler);
}

sphere_collider_ptr_type impl_scene::do_make_sphere_trigger() {
    return make_trigger<impl_sphere_collider>();
}

const_sphere_collider_ptr_type impl_scene::do_make_static_sphere_collider(
    const matrix4x4_type &aTransform, const floating_point_type aRadius) {
    return make_static<impl_sphere_collider>(aTransform, false,
        [&](auto &aSphere) { aSphere.set_radius(aRadius); });
}

const_sphere_collider_ptr_type impl_scene::do_make_static_sphere_trigger(
    const matrix4x4_type &aTransform, const floating_point_type aRadius) {
    return make_static<impl_sphere_collider>(aTransform, true,
        [&](auto &aSphere) { aSphere.set_radius(aRadius); });
}

