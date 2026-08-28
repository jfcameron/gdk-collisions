// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_SCENE_H
#define GDK_COLLISIONS_SCENE_H

#include <gdk/collisions/events.h>
#include <gdk/collisions/response_handler.h>
#include <gdk/collisions/types.h>
#include <gdk/collisions/contact.h>
#include <gdk/collisions/raycast_hit.h>
#include <gdk/collisions/rigid_transform.h>

#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace gdk::collisions {
    /// \brief the 3D space in which colliders exist & collisions occur
    class scene {
    public:
        virtual ~scene() = default;

        /// \brief performs collision detection & response for all colliders in the scene
        void update(const delta_time_type aDeltaTime) { do_update(aDeltaTime); }

        /// \brief hand this step's collision and trigger events to the observers.
        void process_events() { do_process_events(); }

        /// \brief find the nearest collider along a ray.
        [[nodiscard]] std::optional<raycast_hit> raycast(const vector3_type &aOrigin,
            const vector3_type &aDirection,
            const floating_point_type aMaxDistance) const {
            return do_raycast(aOrigin, aDirection, aMaxDistance);
        }

        /// \brief every solid contact detected during the most recent step.
        [[nodiscard]] std::vector<contact> contacts() const {
            return do_contacts();
        }

        /// \brief the contacts from the most recent step that involve aCollider.
        [[nodiscard]] std::vector<contact> contacts_for(const collider &aCollider) const {
            return do_contacts_for(aCollider);
        }

        /// \brief create an axis aligned box collider in the scene
        [[nodiscard]] box_collider_ptr_type make_axis_aligned_box_collider(
            const response_handler &aHandler = response_handlers::slide_preserving_speed()) {
            return do_make_axis_aligned_box_collider(aHandler);
        }

        /// \brief create an axis aligned box trigger volume in the scene
        [[nodiscard]] box_collider_ptr_type make_axis_aligned_box_trigger() {
            return do_make_axis_aligned_box_trigger();
        }

        /// \brief create a new immovable axis aligned box in the scene
        /// \param aHalfExtents the box's size.
        [[nodiscard]] const_box_collider_ptr_type make_static_axis_aligned_box_collider(
            const matrix4x4_type &aTransform,
            const vector3_type &aHalfExtents) {
            require_rigid_transform(aTransform, "make_static_axis_aligned_box_collider");

            return do_make_static_axis_aligned_box_collider(aTransform, aHalfExtents);
        }

        /// \brief create a new immovable axis aligned box trigger volume in the scene
        /// \param aHalfExtents the box's size. 
        [[nodiscard]] const_box_collider_ptr_type make_static_axis_aligned_box_trigger(
            const matrix4x4_type &aTransform,
            const vector3_type &aHalfExtents) {
            require_rigid_transform(aTransform, "make_static_axis_aligned_box_trigger");

            return do_make_static_axis_aligned_box_trigger(aTransform, aHalfExtents);
        }

        /// \brief create a sphere collider in the scene
        [[nodiscard]] sphere_collider_ptr_type make_sphere_collider(
            const response_handler &aHandler = response_handlers::slide_preserving_speed()) {
            return do_make_sphere_collider(aHandler);
        }

        /// \brief create a sphere trigger volume in the scene
        [[nodiscard]] sphere_collider_ptr_type make_sphere_trigger() { return do_make_sphere_trigger(); }

        /// \brief create a new immovable sphere in the scene
        /// \param aRadius the sphere's size.
        [[nodiscard]] const_sphere_collider_ptr_type make_static_sphere_collider(
            const matrix4x4_type &aTransform,
            const floating_point_type aRadius) {
            require_rigid_transform(aTransform, "make_static_sphere_collider");

            return do_make_static_sphere_collider(aTransform, aRadius);
        }

        /// \brief create a new immovable sphere trigger volume in the scene
        /// \param aRadius the sphere's size.
        [[nodiscard]] const_sphere_collider_ptr_type make_static_sphere_trigger(
            const matrix4x4_type &aTransform,
            const floating_point_type aRadius) {
            require_rigid_transform(aTransform, "make_static_sphere_trigger");

            return do_make_static_sphere_trigger(aTransform, aRadius);
        }

        /// \brief create a capsule collider in the scene
        [[nodiscard]] capsule_collider_ptr_type make_capsule_collider(
            const response_handler &aHandler = response_handlers::slide_preserving_speed()) {
            return do_make_capsule_collider(aHandler);
        }

        /// \brief create a new immovable capsule in the scene
        /// \param aRadius the capsule's radius.
        /// \param aHalfHeight distance from the centre to either end of the segment, along local Y.
        [[nodiscard]] const_capsule_collider_ptr_type make_static_capsule_collider(
            const matrix4x4_type &aTransform,
            const floating_point_type aRadius,
            const floating_point_type aHalfHeight) {
            require_rigid_transform(aTransform, "make_static_capsule_collider");

            return do_make_static_capsule_collider(aTransform, aRadius, aHalfHeight);
        }

        /// \brief create an oriented box collider in the scene
        [[nodiscard]] obb_collider_ptr_type make_obb_collider(
            const response_handler &aHandler = response_handlers::slide_preserving_speed()) {
            return do_make_obb_collider(aHandler);
        }

        /// \brief create a new immovable oriented box in the scene
        /// \param aHalfExtents the box's size. 
        [[nodiscard]] const_obb_collider_ptr_type make_static_obb_collider(
            const matrix4x4_type &aTransform,
            const vector3_type &aHalfExtents) {
            require_rigid_transform(aTransform, "make_static_obb_collider");

            return do_make_static_obb_collider(aTransform, aHalfExtents);
        }

        /// \brief create a collider assembled from several primitives
        [[nodiscard]] compound_collider_ptr_type make_compound_collider(
            const response_handler &aHandler = response_handlers::slide_preserving_speed()) {
            return do_make_compound_collider(aHandler);
        }

        /// \brief create a new immovable collider assembled from several primitives.
        ///
        /// \code
        /// const auto pRock = pScene->make_static_compound_collider(transform,
        ///     [](compound_collider &aBuild) {
        ///         aBuild.add_box({0, 0, 0}, {1, 0.5f, 1});
        ///         aBuild.add_sphere({0, 0.8f, 0}, 0.6f);
        ///     });
        /// \endcode
        [[nodiscard]] const_compound_collider_ptr_type make_static_compound_collider(
            const matrix4x4_type &aTransform,
            const std::function<void(compound_collider &)> &aBuild) {
            require_rigid_transform(aTransform, "make_static_compound_collider");

            return do_make_static_compound_collider(aTransform, aBuild);
        }

        /// \brief create a collider whose geometry is a shared triangle mesh.
        ///
        /// \warning A mesh cannot collide with another mesh. \see mesh_collider
        [[nodiscard]] mesh_collider_ptr_type make_mesh_collider(
            const response_handler &aHandler = response_handlers::slide_preserving_speed()) {
            return do_make_mesh_collider(aHandler);
        }

        /// \brief create a trigger volume whose geometry is a shared triangle mesh.
        [[nodiscard]] mesh_collider_ptr_type make_mesh_trigger() { return do_make_mesh_trigger(); }

        /// \brief create an immovable collider whose geometry is a shared triangle mesh.
        [[nodiscard]] const_mesh_collider_ptr_type make_static_mesh_collider(
            const matrix4x4_type &aTransform, const mesh_data_ptr_type &aMesh) {
            require_rigid_transform(aTransform, "make_static_mesh_collider");

            return do_make_static_mesh_collider(aTransform, aMesh);
        }

        /// \brief create an immovable trigger volume whose geometry is a shared triangle mesh
        [[nodiscard]] const_mesh_collider_ptr_type make_static_mesh_trigger(
            const matrix4x4_type &aTransform, const mesh_data_ptr_type &aMesh) {
            require_rigid_transform(aTransform, "make_static_mesh_trigger");

            return do_make_static_mesh_trigger(aTransform, aMesh);
        }

        /// \brief create a collider whose geometry is shared terrain.
        [[nodiscard]] heightfield_collider_ptr_type make_heightfield_collider(
            const response_handler &aHandler = response_handlers::slide_preserving_speed()) {
            return do_make_heightfield_collider(aHandler);
        }

        /// \brief create immovable terrain
        [[nodiscard]] const_heightfield_collider_ptr_type make_static_heightfield_collider(
            const matrix4x4_type &aTransform, const heightfield_data_ptr_type &aHeightfield) {
            require_rigid_transform(aTransform, "make_static_heightfield_collider");

            return do_make_static_heightfield_collider(aTransform, aHeightfield);
        }

        /// \brief create an immovable infinite plane. The normal of the plane is parallel to local +Y
        [[nodiscard]] const_plane_collider_ptr_type make_static_plane_collider(
            const matrix4x4_type &aTransform) {
            require_rigid_transform(aTransform, "make_static_plane_collider");

            return do_make_static_plane_collider(aTransform);
        }

    protected:
        virtual void do_update(const delta_time_type aDeltaTime) = 0;

        virtual void do_process_events() = 0;

        [[nodiscard]] virtual std::vector<contact> do_contacts() const = 0;
        [[nodiscard]] virtual std::vector<contact> do_contacts_for(const collider &aCollider) const = 0;

        [[nodiscard]] virtual std::optional<raycast_hit> do_raycast(const vector3_type &aOrigin,
            const vector3_type &aDirection,
            const floating_point_type aMaxDistance) const = 0;

        [[nodiscard]] virtual box_collider_ptr_type do_make_axis_aligned_box_collider(const response_handler &) = 0;
        [[nodiscard]] virtual box_collider_ptr_type do_make_axis_aligned_box_trigger() = 0;
        [[nodiscard]] virtual const_box_collider_ptr_type do_make_static_axis_aligned_box_collider(const matrix4x4_type &, const vector3_type &) = 0;
        [[nodiscard]] virtual const_box_collider_ptr_type do_make_static_axis_aligned_box_trigger(const matrix4x4_type &, const vector3_type &) = 0;

        [[nodiscard]] virtual sphere_collider_ptr_type do_make_sphere_collider(const response_handler &) = 0;
        [[nodiscard]] virtual sphere_collider_ptr_type do_make_sphere_trigger() = 0;
        [[nodiscard]] virtual const_sphere_collider_ptr_type do_make_static_sphere_collider(const matrix4x4_type &, const floating_point_type) = 0;
        [[nodiscard]] virtual const_sphere_collider_ptr_type do_make_static_sphere_trigger(const matrix4x4_type &, const floating_point_type) = 0;

        [[nodiscard]] virtual capsule_collider_ptr_type do_make_capsule_collider(const response_handler &) = 0;
        [[nodiscard]] virtual const_capsule_collider_ptr_type do_make_static_capsule_collider(const matrix4x4_type &, const floating_point_type,
            const floating_point_type) = 0;

        [[nodiscard]] virtual obb_collider_ptr_type do_make_obb_collider(const response_handler &) = 0;
        [[nodiscard]] virtual const_obb_collider_ptr_type do_make_static_obb_collider(const matrix4x4_type &, const vector3_type &) = 0;

        [[nodiscard]] virtual compound_collider_ptr_type do_make_compound_collider(const response_handler &) = 0;
        [[nodiscard]] virtual const_compound_collider_ptr_type do_make_static_compound_collider(
            const matrix4x4_type &, const std::function<void(compound_collider &)> &) = 0;

        [[nodiscard]] virtual const_plane_collider_ptr_type do_make_static_plane_collider(const matrix4x4_type &) = 0;

        [[nodiscard]] virtual mesh_collider_ptr_type do_make_mesh_collider(const response_handler &) = 0;
        [[nodiscard]] virtual mesh_collider_ptr_type do_make_mesh_trigger() = 0;
        [[nodiscard]] virtual const_mesh_collider_ptr_type do_make_static_mesh_collider(const matrix4x4_type &, const mesh_data_ptr_type &) = 0;
        [[nodiscard]] virtual const_mesh_collider_ptr_type do_make_static_mesh_trigger(const matrix4x4_type &, const mesh_data_ptr_type &) = 0;

        [[nodiscard]] virtual heightfield_collider_ptr_type do_make_heightfield_collider(const response_handler &) = 0;
        [[nodiscard]] virtual const_heightfield_collider_ptr_type do_make_static_heightfield_collider(const matrix4x4_type &, const heightfield_data_ptr_type &) = 0;
    };
}

#endif
