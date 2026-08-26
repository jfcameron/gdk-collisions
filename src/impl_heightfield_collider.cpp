// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_heightfield_collider.h>

using namespace gdk::collisions;

impl_heightfield_collider::impl_heightfield_collider(const impl_collision_policy &aPolicy,
    const response_handler &aResponseHandler,
    const floating_point_type aInverseOverlapWeight, const collider_id_type aId)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, heightfield_shape{})
{}

void impl_heightfield_collider::set_heightfield(const heightfield_data_ptr_type &aHeightfield) {
    if (aHeightfield) {
        const auto pImplementation = std::dynamic_pointer_cast<const impl_heightfield_data>(aHeightfield);

        if (!pImplementation)
            throw exception("gdk::collision: heightfield_data was not built by "
                "impl_heightfield_data::make and cannot be used by this scene implementation");

        std::get<heightfield_shape>(primary_shape()).data = pImplementation;
    }
    else std::get<heightfield_shape>(primary_shape()).data = nullptr;
}

heightfield_data_ptr_type impl_heightfield_collider::heightfield() const {
    return std::get<heightfield_shape>(primary_shape()).data;
}
