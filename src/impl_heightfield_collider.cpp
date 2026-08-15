// © Joseph Cameron - All Rights Reserved

#include <gdk/collision_exception.h>
#include <gdk/impl_heightfield_collider.h>

using namespace gdk;

impl_heightfield_collider::impl_heightfield_collider(const impl_collision_policy &aPolicy,
    const collision_response_handler &aResponseHandler,
    const collision_floating_point_type aInverseOverlapWeight, const collider_id_type aId)
: impl_collider(aPolicy, aResponseHandler, aInverseOverlapWeight, aId, heightfield_shape{})
{}

void impl_heightfield_collider::set_heightfield(const heightfield_data_ptr_type &aHeightfield) {
    if (aHeightfield) {
        const auto pImplementation = std::dynamic_pointer_cast<const impl_heightfield_data>(aHeightfield);

        if (!pImplementation)
            throw collision_exception("gdk::collision: heightfield_data was not built by "
                "impl_heightfield_data::make and cannot be used by this collision_scene implementation");

        std::get<heightfield_shape>(primary_shape()).data = pImplementation;
    }
    else std::get<heightfield_shape>(primary_shape()).data = nullptr;
}

heightfield_data_ptr_type impl_heightfield_collider::heightfield() const {
    return std::get<heightfield_shape>(primary_shape()).data;
}
