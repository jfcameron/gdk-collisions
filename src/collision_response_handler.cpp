// © Joseph Cameron - All Rights Reserved

#include <gdk/collider.h>
#include <gdk/collision_response_handler.h>

using namespace gdk;

namespace {
    collision_vector3_type slide(const collision_vector3_type &aVelocity,
        const contact_context &aContact, const bool aPreserveSpeed) {
        const auto &aCollisionNormal = aContact.collision_normal;
        const auto &aClipPlanes = aContact.clip_planes;
        const auto slideThreshold = aContact.min_swept_speed;

        const auto &velocity = aVelocity;
        const auto vn = velocity.dot_product(aCollisionNormal);

        if (vn >= -slideThreshold) return velocity;

        auto slideVelocity = velocity - aCollisionNormal * vn;

        for (std::size_t first = 0; first + 1 < aClipPlanes.size(); ++first) {
            const auto &plane = aClipPlanes[first];

            if (slideVelocity.dot_product(plane) >= 0) continue;

            const auto facing = plane.dot_product(aCollisionNormal);

            if (facing <= -aContact.clip_plane_parallel_cosine) return collision_vector3_type::zero;

            if (facing >= aContact.clip_plane_parallel_cosine) continue;

            auto along = plane.cross_product(aCollisionNormal);
            const auto length = along.length();

            along = along * (1.0f / length);
            slideVelocity = along * slideVelocity.dot_product(along);

            for (std::size_t other = 0; other + 1 < aClipPlanes.size(); ++other)
                if (other != first && slideVelocity.dot_product(aClipPlanes[other]) < 0)
                    return collision_vector3_type::zero;

            break;
        }

        if (!aPreserveSpeed) return slideVelocity;

        return slideVelocity.normal() * velocity.length();
    }
}

const collision_response_handler &gdk::collision_response_handlers::slide_preserving_speed() {
    static const collision_response_handler handler = [](collider &aThis,
        const contact_context &aContact) {
        const auto velocity = aThis.velocity();
        return slide(velocity, aContact, true) - velocity;
    };

    return handler;
}

const collision_response_handler &gdk::collision_response_handlers::slide_projecting() {
    static const collision_response_handler handler = [](collider &aThis,
        const contact_context &aContact) {
        const auto velocity = aThis.velocity();
        return slide(velocity, aContact, false) - velocity;
    };

    return handler;
}
