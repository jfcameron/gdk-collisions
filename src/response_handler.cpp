// © Joseph Cameron - MIT License

#include <gdk/collisions/collider.h>
#include <gdk/collisions/response_handler.h>

using namespace gdk::collisions;

namespace {
    vector3_type slide(const vector3_type &aVelocity,
        const contact_context &aContact, const bool aPreserveSpeed) {
        const auto &aCollisionNormal = aContact.normal;
        const auto &aClipPlanes = aContact.clip_planes;
        const auto slideThreshold = aContact.min_swept_speed;

        const auto &velocity = aVelocity;
        const auto vn = velocity.dot_product(aCollisionNormal);

        if (vn >= -slideThreshold) return velocity;

        auto slideVelocity = velocity - aCollisionNormal * vn;

        const auto parallel = [&](const vector3_type &aPlane) {
            return aPlane.dot_product(aCollisionNormal) >= aContact.clip_plane_parallel_cosine;
        };

        for (std::size_t first = 0; first < aClipPlanes.size(); ++first) {
            const auto &plane = aClipPlanes[first];

            if (parallel(plane) || slideVelocity.dot_product(plane) >= 0) continue;

            if (plane.dot_product(aCollisionNormal) <= -aContact.clip_plane_parallel_cosine) return vector3_type::zero;

            auto along = plane.cross_product(aCollisionNormal);
            const auto length = along.length();

            along = along * (1.0f / length);
            slideVelocity = along * slideVelocity.dot_product(along);

            for (std::size_t other = 0; other < aClipPlanes.size(); ++other)
                if (other != first && !parallel(aClipPlanes[other]) && slideVelocity.dot_product(aClipPlanes[other]) < 0)
                    return vector3_type::zero;

            break;
        }

        if (!aPreserveSpeed) return slideVelocity;

        return slideVelocity.normal() * velocity.length();
    }
}

const response_handler &gdk::collisions::response_handlers::slide_preserving_speed() {
    static const response_handler handler = [](collider &aThis,
        const contact_context &aContact) {
        const auto velocity = aThis.velocity();
        return slide(velocity, aContact, true) - velocity;
    };

    return handler;
}

const response_handler &gdk::collisions::response_handlers::slide_projecting() {
    static const response_handler handler = [](collider &aThis,
        const contact_context &aContact) {
        const auto velocity = aThis.velocity();
        return slide(velocity, aContact, false) - velocity;
    };

    return handler;
}
