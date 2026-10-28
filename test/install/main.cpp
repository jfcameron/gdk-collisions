// © Joseph Cameron - MIT License

#include <gdk/collisions/exception.h>
#include <gdk/collisions/heightfield_data.h>
#include <gdk/collisions/mesh_collider.h>
#include <gdk/collisions/mesh_data.h>
#include <gdk/collisions/null_kernels.h>
#include <gdk/collisions/scene.h>
#include <gdk/collisions/sphere_collider.h>

int main() {
    using namespace gdk::collisions;

    int entered = 0;

    const auto pScene = make_scene([&entered](const collision_event &aEvent) {
        if (aEvent.type == event_type::enter) ++entered;
    }, nullptr);

    const auto pFloor = pScene->make_static_mesh_collider(matrix4x4_type::identity,
        make_mesh_data({{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, {0, 1, 2, 0, 2, 3}));

    const auto pBall = pScene->make_sphere_collider();
    pBall->set_position({0, 1, 0});
    pBall->set_velocity({0, -2, 0});

    pScene->update(1.0f);
    pScene->process_events();

    try {
        static_cast<void>(make_heightfield_data(1, 1, {0}));

        return 1;
    }
    catch (const exception &) {}

    const auto pGhosts = make_scene(nullptr, nullptr, scene_policy{}, make_null_kernels());
    const auto pGhost = pGhosts->make_sphere_collider();
    pGhost->set_velocity({0, -2, 0});
    pGhosts->update(1.0f);

    return entered == 1 && pBall->transform().translation().y > 0.4f
        && pGhost->transform().translation().y == -2.0f ? 0 : 1;
}
