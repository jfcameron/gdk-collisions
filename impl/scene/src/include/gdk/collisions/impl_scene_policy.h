// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_IMPL_SCENE_POLICY_H
#define GDK_COLLISIONS_IMPL_SCENE_POLICY_H

#include <gdk/collisions/scene_policy.h>

namespace gdk::collisions {
    /// \brief throw unless every value in aPolicy can drive a scene
    /// \exception exception naming the first value that cannot
    void require_valid_policy(const scene_policy &aPolicy);
}

#endif
