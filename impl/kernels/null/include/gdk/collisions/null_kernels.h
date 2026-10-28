// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_NULL_KERNELS_H
#define GDK_COLLISIONS_NULL_KERNELS_H

#include <gdk/collisions/kernels/scene_kernels.h>
#include <gdk/collisions/types.h>

namespace gdk::collisions {
    /// \brief kernels that find nothing: bodies pass through each other and rays strike nothing
    [[nodiscard]] scene_kernels_ptr_type make_null_kernels();
}

#endif
