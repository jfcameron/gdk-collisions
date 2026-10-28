// © Joseph Cameron - MIT License

#include "conformance.h"

#include <gdk/collisions/std_kernels.h>

namespace gdk::collisions::conformance {
    scene_kernels_ptr_type make_kernels() { return make_std_kernels(); }

    std::string_view implementation() { return "std"; }

    bool finds_contacts() { return true; }
}
