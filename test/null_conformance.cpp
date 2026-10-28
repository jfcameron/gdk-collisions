// © Joseph Cameron - MIT License

#include "conformance.h"

#include <gdk/collisions/null_kernels.h>

namespace gdk::collisions::conformance {
    scene_kernels_ptr_type make_kernels() { return make_null_kernels(); }

    std::string_view implementation() { return "null"; }

    bool finds_contacts() { return false; }
}
