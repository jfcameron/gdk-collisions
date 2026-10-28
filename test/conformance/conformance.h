// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_CONFORMANCE_H
#define GDK_COLLISIONS_CONFORMANCE_H

#include <gdk/collisions/kernels/scene_kernels.h>

#include <string_view>

/// \file
/// \brief the kernel interface's conformance suite: link it into the tests of every scene_kernels implementation,
/// and define these three
namespace gdk::collisions::conformance {
    /// \brief a fresh kernels object of the implementation under test
    [[nodiscard]] scene_kernels_ptr_type make_kernels();

    //! its name, in the report. eg: `std`, `null`
    [[nodiscard]] std::string_view implementation();

    //! false for kernels that find nothing by design, as the null kernels do
    [[nodiscard]] bool finds_contacts();
}

#endif
