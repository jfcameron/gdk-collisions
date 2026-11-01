// © Joseph Cameron - MIT License

#include <gdk/collisions/exception.h>

namespace gdk::collisions {
    exception::exception()
    : exception("gdk::collisions::exception")
    {}

    exception::exception(const std::string &aWhat)
    : mpWhat(std::make_shared<const std::string>(aWhat))
    {}

    const char *exception::what() const noexcept { return mpWhat->c_str(); }
}
