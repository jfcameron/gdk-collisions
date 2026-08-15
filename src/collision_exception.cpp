// © Joseph Cameron - All Rights Reserved

#include <gdk/collision_exception.h>

using namespace gdk;

const char *collision_exception::what() const noexcept {
    return mWhat.c_str();
}

collision_exception::collision_exception(std::string aWhat) 
: mWhat(std::move(aWhat))
{}

