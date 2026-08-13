// © Joseph Cameron - All Rights Reserved

#include <gdk/collisions/exception.h>

using namespace gdk::collisions;

const char *exception::what() const noexcept {
    return mWhat.c_str();
}

exception::exception(std::string aWhat) 
: mWhat(std::move(aWhat))
{}

