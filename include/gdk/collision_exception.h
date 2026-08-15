// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISION_EXCEPTION_H
#define GDK_COLLISION_EXCEPTION_H

#include <gdk/collision_types.h>

#include <exception>

namespace gdk {
    /// \brief root exception type for this project
    class collision_exception : public std::exception {
    public:
        collision_exception() = default;
        
        collision_exception(std::string aWhat);
       
         virtual ~collision_exception() override = default;

        virtual const char *what() const noexcept override;

    private:
        std::string mWhat = "gdk::collision_exception";
    };
}

#endif

