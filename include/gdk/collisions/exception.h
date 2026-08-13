// © Joseph Cameron - All Rights Reserved

#ifndef GDK_COLLISIONS_EXCEPTION_H
#define GDK_COLLISIONS_EXCEPTION_H

#include <gdk/collisions/types.h>

#include <exception>

namespace gdk::collisions {
    /// \brief root exception type for this project
    class exception : public std::exception {
    public:
        exception() = default;
        
        exception(std::string aWhat);
       
         virtual ~exception() override = default;

        virtual const char *what() const noexcept override;

    private:
        std::string mWhat = "gdk::collisions::exception";
    };
}

#endif

