// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_EXCEPTION_H
#define GDK_COLLISIONS_EXCEPTION_H

#include <exception>
#include <string>

namespace gdk::collisions {
    /// \brief root exception type for this project
    class exception : public std::exception {
    public:
        exception() = default;

        explicit exception(std::string aWhat);

        virtual ~exception() override = default;

        virtual const char *what() const noexcept override;

    private:
        std::string mWhat = "gdk::collisions::exception";
    };
}

#endif

