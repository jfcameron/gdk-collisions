// © Joseph Cameron - MIT License

#ifndef GDK_COLLISIONS_EXCEPTION_H
#define GDK_COLLISIONS_EXCEPTION_H

#include <exception>
#include <memory>
#include <string>

namespace gdk::collisions {
    /// \brief root exception type for this project
    class exception : public std::exception {
        std::shared_ptr<const std::string> mpWhat;

    public:
        exception();

        explicit exception(const std::string &aWhat);

        exception(const exception &aOther) noexcept = default;

        exception &operator=(const exception &aOther) noexcept = default;

        const char *what() const noexcept override;
    };
}

#endif

