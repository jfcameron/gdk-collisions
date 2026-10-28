# © Joseph Cameron - MIT License

cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

if (TARGET gdkcollisions)
    return()
endif()

project(gdk-collisions
    VERSION 1.0
    DESCRIPTION "3d collision detection and resolution for games: a grid scene of spheres, boxes, capsules, planes, meshes, heightfields and compounds, with triggers, rays and volume queries"
    LANGUAGES CXX)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/jfc-cmake/jfclib.cmake")

option(GDKCOLLISIONS_BUILD_TESTS "Build unit tests" ${PROJECT_IS_TOP_LEVEL})
option(GDKCOLLISIONS_BUILD_DOCS "Build documentation" OFF)
option(GDKCOLLISIONS_BUILD_BENCH "Build the benchmark harness" OFF)

if (GDKCOLLISIONS_BUILD_DOCS AND NOT GDKCOLLISIONS_BUILD_TESTS)
    add_subdirectory(docs)

    return()
endif()

add_subdirectory(extern)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/permissive->)
else()
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-pedantic>)
    endif()

    include(CheckCXXCompilerFlag)

    foreach (_strict_flag "-Werror=c++20-extensions" "-Werror=c++23-extensions")
        string(MAKE_C_IDENTIFIER "HAS${_strict_flag}" _strict_flag_symbol)

        check_cxx_compiler_flag("${_strict_flag}" ${_strict_flag_symbol})

        if (${_strict_flag_symbol})
            add_compile_options($<$<COMPILE_LANGUAGE:CXX>:${_strict_flag}>)
        endif()
    endforeach()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(GDKCOLLISIONS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/exception.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/impl_heightfield_data.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/impl_mesh_data.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/impl_shape.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/response_handler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_aabb_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_capsule_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_collision_event_tracker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_compound_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_heightfield_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_mesh_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_obb_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_plane_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_scene.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_scene_policy.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_scene_queries.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_scene_step.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/impl_sphere_collider.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/null/src/null_kernels.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/src/impl_mover_grid.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/src/impl_narrow_phase.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/src/impl_shape_geometry.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/src/impl_static_grid.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/src/impl_std_kernels.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/src/impl_std_queries.cpp")

set(GDKCOLLISIONS_PRIVATE_INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/scene/src/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/src/include")

add_library(gdkcollisions STATIC ${GDKCOLLISIONS_SOURCES})

add_library(gdk::collisions ALIAS gdkcollisions)

set_target_properties(gdkcollisions PROPERTIES EXPORT_NAME collisions PREFIX "lib")

target_include_directories(gdkcollisions
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/null/include>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    PRIVATE
        ${GDKCOLLISIONS_PRIVATE_INCLUDE_DIRECTORIES})

target_compile_features(gdkcollisions PUBLIC cxx_std_20)

target_link_libraries(gdkcollisions
    PUBLIC
        gdk::math
        jfc::dispatch
    PRIVATE
        $<BUILD_INTERFACE:jfc::hash>)

if (MSVC)
    target_compile_options(gdkcollisions PRIVATE /wd4250)
endif()

add_library(gdkcollisions_implementation INTERFACE)

if (MSVC)
    target_compile_options(gdkcollisions_implementation INTERFACE /wd4250)
endif()

target_include_directories(gdkcollisions_implementation INTERFACE ${GDKCOLLISIONS_PRIVATE_INCLUDE_DIRECTORIES})

target_link_libraries(gdkcollisions_implementation INTERFACE gdkcollisions jfc::hash)

add_library(gdkcollisions_kernel_conformance INTERFACE)

target_sources(gdkcollisions_kernel_conformance INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/test/conformance/kernel_conformance.cpp")

target_include_directories(gdkcollisions_kernel_conformance INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/test/conformance")

target_link_libraries(gdkcollisions_kernel_conformance INTERFACE gdkcollisions)

install(TARGETS gdkcollisions
    EXPORT gdk-collisions-targets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

install(DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/std/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/impl/kernels/null/include/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

install(EXPORT gdk-collisions-targets
    FILE gdk-collisions-targets.cmake
    NAMESPACE gdk::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/gdk-collisions")

write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/gdk-collisions-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

file(WRITE "${PROJECT_BINARY_DIR}/gdk-collisions-config.cmake"
"include(CMakeFindDependencyMacro)
find_dependency(gdk-math 1)
find_dependency(jfc-dispatch 1)
include(\"\${CMAKE_CURRENT_LIST_DIR}/gdk-collisions-targets.cmake\")
")

install(FILES
    "${PROJECT_BINARY_DIR}/gdk-collisions-config.cmake"
    "${PROJECT_BINARY_DIR}/gdk-collisions-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/gdk-collisions")

if (GDKCOLLISIONS_BUILD_TESTS)
    add_subdirectory(test)
endif()

if (GDKCOLLISIONS_BUILD_BENCH)
    add_subdirectory(bench)
endif()

if (GDKCOLLISIONS_BUILD_DOCS)
    add_subdirectory(docs)
endif()
