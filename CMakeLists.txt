# © Joseph Cameron - All Rights Reserved

cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

project(null)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/jfc-cmake/jfclib.cmake")

option(JFC_BUILD_DOCS "Build documentation" OFF)
option(JFC_BUILD_TESTS "Build unit tests" OFF)
option(JFC_BUILD_BENCH "Build the benchmark harness" OFF)

option(JFC_COLLISION_STRICT_FP "slower but build the library for better cross-machine determinism" OFF)

if (JFC_COLLISION_STRICT_FP)
    if (MSVC)
        set(JFC_COLLISION_FP_FLAGS /fp:strict)
    else()
        set(JFC_COLLISION_FP_FLAGS -fno-fast-math -ffp-contract=off)
    endif()
endif()

set(_jfc_outer_build_tests "${JFC_BUILD_TESTS}")
set(_jfc_outer_build_docs "${JFC_BUILD_DOCS}")
set(_jfc_outer_build_bench "${JFC_BUILD_BENCH}")

set(JFC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(JFC_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(JFC_BUILD_BENCH OFF CACHE BOOL "" FORCE)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/extern/gdk-math")

set(JFC_BUILD_TESTS "${_jfc_outer_build_tests}" CACHE BOOL "" FORCE)
set(JFC_BUILD_DOCS "${_jfc_outer_build_docs}" CACHE BOOL "" FORCE)
set(JFC_BUILD_BENCH "${_jfc_outer_build_bench}" CACHE BOOL "" FORCE)

jfc_project(library
    NAME "gdkcollisions"
    VERSION 0.1
    DESCRIPTION "3d collision detection and resolution"
    C++_STANDARD 17
    C_STANDARD 90
    TYPE STATIC

    PUBLIC_INCLUDE_DIRECTORIES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/impl/include

        ${gdkmath_INCLUDE_DIRECTORIES}

    LIBRARIES
        ${gdkmath_LIBRARIES}

    DEPENDENCIES
        "gdkmath"

    SOURCE_LIST
        ${CMAKE_CURRENT_SOURCE_DIR}/src/collision_exception.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/collision_response_handler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/collision_scene.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_axis_aligned_box.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_broadphase_grid.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_capsule_collider.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_collider.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_collision_event_tracker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_collision_scene.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_compound_collider.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_dynamic_broadphase_grid.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_heightfield_collider.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_heightfield_data.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_mesh_collider.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_mesh_data.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_narrow_phase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_obb_collider.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_plane_collider.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/impl_sphere_collider.cpp
)

target_compile_options(gdkcollisions PRIVATE ${JFC_COLLISION_FP_FLAGS})

set_property(TARGET gdkcollisions PROPERTY CXX_EXTENSIONS OFF)

if (JFC_BUILD_TESTS)
    add_subdirectory(test)
endif()

if (JFC_BUILD_BENCH)
    add_subdirectory(bench)
endif()

if (JFC_BUILD_DOCS)
    add_subdirectory(docs)
endif()

