# © Joseph Cameron - MIT License

find_package(Threads REQUIRED)

option(GDKCOLLISIONS_BENCH_PROFILE "Compile the benchmark's library copy with per-phase timers" ON)

add_library(gdkcollisions_bench_objects OBJECT ${GDKCOLLISIONS_SOURCES})

target_include_directories(gdkcollisions_bench_objects PUBLIC
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/impl/kernels/std/include"
    "${PROJECT_SOURCE_DIR}/impl/kernels/null/include"
    ${GDKCOLLISIONS_PRIVATE_INCLUDE_DIRECTORIES})

target_link_libraries(gdkcollisions_bench_objects PUBLIC gdk::math jfc::dispatch jfc::hash)

target_compile_features(gdkcollisions_bench_objects PUBLIC cxx_std_20)

if (GDKCOLLISIONS_BENCH_PROFILE)
    target_compile_definitions(gdkcollisions_bench_objects PUBLIC GDK_COLLISION_PROFILE)
endif()

foreach (_gdkcollisions_bench bench grid)
    add_executable(gdkcollisions_${_gdkcollisions_bench} "${CMAKE_CURRENT_LIST_DIR}/${_gdkcollisions_bench}.cpp")

    target_link_libraries(gdkcollisions_${_gdkcollisions_bench} PRIVATE gdkcollisions_bench_objects Threads::Threads)
endforeach()
