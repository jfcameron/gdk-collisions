# © Joseph Cameron - MIT License

find_package(Threads REQUIRED)

jfc_add_tests(
    C++_STANDARD 20

    TEST_SOURCE_FILES
        "${CMAKE_CURRENT_LIST_DIR}/std_grid_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/collision_overlap_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/collision_parallel_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rigid_transform_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_events_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_kinematic_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_layer_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_motion_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_query_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_response_handler_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_rotation_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_static_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_surface_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/scene_trigger_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/std_conformance.cpp"

    LIBRARIES
        gdkcollisions_implementation
        gdkcollisions_kernel_conformance
        Threads::Threads)

set_property(TARGET ${JFC_TEST_TARGET} PROPERTY CXX_EXTENSIONS OFF)

jfc_add_tests(
    C++_STANDARD 20

    TEST_SOURCE_FILES
        "${CMAKE_CURRENT_LIST_DIR}/null_conformance.cpp"

    LIBRARIES
        gdkcollisions
        gdkcollisions_kernel_conformance)

set_property(TARGET ${JFC_TEST_TARGET} PROPERTY CXX_EXTENSIONS OFF)

if (NOT CMAKE_CROSSCOMPILING)
    add_test(NAME gdk-collisions_install_and_consume
        COMMAND "${CMAKE_COMMAND}"
            "-DBUILD_DIR=${PROJECT_BINARY_DIR}"
            "-DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/install_and_consume"
            "-DCONFIG=$<CONFIG>"
            "-DGENERATOR=${CMAKE_GENERATOR}"
            "-DPLATFORM=${CMAKE_GENERATOR_PLATFORM}"
            "-DTOOLSET=${CMAKE_GENERATOR_TOOLSET}"
            "-DCXX_COMPILER=${CMAKE_CXX_COMPILER}"
            "-DCXX_FLAGS=${CMAKE_CXX_FLAGS}"
            "-DEXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"
            -P "${CMAKE_CURRENT_LIST_DIR}/install/install_and_consume.cmake")
endif()
