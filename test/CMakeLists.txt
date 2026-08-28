# © Joseph Cameron - All Rights Reserved

jfc_add_tests(
    C++_STANDARD 20
    C_STANDARD 90

    TEST_SOURCE_FILES
        "${CMAKE_CURRENT_LIST_DIR}/broadphase_grid_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/collision_overlap_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/collision_parallel_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/rigid_transform_test.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/collision_scene_test.cpp"

    INCLUDE_DIRECTORIES
        "${CMAKE_CURRENT_LIST_DIR}/../include"
        "${CMAKE_CURRENT_LIST_DIR}/../impl/include"
        "${gdkmath_INCLUDE_DIRECTORIES}"

    LIBRARIES
        "${gdkcollisions_LIBRARIES}"
        "${gdkmath_LIBRARIES}")

target_compile_options(${PROJECT_NAME} PRIVATE ${JFC_COLLISION_FP_FLAGS})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)

