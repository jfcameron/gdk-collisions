# © Joseph Cameron - MIT License

if (NOT TARGET gdkmath)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/gdk-math")
endif()

if (NOT TARGET jfc::dispatch)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/jfc-dispatch")
endif()

if (NOT TARGET jfc::hash)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/jfc-hash")
endif()
