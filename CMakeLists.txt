cmake_minimum_required(VERSION 3.25)
project(polygon-mesh-processing LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_DIR)
set(BUILD_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/${BUILD_TYPE_DIR})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BUILD_OUTPUT_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BUILD_OUTPUT_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BUILD_OUTPUT_DIR}/bin)


add_subdirectory(externals)
message(STATUS "include dirs of externals: ${third_party_include_dirs}")

add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tests)

add_custom_target(clean-build
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})