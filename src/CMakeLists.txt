find_package(openmesh REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

add_library(meshlib STATIC
    mds.cc
    mesh_io.cc
    mesh_utils.cc
    color_set_generator.cc)

set_target_properties(meshlib PROPERTIES OUTPUT_NAME "meshlib")
target_compile_definitions(meshlib PUBLIC _USE_MATH_DEFINES)

target_include_directories(meshlib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${third_party_include_dirs}
    ${OPENMESH_INCLUDE_DIRS})

target_link_libraries(meshlib PUBLIC
    ${OPENMESH_LIBRARIES}
    igl::core
    igl::opengl
    igl::glfw
    igl::imgui
    fmt::fmt
    Eigen3::Eigen
    spdlog::spdlog)
