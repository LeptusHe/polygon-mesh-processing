find_package(openmesh REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(CGAL CONFIG REQUIRED)

add_library(meshlib STATIC
    parameterization/mds.cc
    utils/mesh_io.cc
    utils/mesh_utils.cc
    remesh/remesher.cpp
    visualization/color_set_generator.cc)

set_target_properties(meshlib PROPERTIES OUTPUT_NAME "meshlib")
target_compile_definitions(meshlib PUBLIC _USE_MATH_DEFINES)

target_include_directories(meshlib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${third_party_include_dirs}
    ${OPENMESH_INCLUDE_DIRS})

target_link_libraries(meshlib PUBLIC
    ${OPENMESH_LIBRARIES}
    CGAL::CGAL
    igl::core
    igl::opengl
    igl::glfw
    igl::imgui
    fmt::fmt
    Eigen3::Eigen
    spdlog::spdlog)