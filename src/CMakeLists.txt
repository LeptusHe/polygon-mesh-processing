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
    segmentation/kmeans.cc
    visualization/color_set_generator.cc
    utils/vertex_merger.cc
    utils/vertex_merger.h
    simplification/simplification.cc
    simplification/simplification.h
    simplification/ModVertexColorQuadricT.cc
    simplification/ModVertexColorQuadricT.h
    visualization/mesh_vis.cc
    repair/repair.cc
    filesys/filesys.cc
    ../apps/mesh-segmentation/bounds.h)

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
    fmt::fmt
    Eigen3::Eigen
    spdlog::spdlog)
