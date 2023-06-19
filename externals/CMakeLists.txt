set(LIBIGL_BUILD_TESTS OFF CACHE BOOL "disable libigl test")
set(LIBIGL_BUILD_TUTORIALS OFF CACHE BOOL "disable libigl tutorials")
set(LIBIGL_INSTALL OFF CACHE BOOL "disable libigl install")

set(LIBIGL_USE_STATIC_LIBRARY ON CACHE BOOL "use static library")

set(LIBIGL_EMBREE OFF CACHE BOOL "disable embree")
set(LIBIGL_GLFW ON CACHE BOOL "enable glfw")
set(LIBIGL_IMGUI ON CACHE BOOL "enable imgui")
set(LIBIGL_OPENGL ON CACHE BOOL "enable opengl")
set(LIBIGL_PNG OFF CACHE BOOL "disable png")
set(LIBIGL_PREDICATES OFF CACHE BOOL "disable predicates")
set(LIBIGL_XML OFF CACHE BOOL "disable xml")

add_subdirectory(libigl)


set(third_party_include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/libigl/include PARENT_SCOPE)