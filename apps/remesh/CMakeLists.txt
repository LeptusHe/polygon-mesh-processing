add_executable(11-remesh main.cc)
set_target_properties(11-remesh PROPERTIES OUTPUT_NAME "11-remesh")
target_include_directories(11-remesh PRIVATE src)
target_link_libraries(11-remesh PRIVATE meshlib)
