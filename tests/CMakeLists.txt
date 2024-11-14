find_package(Catch2 CONFIG REQUIRED)


add_executable(tests
    polygon-clip-test.cc
    bounds-test.cc
)


target_include_directories(tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${third_party_include_dirs}
    ../src
)


target_link_libraries(tests PRIVATE
    Catch2::Catch2WithMain
    meshlib
)


add_executable(meshlib::tests ALIAS tests)