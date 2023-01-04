add_rules("mode.debug", "mode.release")

add_cxxflags("/bigobj")

enable_debug = is_mode("debug")
print("enable debug:", enable_debug)

add_requires("libigl", {configs = {imgui=true}, debug=enable_debug})
add_requires("fmt", {debug=enable_debug, header_only=true, system=false})
add_requires("glad", {debug=enable_debug, system=false})
add_requires("eigen", {debug=enable_debug, system=false})

target("meshlib")
    set_languages("c++17")
    set_kind("static")
    add_files("src/property/*.cc")
    add_packages("libigl", "glad", "eigen", "fmt")

target("01-geometry-property-visualization")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/geom-prop-vis/*.cc")
    add_packages("libigl", "glad", "eigen", "fmt")
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs("src/")
    add_deps("meshlib")