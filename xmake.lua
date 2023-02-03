add_rules("mode.debug")

add_cxxflags("/bigobj")

enable_debug = is_mode("debug")
print("enable debug:", enable_debug)

imgui = "imgui v1.86"

add_requires("libigl", {configs = {imgui = true}, debug=enable_debug})
add_requires(imgui, {debug=enable_debug, system=false})
add_requires("fmt", {debug=enable_debug, header_only=true, system=false})
add_requires("glad", {debug=enable_debug, system=false})
add_requires("eigen", {debug=enable_debug, system=false})
add_requires("openmesh", {debug=enable_debug, system=false})

target("meshlib")
    set_languages("c++17")
    set_kind("static")
    add_files("src/*.cc")
    add_packages("libigl", "glad", "eigen", "fmt", "openmesh")
    if (enable_debug) then
        set_runtimes("MT")
    end

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

target("02-parametrization-tutte")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/param-tutte/*.cc")
    add_packages("libigl", "glad", "eigen", "fmt")
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs("src/")
    add_deps("meshlib")

target("03-decimation")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/decimation/*.cc")
    add_packages("libigl", "glad", "eigen", "fmt")
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs("src/")
    add_deps("meshlib")

target("04-openmesh")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/openmesh/*.cc")
    add_packages("libigl", "glad", "eigen", "fmt", "openmesh")
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs("src/")
    add_deps("meshlib")

target("05-k-means")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/k-means/*.cc")
    add_packages("libigl", "glad", "eigen", "fmt", "openmesh")
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs("src/")
    add_deps("meshlib")

target("06-iterative-cluster")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/iterative-cluster/*.cc")
    add_packages("libigl", "glad", "eigen", "fmt", "openmesh", imgui)
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs("src/")
    add_deps("meshlib")