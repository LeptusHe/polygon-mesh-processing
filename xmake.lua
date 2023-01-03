add_rules("mode.debug", "mode.release")
add_requires("libigl", {configs = {imgui=true}})
add_requires(
        "vcpkg::glad",
        "vcpkg::eigen3",
        "vcpkg::fmt")

target("01-geometry-property-visualization")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/geom-prop-vis/*.cc")
    add_packages("libigl", "vcpkg::glad", "vcpkg::eigen3", "vcpkg::fmt")