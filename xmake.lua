add_rules("mode.debug")

add_cxxflags("/bigobj")

enable_debug = is_mode("debug")
print("enable debug:", enable_debug)

libigl = "libigl-240"

include_dirs = {"src"} --, "externals/libigl/include"}

package(libigl)
    set_homepage("https://libigl.github.io/")
    set_description("Simple C++ geometry processing library.")
    set_license("MPL-2.0")
    print("install libigl v240")

    add_urls("https://github.com/libigl/libigl/archive/$(version).tar.gz",
            "https://github.com/libigl/libigl.git")
    add_versions("v2.2.0", "b336e548d718536956e2d6958a0624bc76d50be99039454072ecdc5cf1b2ede5")
    add_versions("v2.3.0", "9d6de3bdb9c1cfc61a0a0616fd96d14ef8465995600328c196fa672ee4579b70")
    add_versions("v2.4.0", "f3f53ee6f1e9a6c529378c6b0439cd2cfc0e30b2178b483fe6bea169ce6deb22")

    add_resources("2.x", "libigl_imgui", "https://github.com/libigl/libigl-imgui.git", "7e1053e750b0f4c129b046f4e455243cb7f804f3")

    add_configs("header_only", {description = "Use header only version.", default = true, type = "boolean"})
    add_configs("cgal", {description = "Use CGAL library.", default = false, type = "boolean"})
    add_configs("imgui", {description = "Use imgui with libigl.", default = false, type = "boolean"})

    if is_plat("windows") then
        add_syslinks("comdlg32")
    elseif is_plat("linux") then
        add_syslinks("pthread")
    end

    add_deps("cmake", "eigen")
    on_load("macosx", "linux", "windows", "mingw", function (package)
        if not package:config("header_only") then
            raise("Non-header-only version is not supported yet!")
        end
        if package:config("cgal") then
            package:add("deps", "cgal")
        end
        if package:config("imgui") then
            package:add("deps", "imgui v1.86", {configs = {glfw_opengl3 = true}})
        end
    end)

    on_install("macosx", "linux", "windows", "mingw", function (package)
        print("on install libigl")
        if package:config("imgui") then
            local igl_imgui_dir = package:resourcefile("libigl_imgui")
            print("igl imgui dir: ", igl_imgui_dir)
            os.cp(path.join(igl_imgui_dir, "imgui_fonts_droid_sans.h"), package:installdir("include"))
            print("copy imgui fonts")
        end
        if package:config("header_only") then
            os.cp("include/igl", package:installdir("include"))a
            return
        end
        local configs = {"-DLIBIGL_BUILD_TESTS=OFF", "-DLIBIGL_BUILD_TUTORIALS=OFF", "-DLIBIGL_SKIP_DOWNLOAD=ON"}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:debug() and "Debug" or "Release"))
        table.insert(configs, "-DBUILD_SHARED_LIBS=" .. (package:config("shared") and "ON" or"OFF"))
        if not package:config("shared") then
            table.insert(configs, "-DLIBIGL_USE_STATIC_LIBRARY=ON")
        end
        if package:is_plat("windows") then
            table.insert(configs, "-DIGL_STATIC_RUNTIME=" .. (package:config("vs_runtime"):startswith("MT") and "ON" or "OFF"))
        end
        import("package.tools.cmake").install(package, configs)
        end)

    on_test(function (package)
        assert(package:check_cxxsnippets({test = [[
            void test() {
                Eigen::MatrixXd V(4,2);
                V<<0,0,
                   1,0,
                   1,1,
                   0,1;
                Eigen::MatrixXi F(2,3);
                F<<0,1,2,
                   0,2,3;
                Eigen::SparseMatrix<double> L;
                igl::cotmatrix(V,F,L);
            }
        ]]}, {configs = {languages = "c++14"}, includes = {"igl/cotmatrix.h", "Eigen/Dense", "Eigen/Sparse"}}))
    end)

package("libiglv2402")
    add_deps("cmake")
    print("script dir:", os.scriptdir())
    set_sourcedir(path.join(os.scriptdir(), "externals/libigl"))

    add_resources("2.x", "libigl_imgui", "https://github.com/libigl/libigl-imgui.git", "7e1053e750b0f4c129b046f4e455243cb7f804f3")


    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DLIBIGL_BUILD_TESTS=OFF")
        table.insert(configs, "-DLIBIGL_BUILD_TUTORIALS=OFF")
        table.insert(configs, "-DLIBIGL_EMBREE=OFF")
        table.insert(configs, "-DLIBIGL_XML=OFF")
        table.insert(configs, "-DLIBIGL_COPYLEFT_CORE=OFF")
        table.insert(configs, "-DLIBIGL_COPYLEFT_CGAL=OFF")
        table.insert(configs, "-DLIBIGL_COPYLEFT_COMISO=OFF")
        table.insert(configs, "-DLIBIGL_COPYLEFT_TETGEN=OFF")
        table.insert(configs, "-DLIBIGL_USE_STATIC_LIBRARY=OFF")

        -- local igl_imgui_dir = package:resourcefile("libigl_imgui")
        -- print("igl_imgui_dir:", igl_imgui_dir)
        -- os.cp(path.join(igl_imgui_dir, "imgui_fonts_droid_sans.h"), package:installdir("include"))
        -- os.cp(path.join(igl_imgui_dir, "backends/*.h"), package:installdir("backends"))
        print("copy headers")

        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:debug() and "Debug" or "Release"))
        table.insert(configs, "-DBUILD_SHARED_LIBS=" .. (package:config("shared") and "ON" or "OFF"))

        import("package.tools.cmake").install(package, configs)
    end)
package_end()


imgui = "imgui 1.85"
fmt = "fmt"
glad = "glad"
eigen = "eigen"
openmesh = "openmesh"
glfw = "glfw"
polyscope = "polyscope"

add_requires(libigl, {configs={imgui=true}, debug=enable_debug})
--add_requires(imgui, {configs={glfw_opengl3=true}, debug=enable_debug, system=false})
add_requires(fmt, {configs={header_only=true}, debug=enable_debug, system=false})
add_requires(glad, {debug=enable_debug, system=false})
add_requires(eigen, {debug=enable_debug, system=false})
add_requires(openmesh, {debug=enable_debug, system=false})
add_requires(glfw, {debug=enable_debug, system=false})
add_requires(polyscope, {debug=enable_debug, system=false})

packages = {fmt, glad, eigen, openmesh, glfw, polyscope}

target("meshlib")
    set_languages("c++17")
    set_kind("static")
    add_files("src/*.cc")
    add_packages(packages)
    if (enable_debug) then
        set_runtimes("MT")
    end

target("01-geometry-property-visualization")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/geom-prop-vis/*.cc")
    add_packages(packages)
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs(include_dirs)
    add_deps("meshlib")

target("02-parametrization-tutte")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/param-tutte/*.cc")
    add_packages(packages)
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs(include_dirs)
    add_deps("meshlib")

target("03-decimation")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/decimation/*.cc")
    add_packages(packages)
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs(include_dirs)
    add_deps("meshlib")

target("04-openmesh")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/openmesh/*.cc")
    add_packages(packages)
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs(include_dirs)
    add_deps("meshlib")

target("05-k-means")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/k-means/*.cc")
    add_packages(packages)
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs(include_dirs)
    add_deps("meshlib")

target("06-iterative-cluster")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/iterative-cluster/*.cc")
    add_packages(packages)
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs(include_dirs)
    add_deps("meshlib")

target("07-multi-dimension-scaling")
    set_languages("c++17")
    set_kind("binary")
    add_files("apps/multi-dimension-scaling/*.cc")
    add_packages(packages)
    add_packages(imgui)
    if (enable_debug) then
        set_runtimes("MT")
    end
    add_includedirs(include_dirs)
    add_deps("meshlib")