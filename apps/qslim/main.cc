#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <fmt/format.h>
#include <igl/qslim.h>
#include <igl/qslim_optimal_collapse_edge_callbacks.h>

/*
int main(int argc, char * argv[])
{
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    cout<<"Usage: ./703_Decimation_bin [filename.(off|obj|ply)]"<<endl;
    cout<<"  [space]  toggle animation."<<endl;
    cout<<"  'r'  reset."<<endl;

    MatrixXd V,OV;
    MatrixXi F,OF;

    auto path = argc > 1 ? argv[1] : "data/bunny.off";
    if (!igl::read_triangle_mesh(path, OV, OF)) {
        std::cout << fmt::format("failed to load mesh: [{0}]", path);
        return -1;
    } else {
        std::cout << fmt::format("vertex count: {}\n", V.rows());
        std::cout << fmt::format("face count: {}\n", F.rows());
    }
    return found;
}

int main(int argc, char *argv[])
{
    auto path = argc > 1 ? argv[1] : "data/beetle.obj";

    Mesh mesh;
    OpenMesh::IO::Options opt;
    if (!OpenMesh::IO::read_mesh(mesh, path, opt)) {
        std::cerr << "failed to load mesh from " << path << std::endl;
        return 1;
    } else {
        std::cout << fmt::format("succeed to load mesh from [{0}]", path) << std::endl;
        PrintMeshInfo(mesh);
    }

    vertex_merger vertex_merger;
    auto merged_mesh = vertex_merger.Merge(mesh);
    merged_mesh.garbage_collection();
    PrintMeshInfo(merged_mesh);
    OpenMesh::IO::write_mesh(merged_mesh, "data/merge-terrain-grid3.obj");
    return 0;

    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);
    Eigen::MatrixXi F = Eigen::MatrixXi::Zero(mesh.n_faces(), 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(mesh.n_faces(), 3);

    for (auto fh : mesh.faces()) {
        if (mesh.is_boundary(fh)) {
            C.row(fh.idx()) = Eigen::Vector3d(1, 0, 0);
        } else {
            C.row(fh.idx()) = Eigen::Vector3d(0.5, 0.5f, 0.5f);
        }
    }

    PriorityQueue queue;
    for (auto edgeHandle : mesh.edges()) {
        if (!ValidToCollapse(mesh, edgeHandle)) {
            continue;
        }

        auto length = mesh.calc_edge_length(edgeHandle);
        queue.push(std::make_pair(length, edgeHandle));
    }


    igl::opengl::glfw::Viewer viewer;

    // Prepare array-based edge data structures and priority queue
    VectorXi EMAP;
    MatrixXi E,EF,EI;
    igl::min_heap< std::tuple<double,int,int> > Q;
    Eigen::VectorXi EQ;
    // If an edge were collapsed, we'd collapse it to these points:
    MatrixXd C;
    int num_collapsed;

    // Function to reset original mesh and data structures
    const auto & reset = [&]()
    {
        F = OF;
        V = OV;
        edge_flaps(F,E,EMAP,EF,EI);
        C.resize(E.rows(),V.cols());
        VectorXd costs(E.rows());
        // https://stackoverflow.com/questions/2852140/priority-queue-clear-method
        // Q.clear();
        Q = {};
        EQ = Eigen::VectorXi::Zero(E.rows());
        {
            Eigen::VectorXd costs(E.rows());
            igl::parallel_for(E.rows(),[&](const int e)
            {
                double cost = e;
                RowVectorXd p(1,3);
                shortest_edge_and_midpoint(e,V,F,E,EMAP,EF,EI,cost,p);
                C.row(e) = p;
                costs(e) = cost;
            },10000);
            for(int e = 0;e<E.rows();e++)
            {
                Q.emplace(costs(e),e,0);
            }
        }

        if (ImGui::Button("Merge Vertex")) {
            //vertex_merger vertex_merger;
            std::cout << "n_face: " << mesh.n_faces() << " vertex: " << mesh.n_vertices() << std::endl;
            mesh = vertex_merger.Merge(mesh);
            std::cout << "n_face: " << mesh.n_faces() << " vertex: " << mesh.n_vertices() << std::endl;
        }

        if (ImGui::Button("Color Boundary Vertex")) {
            mesh.request_vertex_colors();
            meshlib::ColorBoundaryVertex(mesh);
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        if (ImGui::Button("Find Boundary Edge")) {
            FindBoundaryEdge(mesh, viewer);
        }

        if (ImGui::Button("Find Collinear Boundary Edge")) {
            FindCollinearBoundaryEdge(mesh, viewer);
        }

        if (ImGui::Button("Collapse Boundary")) {
            CollapseBoundaryEdge(mesh);
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        if (ImGui::Button("Continue To Collapse Boundary")) {
            int count = 0;
            bool found = true;
            do {
                found = CollapseBoundaryEdge(mesh);
                count += 1;
            } while (found);

            std::cout << "found: " << count << std::endl;
            meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);
        }

        ImGui::Checkbox("Continue To Collapse", &continueToCollapse);

        ImGui::InputInt("Face Count", &faceCount);

        if (ImGui::Button("Collapse Edge")) {
            Mesh::VertexHandle vh;
            for (int i = 0; i < faceCount; ++ i) {
                vh = CollapseEdge(mesh, queue);
            }
            if (vh.is_valid()) {
                meshlib::MeshUtils::ConvertMeshToViewer(mesh, viewer);

                auto item = queue.top();
                if (!item.second.is_valid()) {
                    return;
                }

                C = Eigen::VectorXd::Zero(mesh.n_vertices(), 3);
                for (auto vertexHandle : mesh.vertices()) {
                    if (vertexHandle != vh) {
                        C.row(vertexHandle.idx()) = Eigen::Vector3d(0.2f, 0.2f, 0.2f);
                    } else {
                        std::cout << "set color to red" << std::endl;
                        C.row(vertexHandle.idx()) = Eigen::Vector3d(1, 0, 0);
                    }
                }
                std::cout << "face cnt: " << mesh.n_faces() << std::endl;
                viewer.data().set_colors(C);
            }
        }
        num_collapsed = 0;
        viewer.data().clear();
        viewer.data().set_mesh(V,F);
        viewer.data().set_face_based(true);
    };

    const auto &pre_draw = [&](igl::opengl::glfw::Viewer & viewer)->bool
    {
        // If animating then collapse 10% of edges
        if(viewer.core().is_animating && !Q.empty())
        {
            bool something_collapsed = false;
            // collapse edge
            const int max_iter = std::ceil(0.01*Q.size());
            for(int j = 0;j<max_iter;j++)
            {
                if(!collapse_edge(shortest_edge_and_midpoint,V,F,E,EMAP,EF,EI,Q,EQ,C))
                {
                    break;
                }
                something_collapsed = true;
                num_collapsed++;
            }

            if(something_collapsed)
            {
                viewer.data().clear();
                viewer.data().set_mesh(V,F);
                viewer.data().set_face_based(true);
            }
        }
        return false;
    };

    const auto &key_down =
            [&](igl::opengl::glfw::Viewer &viewer,unsigned char key,int mod)->bool
            {
                switch(key)
                {
                    case ' ':
                        viewer.core().is_animating ^= 1;
                        break;
                    case 'R':
                    case 'r':
                        reset();
                        break;
                    default:
                        return false;
                }
                return true;
            };

    reset();
    viewer.core().background_color.setConstant(0.5f);
    viewer.core().is_animating = true;
    viewer.callback_key_down = key_down;
    viewer.callback_pre_draw = pre_draw;
    return viewer.launch();
}
*/