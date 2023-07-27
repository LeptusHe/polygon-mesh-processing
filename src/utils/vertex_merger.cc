#include "vertex_merger.h"

void vertex_merger::Merge(Mesh& mesh, float distance_threshold)
{
    OpenMesh::VPropHandleT<bool> processed;
    mesh.add_property(processed, "processed");

    for (const auto& vh1 : mesh.vertices()) {
        if (!mesh.property(processed, vh1)) {
            for (const auto& vh2 : mesh.vertices()) {
                if (vh1 != vh2 && !mesh.property(processed, vh2)) {
                    Mesh::Point p1 = mesh.point(vh1);
                    Mesh::Point p2 = mesh.point(vh2);

                    double distance = (p1 - p2).norm();
                    if (distance <= distance_threshold) {
                        // 合并位置距离接近的顶点
                        mesh.property(processed, vh2) = true;
                        mesh.collapse_vertex(vh2, vh1);
                    }
                }
            }
        }
    }

    // 删除已处理的顶点属性
    mesh.remove_property(processed);

    // 执行垃圾回收
    mesh.garbage_collection();
}