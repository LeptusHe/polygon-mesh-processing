#pragma once
#include <eigen/eigen>

namespace meshlib {

class FaceNormalCalculator {
public:
    Eigen::MatrixXd Calculate(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
    {
        Eigen::Vector3d invalidNormal(0, 0, 0);

        Eigen::MatrixXd N;
        N.resize(F.rows(), 3);

        const size_t fCount = F.rows();
        for (size_t f = 0; f < fCount; ++ f) {
            const Eigen::Vector3d p0 = V.row(F(f, 0));
            const Eigen::Vector3d p1 = V.row(F(f, 1));
            const Eigen::Vector3d p2 = V.row(F(f, 2));

            auto n0 = (p1 - p0).cross(p2 - p0);
            n0 = n0 / n0.norm();

            N.row(f) = n0;
        }
        return N;
    }
};

} // namespace meshlib
