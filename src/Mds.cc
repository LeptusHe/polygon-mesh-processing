#include "Mds.h"
#include <cassert>
#include <iostream>

Eigen::MatrixXd mds::Compute(const Eigen::MatrixXd& D, unsigned int dim)
{
    assert(D.rows() == D.cols());
    assert(D.rows() >= dim);

    const unsigned n = D.rows();
    const Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n, n) - (1.0 / static_cast<double>(n)) * Eigen::VectorXd::Ones(n) * Eigen::VectorXd::Ones(n).transpose();

    std::cout << H << std::endl;

    const Eigen::MatrixXd K = - 0.5 * H * D * H;
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(K);
    //Eigen::EigenSolver<Eigen::MatrixXd> solver(K);
    if (solver.info() != Eigen::Success) {
        std::cout << "EigenSolver failed" << std::endl;
        return Eigen::MatrixXd();
    }

    Eigen::VectorXd S = solver.eigenvalues().real();
    Eigen::MatrixXd V = solver.eigenvectors().real();

    std::cout << "S: " << S << std::endl;
    std::cout << "V: " << V << std::endl;

    ExtractNLargestEigens(dim, S, V);

    std::cout << "V: " << V << std::endl;
    std::cout << "S: " << S << std::endl;

    std::cout << " S sise" << std::endl;
    auto t = S.cwiseSqrt();
    std::cout << t << std::endl;

    const Eigen::MatrixXd X = Eigen::DiagonalMatrix<double, Eigen::Dynamic>(S.cwiseSqrt()) * V.transpose();

    std::cout << "X: " << X << std::endl;
    return X;
}


void mds::ExtractNLargestEigens(unsigned int n, Eigen::VectorXd& S, Eigen::MatrixXd& V)
{
    // Note: m is the original dimension
    const unsigned m = S.rows();

    // Copy the original matrix
    const Eigen::MatrixXd origV = V;

    // Sort by eigenvalue
    constexpr double epsilon = 1e-16;
    std::vector<std::pair<double, unsigned>> sortS(m);
    for (unsigned i = 0; i < m; ++ i) sortS[i] = std::make_pair(std::max(S(i), epsilon), i);
    std::partial_sort(sortS.begin(), sortS.begin() + n, sortS.end(), std::greater<std::pair<double, unsigned>>());

    // Resize matrices
    S.resize(n);
    V.resize(m, n);

    // Set values
    for (unsigned i = 0; i < n; ++ i)
    {
        S(i)     = sortS[i].first;
        V.col(i) = origV.col(sortS[i].second);
    }
}