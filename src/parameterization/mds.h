#pragma once

#include <Eigen/Eigen>

class mds {
public:
    Eigen::MatrixXd Compute(const Eigen::MatrixXd& D, unsigned int dim);

private:
    void ExtractNLargestEigens(unsigned n, Eigen::VectorXd& S, Eigen::MatrixXd& V);
};


