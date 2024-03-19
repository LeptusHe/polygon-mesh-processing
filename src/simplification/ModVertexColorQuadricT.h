#pragma once

#include <OpenMesh/Tools/Decimater/ModBaseT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <OpenMesh/Core/Geometry/QuadricT.hh>
#include <Eigen/Eigen>
#include <iostream>
#include <fmt/format.h>

using namespace OpenMesh::Decimater;

using Quadric6x6 = std::tuple<Eigen::MatrixXd, Eigen::RowVectorXd, double>;

template <class MeshT>
class ModVertexColorQuadricT : public ModBaseT<MeshT> {
public:
    DECIMATING_MODULE(ModVertexColorQuadricT, MeshT, VertexColorQuadric);

public:
    explicit ModVertexColorQuadricT(MeshT& mesh) : Base(mesh, false)
    {
        unset_max_err();
        Base::mesh().add_property(quadrics_);
    }

    ~ModVertexColorQuadricT() override
    {
        Base::mesh().remove_property(quadrics_);
    }

    double ConvertColor(unsigned char c)
    {
        return static_cast<uint32_t>(c) * 1.0f; // / 255.0f;
    }

public:
    void initialize() override
    {
        if (!quadrics_.is_valid()) {
            Base::mesh().add_property(quadrics_);
        }

        typename Mesh::VertexIter v_it = Base::mesh().vertices_begin();
        typename Mesh::VertexIter v_end = Base::mesh().vertices_end();

        for (; v_it != v_end; ++ v_it) {
            Base::mesh().property(quadrics_, *v_it) = {Eigen::MatrixXd::Zero(6, 6), Eigen::RowVectorXd::Zero(6), 0};
        }

        typename Mesh::FaceIter f_it = Base::mesh().faces_begin();
        typename Mesh::FaceIter f_end = Base::mesh().faces_end();

        typename Mesh::FaceVertexIter fv_it;
        typename Mesh::VertexHandle vh0, vh1, vh2;

        const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);

        for (; f_it != f_end; ++ f_it) {
            fv_it = Base::mesh().fv_iter(*f_it);

            vh0 = *fv_it; ++ fv_it;
            vh1 = *fv_it; ++ fv_it;
            vh2 = *fv_it;

            auto p0 = Base::mesh().point(vh0);
            auto p1 = Base::mesh().point(vh1);
            auto p2 = Base::mesh().point(vh2);

            OpenMesh::Vec3uc c0, c1, c2;
            if (Base::mesh().has_vertex_colors()) {
                c0 = Base::mesh().color(vh0);
                c1 = Base::mesh().color(vh1);
                c2 = Base::mesh().color(vh2);
            } else {
                c0 = OpenMesh::Vec3uc(0, 0, 0);
                c1 = OpenMesh::Vec3uc(0, 0, 0);
                c2 = OpenMesh::Vec3uc(0, 0, 0);
            }

            Eigen::RowVectorXd p(6);
            p << p0[0], p0[1], p0[2], ConvertColor(c0[0]), ConvertColor(c0[1]), ConvertColor(c0[2]);

            Eigen::RowVectorXd q(6);
            q << p1[0], p1[1], p1[2], ConvertColor(c1[0]), ConvertColor(c1[1]), ConvertColor(c1[2]);

            Eigen::RowVectorXd r(6);
            r << p2[0], p2[1], p2[2], ConvertColor(c2[0]), ConvertColor(c2[1]), ConvertColor(c2[2]);

            Eigen::RowVectorXd pq = q - p;
            Eigen::RowVectorXd pr = r - p;

            double area = sqrt(pq.squaredNorm() * pr.squaredNorm() - pow(pr.dot(pq), 2));
            Eigen::RowVectorXd e1 = pq.normalized();
            Eigen::RowVectorXd e2 = (pr - e1.dot(pr) * e1).normalized();

            assert(std::abs(e1.dot(e2)) < 1e-10);

            const Eigen::MatrixXd A = I - e1.transpose() * e1 - e2.transpose() * e2;
            if (!A.fullPivLu().isInvertible()) {
#if DEBUG_LOG
                std::cout << "is not invertible" << std::endl;
                fmt::format("p0=[{}, {}, {}], p1=[{}, {}, {}]", p0[0], p0[1], p0[2], p1[0], p1[1], p1[2]);
                std::cout << "matrix: \n" << A << std::endl;
#endif
            } else {
#if DEBUG_LOG
                std::cout << "invertable mat: \n" << A << std::endl;
#endif
            }

            const Eigen::RowVectorXd b = p.dot(e1) * e1 + p.dot(e2) * e2 - p;
            const double c = (p.dot(p) - pow(p.dot(e1),2) - pow(p.dot(e2),2));

            Quadric6x6 quadric = {area * A, area * b, area * c};

            Base::mesh().property(quadrics_, vh0) += quadric;
            Base::mesh().property(quadrics_, vh1) += quadric;
            Base::mesh().property(quadrics_, vh2) += quadric;
        }
    }

    float collapse_priority(const CollapseInfo& _ci) override
    {
        Quadric6x6 q = Base::mesh().property(quadrics_, _ci.v0);
        q += Base::mesh().property(quadrics_, _ci.v1);

        double err = 0.0;

        const auto& A = std::get<0>(q);
        const auto& b = std::get<1>(q);
        const auto& c = std::get<2>(q);

        if (!A.fullPivLu().isInvertible()) {
#if DEBUG_LOG
            std::cout << "invertible" << std::endl;
#endif
            return Base::ILLEGAL_COLLAPSE;
        } else {
            Eigen::RowVectorXd p = -b * A.inverse();
            err = p.dot(p * A) + 2 * p.dot(b) + c;

            auto pos = Mesh::Point(p[0],  p[1], p[2]);

            if (_ci.v0.idx() == 5257 && _ci.v1.idx() == 5258) {
                std::cout << "out" << std::endl;
            }

            if (flip_normal(Base::mesh(), _ci, _ci.v0, pos)) {
                //std::cout << "flip normal" << std::endl;
                return Base::ILLEGAL_COLLAPSE;
            }

            if (flip_normal(Base::mesh(), _ci, _ci.v1, pos)) {
                //std::cout << "flip normal" << std::endl;
                return Base::ILLEGAL_COLLAPSE;
            }

            return static_cast<float>(err < max_err_) ? err : static_cast<float>(Base::ILLEGAL_COLLAPSE);
        }
    }

    /*
    bool flip_normal(const Mesh::VertexHandle& v0)
    {
        auto& mesh = Base::mesh();
    }
    */
    bool flip_normal(const MeshT& mesh, const CollapseInfo& ci, const typename MeshT::VertexHandle& vh, const typename MeshT::Point& p)
    {
        auto p0 = mesh.point(vh);
        for (const auto fh : mesh.vf_range(vh)) {
            if (fh == ci.fl || fh == ci.fr)
                continue;

            typename MeshT::HalfedgeHandle e;
            for (const auto eh : mesh.fh_range(fh)) {
                if (eh.from() == vh) {
                    e = eh;
                    break;
                }
            }
            auto v1 = mesh.to_vertex_handle(e);
            auto p1 = mesh.point(v1);
            auto prev_e = mesh.prev_halfedge_handle(e);
            auto v2 = mesh.from_vertex_handle(prev_e);
            auto p2 = mesh.point(v2);

            auto n1 = cross(p2 - p1, p0 - p1);
            auto n2 = cross(p2 - p1, p - p1);

            if (dot(n1, n2) < 0)
                return true;
        }
        return false;
    }

    void postprocess_collapse(const CollapseInfo& _ci) override
    {
        Quadric6x6 q = Base::mesh().property(quadrics_, _ci.v0);
        q += Base::mesh().property(quadrics_, _ci.v1);

        const auto& A = std::get<0>(q);
        const auto& b = std::get<1>(q);
        const auto& c = std::get<2>(q);

        if (!A.fullPivLu().isInvertible()) {
            std::cout << "invertible" << std::endl;
        } else {
            if (Base::mesh().status(_ci.v1).locked()) {
                std::cout << "is locked" << std::endl;
            } else {
                Eigen::RowVectorXd p = -b * A.inverse();

                auto old_p0 = Base::mesh().point(_ci.v0);
                auto old_p1 = Base::mesh().point(_ci.v1);

                auto point = Mesh::Point(p[0],  p[1], p[2]);
                Base::mesh().set_point(_ci.v1, point);

                for (auto fh : Base::mesh().vf_range(_ci.v1)) {
                    auto normal = Base::mesh().normal(fh);
                    if (normal[1] < 0) {
                        //std::cout << fmt::format("v0={}, v1={}", _ci.v0.idx(), _ci.v1.idx());

                        normal = Base::mesh().normal(fh);
                        for (auto vh : Base::mesh().fv_range(fh)) {
                            //std::cout << "vertex: " << Base::mesh().point(vh) << std::endl;
                        }

                        normal = Base::mesh().calc_face_normal(fh);
                        //std::cout << "normal: " << normal << std::endl;
                    }
                }


                auto color = Mesh::Color(static_cast<int>(p[3]), static_cast<int>(p[4]), static_cast<int>(p[5]));
                Base::mesh().set_color(_ci.v1, color);
            }
        }

        Base::mesh().property(quadrics_, _ci.v1) += Base::mesh().property(quadrics_, _ci.v0);
    }

    void set_error_tolenrance_factor(double _factor)
    {
        if (this->is_binary()) {
            if (_factor >= 0.0 && _factor <= 1.0) {
                // the smaller the factor, the smaller max_err_ gets
                // thus creating a stricter constraint
                // division by error_tolerance_factor_ is for normalization
                double max_err = max_err_ * _factor / this->error_tolerance_factor_;
                set_max_err(max_err);
                this->error_tolerance_factor_ = _factor;

                initialize();
            }
        }
    }

public: // specific methods

    /** Set maximum quadric error constraint and enable binary mode.
     *  \param _err    Maximum error allowed
     *  \param _binary Let the module work in non-binary mode in spite of the
     *                 enabled constraint.
     *  \see unset_max_err()
     */
    void set_max_err(double _err, bool _binary=true)
    {
        max_err_ = _err;
        Base::set_binary(_binary);
    }

    /// Unset maximum quadric error constraint and restore non-binary mode.
    /// \see set_max_err()
    void unset_max_err()
    {
        max_err_ = DBL_MAX;
        Base::set_binary(false);
    }

    /// Return value of max. allowed error.
    [[nodiscard]] double max_err() const { return max_err_; }

private:
    double max_err_ = DBL_MAX;
    OpenMesh::VPropHandleT<Quadric6x6> quadrics_;
};


inline Quadric6x6& operator+=(Quadric6x6& lhs, const Quadric6x6& rhs)
{
    std::get<0>(lhs) += std::get<0>(rhs);
    std::get<1>(lhs) += std::get<1>(rhs);
    std::get<2>(lhs) += std::get<2>(rhs);

    return lhs;
}