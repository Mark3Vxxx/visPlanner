#ifndef _TYPE_CONVERTER_HPP_
#define _TYPE_CONVERTER_HPP_

#include <Eigen/Eigen>
#include <armadillo>

namespace type_converter {
    // Eigen Vector3d -> Armadillo vec3
    inline arma::vec3 eigenToArma(const Eigen::Vector3d& v) {
        return arma::vec3({v.x(), v.y(), v.z()});
    }

    // Armadillo vec3 -> Eigen Vector3d
    inline Eigen::Vector3d armaToEigen(const arma::vec3& v) {
        return Eigen::Vector3d(v(0), v(1), v(2));
    }

    // Eigen MatrixXd -> Armadillo mat (用于中间点矩阵)
    inline arma::mat eigenToArmaMat(const Eigen::MatrixXd& m) {
        arma::mat am(m.rows(), m.cols());
        for (int i = 0; i < m.rows(); ++i)
            for (int j = 0; j < m.cols(); ++j)
                am(i, j) = m(i, j);
        return am;
    }
}

#endif