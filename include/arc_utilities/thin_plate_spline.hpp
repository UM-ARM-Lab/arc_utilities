#ifndef THIN_PLATE_SPLINE_HPP
#define THIN_PLATE_SPLINE_HPP

#include <Eigen/Dense>
#include <Eigen/QR>
#include "arc_utilities/arc_exceptions.hpp"
#include "arc_utilities/eigen_helpers.hpp"

namespace arc_utilities
{
    // Note that DIMENSIONS must be a postive integer, not something like `Eigen::Dynamic`
    template<int DIMENSIONS = 3>
    class ThinPlateSpline
    {
    public:
        typedef Eigen::Matrix<double, DIMENSIONS,     Eigen::Dynamic> PointSet;
        typedef Eigen::Matrix<double, DIMENSIONS + 1, Eigen::Dynamic> HomogeneousPointSet;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ThinPlateSpline()
            : solved_(false)
        {}

        /**
         * @brief ThinPlateSpline::ThinPlateSpline An arbitrary dimensional thin
         *  plate spline for interpolation between points.
         * @param template_points A (DIMINESION x NUMPOINTS) matrix which define
         *  the starting points that control the warping. I.e. the template is
         *  warped to the target
         * @param target_points A (DIMINESION x NUMPOINTS) matrix which define
         *  the target points that control the warping. I.e. the template is
         *  warped to the target
         */
        template <typename DerivedA, typename DerivedB>
        ThinPlateSpline(
                const Eigen::MatrixBase<DerivedA>& template_points,
                const Eigen::MatrixBase<DerivedB>& target_points)
            : solved_(false)
        {
            setTemplatePoints(template_points);
            setTargetPoints(target_points);
            solveForCoeffs();
        }

        /**
         * @note Checking for the vailidty of the input data is put off until
         *  the warping function is solved
         */
        template <typename Derived>
        void setTemplatePoints(
                const Eigen::MatrixBase<Derived>& template_points)
        {
            using namespace Eigen;
            static_assert(MatrixBase<Derived>::RowsAtCompileTime == DIMENSIONS ||
                          MatrixBase<Derived>::RowsAtCompileTime == Dynamic,
                          "INVALID DATA DIMENSIONS FOR template_points");

            homogeneous_template_points_.resize(DIMENSIONS + 1, template_points.cols());
            homogeneous_template_points_ << template_points, RowVectorXd::Ones(template_points.cols());
            solved_ = false;
        }

        /**
         * @note Checking for the vailidty of the input data is put off until
         *  the warping function is solved
         */
        template <typename Derived>
        void setTargetPoints(
                const Eigen::MatrixBase<Derived>& target_points)
        {
            using namespace Eigen;
            static_assert(MatrixBase<Derived>::RowsAtCompileTime == DIMENSIONS ||
                          MatrixBase<Derived>::RowsAtCompileTime == Dynamic,
                          "INVALID DATA DIMENSIONS FOR target_points");

            target_points_ = target_points;
            solved_ = false;
        }

        void solveForCoeffs()
        {
            using namespace Eigen;
            if (homogeneous_template_points_.cols() != target_points_.cols())
            {
                throw_arc_exception(
                            std::invalid_argument,
                            "template and target points must have the same number of points");
            }

            const auto NUMPOINTS = homogeneous_template_points_.cols();

            // Calculate the kernel matrix for the template (control) points
            const MatrixXd Rsquared =
                    EigenHelpers::CalculateSquaredDistanceMatrix(homogeneous_template_points_);
            const MatrixXd kernel = Rsquared.unaryExpr(&ThinPlateSpline::KernelFunction);

            // Form the system:   A * params = B
            //       A = [K,       template'
            //            template,    0]
            //       B = [target'
            //              0    ]
            MatrixXd A(NUMPOINTS + DIMENSIONS + 1, NUMPOINTS + DIMENSIONS + 1);
            A << kernel,                       homogeneous_template_points_.transpose(),
                 homogeneous_template_points_, MatrixXd::Zero(DIMENSIONS + 1, DIMENSIONS + 1);
            MatrixXd B(NUMPOINTS + DIMENSIONS + 1, DIMENSIONS);
            B << target_points_.transpose(),
                 MatrixXd::Zero(DIMENSIONS + 1, DIMENSIONS);

            // params = A \ B
            const Matrix<double, Dynamic, DIMENSIONS> params =
                    A.colPivHouseholderQr().solve(B).eval();
            warping_coeffs_ = params.topRows(NUMPOINTS).transpose();
            affine_coeffs_ = params.bottomRows(DIMENSIONS + 1).transpose();
            solved_ = true;
        }

        template<typename Derived>
        Eigen::Matrix<double, DIMENSIONS, Eigen::Dynamic> interpolate(
                const Eigen::MatrixBase<Derived>& interpolation_points)
        {
            using namespace Eigen;
            static_assert(MatrixBase<Derived>::ColsAtCompileTime == DIMENSIONS ||
                          MatrixBase<Derived>::ColsAtCompileTime == Dynamic,
                          "INVALID DATA DIMENSIONS FOR interpolation_points");

            if (!solved_)
            {
                solveForCoeffs();
            }

            HomogeneousPointSet homogeneous_interpolation_points;
            homogeneous_interpolation_points.resize(DIMENSIONS + 1, interpolation_points.cols());
            homogeneous_interpolation_points << interpolation_points,
                                                RowVectorXd::Ones(interpolation_points.cols());

            // Calculate the kenerl matrix comparing the new points to the template (control) points
            const MatrixXd Rsquared = EigenHelpers::SquaredDistancesBetweenPointSets(
                        homogeneous_template_points_, homogeneous_interpolation_points);
            const MatrixXd kernel = Rsquared.unaryExpr(&ThinPlateSpline::KernelFunction);

            return affine_coeffs_ * homogeneous_interpolation_points + warping_coeffs_ * kernel;

        }

    protected:
        static double KernelFunction(const double r_sq)
        {
            if (r_sq == 0)
            {
                return 0;
            }
            else
            {
                return r_sq * std::log(std::sqrt(r_sq));
            }
        }

        HomogeneousPointSet homogeneous_template_points_;
        PointSet target_points_;
        Eigen::Matrix<double, DIMENSIONS, DIMENSIONS + 1> affine_coeffs_;
        PointSet warping_coeffs_;
        bool solved_;
    };
}

#endif // THIN_PLATE_SPLINE_HPP
