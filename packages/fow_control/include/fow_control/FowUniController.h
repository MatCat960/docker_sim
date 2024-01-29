#ifndef FOW_UNI_CONTROLLER_H
#define FOW_UNI_CONTROLLER_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>



namespace fow_control
{
    class FowController
    {
    private:
        double fow_angle_; //complete field of view angle of the camera
        double min_distance_; // minimum distance at which the drones should be kept at
        Eigen::SparseMatrix<double> H; //hessian matrix of optimization problem
        Eigen::VectorXd f; // vector of optimization problem
        Eigen::Matrix<double,3,2> A;
        // instantiate the solver
        OsqpEigen::Solver solver;
        Eigen::VectorXd lowerbound;
        bool solver_init;

    public:
        FowController(double fow_angle, double min_distance, int n_robots);
        ~FowController();
        int applyCbf(Eigen::Vector3d &uopt, Eigen::Vector3d &ustar, Eigen::Vector3d &p_i, int n_robot, Eigen::MatrixXd &p_j);
    };
}

#endif // FOW_UNI_CONTROLLER_H