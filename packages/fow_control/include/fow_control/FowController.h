#ifndef FOW_CONTROLLER_H
#define FOW_CONTROLLER_H

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
        double max_distance_; // maximum distance at which the drones should be kept at
        Eigen::SparseMatrix<double> H; //hessian matrix of optimization problem
        Eigen::VectorXd f; // vector of optimization problem
        Eigen::Matrix<double,4,2> A;
        // instantiate the solver
        OsqpEigen::Solver solver;
        Eigen::VectorXd lowerbound;
        Eigen::VectorXd upperbound;
        bool solver_init;
        int max_robots_;
        double max_vel_;
        double min_vel_;

    public:
        FowController(double fow_angle, double min_distance, double max_distance, int max_robots_);
        ~FowController();
        int applyCbf(Eigen::Vector3d &uopt, Eigen::VectorXd &h_out, Eigen::Vector3d &ustar, Eigen::Vector3d &p_i, int n_robot, Eigen::MatrixXd &p_j);
        int applyCbfSingle(Eigen::Vector3d &uopt, Eigen::VectorXd &h_out, Eigen::Vector3d &ustar, Eigen::Vector3d &p_i, int n_robot, Eigen::MatrixXd &p_j, Eigen::MatrixXd slack);
        void setVerbose(bool verbose);
    };
}

#endif // FOW_CONTROLLER_H