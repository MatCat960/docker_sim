#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>



namespace safety_control                // fow_control
{
    class SafetyController              // FowController
    {
    private:
        // double fow_angle_; //complete field of view angle of the camera
        double min_distance_; // minimum distance at which the drones should be kept at
        double max_distance_;
        Eigen::SparseMatrix<double> H; //hessian matrix of optimization problem
        Eigen::VectorXd f; // vector of optimization problem
        // Eigen::Matrix<double,1,2> A;
        // instantiate the solver
        OsqpEigen::Solver solver;
        Eigen::VectorXd lowerbound;
        Eigen::VectorXd upperbound;
        bool solver_init;
        int num_robots_;

    public:
        SafetyController(double min_distance, double max_distance, int num_robots_);
        ~SafetyController();
        int applyCbf(Eigen::Vector3d &uopt, Eigen::VectorXd &h_out, Eigen::Vector3d &ustar, Eigen::Vector3d &p_i, Eigen::MatrixXd &p_j);
        int applyCbfLocal(Eigen::Vector3d &uopt, Eigen::VectorXd &h_out, Eigen::Vector3d &ustar, Eigen::MatrixXd &p_j_i, std::vector<double> &safety_margin);
        void setVerbose(bool verbose);
    };
}

#endif // SAFETY_CONTROLLER_H