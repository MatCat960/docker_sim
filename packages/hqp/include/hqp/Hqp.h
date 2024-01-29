#ifndef HQP_H
#define HQP_H

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>



class Hqp
{
    private:
        int num_robots_;
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
        


    public:
        Hqp(double fow_angle, double min_distance, double max_distance, int num_robots);
        ~Hqp();
        Eigen::VectorXd solve(Eigen::MatrixXd p_j);
        Eigen::VectorXd solve(std::vector<Eigen::VectorXd> p_j_i);
        int solveUnordered(Eigen::MatrixXd &p_j_i, Eigen::MatrixXd slack);
        void setVerbose(bool verbose);
};

#endif // HQP_H