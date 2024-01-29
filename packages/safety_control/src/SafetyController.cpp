#include "safety_control/SafetyController.h"
#include "OsqpEigen/OsqpEigen.h"

namespace safety_control
{
    SafetyController::SafetyController(double min_distance, double max_distance, int num_robots) : min_distance_(min_distance), max_distance_(max_distance), num_robots_(num_robots)
    {

        std::cout << "SafetyController constructor called." << std::endl;
        H.resize(3, 3); // set sparse matrix size
        H.setZero();    // set all zeros
        f.resize(3);

        std::cout << "H INITIALIZED." << std::endl;
        H.insert(0,0) = 0.001;
        H.insert(1,1) = 0.001;
        H.insert(2,2) = 0.001;

        // A << 1.0, 0.0;
        std::cout << "A INITIALIZED." << std::endl;

        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);

        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(3);
        solver.data()->setNumberOfConstraints(num_robots_);
        lowerbound.resize(num_robots_);
        lowerbound.setOnes();
        lowerbound = -std::numeric_limits<double>::infinity()*lowerbound;
        upperbound.resize(num_robots_);
        upperbound.setOnes();
        upperbound = 1.5 * upperbound;

        std::cout << "INITIALIZATION COMPLETED." << std::endl;

        

        solver_init = false;

    }

    SafetyController::~SafetyController()
    {
        solver.clearSolver();
    }
    void SafetyController::setVerbose(bool verbose)
    {
        solver.settings()->setVerbosity(verbose);
    }
    int SafetyController::applyCbf(Eigen::Vector3d &uopt, Eigen::VectorXd &h_out, Eigen::Vector3d &ustar, Eigen::Vector3d &p_i, Eigen::MatrixXd &p_j)
    {
        double gamma_safe = 5.0;
        Eigen::Matrix<double,3,3> R_w_i;
        Eigen::Vector2d p_j_i;
        Eigen::SparseMatrix<double> Acbf;
        Eigen::VectorXd bcbf;

        Acbf.resize(num_robots_, 3);
        bcbf.resize(num_robots_);
        bcbf.setOnes();
        bcbf = std::numeric_limits<double>::infinity()*bcbf;

        R_w_i << cos(p_i(2)), -sin(p_i(2)), 0, sin(p_i(2)), cos(p_i(2)), 0,0,0,1;
        std::cout << "R_w_i:\n"<<R_w_i<<std::endl;

        // Convert velocity input to local frame
        Eigen::Vector3d ustar_local = ustar;
        std::cout<< "ustar_loc:\n"<<ustar_local.transpose()<<std::endl;
        f = -ustar_local.transpose() * H;

        std::cout << "f:\n"<<f.transpose()<<std::endl;

        for (int i = 0; i < num_robots_; i++)
        {
            p_j_i = p_j.col(i).head(2);
            std::cout << "p_"<<i<<"_i: " << p_j_i.transpose() << std::endl;
            Eigen::VectorXd h;
            h.resize(1);
            h(0) = pow(p_j_i.norm(), 2) - pow(min_distance_, 2);
            std::cout << "h: " << h <<std::endl;
            Acbf.insert(i, 0) = -2 * p_j_i(0);
            Acbf.insert(i, 1) = -2 * p_j_i(1);
            Acbf.insert(i, 2) = 0;

            std::cout << "Acbf: " << Acbf << std::endl;
            bcbf(i) = gamma_safe * h(0);
            std::cout << "bcbf: " << bcbf.transpose() << std::endl;

        }


        if (!solver_init)
        {   
            std::cout << "Initializing solver ..." << std::endl;
            // set the initial data of the QP solver
            solver.data()->clearLinearConstraintsMatrix();
            solver.data()->clearHessianMatrix();
            if(!solver.data()->setHessianMatrix(H)) return 1;
            if (!solver.data()->setGradient(f)) return 1;
            if(!solver.data()->setLinearConstraintsMatrix(Acbf)) return 1;
            if(!solver.data()->setLowerBound(lowerbound)) return 1;
            if(!solver.data()->setUpperBound(bcbf)) return 1;
            // instantiate the solver
            if(!solver.initSolver()) return 1;
            solver_init = true;
        }
        else{
            if (!solver.updateGradient(f)) return 1;
            if(!solver.updateLinearConstraintsMatrix(Acbf)) return 1;
            if(!solver.updateUpperBound(bcbf)) return 1;
        }
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return 1;

        // get the controller input
        auto u_i = solver.getSolution();

        // Reverse conversion of velocity input to world frame
        // uopt = R_w_i.transpose()*u_i;
        uopt = u_i;
        auto hdot = -Acbf*u_i;

        h_out = bcbf;
        std::cout << "u_i:\n" << u_i.transpose() << std::endl;
        std::cout << "u_opt:\n" << uopt.transpose() << std::endl;
        std::cout << "h_dot:\n" << hdot.transpose() << std::endl;
        std::cout << "h_dot+gamma(h):\n" << (hdot+bcbf).transpose() << std::endl;
        std::cout << "gamma(h):\n" << bcbf.transpose() << std::endl;
        return 0;
    }

    int SafetyController::applyCbfLocal(Eigen::Vector3d &uopt, Eigen::VectorXd &h_out, Eigen::Vector3d &ustar, Eigen::MatrixXd &p_j_i, std::vector<double> &safety_margin)
    {
        double gamma_safe = 2.0;
        Eigen::Matrix<double,3,3> R_w_i;
        Eigen::SparseMatrix<double> Acbf;
        Eigen::VectorXd bcbf;

        Acbf.resize(2 * num_robots_, 3);
        bcbf.resize(2 * num_robots_);
        bcbf.setOnes();
        bcbf = std::numeric_limits<double>::infinity()*bcbf;

        // R_w_i << cos(p_i(2)), -sin(p_i(2)), 0, sin(p_i(2)), cos(p_i(2)), 0,0,0,1;
        // std::cout << "R_w_i:\n"<<R_w_i<<std::endl;

        // Convert velocity input to local frame
        // Eigen::Vector3d ustar_local = R_w_i*ustar;
        // std::cout<< "ustar_loc:\n"<<ustar_local.transpose()<<std::endl;
        // f = -ustar_local.transpose() * H;
        f = -ustar.transpose() * H;

        std::cout << "f:\n"<<f.transpose()<<std::endl;

        for (int i = 0; i < num_robots_; i++)
        {
            // if(i<n_robot){
            //     // p_j: relative position of j wrt i in world frame
            //     // p_j_i: relative position of j wrt i in local frame (calculated with rotation matrix)
            //     p_j_i = R_w_i.transpose().block<2,3>(0,0) * (p_j.col(i));
            // }else{
            //     p_j_i = {100.0,0.0};
            // }
            if (p_j_i.col(i).norm() < min_distance_ + safety_margin[i])
            {
                std::cout << "--------------------------------------------" << std::endl;
                std::cout << "WARNING: Danger of collision" << std::endl;
            }
            std::cout << "p_"<<i<<"_i: " << p_j_i.col(i).transpose() << std::endl;
            Eigen::VectorXd h;
            h.resize(2);
            h(0) = pow(p_j_i.col(i).norm(), 2) - pow(min_distance_ + safety_margin[i], 2);
            h(1) = -pow(p_j_i.col(i).norm(), 2) + pow(max_distance_, 2);
            std::cout << "h: " << h.transpose() <<std::endl;
            Acbf.insert(2*i, 0) = -2 * (p_j_i(0,i) + safety_margin[i]);
            Acbf.insert(2*i, 1) = -2 * (p_j_i(1,i) + safety_margin[i]);
            Acbf.insert(2*i, 2) = 0;
            Acbf.insert(2*i + 1, 0) = 2 * p_j_i(0,i);
            Acbf.insert(2*i + 1, 1) = 2 * p_j_i(1,i);
            Acbf.insert(2*i + 1, 2) = 0;

            std::cout << "Acbf: " << Acbf << std::endl;
            bcbf(2*i) = gamma_safe * h(0);
            bcbf(2*i + 1) = gamma_safe * h(1);
            std::cout << "bcbf: " << bcbf.transpose() << std::endl;
            std::cout << "--------------------------------------------" << std::endl;

        }


        if (!solver_init)
        {   
            std::cout << "Initializing solver ..." << std::endl;
            // set the initial data of the QP solver
            solver.data()->clearLinearConstraintsMatrix();
            solver.data()->clearHessianMatrix();
            if(!solver.data()->setHessianMatrix(H)) return 1;
            if (!solver.data()->setGradient(f)) return 1;
            if(!solver.data()->setLinearConstraintsMatrix(Acbf)) return 1;
            if(!solver.data()->setLowerBound(lowerbound)) return 1;
            if(!solver.data()->setUpperBound(bcbf)) return 1;
            // instantiate the solver
            if(!solver.initSolver()) return 1;
            solver_init = true;
        }
        else{
            if (!solver.updateGradient(f)) return 1;
            if(!solver.updateLinearConstraintsMatrix(Acbf)) return 1;
            if(!solver.updateUpperBound(bcbf)) return 1;
        }
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return 1;

        // get the controller input
        auto u_i = solver.getSolution();

        // Reverse conversion of velocity input to world frame
        // uopt = R_w_i.transpose()*u_i;

        uopt = u_i;
        auto hdot = -Acbf*u_i;

        h_out = bcbf;
        std::cout << "u_i:\n" << u_i.transpose() << std::endl;
        std::cout << "u_opt:\n" << uopt.transpose() << std::endl;
        std::cout << "h_dot:\n" << hdot.transpose() << std::endl;
        std::cout << "h_dot+gamma(h):\n" << (hdot+bcbf).transpose() << std::endl;
        std::cout << "gamma(h):\n" << bcbf.transpose() << std::endl;
        return 0;
    }

}
