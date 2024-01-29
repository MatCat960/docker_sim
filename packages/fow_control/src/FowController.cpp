#include "fow_control/FowController.h"
#include "OsqpEigen/OsqpEigen.h"

namespace fow_control
{

    FowController::FowController(double fow_angle, double min_distance, double max_distance, int max_robots) : fow_angle_(fow_angle), min_distance_(min_distance), max_distance_(max_distance), max_robots_(max_robots)
    {
        H.resize(3, 3); // set sparse matrix size
        H.setZero();    // set all zeros
        f.resize(3);

        H.insert(0, 0) = 1.0;
        H.insert(1, 1) = 1.0;
        H.insert(2, 2) = 0.5;

        A << tan(fow_angle_ / 2), 1.0, tan(fow_angle_ / 2), -1.0, 1.0, 0.0, -1.0, 0.0;

        // settings
        solver.settings()->setVerbosity(true);
        solver.settings()->setWarmStart(true);

        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(3);
        solver.data()->setNumberOfConstraints(4 * max_robots_);
        lowerbound.resize(4 * max_robots);
        lowerbound.setOnes();
        lowerbound = -std::numeric_limits<double>::infinity() * lowerbound;
        // lowerbound = -1.5 * lowerbound;

        upperbound.resize(4 * max_robots);
        upperbound.setOnes();
        upperbound = std::numeric_limits<double>::infinity() * upperbound;
        // upperbound = 1.5 * upperbound;

        max_vel_ = 1.5;
        min_vel_ = -max_vel_;

        // lowerbound.tail(3) = min_vel_ * Eigen::VectorXd::Ones(3);
        

        solver_init = false;

    }

    FowController::~FowController()
    {
        solver.clearSolver();
    }
    void FowController::setVerbose(bool verbose)
    {
        solver.settings()->setVerbosity(verbose);
    }
    int FowController::applyCbf(Eigen::Vector3d &uopt, Eigen::VectorXd &h_out, Eigen::Vector3d &ustar, Eigen::Vector3d &p_i, int n_robot, Eigen::MatrixXd &p_j)
    {
        double gamma0 = 1.0;
        double gamma_safe = 5.0;
        // Eigen::DiagonalMatrix<double, 4> gamma(0.0050, 0.0050, 5.0, 5.0);
        Eigen::Matrix<double,3,3> R_w_i;
        Eigen::VectorXd b(4);
        b << 0.0, 0.0, min_distance_, max_distance_;
        Eigen::Vector2d p_j_i;
        Eigen::SparseMatrix<double> Acbf;
        Eigen::VectorXd bcbf;

        Acbf.resize(4 * max_robots_, 3);
        bcbf.resize(4 * max_robots_);
        bcbf.setOnes();
        bcbf = std::numeric_limits<double>::infinity()*bcbf;

        R_w_i << cos(p_i(2)), -sin(p_i(2)), 0, sin(p_i(2)), cos(p_i(2)), 0,0,0,1;
        std::cout << "R_w_i:\n"<<R_w_i<<std::endl;
        // Eigen::Vector3d ustar_local = R_w_i*ustar;
        Eigen::Vector3d ustar_local = ustar;
        std::cout<< "ustar_loc:\n"<<ustar_local.transpose()<<std::endl;
        f = -ustar_local.transpose() * H;

        for (int i = 0; i < max_robots_; i++)
        {
            if(i<n_robot){
                // p_j_i = R_w_i.block<2,3>(0,0) * (p_j.col(i));
                p_j_i = p_j.col(i).head(2);
            }else{
                p_j_i = {100.0,0.0};
            }
            std::cout << "p_"<<i<<"_i: " << p_j_i.transpose() << std::endl;
            Eigen::VectorXd h = A * p_j_i - b;  
            h(2) = pow(p_j_i.norm(), 2) - pow(min_distance_, 2);
            h(3) = -pow(p_j_i.norm(), 2) + pow(max_distance_, 2);
            std::cout << "h"<<i<<": "<<h.transpose()<<std::endl;
            Acbf.insert(4 * i, 0) = (tan(fow_angle_ / 2));
            Acbf.insert(4 * i, 1) = 1;
            Acbf.insert(4 * i, 2) =  (p_j_i(0) - p_j_i(1) * tan(fow_angle_ / 2));
            Acbf.insert(4 * i + 1, 0) = tan(fow_angle_ / 2);
            Acbf.insert(4 * i + 1, 1) = -1;
            Acbf.insert(4 * i + 1, 2) = (-p_j_i(0) - p_j_i(1) * tan(fow_angle_ / 2));
            //Acbf.insert(3 * i + 2, 0) = 1;
            //Acbf.insert(3 * i + 2, 1) = 0;                  
            //Acbf.insert(3 * i + 2, 2) = -p_j_i(1);
            Acbf.insert(4 * i + 2, 0) = -2 * p_j_i(0);//(cos(p_i(2))*p_j_i(0) -sin(p_i(2))*p_j_i(1));
            Acbf.insert(4 * i + 2, 1) = -2 * p_j_i(1);//(sin(p_i(2))*p_j_i(0) +cos(p_i(2))*p_j_i(1));
            Acbf.insert(4 * i + 2, 2) = 0;
            // std::cout << "SONO QUI"<<std::endl;    
            Acbf.insert(4 * i + 3, 0) = 2 * p_j_i(0);
            Acbf.insert(4 * i + 3, 1) = 2 * p_j_i(1);
            Acbf.insert(4 * i + 3, 2) = 0;
            // std::cout <<"SONO UQI 2" << std::endl;
            bcbf(4 * i, 0) = gamma0 * h(0)*h(0)*h(0);
            bcbf(4 * i+1, 0) = gamma0 * h(1)*h(1)*h(1);
            bcbf(4 * i+2, 0) = gamma_safe * h(2);
            bcbf(4 * i+3, 0) = gamma_safe * h(3);
            // std::cout << "SONO QUI 3" << std::endl;
            // std::cout << "Acbf:\n"<<Acbf<<std::endl;
        }

        


        if (!solver_init)
        { 
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

    int FowController::applyCbfSingle(Eigen::Vector3d &uopt, Eigen::VectorXd &h_out, Eigen::Vector3d &ustar, Eigen::Vector3d &p_i, int n_robot, Eigen::MatrixXd &p_j, Eigen::MatrixXd slack)
    {
        double gamma0 = 1.0;
        double gamma_safe = 5.0;
        // Eigen::DiagonalMatrix<double, 4> gamma(0.0050, 0.0050, 5.0, 5.0);
        Eigen::Matrix<double,3,3> R_w_i;
        Eigen::VectorXd b(4);
        b << 0.0, 0.0, min_distance_, max_distance_;
        Eigen::Vector2d p_j_i;
        Eigen::SparseMatrix<double> Acbf;
        Eigen::VectorXd bcbf;

        Acbf.resize(4 * n_robot, 3);
        bcbf.resize(4 * n_robot);
        bcbf.setOnes();
        bcbf = std::numeric_limits<double>::infinity()*bcbf;

        R_w_i << cos(p_i(2)), -sin(p_i(2)), 0, sin(p_i(2)), cos(p_i(2)), 0,0,0,1;
        // std::cout << "R_w_i:\n"<<R_w_i<<std::endl;
        // Eigen::Vector3d ustar_local = R_w_i*ustar;
        Eigen::Vector3d ustar_local = ustar;
        // std::cout<< "ustar_loc:\n"<<ustar_local.transpose()<<std::endl;
        f = -ustar_local.transpose() * H;

        for (int i = 0; i < n_robot; i++)
        {
            p_j_i = p_j.col(i).head(2);

            // std::cout << "p_"<<i<<"_i: " << p_j_i.transpose() << std::endl;
            Eigen::VectorXd h = A * p_j_i - b; // + slack.col(i);  
            h(2) = pow(p_j_i.norm(), 2) - pow(min_distance_, 2); // + slack(2,i);
            h(3) = -pow(p_j_i.norm(), 2) + pow(max_distance_, 2); //  + slack(3,i);
            // std::cout << "h"<<i<<": "<<h.transpose()<<std::endl;
            Acbf.insert(4 * i, 0) = (tan(fow_angle_ / 2));
            Acbf.insert(4 * i, 1) = 1;
            Acbf.insert(4 * i, 2) =  (p_j_i(0) - p_j_i(1) * tan(fow_angle_ / 2));
            Acbf.insert(4 * i + 1, 0) = tan(fow_angle_ / 2);
            Acbf.insert(4 * i + 1, 1) = -1;
            Acbf.insert(4 * i + 1, 2) = (-p_j_i(0) - p_j_i(1) * tan(fow_angle_ / 2));
            //Acbf.insert(3 * i + 2, 0) = 1;
            //Acbf.insert(3 * i + 2, 1) = 0;                  
            //Acbf.insert(3 * i + 2, 2) = -p_j_i(1);
            Acbf.insert(4 * i + 2, 0) = 2 * p_j_i(0);//(cos(p_i(2))*p_j_i(0) -sin(p_i(2))*p_j_i(1));
            Acbf.insert(4 * i + 2, 1) = 2 * p_j_i(1);//(sin(p_i(2))*p_j_i(0) +cos(p_i(2))*p_j_i(1));
            Acbf.insert(4 * i + 2, 2) = 0;
            // std::cout << "SONO QUI"<<std::endl;    
            Acbf.insert(4 * i + 3, 0) = -2 * p_j_i(0);
            Acbf.insert(4 * i + 3, 1) = -2 * p_j_i(1);
            Acbf.insert(4 * i + 3, 2) = 0;
            // std::cout <<"SONO UQI 2" << std::endl;
            bcbf(4 * i, 0) = gamma0 * h(0)*h(0)*h(0) + slack(0, i); //*h(0)*h(0);
            bcbf(4 * i+1, 0) = gamma0 * h(1)*h(1)*h(1) + slack(1,i); //*h(1)*h(1);
            bcbf(4 * i+2, 0) = gamma_safe * h(2) + slack(2,i);
            bcbf(4 * i+3, 0) = gamma0 * h(3) + slack(3,i);
            // std::cout << "SONO QUI 3" << std::endl;
            // std::cout << "Acbf:\n"<<Acbf<<std::endl;
        }

        /*
        // Add last 3 rows for max vel constraints
        Acbf.insert(4 * n_robot, 0) = 1;
        Acbf.insert(4 * n_robot, 1) = 0;
        Acbf.insert(4 * n_robot, 2) = 0;
        Acbf.insert(4 * n_robot + 1, 0) = 0;
        Acbf.insert(4 * n_robot + 1, 1) = 1;
        Acbf.insert(4 * n_robot + 1, 2) = 0;
        Acbf.insert(4 * n_robot + 2, 0) = 0;
        Acbf.insert(4 * n_robot + 2, 1) = 0;
        Acbf.insert(4 * n_robot + 2, 2) = 1;
        
    
        // Other 3 rows for min vel constraints
        Acbf.insert(4 * n_robot + 3, 0) = -1;
        Acbf.insert(4 * n_robot + 3, 1) = 0;
        Acbf.insert(4 * n_robot + 3, 2) = 0;
        Acbf.insert(4 * n_robot + 4, 0) = 0;
        Acbf.insert(4 * n_robot + 4, 1) = -1;
        Acbf.insert(4 * n_robot + 4, 2) = 0;
        Acbf.insert(4 * n_robot + 5, 0) = 0;
        Acbf.insert(4 * n_robot + 5, 1) = 0;
        Acbf.insert(4 * n_robot + 5, 2) = -1;

        

        bcbf(4 * n_robot, 0) = max_vel_;
        bcbf(4 * n_robot + 1, 0) = max_vel_;
        bcbf(4 * n_robot + 2, 0) = max_vel_;
        bcbf(4 * n_robot + 3, 0) = -min_vel_;
        bcbf(4 * n_robot + 4, 0) = -min_vel_;
        bcbf(4 * n_robot + 5, 0) = -min_vel_;

        */



        if (!solver_init)
        { 
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
        // uopt = R_w_i.transpose()*u_i;
        uopt = u_i;
        auto hdot = -Acbf*u_i;

        // if (((hdot.tail(6)+bcbf.tail(6)).array() < -0.2).any())
        // {
        //     std::cout << "\033[1;31m hdot + gamma < 0.0\033[0m\n";
        // }


        h_out = bcbf;
        // std::cout << "u_i:\n" << u_i.transpose() << std::endl;
        // std::cout << "u_opt:\n" << uopt.transpose() << std::endl;
        // std::cout << "h_dot:\n" << hdot.transpose() << std::endl;
        // std::cout << "h_dot+gamma(h):\n" << (hdot+bcbf).transpose() << std::endl;
        // std::cout << "gamma(h):\n" << bcbf.transpose() << std::endl;
        return 0;
    }
}
