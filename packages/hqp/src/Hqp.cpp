#include "hqp/Hqp.h"
#include "OsqpEigen/OsqpEigen.h"


Hqp::Hqp(double fow_angle, double min_distance, double max_distance, int num_robots) : fow_angle_(fow_angle), min_distance_(min_distance), max_distance_(max_distance), num_robots_(num_robots)
{
    H.resize(7, 7); // set sparse matrix size
    H.setZero();    // set all zeros
    f.resize(7);
    f.setZero();

    H.insert(0, 0) = 1.0;
    H.insert(1, 1) = 1.0;
    H.insert(2, 2) = 1.0;
    H.insert(3, 3) = 1.0;
    H.insert(4, 4) = 1.0;
    H.insert(5, 5) = 1.0;
    H.insert(6, 6) = 1.0;

    A << tan(fow_angle_ / 2), 1.0, tan(fow_angle_ / 2), -1.0, 1.0, 0.0, -1.0, 0.0;

    // settings
    solver.settings()->setVerbosity(true);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(7);
    solver.data()->setNumberOfConstraints(4 * num_robots_);
    lowerbound.resize(4 * num_robots_);
    lowerbound.setOnes();
    lowerbound = -std::numeric_limits<double>::infinity() * lowerbound;

    

    solver_init = false;

}

Hqp::~Hqp()
{
    solver.clearSolver();
}
void Hqp::setVerbose(bool verbose)
{
    solver.settings()->setVerbosity(verbose);
}

Eigen::VectorXd Hqp::solve(Eigen::MatrixXd p_j)
{
    Eigen::VectorXd b(4);
    b << 0.0, 0.0, min_distance_, max_distance_;
    Eigen::SparseMatrix<double> Acbf;
    Eigen::SparseMatrix<double> Atilde;
    Eigen::VectorXd bcbf, coeff, btilde;

    Acbf.resize(4 * num_robots_, 3);
    Atilde.resize(4 * num_robots_, 7);          // Atilde = [Acbf, -1]

    bcbf.resize(4 * num_robots_);
    bcbf.setOnes();
    bcbf = std::numeric_limits<double>::infinity()*bcbf;

    // 4th column of Atilde
    coeff.resize(4 * num_robots_);
    coeff.setZero();                    // always zero, set to -1 for actual robot

    // vector of omega values (inf when still not calculated)
    Eigen::VectorXd omega_star;
    omega_star.resize(4 * num_robots_);
    omega_star.setOnes();
    omega_star = std::numeric_limits<double>::infinity()*omega_star;

    btilde.resize(4 * num_robots_);
    // btilde = bcbf + omega_star;

    // R_w_i << cos(p_i(2)), -sin(p_i(2)), 0, sin(p_i(2)), cos(p_i(2)), 0,0,0,1;
    // std::cout << "R_w_i:\n"<<R_w_i<<std::endl;
    // // Eigen::Vector3d ustar_local = R_w_i*ustar;
    // Eigen::Vector3d ustar_local = ustar;
    // std::cout<< "ustar_loc:\n"<<ustar_local.transpose()<<std::endl;
    // f = -ustar_local.transpose() * H;
    // f.block<4,1>(3,0) = -omega_star.transpose() * H.block<4,1>

    // Calculate Acbf and bcbf
    for (int i = 0; i < num_robots_; i++)
    {
        Eigen::VectorXd p_j_i = p_j.col(i).head(2);
        // std::cout << "----------------- Robot " << i << " -----------------" << std::endl;
        // std::cout << "p_"<<i<<"_i: " << p_j_i.transpose() << std::endl;
        Eigen::VectorXd h = A * p_j_i - b;  
        h(2) = pow(p_j_i.norm(), 2) - pow(min_distance_, 2);
        h(3) = -pow(p_j_i.norm(), 2) + pow(max_distance_, 2);
        // std::cout << "h"<<i<<": "<<h.transpose()<<std::endl;
        Acbf.insert(4 * i, 0) = (tan(fow_angle_ / 2));
        Acbf.insert(4 * i, 1) = 1;
        Acbf.insert(4 * i, 2) =  (p_j_i(0) - p_j_i(1) * tan(fow_angle_ / 2));
        Acbf.insert(4 * i + 1, 0) = tan(fow_angle_ / 2);
        Acbf.insert(4 * i + 1, 1) = -1;
        Acbf.insert(4 * i + 1, 2) = (-p_j_i(0) - p_j_i(1) * tan(fow_angle_ / 2));
        Acbf.insert(4 * i + 2, 0) = 2 * p_j_i(0);
        Acbf.insert(4 * i + 2, 1) = 2 * p_j_i(1);
        Acbf.insert(4 * i + 2, 2) = 0;
        // std::cout << "SONO QUI"<<std::endl;    
        Acbf.insert(4 * i + 3, 0) = -2 * p_j_i(0);
        Acbf.insert(4 * i + 3, 1) = -2 * p_j_i(1);
        Acbf.insert(4 * i + 3, 2) = 0;
        // std::cout <<"SONO UQI 2" << std::endl;
        bcbf(4 * i, 0) = h(0); // + slack(i,0);
        bcbf(4 * i+1, 0) = h(1); // + slack(i,1);
        bcbf(4 * i+2, 0) = h(2); // + slack(i,2);
        bcbf(4 * i+3, 0) = h(3); // + slack(i,3);
    }

    // std::cout << "Acbf:\n" << Acbf << std::endl;
    // std::cout << "bcbf:\n" << bcbf.transpose() << std::endl;

    omega_star.head(4) = Eigen::VectorXd::Zero(4);            // initialize for 1st iteration
    for (int i = 0; i < num_robots_; i++)
    {
        // std::cout << "----------------- Iteration number " << i << " -----------------" << std::endl;
        // Calculate Atilde and btilde
        // coeff.setZero();                    // always zero, set to -1 for actual robot
        // coeff.block<4, 1>(4*i,0) = -1 * Eigen::VectorXd::Ones(4);
        Atilde.col(0) = Acbf.col(0);
        Atilde.col(1) = Acbf.col(1);
        Atilde.col(2) = Acbf.col(2);
        Atilde.insert(4 * i, 3) = -1.0;
        Atilde.insert(4 * i + 1, 3) = -1.0;
        Atilde.insert(4 * i + 2, 3) = -1.0;
        Atilde.insert(4 * i + 3, 3) = -1.0;
        Atilde.insert(4 * i, 4) = -1.0;
        Atilde.insert(4 * i + 1, 4) = -1.0;
        Atilde.insert(4 * i + 2, 4) = -1.0;
        Atilde.insert(4 * i + 3, 4) = -1.0;
        Atilde.insert(4 * i, 5) = -1.0;
        Atilde.insert(4 * i + 1, 5) = -1.0;
        Atilde.insert(4 * i + 2, 5) = -1.0;
        Atilde.insert(4 * i + 3, 5) = -1.0;
        Atilde.insert(4 * i, 6) = -1.0;
        Atilde.insert(4 * i + 1, 6) = -1.0;
        Atilde.insert(4 * i + 2, 6) = -1.0;
        Atilde.insert(4 * i + 3, 6) = -1.0;
        // std::cout << "Atilde:\n" << Atilde << std::endl;

        btilde << bcbf + omega_star;
        // std::cout << "Btilde: " << btilde.transpose() << std::endl;

        f.tail(4) = 2 * omega_star.block<4, 1>(i,0);
        // std::cout << "\033[0;31m actual_omega_" << i << ": " << omega_star.block<4, 1>(i, 0).transpose() << "\033[0m\n";
        // std::cout << "\033[0;31mGradient: " << f.tail(4).transpose() << "\033[0m\n";

        if (!solver_init)
        { 
            // set the initial data of the QP solver
            solver.data()->clearLinearConstraintsMatrix();
            solver.data()->clearHessianMatrix();
            if(!solver.data()->setHessianMatrix(H)) return Eigen::VectorXd::Zero(4);
            if (!solver.data()->setGradient(f)) return Eigen::VectorXd::Zero(4);
            if(!solver.data()->setLinearConstraintsMatrix(Atilde)) return Eigen::VectorXd::Zero(4);
            if(!solver.data()->setLowerBound(lowerbound)) return Eigen::VectorXd::Zero(4);
            if(!solver.data()->setUpperBound(btilde)) return Eigen::VectorXd::Zero(4);
            // instantiate the solver
            if(!solver.initSolver()) return Eigen::VectorXd::Zero(4);
            solver_init = true;
        }
        else{
            if (!solver.updateGradient(f)) return Eigen::VectorXd::Zero(4);
            if(!solver.updateLinearConstraintsMatrix(Atilde)) return Eigen::VectorXd::Zero(4);
            if(!solver.updateUpperBound(btilde)) return Eigen::VectorXd::Zero(4);
        }
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return Eigen::VectorXd::Zero(4);

        // get solution [ux, uy, w, omega]
        auto sol = solver.getSolution();
        // std::cout << "Solution: " << sol.transpose() << std::endl;
        
        Eigen::VectorXd omega_i = sol.tail(4);
        omega_i = (omega_i.array() < 0.0).select(0, omega_i); // set negative values to zero
        // std::cout << "\033[0;31m solution for constraint " << i << ": " << omega_i.transpose() << "\033[0m\n";
        // std::cout << "Omega_i: " << omega_i.transpose() << std::endl;
        // std::cout << "Solution: " << sol.transpose() << std::endl;
        // std::cout << "Optimal value for omega: " << omega_i << std::endl;
        // std::cout << "Optimal value for omega: " << omega_i << std::endl;

        // update omega_star
        omega_star.block<4, 1>(4*i,0) = omega_i; //* Eigen::VectorXd::Ones(4);
        
        if (i < num_robots_-1)
        {
            omega_star.block<4, 1>(4*(i+1),0).setZero();
        }
    }

    // std::cout << "\033[0;31m Final omega values: " << omega_star.transpose() << "\033[0m\n";
    // std::cout << "Final optimal omega values: " << omega_star.transpose() << std::endl;

    return omega_star;
}

Eigen::VectorXd Hqp::solve(std::vector<Eigen::VectorXd> p_j_i)
{
    Eigen::MatrixXd p_matrix;
    p_matrix.resize(p_j_i[0].size(), p_j_i.size());
    for (int i = 0; i < p_j_i.size(); i++)
    {
        p_matrix.col(i) = p_j_i[i];
    }

    // Eigen::MatrixXd slack_matrix;
    // slack_matrix.resize(slack[0].size(), slack.size());
    // for (int i = 0; i < slack.size(); i++)
    // {
    //     slack_matrix.col(i) = slack[i];
    // }

    return solve(p_matrix);

}

int solveUnordered(Eigen::MatrixXd &p_j_i, Eigen::MatrixXd slack)
{
    // reorder asdasd
}