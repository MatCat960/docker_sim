# include <iostream>
#include <memory>
#include <random>
#include <chrono>

#include "ros/ros.h"

# include <torch/script.h>      // One-stop header

int ROBOTS_MAX = 20;
int N_ROBOTS = 12;
double AREA_W = 30.0;

int main(int argc, char **argv)
{
    // signal(SIGINT, nodeobj_wrapper_function);

    ros::init(argc, argv, "cbf_coverage");
    // auto node = std::make_shared<Controller>();
    
    std::string coverage_path = "/home/mattia/pf-training/SerializedModels/coverage_model_old.pt";
    std::string pf_path = "/home/mattia/pf-training/SerializedModels/pf_model_with_obs.pt";

    // if (argc != 2)
    // {
    //     std::cerr << "usage: test_model <path-to-exported-script-module>\n";
    //     return -1;
    // }

    torch::jit::script::Module cov_mod;
    torch::jit::script::Module pf_mod;
    try
    {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        cov_mod = torch::jit::load(coverage_path);
        pf_mod = torch::jit::load(pf_path);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    std::cout << "ok\n";

    // Create inputs
    std::vector<torch::jit::IValue> cov_inputs;
    at::Tensor robots = torch::zeros({ROBOTS_MAX, 2});
    auto a = robots.accessor<float, 2>();

    std::vector<torch::jit::IValue> pf_inputs;
    at::Tensor obs = torch::zeros({ROBOTS_MAX, 8});
    auto b = obs.accessor<float, 2>();
    
    // Randomly generate robots
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5*AREA_W, 0.5*AREA_W);
    for (int i = 0; i < N_ROBOTS; i++)
    {
        a[i][0] = dis(gen);
        a[i][1] = dis(gen);
        // std::cout << "Robot " << i << " in (" << a[i][0] << ", " << a[i][1] << ")\n";
    }

    robots = robots.view({-1, 2*ROBOTS_MAX});
    cov_inputs.push_back(robots);

    // Fill with observations of the 1st robot
    b[0][0] = a[0][0];
    b[0][1] = a[0][1];
    b[0][2] = 0.5;
    b[0][3] = 0.25;
    b[0][4] = 0.25;
    b[0][5] = 0.5;
    b[0][6] = a[0][0];
    b[0][7] = a[0][1];
    
    // Fill remaining with zeros
    for (int i = 0; i < ROBOTS_MAX; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            b[i][j] = 0.0;
        }
    }
    
    pf_inputs.push_back(obs);
    std::cout << "PF Inputs created\n";
    

    // Execute the model and turn its output into a tensor.
    at::Tensor cov_output = cov_mod.forward(cov_inputs).toTensor();

    auto pfstart = std::chrono::high_resolution_clock::now();
    std::cout << "Coverage output: " << cov_output << "\n"; 
    auto pfend = std::chrono::high_resolution_clock::now();
    std::cout << "Coverage inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(pfend-pfstart).count() << " ms\n";
    
    auto pfstart2 = std::chrono::high_resolution_clock::now();
    at::Tensor pf_output = pf_mod.forward(pf_inputs).toTensor();
    auto pfend2 = std::chrono::high_resolution_clock::now();
    std::cout << "PF inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(pfend2-pfstart2).count() << " ms\n";
    std::cout << "PF output: " << pf_output << "\n";

}