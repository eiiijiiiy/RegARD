#include <iostream>
#include "odas/nlopt.hpp"
// #include <stdio.h>     

// #define DLLEXPORT extern "C" __declspec(dllexport)

extern "C"{
void odas_DIRECT(char * c_filepath, float * rho, float * theta, double * rmse, double * time_cost)
{   
    std::string filepath(c_filepath);

    Solvernlopt solver;
    solver.load_prob(filepath);
    solver.solve(0, 1000);
    solver.validate_best_sol();  
    
    *time_cost = std::round(solver.timecost)/1000; 
    *rmse = std::round(solver.valid_rmse*1000)/1000;

    *rho = solver.best_solution[0];
    *theta = solver.best_solution[1];

//    std::cout << best_solution[0] << " " << best_solution[1];

    // return 0;
}
}