#include "problem_generator.h"
#include <iostream>
#include <vector>

#include "PnPL/pnpl.h"

using namespace std;
using namespace pose_lib;

int main()
{
    ProblemOptions options;
    // options.camera_fov_ = 45; // Narrow
    // options.camera_fov_ = 75; // Medium
    options.camera_fov_ = 120; // Wide
    options.n_point_point_ = 50;
    options.n_line_line_ = 50;

    double tol = 1e-6;
    int n_problems = 1;
    
    vector<AbsolutePoseProblemInstance> problem_instances;
    generate_abspose_problems(n_problems, &problem_instances, options);
    
    AbsolutePoseProblemInstance& problem = problem_instances[0];
    cout<<"Groundtruth Pose:"<<endl;
    cout<<"R = "<<endl<<problem.pose_gt.R<<endl;
    cout<<"t = "<<problem.pose_gt.t.transpose()<<endl;
    
    CameraPose pose;
    pnpl::PnPL(problem.x_point_, problem.X_point_, problem.l_line_line_, problem.X_line_line_, problem.V_line_line_, pose);
    cout<<"PnPL Result:"<<endl;
    cout<<"R = "<<endl<<pose.R<<endl;
    cout<<"t = "<<pose.t.transpose()<<endl;
    
    return 0;
}
