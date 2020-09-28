#include "benchmark.h"
#include "problem_generator.h"
#include <chrono>
#include <iomanip>
#include <iostream>

namespace pose_lib
{

template <typename Solver>
BenchmarkResult benchmark(int n_problems, const ProblemOptions &options, double tol = 1e-6) 
{

    std::vector<AbsolutePoseProblemInstance> problem_instances;
    generate_abspose_problems(n_problems, &problem_instances, options);

    BenchmarkResult result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Run benchmark where we check solution quality
    for (const AbsolutePoseProblemInstance &instance : problem_instances) {
        CameraPoseVector solutions;

        int sols = Solver::solve(instance, &solutions);

        double pose_error = std::numeric_limits<double>::max();

        result.solutions_ += sols;
        //std::cout << "Gt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
        for (const CameraPose &pose : solutions) {
            if (Solver::validator::is_valid(instance, pose, tol))
                result.valid_solutions_++;
            //std::cout << "Pose: " << pose.R << "\n" << pose.t << "\n";
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose));
        }

        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    CameraPoseVector solutions;
    for (int iter = 0; iter < 10; ++iter) {
        int total_sols = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const AbsolutePoseProblemInstance &instance : problem_instances) {
            solutions.clear();

            int sols = Solver::solve(instance, &solutions);

            total_sols += sols;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    std::sort(runtimes.begin(), runtimes.end());
    result.runtime_ns_ = runtimes[runtimes.size() / 2];
    std::cout << "\r                                                                                \r";
    return result;
}

template <typename Solver>
BenchmarkResult benchmark_relative(int n_problems, const ProblemOptions &options, double tol = 1e-6) 
{

    std::vector<RelativePoseProblemInstance> problem_instances;
    generate_relpose_problems(n_problems, &problem_instances, options);

    BenchmarkResult result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    // Run benchmark where we check solution quality
    for (const RelativePoseProblemInstance &instance : problem_instances) {
        CameraPoseVector solutions;

        int sols = Solver::solve(instance, &solutions);

        double pose_error = std::numeric_limits<double>::max();

        result.solutions_ += sols;
        //std::cout << "Gt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
        for (const CameraPose &pose : solutions) {
            if (Solver::validator::is_valid(instance, pose, tol))
                result.valid_solutions_++;
            //std::cout << "Pose: " << pose.R << "\n" << pose.t << "\n";
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose));
        }

        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    CameraPoseVector solutions;
    for (int iter = 0; iter < 10; ++iter) {
        int total_sols = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const RelativePoseProblemInstance &instance : problem_instances) {
            solutions.clear();

            int sols = Solver::solve(instance, &solutions);

            total_sols += sols;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    std::sort(runtimes.begin(), runtimes.end());

    result.runtime_ns_ = runtimes[runtimes.size() / 2];
    std::cout << "\r                                                                                \r";
    return result;
}

} // namespace pose_lib

void print_runtime(double runtime_ns) 
{
    if (runtime_ns < 1e3) {
        std::cout << runtime_ns << " ns";
    } else if (runtime_ns < 1e6) {
        std::cout << runtime_ns / 1e3 << " us";
    } else if (runtime_ns < 1e9) {
        std::cout << runtime_ns / 1e6 << " ms";
    } else {
        std::cout << runtime_ns / 1e9 << " s";
    }
}

void display_result(const std::vector<pose_lib::BenchmarkResult> &results) 
{
    int w = 13;
    // display header
    std::cout << std::setw(2 * w) << "Solver";
    std::cout << std::setw(w) << "Solutions";
    std::cout << std::setw(w) << "Valid";
    std::cout << std::setw(w) << "GT found";
    std::cout << std::setw(w) << "Runtime"
              << "\n";
    for (int i = 0; i < w * 6; ++i)
        std::cout << "-";
    std::cout << "\n";

    int prec = 6;

    for (const pose_lib::BenchmarkResult &result : results) {
        double num_tests = static_cast<double>(result.instances_);
        double solutions = result.solutions_ / num_tests;
        double valid_sols = result.valid_solutions_ / static_cast<double>(result.solutions_) * 100.0;
        double gt_found = result.found_gt_pose_ / num_tests * 100.0;
        double runtime_ns = result.runtime_ns_ / num_tests;

        std::cout << std::setprecision(prec) << std::setw(2 * w) << result.name_;
        std::cout << std::setprecision(prec) << std::setw(w) << solutions;
        std::cout << std::setprecision(prec) << std::setw(w) << valid_sols;
        std::cout << std::setprecision(prec) << std::setw(w) << gt_found;
        std::cout << std::setprecision(prec) << std::setw(w - 3);
        print_runtime(runtime_ns);
        std::cout << "\n";
    }
}

int main() 
{

    std::vector<pose_lib::BenchmarkResult> results;

    pose_lib::ProblemOptions options;
    // options.camera_fov_ = 45; // Narrow
    // options.camera_fov_ = 75; // Medium
    options.camera_fov_ = 120; // Wide

    double tol = 1e-6;

    // P3P
    pose_lib::ProblemOptions p3p_opt = options;
    p3p_opt.n_point_point_ = 3;
    p3p_opt.n_point_line_ = 0;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP3P>(1e5, p3p_opt, tol));

    // P2P1LL
    pose_lib::ProblemOptions p2p1ll_opt = options;
    p2p1ll_opt.n_point_point_ = 2;
    p2p1ll_opt.n_line_line_ = 1;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP2P1LL>(1e4, p2p1ll_opt, tol));

    // P1P2LL
    pose_lib::ProblemOptions p1p2ll_opt = options;
    p1p2ll_opt.n_point_point_ = 1;
    p1p2ll_opt.n_line_line_ = 2;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP1P2LL>(1e4, p1p2ll_opt, tol));

    // P3LL
    pose_lib::ProblemOptions p3ll_opt = options;
    p3ll_opt.n_line_line_ = 3;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP3LL>(1e4, p3ll_opt, tol));

    display_result(results);

    return 0;
}
