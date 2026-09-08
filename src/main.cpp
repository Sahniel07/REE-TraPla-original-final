#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include "dynamic_game_planner.h"
#include "scenarios.h"

#if (ENABLE_QUANTIZATION == 1)
    std::string quantization_status = "_withQuanti";
#else
    std::string quantization_status = "_withoutQuanti";
#endif

/** Identifies the arithmetic format in output filenames */
#if USE_POSIT
    std::string format_tag = "posit" + std::to_string(REAL_BITS) + "es" + std::to_string(POSIT_ES);
#else
    std::string format_tag = std::to_string(REAL_BITS);
#endif

/** Identifies the trigonometric path, so the two paths of the same format do
 *  not overwrite each other's output. See include/trig.h. */
#if USE_POLY
    std::string trig_tag = "_poly";
#else
    std::string trig_tag = "_nopoly";
#endif

/** Scenario to run. Set it here, or from the build with -DSCENARIO=NAME
 *  (wired in CMakeLists.txt, which sweep.sh passes through). The names are
 *  the ScenarioId values in include/scenarios.h. */
#ifndef SCENARIO
#define SCENARIO SCENARIO_INTERSECTION
#endif
static const int scenario_id = SCENARIO;

void save_lanes_to_csv(const std::vector<VehicleState>& traffic, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    // Write CSV header
    file << "lane_type,x,y,s\n";

    for (const auto& vehicle : traffic) {
        std::vector<std::pair<std::string, Lane>> lanes = {
            {"center", vehicle.centerlane},
            {"left", vehicle.leftlane},
            {"right", vehicle.rightlane}
        };

        for (const auto& [lane_type, lane] : lanes) {
            if (lane.present) {  // Only save if the lane exists
                int num_samples = 20;  // Number of points along the lane
                for (int i = 0; i < num_samples; i++) {
                    double s = i * (lane.s_max / num_samples);
                    double x = lane.spline_x(s);
                    double y = lane.spline_y(s);

                    file << lane_type << "," << x << "," << y << "," << s << "\n";
                }
            }
        }
    }

    file.close();
    std::cout << "Lanes saved to " << filename << std::endl;
}

void save_trajectories_to_csv(const std::vector<VehicleState>& traffic, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    file << "vehicle_id,x,y,psi,s,l,time\n";
    
    for (size_t i = 0; i < traffic.size(); i++) {
        file << i << "," << traffic[i].x << "," << traffic[i].y << "," << traffic[i].psi << "," << 0 << "," << 0 << "," << 0 << "\n";
        for (const auto& point : traffic[i].predicted_trajectory) {
            file << i << "," << point.x << "," << point.y << "," << point.psi << "," << point.s << "," << point.l << "," << point.t_start << "\n";
        }
    }

    file.close();
    std::cout << "Trajectories saved to " << filename << std::endl;
}

int main() {

    const std::string scenario_tag = scenario_name(scenario_id);

    DynamicGamePlanner planner_intersection;
    std::cerr << "------------------------ " << scenario_tag
              << " Scenario -----------------------------" << "\n";

    TrafficParticipants traffic_intersection = make_scenario(scenario_id);

    auto start_time = std::chrono::high_resolution_clock::now();

    planner_intersection.run(traffic_intersection);

    auto end_time = std::chrono::high_resolution_clock::now();

    // Compute duration in milliseconds
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Execution Time for run(): " << elapsed_time.count() << " ms" << std::endl;
    std::cout << "Excution Time for integrate(): " << planner_intersection.getRuntimeForIntegrate_ms()<< " ms" << std::endl;

    traffic_intersection = planner_intersection.traffic;

    // Save trajectories to a CSV file
    std::string output_filename = "trajectories_" + scenario_tag + "_" + format_tag + quantization_status + trig_tag + ".csv";
    save_trajectories_to_csv(traffic_intersection, output_filename);
    save_lanes_to_csv(traffic_intersection, "lanes_" + scenario_tag + "_" + format_tag + quantization_status + trig_tag + ".csv");

    // Plot this run into media/ (run from build/, so the script is one level up).
    char eps_str[32];
    std::snprintf(eps_str, sizeof(eps_str), "%g", planner_intersection.param.eps);
    std::string plot_cmd = "python3 ../plot_run.py \"" + output_filename + "\""
                         + " --eps " + eps_str;
#if USE_POSIT
    plot_cmd += " --es " + std::to_string(POSIT_ES);
#endif
    if (std::system(plot_cmd.c_str()) != 0) {
        std::cerr << "Warning: plotting failed (" << plot_cmd << ")" << std::endl;
    }

    return 0;
}
