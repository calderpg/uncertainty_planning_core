#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <functional>
#include <random>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <arc_utilities/simple_rrt_planner.hpp>
#include <uncertainty_planning_core/simple_pid_controller.hpp>
#include <uncertainty_planning_core/simple_uncertainty_models.hpp>
#include <uncertainty_planning_core/uncertainty_contact_planning.hpp>
#include <uncertainty_planning_core/simplese2_robot_helpers.hpp>
#include <uncertainty_planning_core/baxter_linked_common_config.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using namespace uncertainty_contact_planning;

void peg_in_hole_env_linked(ros::Publisher& display_debug_publisher)
{
    const uncertainty_planning_core::OPTIONS options = baxter_linked_common_config::GetOptions();
    std::cout << PrettyPrint::PrettyPrint(options) << std::endl;
    const std::vector<double> joint_uncertainty_params = baxter_linked_common_config::GetJointUncertaintyParams(options);
    assert(joint_uncertainty_params.size() == 7);
    const std::pair<baxter_linked_common_config::SLC, baxter_linked_common_config::SLC> start_and_goal = baxter_linked_common_config::GetStartAndGoal();
    const simplelinked_robot_helpers::SimpleLinkedBaseSampler sampler = baxter_linked_common_config::GetSampler();
    const simplelinked_robot_helpers::ROBOT_CONFIG robot_config = baxter_linked_common_config::GetDefaultRobotConfig(options);
    const Eigen::Affine3d base_transform = baxter_linked_common_config::GetBaseTransform();
    const simplelinked_robot_helpers::SimpleLinkedRobot<baxter_linked_common_config::BaxterJointActuatorModel> robot = baxter_linked_common_config::GetRobot(base_transform, robot_config, joint_uncertainty_params, options.environment_name);
    auto planner_result = uncertainty_planning_core::PlanBaxterUncertainty(options, robot, sampler, start_and_goal.first, start_and_goal.second, display_debug_publisher);
    const auto& policy = planner_result.first;
    const std::map<std::string, double> planner_stats = planner_result.second;
    const double p_goal_reached = planner_stats.at("P(goal reached)");
    if (p_goal_reached >= options.goal_probability_threshold)
    {
        std::cout << "Planner reached goal, saving & loading policy" << std::endl;
        // Save the policy
        assert(uncertainty_planning_core::SaveBaxterPolicy(policy, options.planned_policy_file));
        const auto loaded_policy = uncertainty_planning_core::LoadBaxterPolicy(options.planned_policy_file);
        std::vector<uint8_t> policy_buffer;
        policy.SerializeSelf(policy_buffer);
        std::vector<uint8_t> loaded_policy_buffer;
        loaded_policy.SerializeSelf(loaded_policy_buffer);
        assert(policy_buffer.size() == loaded_policy_buffer.size());
        for (size_t idx = 0; idx > policy_buffer.size(); idx++)
        {
            const uint8_t policy_buffer_byte = policy_buffer[idx];
            const uint8_t loaded_policy_buffer_byte = loaded_policy_buffer[idx];
            assert(policy_buffer_byte == loaded_policy_buffer_byte);
        }
        assert(policy.GetRawPreviousIndexMap().size() == loaded_policy.GetRawPreviousIndexMap().size());
    }
    else
    {
        std::cout << "Planner failed to reach goal" << std::endl;
    }
    // Print out the results & save them to the log file
    const std::string log_results = "++++++++++\n" + PrettyPrint::PrettyPrint(options) + "\nRESULTS:\n" + PrettyPrint::PrettyPrint(planner_stats, false, "\n");
    std::cout << "Planner results for " << options.num_particles << " particles:\n" << log_results << std::endl;
    std::ofstream log_file(options.planner_log_file, std::ios_base::out | std::ios_base::app);
    if (!log_file.is_open())
    {
        std::cerr << "\x1b[31;1m Unable to create folder/file to log to: " << options.planner_log_file << "\x1b[37m \n";
        throw std::invalid_argument( "Log filename must be write-openable" );
    }
    log_file << log_results << std::endl;
    log_file.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "linked_contact_planning_node");
    ros::NodeHandle nh;
    ROS_INFO("Starting Nomdp Contact Planning Node...");
    ros::Publisher display_debug_publisher = nh.advertise<visualization_msgs::MarkerArray>("nomdp_debug_display_markers", 1, true);
    peg_in_hole_env_linked(display_debug_publisher);
    return 0;
}