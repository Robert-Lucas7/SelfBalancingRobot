// SBR implementations for RLTools loop functionality (e.g. step, initial_action, sample_initial_action, etc)

#include <rl_tools/operations/cpu_mux.h>
#include "rl_tools/rl/environments/operations_generic.h"
#include "rl_tools/containers/matrix/matrix.h"
#include "self_balancing_robot/sbr.hpp"
#include "self_balancing_robot/robot_utils.hpp"

#include <thread>
#include <chrono>

namespace rl_tools {
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE&, SBREnvironment<SPEC>& env, typename SBREnvironment<SPEC>::Parameters& parameters){
        return "{}";
    }
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE&, SBREnvironment<SPEC>& env, typename SBREnvironment<SPEC>::Parameters& parameters, typename SBREnvironment<SPEC>::State& state){
        std::string json = "{";
        json += "\"theta\":" + std::to_string(state.theta);
        json += "}";
        return json;
    }

    template<typename DEVICE, typename SPEC>
    void malloc(DEVICE& device, SBREnvironment<SPEC>& env){}
    template<typename DEVICE, typename SPEC>
    void free(DEVICE& device, SBREnvironment<SPEC>& env){}
    template<typename DEVICE, typename SPEC>
    void init(DEVICE& device, SBREnvironment<SPEC>& env){}
    template<typename DEVICE, typename SPEC>
    void initial_parameters(DEVICE& device, const SBREnvironment<SPEC>& env, typename SBREnvironment<SPEC>::Parameters& parameters){ }
    template<typename DEVICE, typename SPEC, typename RNG>
    void sample_initial_parameters(DEVICE& device, const SBREnvironment<SPEC>& env, typename SBREnvironment<SPEC>::Parameters& parameters, RNG& rng){ }

    // TODO: Check what the precise difference is between initial_state and sample_initial_state.
    template <typename DEVICE, typename SPEC>
    void initial_state(DEVICE& device, const SBREnvironment<SPEC>& env, const typename SBREnvironment<SPEC>::Parameters& parameters, typename SBREnvironment<SPEC>::State& state) {
        state.theta = 0;
        RCLCPP_INFO(env.ros->node->get_logger(), "INITIAL STATE SET");
    }

    template <typename DEVICE, typename SPEC, typename RNG>
    void sample_initial_state(DEVICE& device, const SBREnvironment<SPEC>& env, const typename SBREnvironment<SPEC>::Parameters& parameters, typename SBREnvironment<SPEC>::State& state, RNG& rng) {
        using T = typename SPEC::T;
        state.theta = 0;  // TODO: generate small random angle

        geometry_msgs::msg::Twist cmdVelMsg;
        cmdVelMsg.angular.z = 0.0; 
        env.ros->cmdVelPub->publish(cmdVelMsg);

        auto request = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
        request->world_control.reset.all = true;

        auto future = env.ros->controlClient->async_send_request(request);
        
        rclcpp::spin_until_future_complete(env.ros->node, future);
        
        if (env.current_episode > 0) {
            RCLCPP_INFO(env.ros->node->get_logger(), "Episode %d completed in %d steps.", env.current_episode, env.currentStepCount);
        }

        env.current_episode++;
        env.currentStepCount = 0;

        // RCLCPP_INFO(env.ros->node->get_logger(), "Environment reset completed.");
    }

    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    typename SPEC::T step(DEVICE& device, const SBREnvironment<SPEC>& env, const typename SBREnvironment<SPEC>::Parameters& parameters, const typename SBREnvironment<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, typename SBREnvironment<SPEC>::State& next_state, RNG& rng) {
        // TODO: assert action dimension
        using T = typename SPEC::T;
        using PARAMS = typename SPEC::PARAMETERS;

        rclcpp::Time startTime = env.ros->node->now();
        // RCLCPP_INFO(env.ros->node->get_logger(), "Start time: %f", startTime.seconds());

        std::bernoulli_distribution coinFlip(0.5);
        // T direction = random::uniform_int_distribution(device.random, 0, 1, rng);
        // T currentAction = direction == 0 ? 2 : -2; // Simple fixed action for testing
        geometry_msgs::msg::Twist cmdVelMsg;
        T angular_velocity = std::clamp(get(action, 0, 0), -PARAMS::MAX_ANGULAR_VELOCITY, PARAMS::MAX_ANGULAR_VELOCITY);
        cmdVelMsg.angular.z = angular_velocity;

        env.ros->cmdVelPub->publish(cmdVelMsg);


        // Get the new state
        T pitch = env.ros->getPitchFromIMU();
        next_state.theta = pitch;
        // RCLCPP_INFO(env.ros->node->get_logger(), "Prev Theta: %f, New Theta: %f, angular velocity: %f", state.theta, next_state.theta, angular_velocity);

        rclcpp::Time endTime = env.ros->node->now();
        // RCLCPP_INFO(env.ros->node->get_logger(), "End time: %f", endTime.seconds());

        rclcpp::Duration delta = endTime - startTime;

        env.currentStepCount++;

        return delta.seconds(); // Return change in time.
    }

    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    typename SPEC::T reward(DEVICE& device, const SBREnvironment<SPEC>& env, const typename SBREnvironment<SPEC>::Parameters& parameters, const typename SBREnvironment<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename SBREnvironment<SPEC>::State& next_state, RNG& rng){
        using T = typename SPEC::T;
        if (env.ros->terminateEpisode(next_state.theta)) {
            return -100.0; // Large negative reward for termination
        } else {
            return 1.0;
        }
        
    }

    template<typename DEVICE, typename SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    void observe(DEVICE& device, const SBREnvironment<SPEC>& env, const typename SBREnvironment<SPEC>::Parameters& parameters, const typename SBREnvironment<SPEC>::State& state, const SBRObservation<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        // static_assert(OBS_SPEC::ROWS == 1);
        // static_assert(OBS_SPEC::COLS == 3);
        using T = typename SPEC::T;

        set(observation, 0, 0, state.theta);
    }
    template<typename DEVICE, typename SPEC, typename RNG>
    bool terminated(DEVICE& device, const SBREnvironment<SPEC>& env, const typename SBREnvironment<SPEC>::Parameters& parameters, const typename SBREnvironment<SPEC>::State state, RNG& rng){
        using T = typename SPEC::T;
        // RCLCPP_INFO(env.ros->node->get_logger(), "Checking termination with theta: %f", state.theta);
        bool terminating = env.ros->terminateEpisode(state.theta);
        if (terminating) {
            // RCLCPP_INFO(env.ros->node->get_logger(), "Episode terminating due to theta: %f", state.theta);
        }
        return terminating;
    }
}