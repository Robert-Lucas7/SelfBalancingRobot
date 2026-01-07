
#include "self_balancing_robot/robot_utils.hpp"
#include "self_balancing_robot/operations.hpp"

#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn_models/operations_cpu.h>

#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>
#include <rl_tools/rl/algorithms/ppo/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/extrack/operations_cpu.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/save_trajectories/operations_cpu.h>

#include <rclcpp/rclcpp.hpp>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
using T = float;
using TYPE_POLICY = rlt::numeric_types::Policy<T>;
using TI = typename DEVICE::index_t;
using SBR_SPEC = SBRSpecification<T, TI, SBRParameters<T>>;
using ENVIRONMENT = SBREnvironment<SBR_SPEC>;

struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>{
    static constexpr TI N_ENVIRONMENTS = 1;
    static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 1024;
    static constexpr TI BATCH_SIZE = 128;
    static constexpr TI TOTAL_STEP_LIMIT = 1000000;
    static constexpr TI ACTOR_HIDDEN_DIM = 32;
    static constexpr TI CRITIC_HIDDEN_DIM = 32;
    static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::FAST_TANH;
    static constexpr TI STEP_LIMIT = TOTAL_STEP_LIMIT/(ON_POLICY_RUNNER_STEPS_PER_ENV * N_ENVIRONMENTS) + 1;
    static constexpr TI EPISODE_STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;
    struct OPTIMIZER_PARAMETERS: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
        static constexpr T ALPHA = 0.001;
    };
    static constexpr bool NORMALIZE_OBSERVATIONS = true;
    struct PPO_PARAMETERS: rlt::rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
        static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.0;
        static constexpr TI N_EPOCHS = 1;
        static constexpr T GAMMA = 0.9;
        static constexpr T INITIAL_ACTION_STD = 2.0;
    };
};

using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS>;

using LOOP_EXTRACK_CONFIG = rlt::rl::loop::steps::extrack::Config<LOOP_CORE_CONFIG>; // Sets up the experiment tracking structure (https://docs.rl.tools/10-Experiment%20Tracking.html)
template <typename NEXT>
struct LOOP_EVAL_PARAMETERS: rlt::rl::loop::steps::evaluation::Parameters<TYPE_POLICY, TI, NEXT>{
    static constexpr TI EVALUATION_INTERVAL = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / 10;
    static constexpr TI NUM_EVALUATION_EPISODES = 1;
    static constexpr TI N_EVALUATIONS = NEXT::CORE_PARAMETERS::STEP_LIMIT / EVALUATION_INTERVAL;
};
using LOOP_EVALUATION_CONFIG = rlt::rl::loop::steps::evaluation::Config<LOOP_EXTRACK_CONFIG, LOOP_EVAL_PARAMETERS<LOOP_EXTRACK_CONFIG>>; // Evaluates the policy in a fixed interval and logs the return
struct LOOP_SAVE_TRAJECTORIES_PARAMETERS: rlt::rl::loop::steps::save_trajectories::Parameters<TYPE_POLICY, TI, LOOP_EVALUATION_CONFIG>{
    static constexpr TI INTERVAL_TEMP = LOOP_CORE_CONFIG::CORE_PARAMETERS::STEP_LIMIT / 3;
    static constexpr TI INTERVAL = INTERVAL_TEMP == 0 ? 1 : INTERVAL_TEMP;
    static constexpr TI NUM_EPISODES = 1;
};
using LOOP_SAVE_TRAJECTORIES_CONFIG = rlt::rl::loop::steps::save_trajectories::Config<LOOP_EVALUATION_CONFIG, LOOP_SAVE_TRAJECTORIES_PARAMETERS>; // Saves trajectories for replay with the extrack UI
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_SAVE_TRAJECTORIES_CONFIG>;
using LOOP_CONFIG = LOOP_TIMING_CONFIG;

using LOOP_STATE = typename LOOP_CONFIG::template State<LOOP_CONFIG>;


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    DEVICE device;
    TI seed = 0;
    LOOP_STATE ls;

    rlt::malloc(device, ls);

    ls.extrack_config.name = "SelfBalancingRobot_PPO";
    ls.envs->ros = std::make_shared<ROSInterface<T>>("sbr_rl_agent");
    ls.env_eval.ros = ls.envs->ros;

    RCLCPP_INFO(rclcpp::get_logger("rl_main"), "Waiting for Gazebo World Control service...");

    // Ensure the reset service is available before starting the RL loop - Create a utility function that does this automatically?
    if (ls.envs->ros->controlClient) {
        while (!ls.envs->ros->controlClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rl_main"), "Interrupted while waiting for Gazebo. Exiting.");
                return 0;
            }
            RCLCPP_WARN(rclcpp::get_logger("rl_main"), "Service not available yet. Is 'ros_gz_bridge' running?");
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rl_main"), "Gazebo connected! Starting RL loop.");

    rclcpp::Rate loop_rate(100.0);  // Run step loop at 100 Hz

    rlt::init(device, ls, seed);

    while(!rlt::step(device, ls)){
        loop_rate.sleep();
    }
    
    return 0;
}