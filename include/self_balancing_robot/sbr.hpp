// SBR implementations for RLTools (e.g. state, specification)

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

template <typename T>
struct SBRParameters {
    constexpr static T MAX_ANGULAR_VELOCITY = 2.0;
};

template <typename T_T, typename T_TI, typename T_PARAMETERS = SBRParameters<T_T>>
struct SBRSpecification {
    using T = T_T;
    using TI = T_TI;
    using PARAMETERS = T_PARAMETERS; // no different parameters currently - T_PARAMETERS;
};

template <typename T, typename TI>
struct SBRState {  // State - can contain a 'world' representation of the environment
    static constexpr TI STATE_DIM = 1;
    T theta;
};

template <typename TI>
struct SBRObservation {  // Observation - what the agent actually sees
    static constexpr TI DIM = 1;
};

template <typename T>  // Check if this template is necessary
struct ROSInterface {
    rclcpp::Node::SharedPtr node;
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr controlClient;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockSub;

    sensor_msgs::msg::Imu latestImuMsg;
    bool new_imu_data = false;

    rosgraph_msgs::msg::Clock latestClockMsg;
    bool newClockData = false;

    ROSInterface(const std::string& node_name = "sbr_rl_agent") {
        node = std::make_shared<rclcpp::Node>(node_name);
        controlClient = node->create_client<ros_gz_interfaces::srv::ControlWorld>("/control");
        cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        imuSub = node->create_subscription<sensor_msgs::msg::Imu>("/imu", 1, std::bind(&ROSInterface::imu_callback, this, std::placeholders::_1));
        clockSub = node->create_subscription<rosgraph_msgs::msg::Clock>("/clock", 10, std::bind(&ROSInterface::clockCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node->get_logger(), "ROS Interface Initialized");
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        latestImuMsg = *msg;
        new_imu_data = true;
    }

    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
        latestClockMsg = *msg;
        newClockData = true;
    }
    
    T getPitchFromIMU() {
        new_imu_data = false;

        while (!new_imu_data) {
            rclcpp::spin_some(node);
        }

        // TODO: Need to ensure latestImuMsg is not changed while we read it.
        T roll, pitch, yaw;
        robot_utils::quaternion_to_euler<T>(
            latestImuMsg.orientation.x,
            latestImuMsg.orientation.y,
            latestImuMsg.orientation.z,
            latestImuMsg.orientation.w,
            roll,
            pitch,
            yaw
        );
        // RCLCPP_INFO(node->get_logger(), "NOW: %f, Latest IMU header stamp: %d.%d", node->now().seconds(), latestImuMsg.header.stamp.sec, latestImuMsg.header.stamp.nanosec);
        return pitch;
    }
    bool terminateEpisode(T pitch) {
        if(std::abs(pitch) > angles::from_degrees(80.0)){
            return true;
        } else {
            return false;
        }
    }
};

template <typename T_SPEC>
struct SBREnvironment : rl_tools::rl::environments::Environment<typename T_SPEC::T, typename T_SPEC::TI> {
    using SPEC = T_SPEC;
    using T = typename SPEC::T;
    using TI = typename SPEC::TI;
    using Parameters = typename SPEC::PARAMETERS;
    using State = SBRState<T, TI>;
    using Observation = SBRObservation<TI>;
    using ObservationPrivileged = Observation;
    static constexpr TI OBSERVATION_DIM = 1;
    static constexpr TI ACTION_DIM = 1;
    static constexpr TI EPISODE_STEP_LIMIT = 500;

    std::shared_ptr<ROSInterface<T>> ros;
    mutable int current_episode = -1;
    mutable int currentStepCount = 0;
};



