#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"
#include "matplot/matplot.h"
#include "self_balancing_robot/robot_utils.hpp"

using namespace std::chrono_literals;

class RobotControlPID : public rclcpp::Node
{
public:
  RobotControlPID()
  : Node("robot_control_pid")
  {
    controlClient = this->create_client<ros_gz_interfaces::srv::ControlWorld>("/control");
    imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 1, std::bind(&RobotControlPID::imu_callback, this, std::placeholders::_1));
    
    pauseSimulation(false);

    timer = this->create_wall_timer(
      std::chrono::milliseconds(1000/callbackHz),
      std::bind(&RobotControlPID::control_loop, this));

    cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  /* Store the latest IMU message to be used in the control loop */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latestImuMsg = *msg;
    isImuData = true;
  }

  /* The control loop to keep the robot balanced */
  void control_loop(){
    // if there is IMU data, then carry on with the control logic
    if (isImuData){
      double roll, pitch, yaw;  // currently only the pitch is used
      // Convert the orientation data from the IMU from quaternion to euler angles
      // <double> type used to keep functionality the same as before (before templating)
      robot_utils::quaternion_to_euler<double>(latestImuMsg.orientation.x, latestImuMsg.orientation.y, latestImuMsg.orientation.z, latestImuMsg.orientation.w, roll, pitch, yaw);

      if (isFirstIteration) {
        lastTime = std::chrono::steady_clock::now();
        isFirstIteration = false;
      }

      if (robot_utils::checkEpisodeFinished<double>(pitch)) {
        pauseSimulation(true);
        timer->cancel();  // stop the control loop
        RCLCPP_INFO(this->get_logger(), "The Robot Has Fallen. Stopping the simulation.");

        // Plot the pitch values
        matplot::plot(pitch_values);
        matplot::title("Pitch Values");
        matplot::xlabel("Points");
        matplot::ylabel("Pitch (radians)");

        matplot::show();
      } else {  // the robot has not fallen so continue with control logic.
        pitch_values.push_back(pitch);
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedTime = currentTime - lastTime;

        double dt = elapsedTime.count();

        // These parameters are not optimal but are sufficient to demonstrate.
        double KP = 260.0;
        double KI = 8.0;
        double KD = 0.1; 

        // Set/increment the error values
        double error = 0.0 - pitch;
        double errorDerivative = (error - previousError) / dt;
        errorIntegral += error * dt;

        double correction = -KP * error - KI * errorIntegral - KD * errorDerivative;

        previousError = error;
      
        // limit the angular velocity within the specified range.
        double requestedAngularVelocity = std::clamp(correction, -max_angular_velocity, max_angular_velocity);

        geometry_msgs::msg::Twist cmdVelMsg;
        cmdVelMsg.angular.z = requestedAngularVelocity; 
        cmdVelPub->publish(cmdVelMsg);

        RCLCPP_INFO(this->get_logger(), "Pitch: '%f' radians, '%f' angular velocity set", pitch, requestedAngularVelocity);

        lastTime = currentTime;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No IMU data received yet.");
    }
  }

  void pauseSimulation(bool pause){
    if(!controlClient->wait_for_service(5s)) {
      RCLCPP_WARN(this->get_logger(), "Service not available after waiting");
      return;
    }
    auto request = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    request->world_control.pause = pause; 
    auto future = controlClient->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Pause=%d for /world/empty/control", pause);
  }

  rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr controlClient;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;
  rclcpp::TimerBase::SharedPtr timer;
  sensor_msgs::msg::Imu latestImuMsg;
  bool isImuData = false;
  int callbackHz = 100;  // control loop frequency in Hz
  double errorIntegral = 0.0;
  double previousError = 0.0;
  double max_angular_velocity = 5.0;  // maximum angular velocity in rad/s
  std::vector<double> pitch_values;
  std::chrono::steady_clock::time_point lastTime;
  bool isFirstIteration = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotControlPID>());
  rclcpp::shutdown();
  return 0;
}
