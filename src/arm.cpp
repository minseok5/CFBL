#include "rclcpp/rclcpp.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"

class ArmClientNode : public rclcpp::Node
{
public:
    ArmClientNode()
    : Node("arm_client_node")
    {
        client_ = this->create_client<crazyflie_interfaces::srv::Arm>("/cf231/arm");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /cf1/arm service...");
            if (!rclcpp::ok()) return;
        }

        // Start arming and disarming in a separate thread
        thread_ = std::thread([this]() {
            sendArmRequest(true);
            rclcpp::sleep_for(std::chrono::seconds(3));
            sendArmRequest(false);

            // Shutdown the node after finishing
            rclcpp::shutdown();
        });
    }

    ~ArmClientNode()
    {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    void sendArmRequest(bool arm)
{
    auto request = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
    request->arm = arm;

    auto future = client_->async_send_request(request);
    // Wait for the result without spinning again
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
    {
        RCLCPP_INFO(this->get_logger(), arm ? "Drone armed." : "Drone disarmed.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call arm service.");
    }
}


    rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedPtr client_;
    std::thread thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmClientNode>();
    rclcpp::spin(node);  // Only spin once
    return 0;
}














// #include <memory>  // For smart pointers
// #include <chrono>  // For time durations
// #include <cmath>   // For math operations
// #include <rclcpp/rclcpp.hpp>  // ROS2 C++ client library
// #include "crazyflie_interfaces/srv/arm.hpp"  // Arm service definition
// #include "crazyflie_interfaces/srv/takeoff.hpp"  // Takeoff service definition
// #include "crazyflie_interfaces/msg/hover.hpp"  // Hover message definition (possible typo: should be 'interfaces')
// #include "builtin_interfaces/msg/duration.hpp"  // For builtin_interfaces::msg::Duration

// using std::placeholders::_1;
// using std::placeholders::_2;

// // Main node class for drone control
// class DroneControlNode : public rclcpp::Node {
// public:
//   // Constructor: initializes publishers, service clients, and timer
//   DroneControlNode() : Node("drone_control_node") {
//     hover_pub_ = this->create_publisher<crazyflie_interfaces::msg::Hover>(
//       "hover", 10);  // Publisher for hover commands

//     arm_client_ = this->create_client<crazyflie_interfaces::srv::Arm>("arm");  // Client for arming/disarming
//     takeoff_client_ = this->create_client<crazyflie_interfaces::srv::Takeoff>("takeoff");  // Client for takeoff

//     timer_ = this->create_wall_timer(
//       std::chrono::seconds(1),
//       std::bind(&DroneControlNode::executeSequence, this));  // Timer to run the sequence
//   }

// private:
//   rclcpp::TimerBase::SharedPtr timer_;  // Timer for sequencing actions
//   rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr hover_pub_;  // Publisher for hover messages
//   rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedPtr arm_client_;  // Service client for arming/disarming
//   rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr takeoff_client_;  // Service client for takeoff
//   int step_ = 0;  // Step counter for the sequence

//   // Main sequence logic, called by timer
//   void executeSequence() {
//     switch (step_) {
//       case 0:
//         RCLCPP_INFO(this->get_logger(), "Arming...");
//         callArmService(true);  // Arm the drone
//         break;
//       case 1:
//         RCLCPP_INFO(this->get_logger(), "Taking off...");
//         callTakeoffService(1.0, 2.0);  // Take off to 1m in 2s
//         break;
//       case 2:
//         RCLCPP_INFO(this->get_logger(), "Hovering...");
//         publishHover(0.0, 0.0, 0.0, 0.5);  // Hover in place, z=0.5 m/s
//         break;
//       case 3:
//         RCLCPP_INFO(this->get_logger(), "Landing...");
//         publishHover(0.0, 0.0, 0.0, 0.0);  // Stop thrust to descend
//         break;
//       case 4:
//         RCLCPP_INFO(this->get_logger(), "Disarming...");
//         callArmService(false);  // Disarm the drone
//         break;
//       case 5:
//         RCLCPP_INFO(this->get_logger(), "Sequence complete.");
//         timer_->cancel();  // Stop the timer after sequence is done
//         break;
//     }
//     step_++;  // Move to next step
//   }

//   // Call the arm/disarm service
//   void callArmService(bool arm) {
//     if (!arm_client_->wait_for_service(std::chrono::seconds(2))) {
//       RCLCPP_ERROR(this->get_logger(), "Arm service not available");
//       return;
//     }
//     auto request = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
//     request->arm = arm;  // Set arm/disarm value

//     auto result = arm_client_->async_send_request(request);  // Send async request
//   }

//   // Call the takeoff service
//   void callTakeoffService(float height, float duration) {
//     if (!takeoff_client_->wait_for_service(std::chrono::seconds(2))) {
//       RCLCPP_ERROR(this->get_logger(), "Takeoff service not available");
//       return;
//     }
//     auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
//     request->height = height;      // Desired takeoff height
//     request->duration.sec = static_cast<int32_t>(duration);  // Whole seconds
//     request->duration.nanosec = static_cast<uint32_t>((duration - static_cast<int32_t>(duration)) * 1e9);  // Fractional seconds

//     auto result = takeoff_client_->async_send_request(request);  // Send async request
//   }

//   // Publish a hover message
//   void publishHover(float vx, float vy, float yawrate, float z) {
//     crazyflie_interfaces::msg::Hover hover;
//     hover.vx = vx;           // X velocity
//     hover.vy = vy;           // Y velocity
//     hover.yaw_rate = yawrate; // Yaw rate
//     hover.z_distance = z;    // Z distance or thrust
//     hover_pub_->publish(hover);  // Publish hover command
//   }
// };

// // Main function: initialize and spin the node
// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);  // Initialize ROS2
//   rclcpp::spin(std::make_shared<DroneControlNode>());  // Start node event loop
//   rclcpp::shutdown();  // Shutdown ROS2
//   return 0;
// }
