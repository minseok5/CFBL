#include "rclcpp/rclcpp.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"

class ArmClientNode : public rclcpp::Node
{
public:
    ArmClientNode()
    : Node("arm_client_node"), step_(0)
    {
        client_ = this->create_client<crazyflie_interfaces::srv::Arm>("/cf231/arm");
        takeoff_client_ = this->create_client<crazyflie_interfaces::srv::Takeoff>("/cf231/takeoff");
        hover_pub_ = this->create_publisher<crazyflie_interfaces::msg::Hover>("/cf231/hover", 10);
        land_client_ = this->create_client<crazyflie_interfaces::srv::Land>("/cf231/land");

        // Wait for the services to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /cf231/arm service...");
            if (!rclcpp::ok()) return;
        }
        while (!takeoff_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /cf231/takeoff service...");
            if (!rclcpp::ok()) return;
        }
        while (!land_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /cf231/land service...");
            if (!rclcpp::ok()) return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ArmClientNode::sequenceCallback, this)
        );
    }

    ~ArmClientNode() = default;

private:
    void sendArmRequest(bool arm)
    {
        auto request = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
        request->arm = arm;
        auto future = client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
        {
            RCLCPP_INFO(this->get_logger(), arm ? "Drone armed." : "Drone disarmed.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call arm service.");
        }
    }

    void sendTakeoffRequest(float height, float duration)
    {
        auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
        request->height = height;
        request->duration.sec = static_cast<int32_t>(duration);
        request->duration.nanosec = static_cast<uint32_t>((duration - static_cast<int32_t>(duration)) * 1e9);
        auto future = takeoff_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff command sent.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call takeoff service.");
        }
    }

    void publishHover(float vx, float vy, float yawrate, float z)
    {
        crazyflie_interfaces::msg::Hover hover;
        hover.vx = vx;
        hover.vy = vy;
        hover.yaw_rate = yawrate;
        hover.z_distance = z;
        hover_pub_->publish(hover);
        RCLCPP_INFO(this->get_logger(), "Hover command published: vx=%.2f vy=%.2f yawrate=%.2f z=%.2f", vx, vy, yawrate, z);
    }

    void sendLandRequest(float height, float duration)
    {
        auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
        request->height = height;
        request->duration.sec = static_cast<int32_t>(duration);
        request->duration.nanosec = static_cast<uint32_t>((duration - static_cast<int32_t>(duration)) * 1e9);
        auto future = land_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
        {
            RCLCPP_INFO(this->get_logger(), "Land command sent.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call land service.");
        }
    }

    void sequenceCallback()
    {
        switch (step_)
        {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Arming...");
                sendArmRequest(true);
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "Taking off...");
                sendTakeoffRequest(1.0, 3.0); // 1m height, 3s duration
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "Hovering...");
                publishHover(0.0, 0.0, 0.0, 0.5); // Hover in place
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "Landing...");
                sendLandRequest(0.02, 3.0); // Land to 0.02m over 3 seconds
                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "Disarming...");
                sendArmRequest(false);
                break;
            case 5:
                RCLCPP_INFO(this->get_logger(), "Sequence complete. Shutting down.");
                timer_->cancel();
                rclcpp::shutdown();
                break;
        }
        step_++;
    }

    rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedPtr client_;
    rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr takeoff_client_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr hover_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int step_;
    float landing_z_;
    rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr land_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmClientNode>();
    rclcpp::spin(node);
    return 0;
}