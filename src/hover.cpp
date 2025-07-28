#include "rclcpp/rclcpp.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"
#include <vector>
#include <string>

class MultiArmClientNode : public rclcpp::Node
{
public:
    MultiArmClientNode()
    : Node("multi_arm_client_node"), step_(0)
    {
        drone_ids_ = {"cf01", "cf02"};
        for (const auto& drone_id : drone_ids_) {
            arm_clients_.push_back(this->create_client<crazyflie_interfaces::srv::Arm>("/" + drone_id + "/arm"));
            takeoff_clients_.push_back(this->create_client<crazyflie_interfaces::srv::Takeoff>("/" + drone_id + "/takeoff"));
            hover_pubs_.push_back(this->create_publisher<crazyflie_interfaces::msg::Hover>("/" + drone_id + "/hover", 10));
            land_clients_.push_back(this->create_client<crazyflie_interfaces::srv::Land>("/" + drone_id + "/land"));
        }

        // Wait for all services to be available
        for (size_t i = 0; i < drone_ids_.size(); ++i) {
            while (!arm_clients_[i]->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "Waiting for /%s/arm service...", drone_ids_[i].c_str());
                if (!rclcpp::ok()) return;
            }
            while (!takeoff_clients_[i]->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "Waiting for /%s/takeoff service...", drone_ids_[i].c_str());
                if (!rclcpp::ok()) return;
            }
            while (!land_clients_[i]->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "Waiting for /%s/land service...", drone_ids_[i].c_str());
                if (!rclcpp::ok()) return;
            }
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&MultiArmClientNode::sequenceCallback, this)
        );
    }

    ~MultiArmClientNode() = default;

private:
    void sendArmRequest(bool arm)
    {
        for (size_t i = 0; i < drone_ids_.size(); ++i) {
            auto request = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
            request->arm = arm;
            auto future = arm_clients_[i]->async_send_request(request);
            if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
            {
                RCLCPP_INFO(this->get_logger(), "%s: %s", drone_ids_[i].c_str(), arm ? "Drone armed." : "Drone disarmed.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "%s: Failed to call arm service.", drone_ids_[i].c_str());
            }
        }
    }

    void sendTakeoffRequest(float height, float duration)
    {
        for (size_t i = 0; i < drone_ids_.size(); ++i) {
            auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
            request->height = height;
            request->duration.sec = static_cast<int32_t>(duration);
            request->duration.nanosec = static_cast<uint32_t>((duration - static_cast<int32_t>(duration)) * 1e9);
            auto future = takeoff_clients_[i]->async_send_request(request);
            if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
            {
                RCLCPP_INFO(this->get_logger(), "%s: Takeoff command sent.", drone_ids_[i].c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "%s: Failed to call takeoff service.", drone_ids_[i].c_str());
            }
        }
    }

    void publishHover(float vx, float vy, float yawrate, float z)
    {
        for (size_t i = 0; i < drone_ids_.size(); ++i) {
            crazyflie_interfaces::msg::Hover hover;
            hover.vx = vx;
            hover.vy = vy;
            hover.yaw_rate = yawrate;
            hover.z_distance = z;
            hover_pubs_[i]->publish(hover);
            RCLCPP_INFO(this->get_logger(), "%s: Hover command published: vx=%.2f vy=%.2f yawrate=%.2f z=%.2f", drone_ids_[i].c_str(), vx, vy, yawrate, z);
        }
    }

    void sendLandRequest(float height, float duration)
    {
        for (size_t i = 0; i < drone_ids_.size(); ++i) {
            auto request = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
            request->height = height;
            request->duration.sec = static_cast<int32_t>(duration);
            request->duration.nanosec = static_cast<uint32_t>((duration - static_cast<int32_t>(duration)) * 1e9);
            auto future = land_clients_[i]->async_send_request(request);
            if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
            {
                RCLCPP_INFO(this->get_logger(), "%s: Land command sent.", drone_ids_[i].c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "%s: Failed to call land service.", drone_ids_[i].c_str());
            }
        }
    }

    void sequenceCallback()
    {
        switch (step_)
        {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Arming drones...");
                sendArmRequest(true);
                break;
            case 1:
                RCLCPP_INFO(this->get_logger(), "Taking off drones...");
                sendTakeoffRequest(1.0, 3.0); // 1m height, 3s duration
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "Hovering drones...");
                publishHover(0.0, 0.0, 0.0, 0.5); // Hover in place
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "Landing drones...");
                sendLandRequest(0.02, 3.0); // Land to 0.02m over 3 seconds
                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "Disarming drones...");
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

    std::vector<std::string> drone_ids_;
    std::vector<rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedPtr> arm_clients_;
    std::vector<rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr> takeoff_clients_;
    std::vector<rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr> hover_pubs_;
    std::vector<rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr> land_clients_;
    rclcpp::TimerBase::SharedPtr timer_;
    int step_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiArmClientNode>();
    rclcpp::spin(node);
    return 0;
}