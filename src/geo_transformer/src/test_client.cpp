#include <rclcpp/rclcpp.hpp>
#include "geo_transformer/srv/local_coordinate_set.hpp"
#include "geo_transformer/srv/from_ll.hpp"

class TestClient : public rclcpp::Node {
public:
    explicit TestClient() : Node("test_client") {
        // Create service clients
        local_coord_client_ = create_client<geo_transformer::srv::LocalCoordinateSet>(
            "local_coordinate_set");
        from_ll_client_ = create_client<geo_transformer::srv::FromLL>(
            "from_ll");

        // Start testing once node is created
        test_services();
    }

private:
    void test_services() {
        RCLCPP_INFO(get_logger(), "Waiting for services to become available...");
        
        // Wait for both services to be available
        local_coord_client_->wait_for_service();
        from_ll_client_->wait_for_service();
        
        RCLCPP_INFO(get_logger(), "Services are ready");

        // Test 1: Set local coordinate system origin at Seattle Space Needle
        {
            auto request = std::make_shared<geo_transformer::srv::LocalCoordinateSet::Request>();
            request->latitude = 47.6205;   // Space Needle latitude
            request->longitude = -122.3493; // Space Needle longitude
            request->altitude = 158.5;      // Space Needle height in meters

            RCLCPP_INFO(get_logger(), "Setting local origin to Space Needle location...");
            auto future = local_coord_client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                auto result = future.get();
                RCLCPP_INFO(get_logger(), "Set local origin result: success=%d, message=%s",
                           result->success, result->message.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to call set local origin service");
                return;
            }
        }

        // Test 2: Convert Pike Place Market coordinates to local XYZ
        {
            auto request = std::make_shared<geo_transformer::srv::FromLL::Request>();
            request->latitude = 47.6097;   // Pike Place Market latitude
            request->longitude = -122.3422; // Pike Place Market longitude
            request->altitude = 27.0;       // Approximate altitude in meters

            RCLCPP_INFO(get_logger(), "Converting Pike Place Market coordinates...");
            auto future = from_ll_client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                auto result = future.get();
                RCLCPP_INFO(get_logger(), "Pike Place Market local coordinates: x=%.2f, y=%.2f, z=%.2f",
                           result->x, result->y, result->z);
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to call coordinate conversion service");
            }
        }
    }

    rclcpp::Client<geo_transformer::srv::LocalCoordinateSet>::SharedPtr local_coord_client_;
    rclcpp::Client<geo_transformer::srv::FromLL>::SharedPtr from_ll_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
