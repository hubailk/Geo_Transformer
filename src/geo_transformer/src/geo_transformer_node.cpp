#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "geo_transformer/srv/local_coordinate_set.hpp"
#include "geo_transformer/srv/from_ll.hpp"
#include <limits>

class GeoTransformerNode : public rclcpp::Node {
public:
    GeoTransformerNode() : Node("geo_transformer") {
        // Initialize services
        local_coord_service_ = create_service<geo_transformer::srv::LocalCoordinateSet>(
            "local_coordinate_set",
            std::bind(&GeoTransformerNode::handle_set_local, this, std::placeholders::_1, std::placeholders::_2));
            
        from_ll_service_ = create_service<geo_transformer::srv::FromLL>(
            "from_ll",
            std::bind(&GeoTransformerNode::handle_from_ll, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_set_local(
        const std::shared_ptr<geo_transformer::srv::LocalCoordinateSet::Request> request,
        std::shared_ptr<geo_transformer::srv::LocalCoordinateSet::Response> response)
    {
        try {
            // Initialize local cartesian coordinate system with the given origin
            local_cartesian_.Reset(request->latitude, request->longitude, request->altitude);
            response->success = true;
            response->message = "Local coordinate system origin set successfully";
            RCLCPP_INFO(get_logger(), "Set local origin at lat=%.6f, lon=%.6f, alt=%.2f",
                       request->latitude, request->longitude, request->altitude);
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Error setting local origin: ") + e.what();
            RCLCPP_ERROR(get_logger(), "Error setting local origin: %s", e.what());
        }
    }

    void handle_from_ll(
        const std::shared_ptr<geo_transformer::srv::FromLL::Request> request,
        std::shared_ptr<geo_transformer::srv::FromLL::Response> response)
    {
        try {
            // Convert lat/lon/alt to local cartesian coordinates
            local_cartesian_.Forward(request->latitude, request->longitude, request->altitude,
                                   response->x, response->y, response->z);
            RCLCPP_INFO(get_logger(), "Converted lat=%.6f, lon=%.6f, alt=%.2f to x=%.2f, y=%.2f, z=%.2f",
                       request->latitude, request->longitude, request->altitude,
                       response->x, response->y, response->z);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error converting coordinates: %s", e.what());
            // Set invalid values to indicate error
            response->x = response->y = response->z = std::numeric_limits<double>::quiet_NaN();
        }
    }

    GeographicLib::LocalCartesian local_cartesian_;
    rclcpp::Service<geo_transformer::srv::LocalCoordinateSet>::SharedPtr local_coord_service_;
    rclcpp::Service<geo_transformer::srv::FromLL>::SharedPtr from_ll_service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeoTransformerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
