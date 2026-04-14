#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class ModelSpawner : public rclcpp::Node
{
public:
  ModelSpawner()
  : Node("model_spawner")
  {
    marker_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&ModelSpawner::publish_marker, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing workpiece marker on /visualization_marker in frame 'base_link'.");
  }

private:
  void publish_marker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "workpiece";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Place the object in front of the robot at a reachable height.
    marker.pose.position.x = 1.10;
    marker.pose.position.y = 0.00;
    marker.pose.position.z = 0.20;

    // Rotate the mesh so it sits in a more natural orientation in front of the robot.
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.70710678;
    marker.pose.orientation.w = 0.70710678;

    // The mesh bounding box is already around 1.35 x 1.44 x 1.38 units,
    // so keeping unit scale makes it visible in RViz.
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.85F;
    marker.color.g = 0.85F;
    marker.color.b = 0.90F;
    marker.color.a = 1.0F;

    marker.mesh_resource = "package://model_spawner/meshes/tie.stl";
    marker.mesh_use_embedded_materials = false;
    marker.lifetime = rclcpp::Duration(0, 0);

    marker_publisher_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModelSpawner>());
  rclcpp::shutdown();
  return 0;
}
