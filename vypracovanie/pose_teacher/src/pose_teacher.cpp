#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include <abb_irb4600_ikfast/abb_irb4600_ikfast.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class PoseTeacher : public rclcpp::Node
{
public:
  PoseTeacher()
  : Node("pose_teacher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    marker_initialized_(false),
    current_joint_values_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    has_valid_joint_solution_(true)
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    tool_frame_ = this->declare_parameter<std::string>("tool_frame", "tool0");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);

    marker_server_ =
      std::make_shared<interactive_markers::InteractiveMarkerServer>("gizmo", this);
    joint_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    initialize_timer_ =
      this->create_wall_timer(200ms, std::bind(&PoseTeacher::try_initialize_marker, this));
    joint_state_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&PoseTeacher::publish_joint_state, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Pose teacher is waiting for TF from '%s' to '%s'.",
      base_frame_.c_str(),
      tool_frame_.c_str());
  }

private:
  void try_initialize_marker()
  {
    if (marker_initialized_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tool_transform;
    try {
      tool_transform = tf_buffer_.lookupTransform(base_frame_, tool_frame_, rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Still waiting for TF %s -> %s: %s",
        base_frame_.c_str(),
        tool_frame_.c_str(),
        ex.what());
      return;
    }

    auto marker = make_gizmo_marker(tool_transform);
    marker_server_->insert(
      marker,
      std::bind(&PoseTeacher::process_feedback, this, std::placeholders::_1));
    marker_server_->applyChanges();

    marker_initialized_ = true;
    initialize_timer_->cancel();

    RCLCPP_INFO(
      this->get_logger(),
      "Interactive marker 'gizmo' was attached to '%s' in frame '%s'.",
      tool_frame_.c_str(),
      base_frame_.c_str());
  }

  void publish_joint_state()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.name = joint_names_;
    msg.position.assign(
      current_joint_values_.begin(),
      current_joint_values_.end());
    joint_state_publisher_->publish(msg);
  }

  visualization_msgs::msg::InteractiveMarker make_gizmo_marker(
    const geometry_msgs::msg::TransformStamped & tool_transform) const
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = base_frame_;
    marker.name = "gizmo";
    marker.description = "6-DOF gizmo";
    marker.scale = 0.35;

    marker.pose.position.x = tool_transform.transform.translation.x;
    marker.pose.position.y = tool_transform.transform.translation.y;
    marker.pose.position.z = tool_transform.transform.translation.z;
    marker.pose.orientation = tool_transform.transform.rotation;

    auto visual_control = make_visual_control();
    marker.controls.push_back(visual_control);

    add_axis_controls(marker);
    return marker;
  }

  visualization_msgs::msg::InteractiveMarkerControl make_visual_control() const
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;

    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = 0.95F;
    marker.color.g = 0.60F;
    marker.color.b = 0.10F;
    marker.color.a = 1.0F;

    control.markers.push_back(marker);
    return control;
  }

  void add_axis_controls(visualization_msgs::msg::InteractiveMarker & marker) const
  {
    add_control(
      marker,
      "rotate_x",
      1.0,
      1.0,
      0.0,
      0.0,
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    add_control(
      marker,
      "move_x",
      1.0,
      1.0,
      0.0,
      0.0,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);

    add_control(
      marker,
      "rotate_z",
      1.0,
      0.0,
      1.0,
      0.0,
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    add_control(
      marker,
      "move_z",
      1.0,
      0.0,
      1.0,
      0.0,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);

    add_control(
      marker,
      "rotate_y",
      1.0,
      0.0,
      0.0,
      1.0,
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS);
    add_control(
      marker,
      "move_y",
      1.0,
      0.0,
      0.0,
      1.0,
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS);
  }

  void add_control(
    visualization_msgs::msg::InteractiveMarker & marker,
    const std::string & name,
    double w,
    double x,
    double y,
    double z,
    uint8_t interaction_mode) const
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = name;
    control.orientation.w = w;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.interaction_mode = interaction_mode;
    marker.controls.push_back(control);
  }

  void process_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      solve_and_publish_ik(feedback->pose);
      return;
    }

    if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
    {
      const auto & p = feedback->pose.position;
      const auto & q = feedback->pose.orientation;
      RCLCPP_INFO(
        this->get_logger(),
        "Gizmo released at position [%.3f, %.3f, %.3f], orientation [%.3f, %.3f, %.3f, %.3f].",
        p.x,
        p.y,
        p.z,
        q.x,
        q.y,
        q.z,
        q.w);

      if (has_valid_joint_solution_) {
        RCLCPP_INFO(
          this->get_logger(),
          "Current joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
          current_joint_values_[0],
          current_joint_values_[1],
          current_joint_values_[2],
          current_joint_values_[3],
          current_joint_values_[4],
          current_joint_values_[5]);
      }
    }
  }

  void solve_and_publish_ik(const geometry_msgs::msg::Pose & target_pose)
  {
    const Eigen::Affine3d pose = pose_msg_to_eigen(target_pose);
    const auto solutions = ikfast_abb::computeIK(pose);

    if (solutions.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "IK solver did not find a valid solution for the current gizmo pose.");
      return;
    }

    const auto best_solution = pick_closest_solution(solutions, current_joint_values_);
    current_joint_values_ = best_solution;
    has_valid_joint_solution_ = true;
    publish_joint_state();
  }

  Eigen::Affine3d pose_msg_to_eigen(const geometry_msgs::msg::Pose & pose_msg) const
  {
    Eigen::Quaterniond q(
      pose_msg.orientation.w,
      pose_msg.orientation.x,
      pose_msg.orientation.y,
      pose_msg.orientation.z);

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() << pose_msg.position.x, pose_msg.position.y, pose_msg.position.z;
    pose.linear() = q.normalized().toRotationMatrix();
    return pose;
  }

  ikfast_abb::JointValues pick_closest_solution(
    const ikfast_abb::Solutions & solutions,
    const ikfast_abb::JointValues & reference) const
  {
    double best_distance = std::numeric_limits<double>::max();
    ikfast_abb::JointValues best_solution = solutions.front();

    for (const auto & solution : solutions) {
      const double distance = joint_distance(solution, reference);
      if (distance < best_distance) {
        best_distance = distance;
        best_solution = solution;
      }
    }

    return best_solution;
  }

  double joint_distance(
    const ikfast_abb::JointValues & lhs,
    const ikfast_abb::JointValues & rhs) const
  {
    double sum = 0.0;
    for (size_t i = 0; i < lhs.size(); ++i) {
      const double diff = normalize_angle(lhs[i] - rhs[i]);
      sum += diff * diff;
    }
    return sum;
  }

  double normalize_angle(double angle) const
  {
    constexpr double two_pi = 2.0 * M_PI;
    while (angle > M_PI) {
      angle -= two_pi;
    }
    while (angle < -M_PI) {
      angle += two_pi;
    }
    return angle;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr initialize_timer_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  std::string base_frame_;
  std::string tool_frame_;
  std::vector<std::string> joint_names_;
  double publish_rate_hz_;
  bool marker_initialized_;
  ikfast_abb::JointValues current_joint_values_;
  bool has_valid_joint_solution_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTeacher>());
  rclcpp::shutdown();
  return 0;
}
