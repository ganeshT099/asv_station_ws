#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

#include <ignition/msgs/marker.pb.h>
#include <ignition/transport/Node.hh>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr char kReferenceTopic[] = "/asv_path_tracking/reference_path";
constexpr char kActualTopic[] = "/asv_path_tracking/actual_path";
constexpr char kGazeboMarkerTopic[] = "/marker";
constexpr char kReferenceNamespace[] = "asv_path_tracking/reference";
constexpr char kActualNamespace[] = "asv_path_tracking/actual";
}

class GazeboPathMarkerBridge : public rclcpp::Node
{
public:
  GazeboPathMarkerBridge()
  : Node("asv_path_tracking_gazebo_marker_bridge")
  {
    this->declare_parameter<double>("line_width", 0.045);
    this->declare_parameter<double>("path_z", 0.12);
    this->declare_parameter<bool>("publish_reference", true);
    this->declare_parameter<bool>("publish_actual", true);
    this->lineWidth = this->get_parameter("line_width").as_double();
    this->pathZ = this->get_parameter("path_z").as_double();
    this->publishReference = this->get_parameter("publish_reference").as_bool();
    this->publishActual = this->get_parameter("publish_actual").as_bool();

    this->referenceSub = this->create_subscription<nav_msgs::msg::Path>(
      kReferenceTopic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr message)
      {
        if (this->publishReference) {
          this->publishLine(
            kReferenceNamespace, 1001, *message, 1.0, 0.72, 0.04, 0.95);
        }
      });

    this->actualSub = this->create_subscription<nav_msgs::msg::Path>(
      kActualTopic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const nav_msgs::msg::Path::SharedPtr message)
      {
        if (this->publishActual) {
          this->publishLine(
            kActualNamespace, 1002, *message, 0.04, 0.95, 0.95, 0.98);
        }
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Gazebo path marker bridge ready on %s",
      kGazeboMarkerTopic);
  }

private:
  void publishLine(
    const std::string & namespaceName,
    std::uint64_t id,
    const nav_msgs::msg::Path & path,
    double red,
    double green,
    double blue,
    double alpha)
  {
    ignition::msgs::Marker marker;
    marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    marker.set_ns(namespaceName);
    marker.set_id(id);
    marker.set_type(ignition::msgs::Marker::LINE_STRIP);
    marker.set_visibility(ignition::msgs::Marker::GUI);
    marker.mutable_pose()->mutable_orientation()->set_w(1.0);
    marker.mutable_scale()->set_x(std::max(this->lineWidth, 0.005));

    auto * material = marker.mutable_material();
    material->mutable_ambient()->set_r(red);
    material->mutable_ambient()->set_g(green);
    material->mutable_ambient()->set_b(blue);
    material->mutable_ambient()->set_a(alpha);
    material->mutable_diffuse()->set_r(red);
    material->mutable_diffuse()->set_g(green);
    material->mutable_diffuse()->set_b(blue);
    material->mutable_diffuse()->set_a(alpha);
    material->mutable_emissive()->set_r(red * 0.45);
    material->mutable_emissive()->set_g(green * 0.45);
    material->mutable_emissive()->set_b(blue * 0.45);
    material->mutable_emissive()->set_a(alpha);

    for (const auto & pose : path.poses) {
      auto * point = marker.add_point();
      point->set_x(pose.pose.position.x);
      point->set_y(pose.pose.position.y);
      point->set_z(pose.pose.position.z + this->pathZ);
    }

    if (path.poses.size() < 2) {
      marker.set_action(ignition::msgs::Marker::DELETE_MARKER);
      marker.clear_point();
    }
    this->gazeboNode.Request(kGazeboMarkerTopic, marker);
  }

  double lineWidth{0.045};
  double pathZ{0.12};
  bool publishReference{true};
  bool publishActual{true};
  ignition::transport::Node gazeboNode;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr referenceSub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr actualSub;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPathMarkerBridge>());
  rclcpp::shutdown();
  return 0;
}
