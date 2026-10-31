// Bridge ROS 2 sea-state commands into Gazebo Fortress.
//
//  * <model>/environment/wrench (geometry_msgs/Wrench, world frame) is applied
//    to <model>::<link> through the ApplyLinkWrench system as a persistent
//    wrench, so wind force, wind yaw moment and wave heave/pitch act on the
//    hull every physics step.
//  * environment/markers (geometry_msgs/PoseArray) moves the visual-only
//    models named in `marker_names` with the UserCommands set_pose service.
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ignition/msgs/marker.pb.h>
#include <geometry_msgs/msg/wrench.hpp>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/entity_wrench.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

class SeaStateGzBridge : public rclcpp::Node
{
public:
  SeaStateGzBridge()
  : rclcpp::Node("sea_state_gz_bridge")
  {
    world_ = declare_parameter<std::string>("world", "asv_research_pool");
    link_ = declare_parameter<std::string>("link", "base_footprint");
    models_ = declare_parameter<std::vector<std::string>>(
      "models", std::vector<std::string>{"research_catamaran"});
    if (declare_parameter<bool>("ghost", false)) {
      models_.push_back("ghost_catamaran");
    }
    marker_names_ = declare_parameter<std::vector<std::string>>(
      "marker_names", std::vector<std::string>{});
    marker_rate_ = declare_parameter<double>("marker_rate", 8.0);

    wrench_pub_ = gz_.Advertise<ignition::msgs::EntityWrench>(
      "/world/" + world_ + "/wrench/persistent");
    for (const auto & model : models_) {
      subs_.push_back(create_subscription<geometry_msgs::msg::Wrench>(
        "/" + model + "/environment/wrench", 10,
        [this, model](const geometry_msgs::msg::Wrench::SharedPtr msg) {
          // ApplyLinkWrench *accumulates* persistent wrenches, so publish only
          // the change from the last commanded value; the running sum then
          // equals the wrench we want.
          std::array<double, 6> now{
            msg->force.x, msg->force.y, msg->force.z,
            msg->torque.x, msg->torque.y, msg->torque.z};
          auto & last = last_[model];
          std::array<double, 6> delta;
          bool changed = false;
          for (int i = 0; i < 6; ++i) {
            delta[i] = now[i] - last[i];
            changed = changed || std::abs(delta[i]) > 1e-9;
          }
          if (!changed) {
            return;
          }
          last = now;
          ignition::msgs::EntityWrench out;
          out.mutable_entity()->set_name(model + "::" + link_);
          out.mutable_entity()->set_type(ignition::msgs::Entity::LINK);
          auto * w = out.mutable_wrench();
          w->mutable_force()->set_x(delta[0]);
          w->mutable_force()->set_y(delta[1]);
          w->mutable_force()->set_z(delta[2]);
          w->mutable_torque()->set_x(delta[3]);
          w->mutable_torque()->set_y(delta[4]);
          w->mutable_torque()->set_z(delta[5]);
          wrench_pub_.Publish(out);
        }));
    }
    if (!marker_names_.empty()) {
      marker_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "/research_catamaran/environment/markers", 10,
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
          const auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration<double>(now - last_marker_).count() <
            1.0 / marker_rate_)
          {
            return;
          }
          last_marker_ = now;
          const std::size_t count =
          std::min(msg->poses.size(), marker_names_.size());
          for (std::size_t i = 0; i < count; ++i) {
            ignition::msgs::Pose req;
            req.set_name(marker_names_[i]);
            const auto & p = msg->poses[i];
            req.mutable_position()->set_x(p.position.x);
            req.mutable_position()->set_y(p.position.y);
            req.mutable_position()->set_z(p.position.z);
            req.mutable_orientation()->set_x(p.orientation.x);
            req.mutable_orientation()->set_y(p.orientation.y);
            req.mutable_orientation()->set_z(p.orientation.z);
            req.mutable_orientation()->set_w(p.orientation.w);
            gz_.Request(
              "/world/" + world_ + "/set_pose", req,
              &SeaStateGzBridge::ignore_reply, this);
          }
        });
    }
    marker_array_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/research_catamaran/environment/gz_markers", 10,
      [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        for (const auto & m : msg->markers) {
          draw_ros_marker(m);
        }
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/research_catamaran/station_keeping/visualization/actual_path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_trail_).count() < 0.5) {
          return;
        }
        last_trail_ = now;
        // Keep the newest 400 points so the trail stays cheap to redraw.
        const std::size_t n = msg->poses.size();
        const std::size_t first = n > 400 ? n - 400 : 0;
        std::vector<std::array<double, 3>> pts;
        for (std::size_t i = first; i < n; ++i) {
          const auto & p = msg->poses[i].pose.position;
          pts.push_back({p.x, p.y, 0.03});
        }
        send_line("trail", 1, pts, 0.16, 0.47, 0.84, 1.0, 0.02);
      });
    RCLCPP_INFO(
      get_logger(), "Applying wrenches to %zu model(s); %zu marker(s).",
      models_.size(), marker_names_.size());
  }

  ~SeaStateGzBridge() override
  {
    // Clear persistent wrenches so a stopped bridge leaves the hull free.
    ignition::msgs::Entity clear;
    auto pub = gz_.Advertise<ignition::msgs::Entity>(
      "/world/" + world_ + "/wrench/clear");
    for (const auto & model : models_) {
      clear.set_name(model + "::" + link_);
      clear.set_type(ignition::msgs::Entity::LINK);
      pub.Publish(clear);
    }
  }

private:
  static void set_colour(
    ignition::msgs::Marker & m, double r, double g, double b, double a)
  {
    auto * material = m.mutable_material();
    for (auto * c : {material->mutable_ambient(), material->mutable_diffuse(),
        material->mutable_emissive()})
    {
      c->set_r(r); c->set_g(g); c->set_b(b); c->set_a(a);
    }
  }

  void send_line(
    const std::string & ns, int id,
    const std::vector<std::array<double, 3>> & pts,
    double r, double g, double b, double a, double width)
  {
    ignition::msgs::Marker m;
    m.set_ns(ns);
    m.set_id(id);
    m.set_action(ignition::msgs::Marker::ADD_MODIFY);
    m.set_type(ignition::msgs::Marker::LINE_STRIP);
    m.set_visibility(ignition::msgs::Marker::GUI);
    m.mutable_pose()->mutable_orientation()->set_w(1.0);
    m.mutable_scale()->set_x(width);
    set_colour(m, r, g, b, a);
    for (const auto & p : pts) {
      auto * q = m.add_point();
      q->set_x(p[0]); q->set_y(p[1]); q->set_z(p[2]);
    }
    gz_.Request("/marker", m);
  }

  void draw_ros_marker(const visualization_msgs::msg::Marker & rm)
  {
    using Rm = visualization_msgs::msg::Marker;
    const double r = rm.color.r, g = rm.color.g, b = rm.color.b, a = rm.color.a;
    if (rm.type == Rm::LINE_STRIP) {
      std::vector<std::array<double, 3>> pts;
      for (const auto & p : rm.points) {
        pts.push_back({p.x, p.y, p.z});
      }
      send_line(rm.ns, rm.id, pts, r, g, b, a, std::max(rm.scale.x, 0.01));
    } else if (rm.type == Rm::ARROW && rm.points.size() == 2) {
      // Shaft plus two barbs, drawn as one polyline.
      const auto & s = rm.points[0];
      const auto & e = rm.points[1];
      const double dx = e.x - s.x, dy = e.y - s.y;
      const double len = std::max(std::hypot(dx, dy), 1e-6);
      const double ux = dx / len, uy = dy / len;
      const double head = std::min(0.25 * len, 0.25);
      auto barb = [&](double sign) {
          const double c = std::cos(2.6), sn = std::sin(2.6 * sign);
          return std::array<double, 3>{
          e.x + head * (ux * c - uy * sn), e.y + head * (ux * sn + uy * c),
          e.z};
        };
      send_line(
        rm.ns, rm.id,
        {{s.x, s.y, s.z}, {e.x, e.y, e.z}, barb(1.0), {e.x, e.y, e.z},
          barb(-1.0)},
        r, g, b, a, std::max(rm.scale.x, 0.02));
    } else if (rm.type == Rm::TEXT_VIEW_FACING) {
      ignition::msgs::Marker m;
      m.set_ns(rm.ns);
      m.set_id(rm.id);
      m.set_action(ignition::msgs::Marker::ADD_MODIFY);
      m.set_type(ignition::msgs::Marker::TEXT);
      m.set_visibility(ignition::msgs::Marker::GUI);
      m.set_text(rm.text);
      m.mutable_scale()->set_x(rm.scale.z);
      m.mutable_scale()->set_y(rm.scale.z);
      m.mutable_scale()->set_z(rm.scale.z);
      m.mutable_pose()->mutable_position()->set_x(rm.pose.position.x);
      m.mutable_pose()->mutable_position()->set_y(rm.pose.position.y);
      m.mutable_pose()->mutable_position()->set_z(rm.pose.position.z);
      m.mutable_pose()->mutable_orientation()->set_w(1.0);
      set_colour(m, r, g, b, a);
      gz_.Request("/marker", m);
    }
  }

  void ignore_reply(const ignition::msgs::Boolean &, const bool) {}

  std::map<std::string, std::array<double, 6>> last_;
  std::string world_, link_;
  std::vector<std::string> models_, marker_names_;
  double marker_rate_{8.0};
  ignition::transport::Node gz_;
  ignition::transport::Node::Publisher wrench_pub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr> subs_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr marker_sub_;
  std::chrono::steady_clock::time_point last_marker_{};
  std::chrono::steady_clock::time_point last_trail_{};
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_array_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeaStateGzBridge>());
  rclcpp::shutdown();
  return 0;
}
