#include <map>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "DataStreamClient.h"

namespace vicon = ViconDataStreamSDK::CPP;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TF2Stamped = geometry_msgs::msg::TransformStamped;

struct ViconNode : public rclcpp::Node {
  vicon::Client client;

  std::string hostname;
  size_t buffer_size;
  std::string ns_name;
  std::map<std::string, rclcpp::Publisher<PoseStamped>::SharedPtr> pub_pose_map;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  ViconNode() : Node{"vicon_node"} {
    declare_parameter<std::string>("hostname", "127.0.0.1");
    declare_parameter<int>("buffer_size", 200);
    declare_parameter<std::string>("namespace", "vicon");
    get_parameter("hostname", hostname);
    get_parameter("buffer_size", buffer_size);
    get_parameter("namespace", ns_name);

    std::signal(SIGINT, &ViconNode::signal_handler);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  static void signal_handler(int signum) { rclcpp::shutdown(); }

  bool connect() {
    // Connect to server
    RCLCPP_INFO(rclcpp::get_logger("vicon_node"), "Connecting to [%s] ...",
                hostname.c_str());
    const int num_retries = 5;
    for (int i = 0; i < num_retries; i++) {
      if (client.IsConnected().Connected) {
        break;
      }
      if (client.Connect(hostname).Result != vicon::Result::Success) {
        RCLCPP_WARN(rclcpp::get_logger("vicon_node"),
                    "Failed to connect, retrying ...");
        sleep(1);
      }
    }

    // Check connection
    if (client.Connect(hostname).Result != vicon::Result::Success) {
      RCLCPP_FATAL(rclcpp::get_logger("vicon_node"),
                   "Failed to connect to vicon");
      return false;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("vicon_node"), "Connected!");
    }

    // Perform further initialization
    client.EnableSegmentData();
    client.EnableMarkerData();
    client.EnableUnlabeledMarkerData();
    client.EnableMarkerRayData();
    client.EnableDeviceData();
    client.EnableDebugData();
    client.SetStreamMode(vicon::StreamMode::ClientPull);
    client.SetBufferSize(buffer_size);

    return true;
  }

  bool disconnect() {
    if (!client.IsConnected().Connected) {
      return true;
    }

    sleep(1);
    client.DisableSegmentData();
    client.DisableMarkerData();
    client.DisableUnlabeledMarkerData();
    client.DisableDeviceData();
    client.DisableCentroidData();

    RCLCPP_INFO(rclcpp::get_logger("vicon_node"), "Disconnecting from [%s] ...",
                hostname.c_str());
    client.Disconnect();

    if (!client.IsConnected().Connected) {
      RCLCPP_INFO(rclcpp::get_logger("vicon_node"),
                  "Successfully disconnected!");
      return true;
    }

    RCLCPP_FATAL(rclcpp::get_logger("vicon_node"), "Failed to disconnect!");
    return false;
  }

  void get_frame() {
    client.GetFrame();
    const size_t sub_count = client.GetSubjectCount().SubjectCount;

    for (size_t sub_index = 0; sub_index < sub_count; ++sub_index) {
      const std::string sub_name = client.GetSubjectName(sub_index).SubjectName;
      const auto seg_count = client.GetSegmentCount(sub_name).SegmentCount;

      for (size_t seg_index = 0; seg_index < seg_count; ++seg_index) {
        const std::string seg_name =
            client.GetSegmentName(sub_name, seg_index).SegmentName;
        publish(sub_name, seg_name);
      }
    }
  }

  void publish(const std::string &sub_name, const std::string &seg_name) {
    const std::string topic_name = sub_name + "/" + seg_name;
    if (pub_pose_map.count(topic_name)) {
      // clang-format off
      const auto trans = client.GetSegmentGlobalTranslation(sub_name, seg_name);
      const auto pos_x = trans.Translation[0] * 1e-3;  // Convert mm to m
      const auto pos_y = trans.Translation[1] * 1e-3;  // Convert mm to m
      const auto pos_z = trans.Translation[2] * 1e-3;  // Convert mm to m
      const auto rot = client.GetSegmentGlobalRotationQuaternion(sub_name, seg_name);
      const auto qx = rot.Rotation[0];
      const auto qy = rot.Rotation[1];
      const auto qz = rot.Rotation[2];
      const auto qw = rot.Rotation[3];
      // clang-format on

      // Publish pose stamped message
      PoseStamped msg;
      msg.header.frame_id = sub_name + "/" + seg_name;
      msg.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
      msg.pose.position.x = pos_x;
      msg.pose.position.y = pos_y;
      msg.pose.position.z = pos_z;
      msg.pose.orientation.x = qx;
      msg.pose.orientation.y = qy;
      msg.pose.orientation.z = qz;
      msg.pose.orientation.w = qw;
      pub_pose_map[topic_name]->publish(msg);

      // Publish pose stamped message
      TF2Stamped tf2_msg;
      tf2_msg.header.frame_id = "map";
      tf2_msg.child_frame_id = sub_name + "/" + seg_name;
      tf2_msg.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
      tf2_msg.transform.translation.x = pos_x;
      tf2_msg.transform.translation.y = pos_y;
      tf2_msg.transform.translation.z = pos_z;
      tf2_msg.transform.rotation.x = qx;
      tf2_msg.transform.rotation.y = qy;
      tf2_msg.transform.rotation.z = qz;
      tf2_msg.transform.rotation.w = qw;
      tf_broadcaster->sendTransform(tf2_msg);

    } else if (pub_pose_map.count(topic_name) == 0) {
      const std::string topic_name = ns_name + "/" + sub_name + "/" + seg_name;
      const std::string key = sub_name + "/" + seg_name;
      RCLCPP_INFO(rclcpp::get_logger("vicon_node"), "Creating publisher [%s]",
                  key.c_str());
      pub_pose_map[key] = create_publisher<PoseStamped>(topic_name, 1);
    }
  }
};

int main(int argc, char **argv) {
  // Initialize ROS Node
  rclcpp::init(argc, argv);

  // Connect and loop Vicon Node
  ViconNode node;
  node.connect();
  while (rclcpp::ok()) {
    node.get_frame();
  }

  // Disconnect
  node.disconnect();
  rclcpp::shutdown();

  return 0;
}
