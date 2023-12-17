#include <map>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "DataStreamClient.h"

namespace vicon = ViconDataStreamSDK::CPP;
using PoseStamped = geometry_msgs::msg::PoseStamped;

struct ViconNode : public rclcpp::Node {
  vicon::Client client;

  std::string hostname;
  size_t buffer_size;
  std::string ns_name;
  std::map<std::string, rclcpp::Publisher<PoseStamped>::SharedPtr> pub_map;

  ViconNode() : Node{"vicon_node"} {
    declare_parameter<std::string>("hostname", "127.0.0.1");
    declare_parameter<int>("buffer_size", 200);
    declare_parameter<std::string>("namespace", "vicon");
    get_parameter("hostname", hostname);
    get_parameter("buffer_size", buffer_size);
    get_parameter("namespace", ns_name);

    std::signal(SIGINT, &ViconNode::signal_handler);
  }

  static void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("vicon_node"), "Received signal %d", signum);
    rclcpp::shutdown();
  }

  bool connect() {
    // Connect to server
    printf("Connecting to [%s] ...\n", hostname.c_str());
    const int num_retries = 5;
    for (int i = 0; i < num_retries; i++) {
      if (client.IsConnected().Connected) {
        break;
      }
      if (client.Connect(hostname).Result != vicon::Result::Success) {
        printf("Failed to connect, retrying ...\n");
        sleep(1);
      }
    }

    // Check connection
    if (client.Connect(hostname).Result != vicon::Result::Success) {
      printf("Failed to connect to vicon\n");
      return false;
    } else {
      printf("Connected!\n");
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

    printf("Disconnecting from [%s] ...\n", hostname.c_str());
    client.Disconnect();

    if (!client.IsConnected().Connected) {
      printf("Successfully disconnected!\n");
      return true;
    }

    printf("Failed to disconnect!\n");
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
    if (pub_map.count(topic_name)) {
      // clang-format off
      const auto trans = client.GetSegmentGlobalTranslation(sub_name, seg_name);
      const auto rot = client.GetSegmentGlobalRotationQuaternion(sub_name, seg_name);
      // clang-format on

      PoseStamped msg;
      msg.header.frame_id = sub_name + "/" + seg_name;
      msg.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
      msg.pose.position.x = trans.Translation[0] * 1e-3; // Convert mm to m
      msg.pose.position.y = trans.Translation[1] * 1e-3; // Convert mm to m
      msg.pose.position.z = trans.Translation[2] * 1e-3; // Convert mm to m
      msg.pose.orientation.x = rot.Rotation[0];
      msg.pose.orientation.y = rot.Rotation[1];
      msg.pose.orientation.z = rot.Rotation[2];
      msg.pose.orientation.w = rot.Rotation[3];
      pub_map[topic_name]->publish(msg);

    } else if (pub_map.count(topic_name) == 0) {
      const std::string topic_name = ns_name + "/" + sub_name + "/" + seg_name;
      const std::string key = sub_name + "/" + seg_name;
      printf("Creating publisher [%s]\n", key.c_str());
      pub_map[key] = create_publisher<PoseStamped>(topic_name, 1);
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
