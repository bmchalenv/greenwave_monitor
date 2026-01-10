// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "greenwave_monitor.hpp"

#include <optional>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

using namespace std::chrono_literals;
using namespace greenwave_diagnostics::constants;

GreenwaveMonitor::GreenwaveMonitor(const rclcpp::NodeOptions & options)
: Node("greenwave_monitor",
    rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  RCLCPP_INFO(this->get_logger(), "Starting GreenwaveMonitorNode");

  // Get the topics parameter (declare only if not already declared from overrides)
  if (!this->has_parameter("topics")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    this->declare_parameter<std::vector<std::string>>("topics", {""}, descriptor);
  }

  // Timer callback to publish diagnostics and print feedback
  timer_ = this->create_wall_timer(
    1s, std::bind(&GreenwaveMonitor::timer_callback, this));

  // Defer topic discovery to allow the ROS graph to settle before querying other nodes
  init_timer_ = this->create_wall_timer(
    0ms, [this]() {
      init_timer_->cancel();
      deferred_init();
    });
}

void GreenwaveMonitor::deferred_init()
{
  // Get all topics from YAML and parameters
  std::set<std::string> all_topics = get_topics_from_parameters();
  auto topics_param = this->get_parameter("topics").as_string_array();
  for (const auto & topic : topics_param) {
    if (!topic.empty()) {
      all_topics.insert(topic);
    }
  }
  // Callback for accepting/rejecting parameter changes
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&GreenwaveMonitor::on_parameter_change, this, std::placeholders::_1));
  // Subscribe to parameter events to execute pending topic additions and track external monitoring
  param_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", 10,
    std::bind(&GreenwaveMonitor::on_parameter_event, this, std::placeholders::_1));
  // Set all topics in YAML and from param to enabled by default
  for (const auto & topic : all_topics) {
    std::string message;
    this->set_parameter(rclcpp::Parameter(std::string(kTopicParamPrefix) + topic + kEnabledSuffix, true));
  }
}

void GreenwaveMonitor::topic_callback(
  const std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & topic, const std::string & type)
{
  auto msg_timestamp = GetTimestampFromSerializedMessage(msg, type);
  greenwave_diagnostics_[topic]->updateDiagnostics(msg_timestamp.time_since_epoch().count());
}

void GreenwaveMonitor::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "====================================================");
  if (greenwave_diagnostics_.empty()) {
    RCLCPP_INFO(this->get_logger(), "No topics to monitor");
  }
  for (auto & [topic, diagnostics] : greenwave_diagnostics_) {
    diagnostics->publishDiagnostics();
    RCLCPP_INFO(
      this->get_logger(), "Frame rate for topic %s: %.2f hz",
      topic.c_str(), diagnostics->getFrameRateNode());
    RCLCPP_INFO(
      this->get_logger(), "Latency for topic %s: %.2f ms",
      topic.c_str(), diagnostics->getLatency());
  }
  RCLCPP_INFO(this->get_logger(), "====================================================");
}

std::optional<std::string> parse_enabled_topic_param(const std::string & name)
{
  size_t plen = std::strlen(kTopicParamPrefix), slen = std::strlen(kEnabledSuffix);
  if (name.size() <= plen + slen || name.rfind(kTopicParamPrefix, 0) != 0 ||
    name.compare(name.size() - slen, slen, kEnabledSuffix) != 0)
  {
    return std::nullopt;
  }
  std::string topic = name.substr(plen, name.size() - plen - slen);
  return (!topic.empty() && topic[0] == '/') ? std::optional{topic} : std::nullopt;
}

rcl_interfaces::msg::SetParametersResult GreenwaveMonitor::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    // Ensure it matches greenwave_diagnostics.<topic>.enabled pattern
    auto topic_opt = parse_enabled_topic_param(param.get_name());
    if (!topic_opt.has_value()) {
      continue;
    }
    const std::string & topic = topic_opt.value();
    // Enabled parameter must be a boolean
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      continue;
    }
    // Handle enabled (enabled=true) parameter
    bool enabled = param.as_bool();
    if (enabled) {
      // Reject if already monitored internally
      if (greenwave_diagnostics_.find(topic) != greenwave_diagnostics_.end()) {
        result.reason = "Topic already monitored: " + topic;
        return result;
      }
      // Reject if no publishers found for topic
      std::vector<rclcpp::TopicEndpointInfo> publishers;
      publishers = this->get_publishers_info_by_topic(topic);
      if (publishers.empty()) {
        result.reason = "No publishers found for topic: " + topic;
        return result;
      }
      // Reject if external node already has enabled parameter for this topic
      std::string pub_node_name = publishers[0].node_name();
      if (external_topic_to_node_.find(topic) == external_topic_to_node_.end() &&
          pub_node_name != this->get_fully_qualified_name()) {
        rclcpp::NodeOptions temp_opts;
        temp_opts.start_parameter_services(false);
        temp_opts.start_parameter_event_publisher(false);
        auto temp_node = std::make_shared<rclcpp::Node>(
          "_gw_param_check", "/_greenwave_internal", temp_opts);
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, pub_node_name);
        if (param_client->wait_for_service(std::chrono::milliseconds(100)) &&
            param_client->has_parameter(std::string(kTopicParamPrefix) + topic + kEnabledSuffix)) {
          external_topic_to_node_[topic] = pub_node_name;
          result.reason = "Topic being monitored by external node: " + pub_node_name;
          return result;
        }
      }
      // Store the type for use in add_topic
      topic_to_type_[topic] = publishers[0].topic_type();
    // Handle disabled (enabled=false) parameter
    } else {
      // Reject if not being monitored internally
      if (greenwave_diagnostics_.find(topic) == greenwave_diagnostics_.end()) {
        result.successful = false;
        result.reason = "Topic not being monitored: " + topic;
        return result;
      }
    }
  }

  return result;
}

void GreenwaveMonitor::on_parameter_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
{
  // Skip parameter events from other nodes
  if (msg->node != this->get_fully_qualified_name()) {
    return;
  }
  // Process new and changed parameters - execute topic additions and removals
  auto apply_parameter_changes = [this](const auto & params) {
    for (const auto & param : params) {
      // Ensure it matches greenwave_diagnostics.<topic>.enabled pattern
      auto topic_opt = parse_enabled_topic_param(param.name);
      if (!topic_opt.has_value()) {
        continue;
      }
      // Do nothing if greenwave_diagnostics_ object for this topic exists
      if (greenwave_diagnostics_.find(topic_opt.value()) != greenwave_diagnostics_.end()) {
        continue;
      }
      // Extract topic name and enabled value
      const std::string & topic = topic_opt.value();
      bool enabled = param.value.bool_value;
      std::string message;
      // Handle enabled=true parameter
      if (enabled) {
        add_topic(topic, message);
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
      // Handle enabled=false parameter
      } else {
        remove_topic(topic, message);
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
      }
    }
  };
  apply_parameter_changes(msg->new_parameters);
  apply_parameter_changes(msg->changed_parameters);
}

bool GreenwaveMonitor::has_header_from_type(const std::string & type_name)
{
  // We use a cache to avoid repeated lookups for the same message type.
  // ex. {sensor_msgs/msg/Image : true, std_msgs/msg/String : false}
  static std::unordered_map<std::string, bool> type_has_header_cache;

  static std::mutex has_header_cache_mutex;
  std::lock_guard<std::mutex> lock(has_header_cache_mutex);

  if (type_has_header_cache.find(type_name) != type_has_header_cache.end()) {
    return type_has_header_cache[type_name];
  }

  // rosidl typesupport API is unstable across ROS distributions, so we use this
  // map as a more robust way to determine if a message type has a header
  static const std::unordered_map<std::string, bool> known_header_types = {
    // sensor_msgs
    {"sensor_msgs/msg/Image", true},
    {"sensor_msgs/msg/CompressedImage", true},
    {"sensor_msgs/msg/CameraInfo", true},
    {"sensor_msgs/msg/PointCloud2", true},
    {"sensor_msgs/msg/LaserScan", true},
    {"sensor_msgs/msg/Imu", true},
    {"sensor_msgs/msg/NavSatFix", true},
    {"sensor_msgs/msg/MagneticField", true},
    {"sensor_msgs/msg/FluidPressure", true},
    {"sensor_msgs/msg/Illuminance", true},
    {"sensor_msgs/msg/RelativeHumidity", true},
    {"sensor_msgs/msg/Temperature", true},
    {"sensor_msgs/msg/Range", true},
    {"sensor_msgs/msg/PointCloud", true},

    // geometry_msgs
    {"geometry_msgs/msg/PoseStamped", true},
    {"geometry_msgs/msg/TwistStamped", true},
    {"geometry_msgs/msg/AccelStamped", true},
    {"geometry_msgs/msg/Vector3Stamped", true},
    {"geometry_msgs/msg/PointStamped", true},
    {"geometry_msgs/msg/QuaternionStamped", true},
    {"geometry_msgs/msg/TransformStamped", true},
    {"geometry_msgs/msg/WrenchStamped", true},

    // nav_msgs
    {"nav_msgs/msg/OccupancyGrid", true},
    {"nav_msgs/msg/GridCells", true},
    {"nav_msgs/msg/Path", true},
    {"nav_msgs/msg/Odometry", true},

    // visualization_msgs
    {"visualization_msgs/msg/Marker", true},
    {"visualization_msgs/msg/MarkerArray", true},
    {"visualization_msgs/msg/InteractiveMarker", true},

    // std_msgs (no headers)
    {"std_msgs/msg/String", false},
    {"std_msgs/msg/Int32", false},
    {"std_msgs/msg/Float64", false},
    {"std_msgs/msg/Bool", false},
    {"std_msgs/msg/Empty", false},
    {"std_msgs/msg/Header", false},  // Header itself doesn't have a header

    // Common message types without headers
    {"geometry_msgs/msg/Twist", false},
    {"geometry_msgs/msg/Pose", false},
    {"geometry_msgs/msg/Point", false},
    {"geometry_msgs/msg/Vector3", false},
    {"geometry_msgs/msg/Quaternion", false}
  };

  auto it = known_header_types.find(type_name);
  bool has_header = (it != known_header_types.end()) ? it->second : false;

  type_has_header_cache[type_name] = has_header;

  // Fallback of no header in case of unknown type, log for reference
  if (it == known_header_types.end()) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "Unknown message type '%s' - assuming no header. Consider adding to registry.",
      type_name.c_str());
  }

  return has_header;
}

bool GreenwaveMonitor::add_topic(
  const std::string & topic, std::string & message)
{
  RCLCPP_INFO(this->get_logger(), "Adding subscription for topic '%s'", topic.c_str());

  auto type = topic_to_type_[topic];
  if (type.empty()) {
    message = "No type found for topic: " + topic;
    return false;
  }

  auto sub = this->create_generic_subscription(
    topic,
    type,
    rclcpp::QoS(
      rclcpp::KeepLast(10), rmw_qos_profile_sensor_data),
    [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      this->topic_callback(msg, topic, type);
    });

  greenwave_diagnostics::GreenwaveDiagnosticsConfig diagnostics_config;
  diagnostics_config.enable_all_topic_diagnostics = true;

  subscriptions_.push_back(sub);
  greenwave_diagnostics_.emplace(
    topic,
    std::make_unique<greenwave_diagnostics::GreenwaveDiagnostics>(
      *this, topic, diagnostics_config));

  message = "Successfully added topic: " + topic;
  return true;
}

bool GreenwaveMonitor::remove_topic(const std::string & topic, std::string & message)
{
  auto diag_it = greenwave_diagnostics_.find(topic);
  if (diag_it == greenwave_diagnostics_.end()) {
    message = "Nothing to remove, topic not being monitored";
    return true;
  }

  // Find and remove the subscription
  auto sub_it = std::find_if(
    subscriptions_.begin(), subscriptions_.end(),
    [&topic](const auto & sub) {
      return sub->get_topic_name() == topic;
    });
  if (sub_it != subscriptions_.end()) {
    subscriptions_.erase(sub_it);
  }

  // NOTE: the parameters are not removed when the diagnostics object is destroyed. This allows
  // for settings to persist even when a topic is not available.
  greenwave_diagnostics_.erase(diag_it);

  message = "Successfully removed topic";
  return true;
}

std::set<std::string> GreenwaveMonitor::get_topics_from_parameters()
{
  std::set<std::string> topics;

  // List all parameters with "greenwave_diagnostics." prefix
  auto list_result = this->list_parameters({"greenwave_diagnostics"}, 10);

  for (const auto & param_name : list_result.names) {
    // Parameter names are like "greenwave_diagnostics./my_topic.enabled"
    // We need to extract the topic name (e.g., "/my_topic")
    if (param_name.find(kTopicParamPrefix) != 0) {
      continue;
    }

    // Remove the "greenwave_diagnostics." prefix
    std::string remainder = param_name.substr(
      std::strlen(kTopicParamPrefix));

    // Find the last '.' to separate topic name from parameter suffix
    // Topic names can contain '/' but parameter suffixes are like ".enabled", ".tolerance", etc.
    size_t last_dot = remainder.rfind('.');
    if (last_dot == std::string::npos || last_dot == 0) {
      continue;
    }

    std::string topic_name = remainder.substr(0, last_dot);
    if (!topic_name.empty() && topic_name[0] == '/') {
      topics.insert(topic_name);
    }
  }

  // Filter out topics with enabled=false
  std::set<std::string> filtered_topics;
  for (const auto & topic : topics) {
    std::string enabled_param = std::string(kTopicParamPrefix) + topic + kEnabledSuffix;

    if (this->has_parameter(enabled_param)) {
      auto param = this->get_parameter(enabled_param);
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL && !param.as_bool())
      {
        continue;
      }
    }
    filtered_topics.insert(topic);
  }

  return filtered_topics;
}

// From ros2_benchmark monitor_node.cpp
// This assumes the message has a std_msgs header as the first
std::chrono::time_point<std::chrono::system_clock>
GreenwaveMonitor::GetTimestampFromSerializedMessage(
  std::shared_ptr<rclcpp::SerializedMessage> serialized_message_ptr,
  const std::string & type)
{
  if (!has_header_from_type(type)) {
    return std::chrono::time_point<std::chrono::system_clock>();  // timestamp 0 as fallback
  }

  int32_t timestamp_sec;
  uint8_t * sec_byte_ptr = reinterpret_cast<uint8_t *>(&timestamp_sec);
  *(sec_byte_ptr + 0) = serialized_message_ptr->get_rcl_serialized_message().buffer[4];
  *(sec_byte_ptr + 1) = serialized_message_ptr->get_rcl_serialized_message().buffer[5];
  *(sec_byte_ptr + 2) = serialized_message_ptr->get_rcl_serialized_message().buffer[6];
  *(sec_byte_ptr + 3) = serialized_message_ptr->get_rcl_serialized_message().buffer[7];

  uint32_t timestamp_nanosec;
  uint8_t * ns_byte_ptr = reinterpret_cast<uint8_t *>(&timestamp_nanosec);
  *(ns_byte_ptr + 0) = serialized_message_ptr->get_rcl_serialized_message().buffer[8];
  *(ns_byte_ptr + 1) = serialized_message_ptr->get_rcl_serialized_message().buffer[9];
  *(ns_byte_ptr + 2) = serialized_message_ptr->get_rcl_serialized_message().buffer[10];
  *(ns_byte_ptr + 3) = serialized_message_ptr->get_rcl_serialized_message().buffer[11];

  std::chrono::time_point<std::chrono::system_clock> timestamp(
    std::chrono::seconds(timestamp_sec) + std::chrono::nanoseconds(timestamp_nanosec));
  return timestamp;
}
