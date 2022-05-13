/*
 * Copyright (C) 2020-2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include "traffic_incident_parser/traffic_incident_parser_node.hpp"

namespace traffic_incident_parser
{
  namespace std_ph = std::placeholders;

  TrafficIncidentParserNode::TrafficIncidentParserNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options),
        wm_listener_(this->get_node_base_interface(), this->get_node_logging_interface(), 
                     this->get_node_topics_interface(), this->get_node_parameters_interface()),
        traffic_parser_worker_(wm_listener_.getWorldModel(), std::bind(&TrafficIncidentParserNode::publishTrafficControlMessage, this, std_ph::_1), get_node_logging_interface(), get_clock())
  {
  }

  carma_ros2_utils::CallbackReturn TrafficIncidentParserNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Setup subscribers
    projection_sub_ = create_subscription<std_msgs::msg::String>("georeference", 1,
                                                              std::bind(&TrafficIncidentParserWorker::georeferenceCallback, &traffic_parser_worker_, std_ph::_1));
    mobility_operation_sub_ = create_subscription<carma_v2x_msgs::msg::MobilityOperation>("incoming_mobility_operation", 1,
                                                              std::bind(&TrafficIncidentParserWorker::mobilityOperationCallback, &traffic_parser_worker_, std_ph::_1));

    // Setup publishers
    rclcpp::PublisherOptions publisher_options;
    publisher_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // A publisher initialized with this will have intra-process comms disabled

    auto publisher_qos = rclcpp::QoS(rclcpp::KeepAll()); // A publisher with this QoS will store all messages that it has sent on the topic
    publisher_qos.transient_local();  // A publisher with this QoS will re-send all (when KeepAll is used) messages to all late-joining subscribers 

    traffic_control_msg_pub_ = create_publisher<carma_v2x_msgs::msg::TrafficControlMessage>("geofence", publisher_qos, publisher_options);

    auto publisher_qos2 = rclcpp::QoS(rclcpp::KeepAll()); // New QoS with KeepAll as the initial setting
    publisher_qos2.transient_local(); 
    test_pub_ = create_publisher<std_msgs::msg::String>("test_topic", publisher_qos2, publisher_options);

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn TrafficIncidentParserNode::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    std_msgs::msg::String msg1;
    msg1.data = "Test1";

    std_msgs::msg::String msg2;
    msg2.data = "Test2";

    std_msgs::msg::String msg3;
    msg3.data = "Test3";

    test_pub_->publish(msg1);
    test_pub_->publish(msg2);
    test_pub_->publish(msg3);
    
    return CallbackReturn::SUCCESS;
  }

  void TrafficIncidentParserNode::publishTrafficControlMessage(const carma_v2x_msgs::msg::TrafficControlMessage& traffic_control_msg)
  {
    traffic_control_msg_pub_->publish(traffic_control_msg);
  }

} // traffic_incident_parser

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_incident_parser::TrafficIncidentParserNode)
