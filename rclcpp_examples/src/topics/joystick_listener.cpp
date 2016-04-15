// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

void chatterCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  for(int i=0; i<msg->axes.size(); i++)
    std::cout << "axis " << i << ": " << msg->axes[i] << std::endl;
  for(int i=0; i<msg->buttons.size(); i++)
    std::cout << "button " << i << ": " << msg->buttons[i] << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("listener");

  auto sub = node->create_subscription<sensor_msgs::msg::Joy>(
    "joy", chatterCallback, rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}
