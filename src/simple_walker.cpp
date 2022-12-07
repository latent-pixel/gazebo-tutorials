/**
 * Copyright (c) 2022 Bhargav Kumar Soothram
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file simple_walker.cpp
 * @author Bhargav Kumar Soothram (bsoothra@umd.edu)
 * @brief A simple obstacle avoidance driver
 * @version 0.1
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <simple_walker.hpp>

WalkerNode::WalkerNode(const std::string &node_name) : Node(node_name) {
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1,
      std::bind(&WalkerNode::topic_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
}

void WalkerNode::topic_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
  // Look around for obstacles
  float min = 1;
  std::array<int, 41> degrees = {
      0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,
      14,  15,  16,  17,  18,  19,  20,  340, 341, 342, 343, 344, 345, 346,
      347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359};
  for (auto i : degrees) {
    float current = _msg->ranges[i];
    if (current < min) {
      min = current;
    }
  }
  auto message = this->next_move(min);
  publisher_->publish(message);
}

auto WalkerNode::next_move(float distance) -> geometry_msgs::msg::Twist {
  auto msg = geometry_msgs::msg::Twist();
  // logic
  RCLCPP_INFO(this->get_logger(), "Distance is: '%f'", distance);
  if (distance < 0.5) {
    // turn around
    msg.linear.x = 0;
    msg.angular.z = 0.8;
  } else {
    // go straight ahead
    msg.linear.x = 0.2;
    msg.angular.z = 0;
  }
  return msg;
}

auto main(int argc, char *argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkerNode>("my_node"));
  rclcpp::shutdown();
  return 0;
}
