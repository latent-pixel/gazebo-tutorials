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
 * @file simple_walker.hpp
 * @author Bhargav Kumar Soothram (bsoothra@umd.edu)
 * @brief Header file for simple_walker.cpp
 * @version 0.1
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SIMPLE_WALKER_HPP
#define SIMPLE_WALKER_HPP

#include "geometry_msgs/msg/twist.hpp"     // Twist
#include "rclcpp/rclcpp.hpp"               // ROS Core Libraries
#include "sensor_msgs/msg/laser_scan.hpp"  // Laser Scan


class WalkerNode : public rclcpp::Node {
    public:
    WalkerNode(const std::string &node_name);

    private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
    auto next_move(float distance) -> geometry_msgs::msg::Twist;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};
#endif