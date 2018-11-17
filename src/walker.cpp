/**
 * @file    walker.hpp
 * @author  Shivang Patel
 * @copyright MIT License (c) 2018 Shivang Patel
 *
 * @brief DESCRIPTION
 * Implementation of class called lawnmover.
 *
 * @license MIT License
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <walker.hpp>

Walker::Walker(ros::NodeHandle node_) {
  velocity_pub_ =
      node_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);
  laser_sub_ = node_.subscribe("/scan", 10, &Walker::laserCallback, this);

  //> Resetting the the velocities.
  twist_msgs_.linear.x = 0;
  twist_msgs_.angular.z = 0;
  velocity_pub_.publish(twist_msgs_);

  ROS_INFO("TURTLEBOT_WALKER INTIALIZED.");
}

Walker::~Walker() {
  ROS_INFO("TURTLEBOT_WALKER IS SHUTTING DOWN.");

  //> Resetting the velocities.
  twist_msgs_.linear.x = 0;
  twist_msgs_.angular.z = 0;
  velocity_pub_.publish(twist_msgs_);
  ROS_INFO("TURTLEBOT_WALKER VELOCITIES RESETTED.");
}

void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  ROS_DEBUG("CALLING LaserCallback.");
  //> If obst is nearby, start turning.
  for (const auto& itr : scan->ranges) {
    //> scan->range_min is 0.45
    if (itr <= scan->range_min + 0.55) {
      twist_msgs_.linear.x = 0;
      twist_msgs_.angular.z = max_value_;
      velocity_pub_.publish(twist_msgs_);
      ROS_WARN("TURNING.");
      return;
    }
  }
  // Else walk straight
  twist_msgs_.linear.x = max_value_;
  twist_msgs_.angular.z = 0;
  velocity_pub_.publish(twist_msgs_);
  ROS_DEBUG("GOING FORWARD");
  return;
}
