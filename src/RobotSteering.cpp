/**
 *  MIT License
 *
 *  Copyright (c) 2019 Raja Srinivas Iskala
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file RobotSteering.cpp
 *@author Raja Srinivas Iskala
 *@copyright MIT License
 *@brief implements RobotSteering class functions
 */

#include "robot_walker/RobotSteering.h"

/**
 * @brief Constructor of the class.
 */
RobotSteering::RobotSteering() {
}

/**
 * @brief destructor of the class.
 */
RobotSteering::~RobotSteering() {
}

/**
 * @brief Initialization of Publisher.
 * @param None.
 * @return None.
 */
void RobotSteering::publisher() {
  // Publish the velocity
  publishVelocity = nodeHandler.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 100);
  // Display publisher message
  ROS_INFO_STREAM("Started the Publisher");
}

/**
 * @brief Initialization of subscriber.
 * @param None.
 * @return None.
 */
void RobotSteering::subscriber() {
  // Subscribe to laser scan to detect obstacles
  subscribeLaser = nodeHandler.subscribe("/scan", 100,
                                         &RobotSteering::laserScanCallback,
                                         this);
  // Display subscriber message
  ROS_INFO_STREAM("Started the Subscriber");
}

/**
 * @brief Callback function for the laser scan topic
 * @param msg is a pointer to message of type
 * sensor_msgs::LaserScan.
 * @return None.
 */
void RobotSteering::laserScanCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  thresholdGap = *(msg->ranges.begin());
  // Finding the distance to the nearest obstacle
  for (auto i : msg->ranges) {
    if (i < thresholdGap && !std::isnan(i)) {
      // Set the smallest value to the minGap.
      thresholdGap = i;
      // Check if the obstacle not in the search space.
      if (std::isnan (thresholdGap)) {
        // Display the obstacle presence
        ROS_INFO_STREAM("No Obstacle ahead");
      } else {
        // Display the nearest obstacle distance
        ROS_INFO_STREAM(
            "The Minimum gap to near Obstacle is : \t" << thresholdGap);
      }
      // checks for the obstacle in path ahead.
      if (thresholdGap < msg->range_min + 0.5 && !std::isnan(thresholdGap)) {
        isInCollision = true;
        // Display the obstacle presence
        ROS_INFO_STREAM("Robot turning due to obstacle ahead");
      }
      // Function to run the robot based on the object point.
      robotMovement (isInCollision);
    }
  }
}

/**
 * @brief moves the robot forward until its close to obstacle.
 * If its close then robot rotates
 * @param collide is a boolean that says true if in collision
 * @return None.
 */
void RobotSteering::robotMovement(bool collide) {
  if (collide) {
    // robot's linear movement seizes
    message.linear.x = 0.0;
    // robot turns about its z-axis
    message.angular.z = angularVelocity;
  } else {
    // robot doesn't turn around
    message.angular.z = 0.0;
    // robot moves in linear x direction
    message.linear.x = linearVelocity;
  }
  // Publish the values to the robot to follow.
  publishVelocity.publish(message);
}

