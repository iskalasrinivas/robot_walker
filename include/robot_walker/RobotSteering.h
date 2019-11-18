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
 *@file RobotSteering.h
 *@author Raja Srinivas Iskala
 *@copyright MIT License
 *@brief defination of RobotSteering class functions
 */

#ifndef APP_ROBOTSTEERING_H_
#define APP_ROBOTSTEERING_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class RobotSteering {
public:
  RobotSteering();
  ~RobotSteering();

  /**
   * @brief Initialization of Publisher.
   * @param None.
   * @return None.
   */
  void publisher();

  /**
   * @brief Initialization of subscriber.
   * @param None.
   * @return None.
   */
  void subscriber();

  /**
   * @brief Callback function for the laser scan topic
   * @param msg is a pointer to message of type
   * sensor_msgs::LaserScan.
   * @return None.
   */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief moves the robot forward until its close to obstacle.
   * If its close then robot rotates
   * @param collide is a boolean that says true if in collision
   * @return None.
   */
  void robotMovement(bool collide);

private:
  geometry_msgs::Twist message;
  float linearVelocity = 0.5;
  float angularVelocity = 1.0;
  ros::NodeHandle nodeHandler;
  ros::Publisher publishVelocity;
  ros::Subscriber subscribeLaser;
  bool isInCollision = false;
  float thresholdGap = 0.0;
};

#endif /* APP_ROBOTSTEERING_H_ */

