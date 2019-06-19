/*
 *  Copyright (c) 2017, Yukihiro Saito
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 * this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>

using namespace std;

class TwistCmdConverter {
private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_twist_;
  ros::Publisher pub_twist_;

  int queue_size_;
  string twist_in_, twist_out_;
  double w_linear_x_;
  double w_angular_z_;

public:
  TwistCmdConverter()
      : nh_("~"), queue_size_(1), twist_in_("/twist_in"),
        twist_out_("/twist_out") {
    // Subscribe to the cloud topic using both the old message format and the
    // new
    sub_twist_ = nh_.subscribe(twist_in_, queue_size_,
                               &TwistCmdConverter::twistCallback, this);

    pub_twist_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(twist_out_, queue_size_);
    nh_.resolveName(twist_in_).c_str();
    nh_.resolveName(twist_out_).c_str();
    w_linear_x_ = 1.0;
    w_angular_z_ = 1.0;
    nh_.getParam("weight_linear_x", w_linear_x_);
    nh_.getParam("weight_angular_z", w_angular_z_);
  }

  void twistCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
    if (pub_twist_.getNumSubscribers() <= 0) {
      return;
    }

    ackermann_msgs::AckermannDriveStamped output;
    output.header = msg->header;

    output.drive.speed = msg->twist.linear.x * w_linear_x_;
    output.drive.steering_angle = msg->twist.angular.z * w_angular_z_;

    pub_twist_.publish(output);
  }
};

/* -- twist converter main -- */
int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "twist_cmd_converter",
            ros::init_options::AnonymousName);

  TwistCmdConverter p;
  ros::spin();

  return (0);
}
