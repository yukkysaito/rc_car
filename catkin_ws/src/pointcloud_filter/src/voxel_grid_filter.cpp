/*
 *  Copyright (c) 2015, Yukihiro Saito
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

#include <dynamic_reconfigure/server.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_filter/VoxelGridFilterConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

using namespace std;

class VoxelGridFilter {
private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<pointcloud_filter::VoxelGridFilterConfig> server_;

  int queue_size_;
  string in_, out_;
  string frame_id_;
  double leaf_size_;
  void cloud_cb(const sensor_msgs::PointCloud2 &msg) {
    if (pub_.getNumSubscribers() <= 0) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> pcl_msg;
    pcl::fromROSMsg(msg, pcl_msg);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg_ptr(
        new pcl::PointCloud<pcl::PointXYZI>(pcl_msg));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pcl_msg_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_grid_filter.setInputCloud(pcl_msg_ptr);
    voxel_grid_filter.filter(*filtered_pcl_msg_ptr);
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_pcl_msg_ptr, output_msg);
    pub_.publish(output_msg);
  }

public:
  VoxelGridFilter()
      : nh_("~"), queue_size_(1), in_("/voxel_grid_filter_in"),
        out_("/voxel_grid_filter_out"), leaf_size_(0.1) {

    // Subscribe to the cloud topic using both the old message format and the
    // new
    sub_ = nh_.subscribe(in_, queue_size_, &VoxelGridFilter::cloud_cb, this);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_, queue_size_);
    server_.setCallback(
        boost::bind(&VoxelGridFilter::dynamic_reconfigure_cb, this, _1, _2));

    nh_.resolveName(in_).c_str();
    nh_.resolveName(out_).c_str();
  }
  void dynamic_reconfigure_cb(pointcloud_filter::VoxelGridFilterConfig &config,
                              uint32_t level) {
    leaf_size_ = config.leaf_size;
  }
};

int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "voxel_grid_filter", ros::init_options::AnonymousName);

  VoxelGridFilter vgf;
  ros::spin();

  return (0);
}
