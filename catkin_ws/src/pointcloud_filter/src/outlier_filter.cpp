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
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_filter/OutlierFilterConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

using namespace std;

class OutlierFilter {
private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  dynamic_reconfigure::Server<pointcloud_filter::OutlierFilterConfig> server_;

  int queue_size_;
  string in_, out_;
  string frame_id_;
  double leaf_size_;

  int filter_method_;
  double sor_k_;
  double sor_mul_thresh_;
  double ror_rs_;
  double ror_min_neighbors_;

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
    switch (filter_method_) {
    case 0: {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
      sor.setInputCloud(pcl_msg_ptr);
      sor.setMeanK(sor_k_);
      sor.setStddevMulThresh(sor_mul_thresh_);
      sor.filter(*filtered_pcl_msg_ptr);
      break;
    }
    case 1: {
      pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
      // build the filter
      outrem.setInputCloud(pcl_msg_ptr);
      outrem.setRadiusSearch(ror_rs_);
      outrem.setMinNeighborsInRadius(ror_min_neighbors_);
      // apply filter
      outrem.filter(*filtered_pcl_msg_ptr);
      break;
    }
    default: {
      filtered_pcl_msg_ptr = pcl_msg_ptr;
      break;
    }
    }
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_pcl_msg_ptr, output_msg);
    pub_.publish(output_msg);
  }

  void dynamic_reconfigure_cb(pointcloud_filter::OutlierFilterConfig &config,
                              uint32_t level) {
    filter_method_ = config.outlier_filter;
    sor_k_ = config.sor_k;
    sor_mul_thresh_ = config.sor_mul_thresh;
    ror_rs_ = config.ror_rs;
    ror_min_neighbors_ = config.ror_min_neighbors;
  }

public:
  OutlierFilter()
      : nh_("~"), queue_size_(1), in_("/outlier_filter_in"),
        out_("/outlier_filter_out"), filter_method_(1), sor_k_(50),
        sor_mul_thresh_(1.0), ror_rs_(0.8), ror_min_neighbors_(2.0) {
    // Subscribe to the cloud topic using both the old message format and
    // the new
    sub_ = nh_.subscribe(in_, queue_size_, &OutlierFilter::cloud_cb, this);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_, queue_size_);

    server_.setCallback(
        boost::bind(&OutlierFilter::dynamic_reconfigure_cb, this, _1, _2));
    nh_.resolveName(in_).c_str();
    nh_.resolveName(out_).c_str();
  }
};

/* ---[ */
int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "outlier_filter", ros::init_options::AnonymousName);

  OutlierFilter of;
  ros::spin();

  return (0);
}
