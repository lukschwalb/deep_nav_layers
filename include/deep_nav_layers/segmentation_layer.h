// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#ifndef SEGMENTATION_LAYER_H_
#define SEGMENTATION_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

//#define DEBUG_IMAGES
#ifdef DEBUG_IMAGES
#  define ImageDebug(NAME, IMG) (cv::imwrite(NAME, IMG));
#else
#  define ImageDebug(NAME, IMG) NULL;
#endif // DEBUG

namespace segmentation_layer
{

  class SegmentationLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  SegmentationLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized() {
    return true;
  }
  virtual void matchSize();

private:
  // The maximum range in each direction in meter
  float x_range;
  float y_range;

  // Ratio to convert pixels from the img to meters
  float m_per_pixel;
  int warp_width;
  int warp_height;

  ros::Subscriber img_sub;

  bool rolling_window;
  double resolution; // Resolution of the master costmap
  bool new_data; // Indicates new data for a map update
  bool map_ready; // Indicates the master costmap is resized

  float img_size_x_meter, img_size_y_meter; // Maximum x_range based on the img size
  cv::Size costmap_size;

  cv::Mat scaled; // Scaled version of input image, measurements already match resolution
  cv::Mat h; // Homography matrix

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  void segNetCb(const sensor_msgs::Image::ConstPtr &msg);
};
}
#endif
