// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#include <ros/package.h>
#include <deep_nav_layers/segmentation_layer.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include<costmap_2d/costmap_layer.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(segmentation_layer::SegmentationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace segmentation_layer
{

SegmentationLayer::SegmentationLayer() {}

void SegmentationLayer::onInitialize() {
  std::string segmentation_data;
  std::string path_homography;

  ros::NodeHandle nh("~/" + name_);
  nh.getParam("path_homography",  path_homography); // Full path to the homography matrix
  nh.param<std::string>("segmentation_data",  segmentation_data, "/tinycar/road_segmentation");
  nh.param<int>("warp_width",  warp_width, 2400); // width of the costmap
  nh.param<int>("warp_height",  warp_height, 2400); // height of the costmap
  nh.param<float>("m_per_pixel",  m_per_pixel, 0.03); // ratio to convert pixels into meters
  nh.param<float>("x_range",  x_range, 30.0); // meters to the left/right of the bot to modify costmap
  nh.param<float>("y_range",  y_range, 30.0); // meters in front of the bot to modify costmap

  current_ = true;

  new_data = false;
  map_ready = false;
  default_value_ = NO_INFORMATION;
  
  rolling_window = layered_costmap_->isRolling();
  img_size_x_meter = warp_width * m_per_pixel / 2;
  img_size_y_meter = warp_height * m_per_pixel / 2;

  // Master costmap size is already locked (in case of StaticLayer)
  // we have to call matchSize ourself 
  if (layered_costmap_->isSizeLocked()) {
    matchSize();
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SegmentationLayer::reconfigureCB, this, _1, _2);
  dsrv_->clearCallback();
  dsrv_->setCallback(cb);

  img_sub = nh.subscribe<sensor_msgs::Image>(segmentation_data, 1, &SegmentationLayer::segNetCb, this);

  cv::FileStorage fs(path_homography, cv::FileStorage::READ);
  fs["homography"] >> h;
  fs.release();
}


void SegmentationLayer::segNetCb(const sensor_msgs::Image::ConstPtr &msg)
{
  if (!map_ready)
    return;

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat warped, cropped;
  cv::warpPerspective(cv_ptr->image, warped, h, cv::Size(warp_width, warp_height));
  ImageDebug("warped.png", warped)

  int x_range_pixels = x_range / m_per_pixel;
  int y_range_pixels = y_range / m_per_pixel;

  // crop projection to only go a user defined number of meters in x and y direction
  cv::Rect ROI = cv::Rect(warped.cols/2 - x_range_pixels, warped.rows/2 - y_range_pixels, x_range_pixels*2, y_range_pixels*2);
  warped = cv::Mat(warped, ROI); // note that this is just a reference
  ImageDebug("roi.png", warped)

  // Scale the segmentation data to the size of our costmap
  cv::resize(warped, scaled, costmap_size);
  ImageDebug("scaled.png", warped)

  new_data = true;
}
  
void SegmentationLayer::matchSize() {
  map_ready = true;
  costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  resolution = master->getResolution();

  // Determine what's the limiting factor for our range in x and y direction
  if(x_range > master->getSizeInMetersX() / 2)
    x_range = master->getSizeInMetersX() / 2;
  if (x_range > img_size_x_meter)
    x_range = img_size_x_meter;

  if(y_range > master->getSizeInMetersY() / 2)
    y_range = master->getSizeInMetersY() / 2;
  if (y_range > img_size_y_meter)
    y_range = img_size_y_meter;
  
  ROS_INFO("x_range: %f  y_range: %f", x_range, y_range);
  costmap_size = cv::Size(ceil(x_range*2 / resolution), ceil(y_range*2 / resolution));

  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
	    master->getOriginX(), master->getOriginY());
}
  
  // allows the plugin to dynamically change the configuration of the costmap
void SegmentationLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level) {
  enabled_ = config.enabled;
}

void SegmentationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y) {
  if (!enabled_)
    return;

  if (!new_data)
    return;

  if (!map_ready)
    return;
  
  if (rolling_window) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // Draw empty space for the cars footprint
  float origin_x = scaled.cols / 2;
  float origin_y = scaled.rows / 2;
  float to_meter = 1 / resolution;
  cv::Point2f origin = cv::Point2f(origin_x, origin_y);
  cv::Point2i top_left = cv::Point2f(origin_x + 3.5 * to_meter, origin_y - 3.5 * to_meter);
  cv::Point2i bottom_right = cv::Point2f(origin_x - 3.5 * to_meter, origin_y + 3.5 * to_meter);

  cv::rectangle(scaled, top_left, bottom_right, cv::Scalar(255,255,255), CV_FILLED);
  ImageDebug("footprint.png", scaled)

  // convert to degrees and adjust starting point
  double angle = (robot_yaw*180)/M_PI - 90;

  // Rotate the street data to the robots orientation
  cv::Mat map;
  cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(scaled.cols/2, scaled.rows/2), angle, 1);
  cv::warpAffine(scaled, map, rotation_matrix, scaled.size());
  ImageDebug("rotated.png", map)

  // Simplify the map
  int ddepth = CV_8U;
  cv::Mat edges, inverted;
  cv::Laplacian(map, edges, ddepth, 5, 1, 0, cv::BORDER_DEFAULT);
  ImageDebug("edges.png", edges)


  for(int y=0; y < map.rows; y++) {
    uchar* pixel_clear = map.ptr<uchar>(y);
    uchar* pixel_set = edges.ptr<uchar>(y);

    for(int x=0; x < map.cols; x++) {
      int value_clear = pixel_clear[x];
      int value_set = pixel_set[x];

      if (value_set == 255) {
        // shift over point so origin of image is at (0,0)
        double mark_x = robot_x + ((x - origin.x) * resolution);
        double mark_y = robot_y + ((origin.y - y) * resolution);

        unsigned int mx, my;
        if(worldToMap(mark_x, mark_y, mx, my)) {
          setCost(mx, my, LETHAL_OBSTACLE);
        }
      }
      if (value_clear == 255) {
        double mark_x = robot_x + (x - origin.x) * resolution;
        double mark_y = robot_y + (origin.y - y) * resolution;

        unsigned int mx, my;
        if(worldToMap(mark_x, mark_y, mx, my)) {
          setCost(mx, my, FREE_SPACE);
        }
      }

    }
  }

  *min_x = robot_x - x_range;
  *min_y = robot_y - y_range;
  *max_x = robot_x + x_range;
  *max_y = robot_y + y_range;

  new_data = false;
}

  // actually update the costs within the bounds
void SegmentationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j) {
  if (!enabled_)
    return;

  if (!map_ready)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
	      continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

} // end namespace
