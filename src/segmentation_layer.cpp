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

void SegmentationLayer::onInitialize()
{
  std::string segmentation_data;
  std::string obstacle_ids_str;
  std::string path_ids_str;
  std::string file_homography;
  
  ros::NodeHandle nh("~/" + name_);
  nh.getParam("file_homography",  file_homography); // Filename of the homography matrix
  nh.param<std::string>("segmentation_data",  segmentation_data, "/tinycar/road_segmentation");
  nh.param<bool>("clear_only",  clear_only, false); // if true no lethal obstacles will be set (used for global costmap)
  nh.param<int>("warp_width",  warp_width, 2400); // width of the costmap
  nh.param<int>("warp_height",  warp_height, 2400); // height of the costmap
  nh.param<float>("m_per_pixel",  m_per_pixel, 0.03); // ratio to convert pixels into meters
  nh.param<float>("x_range",  x_range, 30.0); // meters to the left/right of the bot to modify costmap
  nh.param<float>("y_range",  y_range, 30.0); // meters in front of the bot to modify costmap

  x_range = (x_range < master->getSizeInMetersX()) ? master->getSizeInMetersX() : x_range;
  y_range = (y_range < master->getSizeInMetersY()) ? master->getSizeInMetersY() : y_range;
  std::cout << "X_range: " << x_range << " Y_range: " << y_range << std::endl;

  std::string path_homography = ros::package::getPath("deep_nav_layers") + "/calibrate_homography/" + file_homography;

  master = layered_costmap_->getCostmap();
  current_ = true;
  new_data = false;
  default_value_ = NO_INFORMATION;
  matchSize();
  rolling_window_ = layered_costmap_->isRolling();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SegmentationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  seg_sub_  = nh.subscribe<sensor_msgs::Image>(segmentation_data, 1, &SegmentationLayer::segNetCb, this);

  cv::FileStorage fs(path_homography, cv::FileStorage::READ);
  fs["homography"] >> h;
  fs.release();
}


void SegmentationLayer::segNetCb(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat warped, cropped;
  cv::warpPerspective(cv_ptr->image, warped, h, cv::Size(warp_width, warp_height));

  int x_range_pixels = x_range / m_per_pixel;
  int y_range_pixels = y_range / m_per_pixel;

  if (x_range_pixels < warped.cols/2 && y_range_pixels < warped.rows/2) {
    // crop projection to only go a user defined number of meters in x and y direction
    cv::Rect ROI = cv::Rect(warped.cols/2 - x_range_pixels, warped.rows/2 - y_range_pixels, x_range_pixels*2, y_range_pixels*2);
    warped = cv::Mat(warped, ROI); // note that this is just a reference
  }

  // Scale the segmentation data to our costmap resolution
  cv::Size scaled_size(ceil(x_range / master->getResolution()), ceil(y_range / master->getResolution()));
  cv::resize(warped, scaled, scaled_size);

  new_data = true;
}
  
void SegmentationLayer::matchSize()
{
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
	    master->getOriginX(), master->getOriginY());
}
  
  // allows the plugin to dynamically change the configuration of the costmap
void SegmentationLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

  // determines the area of the costmap that is potentially going to be changed
void SegmentationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y) {
  if (!enabled_)
    return;

  if (!new_data)
    return;
  
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // Draw empty space as the cars footprint
  float origin_x = scaled.cols / 2;
  float origin_y = scaled.rows / 2;
  float to_meter = 1 / master->getResolution();
  cv::Point2f origin = cv::Point2f(origin_x, origin_y);
  cv::Point2i top_left = cv::Point2f(origin_x + 4 * to_meter, origin_y - 2 * to_meter);
  cv::Point2i bottom_right = cv::Point2f(origin_x - 4 * to_meter, origin_y + 2 * to_meter);
  cv::rectangle(scaled, top_left, bottom_right, cv::Scalar(255,255,255), CV_FILLED);
  //cv::Point2f origin = cv::Point2f(overlay.cols/2, overlay.rows/2);
  //cv::circle(overlay, cv::Point2f((overlay.cols/2) + 10, overlay.rows/2), 100 * resize_factor, cv::Scalar(255,255,255),CV_FILLED, 8,0);
  

  // convert to degrees and adjust starting point
  double angle = (robot_yaw*180)/M_PI - 90;

  // Rotate the street data to the robots orientation
  cv::Mat map;
  cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(scaled.cols/2, scaled.rows/2), angle, 1);
  cv::warpAffine(scaled, map, rotation_matrix, scaled.size());
  
  // Simplify the map
  int ddepth = CV_8U;
  cv::Mat laplacian, inverted;
  cv::Laplacian(map, map, ddepth, 5, 1, 0, cv::BORDER_DEFAULT);
  cv::threshold(map, map, 220, 255, cv::THRESH_BINARY_INV);

  for(int y=0; y < map.rows; y++){
    uchar* pixel;
    if(clear_only) {
      pixel = map.ptr<uchar>(y); 
    } else {
      pixel = map.ptr<uchar>(y); 
    }

    for(int x=map.cols / 3; x < map.cols; x++){ // ignore map behind car
      int value = pixel[x];//inverted.at<unsigned char>(cv::Point(x,y));

      if(!clear_only) {
        if (value == 0) {
          // shift over point so origin of image is at (0,0)
          double mark_x = robot_x + (x - origin.x) * master->getResolution();
          double mark_y = robot_y + (origin.y - y) * master->getResolution();
          unsigned int mx, my;

          if(worldToMap(mark_x, mark_y, mx, my)){
            setCost(mx, my, LETHAL_OBSTACLE);
          }
        }
      }
      
      if (value == 255) {
        double mark_x = robot_x + (x - origin.x) * master->getResolution();
        double mark_y = robot_y + (origin.y - y) * master->getResolution();
        unsigned int mx, my;
        if(worldToMap(mark_x, mark_y, mx, my)){
          setCost(mx, my, FREE_SPACE);

        }
      }

    }
  }

  // REVIEW: potentially make this configurable, or calculated?
  *min_x = -1000; // 20 meters, max size
  *min_y = -1000;
  *max_x = 1000;
  *max_y = 1000;

  new_data = false;
}

  // actually update the costs within the bounds
void SegmentationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
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
