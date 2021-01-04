// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#include <ros/package.h>
#include <deep_nav_layers/segmentation_layer.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include<costmap_2d/costmap_layer.h>

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
  nh.param<std::string>("obstacle_ids_str",  obstacle_ids_str, "14 15"); // zero-indexed IDs to be marked as an obstacle
  nh.param<std::string>("path_ids_str",  path_ids_str, "3"); // zero-indexed ID to clear obstacles
  nh.param<int>("costmap_width",  costmap_width, 2400); // width of the costmap
  nh.param<int>("costmap_height",  costmap_height, 2400); // height of the costmap
  nh.param<float>("resize_factor",  resize_factor, 0.2); // factor to resize the costmap size
  nh.param<float>("m_per_pixel",  m_per_pixel, 0.03); // ratio to convert pixels into meters
  nh.param<float>("x_range",  x_range, 300.0); // pixels to the left/right of the bot to modify costmap
  nh.param<float>("y_range",  y_range, 300.0); // pixels in front of the bot to modify costmap
  //m_per_pixel *= resize_factor;
  x_range *= resize_factor;
  y_range *= resize_factor;
  
  

  std::string path_homography = ros::package::getPath("deep_nav_layers") + "/calibrate_homography/" + file_homography;

  parseIntSet(obstacle_ids_str, obstacle_ids);
  parseIntSet(path_ids_str, path_ids);

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
  std::cout << "Loaded homography: " << h << std::endl;
  fs.release();
}



void SegmentationLayer::parseIntSet(const std::string &raw_list, std::set<int> &int_set)
{
  std::stringstream ss(raw_list);

  int i;
  while (ss >> i)
  {
    int_set.insert(i);

    if (ss.peek() == ' ')
      return;
      ss.ignore();
  }
}

void SegmentationLayer::segNetCb(const sensor_msgs::Image::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  cv::warpPerspective(cv_ptr->image, warped, h, cv::Size(costmap_width, costmap_height));

  cv::resize(warped, resized, cv::Size(), resize_factor, resize_factor);
  
  int x_range_pixels = x_range;///m_per_pixel;
  int y_range_pixels = y_range;///m_per_pixel;

  // crop projection to only go a user defined number of meters in x and y direction
  cv::Rect ROI = cv::Rect(resized.cols/2 - x_range_pixels, resized.rows/2 - y_range_pixels, x_range_pixels*2, y_range_pixels*2);
  cropped = cv::Mat(resized, ROI); // note that this is just a reference

  new_data = true;
}
  
void SegmentationLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
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
                                           double* min_y, double* max_x, double* max_y)
{

  if (!enabled_)
    return;

  if (!new_data)
    return;
  
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }


  // convert to degrees and adjust starting point
  double angle = (robot_yaw*180)/M_PI - 90;

  cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(cropped.cols/2, cropped.rows/2), angle, 1);
  cv::Mat overlay;

  cv::warpAffine(cropped, overlay, rotation_matrix, cropped.size());
  cv::Point2f origin = cv::Point2f(overlay.cols/2, overlay.rows/2);
  cv::circle(overlay, origin, 35, cv::Scalar(255,255,255),CV_FILLED, 8,0);

  float fac = 1/resize_factor;
  for(int y=0; y<overlay.rows; y++){
      for(int x=0; x<overlay.cols; x++){
        int value = overlay.at<unsigned char>(cv::Point(x,y));
        if (obstacle_ids.find(value) != obstacle_ids.end()) {
          // shift over point so origin of image is at (0,0)
          double mark_x = robot_x + (x - origin.x)*fac*m_per_pixel;
          double mark_y = robot_y + (origin.y - y)*fac*m_per_pixel;

          unsigned int mx, my;
          

          if(worldToMap(mark_x, mark_y, mx, my)){
            //setCost(mx, my, LETHAL_OBSTACLE);
            //setCost(mx, my, 100);
          }
        }
        if (path_ids.find(value) != path_ids.end()) {
          double mark_x = robot_x + (x - origin.x)*fac*m_per_pixel;
          double mark_y = robot_y + (origin.y - y)*fac*m_per_pixel;
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
      //if (costmap_[index] == NO_INFORMATION)
	      //continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

} // end namespace
