/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v0.1.0: drwnz (david.wong@tier4.jp) *
 *
 * selected_points_publisher.cpp
 *
 *  Created on: December 5th 2019
 */

#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "OGRE/OgreCamera.h"

#include "selected_points_publisher/selected_points_publisher.hpp"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>
#include <visualization_msgs/Marker.h>

#include <nlohmann/json.hpp>

#include <fstream>
#include <chrono>
#include <ostream>
#include <ratio>
#include <thread>
#include <iostream>
#include <vector>
#include <tuple>
#include <ctime>
#include <iostream>
#include <iterator>
#include <locale>

namespace common{
    // Get current date/time, format is YYYY-MM-DD HH:mm:ss, format = "%Y-%m-%d %X" or "%Y-%m-%d-%H-%M-%S"
    std::string getCurrentDateTime(const std::string & format = "%Y-%m-%d %X") {
        time_t     now = time(0);
        struct tm  tstruct;
        char       buf[80];
        tstruct = *localtime(&now);
        // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
        // for more information about date/time format
        strftime(buf, sizeof(buf), format.c_str(), &tstruct);

        return buf;
    }

}


namespace rviz_plugin_selected_points_publisher
{
SelectedPointsPublisher::SelectedPointsPublisher()
{
  updateTopic();
}

SelectedPointsPublisher::~SelectedPointsPublisher()
{
    dumpData();
}

void SelectedPointsPublisher::updateTopic()
{
  node_handle_.param("frame_id", tf_frame_, std::string("/base_link"));
  rviz_cloud_topic_ = std::string("/rviz_selected/points");
    rviz_marker_topic_ = std::string("/rviz_selected/markers");

  rviz_selected_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(rviz_cloud_topic_.c_str(), 1);
    marker_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>(rviz_marker_topic_.c_str(),10);

    marker_msg.header.frame_id = "map";//fixed_frame;
    marker_msg.type = visualization_msgs::Marker::CYLINDER;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.05;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;
    marker_msg.header.stamp = ros::Time();
    marker_msg.color.a = 0.5; // Don't forget to set the alpha!
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;

  num_selected_points_ = 0;
}

void SelectedPointsPublisher::dumpData(){
    nlohmann::json json = points_vec_vec;
    nlohmann::json json_center = points_center_vec;

    std::string stamp = common::getCurrentDateTime("%Y-%m-%d-%H-%M-%S");

    std::string file_path = "/tmp/selected_points-" + stamp + ".json";


    std::fstream s(file_path.c_str(), s.binary | s.trunc | s.in | s.out);
    s << "\n<rosparam param=\"marker_points_in_map\">\n"
      << json_center.dump()
      <<"\n</rosparam>\n";

    s << "\n<rosparam param=\"marker_points_in_map_cluster\">\n"
      << json.dump()
      <<"\n</rosparam>\n";
    s.close();
    std::cout << "write data to " << file_path << std::endl;

}
int SelectedPointsPublisher::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  if (event->type() == QKeyEvent::KeyPress)
  {
    if (event->key() == 'C')
    {
      ROS_INFO_STREAM_NAMED("SelectedPointsPublisher::processKeyEvent", "Cleaning previous selection (selected area "
                                                                        "and points).");
      rviz::SelectionManager* selection_manager = context_->getSelectionManager();
      rviz::M_Picked selection = selection_manager->getSelection();
      selection_manager->removeSelection(selection);
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
      marker.header.stamp = ros::Time::now();
      marker.ns = "basic_shapes";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.lifetime = ros::Duration();
      num_selected_points_ = 0;
    }
    else  if (event->key() == 'c')
      {
          ROS_INFO_STREAM_NAMED("SelectedPointsPublisher::processKeyEvent", "Cleaning previous selection (selected area "
                                                                            "and points). points_vec_vec.pop_back(), points_center_vec.pop_back() ");
          rviz::SelectionManager* selection_manager = context_->getSelectionManager();
          rviz::M_Picked selection = selection_manager->getSelection();
          selection_manager->removeSelection(selection);
          visualization_msgs::Marker marker;
          // Set the frame ID and timestamp.  See the TF tutorials for information on these.
          marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
          marker.header.stamp = ros::Time::now();
          marker.ns = "basic_shapes";
          marker.id = 0;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::DELETE;
          marker.lifetime = ros::Duration();
          num_selected_points_ = 0;

          if(!points_vec_vec.empty()){
              points_vec_vec.pop_back();
          }
          if(!points_center_vec.empty()){
              points_center_vec.pop_back();
          }

      }
    else if (event->key() == 'p' ||   event->key() == 'P')
    {
      ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic",
                            "Publishing " << num_selected_points_ << " selected points to topic "
                                          << node_handle_.resolveName(rviz_cloud_topic_));
      rviz_selected_publisher_.publish(selected_points_);

        dumpData();

    }
    else if(   event->key() == 'l' ||  event->key() == 'L'){
        int marker_num = points_center_vec.size();

        ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic",
                              "Publishing " << marker_num << " marker to topic "
                                            << node_handle_.resolveName(rviz_marker_topic_));


        marker_array_msg.markers.resize(marker_num + marker_num,marker_msg);
        for(int i = 0; i <marker_num;i++){
            marker_array_msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
            marker_array_msg.markers[i].pose.position.x = points_center_vec[i][0];
            marker_array_msg.markers[i].pose.position.y = points_center_vec[i][1];
            marker_array_msg.markers[i].id = i;


            marker_array_msg.markers[i+marker_num].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_array_msg.markers[i+marker_num].pose.position.x = points_center_vec[i][0];
            marker_array_msg.markers[i+marker_num].pose.position.y = points_center_vec[i][1];
            marker_array_msg.markers[i+marker_num].id = i+marker_num;

            char s[100];
            sprintf(s, R"(%d : [%.3f,%.3f,%.1f,%.1f])",i, points_center_vec[i][0],points_center_vec[i][1],points_center_vec[i][2],points_center_vec[i][3]);
            marker_array_msg.markers[i+marker_num].text = s;
        }

        marker_pub_.publish(marker_array_msg);
    }
  }
  return 0;
}

int SelectedPointsPublisher::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = rviz::SelectionTool::processMouseEvent(event);
  if (event.alt())
  {
    selecting_ = false;
  }
  else
  {
    if (event.leftDown())
    {
      selecting_ = true;
    }
  }

  if (selecting_)
  {
    if (event.leftUp())
    {
      this->processSelectedArea();
    }
  }
  return flags;
}

int SelectedPointsPublisher::processSelectedArea()
{
  rviz::SelectionManager* selection_manager = context_->getSelectionManager();
  rviz::M_Picked selection = selection_manager->getSelection();
  rviz::PropertyTreeModel* model = selection_manager->getPropertyModel();

  selected_points_.header.frame_id = context_->getFixedFrame().toStdString();
  selected_points_.height = 1;
  selected_points_.point_step = 4 * 4;
  selected_points_.is_dense = false;
  selected_points_.is_bigendian = false;
  selected_points_.fields.resize(4);

  selected_points_.fields[0].name = "x";
  selected_points_.fields[0].offset = 0;
  selected_points_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  selected_points_.fields[0].count = 1;

  selected_points_.fields[1].name = "y";
  selected_points_.fields[1].offset = 4;
  selected_points_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  selected_points_.fields[1].count = 1;

  selected_points_.fields[2].name = "z";
  selected_points_.fields[2].offset = 8;
  selected_points_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  selected_points_.fields[2].count = 1;

  selected_points_.fields[3].name = "intensity";
  selected_points_.fields[3].offset = 12;
  selected_points_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  selected_points_.fields[3].count = 1;

  int i = 0;
  float mean_x = 0.0;
  float mean_y = 0.0;
  float mean_z = 0.0;
  float max_intensity = 0.0;
  float min_intensity = 1e7;


  std::vector<std::array<float,4>> points_vec;
  std::array<float,4> point{0.0,0.0,0.0,0.0};

    while (model->hasIndex(i, 0))
  {
    selected_points_.row_step = (i + 1) * selected_points_.point_step;
    selected_points_.data.resize(selected_points_.row_step);

    QModelIndex child_index = model->index(i, 0);

    rviz::Property* child = model->getProp(child_index);
    rviz::VectorProperty* subchild = (rviz::VectorProperty*)child->childAt(0);
    Ogre::Vector3 point_data = subchild->getVector();

      mean_x += point_data.x;
      mean_y += point_data.y;
      mean_z += point_data.z;
      point[0] = point_data.x;
      point[1] = point_data.y;
      point[3] = point_data.z;


      uint8_t* data_pointer = &selected_points_.data[0] + i * selected_points_.point_step;
    *(float*)data_pointer = point_data.x;
    data_pointer += 4;
    *(float*)data_pointer = point_data.y;
    data_pointer += 4;
    *(float*)data_pointer = point_data.z;
    data_pointer += 4;

    // Search for the intensity value
    for (int j = 1; j < child->numChildren(); j++)
    {
      rviz::Property* grandchild = child->childAt(j);
      QString nameOfChild = grandchild->getName();
      QString nameOfIntensity("intensity");

      if (nameOfChild.contains(nameOfIntensity))
      {
        rviz::FloatProperty* floatchild = (rviz::FloatProperty*)grandchild;
        float intensity = floatchild->getValue().toFloat();
          max_intensity = (intensity>max_intensity)?intensity:max_intensity;
          min_intensity = (intensity < min_intensity)?intensity:min_intensity;

          *(float*)data_pointer = intensity;
          point[2] = intensity;

        break;
      }
    }
      points_vec.push_back(point);
    data_pointer += 4;
    i++;
  }

    points_vec_vec.push_back(points_vec);
  num_selected_points_ = i;
  ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                        "Number of points in the selected area: " << num_selected_points_);
  if(num_selected_points_ > 0){
      mean_x /= num_selected_points_;
      mean_y /= num_selected_points_;
      mean_z /= num_selected_points_;

      ROS_INFO("Select Points mean position [ %.3f %.3f %.3f ], intensity [ %.3f %.3f ]",mean_x,mean_y,mean_z, min_intensity, max_intensity );

  }
    points_center_vec.emplace_back(std::array<float,4>{ mean_x,mean_y,min_intensity,max_intensity });

  selected_points_.width = i;
  selected_points_.header.stamp = ros::Time::now();
  return 0;
}
}  // namespace rviz_plugin_selected_points_publisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_selected_points_publisher::SelectedPointsPublisher, rviz::Tool)
