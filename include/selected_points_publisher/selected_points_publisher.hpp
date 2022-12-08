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
 * selected_points_publisher.hpp
 *
 *  Created on: December 5th 2019
 */

#ifndef SELECTED_POINTS_PUBLISHER_HPP
#define SELECTED_POINTS_PUBLISHER_HPP

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include "rviz/tool.h"
#include <QCursor>
#include <QObject>
#endif
#include "visualization_msgs/MarkerArray.h"

#include <sensor_msgs/PointCloud2.h>
#include "rviz/default_plugin/tools/selection_tool.h"
#include <vector>
#include <array>

namespace rviz_plugin_selected_points_publisher
{
class SelectedPointsPublisher;

class SelectedPointsPublisher : public rviz::SelectionTool
{
  Q_OBJECT
public:
  SelectedPointsPublisher();
  virtual ~SelectedPointsPublisher();
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
  virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);

public Q_SLOTS:
  void updateTopic();

protected:
  int processSelectedArea();
  ros::NodeHandle node_handle_;
  ros::Publisher rviz_selected_publisher_;
  ros::Subscriber pointcloud_subscriber_;

    ros::Publisher marker_pub_;

    std::string tf_frame_;
  std::string rviz_cloud_topic_;
    std::string rviz_marker_topic_;

    std::string subscribed_cloud_topic_;
    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker  marker_msg;

  sensor_msgs::PointCloud2 selected_points_;
    std::vector<std::vector<std::array<float,4>> > points_vec_vec;
    std::vector<std::array<float,4>> points_center_vec;

  bool selecting_;
  int num_selected_points_;

  void dumpData();
};
}  // namespace rviz_plugin_selected_points_publisher

#endif  // SELECTED_POINTS_PUBLISHER_HPP
