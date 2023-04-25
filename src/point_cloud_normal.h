/*
 * Copyright (c) 2023, ARCS Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the ARCS Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARCS_RVIZ_POINT_CLOUD_NORMAL_H
#define ARCS_RVIZ_POINT_CLOUD_NORMAL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <deque>
#include <queue>
#include <vector>

#include <QObject>
#include <QList>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <message_filters/time_sequencer.h>

#include <pluginlib/class_loader.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#endif

namespace rviz {
class BoolProperty;
class Display;
class DisplayContext;
class EnumProperty;
class FloatProperty;
class IntProperty;
class ColorProperty;
class Arrow2;
class PixArrow;

typedef std::vector<std::string> V_string;

/**
 * \class PointCloudNormal
 * \brief Displays normals of a point cloud of type sensor_msgs::PointCloud
 */
class PointCloudNormal : public QObject {
  Q_OBJECT
 public:
  enum NormalMode {
    NM_2D_ARROW,
    NM_ARROW,
  };

  PointCloudNormal(Display* display);
  ~PointCloudNormal() override;

  void initialize(DisplayContext* context, Ogre::SceneNode* scene_node);

  void fixedFrameChanged();
  void reset();

  void addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);

  Display* getDisplay() { return display_; }

  EnumProperty* style_property_;
  ColorProperty* arrow_color_property_;
  FloatProperty* alpha_property_;
  IntProperty* buffer_length_property_;
  // for 3D arrow
  FloatProperty* arrow_shaft_length_property_;
  FloatProperty* arrow_head_length_property_;
  FloatProperty* arrow_shaft_diameter_property_;
  FloatProperty* arrow_head_diameter_property_;
  // for 2D arrow
  FloatProperty* pix_arrow_shaft_length_property_;
  FloatProperty* pix_arrow_head_length_property_;

 private Q_SLOTS:
  void updateStyle();
  void updateArrowColor();
  void updateArrowGeometry();
  void updatePixArrowGeometry();
  void updateBufferLength();

 private:
  void processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void updateStatus();

  void allocateArrowVector(std::vector<rviz::Arrow2*>& arrow_vect, size_t num);
  void allocatePixArrowVector(std::vector<rviz::PixArrow*>& pix_arrow_vect, size_t num);
  void destroyArrowChain();
  void destroyPixArrowChain();

  void pixArrow2Arrow();
  void arrow2PixArrow();

  std::vector<std::vector<rviz::Arrow2*> > arrow_chain_;
  std::vector<std::vector<rviz::PixArrow*> > pix_arrow_chain_;
  std::vector<sensor_msgs::PointCloud2ConstPtr> cloud_chain_;

  Ogre::SceneNode* scene_node_;

  Display* display_;
  DisplayContext* context_;

  size_t buffer_index_;
  NormalMode current_mode_;
};

}  // namespace rviz

#endif  // RVIZ_POINT_CLOUD_COMMON_H