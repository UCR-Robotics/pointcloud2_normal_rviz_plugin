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

#include <QColor>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include <ros/time.h>

#include <pluginlib/class_loader.hpp>

#include <rviz/default_plugin/point_cloud_transformer.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/validate_floats.h>

#include "arrow.h"
#include "pix_arrow.h"
#include "point_cloud_normal.h"

namespace rviz {

PointCloudNormal::PointCloudNormal(Display* display) : display_(display), buffer_index_(0), current_mode_(NM_ARROW) {
  style_property_ = new EnumProperty("Style", "Arrow", "Rendering mode to use, in order of computational complexity.",
                                     display_, SLOT(updateStyle()), this);
  style_property_->addOption("2D Arrow", PointCloudNormal::NM_2D_ARROW);
  style_property_->addOption("Arrow", PointCloudNormal::NM_ARROW);

  arrow_color_property_ = new ColorProperty("Arrow Color", QColor(0x66, 0xcc, 0xff), "Color of the normal arrows",
                                            display_, SLOT(updateArrowColor()), this);

  alpha_property_ = new FloatProperty("Alpha", 1.0,
                                      "Amount of transparency to apply to the points. "
                                      "Note that this is experimental and does not always look correct.",
                                      display_, SLOT(updateArrowColor()), this);
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  buffer_length_property_ =
      new IntProperty("Buffer Length", 1, "Number of frames to display.", display_, SLOT(updateBufferLength()), this);
  buffer_length_property_->setMin(1);

  arrow_shaft_length_property_ =
      new FloatProperty("Shaft Length", 0.3, "Length of the arrow shaft.", display_, SLOT(updateArrowGeometry()), this);
  arrow_head_length_property_ =
      new FloatProperty("Head Length", 0.2, "Length of the arrow head.", display_, SLOT(updateArrowGeometry()), this);
  arrow_shaft_diameter_property_ = new FloatProperty("Shaft Diameter", 0.1, "Diameter of the arrow shaft.", display_,
                                                     SLOT(updateArrowGeometry()), this);
  arrow_head_diameter_property_ = new FloatProperty("Head Diameter", 0.3, "Diameter of the arrow head.", display_,
                                                    SLOT(updateArrowGeometry()), this);

  pix_arrow_shaft_length_property_ = new FloatProperty("2D Arrow Shaft Length", 0.5, "Length of the 2D arrow shaft.",
                                                       display_, SLOT(updatePixArrowGeometry()), this);
  pix_arrow_head_length_property_ = new FloatProperty("2D Arrow Head Length", 0.2, "Length of the 2D arrow head.",
                                                      display_, SLOT(updatePixArrowGeometry()), this);
  pix_arrow_shaft_length_property_->hide();
  pix_arrow_head_length_property_->hide();
}

void PointCloudNormal::initialize(DisplayContext* context, Ogre::SceneNode* scene_node) {
  context_ = context;
  scene_node_ = scene_node;

  updateStyle();
  updateArrowColor();
  updateArrowGeometry();
  updatePixArrowGeometry();
  updateBufferLength();
}

PointCloudNormal::~PointCloudNormal() {
  destroyArrowChain();
  destroyPixArrowChain();
}

void PointCloudNormal::updateBufferLength() {
  int new_buffer_length = buffer_length_property_->getInt();
  if (new_buffer_length == arrow_chain_.size()) return;

  // create new containers
  std::vector<std::vector<rviz::Arrow2*> > arrow_chain(new_buffer_length);
  std::vector<std::vector<rviz::PixArrow*> > pix_arrow_chain(new_buffer_length);
  std::vector<sensor_msgs::PointCloud2ConstPtr> cloud_chain(new_buffer_length);

  if (new_buffer_length > arrow_chain_.size()) {
    size_t offset = new_buffer_length - arrow_chain_.size();
    for (size_t i = 0; i < arrow_chain_.size(); ++i) {
      arrow_chain[i + offset].swap(arrow_chain_[(i + buffer_index_) % arrow_chain_.size()]);
      pix_arrow_chain[i + offset].swap(pix_arrow_chain_[(i + buffer_index_) % arrow_chain_.size()]);
      cloud_chain[i + offset].swap(cloud_chain_[(i + buffer_index_) % arrow_chain_.size()]);
    }
    buffer_index_ = 0;
  } else if (new_buffer_length < arrow_chain_.size()) {
    for (size_t i = 0; i < arrow_chain_.size(); ++i) {
      arrow_chain[i % new_buffer_length].swap(arrow_chain_[(i + buffer_index_) % arrow_chain_.size()]);
      pix_arrow_chain[i % new_buffer_length].swap(pix_arrow_chain_[(i + buffer_index_) % arrow_chain_.size()]);
      cloud_chain[i % new_buffer_length].swap(cloud_chain_[(i + buffer_index_) % arrow_chain_.size()]);
    }
    buffer_index_ = 0;
  }
  // destroy unused arrows
  destroyArrowChain();
  destroyPixArrowChain();

  // replace old buffer by new buffer
  arrow_chain_.swap(arrow_chain);
  pix_arrow_chain_.swap(pix_arrow_chain);
  cloud_chain_.swap(cloud_chain);

  context_->queueRender();
}

void PointCloudNormal::arrow2PixArrow() {
  destroyPixArrowChain();

  float head_length = pix_arrow_head_length_property_->getFloat();
  float shaft_length = pix_arrow_shaft_length_property_->getFloat();

  QColor color = arrow_color_property_->getColor();
  float alpha = alpha_property_->getFloat();

  pix_arrow_chain_.resize(arrow_chain_.size());
  for (size_t i = 0; i < arrow_chain_.size(); ++i) {
    auto& arrow_vect = arrow_chain_[i];
    auto& pix_arrow_vect = pix_arrow_chain_[i];
    allocatePixArrowVector(pix_arrow_vect, arrow_vect.size());
    for (size_t j = 0; j < arrow_vect.size(); ++j) {
      pix_arrow_vect[j]->set(shaft_length, head_length);
      pix_arrow_vect[j]->setPosition(arrow_vect[j]->getPosition());
      pix_arrow_vect[j]->setOrientation(arrow_vect[j]->getOrientation());
      pix_arrow_vect[j]->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
    }
  }
}

void PointCloudNormal::pixArrow2Arrow() {
  destroyArrowChain();

  float shaft_length = arrow_shaft_length_property_->getFloat();
  float shaft_diameter = arrow_shaft_diameter_property_->getFloat();
  float head_length = arrow_head_length_property_->getFloat();
  float head_diameter = arrow_head_diameter_property_->getFloat();

  QColor color = arrow_color_property_->getColor();
  float alpha = alpha_property_->getFloat();

  arrow_chain_.resize(pix_arrow_chain_.size());
  for (size_t i = 0; i < pix_arrow_chain_.size(); ++i) {
    auto& pix_arrow_vect = pix_arrow_chain_[i];
    auto& arrow_vect = arrow_chain_[i];
    allocateArrowVector(arrow_vect, pix_arrow_vect.size());
    for (size_t j = 0; j < pix_arrow_vect.size(); ++j) {
      arrow_vect[j]->set(shaft_length, shaft_diameter, head_length, head_diameter);
      arrow_vect[j]->setPosition(pix_arrow_vect[j]->getPosition());
      arrow_vect[j]->setOrientation(pix_arrow_vect[j]->getOrientation());
      arrow_vect[j]->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
    }
  }
}

void PointCloudNormal::updateStyle() {
  PointCloudNormal::NormalMode mode = (PointCloudNormal::NormalMode)style_property_->getOptionInt();
  if (mode == current_mode_) return;

  if (mode == PointCloudNormal::NM_ARROW) {
    arrow_head_diameter_property_->show();
    arrow_head_length_property_->show();
    arrow_shaft_diameter_property_->show();
    arrow_shaft_length_property_->show();
    pix_arrow_shaft_length_property_->hide();
    pix_arrow_head_length_property_->hide();

    pixArrow2Arrow();
    destroyPixArrowChain();
  } else {
    arrow_head_diameter_property_->hide();
    arrow_head_length_property_->hide();
    arrow_shaft_diameter_property_->hide();
    arrow_shaft_length_property_->hide();
    pix_arrow_shaft_length_property_->show();
    pix_arrow_head_length_property_->show();

    arrow2PixArrow();
    destroyArrowChain();
  }
  current_mode_ = mode;
  context_->queueRender();
}

void PointCloudNormal::updateArrowGeometry() {
  float shaft_length = arrow_shaft_length_property_->getFloat();
  float shaft_diameter = arrow_shaft_diameter_property_->getFloat();
  float head_length = arrow_head_length_property_->getFloat();
  float head_diameter = arrow_head_diameter_property_->getFloat();
  for (size_t i = 0; i < arrow_chain_.size(); ++i) {
    auto& arrow_vect = arrow_chain_[i];
    for (size_t j = 0; j < arrow_vect.size(); ++j) {
      arrow_vect[j]->set(shaft_length, shaft_diameter, head_length, head_diameter);
    }
  }
  context_->queueRender();
}

void PointCloudNormal::updatePixArrowGeometry() {
  float head_length = pix_arrow_head_length_property_->getFloat();
  float shaft_length = pix_arrow_shaft_length_property_->getFloat();

  for (size_t i = 0; i < pix_arrow_chain_.size(); ++i) {
    auto& pix_arrow_vect = pix_arrow_chain_[i];
    for (size_t j = 0; j < pix_arrow_vect.size(); ++j) {
      pix_arrow_vect[j]->set(shaft_length, head_length);
    }
  }
  context_->queueRender();
}

void PointCloudNormal::updateArrowColor() {
  QColor color = arrow_color_property_->getColor();
  float alpha = alpha_property_->getFloat();

  PointCloudNormal::NormalMode mode = (PointCloudNormal::NormalMode)style_property_->getOptionInt();
  if (mode == PointCloudNormal::NM_ARROW) {
    for (size_t i = 0; i < arrow_chain_.size(); ++i) {
      auto& arrow_vect = arrow_chain_[i];
      for (size_t j = 0; j < arrow_vect.size(); ++j) {
        arrow_vect[j]->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
      }
    }
  } else {
    for (size_t i = 0; i < pix_arrow_chain_.size(); ++i) {
      auto& pix_arrow_vect = pix_arrow_chain_[i];
      for (size_t j = 0; j < pix_arrow_vect.size(); ++j) {
        pix_arrow_vect[j]->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
      }
    }
  }
  context_->queueRender();
}

void PointCloudNormal::reset() {
  destroyArrowChain();
  destroyPixArrowChain();
  cloud_chain_.clear();

  size_t buffer_length = buffer_length_property_->getInt();
  arrow_chain_.resize(buffer_length);
  pix_arrow_chain_.resize(buffer_length);
  cloud_chain_.resize(buffer_length);
}

void PointCloudNormal::allocateArrowVector(std::vector<rviz::Arrow2*>& arrow_vect, size_t num) {
  if (num > arrow_vect.size()) {
    for (size_t i = arrow_vect.size(); i < num; ++i) {
      rviz::Arrow2* arrow = new rviz::Arrow2(context_->getSceneManager(), scene_node_);
      arrow_vect.push_back(arrow);
    }
  } else if (num < arrow_vect.size()) {
    for (size_t i = num; i < arrow_vect.size(); ++i) {
      delete arrow_vect[i];
    }
    arrow_vect.resize(num);
  }
}

void PointCloudNormal::allocatePixArrowVector(std::vector<rviz::PixArrow*>& pix_arrow_vect, size_t num) {
  if (num > pix_arrow_vect.size()) {
    for (size_t i = pix_arrow_vect.size(); i < num; ++i) {
      PixArrow* pix_arrow = new PixArrow(context_->getSceneManager(), scene_node_);
      pix_arrow_vect.push_back(pix_arrow);
    }
  } else if (num < pix_arrow_vect.size()) {
    for (size_t i = num; i < pix_arrow_vect.size(); ++i) {
      delete pix_arrow_vect[i];
    }
    pix_arrow_vect.resize(num);
  }
}

void PointCloudNormal::destroyArrowChain() {
  for (size_t i = 0; i < arrow_chain_.size(); ++i) {
    allocateArrowVector(arrow_chain_[i], 0);
  }
  arrow_chain_.resize(0);
}

void PointCloudNormal::destroyPixArrowChain() {
  for (size_t i = 0; i < pix_arrow_chain_.size(); ++i) {
    allocatePixArrowVector(pix_arrow_chain_[i], 0);
  }
  pix_arrow_chain_.resize(0);
}

void PointCloudNormal::updateStatus() {
  std::stringstream ss;
  // ss << "Showing [" << total_point_count_ << "] points from [" << clouds_.size() << "] messages";
  display_->setStatusStd(StatusProperty::Ok, "Points", ss.str());
}

void PointCloudNormal::processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  size_t buffer_length = buffer_length_property_->getInt();
  cloud_chain_[buffer_index_] = cloud;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(cloud->header, position, orientation)) {
    ROS_ERROR("Error transforming from frame '%s' to frame '%s'", cloud->header.frame_id.c_str(),
              qPrintable(context_->getFixedFrame()));
  }

  Ogre::Matrix4 transform(orientation);
  transform.setTrans(position);

  Ogre::ColourValue color = arrow_color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  NormalMode mode = (NormalMode)style_property_->getOptionInt();

  uint32_t num_points = cloud->width;

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");
  int32_t normal_xi = findChannelIndex(cloud, "normal_x");
  int32_t normal_yi = findChannelIndex(cloud, "normal_y");
  int32_t normal_zi = findChannelIndex(cloud, "normal_z");
  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t normal_xoff = cloud->fields[normal_xi].offset;
  const uint32_t normal_yoff = cloud->fields[normal_yi].offset;
  const uint32_t normal_zoff = cloud->fields[normal_zi].offset;
  const uint32_t point_step = cloud->point_step;

  if (mode == NM_ARROW) {
    float shaft_length = arrow_shaft_length_property_->getFloat();
    float shaft_diameter = arrow_shaft_diameter_property_->getFloat();
    float head_length = arrow_head_length_property_->getFloat();
    float head_diameter = arrow_head_diameter_property_->getFloat();

    auto& arrow_vect = arrow_chain_[buffer_index_];
    allocateArrowVector(arrow_vect, num_points);
    for (uint32_t i = 0; i < num_points; ++i) {
      const uint8_t* ptr = cloud->data.data() + i * point_step;
      float x = *reinterpret_cast<const float*>(ptr + xoff);
      float y = *reinterpret_cast<const float*>(ptr + yoff);
      float z = *reinterpret_cast<const float*>(ptr + zoff);
      float normal_x = *reinterpret_cast<const float*>(ptr + normal_xoff);
      float normal_y = *reinterpret_cast<const float*>(ptr + normal_yoff);
      float normal_z = *reinterpret_cast<const float*>(ptr + normal_zoff);

      Ogre::Vector3 xpos = transform * Ogre::Vector3(x, y, z);
      Ogre::Vector3 xdir = orientation * Ogre::Vector3(normal_x, normal_y, normal_z);
      arrow_vect[i]->setColor(color);
      arrow_vect[i]->set(shaft_length, shaft_diameter, head_length, head_diameter);
      arrow_vect[i]->setPosition(xpos);
      arrow_vect[i]->setDirection(xdir);
    }
  } else if (mode == NM_2D_ARROW) {
    float head_length = pix_arrow_head_length_property_->getFloat();
    float shaft_length = pix_arrow_shaft_length_property_->getFloat();

    auto& pix_arrow_vect = pix_arrow_chain_[buffer_index_];
    allocatePixArrowVector(pix_arrow_vect, num_points);
    for (size_t i = 0; i < pix_arrow_vect.size(); ++i) {
      const uint8_t* ptr = cloud->data.data() + i * point_step;
      float x = *reinterpret_cast<const float*>(ptr + xoff);
      float y = *reinterpret_cast<const float*>(ptr + yoff);
      float z = *reinterpret_cast<const float*>(ptr + zoff);
      float normal_x = *reinterpret_cast<const float*>(ptr + normal_xoff);
      float normal_y = *reinterpret_cast<const float*>(ptr + normal_yoff);
      float normal_z = *reinterpret_cast<const float*>(ptr + normal_zoff);

      Ogre::Vector3 xpos = transform * Ogre::Vector3(x, y, z);
      Ogre::Vector3 xdir = orientation * Ogre::Vector3(normal_x, normal_y, normal_z);
      pix_arrow_vect[i]->set(shaft_length, head_length);
      pix_arrow_vect[i]->setColor(color);
      pix_arrow_vect[i]->setPosition(xpos);
      // pix_arrow_vect[i]->setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Y.getRotationTo(xdir));
      pix_arrow_vect[i]->setDirection(xdir);
    }
  } else {
    ROS_ERROR("Error arrow style '%d'", mode);
  }
  context_->queueRender();

  buffer_index_ += 1;
  buffer_index_ %= buffer_length;
}

void PointCloudNormal::addMessage(const sensor_msgs::PointCloud2ConstPtr& cloud) { processMessage(cloud); }

void PointCloudNormal::fixedFrameChanged() { reset(); }

}  // namespace rviz