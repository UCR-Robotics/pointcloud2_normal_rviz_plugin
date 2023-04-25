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

#include "pix_arrow.h"

#include <sstream>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

namespace rviz {
PixArrow::PixArrow(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node) : Object(manager) {
  if (!parent_node) {
    parent_node = manager->getRootSceneNode();
  }
  manual_object_ = manager->createManualObject();
  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "LineMaterial" << count++;

  // NOTE: The second parameter to the create method is the resource group the material will be added to.
  // If the group you name does not exist (in your resources.cfg file) the library will assert() and your
  // program will crash
  manual_object_material_ =
      Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  manual_object_material_->setReceiveShadows(false);
  manual_object_material_->getTechnique(0)->setLightingEnabled(true);
  manual_object_material_->getTechnique(0)->getPass(0)->setDiffuse(0, 0, 0, 0);
  manual_object_material_->getTechnique(0)->getPass(0)->setAmbient(1, 1, 1);

  scene_node_->attachObject(manual_object_);
}

PixArrow::~PixArrow() {
  if (scene_node_->getParentSceneNode()) {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
  }
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(manual_object_);
  Ogre::MaterialManager::getSingleton().remove(manual_object_material_->getName());
}

void PixArrow::set(float shaft_length, float head_length) {
  Ogre::Vector3 start(0, 0, 0);
  Ogre::Vector3 end(0, shaft_length, 0);
  // 50 degrees arrow head, 25 degrees each side. sin(25deg)=0.4226, cos(25deg)=0.9063
  Ogre::Vector3 left(-0.4226 * head_length, shaft_length - 0.9063 * head_length, 0);
  Ogre::Vector3 right(0.4226 * head_length, shaft_length - 0.9063 * head_length, 0);

  manual_object_->clear();
  manual_object_->begin(manual_object_material_->getName(), Ogre::RenderOperation::OT_LINE_LIST);
  manual_object_->position(start);
  manual_object_->position(end);
  manual_object_->position(left);
  manual_object_->position(end);
  manual_object_->position(right);
  manual_object_->position(end);
  manual_object_->end();
  setVisible(true);
}

void PixArrow::setVisible(bool visible) { scene_node_->setVisible(visible, true); }

void PixArrow::setPosition(const Ogre::Vector3& position) { scene_node_->setPosition(position); }

void PixArrow::setDirection(const Ogre::Vector3& direction) {
  if (!direction.isZeroLength()) {
    setOrientation(Ogre::Vector3::UNIT_Y.getRotationTo(direction));
  }
}

void PixArrow::setOrientation(const Ogre::Quaternion& orientation) { scene_node_->setOrientation(orientation); }

void PixArrow::setScale(const Ogre::Vector3& scale) { scene_node_->setScale(scale); }

void PixArrow::setColor(const Ogre::ColourValue& c) {
  // this is consistent with the behaviour in the Shape class.

  // manual_object_material_->getTechnique(0)->setAmbient(c * 0.5);
  // manual_object_material_->getTechnique(0)->setDiffuse(c);
  manual_object_material_->getTechnique(0)->getPass(0)->setEmissive(c);

  if (c.a < 0.9998) {
    manual_object_material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    manual_object_material_->getTechnique(0)->setDepthWriteEnabled(false);
  } else {
    manual_object_material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    manual_object_material_->getTechnique(0)->setDepthWriteEnabled(true);
  }
}

void PixArrow::setColor(float r, float g, float b, float a) { setColor(Ogre::ColourValue(r, g, b, a)); }

// where are the void Line::setColour(...) convenience methods??? ;)

const Ogre::Vector3& PixArrow::getPosition() { return scene_node_->getPosition(); }

const Ogre::Quaternion& PixArrow::getOrientation() { return scene_node_->getOrientation(); }

void PixArrow::setUserData(const Ogre::Any& data) { manual_object_->getUserObjectBindings().setUserAny(data); }

}  // namespace rviz