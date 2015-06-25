/*
 * Copyright (c) 2013, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <string>
#include <iostream>

#include <QWidget>
#include <QDesktopWidget>
#include <QApplication>

//#include <OVR.h>

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>

#include <ros/package.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>

#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>

#include "naviman/naviman_display.h"

namespace naviman
{
navmanDisplay::navmanDisplay()
: render_widget_(0)
, scene_node_(0)
, window(0) 
{
  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);
//  connect( QApplication::desktop(), SIGNAL( screenCountChanged ( int ) ), this, SLOT( onScreenCountChanged(int)) );
  n_cameras[0] = 0, n_cameras[1] = 0;
}
navmanDisplay::~navmanDisplay()
{
  for (int i = 0; i < 1; ++i)
  { 
   // if (n_viewports[i])
   // {
    //  window->removeViewport(i);
     // n_viewports[i] = 0;
   // }
  //  if (n_cameras[i])
   // {
      n_cameras[i]->getParentSceneNode()->detachObject(n_cameras[i]);
      scene_manager_->destroyCamera(n_cameras[i]);
      n_cameras[i] = 0;
   // }
  }
  if (scene_node_)
  {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
    scene_manager_->destroySceneNode(scene_node_);
    scene_node_ = 0;
  }
//  scene_manager_ = 0;
  window = 0;
  delete render_widget_;
  n_cameras[0] = 0, n_cameras[1] = 0;
}

//Overrides from rviz Display
void navmanDisplay::onInitialize()
{
  render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("Navigation_Manipulation Space");

  render_widget_->setParent(context_->getWindowManager()->getParentWindow());
  render_widget_->setWindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint);

  window = render_widget_->getRenderWindow();
  window->setVisible(false);
  window->setAutoUpdated(false);
  window->addListener(this);

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}
void navmanDisplay::update(float wall_dt, float ros_dt)
{
  updateCamera();
  window = render_widget_->getRenderWindow();
  window->update(false);
}
void navmanDisplay::reset(){}

void navmanDisplay::updateCamera()
{

  Ogre::Vector3 pos;
  Ogre::Quaternion ori;
{
  Ogre::Quaternion r;
  r.FromAngleAxis( Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_X );
  ori = ori * r;
  r.FromAngleAxis( Ogre::Radian(-M_PI*0.5), Ogre::Vector3::UNIT_Y );
  ori = ori * r;
}
//  pos += offset_property_->getVector();
  scene_node_->setPosition(0,10,10);

  Ogre::Vector3 x_axis = ori * Ogre::Vector3(1,0,0);
  float yaw = atan2( x_axis.y, x_axis.x );// - M_PI*0.5;

  // we're working in OpenGL coordinates now
  ori.FromAngleAxis( Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z );
{
  Ogre::Quaternion r;
  r.FromAngleAxis( Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_X );
  ori = ori * r;
}
  scene_node_->setOrientation(ori);

}
//Overrides from OgreTargetListener
void navmanDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
updateCamera();
}
void navmanDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  window = render_widget_->getRenderWindow();
  window->swapBuffers();
}
void navmanDisplay::cameraSetup()
{
  Ogre::ColourValue bg_color = context_->getViewManager()->getRenderPanel()->getViewport()->getBackgroundColour();

  window = render_widget_->getRenderWindow();
  n_cameras[0] = scene_manager_->createCamera("Camera_left");
  //n_camera[1] = scene_manager_->createCamera("Camera_Right");
  scene_node_->attachObject(n_cameras[0]);
  n_cameras[0]->setNearClipDistance(0.01f);
  n_cameras[0]->setFarClipDistance(10000.0f);
  n_cameras[0]->setPosition(0,0,0);
  n_cameras[0]->lookAt(Ogre::Vector3(0,-10,-10));
  n_viewports[0] = window->addViewport(n_cameras[0],0,0,0,1.0,1.0f);
  n_viewports[0]->setBackgroundColour(bg_color);
}
void navmanDisplay::onEnable()
{
  if(!n_cameras[0])
  {
  cameraSetup();
  }
  render_widget_->setVisible(true);
}
void navmanDisplay::onDisable()
{
render_widget_ ->setVisible(false);
}

//Q_SLOTS will be populated here as necessary.

/* PLACEHOLDERS
rviz::BoolProperty* boolproperty_
rviz::FloatProperty*
rviz::TfFrameProperty*
rviz::VectorProperty*
*/
}//namespace navman
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(naviman::navmanDisplay, rviz::Display )
