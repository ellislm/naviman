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
#include <rviz/properties/quaternion_property.h>

#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>
#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include "naviman/naviman_display.h"

#define _x 0
#define _y 1
#define _z 2

namespace naviman
{
navmanDisplay::navmanDisplay()
: render_widget_(0)
, scene_node_(0)
, window_(0)
{
  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);
//  connect( QApplication::desktop(), SIGNAL( screenCountChanged ( int ) ), this, SLOT( onScreenCountChanged(int)) );
  n_cameras_[0] = 0, n_cameras_[1] = 0;
}
navmanDisplay::~navmanDisplay()
{
  for (int i = 0; i < 2; ++i)
  {
   if (n_viewports[i])
   {
      window_->removeViewport(i);
      n_viewports[i] = 0;
    }
    if (n_cameras_[i])
   {
      n_cameras_[i]->getParentSceneNode()->detachObject(n_cameras_[i]);
      scene_manager_->destroyCamera(n_cameras_[i]);
      n_cameras_[i] = 0;
   }
  }
  if (scene_node_)
  {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
    scene_manager_->destroySceneNode(scene_node_);
    scene_node_ = 0;
  }
//  scene_manager_ = 0;
  window_ = 0;
  delete render_widget_;
  delete use_manual_coords_;
  delete camera_Focus_;
  delete sn_Position_;
  delete xyz_Scalar_;
}

//Overrides from rviz Display
void navmanDisplay::onInitialize()
{
  use_manual_coords_ = new rviz::BoolProperty("Typed Coords",false,
          "Typed camera coordinates override hydra control",this);
//  camera_Position_ = new rviz::VectorProperty("Position",Ogre::Vector3(-0.032f,0,0),
  //                            "Position of camera to world base frame",this);
  camera_Focus_ = new rviz::VectorProperty("Focus",Ogre::Vector3(0,0,0),
                                              "Focus Point of Camera",this);
  sn_Position_ = new rviz::VectorProperty("SN Position",Ogre::Vector3(0,-5,2),
                          "Position of scene node to world base frame",this);
  xyz_Scalar_ = new rviz::VectorProperty("X,Y,Z Scalars",Ogre::Vector3(2,2,2),
                          "Scalars for X, Y, and Z of controller motion input",this);
  render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("RVinci");
  render_widget_->resize(1280,480);
  render_widget_->show();
//  render_widget_->setParent(context_->getWindowManager()->getParentWindow());
  render_widget_->setWindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint);
  window_ = render_widget_->getRenderWindow();
  window_->setVisible(false);
  window_->setAutoUpdated(false);
  window_->addListener(this);

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  pubsubSetup();
}
void navmanDisplay::update(float wall_dt, float ros_dt)
{
  updateCamera();
  window_ = render_widget_->getRenderWindow();
  window_->update(false);
}
void navmanDisplay::reset(){}

void navmanDisplay::pubsubSetup()
{
subscriber_camera_ = nh_.subscribe<razer_hydra::Hydra>("hydra_calib",10, &navmanDisplay::camsubCallback,this);
publisher_rhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("navman_cursor_right/update",10);
publisher_lhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("navman_cursor_left/update",10);
}

void navmanDisplay::camsubCallback(const razer_hydra::Hydra::ConstPtr& hydra_sub)
{
for (int i = 0; i<2; ++i)
{
cam_pos[3*i] = hydra_sub->paddles[i].transform.translation.y;
cam_pos[3*i+1] = hydra_sub->paddles[i].transform.translation.x;
cam_pos[3*i+2] = hydra_sub->paddles[i].transform.translation.z;
}
right_trigger_ = hydra_sub->paddles[1].trigger;
right_bumper_ = hydra_sub->paddles[1].buttons[0];
left_bumper_ = hydra_sub->paddles[0].buttons[0];
if (!right_bumper_)
{
interaction_cursor_msgs::InteractionCursorUpdate lhcursor;
interaction_cursor_msgs::InteractionCursorUpdate rhcursor;
rhcursor.pose.header.frame_id = "/camera_frame";
rhcursor.pose.header.stamp = ros::Time::now();
rhcursor.pose.pose.position.x = cam_pos[3];
rhcursor.pose.pose.position.y = cam_pos[4];
rhcursor.pose.pose.position.z = cam_pos[5];
//rhcursor.pose.pose.orientation = hydra_sub->paddles[1].transform.rotation();

lhcursor.pose.header.frame_id = "/camera_frame";
lhcursor.pose.header.stamp = ros::Time::now();
lhcursor.pose.pose.position.x = cam_pos[0];
lhcursor.pose.pose.position.y = cam_pos[1];
lhcursor.pose.pose.position.z = cam_pos[2];
//lhcursor.pose.pose.orientation = hydra_sub->paddles[0].transform.rotation();
publisher_rhcursor_.publish(rhcursor);
publisher_lhcursor_.publish(lhcursor);
}
}
void navmanDisplay::updateCamera()
{
  Ogre::Vector3 xyzscale = xyz_Scalar_->getVector();
  for (int i = 0; i<2; ++i)
  {
  if(use_manual_coords_->getBool())
    {
      scene_node_->setPosition(sn_Position_->getVector());
      n_cameras_[i]->lookAt(camera_Focus_->getVector());
      n_cameras_[i]->setFixedYawAxis(true, scene_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
    }
  if(!use_manual_coords_->getBool() && right_bumper_)
    {
     camera_Focus_->setVector(Ogre::Vector3(cam_pos[_x+3],cam_pos[_y+3],cam_pos[_z+3])*xyzscale);
     // n_cameras_[i]->setPosition(cam_pos[_x]*5,cam_pos[_y]*5,cam_pos[_z]*5);
     n_cameras_[i]->setFixedYawAxis(true, scene_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
     n_cameras_[i]->lookAt(camera_Focus_->getVector()*xyzscale);
     hydra_base_tf_.setOrigin(tf::Vector3(cam_pos[_x+3]*xyzscale.x,cam_pos[_y+3]*xyzscale.y,cam_pos[_z+3]*xyzscale.z));
     hydra_base_tf_.setRotation(tf::Quaternion(0.0,0.0,0.0,1));
     if(left_bumper_)
      {
        scene_node_->setPosition(sn_Position_->getVector()+Ogre::Vector3(cam_pos[_x+3],cam_pos[_y+3],cam_pos[_z+3])*xyzscale);
      }
    
    }
  }
  br_.sendTransform(tf::StampedTransform(hydra_base_tf_, ros::Time::now(), "base_link","/camera_frame"));
}
//Overrides from OgreTargetListener
void navmanDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
updateCamera();
}
void navmanDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  window_ = render_widget_->getRenderWindow();
  window_->swapBuffers();
}
void navmanDisplay::cameraSetup()
{
  Ogre::ColourValue bg_color = context_->getViewManager()->getRenderPanel()->getViewport()->getBackgroundColour();
  window_ = render_widget_->getRenderWindow();
  n_cameras_[0] = scene_manager_->createCamera("Camera_Left");
  n_cameras_[1] = scene_manager_->createCamera("Camera_Right");
  scene_node_->setOrientation(Ogre::Quaternion(0,0,0,1));
  scene_node_->setPosition(sn_Position_->getVector());
  for(int i = 0; i<2; ++i)
  {
  scene_node_->attachObject(n_cameras_[i]);
  n_cameras_[i]->setNearClipDistance(0.01f);
  n_cameras_[i]->setFarClipDistance(10000.0f);
  n_cameras_[i]->setPosition((i*2 -1)*0.064f*0.5f,0,0);
  n_cameras_[i]->lookAt(0,0,0);
  n_viewports[i] = window_->addViewport(n_cameras_[i],i,0.5f*i,0,0.5f,1.0f);
  n_viewports[i]->setBackgroundColour(bg_color);
  }
}
void navmanDisplay::onEnable()
{
  if(!n_cameras_[0])
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
