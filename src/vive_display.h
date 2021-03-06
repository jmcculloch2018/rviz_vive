/*Copyright (c) 2018 André Gilerson (andre.gilerson@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.*/

#pragma once

#ifndef Q_MOC_RUN
#include "rviz/display.h"
#endif

#include <OGRE/OgreTexture.h>
#include <OGRE/RenderSystems/GL/OgreGLTexture.h>

#include <openvr.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>


namespace rviz_vive
{

class ViveDisplay: public rviz::Display
{
Q_OBJECT
public:
    ViveDisplay();
    virtual ~ViveDisplay();

    virtual void onInitialize();
    virtual void update(float wall_dt, float ros_dt);
    virtual void reset();

public Q_SLOTS:
    void updateInputTopic();
    void updateOutputTopic();
private:
    rviz::BoolProperty *_horizontalProperty;
    rviz::BoolProperty *_callibrateProperty;

    // point cloud 
    void recievePointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
    rviz::RosTopicProperty *_inputTopicProperty;
    rviz::StringProperty *_outputTopicProperty;
    bool _paused;
    bool _lastPressed;
    sensor_msgs::PointCloud2::ConstPtr _last_msg;


    Ogre::Matrix4 MatSteamVRtoOgre4(vr::HmdMatrix34_t matrix);
	Ogre::Matrix4 MatSteamVRtoOgre4(vr::HmdMatrix44_t matrix);
    Ogre::Matrix4 MakeTransformHorizontal(Ogre::Matrix4 mat); // ignores pitch / roll
 
    Ogre::Quaternion MakeQuaternionHorizontal(Ogre::Quaternion ori); 

    void handleInput();

   	vr::IVRSystem* _pHMD;

	rviz::DisplayContext* _pDisplayContext;
   	Ogre::SceneManager* _pSceneManager;

    // position of HMD when cal was last pressed
    Ogre::Vector3 _hmdCalPos; 
    
    Ogre::Camera* _pCameras[2];
    Ogre::SceneNode* _pSceneNode;
    Ogre::SceneNode* _pCameraNode;
	Ogre::Viewport* _ports[2];
	Ogre::TexturePtr _renderTextures[2];
	Ogre::RenderTexture* _pRenderTextures[2];
    GLuint _pRenderTexutresId[2];

    vr::VRTextureBounds_t GLBounds;

    vr::TrackedDevicePose_t _steamVrPose[vr::k_unMaxTrackedDeviceCount];
    Ogre::Matrix4 _trackedDevicePose[vr::k_unMaxTrackedDeviceCount];

    tf2_ros::TransformBroadcaster _broadcaster;

    ros::Publisher _update_pub;
    ros::Subscriber _point_cloud_sub;
    ros::Publisher _point_cloud_pub;

};

};
