/*Copyright (c) 2018 André Gilerson (andre.gilerson@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.*/

#include "vive_display.h"

#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/properties/bool_property.h>

#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <OGRE/OgreRoot.h>

#include <RenderSystems/GL/OgreGLTextureManager.h>

const float g_defaultIPD = 0.064f;

namespace rviz_vive
{

ViveDisplay::ViveDisplay()
{
	GLBounds = {};
	GLBounds.uMin = 0;
	GLBounds.uMax = 1;
	GLBounds.vMin = 1;
	GLBounds.vMax = 0;
}

ViveDisplay::~ViveDisplay()
{

}

void ViveDisplay::onInitialize()
{
    vr::EVRInitError eError = vr::VRInitError_None;
	_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );

	if ( eError != vr::VRInitError_None )
	{
		_pHMD = NULL;
		std::cout << "Unable to init VR runtime";
    }

  	vr::EVRInitError peError = vr::VRInitError_None;
    if ( !vr::VRCompositor() )
	{
		std::cout << "Compositor initialization failed. See log file for details";
	}

	_pDisplayContext = context_;
    _pSceneManager = scene_manager_;
	_pSceneNode = _pSceneManager->getRootSceneNode()->createChildSceneNode();

    if (_pSceneNode)
		_pCameraNode = _pSceneNode->createChildSceneNode("StereoCameraNode");
	else
		_pCameraNode = _pSceneManager->getRootSceneNode()->createChildSceneNode("StereoCameraNode");

    _pCameras[0] = _pSceneManager->createCamera("CameraLeft");
	_pCameras[1] = _pSceneManager->createCamera("CameraRight");

	Ogre::GLTextureManager* textureManager = static_cast<Ogre::GLTextureManager*>(Ogre::TextureManager::getSingletonPtr());
    _renderTextures[0]= Ogre::TextureManager::getSingleton().createManual(
		"RenderTexture1", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,
		1512, 1680, 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET);
	_renderTextures[1] = Ogre::TextureManager::getSingleton().createManual(
		"RenderTexture2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,
		1512, 1680, 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET);
	_pRenderTextures[0] = _renderTextures[0]->getBuffer()->getRenderTarget();
    _pRenderTexutresId[0] = static_cast<Ogre::GLTexture*>(textureManager->getByName("RenderTexture1").getPointer())->getGLID();
    _pRenderTextures[1] = _renderTextures[1]->getBuffer()->getRenderTarget();
    _pRenderTexutresId[1] = static_cast<Ogre::GLTexture*>(textureManager->getByName("RenderTexture2").getPointer())->getGLID();

	vr::HmdMatrix44_t prj[2] = { 
		_pHMD->GetProjectionMatrix(vr::Eye_Left, 0.01, 100),
		_pHMD->GetProjectionMatrix(vr::Eye_Right, 0.01, 100)};

	vr::HmdMatrix34_t e2hTransform[2] = {
		_pHMD->GetEyeToHeadTransform(vr::Eye_Left),
		_pHMD->GetEyeToHeadTransform(vr::Eye_Right),
	};

    for (int i = 0; i < 2; ++i)
	{
		_pCameras[i]->detachFromParent();
		_pCameraNode->attachObject(_pCameras[i]);
		_pCameras[i]->setPosition(MatSteamVRtoOgre4(e2hTransform[i]).getTrans());
		_pCameras[i]->setCustomProjectionMatrix(true, MatSteamVRtoOgre4(prj[i]));

        _ports[i] = _pRenderTextures[i]->addViewport(_pCameras[i]);
        _ports[i]->setClearEveryFrame(true);
		_ports[i]->setBackgroundColour(Ogre::ColourValue::Black);
        _ports[i]->setOverlaysEnabled(false);
	}

	// Set up properties
	_horizontalProperty = new rviz::BoolProperty( "Fixed Horizon", true, "If checked, will ignore the pitch component of the RViz camera.", this);
	_callibrateProperty = new rviz::BoolProperty( "Calibrate", false, "If checked, will reset view in vive to match rviz camera position", this);
	_inputTopicProperty = new rviz::ROSTopicStringProperty( "Input PointCloud Topic", "", "POINT_CLOUD_MESSAGE_TYPE", 
		"Topic on which rgbd point cloud is published", this);
	_outputTopicProperty = new rviz::ROSTopicStringProperty( "Input PointCloud Topic", "", "POINT_CLOUD_MESSAGE_TYPE", 
		"Topic on which rgbd point cloud is published", this);

	_stringProperty = new rviz::StringProperty( "Point Cloud Input Channel")
	// set up publisher
	_update_pub = update_nh_.advertise<std_msgs::String>("viveUpdate", 1000);

	_hmdCalPos = Ogre::Vector3::ZERO;

}

void ViveDisplay::update(float wall_dt, float ros_dr) {
	// wait for new vive data
	handleInput();

	// get position of camera
	Ogre::Camera *cam = _pDisplayContext->getViewManager()->getCurrent()->getCamera();
	Ogre::Vector3 pos = cam->getDerivedPosition();
	Ogre::Quaternion ori = cam->getDerivedOrientation();

	// ignore every component except yaw if horizontal checked
	if (_horizontalProperty->getBool()) {
		ori = MakeQuaternionHorizontal(ori);
	}

	// if callibrate checked store hmd position
	if (_callibrateProperty->getBool() && 
		_steamVrPose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
		_hmdCalPos = _trackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].getTrans();
		_callibrateProperty->setBool(false);
	}

	pos = pos - ori * _hmdCalPos; // adjust for hmd position so when callibrate head is at camera position

	// set scene node position (base of vr world frame)
	_pSceneNode->setPosition(pos);
	_pSceneNode->setOrientation(ori);

	// set camera position based on hmd pose
	if (_steamVrPose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
	{
		Ogre::Vector3 vivePos = _trackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].getTrans();
		Ogre::Quaternion viveOri = _trackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].extractQuaternion();
		_pCameraNode->setPosition(vivePos);
		_pCameraNode->setOrientation(viveOri);

	}

	// update cameras
    _pRenderTextures[0]->update(true);
    _pRenderTextures[1]->update(true);

    // send images to vive
	if (_pHMD) 
	{
		vr::Texture_t leftEyeTexture = {(void*)_pRenderTexutresId[0], vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
		vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture,   &GLBounds );
		vr::Texture_t rightEyeTexture = {(void*)_pRenderTexutresId[1], vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
		vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture, &GLBounds);
    }

    // publish base transform to ros topic
    
    geometry_msgs::TransformStamped msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base";
    msg.child_frame_id = "world";
    
    msg.transform.translation.x = pos.x;
    msg.transform.translation.y = pos.y;
    msg.transform.translation.z = pos.z;
    
    msg.transform.rotation.w = ori.w;
    msg.transform.rotation.x = ori.x;
    msg.transform.rotation.y = ori.y;
    msg.transform.rotation.z = ori.z;
	
	_broadcaster.sendTransform(msg);

	// publish message so we know update frequency
	std_msgs::String strMsg;
  	strMsg.data = "update";
  	_update_pub.publish(strMsg);
}

void ViveDisplay::reset()
{

}

Ogre::Matrix4 ViveDisplay::MatSteamVRtoOgre4(vr::HmdMatrix34_t matrix)
{
	return Ogre::Matrix4 (
		matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
		matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
		matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
		0.0, 0.0, 0.0, 1.0f
		);
}

Ogre::Matrix4 ViveDisplay::MatSteamVRtoOgre4(vr::HmdMatrix44_t matrix)
{
	return Ogre::Matrix4 (
		matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
		matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
		matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
		matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]
		);
}

// ignores pitch / roll
Ogre::Matrix4 ViveDisplay::MakeTransformHorizontal(Ogre::Matrix4 mat) {
	// extract trans and quat
	Ogre::Vector3 trans = mat.getTrans();
	Ogre::Quaternion ori = mat.extractQuaternion();

	// transform x_axis by quat to get its yaw
	Ogre::Vector3 x_axis = ori * Ogre::Vector3(1,0,0);
	float yaw = atan2( x_axis.y, x_axis.x );// - M_PI*0.5;

	// we're working in OpenGL coordinates now
	ori.FromAngleAxis( Ogre::Radian(yaw), Ogre::Vector3::UNIT_X );
	Ogre::Quaternion r;
	r.FromAngleAxis( Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_X );
	// ori = ori * r;

	//reconstruct matrix
	mat.makeTransform(trans, Ogre::Vector3(1, 1, 1), ori);
	return mat;

}

Ogre::Quaternion ViveDisplay::MakeQuaternionHorizontal(Ogre::Quaternion ori) {
	// transform x_axis by quat to get its yaw
	Ogre::Vector3 x_axis = ori * Ogre::Vector3(1,0,0);
	float yaw = atan2( x_axis.y, x_axis.x );// - M_PI*0.5;

	// we're working in OpenGL coordinates now
	ori.FromAngleAxis( Ogre::Radian(yaw), Ogre::Vector3::UNIT_Z );
	Ogre::Quaternion r;
	r.FromAngleAxis( Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_X );
	ori = ori * r;

	return ori;

}

void ViveDisplay::handleInput()
{
	vr::VRCompositor()->WaitGetPoses(_steamVrPose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

	for (uint32_t nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice)
	{
		if (_steamVrPose[nDevice].bPoseIsValid)
		{
			_trackedDevicePose[nDevice] = MatSteamVRtoOgre4(_steamVrPose[nDevice].mDeviceToAbsoluteTracking);
		}
	}
}

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_vive::ViveDisplay, rviz::Display)
