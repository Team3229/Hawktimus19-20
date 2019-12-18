// Author: Team 3229 Programming Subteam

#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <memory>
#include <string>

//frc includes
#include <frc/GenericHID.h>
#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>
#include <frc/smartdashboard/Smartdashboard.h>

class Camera
{
public:
	Camera();

private:
	cs::VideoSource camera;

	// Constants for Camera Settings
	const int FPS = 15;
	const int BRIGHTNESS = 40;

};

#endif // CAMERA_H