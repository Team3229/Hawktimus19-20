#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <memory>
#include <string>

#include <frc/GenericHID.h>
#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Camera
{
public:
    Camera();
private:
    cs::VideoSource camera;
    const int FPS = 15;
    const int BRIGHTNESS = 40;
};
#endif