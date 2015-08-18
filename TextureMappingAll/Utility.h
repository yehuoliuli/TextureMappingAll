#pragma once
#include "ZImage.h"
#include "CameraMotion.h"
#include "CameraFrame.h"
namespace WANG
{
	float DspVl(Wml::Vector2f p, ZFloatImage& DspMp);
	Wml::Vector3f ThreeDToImg(CameraFrame* pFrame, Wml::Vector3f pt3d);
	Wml::Vector3d ThreeDToImgD(CameraFrame* pFrame, Wml::Vector3f pt3d);
	Wml::Vector3f CalcCameraNorm(CameraFrame* pFrame);
}
