#include "Utility.h"
namespace WANG
{
	float DspVl(Wml::Vector2f p, ZFloatImage& DspMp)
	{
		//TODO 此处修改 DSPvl
		int x = int(p.X());
		int y = int(p.Y());

		float dx = p.X() - x;
		float dy = p.Y() - y;
		if(x == DspMp.GetWidth() - 1)
			x--;
		if(y == DspMp.GetHeight() - 1)
			y--;
		//TODO 这块之前改的
		//return DspMp.at(p.X()+0.5, p.Y()+0.5);

		return ((1.0 - dx) * (1.0 - dy) * DspMp.at(x, y)
			+ dx * (1.0 - dy) * DspMp.at(x + 1, y)
			+ (1.0 - dx) * dy * DspMp.at(x, y + 1)
			+ dx * dy * DspMp.at(x + 1, y + 1));
	}
	Wml::Vector3f ThreeDToImg(CameraFrame* pFrame, Wml::Vector3f pt3d)
	{
		Wml::Matrix4d& P = pFrame->m_objAbsTransformMG;
		Wml::Vector3d v_3d;

		for(int i=0; i<3; ++i){
			v_3d[i] = P(i,3);
			for(int j=0; j<3; ++j){
				v_3d[i] += P(i,j) * pt3d[j];
			}
		}

		double fx = pFrame->m_K(0,0) * pFrame->m_fscale;
		double fy = pFrame->m_K(1,1) * pFrame->m_fscale;
		double skew = pFrame->m_K(0,1) * pFrame->m_fscale;
		double cx = pFrame->m_K(0,2);
		double cy = pFrame->m_K(1,2);

		Wml::Vector3f imgPt;

		v_3d[0] /= v_3d[2];
		v_3d[1] /= v_3d[2];

		imgPt.X() = v_3d[0] * fx + v_3d[1] * skew + cx;
		imgPt.Y() = v_3d[1] * fy + cy;
		imgPt.Z() = v_3d[2];

		return imgPt;
	}

	Wml::Vector3d ThreeDToImgD(CameraFrame* pFrame, Wml::Vector3f pt3d)
	{
		Wml::Matrix4d& P = pFrame->m_objAbsTransformMG;
		Wml::Vector3d v_3d;

		for(int i=0; i<3; ++i){
			v_3d[i] = P(i,3);
			for(int j=0; j<3; ++j){
				v_3d[i] += P(i,j) * pt3d[j];
			}
		}

		double fx = pFrame->m_K(0,0) * pFrame->m_fscale;
		double fy = pFrame->m_K(1,1) * pFrame->m_fscale;
		double skew = pFrame->m_K(0,1) * pFrame->m_fscale;
		double cx = pFrame->m_K(0,2);
		double cy = pFrame->m_K(1,2);

		Wml::Vector3d imgPt;

		v_3d[0] /= v_3d[2];
		v_3d[1] /= v_3d[2];

		imgPt.X() = v_3d[0] * fx + v_3d[1] * skew + cx;
		imgPt.Y() = v_3d[1] * fy + cy;
		imgPt.Z() = v_3d[2];

		return imgPt;
	}

	Wml::Vector3f CalcCameraNorm(CameraFrame* pFrame)
	{
		Wml::Vector3f camera_normal;
		Wml::Matrix3d view_rotm=pFrame->GetViewAbsRotMG();
		camera_normal[0]=-view_rotm[0][2];
		camera_normal[1]=-view_rotm[1][2];
		camera_normal[2]=-view_rotm[2][2];
		camera_normal.Normalize();
		return camera_normal;
	}
}