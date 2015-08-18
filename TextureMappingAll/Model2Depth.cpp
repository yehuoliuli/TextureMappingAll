#include "Model2Depth.h"
#include <WmlMathLib.h>
#include <limits>
#include <Utility.h>
#include <omp.h>
namespace WANG
{
	void Model2Depth::SetData(ObjInterface* _objdata_ptr, CameraMotion* _sfm)
	{
		sfm_=_sfm;
		objdata_ptr_=_objdata_ptr;
		int width=_sfm->m_iWidth;
		int height=_sfm->m_iHeight;
		//create fbo_ptr
		if(!fbo_ptr_||width!=fbo_ptr_->GetWidth()||height!=fbo_ptr_->GetHeight())
		{
			SAFE_DELETE(fbo_ptr_);
			fbo_ptr_=new GLFrameBufferObject(width, height);
			fbo_ptr_->Bind(GL_DRAW_FRAMEBUFFER);
			fbo_ptr_->AddAttachment(GL_DEPTH_ATTACHMENT, GL_DEPTH_COMPONENT32F);
		}

		//create meshdata
		SAFE_DELETE(meshdata_ptr_);
		meshdata_ptr_=new GLMeshData;
		std::vector<Wml::Vector3f>& points3d=objdata_ptr_->points3d;
		std::vector<Wml::Vector3i>& faces=objdata_ptr_->faces;
		meshdata_ptr_->VertexPointer(points3d.size(), &(points3d[0][0]), GL_FLOAT, GL_STATIC_DRAW, 3);
		meshdata_ptr_->ElementArrayG(faces.size(), &(faces[0][0]), 3, GL_STATIC_DRAW);
		objdata_ptr_->getBoundingBox(x_range_, y_range_, z_range_);
	}

	Wml::Vector2d Model2Depth::GetDepthData(int _frame_id, TinyImage& _out_timg, GLenum _type)
	{
		CameraFrame* cur_frame=sfm_->GetCameraFrame(_frame_id);
		//set opengl state
		glEnable(GL_DEPTH_TEST);
		glClearDepth(1.0);
		glViewport(0, 0, fbo_ptr_->GetWidth(), fbo_ptr_->GetHeight());
		
		//set modelview matrix and projection matrix
		Wml::Matrix4d cur_RT=cur_frame->GetObjAbsTransformGL();
		meshdata_ptr_->LoadTransposedModelViewMatrixd(cur_RT[0]);
		GLdouble znear, zfar;
		GLdouble left, right;
		GLdouble top, bottom;
		//robust method
		std::vector<Wml::Vector3i>& faces=objdata_ptr_->faces;
		std::vector<Wml::Vector3f>& points3d=objdata_ptr_->points3d;
		znear=std::numeric_limits<double>::max();
		zfar=0;
		std::vector<double> points_depth(points3d.size(), -1);
#pragma omp parallel for num_threads(omp_get_num_procs()-1)//, firstprivate(link_map, srcNrmVec_tmp, _R)
		for (int i=0; i<points3d.size(); ++i)
		{
			Wml::Vector3f curp=points3d[i];
			Wml::Vector3d imgp=ThreeDToImgD(cur_frame, curp);
			if(!(imgp.X() > 0 && imgp.X() < cur_frame->m_iWidth - 1 &&
				imgp.Y() > 0 && imgp.Y() < cur_frame->m_iHeight - 1 ))
				continue;

			if(imgp.Z()<=0) continue;
			points_depth[i]=imgp.Z();
		}

		for(int i=0; i<points_depth.size(); ++i)
		{
			if(points_depth[i]==-1)
				continue;
			znear=std::min(points_depth[i], znear);
			zfar=std::max(points_depth[i], zfar);
		}
		//GetNearFar(x_range_, y_range_, z_range_, cur_RT, znear, zfar);
		//PRINT(znear); PRINT(zfar);
		cur_frame->GetFrustumGL(znear, left, right, bottom, top);
		meshdata_ptr_->Frustum(left, right, bottom, top, znear, zfar);
		fbo_ptr_->Clear(GL_DEPTH_BUFFER_BIT);
		meshdata_ptr_->DrawMesh(*fbo_ptr_, GL_TRIANGLES);

		//read pixels
		fbo_ptr_->ReadPixels(GL_DEPTH_ATTACHMENT, GL_DEPTH_COMPONENT, _type, _out_timg);
		return Wml::Vector2d(znear, zfar);
	}

	bool Model2Depth::OcclusionQuery(Wml::Vector3f _p0, Wml::Vector3f _p1, Wml::Vector3f _p2)
	{
		fbo_ptr_->Bind(GL_DRAW_FRAMEBUFFER);
		GLuint query_id;
		glGenQueries(1, &query_id);
		glBeginQuery(GL_SAMPLES_PASSED, query_id);
		glBegin(GL_TRIANGLES);
		glVertex3fv(&_p0[0]);
		glVertex3fv(&_p1[0]);
		glVertex3fv(&_p2[0]);
		glEnd();
		glEndQuery(GL_SAMPLES_PASSED);
		CheckError_();
		int queries_count=100;
		GLint samples;
		GLint query_is_ready=false;
		while(!query_is_ready&&query_is_ready--)
			glGetQueryObjectiv(query_id, GL_QUERY_RESULT_AVAILABLE, &query_is_ready);
		if(query_is_ready)
			glGetQueryObjectiv(query_id, GL_QUERY_RESULT, &samples);
		else
		{
			cout<<"Result not ready, return true!!"<<endl;
			return true;
		}
		CheckError_();
		//cout<<"Samples: "<<samples<<endl;
		if(samples==3)
			return true;
		return false;
	}
}