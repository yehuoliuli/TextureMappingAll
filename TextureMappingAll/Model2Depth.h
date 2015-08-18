#include <WangOpengl_helper.h>
#include <Wang_Helper.h>
#include <CameraFrame.h>
#include <CameraMotion.h>
#include <ObjInterface.h>
namespace WANG
{
	class Model2Depth
	{
	public:
		Model2Depth(): sfm_(NULL), objdata_ptr_(NULL), meshdata_ptr_(NULL), fbo_ptr_(NULL)
		{
#ifdef GL_CHECK_ERROR
			TestGLContext();
#endif
		};
		~Model2Depth()
		{ 
			SAFE_DELETE(meshdata_ptr_);
			SAFE_DELETE(fbo_ptr_); 
		};
		void SetData(ObjInterface* _objdata_ptr, CameraMotion* _sfm);

		//return znear and zfar
		Wml::Vector2d GetDepthData(int _frame_id, TinyImage& _out_timg, GLenum _type=GL_FLOAT);
		//Occlusion query
		bool OcclusionQuery(Wml::Vector3f _p0, Wml::Vector3f _p1, Wml::Vector3f _p2);
	private:
		CameraMotion* sfm_;
		ObjInterface* objdata_ptr_;
		GLMeshData* meshdata_ptr_;
		GLFrameBufferObject* fbo_ptr_;

		//tmp algorithm data
		Wml::Vector2f x_range_, y_range_, z_range_;
	};

	template<class T1, class T2>
	void GetNearFar(Wml::Vector2<T1> _x_range, Wml::Vector2<T1> _y_range, Wml::Vector2<T1> _z_range, Wml::Matrix4<T2>& _RT, double& _znear, double& _zfar)
	{
		Wml::Matrix3d rotm;
		for(int i=0; i<3; ++i)
			for(int j=0; j<3; ++j)
				rotm[i][j]=_RT[i][j];
		//std::cout<<mask<<" "<<pattern<<std::endl;
		double* cur_RT_row2=_RT[2];
		_znear=std::numeric_limits<double>::max();
		_zfar=-_znear;
		for(int box_coner_iter=0; box_coner_iter<8; ++box_coner_iter)
		{
			Wml::Vector4d cur_point(_x_range[((box_coner_iter&4)>>2)], _y_range[((box_coner_iter&2)>>1)], _z_range[(box_coner_iter&1)], 1);
			double cur_z=cur_point[0]*cur_RT_row2[0]+cur_point[1]*cur_RT_row2[1]+cur_point[2]*cur_RT_row2[2]+cur_point[3]*cur_RT_row2[3];
			_znear=std::min(_znear, -cur_z); _zfar=std::max(_zfar, -cur_z);
		}
		//PRINT(_znear);
		//PRINT(_zfar);
		//getchar();
		_znear=std::max(_znear, 0.1);
		if(_zfar<=_znear) { std::cout<<"_zfar<=_znear"<<std::endl; exit(0); }
	}
}