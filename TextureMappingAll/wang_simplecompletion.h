#pragma once
namespace WANG
{
	//判断是否一个点在三角形内部 这个函数假设p点在平面内部 以A点为顶点 u B-A v C-A 
	template<int size>
	bool point_in_triangle(const Wml::Vector<size, float>P, const Wml::Vector<size, float> A, const Wml::Vector<size, float> B,const Wml::Vector<size, float> C, float& _u, float& _v)
	{
		Wml::Vector<size, float> v1=C-A;
		Wml::Vector<size, float> v0=B-A;
		Wml::Vector<size, float> v2=P-A;
		float dot00=v0.Dot(v0);
		float dot01=v0.Dot(v1);
		float dot02=v0.Dot(v2);
		float dot11=v1.Dot(v1);
		float dot12=v1.Dot(v2);

		float inver_deno=1.0/(dot00*dot11-dot01*dot01);
		float u=(dot11*dot02-dot01*dot12)*inver_deno;
		if(u<0||u>1)
			return false;
		float v = (dot00 * dot12 - dot01 * dot02) * inver_deno;
		_u=u; _v=v;
		if(v<0||v>1)
			return false;
		return u+v<=1;
	}
	void copy_triangle(const cv::Mat3b& src_img, cv::Mat3f& tar_img, cv::Mat1b& mask, std::vector<Wml::Vector2f>& src_triangle, std::vector<Wml::Vector2f>& tar_triangle, 
						cv::Mat1i& label_msk=cv::Mat1i(), int label=-1);
	void simplecompletion(cv::Mat3b& img, const cv::Mat1b& mask, int patch_size);
	void fill_triangle(const cv::Mat1b& mask, std::vector<Wml::Vector2f>& tar_triangele);
	void average(cv::Mat3b&A, cv::Mat1b mask, cv::Mat1b &mask1, int patch_w);
}