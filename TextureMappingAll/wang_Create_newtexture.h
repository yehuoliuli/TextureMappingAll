#pragma once
#include "wang_simplecompletion.h"
#define TEXTURESIZE 1024
namespace WANG
{
	void CreateNewTexture(std::vector<std::vector<int>>& TexFrmTriAglMap, std::vector<Wml::Vector2f>& tex_points, std::vector<Wml::Vector3i>& tex_faces, std::vector<Wml::Vector2f>& uv_points, std::vector<Wml::Vector3i>& faces_uv_indexs, std::string loadfilepath, std::string savefilepath, std::vector<std::string>& tex_filenames, int TextureSize=8192);
	void Extendboundary(cv::Mat3b& src, cv::Mat1b& mask, int iter_nums=5);
}
