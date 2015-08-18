#pragma once
//改善TM: 纹理映射的相关函数
#include "ZImage.h"
//均化法向量
#define SMOOTHNRMR 10
#define OBJOFFSET 0
namespace WANG
{
	struct RangeNode
	{
		static float r;
		std::pair<float, int> data;
		RangeNode() {data.first=data.second=0;};
		RangeNode(float _datacost, int _index): data(_datacost, _index) {};
		RangeNode(std::pair<float, int> _data): data(_data){};
		bool operator < (const RangeNode& node) const {return data.first<node.data.first-r;}
	};
	void AdaptiveSample(std::vector<std::pair<float, int>>& origindata, std::vector<std::pair<float, int>>& newdata);
	//float RangeNode::ratio=0.9;
}
//void smoothNrmVec(const std::vector<Wml::Vector3f>& srcNrmVec, std::vector<Wml::Vector3f>& trgNrmVec, const std::vector<std::vector<int>> link_map, int _R);
void smoothNrmVec(std::vector<Wml::Vector3f> srcNrmVec, std::vector<Wml::Vector3f>& trgNrmVec, const std::vector<std::vector<int>> link_map, int _R);
inline Wml::Vector3f getColor(Wml::Vector2f p, ZByteImage& img)
{
	int x = int(p.X());
	if(x == img.GetWidth() - 1)
		x--;
	int y = int(p.Y());
	if(y == img.GetHeight() - 1)
		y--;
	float dx = p.X() - x;
	float dy = p.Y() - y;
	Wml::Vector3f color;
	for(int i=0; i<3; ++i)
		color[i]=((1.0 - dx) * (1.0 - dy) * img.at(x, y, i)
		+ dx * (1.0 - dy) * img.at(x + 1, y, i)
		+ (1.0 - dx) * dy * img.at(x, y + 1, i)
		+ dx * dy * img.at(x + 1, y + 1, i));
	return color;
	
}