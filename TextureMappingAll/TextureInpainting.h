#pragma once
#include <ObjInterface.h>
#include <wang_simplecompletion.h>
namespace WANG
{
	void RepairTexture(ObjInterface* _obj_data, int _r, int _patchsize);
	void SimpleParameterization(ObjInterface* _obj_data, std::vector<int>& _target_faces, std::vector<std::vector<Wml::Vector2f>>& _result);
	void AddNewTexture(ObjInterface* _obj_data, std::vector<std::vector<Wml::Vector2f>>& _parameterization , std::vector<int>& _target_faces, int _widht, int _height);
};