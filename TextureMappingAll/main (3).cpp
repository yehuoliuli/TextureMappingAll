// TexturePackage.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "TexturePackage.h"
#include "Wang_Helper.h"
int main(int argc, char* argv[])
{
	std::string loadfilepath;
	std::string objfile="new_Model.obj";
	std::string new_teximgname="PackingResult.jpg";
	if(argc>1)
	{
		loadfilepath=argv[1];
		loadfilepath+="\\";
	}
	else
	{
		cout<<"Error: input parameters is not enough!"<<endl;
		cout<<"Useage: TexturePackage.exe loadfilepath [Modelname='new_Model.obj']"<<endl;
	}
	if(argc>2)
		objfile=argv[2];
	std::string savefilepath=loadfilepath+"PackingResult\\";
	WANG::CreateDir(savefilepath);
	PRINT(loadfilepath);
	PRINT(objfile);
	WANG::ObjInterface objdata(loadfilepath, objfile);
	cv::Mat3b new_textureimg;
	WANG::TexturePackage(objdata, new_textureimg);
	cv::imwrite(savefilepath+new_teximgname, new_textureimg);
	objdata.save_obj(savefilepath);
	return 0;
}
