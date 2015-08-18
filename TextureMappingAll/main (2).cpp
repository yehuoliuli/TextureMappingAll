#include "stdafx.h"
#include <QtGui/QApplication>
#include <QGLWidget>
#include "Texture_mapping.h"
#include "TextureInpainting.h"
#include "Wang_Helper.h"

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	//initialize opengl context
	QGLWidget gl_context;
	gl_context.makeCurrent();
	glewInit();

	std::string loadfilepath, actsname;
	WANG::TM::TMParameters tmparams;
	WANG::TextureMappingParser(argc, argv, loadfilepath, actsname, tmparams);
	//prepare data
	WANG::ObjInterface objdata;
	std::vector<int> prefer_set;

	lpGlobalMotion->LoadPrjFile((loadfilepath+actsname).c_str());
	printf("load obj file!!\n");
	objdata.read_obj(loadfilepath+"\\Recon", "Model.obj");
	printf("Start parse pefer.txt, you should place it in Recon if you want using this feature!!\n");
	WANG::File2Vector_(prefer_set, loadfilepath+"\\Recon\\prefer.txt");

	//getchar();
	WANG::Timer time_count;
	time_count.Start();
	WANG::TM texmapping;
	texmapping.SetData(tmparams, &objdata, prefer_set);
	texmapping.run();
	time_count.End();

	time_count.Start();
	WANG::RepairTexture(&objdata, 20, 10);
	time_count.End();
	objdata.save_obj(objdata.loadfilepath, "new_Model.obj");
	std::cout<<"\a\a\a\a\a"<<std::endl;
}
