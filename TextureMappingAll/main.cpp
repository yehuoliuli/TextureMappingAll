#include "stdafx.h"
#include <QtGui/QApplication>
#include <QGLWidget>
#include "Texture_mapping.h"
#include "TextureInpainting.h"
#include "Wang_Helper.h"
#include "wang_poissonfusion.h"
#include "TexturePackage.h"
#include <WangCUDA_Helper.h>
#include "TextureMappingParser.h"
#include "tclap/CmdLine.h"
int main(int argc, char* argv[])
{
	//getchar();
	WANG::ShowCurGPUMemory();
	//exit(0);
	QApplication a(argc, argv);
	//initialize opengl context
	QGLWidget gl_context;
	gl_context.makeCurrent();
	glewInit();
	WANG::Timer global_tcount;
	global_tcount.Start();
	std::string loadfilepath, actsname;
	WANG::TMParameters tmparams;
	WANG::TextureMappingParser(loadfilepath, actsname, tmparams, argc, argv);
	cout<<tmparams;
	//getchar();
	//prepare data
	WANG::ObjInterface objdata;
	std::vector<int> prefer_set;

	lpGlobalMotion->LoadPrjFile((loadfilepath+actsname).c_str());
	//exit(0);
	/*PRINT(lpGlobalMotion->Fx());
	PRINT(lpGlobalMotion->GetCameraFrame(10)->m_objAbsTransformMG);
	getchar();*/
	printf("load obj file!!\n");
	objdata.read_obj(loadfilepath+"\\Rec", "Model.obj");
	printf("Start parse pefer.txt, you should place it in Rec if you want using this feature!!\n");
	WANG::File2Vector_(prefer_set, loadfilepath+"\\Rec\\prefer.txt");

	//getchar();
	WANG::Timer time_count;
	time_count.Start();
	WANG::TM texmapping;
	texmapping.SetData(tmparams, &objdata, prefer_set);
	std::vector<std::pair<int, int>> texid2frmid;
	texmapping.run(texid2frmid);
	time_count.End();
	WANG::ShowCurGPUMemory();
	//getchar();
	//PRINT(objdata.tex_rois.size());
	//exit(0);
	//time_count.Start();
	//WANG::RepairTexture(&objdata, 20, 10);
	//time_count.End();
	//WANG::ShowCurGPUMemory();                                                      m 

	//getchar();
	//PRINT(objdata.tex_rois[70]);
	//exit(0);
	objdata.save_obj(objdata.loadfilepath, "Model.obj", false);
	//std::cout<<"\a\a\a\a\a"<<std::endl;
	//return 0;
	//exit(0);
	/*char dopoisson=0;
	cout<<"Do PoissonFusion? (y/n)"<<endl;
	cin>>dopoisson;
	if(dopoisson=='n')
	return 0;*/

	//PRINT(objdata.tex_faces[129175]);
	//PRINT(objdata.tex_id[129175]);
	//getchar();
	WANG::PoissonFusion poissonfusion;
	poissonfusion.Setdata(&objdata);
	poissonfusion.Run(100000000, 500);
	objdata.save_obj(objdata.loadfilepath, "Model.obj", false);
	//PRINT(objdata.tex_faces[129175]);
	//getchar();
	cv::Mat3b final_texture;
	std::string new_texture_name=WANG::TexturePackage(objdata, final_texture);
	std::string savefilepath=objdata.loadfilepath+"PackingResult\\";
	WANG::CreateDir(savefilepath);
	WANG::Vector2File(texid2frmid, savefilepath+"texid2frmid.txt");
	WANG_DEBUG_OUT;
	cv::imwrite(savefilepath+new_texture_name, final_texture);
	WANG_DEBUG_OUT;
	objdata.save_obj(savefilepath, "Model.obj", false);
	WANG_DEBUG_OUT;
	objdata.writeMyObj(savefilepath);
	WANG_DEBUG_OUT;
	global_tcount.End();
	std::cout<<"\a\a\a\a\a"<<std::endl;
}
