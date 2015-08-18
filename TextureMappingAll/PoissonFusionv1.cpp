#include "stdafx.h"
#include "wang_poissonfusion.h"
#include "Wang_Helper.h"
#include <ctime>
//using namespace std;

int main(int argc, char* argv[])
{
	WANG::PoissonFusion p;
	std::string filepath;
	std::string obj="Model.obj";
	int HandledVarNum=40000000;
	int TextureSize=8192;
	if(argc==1)
	{
		std::cout<<"Please input parameters"<<std::endl;
		return 0;
	}
	if(argc>1)
		filepath=argv[1];
	if(argc>2)
		obj=argv[2];
	if(argc>3)
		WANG::STRto(argv[3], TextureSize);
	if(argc>4)
		WANG::STRto(argv[4], HandledVarNum);
	PRINT(filepath); PRINT(obj); 
	PRINT(TextureSize);
	printf("HandledVarNum: %e\n", (float)HandledVarNum);

	WANG::ObjInterface objdata;
	std::string FileFormat=obj.substr(obj.find_last_of("."));
	if(FileFormat==".obj")
		objdata.read_obj(filepath, obj);
	else if(FileFormat==".objyml")
		objdata.read_objyml(filepath, obj);
	else
	{
		printf("Can not parse this file with %s file format", FileFormat.c_str());
		exit(0);
	}

	//getchar();
	p.Setdata(&objdata);
	//p.build_link_map();
	//p.build_teximg_components();
	WANG::Timer time_count;
	time_count.Start();
	p.Run(TextureSize, HandledVarNum);
	//objdata.save_obj(objdata.loadfilepath, "new_Model.obj", false);
	//p.iterative_poisson_fusion();
	time_count.End();
	//p.repair_texture(20);
	return 0;

}