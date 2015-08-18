#include "TextureMappingParser.h"
#include <tclap/CmdLine.h>
namespace WANG
{
	void TextureMappingParser(std::string& _loadfilepath, std::string& _actname, TMParameters& _tmparames, int argc, char* argv[])
	{
		try
		{
			TCLAP::CmdLine cmd("TextureMapping Authored by BoshengWang", ' ', "1.0");
			TCLAP::ValueArg<std::string> loadfilepath_arg("f", "path", "The file path", true, "ILLEGAL PATH", "string");
			TCLAP::ValueArg<std::string> actname_arg("a", "actname", "The act file name", true, "ILLEGAL ACT NAME", "string");
			TCLAP::ValueArg<int> TFStep_arg("t", "TFStep", "The step for texture mpping", true, 10, "int");
			TCLAP::ValueArg<float> DTol_arg("d", "DTol", "The depth threshold", true, 0.003, "float");
			TCLAP::ValueArg<int> CFNum_arg("c", "CFNum", "the number of lables for graphcut", true, 10, "int");
			TCLAP::ValueArg<int> MaxGCIternums_arg("g", "MaxGCIternums", "The max iterations for Graph cut", true, 100, "int");
			TCLAP::ValueArg<int> BounarySize_arg("b", "BoundarySize", "The boundary size", false, 5, "int");
			TCLAP::ValueArg<int> SmoothNrmR_arg("s", "SmoothNrmR", "The radius for normal smoothing", true, 0, "int"); 
			TCLAP::ValueArg<int> SmoothWeight_arg("w", "SmoothW", "The weight for smooth term", true, 30, "int");
			TCLAP::ValueArg<bool> WithUV_arg("u", "UV", "The input uv", false, false, "bool");
			cmd.add(loadfilepath_arg);
			cmd.add(actname_arg);
			cmd.add(TFStep_arg);
			cmd.add(DTol_arg);
			cmd.add(CFNum_arg);
			cmd.add(MaxGCIternums_arg);
			cmd.add(BounarySize_arg);
			cmd.add(SmoothNrmR_arg);
			cmd.add(SmoothWeight_arg);
			cmd.add(WithUV_arg);
			cmd.parse(argc, argv);

			//get results
			_loadfilepath=loadfilepath_arg.getValue()+"\\";
			_actname=actname_arg.getValue();
			_tmparames.TFStep=TFStep_arg.getValue();
			_tmparames.DTol=DTol_arg.getValue();
			_tmparames.CFNum=CFNum_arg.getValue();
			_tmparames.MaxGCIternums=MaxGCIternums_arg.getValue();
			_tmparames.BoudarySize=BounarySize_arg.getValue();
			_tmparames.SmoothNrmR=SmoothNrmR_arg.getValue();
			_tmparames.SmoothWeight=SmoothWeight_arg.getValue();
			_tmparames.WithUv=WithUV_arg.getValue();
		} catch(TCLAP::ArgException& e)
		{ std::cerr<<"ColorMappingParser error: "<<e.error()<<" for arg "<<e.argId()<<std::endl; }
	}
}