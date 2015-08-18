#pragma once
//#include ".\CCSMatrix.h"
//#include "SparseMatrix_ListType.h"
#include "ZImage.h"
#include "ObjInterface.h"
//#define GPU_VERSION
//#include "wang_UV_atlas.h"
namespace WANG
{
	class PoissonFusion
	{
	public:
		struct TexImgM// Texture img loading manager 
		{
			enum {INVALID=-2, USED=-1};
			enum {NOTHANDLED=0, HANDLING=1, HANDLED=2};
			TexImgM(ObjInterface& _objdata, std::string _savefilepath):objdata(_objdata), savefilepath(_savefilepath)
			{
				if(!objdata.TextureSeparated) {printf("This TexImgManager is only for obj with separated textures\n!!"); exit(0);}
				isprocessed.resize(objdata.tex_rois.size(), NOTHANDLED);
				isboundary.resize(objdata.tex_rois.size(), false);
				isLoaded.resize(objdata.original_teximg_components.size(), false);
				tex_components.resize(objdata.tex_rois.size());
				TeximgVarNums.resize(objdata.original_teximg_components.size(), 0);
				for (int i=0; i<objdata.original_teximg_components.size(); ++i)
					for(int j=0; j<objdata.original_teximg_components[i].second.size(); ++j)
					{
						Wml::Vector4<int> cur_roi=objdata.tex_rois[objdata.original_teximg_components[i].second[j]];
						TeximgVarNums[i]+=(cur_roi[3]-cur_roi[1])*(cur_roi[2]-cur_roi[0]);
					}
				//we should clear the components_link_map which means we delete the invalid components in link_map
				std::vector<std::set<int>>& components_link_map=objdata.teximg_link_map;
				std::set<int>& invalid_texid=objdata.invalid_texid;
				for (std::set<int>::iterator iter=invalid_texid.begin(); iter!=invalid_texid.end(); ++iter)
				{
					int cur_invalid_texid=*iter;
					std::set<int>& cur_neghbors=components_link_map[cur_invalid_texid];
					for(std::set<int>::iterator iter1=cur_neghbors.begin(); iter1!=cur_neghbors.end(); ++iter1)
						components_link_map[*iter1].erase(cur_invalid_texid);
					cur_neghbors.clear();
				}
			};
			//check the unhandled components and the  
			void Check();
			void LoadTex(std::vector<int>& texids);
			//this fuction will clear the processed textures components and modify the teximg_components
			void ClearTex(std::vector<int>& texids);
			//this fuction will get sub_link_map by tailoring the original compoments_link_map which means it will modify the original compoments_link_map
			void GetSubTexLinkMap(const std::vector<int>& handled_tex_ids, std::vector<std::pair<int, std::vector<int>>>& sub_link_map);
			//Remapp texture according to the new uv coordiates
			void SaveTex(std::vector<int>& texids, cv::Mat3f& tar_img, cv::Mat1b& mask, bool remapping);
		    ObjInterface& objdata;
			std::string savefilepath;
			std::vector<int> isprocessed;//the tex pieces can be saved safely namely there are no tex seams existing in them
			std::vector<bool> isboundary;
			std::vector<bool> isLoaded;//the tex image have been loaded
			std::vector<cv::Mat3b> tex_components;
			std::vector<int> TeximgVarNums;// the VarNums in one tex_img

		};
		PoissonFusion(){};

		void Clear();
		void Setdata(ObjInterface* _objdata);
		std::string GetSaveFilePath() { return savefilepath; };
		void Run(int HandledVarNum, int max_iters, int NewTexSize=8192);
		//poisson
		void iterative_poisson_fusion(int HandledVarNum, int max_iters);
		void poisson_fusion(std::vector<std::pair<int, std::vector<int>>>& sub_link_map, int max_iters);// poisson fusion for sub graph
		//reserve memory for mtx
		void ReserveRoomForSparseMtx(std::vector<std::pair<int, std::vector<int>>>& sub_link_map);
		//build poisson
		void AddSpatialPriorsForOne(cv::Mat3b& tex_img, ZIntImage& index_img, int c);
		void AddBoundaryConstraintForOne(cv::Mat3b& tex_img, ZIntImage& index_img, int c);
		void AddCommonPointsPriorsForTwo(cv::Mat3b& tex_images1, cv::Mat3b& tex_images2, std::vector<Wml::Vector2<int>>& link_edges, ZIntImage& index_img1, ZIntImage& index_img2);
		void AddSpatialPriors(const std::vector<std::pair<int, std::vector<int>>>& sub_link_map, int c);
		void AddCommonPointsPriors(const std::vector<std::pair<int, std::vector<int>>>& sub_link_map);
	private:
		//outter data
		ObjInterface* objdata;

		//inner data
		bool remapping;
		std::string savefilepath;
		//new texutre
		cv::Mat3f new_teximg;
		cv::Mat1b mask;
		// TexImg Manager
		std::shared_ptr<TexImgM> TexImgMptr;
		//linear equation data
		int iVarCount;
		std::vector<ZIntImage> var_index_images;//标记图像中像素对应的变量
		//ZIntImage var_index_images[2];
		//CCSMatrix* A;
		Eigen::VectorXf X, B;
#ifdef GPU_VERSION
		Eigen::SparseMatrix<float, Eigen::RowMajor> mtx;
#else
		Eigen::SparseMatrix<float> mtx;
#endif
		//CSparseMatrix_ListType _mtx;
	};
	//CCSMatrix* EigenSparse2CCS(Eigen::SparseMatrix<float>& mtx);
}
