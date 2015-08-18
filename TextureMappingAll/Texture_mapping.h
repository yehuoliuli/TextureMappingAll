#pragma once
#include "GCoptimization.h"
#include "ZImage.h"
#include "CameraMotion.h"
#include "CameraFrame.h"
#include "ImproveTM.h"
#include "ObjInterface.h"
namespace WANG
{
	struct TMParameters 
	{
		int TFStep;//Texture Frame Step
		float DTol; // Disparity Threshold or tolerance
		int CFNum;// Candidate Frame Number
		int MaxGCIternums;// Maximal GC Iterations
		int BoudarySize;// Frame Boundary Siz;
		int SmoothNrmR;// Smooth Normals Radius
		bool WithUv;
		bool HighResolution;
		int ImageSize;
		int SmoothWeight;
		TMParameters(): TFStep(10), DTol(0.003), CFNum(10), MaxGCIternums(100), BoudarySize(2), SmoothNrmR(15), WithUv(false), HighResolution(false), ImageSize(8192){};
		/*	TMParameters(float _MaxDR, float _MinDR, int _TFStep,  float _DTol, 
		int _SmoothNrmR, int _CFNum, int _MaxGCIternums, int _BoudarySize, bool _WithUv, bool _HighResolution, int _ImageSize)
		{
		TFStep=_TFStep, DTol=_DTol;
		SmoothNrmR=_SmoothNrmR, CFNum=_CFNum, MaxGCIternums=_MaxGCIternums, BoudarySize=_BoudarySize;
		WithUv=_WithUv, HighResolution=_HighResolution;
		ImageSize=_ImageSize;
		}
		TMParameters(TMParameters& tmp)
		{
		TFStep=tmp.TFStep, DTol=tmp.DTol;
		SmoothNrmR=tmp.SmoothNrmR, CFNum=tmp.CFNum, MaxGCIternums=tmp.CFNum, BoudarySize=tmp.BoudarySize;
		WithUv=tmp.WithUv, HighResolution=tmp.HighResolution;
		ImageSize=tmp.ImageSize;
		}*/
		/*TMParameters& operator=(const TMParameters& tmp)
		{
			TFStep=tmp.TFStep, DTol=tmp.DTol;
			SmoothNrmR=tmp.SmoothNrmR, CFNum=tmp.CFNum, MaxGCIternums=tmp.CFNum, BoudarySize=tmp.BoudarySize;
			WithUv=tmp.WithUv, HighResolution=tmp.HighResolution;
			ImageSize=tmp.ImageSize;
			return *this;
		}*/
	};
	class CGCOSmoothExtData
	{
	public:
		//const std::vector<int>& m_CompPxlIdx;
		//const std::vector<std::vector<float> >& m_CandArea;
		const std::vector<std::map<int, double>>& CandFrmMap;
		std::vector<ZByteImage>& SrcImgs;
		const std::vector<Wml::Vector3f>& points3d;
		const std::vector<Wml::Vector3i >& faces;
		//const std::vector<std::vector<Wml::Vector<6, float> > >& m_CandTexCrd;
		GCoptimization::EnergyTermType m_SmoothWt;
		//TODO 添加最小边长度
		float meta;

	public:
		CGCOSmoothExtData::CGCOSmoothExtData(const std::vector<std::map<int, double> >& _CandFrmMap, std::vector<ZByteImage>& _SrcImgs, const std::vector<Wml::Vector3f>& _points3d, const std::vector<Wml::Vector3<int> >& _faces, GCoptimization::EnergyTermType SmoothWt, float _min_edge_len)
			//: m_CompPxlIdx(CompPxlIdx)
			: CandFrmMap(_CandFrmMap)
			, SrcImgs(_SrcImgs)
			, points3d(_points3d)
			, faces(_faces)
			, m_SmoothWt(SmoothWt)
			, meta(_min_edge_len)
		{
		}
		~CGCOSmoothExtData(void)
		{
		}

		static GCoptimization::EnergyTermType SmoothFunc(int p1, int p2, int l1, int l2, void* sExtraData);
		static std::vector<Wml::Vector2<int> > DrawLine(const Wml::Vector2<int>& p1, const Wml::Vector2<int>& p2);
	};
	//Texture Mapping
	class TM
	{
	public:
		class QFun
		{
		public:
			bool operator()(std::pair<int, double> a, std::pair<int, double> b) {return a.second>b.second;};
		};
		TM(){ objdata=NULL; };
		void clear()
		{
			p=TMParameters();
			mean_len=0;
			CandFrmVec.clear();
			iInVldTriAglNum=0;
			prefer_set.clear();
		}
		void SetData(TMParameters _p, ObjInterface* _objdata, std::vector<int>& _prefer_set=std::vector<int>())
		{
			clear();
			p=_p; objdata=_objdata;
			for(int i=0; i<_prefer_set.size(); ++i)
				prefer_set.insert(_prefer_set[i]);
			savefilepath=_objdata->loadfilepath+"\\ModelWithSequence\\";
		}
		std::string GetSaveFilePath() { return savefilepath; };
		TMParameters& GetTMParameters() { return p; };

		void run(std::vector<std::pair<int, int>>& _texid2frmid);
		void Parser(int argc, char* argv[]);
		void getMeanLen();
		void getCandFrmVec();
		void getCandFrm(std::vector<std::vector<std::pair<int, double>>>& CandFrm, std::vector<Wml::Vector3f>& NrmVec);
		void getCandFrmGL(std::vector<std::map<int, double>>& CandFrm, std::vector<Wml::Vector3f>& NrmVec);
		double GetDataCost(CameraFrame* frame, Wml::Vector3f& normal, Wml::Vector3f& pt1, Wml::Vector3f& pt2, Wml::Vector3f& pt3);
	private:
		TMParameters p;
		ObjInterface* objdata;
		
		//data for alorgrithm
		std::string savefilepath;
		std::set<int> prefer_set;
		float mean_len;
		std::vector<int> CandFrmVec;// the Frame Sequence after uniformly sampling
 		int iInVldTriAglNum;
	};

	/*void TextureMappingParser(int argc, char* argv[], std::string& loadfilepath, std::string& actsname, TM::TMParameters& tmparames);*/
}

namespace std
{
	ostream& operator<< (std::ostream& _out, const WANG::TMParameters& _tmparams);
}