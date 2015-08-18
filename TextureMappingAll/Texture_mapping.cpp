// Texture_mapping.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "Model2Depth.h"
#include <Wang_Helper.h>
#include <cstring>
#include <algorithm>
#include <queue>
#include "Utility.h"
#include <io.h>
#include <direct.h>
#include "Texture_mapping.h"
#include "wang_Create_newtexture.h"
#include <fstream>
#include <limits>
#include "BlurMetric.h"
extern CameraMotion* lpGlobalMotion; 
namespace WANG
{
	std::vector<Wml::Vector2<int> > CGCOSmoothExtData::DrawLine(const Wml::Vector2<int>& p1, const Wml::Vector2<int>& p2)
	{
		std::vector<Wml::Vector2<int>> line;
		//PointList.clear();
		int absx=std::abs(p1.X()-p2.X());
		int absy=std::abs(p2.Y()-p1.Y());
		int x_temp=p1.X()<p2.X()?1:-1;
		int y_temp=p1.Y()<p2.Y()?1:-1;
		if (absy>absx)
		{
			for (int itery=0;itery<=absy;itery++)
			{
				int y=p1.Y()+y_temp*itery;
				int x=(double)(p2.X()-p1.X())/(p2.Y()-p1.Y())*y_temp*itery+p1.X();
				line.push_back(Wml::Vector2<int>(x, y));
			}
		}
		else
		{
			for (int iterx=0;iterx<=absx;iterx++)
			{
				int x=p1.X()+x_temp*iterx;
				int y=(double)(p2.Y()-p1.Y())/(p2.X()-p1.X())*x_temp*iterx+p1.Y();
				line.push_back(Wml::Vector2<int> (x, y));
			}
		}
		return line;
	}
	GCoptimization::EnergyTermType CGCOSmoothExtData::SmoothFunc(int t1, int t2, int l1, int l2, void* sExtraData)
	{
		
		CGCOSmoothExtData* pExtData = static_cast<CGCOSmoothExtData*>(sExtraData);
		//const vector<int>& CompPxlIdx = pExtData->m_CompPxlIdx;
		const std::vector<std::map<int, double>>& CandFrmMap = pExtData->CandFrmMap;
		std::vector<ZByteImage>& SrcImgs = pExtData->SrcImgs;
		const std::vector<Wml::Vector3f>& points3d = pExtData->points3d;
		const std::vector<Wml::Vector3<int> >& faces = pExtData->faces;
		//这样设置是合理的 无效面片的datacost和smoothcost都为0 相当于不贡献于能量函数
		if(CandFrmMap[t1].empty()||CandFrmMap[t2].empty()) return 0;
		bool t1_f_l1=CandFrmMap[t1].find(l1)==CandFrmMap[t1].end(); bool t1_f_l2=CandFrmMap[t1].find(l2)==CandFrmMap[t1].end();
		bool t2_f_l1=CandFrmMap[t2].find(l1)==CandFrmMap[t2].end(); bool t2_f_l2=CandFrmMap[t2].find(l2)==CandFrmMap[t2].end();
		if (t1_f_l1&&t1_f_l2&&t2_f_l1&&t2_f_l2) return GCO_MAX_ENERGYTERM;
		if(l1== l2)	
			return 0;
	/*	else
			return 100*abs(CandFrmMap[t1][l1].first-CandFrmMap[t2][l2].first);*/
		else
		{
			//查找相邻边
			const Wml::Vector3<int>& T1 = faces[t1];
			const Wml::Vector3<int>& T2 = faces[t2];

			Wml::Vector2<int> E(-1, -1);
			for(int i = 0; i < 3; i++)
			{
				Wml::Vector2<int> E1(T1[i], T1[(i + 1) % 3]);
				for(int k = 0; k < 3; k++) 
				{
					Wml::Vector2<int> E2(T2[k], T2[(k + 1) % 3]);
					if(E1[0] == E2[0] && E1[1] == E2[1] || E1[0] == E2[1] && E1[1] == E2[0])
					{
						E = E1;
						break;
					}
				}
				if(E != Wml::Vector2<int>(-1, -1))
					break;
			}
			float edge_len=(points3d[E[0]]-points3d[E[1]]).Length();
			//TODO: 改变采样系数
			int samplenums=MIN(int(edge_len/pExtData->meta), 1);
		//	printf("%d\n", samplenums);
			/*FILE* edge_file=fopen("edge_file.txt", "w");
			fprintf(edge_file, "%f  ", edge_len);
			fclose(edge_file);*/
			if(E != Wml::Vector2<int>(-1, -1))
			{
				Wml::Vector3f p;
				CameraFrame* pFrm1 = lpGlobalMotion->GetCameraFrame(l1);

				p=ThreeDToImg(pFrm1, points3d[E[0]]);
				Wml::Vector2<float> p1(p.X() , p.Y());

				p=ThreeDToImg(pFrm1, points3d[E[1]]);
				Wml::Vector2<float> p2(p.X(), p.Y());
				ZByteImage& ImgMap1 = SrcImgs[l1];
				int img1_height=ImgMap1.GetHeight();
				int img1_width=ImgMap1.GetWidth();
				if(!(p1.X()>1&&p1.X()<img1_width-2)) { /*cout<<"cao"<<endl;*/ return GCO_MAX_ENERGYTERM; }
				if(!(p1.Y()>1&&p1.Y()<img1_height-2)) { /*cout<<"cao"<<endl;*/ return GCO_MAX_ENERGYTERM; }
				if(!(p2.X()>1&&p2.X()<img1_width-2)) { /*cout<<"cao"<<endl;*/ return GCO_MAX_ENERGYTERM; }
				if(!(p2.Y()>1&&p2.Y()<img1_height-2)) { /*cout<<"cao"<<endl;*/ return GCO_MAX_ENERGYTERM; }
				CameraFrame* pFrm2 = lpGlobalMotion->GetCameraFrame(l2);
				p=ThreeDToImg(pFrm2, points3d[E[0]]);
				Wml::Vector2<float> p3(p.X(), p.Y());

				p=ThreeDToImg(pFrm2, points3d[E[1]]);
				Wml::Vector2<float> p4(p.X(), p.Y());
				ZByteImage& ImgMap2=SrcImgs[l2];
				int img2_height=ImgMap2.GetHeight();
				int img2_width=ImgMap2.GetWidth();
				if(!(p3.X()>1&&p3.X()<img2_width-2)) { /*cout<<"cao"<<endl;*/ return GCO_MAX_ENERGYTERM; }
				if(!(p3.Y()>1&&p3.Y()<img2_height-2)) { /*cout<<"cao"<<endl;*/ return GCO_MAX_ENERGYTERM; }
				if(!(p4.X()>1&&p4.X()<img2_width-2)) { /*cout<<"cao"<<endl;*/ return GCO_MAX_ENERGYTERM; }
				if(!(p4.Y()>1&&p4.Y()<img2_height-2)) { /*cout<<"cao"<<endl;*/ return GCO_MAX_ENERGYTERM; }
				float smoothcost=0;
				if(samplenums!=0)
				{
					for (int i=0; i<=samplenums; ++i)
					{
						float cur_ratio=(float)i/samplenums;
						Wml::Vector2f curp_Frm1=p1*cur_ratio+p2*(1-cur_ratio);
						Wml::Vector2f curp_Frm2=p3*cur_ratio+p4*(1-cur_ratio);
						//WANG_DEBUG_OUT;
						Wml::Vector3f color_Frm1=getColor(curp_Frm1, ImgMap1);
						//WANG_DEBUG_OUT;
						Wml::Vector3f color_Frm2=getColor(curp_Frm2, ImgMap2);
						//WANG_DEBUG_OUT;
						smoothcost=smoothcost+(color_Frm1-color_Frm2).Length();
					}
				}
				//return smoothcost/samplenums*20*pExtData->m_SmoothWt;
				return smoothcost*pExtData->m_SmoothWt;
				
			}
			else
				return GCO_MAX_ENERGYTERM;
		}
	}
	void TM::run(std::vector<std::pair<int, int>>& _texid2frmid)
	{
		WANG::Timer time_count;
		if(!objdata) {printf("Parsing is not done!!\n"); return;}
		//prepare obj data
		//data that program needs
		std::vector<Wml::Vector3i>& faces=objdata->faces;
		std::vector<Wml::Vector3f>& points3d=objdata->points3d;
		std::vector<std::vector<int>>& link_map=objdata->link_map;
		objdata->build_link_map();
		//data that programe do not need
		std::vector<int>& tex_id=objdata->tex_id;
		std::vector<Wml::Vector2f>& tex_points=objdata->tex_points;
		std::vector<Wml::Vector3i>& tex_faces=objdata->tex_faces;
		std::vector<std::string>& tex_filenames=objdata->tex_filenames;
		std::vector<Wml::Vector4<int>>& tex_rois=objdata->tex_rois;

		tex_rois.clear();
		std::set<int>& invalid_texid = objdata->invalid_texid;
		//withuv is true
		std::vector<Wml::Vector2f> uv_points;
		std::vector<Wml::Vector3i> uv_faces;
		if(p.WithUv)
		{
			uv_points=tex_points;
			uv_faces=tex_faces;
			if(uv_points.empty()||uv_faces.empty())
			{
				printf("There are no valid tex_points data exsting in objfile!!\n");
				p.WithUv=false;
			}
		}
		tex_id=std::vector<int>(faces.size(), 0);
		tex_faces=std::vector<Wml::Vector3i>(faces.size(), Wml::Vector3i(-1, -1, -1));
		tex_points.clear();
		tex_filenames=std::vector<std::string>(1);
		tex_rois.push_back(Wml::Vector4<int>(0, 0, 0, 0));
		invalid_texid.clear();
		invalid_texid.insert(0);

		int PointsNum = points3d.size();
		int FacesNum = faces.size();
		printf("Number of Vertices: %d, Number of Triangles: %d\n", PointsNum, FacesNum);
		//getchar();
		//总共的帧的数量
		int iFrmNum = lpGlobalMotion->GetFrameCount();
		
		//get CandFrmVec
		for(int i=0; i<iFrmNum; i+=p.TFStep)
		{
			if(prefer_set.empty())
				CandFrmVec.push_back(i);
			else if(prefer_set.find(i)!=prefer_set.end())
				CandFrmVec.push_back(i);
		}
		//time_count.Start();
		//getCandFrmVec();
		//time_count.End();
		//exit(0);
		iInVldTriAglNum = 0;		
		
		//法向量
		std::vector<Wml::Vector3f> NrmVec;
		objdata->getNormal(NrmVec);
		
		//TODO 三角面片邻域数据 TriAglNbr 同时统计最小边的长度 最小边 采样5个点
		getMeanLen();
		
		//smooth 法向量
		std::vector<Wml::Vector3f> NrmVectemp(NrmVec.size());
		printf("smooth Nrm start!\n");
		time_count.Start();
		smoothNrmVec(NrmVec, NrmVectemp, link_map, p.SmoothNrmR);
		NrmVectemp=NrmVec;
		time_count.End();
		printf("smooth Nrm end!\n");
		//exit(0);
		NrmVec.swap(NrmVectemp);
		NrmVectemp.swap(std::vector<Wml::Vector3f>());
		 
		//计算candidat frame
		std::vector<std::map<int, double>> CandFrmmap;
		time_count.Start();
		//getCandFrm(CandFrmmap, NrmVec);
		getCandFrmGL(CandFrmmap, NrmVec);
		time_count.End();
		//exit(0);
		printf("Number of Invalid Triangles: %d\n", iInVldTriAglNum);
		//exit(0);
		int iGCOLblNum = 0;
		for(int t = 0; t < FacesNum; t++)
			iGCOLblNum = MAX(iGCOLblNum, CandFrmmap[t].size());
		printf("iGCOLblNum: %d\n", iGCOLblNum);
		std::vector<ZByteImage> SrcImgMap(iFrmNum);

		cout<<"load color image"<<endl;
		//getchar();
		for(int ff = 0; ff < CandFrmVec.size(); ff++)
		{
			int f = CandFrmVec[ff];
			CameraFrame* pCFrm = lpGlobalMotion->GetCameraFrame(f);
			cv::Mat3b Srcimg=cv::imread(pCFrm->m_sImgName);
			SrcImgMap[f].CreateAndInit(pCFrm->m_iWidth, pCFrm->m_iHeight, 3);

			for(int y = 0; y < pCFrm->m_iHeight; y++)
			{
				for(int x = 0; x < pCFrm->m_iWidth; x++)
				{
					cv::Vec3b c=Srcimg(y, x);
					SrcImgMap[f].at(x, y, 0) = c[0];
					SrcImgMap[f].at(x, y, 1) = c[1];
					SrcImgMap[f].at(x, y, 2) = c[2];
				}
			}
		}
		cout<<"laod end"<<endl;
		//getchar();
		//getchar();
		//每个面片对应的视频序列的帧id
		std::vector<int> Frame_ids(FacesNum, -1);
		if(FacesNum > 1 && iGCOLblNum > 1)
		{
			//GCO result
			std::vector<int> Labels(FacesNum, -1);
			try
			{
				//////////////////////////////////////////////////////////////////////////
				//Graph Cuts
				GCoptimizationGeneralGraph GCO(FacesNum, iFrmNum);
				std::vector<GCoptimization::EnergyTermType> DataCost(FacesNum * iFrmNum);
				for(int t = 0; t < FacesNum; t++)
				{
					if(CandFrmmap[t].empty())
					{
						for(int k = 0; k < iFrmNum; k++)
							DataCost[t * iFrmNum + k] = 0;
					} 
					else
					{
						std::map<int, double>& cur_cand_frm=CandFrmmap[t];
						for(int k = 0; k < iFrmNum; k++)
						{
							//DataCost[t * iGCOLblNum + k] = (k < CandFrmmap[t].size()) ? int(CandFrmmap[t][k].second * 255.0f + 0.5f) : GCO_MAX_ENERGYTERM;
							std::map<int, double>::iterator it=cur_cand_frm.find(k);
							if (it!=cur_cand_frm.end())
								DataCost[t*iFrmNum+k]=it->second*255.0;
							else
								DataCost[t*iFrmNum+k]=GCO_MAX_ENERGYTERM;
						}
					}
				}
				GCO.setDataCost(&DataCost[0]);
				//TODO setSmoothCost
				CGCOSmoothExtData sExtraData(CandFrmmap, SrcImgMap, points3d, faces, p.SmoothWeight, mean_len/40);
				GCO.setSmoothCost(&CGCOSmoothExtData::SmoothFunc, &sExtraData);
				//Set Up Neighbors
				printf("Set Up Neighbors!\n");
				for (int i=0; i<link_map.size(); ++i)
				{
					for(int j=0; j<link_map[i].size(); ++j)
						if(link_map[i][j]>i)
							GCO.setNeighbors(i, link_map[i][j]);
				}
				WANG_DEBUG_OUT;
				//getchar();
				GCO.swap(p.MaxGCIternums);
				WANG_DEBUG_OUT;
				for(int i = 0; i < FacesNum; i++)
					Labels[i] = GCO.whatLabel(i);
			}
			catch(GCException e)
			{
				e.Report();
			}
			for(int t = 0; t < FacesNum; t++)
			{
				int iCandIdx = Labels[t];
				if (!CandFrmmap[t].empty())
					Frame_ids[t]=iCandIdx;
				/*if(iCandIdx >= 0 && iCandIdx < CandFrmmap[t].size())
					Frame_ids[t] = CandFrmmap[t][iCandIdx].first;
				else if(!CandFrmmap[t].empty())
					Frame_ids[t] = CandFrmmap[t].back().first;*/
			}

		}
		else
		{
			/*for(int t = 0; t < FacesNum; t++)
			{
				if(!CandFrmmap[t].empty())
					Frame_ids[t] = CandFrmmap[t].back().first;
			}*/

		}
		//release SrcImgMap
		for (int i=0; i<SrcImgMap.size(); ++i)
			SrcImgMap[i].Clear();
		RELEASE_VECTOR(SrcImgMap);
		cout<<"Release srcimgmap end!!"<<endl;
		//getchar();
		//get data connected with texture img
		//tex_id tex_filename invalid_set tex_points tex_faces
		std::map<int, int> CandFrmid2texid;
		int tex_idnums=1;
		for(int t = 0; t < FacesNum; t++)
		{
			int Frmid=Frame_ids[t];
			if(Frmid!=-1)
			{
				int cur_tex_points_num=tex_points.size();
				std::pair<std::map<int, int>::iterator, bool> insert_r=CandFrmid2texid.insert(std::make_pair(Frmid, tex_idnums));
				tex_id[t]=insert_r.first->second;
				tex_faces[t]=Wml::Vector3i(cur_tex_points_num, cur_tex_points_num+1, cur_tex_points_num+2);
				CameraFrame* cur_cf=lpGlobalMotion->GetCameraFrame(Frmid);
				//get tex points
				for(int i=0; i<3; ++i)
				{
					Wml::Vector3f p=ThreeDToImg(cur_cf, points3d[faces[t][i]]);
					tex_points.push_back(Wml::Vector2f(p.X()/cur_cf->m_iWidth, 1-p.Y()/cur_cf->m_iHeight));
				}
				if(insert_r.second)
				{
					++tex_idnums;
					//Absolute path filename
					CameraFrame* cur_frame=lpGlobalMotion->GetCameraFrame(Frmid);
					std::string AimgName=cur_frame->m_sImgName;
					tex_filenames.push_back(AimgName.substr(AimgName.find_last_of('\\')+1));
					tex_rois.push_back(Wml::Vector4<int>(0, 0, cur_frame->GetImgWidth(), cur_frame->GetImgHeight()));
				}
			}
			else
			{
				tex_id[t]=0;
				tex_faces[t]=Wml::Vector3i(0, 0, 0);
			}
		}
		for(std::map<int, int>::iterator it=CandFrmid2texid.begin(); it!=CandFrmid2texid.end(); ++it)
			_texid2frmid.push_back(std::make_pair(it->second, it->first));
		std::sort(_texid2frmid.begin(), _texid2frmid.end());
		//save objwithSequence
		if(_access(savefilepath.c_str(), 0)==-1)
		{
			if(_mkdir(savefilepath.c_str())==-1)
				printf("create savefilepath: %s unsucessfully!!\n", savefilepath.c_str());
			else
				printf("create savefilepath: %s sucessfully!!\n", savefilepath.c_str());
		}
		else
			printf("The savefilepath: %s exists!!\n", savefilepath.c_str());
		objdata->loadfilepath=lpGlobalMotion->ProjectDir();
		if(p.HighResolution)
		{
			std::string loadoriginpath=lpGlobalMotion->ProjectDir()+"Rectify\\";
			if(_access(lpGlobalMotion->ProjectDir().c_str(), 0)==-1)
			{
				printf("There are no original images  existing in folder: %s, we will use the folder of video sequence to save model\n", loadoriginpath.c_str());
				loadoriginpath=lpGlobalMotion->ProjectDir();
			}
			else
			{
				printf("The original imges names should use the prefix rect\nRenaming Starts!!\n");
				std::string prefix_name="rect";
				for (int i=0; i<objdata->tex_filenames.size(); ++i)
				{
					std::string& cur_name=objdata->tex_filenames[i];
					if(cur_name.empty()) continue;
					std::string newname=prefix_name+cur_name.substr(cur_name.find_first_of("0123456789"));
					cur_name=newname;
				}
			}
			objdata->loadfilepath=loadoriginpath;
		}
		objdata->save_tex_imgs(savefilepath);
		objdata->loadfilepath=savefilepath;
		//if withuv=true, Merge texture_info first before save_objyaml
		if(p.WithUv)
		{
			objdata->tex_id_v.resize(2);
			objdata->tex_points_v.resize(2);
			objdata->tex_faces_v.resize(2);
			objdata->tex_filenames_v.resize(2);

			objdata->tex_points_v[1]=uv_points;
			objdata->tex_faces_v[1]=uv_faces;
			//objdata->tex_id_v[1].resize(objdata->faces.size(), 0);
			//objdata->tex_filenames_v[1].resize(objdata->faces.size());
			//objdata->save_objyml(savefilepath, std::string("Model.objyml"));
			////when withuv is true, we will create a new directory to save 
			//std::string saveuvpath=lpGlobalMotion->ProjectDir()+"Rec\\ModelWithOneTexture\\";
			//if(_access(saveuvpath.c_str(), 0)==-1)
			//{
			//	if(_mkdir(saveuvpath.c_str())==-1)
			//		printf("create saveuvpath: %s unsucessfully!!\n", saveuvpath.c_str());
			//	else
			//		printf("create saveuvpath: %s sucessfully!!\n", saveuvpath.c_str());
			//}
			//else
			//	printf("The saveuvpath: %s exists!!\n", saveuvpath.c_str());
			//CreateNewTexture(objdata->teximg_faces, objdata->tex_points, objdata->tex_faces, uv_points, uv_faces, objdata->loadfilepath, saveuvpath, objdata->tex_filenames, p.ImageSize);
			//objdata->tex_points.swap(uv_points);
			//objdata->tex_faces.swap(uv_faces);
			//for(int i=0; i<tex_id.size(); ++i) tex_id[i]=1;
			//tex_filenames.clear(); tex_filenames.push_back(""); tex_filenames.push_back("_texture1.jpg");
			//objdata->save_obj(saveuvpath, "Model.obj", false);
		}
	}

	void TM::getMeanLen()
	{
		const std::vector<Wml::Vector3i>& faces=objdata->faces;
		const std::vector<Wml::Vector3f>& points3d=objdata->points3d;
		float min_edge_len=FLT_MAX;
		float max_edge_len=FLT_MIN;
		float edge_len_sum=0;
		int edge_sum=0;
		FILE* cao=fopen("cao.txt", "w");
		//CAO;
		for(int t = 0; t < faces.size(); t++)
		{
			for(int i = 0; i < 3; i++)
			{
				int v1 = faces[t][i];
				int v2 = faces[t][(i + 1) % 3];
				edge_sum++;
				if((points3d[v1]-points3d[v2]).Length()==0)
					fprintf(cao, "v1: %d %f %f %f v2: %d %f %f %f\n", v1, points3d[v1].X(), points3d[v1].Y(), points3d[v1].Z(), 
					v2, points3d[v2].X(), points3d[v2].Y(), points3d[v2].Z());
				else
				{
					min_edge_len=MIN(min_edge_len, (points3d[v1]-points3d[v2]).Length());
					max_edge_len=MAX(max_edge_len, (points3d[v1]-points3d[v2]).Length());
					edge_len_sum=edge_len_sum+(points3d[v1]-points3d[v2]).Length();
				}
			}
		}
		mean_len=edge_len_sum/edge_sum;
		fprintf(cao, "min_edge_len= %f\n", min_edge_len);
		fprintf(cao, "max_edge_len= %f\n", max_edge_len);
		fprintf(cao, "mean_edge_len= %f\n", mean_len);
		fclose(cao);
		printf("end cao!!\n");
	}

	void TM::getCandFrmVec()
	{
		FUNCTION_START;
		int interval_start=3;
		int interval_end=p.TFStep;
		//intialization
		int last_selected_frameid=-1;
		int frames_ind=lpGlobalMotion->GetFrameCount();
		CandFrmVec.clear();
		while(frames_ind>0)
		{
			double min_blur_score=std::numeric_limits<double>::max();
			int min_frameid=-1;
			int i;
			if(last_selected_frameid!=-1)
			{
				i=last_selected_frameid+interval_start;
				frames_ind=frames_ind-(interval_start-1);
			}
			else
				i=last_selected_frameid+1;
			for(; i<=last_selected_frameid+interval_end&&frames_ind>0; ++i)
			{
				//PRINT(i); PRINT(frames_ind);
				CameraFrame* cur_frame=lpGlobalMotion->GetCameraFrame(i);
				cur_frame->LoadImage();
				cv::Mat1b cur_gray_img;
				cv::cvtColor(cur_frame->img, cur_gray_img, CV_BGR2GRAY);
				cur_frame->ReleaseImage();
				double cur_blurscore=WANG::BlurMetirc(cur_gray_img);
				if(cur_blurscore<min_blur_score)
				{
					min_frameid=i;
					min_blur_score=cur_blurscore;
				}
				--frames_ind;
			}
			//PRINT(frames_ind); PRINT(min_frameid);
			if(frames_ind>0)
				CandFrmVec.push_back(min_frameid);
			//PRINT(min_frameid);
			//getchar();
			last_selected_frameid=min_frameid;
		}
		FUNCTION_END;
	}
	
	//void TM::getCandFrm(std::vector<std::vector<std::pair<int, double>>>& CandFrm, std::vector<Wml::Vector3f>& NrmVec)
	//{
	//	FUNCTION_START;
	//	std::vector<Wml::Vector3i>& faces=objdata->faces;
	//	std::vector<Wml::Vector3f>& points3d=objdata->points3d;
	//	std::vector<std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, QFun>> CandFrm_heap(faces.size());
	//	std::vector<Wml::Vector2d> min_max_vec(faces.size(), Wml::Vector2d(1e10, -1e10)); 

	//	//getchar();
	//	for (int f=0; f<CandFrmVec.size(); ++f)
	//	{
	//		int cur_frame_id=CandFrmVec[f];
	//		CameraFrame* cur_frame=lpGlobalMotion->GetCameraFrame(cur_frame_id);
	//		cur_frame->Swap_In(SWAP_DEPTH);
	//		ZFloatImage& cur_dsp_img=cur_frame->m_DspImg;
	//		for (int i=0; i<faces.size(); ++i)
	//		{
	//			Wml::Vector3i cur_face=faces[i];
	//			Wml::Vector2d& cur_min_max=min_max_vec[i];
	//			//occlusion test
	//			bool is_visible=true;
	//			for(int j=0; j<3; ++j)
	//			{
	//				Wml::Vector3f curp=points3d[faces[i][j]];
	//				Wml::Vector3f imgp=ThreeDToImg(cur_frame, curp);
	//				if(!(imgp.X() > p.BoudarySize && imgp.X() < cur_frame->m_iWidth - 1 - p.BoudarySize &&
	//					imgp.Y() > p.BoudarySize && imgp.Y() < cur_frame->m_iHeight - 1 - p.BoudarySize))
	//				{
	//					is_visible=false;
	//					break;
	//				}
	//				if(!((fabs(DspVl(Wml::Vector2f(imgp.X(), imgp.Y()), cur_dsp_img) - 1.0 / imgp.Z()) / (p.MaxDR - p.MinDR)) < p.DTol))
	//				{
	//					is_visible = false;
	//					break;
	//				}
	//			}
	//			if(!is_visible) continue;
	//			
	//			//caculate datacost
	//			double cur_score=GetDataCost(cur_frame, NrmVec[i], points3d[cur_face[0]], points3d[cur_face[1]], points3d[cur_face[2]]);
	//			auto& cur_CandFrm_heap=CandFrm_heap[i];
	//			cur_min_max[0]=MIN(cur_min_max[0], cur_score);
	//			cur_min_max[1]=MAX(cur_min_max[1], cur_score);

	//			if(cur_CandFrm_heap.size()<p.CFNum)
	//				cur_CandFrm_heap.push(std::make_pair(cur_frame_id, cur_score));
	//			else
	//			{
	//				if(cur_CandFrm_heap.top().second<cur_score)
	//				{
	//					cur_CandFrm_heap.pop();
	//					cur_CandFrm_heap.push(std::make_pair(cur_frame_id, cur_score));
	//				}
	//			}
	//		}
	//		cur_frame->m_DspImg.Clear();
	//	}
	//	CandFrm.clear(); CandFrm.resize(faces.size());
	//	for(int i=0; i<faces.size(); ++i)
	//	{
	//		if(CandFrm_heap[i].empty()) ++iInVldTriAglNum;
	//		CandFrm[i].resize(CandFrm_heap[i].size());
	//	}
	//	for(int i=0; i<CandFrm_heap.size(); ++i)
	//	{
	//		std::vector<std::pair<int, double>>& cur_CandFrm=CandFrm[i];
	//		int cur_CandFrm_size=cur_CandFrm.size()-1;
	//		while(!CandFrm_heap[i].empty())
	//		{
	//			std::pair<int, double> cur_top=CandFrm_heap[i].top();
	//			CandFrm_heap[i].pop();
	//			Wml::Vector2d cur_min_max=min_max_vec[i];
	//			cur_top.second=(cur_min_max[1]-cur_top.second)/(cur_min_max[1]-cur_min_max[0]);
	//			cur_CandFrm[cur_CandFrm_size--]=cur_top;
	//		}
	//	}
	//	FUNCTION_END;
	//}

	void TM::getCandFrmGL(std::vector<std::map<int, double>>& CandFrm, std::vector<Wml::Vector3f>& NrmVec)
	{
		FUNCTION_START;
		std::vector<Wml::Vector3i>& faces=objdata->faces;
		std::vector<Wml::Vector3f>& points3d=objdata->points3d;
		std::vector<std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, QFun>> CandFrm_heap(faces.size());
		std::vector<Wml::Vector2d> min_max_vec(faces.size(), Wml::Vector2d(1e10, -1e10)); 

		//prepare GPU data
		Model2Depth model2depth;
		model2depth.SetData(objdata, lpGlobalMotion);

		//getchar();
		//for save depth
		int depth_num=0;
		std::string depth_path=lpGlobalMotion->ProjectDir()+std::string("model2depth\\");
		WANG::CreateDir(depth_path);
		for (int f=0; f<CandFrmVec.size(); ++f)
		{
			int cur_frame_id=CandFrmVec[f];
			CameraFrame* cur_frame=lpGlobalMotion->GetCameraFrame(cur_frame_id);

			//Get depth
			GLDepthData<float> gl_depthimg;
			Wml::Vector2d z_range=model2depth.GetDepthData(cur_frame_id, gl_depthimg, GL_FLOAT);
			gl_depthimg.SetZNearFar(z_range[0], z_range[1]);
			//PRINT(z_range);
			cv::Mat1b out_depth(gl_depthimg.height_, gl_depthimg.width_);
			for(int r=0; r<out_depth.rows; ++r)
			{
				uchar* out_depth_ptr=out_depth[r];
				for(int c=0; c<out_depth.cols; ++c)
					out_depth_ptr[c]=cv::saturate_cast<uchar>(gl_depthimg(r, c)*255);
			}
			gl_depthimg.Convert2AbsoluteZ();
			//cv::imwrite(lpGlobalMotion->ProjectDir()+std::string("_depth")+WANG::STR(depth_num++)+".png", out_depth);
			{
				std::string cur_img_name=cur_frame->m_sImgName;
				cv::imwrite(depth_path+cur_img_name.substr(cur_img_name.find_last_of("\\/")+1), out_depth);
				//cout<<cur_img_name.substr(cur_img_name.find_last_of("\\/")+1)<<endl;
				//PRINT(z_range);
				//getchar();
				//getchar();
			}
			
			double norm_coeff1=(z_range[0]+z_range[1])/(z_range[1]-z_range[0]);
			double norm_coeff2=(2*z_range[0]*z_range[1])/(z_range[1]-z_range[0]);
			//
			//std::vector<double> zdiff;
			//double max_zdiff=0;
			for (int i=0; i<faces.size(); ++i)
			{
				Wml::Vector3i cur_face=faces[i];
				Wml::Vector2d& cur_min_max=min_max_vec[i];
				//occlusion test
				//bool is_visible=true;
				int visible_p_num=0;
			/*	is_visible=model2depth.OcclusionQuery(points3d[cur_face[0]], points3d[cur_face[1]], points3d[cur_face[2]]);*/
				for(int j=0; j<3; ++j)
				{
					Wml::Vector3f curp=points3d[faces[i][j]];
					Wml::Vector3d imgp=ThreeDToImgD(cur_frame, curp);
					double norm_z=(imgp.Z()*norm_coeff1-norm_coeff2)/imgp.Z();
					norm_z=(norm_z+1.0)/2.0;
					//if(norm_z<0||norm_z>=1) { PRINT(norm_z); getchar();}
					if(!(imgp.X() > p.BoudarySize && imgp.X() < cur_frame->m_iWidth - 1 - p.BoudarySize &&
						imgp.Y() > p.BoudarySize && imgp.Y() < cur_frame->m_iHeight - 1 - p.BoudarySize))
					{
						//is_visible=false;
						break;
					}
					if (gl_depthimg.OcclusionQuery(imgp.X(), imgp.Y(), -imgp.Z(), p.DTol))
					{
						++visible_p_num;
						//is_visible=false;
						//break;
					}
					//int x=(int)(imgp.X()+0.5); int y=(int)(imgp.Y()+0.5);
					//float cur_depth=gl_depthimg(y, x);
					/*int x00=(int)(imgp.X()); int y00=(int)(imgp.Y());
					int x01=x00; int y01=y00+1;
					int x11=x00+1; int y11=y00+1;
					int x10=x00+1; int y10=y00;
					float cur_depth00=gl_depthimg(y00, x00);
					float cur_depth01=gl_depthimg(y01, x01);
					float cur_depth11=gl_depthimg(y11, x11);
					float cur_depth10=gl_depthimg(y10, x10);
					float cur_depth=std::numeric_limits<float>::max();
					cur_depth=std::min(cur_depth, cur_depth00);
					cur_depth=std::min(cur_depth, cur_depth01);
					cur_depth=std::min(cur_depth, cur_depth11);
					cur_depth=std::min(cur_depth, cur_depth10);*/
					//if(cur_depth!=1.0)
					//{
					//	int y=imgp.Y()+0.5; int x=imgp.X()+0.5;
					//	PRINT(gl_depthimg(y, x));
					//	PRINT(gl_depthimg(y, x)-norm_z);
					//	PRINT(cur_depth-norm_z);
					//	//PRINT(cur_depth); PRINT(norm_z);
					//	//PRINT(imgp); PRINT(z_range);
					//	getchar();
					//}
					/*max_zdiff=MAX(fabs(cur_depth-norm_z), max_zdiff);
					zdiff.push_back(norm_z-cur_depth);*/
					//if(fabs(cur_depth-norm_z)>0.01)
					/*if(fabs(norm_z-cur_depth)>0.004)
					{
						is_visible=false;
						break;
					}*/
				}
				//add center point constraint
				Wml::Vector3f cpoint=(points3d[faces[i][0]]+points3d[faces[i][1]]+points3d[faces[i][2]])/3.0;
				Wml::Vector3d cimgp=ThreeDToImgD(cur_frame, cpoint);
				double cnorm_z=(cimgp.Z()*norm_coeff1-norm_coeff2)/cimgp.Z();
				cnorm_z=(cnorm_z+1.0)/2.0;
				//if(norm_z<0||norm_z>=1) { PRINT(norm_z); getchar();}
				if(!(cimgp.X() > p.BoudarySize && cimgp.X() < cur_frame->m_iWidth - 1 - p.BoudarySize &&
					cimgp.Y() > p.BoudarySize && cimgp.Y() < cur_frame->m_iHeight - 1 - p.BoudarySize))
				{
					//is_visible=false;
					break;
				}
				if (gl_depthimg.OcclusionQuery(cimgp.X(), cimgp.Y(), -cimgp.Z(), p.DTol))
				{
					++visible_p_num;
					//is_visible=false;
					//break;
				}
				//if(!is_visible) continue;
				//if(visible_p_num!=3) continue;
				//PRINT(visible_p_num);
				//getchar();
				if (visible_p_num==0) continue;
				//caculate datacost
				double cur_score=GetDataCost(cur_frame, NrmVec[i], points3d[cur_face[0]], points3d[cur_face[1]], points3d[cur_face[2]]);
				if (!(visible_p_num==4))
					cur_score=0.001*cur_score;
				auto& cur_CandFrm_heap=CandFrm_heap[i];
				cur_min_max[0]=MIN(cur_min_max[0], cur_score);
				cur_min_max[1]=MAX(cur_min_max[1], cur_score);

				if(cur_CandFrm_heap.size()<p.CFNum)
					cur_CandFrm_heap.push(std::make_pair(cur_frame_id, cur_score));
				else
				{
					if(cur_CandFrm_heap.top().second<cur_score)
					{
						cur_CandFrm_heap.pop();
						cur_CandFrm_heap.push(std::make_pair(cur_frame_id, cur_score));
					}
				}
			}
			//PRINT(max_zdiff);
			//getchar();
			//Vector2File(zdiff, "zdiff.txt");
			//exit(0);
			cur_frame->m_DspImg.Clear();
		}
		CandFrm.clear(); CandFrm.resize(faces.size());
		for(int i=0; i<faces.size(); ++i)
		{
			if(CandFrm_heap[i].empty()) ++iInVldTriAglNum;
			//CandFrm[i].resize(CandFrm_heap[i].size());
		}
		for(int i=0; i<CandFrm_heap.size(); ++i)
		{
			std::map<int, double>& cur_CandFrm=CandFrm[i];
			//int cur_CandFrm_size=cur_CandFrm.size()-1;
			while(!CandFrm_heap[i].empty()&&cur_CandFrm.size()<p.CFNum)
			{
				std::pair<int, double> cur_top=CandFrm_heap[i].top();
				CandFrm_heap[i].pop();
				Wml::Vector2d cur_min_max=min_max_vec[i];
				cur_top.second=(cur_min_max[1]-cur_top.second)/(cur_min_max[1]-cur_min_max[0]);
				//cur_CandFrm[cur_CandFrm_size--]=cur_top;
				cur_CandFrm[cur_top.first]=cur_top.second;
			}
			//PRINT(cur_CandFrm.size());
			//getchar();
		}
		FUNCTION_END;
	}
	double TM::GetDataCost(CameraFrame* frame, Wml::Vector3f& normal, Wml::Vector3f& pt1, Wml::Vector3f& pt2, Wml::Vector3f& pt3)
	{
		int iWidth = frame->m_iWidth;
		int iHeight = frame->m_iHeight;
		Wml::Vector3f camPos(frame->m_viewAbsTransformMG(0,3), frame->m_viewAbsTransformMG(1,3), frame->m_viewAbsTransformMG(2,3));
		Wml::Vector3f ptDir1 = camPos - pt1;
		Wml::Vector3f ptDir2 = camPos - pt2;
		Wml::Vector3f ptDir3 = camPos - pt3;
		ptDir1.Normalize();
		ptDir2.Normalize();
		ptDir3.Normalize();

		double angle1 = normal.Dot(ptDir1);
		double angle2 = normal.Dot(ptDir2);
		double angle3 = normal.Dot(ptDir3);

		if(angle1 >= 0)
			angle1 = (angle1*angle1) + 1.0;
		else
			angle1 = 0;

		if(angle2 >= 0)
			angle2 = (angle2*angle2) + 1.0;
		else
			angle2 = 0;

		if(angle3 >= 0)
			angle3 = (angle3*angle3) + 1.0;
		else
			angle3 = 0;

		double f1 = 1.0, f2 = 1.0, f3 = 1.0;
		//这其实是个奇次坐标 z记录了 在摄像机坐标系下的z深度
		Wml::Vector3f imgPt1 = ThreeDToImg(frame, pt1);
		Wml::Vector3f imgPt2 = ThreeDToImg(frame, pt2);
		Wml::Vector3f imgPt3 = ThreeDToImg(frame, pt3);

		//double x = imgPt1.X();
		//double y = imgPt1.Y();
		//double z = imgPt1.Z();
		//if( x>=2 && x<=iWidth-3 && y>=2  && y<=iHeight-3 )
		//	f1 = (1.0/z + p.MaxDR) / p.MaxDR;

		//x = imgPt2.X();
		//y = imgPt2.Y();
		//z = imgPt2.Z();
		//if( x>=2 && x<=iWidth-3 && y>=2  && y<=iHeight-3 )
		//	f2 = (1.0/z + p.MaxDR) / p.MaxDR;

		//x = imgPt3.X();
		//y = imgPt3.Y();
		//z = imgPt3.Z();
		//if( x>=2 && x<=iWidth-3 && y>=2  && y<=iHeight-3 )
		//	f3 = (1.0/z + p.MaxDR) / p.MaxDR;

		//double score = angle1 * f1 + angle2 * f2 + angle3 * f3;
		double score=angle1/ptDir1.SquaredLength()+angle2/ptDir2.SquaredLength()+angle3/ptDir3.SquaredLength();
		return score;
	}

	/*void TextureMappingParser(int argc, char* argv[], std::string& loadfilepath, std::string& actsname, TM::TMParameters& tmparames)
	{
		FUNCTION_START;
		printf("The first argument must be the path where the acts file exits\nThe second argument must be the name of acts file\n");
		loadfilepath=argv[1]; actsname=argv[2];
		loadfilepath=loadfilepath+"\\";
		//parse acts file
		for(int i=3; i<argc; ++i)
		{
			if(strcmp(argv[i],"--MaxDR")==0)
			{
				if(sscanf(argv[++i], "%f", &(tmparames.MaxDR))==-1)
				{
					printf("An error occured when sscanf MaxDR!!\n");
					exit(0);
				}
			}
			else if (strcmp(argv[i], "--MinDR")==0)
			{
				if(sscanf(argv[++i], "%f", &(tmparames.MinDR))==-1)
				{
					printf("An error occured when sscanf MinDR!!\n");
					exit(0);
				}
			}
			else if(strcmp(argv[i], "--Step")==0)
			{
				if(sscanf(argv[++i], "%d", &(tmparames.TFStep))==-1)
				{
					printf("An error occured when sscanf TFStep!!\n");
					exit(0);
				}
			}
			else if(strcmp(argv[i], "--DTol")==0)
			{
				if(sscanf(argv[++i], "%f", &(tmparames.DTol))==-1)
				{
					printf("An error occured when sscanf DTol!!\n");
					exit(0);
				}
			}
			else if(strcmp(argv[i], "--CFNum")==0)
			{
				if(sscanf(argv[++i], "%d", &(tmparames.CFNum))==-1)
				{
					printf("An error occured when sscanf CFNum!!\n");
					exit(0);
				}
			}
			else if(strcmp(argv[i], "--MaxGCIternums")==0)
			{
				if(sscanf(argv[++i], "%d", &(tmparames.MaxGCIternums))==-1)
				{
					printf("An error occured when sscanf MaxGCIternums!!\n");
					exit(0);
				}
			}
			else if(strcmp(argv[i], "--BoudarySize")==0)
			{
				if(sscanf(argv[++i], "%d", &(tmparames.BoudarySize))==-1)
				{
					printf("An error occured when sscanf BoudarySize!!\n");
					exit(0);
				}
			}
			else if(strcmp(argv[i], "--SmoothNrmR")==0)
			{
				if(sscanf(argv[++i], "%d", &(tmparames.SmoothNrmR))==-1)
				{
					printf("An error occured when sscanf SmoothNrmR!!\n");
					exit(0);
				}
			}
			else if(strcmp(argv[i], "--WithUv")==0)
			{
				int temp;
				if(sscanf(argv[++i], "%d", &temp)==-1)
				{
					printf("An error occured when sscanf WithUv!!\n");
					exit(0);
				}
				tmparames.WithUv=false;
				if(temp!=0)
					tmparames.WithUv=true;
			}
			else if(strcmp(argv[i], "--HighResolution")==0)
			{
				int temp;
				if(sscanf(argv[++i], "%d", &temp)==-1)
				{
					printf("An error occured when sscanf HighResosution!!\n");
					exit(0);
				}
				tmparames.HighResolution=false;
				if(temp!=0)
					tmparames.HighResolution=true;
			}
			else if(strcmp(argv[i], "--ImageSize")==0)
			{
				int temp;
				if(sscanf(argv[++i], "%d", &temp)==-1)
				{
					printf("An error occured when sscanf HighResosution!!\n");
					exit(0);
				}
				if(temp>0)
					tmparames.ImageSize=temp;
				else
				{
					std::cout<<"The ImageSize "<<temp<<"is invalid!!"<<std::endl;
					exit(0);
				}
			}
		}
		printf("The aguments list:\n");
		printf("MaxDR= %.6f   MinDR= %.6f   TFStep= %d   DTol= %.6f\nSmoothNrmR=%d   CFNum=%d   MaxGCIternums=%d   BoudarySize=%d, ImageSize=%d\n",
			tmparames.MaxDR, tmparames.MinDR, tmparames.TFStep, tmparames.DTol, tmparames.SmoothNrmR, tmparames.CFNum, tmparames.MaxGCIternums, tmparames.BoudarySize, tmparames.ImageSize);
		FUNCTION_END;
		cout<<endl;
	}*/

}

namespace std
{
	ostream& operator<< (std::ostream& _out, const WANG::TMParameters& _tmparams)
	{
		cout<<"TFStep: "<<_tmparams.TFStep<<"  ";
		cout<<"DTol: "<<_tmparams.DTol<<"  ";
		cout<<"CFNum: "<<_tmparams.CFNum<<"  ";
		cout<<"MaxGCIternums: "<<_tmparams.MaxGCIternums<<"  ";
		cout<<endl;
		cout<<"BoundarySize: "<<_tmparams.BoudarySize<<"  ";
		cout<<"SmoothNrmR: "<<_tmparams.SmoothNrmR<<"  ";
		cout<<"SmoothWeight: "<<_tmparams.SmoothWeight<<"  ";
		cout<<"WithUv: "<<_tmparams.WithUv;
		cout<<endl;
		return _out;
	}
}
