#include "stdafx.h"
#include <io.h>
#include <direct.h>
#include <Wang_Helper.h>
#include "wang_Create_newtexture.h"
#include "wang_poissonfusion.h"
#include "conjugateGradientPrecondv1.h"
static double fixed_coefficient=0.01;
namespace WANG
{
	void PoissonFusion::TexImgM::Check()
	{
		//set invalid components as is_processed;
		for (std::set<int>::iterator iter=objdata.invalid_texid.begin(); iter!=objdata.invalid_texid.end(); ++iter)
			isprocessed[*iter]=HANDLED;
		std::ofstream TexImgMtest("TexImgMtest.txt");
		std::vector<std::set<int>>& components_link_map=objdata.teximg_link_map;
		TexImgMtest<<"The unhandled components id: "<<std::endl;
		std::vector<int> unhandledids;
		for (int i=0; i<isprocessed.size(); ++i)
		{
			if(isprocessed[i]!=HANDLED)
			{
				TexImgMtest<<i<<"  ";
				unhandledids.push_back(i);
			}
		}
		TexImgMtest<<std::endl;
		std::ostream_iterator<int> TexImgMtest_iter(TexImgMtest, " ");
		for(int i=0; i<unhandledids.size(); ++i)
		{
			int cur_id=unhandledids[i];
			TexImgMtest<<"Cur id: "<<cur_id<<std::endl;
			std::copy(components_link_map[cur_id].begin(), components_link_map[cur_id].end(), TexImgMtest_iter);
			TexImgMtest<<std::endl;
		}
	}
	void PoissonFusion::TexImgM::SaveTex(std::vector<int>& texids, cv::Mat3f& tar_img, cv::Mat1b& mask, bool remapping)
	{
		std::vector<Wml::Vector4<int>>& tex_rois=objdata.tex_rois;
		std::vector<std::pair<std::string, std::vector<int>>>& original_teximg_components=objdata.original_teximg_components;
		std::vector<std::string>& tex_filenames=objdata.tex_filenames;
		std::vector<std::vector<int>>& teximg_faces=objdata.teximg_faces;
		std::vector<Wml::Vector3i>& tex_faces=objdata.tex_faces;
		std::vector<Wml::Vector2f>& tex_points=objdata.tex_points;
		if(remapping)
		{
			std::vector<Wml::Vector2f>& new_tex_points=objdata.tex_points_v[1];
			std::vector<Wml::Vector3i>& new_tex_faces=objdata.tex_faces_v[1];
			for (int i=0; i<texids.size(); ++i)
			{
				int cur_texid=texids[i];
				std::vector<int>& cur_components=original_teximg_components[cur_texid].second;
				for (int j=0; j<cur_components.size(); ++j)
				{
					int cur_component_id=cur_components[j];
					std::vector<int>& cur_component_faces=teximg_faces[cur_component_id];
					if(isprocessed[cur_component_id]==HANDLED)
					{
						if(tex_components[cur_component_id].empty()) {printf("The %d component is not existing in TexImgM!!\n"); continue;}
						for (int f=0; f<cur_component_faces.size(); ++f)
						{
							int curfaceid=cur_component_faces[f];
							Wml::Vector3i curtexfaceid=tex_faces[curfaceid];
							Wml::Vector3i newcurtexfaceid=new_tex_faces[curfaceid];
							std::vector<Wml::Vector2f> src_trg(3);
							std::vector<Wml::Vector2f> tar_trg(3);
							for(int c=0; c<3; ++c)
							{
								src_trg[c]=tex_points[curtexfaceid[c]];
								tar_trg[c]=new_tex_points[newcurtexfaceid[c]];
							}
							copy_triangle(tex_components[cur_component_id], tar_img, mask, src_trg, tar_trg);
						}
						if(isprocessed[cur_component_id]==HANDLED)
							TeximgVarNums[cur_texid]-=tex_components[cur_component_id].rows*tex_components[cur_component_id].cols;
					}
				}
			}
		}
		else
		{
			for (int i=0; i<texids.size(); ++i)
			{
				int cur_texid=texids[i];
				std::vector<int>& cur_components=original_teximg_components[cur_texid].second;
				for (int j=0; j<cur_components.size(); ++j)
				{
					int cur_component_id=cur_components[j];
					if(isprocessed[cur_component_id]==HANDLED)
					{
						if(tex_components[cur_component_id].empty()) {printf("The %d component is not existing in TexImgM!!\n"); continue;}
						cv::imwrite(savefilepath+ObjInterface::TexComponentName(tex_filenames[cur_component_id], tex_rois[cur_component_id]), tex_components[cur_component_id]);
						if(isprocessed[cur_component_id]==HANDLED)
							TeximgVarNums[cur_texid]-=tex_components[cur_component_id].rows*tex_components[cur_component_id].cols;
					}
				}
			}
		}
		
	}
	void PoissonFusion::TexImgM::LoadTex(std::vector<int>& texids)
	{
		std::vector<Wml::Vector4<int>>& tex_rois=objdata.tex_rois;
		std::vector<std::pair<std::string, std::vector<int>>>& original_teximg_components=objdata.original_teximg_components;
		for (int i=0; i<texids.size(); ++i)
		{
			int cur_texid=texids[i];
			if(isLoaded[cur_texid]) continue;
			isLoaded[cur_texid]=true;
			cv::Mat3b srcimg=cv::imread(objdata.loadfilepath+original_teximg_components[cur_texid].first);
			/*if(srcimg.size()==cv::Size(200, 200))
			{
				PRINT(srcimg.size());
				PRINT(cur_texid);
				PRINT(original_teximg_components[cur_texid].first);
				PRINT(original_teximg_components[cur_texid].second.size());
				for(int cao=0; cao<original_teximg_components[cur_texid].second.size(); ++cao)
				{
					int cur_roiid=original_teximg_components[cur_texid].second[cao];
					PRINT(cur_roiid);
					PRINT(tex_rois[cur_roiid]);
					cv::Rect roi(tex_rois[cur_roiid][0], tex_rois[cur_roiid][1], tex_rois[cur_roiid][2]-tex_rois[cur_roiid][0], tex_rois[cur_roiid][3]-tex_rois[cur_roiid][1]);
					PRINT(roi);
				}
				exit(0);
			}*/
			if(srcimg.empty()) {printf("the teximg %s is empty!\n", original_teximg_components[cur_texid].first); continue;}
			for (int j=0; j<original_teximg_components[cur_texid].second.size(); ++j)
			{
				int cur_roiid=original_teximg_components[cur_texid].second[j];
				cv::Rect roi(tex_rois[cur_roiid][0], tex_rois[cur_roiid][1], tex_rois[cur_roiid][2]-tex_rois[cur_roiid][0], tex_rois[cur_roiid][3]-tex_rois[cur_roiid][1]);
				//PRINT(srcimg.size());
				//PRINT(roi);
				tex_components[cur_roiid]=cv::Mat3b(srcimg, roi).clone();
			}
		}
	}
	void PoissonFusion::TexImgM::ClearTex(std::vector<int>& texids)
	{
		//this fuctiong delete tex components which have been handled and delete them in original_teximg_components
		std::vector<std::pair<std::string, std::vector<int>>>& original_teximg_components=objdata.original_teximg_components;
		for (int i=0; i<texids.size(); ++i)
		{
			int cur_texid=texids[i];
			std::vector<int> temp;
			for (int j=0; j<original_teximg_components[cur_texid].second.size(); ++j)
			{
				int cur_roiid=original_teximg_components[cur_texid].second[j];
				if(isprocessed[cur_roiid]==HANDLED)
					tex_components[cur_roiid].release();
				else
					temp.push_back(cur_roiid);
			}
			original_teximg_components[cur_texid].second.swap(temp);
		}
	}
	void PoissonFusion::TexImgM::GetSubTexLinkMap(const std::vector<int>& handled_tex_ids, std::vector<std::pair<int, std::vector<int>>>& sub_link_map)
	{
		printf("Start GetSubLinkMap!!\n");
		std::vector<Wml::Vector4<int>>& tex_rois=objdata.tex_rois;
		std::vector<std::pair<std::string, std::vector<int>>>& original_teximg_components=objdata.original_teximg_components;
		std::vector<std::set<int>>& components_link_map=objdata.teximg_link_map;
		std::vector<int> sub_hash(tex_rois.size(), INVALID);
		for (int i=0; i<handled_tex_ids.size(); ++i)
		{
			int cur_handled_id=handled_tex_ids[i];
			for (int j=0; j<original_teximg_components[cur_handled_id].second.size(); ++j)
				sub_hash[original_teximg_components[cur_handled_id].second[j]]=USED;
		}
		//
		for (int i=0; i<handled_tex_ids.size(); ++i)
		{
			int cur_handled_id=handled_tex_ids[i];
			const std::vector<int>& cur_components=original_teximg_components[cur_handled_id].second;
			for (int j=0; j<cur_components.size(); ++j)
			{
				
				int cur_component_id=cur_components[j];
				std::pair<int, std::vector<int>> temp(cur_component_id, std::vector<int>());//update sub_link_map
				std::set<int> new_neighbors;//update components_link_map
				std::set<int>& cur_neighbors=components_link_map[cur_component_id];
				for (std::set<int>::iterator iter=cur_neighbors.begin(); iter!=cur_neighbors.end(); ++iter)
				{
					if(sub_hash[*iter]!=INVALID)
						temp.second.push_back(*iter);
					else
						new_neighbors.insert(*iter);
				}
				//update boundary flag according to the last state flags of isprocessed
				if(isprocessed[cur_component_id]==HANDLING)
					isboundary[cur_component_id]=true;
				else
					isboundary[cur_component_id]=false;
				//update isprocessed flag state
				if(temp.second.size()==cur_neighbors.size())
					isprocessed[cur_component_id]=HANDLED;
				else if(!temp.second.empty())
					isprocessed[cur_component_id]=HANDLING;
				if(temp.second.empty())
					continue;
				//update 
				sub_hash[cur_component_id]=sub_link_map.size();
				sub_link_map.push_back(temp);
				cur_neighbors=new_neighbors;
			}
		}
		//
		for (int i=0; i<sub_link_map.size(); ++i)
		{
			std::vector<int>& cur_neighbors=sub_link_map[i].second;
			for(int j=0; j<cur_neighbors.size(); ++j)
			{
				int cur_neighbor_id=cur_neighbors[j];
				if(sub_hash[cur_neighbor_id]==USED||sub_hash[cur_neighbor_id]==INVALID)
					printf("It is impossiable!!\n");
				cur_neighbors[j]=sub_hash[cur_neighbor_id];
			}
		}
		printf("finish GetSubLinkMap!!\n");
		//exit(0);
	}
	//PoissonFusion
	void PoissonFusion::Clear()
	{
		objdata=NULL;
		remapping=false;
		new_teximg.release();
		mask.release();
		TexImgMptr.reset();
		var_index_images.clear();

		X.resize(0); B.resize(0);
		mtx.resize(0, 0);
	}
	void PoissonFusion::Setdata(ObjInterface* _objdata)
	{
		Clear();
		objdata=_objdata;
		savefilepath=objdata->loadfilepath+"\\AfterPoisson\\";
		//A=NULL;
	}
	void PoissonFusion::Run(int HandledVarNum, int max_iters, int NewTexSize)
	{
		if(!objdata) { cout<<"No objdata exists in PoissonFusion!!"<<endl; exit(0); }
		//create new path to save data
		if(_access(savefilepath.c_str(), 0)==0) printf("Path %s  exists\n", savefilepath.c_str());
		else
		{
			if(_mkdir(savefilepath.c_str())==0)
				printf("create path %s succeed!!\n", savefilepath.c_str());
			else
			{
				printf("create path %s failed!!\n", savefilepath.c_str());
				exit(0);
			}
		}
		//do uvatlas
		//UVatlas uvatlas(savefilepath);
		/*printf("start compute IMT!!\n");
		cv::Mat3b texture_img=cv::imread(objdata.loadfilepath+objdata.tex_filenames[1]);
		int image_height=texture_img.rows;
		int image_width=texture_img.cols;
		printf("img height and width: %d %d\n", image_height, image_width);
		uvatlas.compute_IMT(objdata.points3d, objdata.faces, objdata.tex_points, objdata.tex_faces, image_height, image_width);
		printf("compute IMT end!!\n");*/
		//uvatlas.run_UVatlas(objdata.points3d, objdata.faces, UVatlas::UVatlas_Setting(0, 0.1667, NewTexSize, NewTexSize, 0.01*NewTexSize), new_tex_points, new_tex_faces);
		//allocate remapping
	/*	if(objdata->tex_points_v.size()>=2)
			remapping=true;*/
		//create new texture
		if(remapping)
		{
			new_teximg=cv::Mat3f(NewTexSize, NewTexSize, cv::Vec3f(0.0, 0.0, 0.0));
			mask=cv::Mat1b(NewTexSize, NewTexSize, uchar(0));
		}
		iterative_poisson_fusion(HandledVarNum, max_iters);
		//save texture
		if(remapping)
		{
			printf("average the image!!!\n");
			cv::Mat3b save_newtexture=cvCreateImage(cvSize(NewTexSize, NewTexSize), IPL_DEPTH_8U, 3);
			std::cout<<new_teximg.rows<<" "<<new_teximg.cols<<std::endl;
			for (int r=0; r<new_teximg.rows; ++r)
			{
				//printf("!!!!!!!  ");
				cv::Vec3f* fptr=new_teximg[r];
				cv::Vec3b* save_newtexture_ptr=save_newtexture[r];
				uchar* newtexture_mask_ptr=mask[r];
				for (int c=0; c<new_teximg.cols; ++c)
				{
					if(newtexture_mask_ptr[c]!=0)
					{
						fptr[c]=fptr[c]/newtexture_mask_ptr[c];
						newtexture_mask_ptr[c]=255;
						save_newtexture_ptr[c][0]=fptr[c][0];
						save_newtexture_ptr[c][1]=fptr[c][1];
						save_newtexture_ptr[c][2]=fptr[c][2];
					}
					else
					{
						save_newtexture_ptr[c]=cv::Vec3b(125, 125, 125);
					}
				}
			}
			//realse memory
			new_teximg.release();
			//mask.release();
			//ExtendBoundary
			Extendboundary(save_newtexture, mask, 0.005*NewTexSize);
			//保存图像
			printf("save new_texture.jpg start!!!\n");
			//cv::imwrite("new_texture.jpg", save_newtexture);
			cvSaveImage((savefilepath+"_texture1.jpg").c_str(), &((IplImage)save_newtexture));
			//cvSaveImage((savefilepath+"_texture1mask.jpg").c_str(), &((IplImage)newtexture_mask));
			printf("save new_texture.jpg end!!!\n");
			//save obj
			std::vector<std::string> tex_filenames_temp;
			tex_filenames_temp.push_back("_texture1.jpg");
			objdata->tex_points.swap(objdata->tex_points_v[1]);
			objdata->tex_faces.swap(objdata->tex_faces_v[1]);
			objdata->tex_id.swap(std::vector<int>(objdata->faces.size(), 0));
			objdata->tex_filenames.swap(tex_filenames_temp);
			objdata->invalid_texid.clear();
			objdata->TextureSeparated=false;
		}
		std::cout<<"save obj"<<std::endl;
		//objdata->save_tex_imgs(savefilepath);
		objdata->loadfilepath=savefilepath;
		objdata->UpdateAfterSeparate();
		printf("\a\a\a\a\a");

	}
	void PoissonFusion::iterative_poisson_fusion(int HandledVarNum, int max_iters)
	{
		printf("start poissonfusion, process %d variables at one time!!\n", HandledVarNum);
		//get original teximg_link_map
		objdata->build_link_map();
		std::vector<std::set<int>> original_teximg_link_map=objdata->teximg_link_map;
		objdata->SeparateTexture();
		objdata->original_tex_rois=objdata->tex_rois;
		//PRINT(objdata->tex_faces[129175]);
		//exit(0);
		objdata->build_teximg_faces();
		//这里的迭代算法是一次处理一张纹理图周围N张纹理的缝 然后将其中不在边界上的纹理图在一个数据结构中删掉 然后在从中找到一个新的点重新计算
		//initialization
		TexImgMptr.reset(new TexImgM(*objdata, savefilepath));
		std::vector<bool> isprocessed(original_teximg_link_map.size(), false);
		int isprocessed_num=0;
		//find the first valid texture
		while(objdata->original_teximg_components[isprocessed_num].first.empty())
			++isprocessed_num;
		//所有的handle_tex_ids
		//FILE* test=fopen("iterative_poisson_fusion.txt", "w");
		while(isprocessed_num<isprocessed.size())
		{
			printf("%d\n", isprocessed_num);
			//BFS data
			std::deque<int> Q;
			std::vector<bool> is_visited(original_teximg_link_map.size(), false);
			std::vector<int> handled_tex_ids;
			int VarNum=0;
			//push head
			bool flag=false;
			Q.push_back(isprocessed_num);
			is_visited[isprocessed_num]=true;
			handled_tex_ids.push_back(isprocessed_num);
			VarNum=VarNum+TexImgMptr->TeximgVarNums[isprocessed_num];
			if(VarNum>HandledVarNum){printf("ERROR!!The texture %s pixels are larger than %d!!\n", objdata->tex_filenames[isprocessed_num], HandledVarNum); exit(0);}

			//这个flag标识是否达到N这个界限
			while(!Q.empty()&&!flag)
			{
				int h=Q.front();
				//printf("%d\n", h);
				Q.pop_front();
				std::set<int>::iterator iter=original_teximg_link_map[h].begin();
				while(iter!=original_teximg_link_map[h].end())
				{
					int cur=*iter;
					if(!is_visited[cur])
					{
						//conservative estimate
						int curTeximgVarNums=TexImgMptr->TeximgVarNums[cur];
						VarNum=VarNum+curTeximgVarNums;
						if(VarNum>HandledVarNum)
						{
							flag=true;
							break;
						}
						Q.push_back(cur);
						is_visited[cur]=true;
						if(curTeximgVarNums!=0)
							handled_tex_ids.push_back(cur);
					}
					//modify graph
					std::set<int>::iterator cur_iter=iter++;
					original_teximg_link_map[cur].erase(h);
					original_teximg_link_map[h].erase(cur_iter);
				}
				if(!flag)
					isprocessed[h]=true;//说明这个h被完全处理了
			}
			printf("Currently handled tex nums: %d\n", handled_tex_ids.size());
			for (int i=0; i<handled_tex_ids.size(); ++i)
				printf("%d%c", handled_tex_ids[i], i<handled_tex_ids.size()-1&&((i+1)%15)?' ':'\n');
			//get sub_link_map
			std::vector<std::pair<int, std::vector<int>>> sub_link_map;
			TexImgMptr->GetSubTexLinkMap(handled_tex_ids, sub_link_map);
			TexImgMptr->LoadTex(handled_tex_ids);
			//poisson_fusion(sub_link_map, max_iters);
			//save tex
			TexImgMptr->SaveTex(handled_tex_ids, new_teximg, mask, remapping);
			//getchar();
			TexImgMptr->ClearTex(handled_tex_ids);
			while(isprocessed_num<isprocessed.size()&&isprocessed[isprocessed_num])
				++isprocessed_num;
			//getchar();
		}
		TexImgMptr->Check();
		//cnmb
		/*std::vector<int> handled_tex_ids;
		for(int i=0; i<objdata->tex_filenames.size(); ++i)
			if(objdata->invalid_texid.find(i)==objdata->invalid_texid.end())
				handled_tex_ids.push_back(i);*/
		//std::vector<std::pair<int, std::vector<int>>> sub_link_map;
		//TexImgMptr->GetSubTexLinkMap(handled_tex_ids, sub_link_map);
		//TexImgMptr->LoadTex(handled_tex_ids);
		////poisson_fusion(sub_link_map, max_iters);
		////save tex
		//TexImgMptr->SaveTex(handled_tex_ids, new_teximg, mask, remapping);
		////getchar();
		//TexImgMptr->ClearTex(handled_tex_ids);
	//	fclose(test);
		printf("\a\a\a\a\a");

	}
	void PoissonFusion::ReserveRoomForSparseMtx(std::vector<std::pair<int, std::vector<int>>>& sub_link_map)
	{
		int texnums=sub_link_map.size();
		Eigen::VectorXi reserve_room(iVarCount);
#ifdef GPU_VERSION
		reserve_room.fill(5);
#else
		reserve_room.fill(3);
#endif
		std::map<Wml::Vector2<int>, std::vector<Wml::Vector2<int>>>& tex_seams=objdata->tex_seams;
		std::map<Wml::Vector2<int>, std::vector<Wml::Vector2<int>>>::iterator r;
		std::vector<Wml::Vector3i>& faces=objdata->faces;
		std::vector<Wml::Vector2f>& tex_points=objdata->tex_points;
		std::vector<Wml::Vector3i>& tex_faces=objdata->tex_faces;
		//遍历所有存在的缝
		for (int i=0; i<sub_link_map.size(); ++i)
		{
			int cur_roiid=sub_link_map[i].first;
			const std::vector<int>& cur_neighbors=sub_link_map[i].second;
			for (int j=0; j<cur_neighbors.size(); ++j)
			{
				if(cur_neighbors[j]<i) continue;
				int cur_neighbor_roiid=sub_link_map[cur_neighbors[j]].first;
				int cur_roiid_temp=cur_roiid; int cur_index_temp=i;
				int cur_neighbor_roiid_temp=cur_neighbor_roiid; int neighbor_index_temp=cur_neighbors[j];
				if(cur_roiid_temp>cur_neighbor_roiid_temp) {std::swap(cur_roiid_temp, cur_neighbor_roiid_temp); std::swap(cur_index_temp, neighbor_index_temp);}
				
				cv::Mat3b& tex_images1=TexImgMptr->tex_components[cur_roiid_temp];
				cv::Mat3b& tex_images2=TexImgMptr->tex_components[cur_neighbor_roiid_temp];
				std::vector<Wml::Vector2<int>>& link_edges=tex_seams[Wml::Vector2<int>(cur_roiid_temp, cur_neighbor_roiid_temp)];
				ZIntImage& index_img1=var_index_images[cur_index_temp];
				ZIntImage& index_img2=var_index_images[neighbor_index_temp];

				if(tex_images1.cols!=index_img1.GetWidth()||tex_images1.rows!=index_img1.GetHeight())
				{
					printf("tex_images1 size is not equal to index_img1!!\n");
					exit(0);
				}
				if(tex_images2.cols!=index_img2.GetWidth()||tex_images2.rows!=index_img2.GetHeight())
				{
					printf("tex_images2 size is not equal to index_img2!!\n");
					exit(0);
				}
				//Add Priors of Common Points
				int invalid_nums=0;
				for (int link_edges_i=0; link_edges_i<link_edges.size(); ++link_edges_i)
				{
					int f1=link_edges[link_edges_i][0]; 
					int f2=link_edges[link_edges_i][1];
					int f1_compoint_pos[2], f2_compoint_pos[2], p_nums=0;
					for(int i=0; i<3; ++i)
					{
						for(int j=0; j<3; ++j)
						{	
							if(faces[f1][i]==faces[f2][j])
							{
								f1_compoint_pos[p_nums]=i;
								f2_compoint_pos[p_nums]=j;
								++p_nums;
								if(p_nums==2)
									break;
							}
						}
						if(p_nums==2)
							break;
					}
					Wml::Vector2f imgPt1_1, imgPt1_2, imgPt2_1, imgPt2_2;
					Wml::Vector2f imgPt1_1_uniform, imgPt1_2_uniform, imgPt2_1_uniform, imgPt2_2_uniform;
					imgPt1_1_uniform=tex_points[tex_faces[f1][f1_compoint_pos[0]]];
					imgPt2_1_uniform=tex_points[tex_faces[f1][f1_compoint_pos[1]]];
					imgPt1_2_uniform=tex_points[tex_faces[f2][f2_compoint_pos[0]]];
					imgPt2_2_uniform=tex_points[tex_faces[f2][f2_compoint_pos[1]]];

					imgPt1_1.X()=(tex_images1.cols)*imgPt1_1_uniform[0];
					imgPt1_1.Y()=(tex_images1.rows)*(1-imgPt1_1_uniform[1]);

					imgPt2_1.X()=(tex_images1.cols)*imgPt2_1_uniform[0];
					imgPt2_1.Y()=(tex_images1.rows)*(1-imgPt2_1_uniform[1]);

					imgPt1_2.X()=(tex_images2.cols)*imgPt1_2_uniform[0];
					imgPt1_2.Y()=(tex_images2.rows)*(1-imgPt1_2_uniform[1]);

					imgPt2_2.X()=(tex_images2.cols)*imgPt2_2_uniform[0];
					imgPt2_2.Y()=(tex_images2.rows)*(1-imgPt2_2_uniform[1]);

					double samples=MAX(((imgPt1_1-imgPt2_1).Length()+(imgPt1_2-imgPt2_2).Length())/2, 2.0);
					for(int k = 0; k <= samples; ++k)
					{
						Wml::Vector2f pt1, pt2;
						pt1 = imgPt1_1 * (k / samples) + imgPt2_1 * ((samples - k) / samples);
						pt2 = imgPt1_2 * (k / samples) + imgPt2_2 * ((samples - k) / samples);
						{
							int x1 = pt1.X() + 0.5;
							int y1 = pt1.Y() + 0.5;
							int x2 = pt2.X() + 0.5;
							int y2 = pt2.Y() + 0.5;

							int iW1 = tex_images1.cols;
							int iH1 = tex_images1.rows;
							int iW2 = tex_images2.cols;
							int iH2 = tex_images2.rows;

							if(x1>=1 && x1<=iW1-2 && y1>=1 && y1<=iH1-2 && x2>=1 && x2<=iW2-2 && y2>=1 && y2<=iH2-2)
							{
								Wml::GMatrixf A(8,1),ATA(8,8);
								Wml::Vector2<int> p1_00,p1_01,p1_10,p1_11;
								float  a_00,a_01,a_10,a_11;
								float dx1,dy1;
								Wml::Vector2<int> p2_00,p2_01,p2_10,p2_11;
								float  b_00,b_01,b_10,b_11;
								float dx2,dy2;

								p1_00.X()= pt1.X(); p1_00.Y() = pt1.Y();
								p1_01.X()= pt1.X(); p1_01.Y() = pt1.Y()+1;
								p1_10.X()= pt1.X()+1; p1_10.Y() = pt1.Y();
								p1_11.X()= pt1.X()+1; p1_11.Y()= pt1.Y()+1;
								dx1 = pt1.X() - p1_00.X();
								dy1 = pt1.Y() - p1_00.Y();
								a_00 = (1-dx1) * (1-dy1);
								a_01 = (1-dx1) * dy1;
								a_10 = dx1 * (1-dy1);
								a_11 = dx1 * dy1;

								p2_00.X() = pt2.X(); p2_00.Y()= pt2.Y();
								p2_01.X()= pt2.X(); p2_01.Y()= pt2.Y()+1;
								p2_10.X()= pt2.X()+1; p2_10.Y() = pt2.Y();
								p2_11.X()= pt2.X()+1; p2_11.Y()= pt2.Y()+1;
								dx2 = pt2.X() - p2_00.X();
								dy2 = pt2.Y() - p2_00.Y();
								b_00 = (1-dx2) * (1-dy2);
								b_01 = (1-dx2) * dy2;
								b_10 = dx2 * (1-dy2);
								b_11 = dx2 * dy2;

								A(0,0) = a_00, A(1,0) = a_01, A(2,0) = a_10, A(3,0) = a_11;
								A(4,0) = -b_00, A(5,0) = -b_01, A(6,0) = -b_10, A(7,0) = -b_11;

								ATA = A * A.Transpose();

								std::vector<int> local_inds(8);
								local_inds[0] =index_img1.at(p1_00.X(), p1_00.Y());
								local_inds[1] =index_img1.at(p1_01.X(), p1_01.Y());
								local_inds[2] =index_img1.at(p1_10.X(), p1_10.Y());
								local_inds[3] =index_img1.at(p1_11.X(), p1_11.Y());

								local_inds[4] =index_img2.at(p2_00.X(), p2_00.Y());
								local_inds[5] =index_img2.at(p2_01.X(), p2_01.Y());
								local_inds[6] =index_img2.at(p2_10.X(), p2_10.Y());
								local_inds[7] =index_img2.at(p2_11.X(), p2_11.Y());

								for(int i=0;i<8;++i)
#ifdef GPU_VERSION
									reserve_room[local_inds[i]]+=7;
#else
								{
									for(int j=0; j<=i; ++j)
									{
										if(local_inds[i]<0||local_inds[j]<0)
										{
											printf("%f,%f,%f,%f---%d,%d,%d,%d\n",pt1.X(),pt1.Y(),pt2.X(),pt2.Y(),var_index_images[0].GetWidth(),var_index_images[0].GetHeight(),
												var_index_images[1].GetWidth(),var_index_images[1].GetHeight());
											printf("%d,%d\n",local_inds[i],local_inds[j]);
										}
										//if(i!=j)
										int i_temp=local_inds[i], j_temp=local_inds[j];
										if(i_temp<j_temp) std::swap(i_temp, j_temp);
										++reserve_room[j_temp];
									}
								}
#endif
							}
						}
					}
				}

			}
		}
		if(mtx.outerSize()!=reserve_room.size())
		{
			cout<<"mtx.outerSize()!=reserve_room.size()"<<endl;
			exit(0);
		}
		else
			mtx.reserve(reserve_room);
		//std::ofstream freserve_room("reserve_room.txt");
		//freserve_room<<reserve_room<<endl;
	}
	void PoissonFusion::AddSpatialPriorsForOne(cv::Mat3b& tex_img, ZIntImage& index_img, int c)
	{
		int tex_height=tex_img.rows;
		int tex_width=tex_img.cols;
		if(tex_height!=index_img.GetHeight()||tex_width!=index_img.GetWidth())
		{
			printf("The Sizes of tex_img and index_img are not equal!!\n");
			exit(0);
		}
		//对x赋值
		for(int y=0; y<tex_height; ++y)
		{	
			cv::Vec3b* tex_img_ptr=tex_img[y];
			for(int x=0; x<tex_width; ++x)
				X[index_img.at(x, y)]=tex_img_ptr[x][c];
		}
		// Add Spatial Priors
		// To bulid the Eigen Sparse Matrix with high efficiency, Set the diagonal elements first
		if(c==0)
		{
			for(int y=0; y<tex_height; ++y)
			{
				int index1=index_img.at(0, y), index2=index_img.at(tex_width-1, y);
				if(y==0||y==tex_height-1)
				{
					mtx.insert(index1, index1)=2;
					mtx.insert(index2, index2)=2;
				}
				else
				{
					mtx.insert(index1, index1)=3;
					mtx.insert(index2, index2)=3;
				}
			}
			for(int x=1; x<tex_width-1; ++x)
			{
				int cur_index=index_img.at(x, 0);
				mtx.insert(cur_index, cur_index)=3;
				cur_index=index_img.at(x, tex_height-1);
				mtx.insert(cur_index, cur_index)=3;
			}
			for(int y=1; y<tex_height-1; ++y)
				for(int x=1; x<tex_width-1; ++x)
				{
					int cur_index=index_img.at(x, y);
					mtx.insert(cur_index, cur_index)=4;
				}
		}
		//x direction
		for (int y=0; y<tex_height; ++y)
		{
			cv::Vec3b* tex_img_ptr=tex_img[y];
			for (int x=0; x<tex_width-1; ++x)
			{
				int index=index_img.at(x, y);
				int index2=index_img.at(x+1, y);
				if(c == 0)
				{
					mtx.insert(index2, index)=-1;
#ifdef GPU_VERSION
					mtx.insert(index, index2)=-1;
#endif
					/*_mtx.Element_Add(index, index, 1);
					_mtx.Element_Add(index, index2, -1);
					_mtx.Element_Add(index2, index, -1);
					_mtx.Element_Add(index2, index2, 1);*/
				}
				B[index]+=tex_img_ptr[x][c]-tex_img_ptr[x+1][c];
				B[index2]-=tex_img_ptr[x][c]-tex_img_ptr[x+1][c];
			}
		}
		//y direction
		for (int y=0; y<tex_height-1; ++y)
		{
			cv::Vec3b* cur_row_ptr=tex_img[y];
			cv::Vec3b* next_row_ptr=tex_img[y+1];
			for (int x=0; x<tex_width; ++x)
			{
				int index=index_img.at(x, y);
				int index2=index_img.at(x, y+1);
				if(c == 0)
				{
					mtx.insert(index2, index)=-1;
#ifdef GPU_VERSION
					mtx.insert(index, index2)=-1;
#endif
				/*	_mtx.Element_Add(index, index, 1);
					_mtx.Element_Add(index, index2, -1);
					_mtx.Element_Add(index2, index, -1);
					_mtx.Element_Add(index2, index2, 1);*/
				}
				B[index]+=cur_row_ptr[x][c]-next_row_ptr[x][c];
				B[index2]-=cur_row_ptr[x][c]-next_row_ptr[x][c];
			}
		}
	}
	void PoissonFusion::AddBoundaryConstraintForOne(cv::Mat3b& tex_img, ZIntImage& index_img, int c)
	{
		//printf("AddBoundaryConstraintForOne\n");
		int tex_height=tex_img.rows;
		int tex_width=tex_img.cols;
		for (int y=0; y<tex_height; ++y)
		{
			cv::Vec3b* tex_img_ptr=tex_img[y];
			for (int x=0; x<tex_width; ++x)
			{
				int index=index_img.at(x, y);
				if(c == 0)
				{
					mtx.coeffRef(index,index)+=fixed_coefficient;
					//_mtx.Element_Add(index,index, fixed_coefficient);
				}
				B[index] += tex_img_ptr[x][c] * fixed_coefficient;
			}
		}
	}
	void PoissonFusion::AddSpatialPriors(const std::vector<std::pair<int, std::vector<int>>>& sub_link_map, int c)
	{
		for (int i=0; i<sub_link_map.size(); ++i)
		{
			int cur_component_id=sub_link_map[i].first;
			AddSpatialPriorsForOne(TexImgMptr->tex_components[cur_component_id], var_index_images[i], c);
			if(TexImgMptr->isboundary[cur_component_id])
				AddBoundaryConstraintForOne(TexImgMptr->tex_components[cur_component_id], var_index_images[i], c);
		}
	}
	void PoissonFusion::AddCommonPointsPriorsForTwo(cv::Mat3b& tex_images1, cv::Mat3b& tex_images2, std::vector<Wml::Vector2<int>>& link_edges, ZIntImage& index_img1, ZIntImage& index_img2)
	{
		if(tex_images1.cols!=index_img1.GetWidth()||tex_images1.rows!=index_img1.GetHeight())
		{
			printf("tex_images1 size is not equal to index_img1!!\n");
			exit(0);
		}
		if(tex_images2.cols!=index_img2.GetWidth()||tex_images2.rows!=index_img2.GetHeight())
		{
			printf("tex_images2 size is not equal to index_img2!!\n");
			exit(0);
		}
		std::vector<Wml::Vector3i>& faces=objdata->faces;
		std::vector<Wml::Vector2f>& tex_points=objdata->tex_points;
		std::vector<Wml::Vector3i>& tex_faces=objdata->tex_faces;
		//Add Priors of Common Points
		int invalid_nums=0;
		for (int link_edges_i=0; link_edges_i<link_edges.size(); ++link_edges_i)
		{
			int f1=link_edges[link_edges_i][0]; 
			int f2=link_edges[link_edges_i][1];
			int f1_compoint_pos[2], f2_compoint_pos[2], p_nums=0;
			for(int i=0; i<3; ++i)
			{
				for(int j=0; j<3; ++j)
				{	
					if(faces[f1][i]==faces[f2][j])
					{
						f1_compoint_pos[p_nums]=i;
						f2_compoint_pos[p_nums]=j;
						++p_nums;
						if(p_nums==2)
							break;
					}
				}
				if(p_nums==2)
					break;
			}
			Wml::Vector2f imgPt1_1, imgPt1_2, imgPt2_1, imgPt2_2;
			Wml::Vector2f imgPt1_1_uniform, imgPt1_2_uniform, imgPt2_1_uniform, imgPt2_2_uniform;
			imgPt1_1_uniform=tex_points[tex_faces[f1][f1_compoint_pos[0]]];
			imgPt2_1_uniform=tex_points[tex_faces[f1][f1_compoint_pos[1]]];
			imgPt1_2_uniform=tex_points[tex_faces[f2][f2_compoint_pos[0]]];
			imgPt2_2_uniform=tex_points[tex_faces[f2][f2_compoint_pos[1]]];

			imgPt1_1.X()=(tex_images1.cols)*imgPt1_1_uniform[0];
			imgPt1_1.Y()=(tex_images1.rows)*(1-imgPt1_1_uniform[1]);

			imgPt2_1.X()=(tex_images1.cols)*imgPt2_1_uniform[0];
			imgPt2_1.Y()=(tex_images1.rows)*(1-imgPt2_1_uniform[1]);

			imgPt1_2.X()=(tex_images2.cols)*imgPt1_2_uniform[0];
			imgPt1_2.Y()=(tex_images2.rows)*(1-imgPt1_2_uniform[1]);

			imgPt2_2.X()=(tex_images2.cols)*imgPt2_2_uniform[0];
			imgPt2_2.Y()=(tex_images2.rows)*(1-imgPt2_2_uniform[1]);

			double samples=MAX(((imgPt1_1-imgPt2_1).Length()+(imgPt1_2-imgPt2_2).Length())/2, 2.0);
			for(int k = 0; k <= samples; ++k)
			{
				Wml::Vector2f pt1, pt2;
				pt1 = imgPt1_1 * (k / samples) + imgPt2_1 * ((samples - k) / samples);
				pt2 = imgPt1_2 * (k / samples) + imgPt2_2 * ((samples - k) / samples);
				{
					int x1 = pt1.X() + 0.5;
					int y1 = pt1.Y() + 0.5;
					int x2 = pt2.X() + 0.5;
					int y2 = pt2.Y() + 0.5;

					int iW1 = tex_images1.cols;
					int iH1 = tex_images1.rows;
					int iW2 = tex_images2.cols;
					int iH2 = tex_images2.rows;

					if(x1>=1 && x1<=iW1-2 && y1>=1 && y1<=iH1-2 && x2>=1 && x2<=iW2-2 && y2>=1 && y2<=iH2-2)
					{
						Wml::GMatrixf A(8,1),ATA(8,8);
						Wml::Vector2<int> p1_00,p1_01,p1_10,p1_11;
						float  a_00,a_01,a_10,a_11;
						float dx1,dy1;
						Wml::Vector2<int> p2_00,p2_01,p2_10,p2_11;
						float  b_00,b_01,b_10,b_11;
						float dx2,dy2;

						p1_00.X()= pt1.X(); p1_00.Y() = pt1.Y();
						p1_01.X()= pt1.X(); p1_01.Y() = pt1.Y()+1;
						p1_10.X()= pt1.X()+1; p1_10.Y() = pt1.Y();
						p1_11.X()= pt1.X()+1; p1_11.Y()= pt1.Y()+1;
						dx1 = pt1.X() - p1_00.X();
						dy1 = pt1.Y() - p1_00.Y();
						a_00 = (1-dx1) * (1-dy1);
						a_01 = (1-dx1) * dy1;
						a_10 = dx1 * (1-dy1);
						a_11 = dx1 * dy1;
						//The new feature: when dramatic change happened to the pixels in the seams, we do not diffuse the color\

						cv::Vec3f p1_color(0, 0, 0);
						cv::Vec3b c=tex_images1(p1_00.Y(), p1_00.X()); 
						p1_color+=a_00*cv::Vec3f(c[0], c[1], c[2]);
						c=tex_images1(p1_01.Y(), p1_01.X()); 
						p1_color+=a_01*cv::Vec3f(c[0], c[1], c[2]);
						c=tex_images1(p1_10.Y(), p1_10.X()); 
						p1_color+=a_10*cv::Vec3f(c[0], c[1], c[2]);
						c=tex_images1(p1_11.Y(), p1_11.X()); 
						p1_color+=a_11*cv::Vec3f(c[0], c[1], c[2]);

						p2_00.X() = pt2.X(); p2_00.Y()= pt2.Y();
						p2_01.X()= pt2.X(); p2_01.Y()= pt2.Y()+1;
						p2_10.X()= pt2.X()+1; p2_10.Y() = pt2.Y();
						p2_11.X()= pt2.X()+1; p2_11.Y()= pt2.Y()+1;
						dx2 = pt2.X() - p2_00.X();
						dy2 = pt2.Y() - p2_00.Y();
						b_00 = (1-dx2) * (1-dy2);
						b_01 = (1-dx2) * dy2;
						b_10 = dx2 * (1-dy2);
						b_11 = dx2 * dy2;

						cv::Vec3f p2_color(0, 0, 0);
						c=tex_images2(p2_00.Y(), p2_00.X()); 
						p2_color+=b_00*cv::Vec3f(c[0], c[1], c[2]);
						c=tex_images2(p2_01.Y(), p2_01.X()); 
						p2_color+=b_01*cv::Vec3f(c[0], c[1], c[2]);
						c=tex_images2(p2_10.Y(), p2_10.X()); 
						p2_color+=b_10*cv::Vec3f(c[0], c[1], c[2]);
						c=tex_images2(p2_11.Y(), p2_11.X()); 
						p2_color+=b_11*cv::Vec3f(c[0], c[1], c[2]);

						//if((p1_color[0]+p1_color[1]+p1_color[2])/3.0-(p2_color[0]+p2_color[1]+p2_color[2])/3.0>50) continue;

						A(0,0) = a_00, A(1,0) = a_01, A(2,0) = a_10, A(3,0) = a_11;
						A(4,0) = -b_00, A(5,0) = -b_01, A(6,0) = -b_10, A(7,0) = -b_11;

						ATA = A * A.Transpose();

						std::vector<int> local_inds(8);
						local_inds[0] =index_img1.at(p1_00.X(), p1_00.Y());
						local_inds[1] =index_img1.at(p1_01.X(), p1_01.Y());
						local_inds[2] =index_img1.at(p1_10.X(), p1_10.Y());
						local_inds[3] =index_img1.at(p1_11.X(), p1_11.Y());

						local_inds[4] =index_img2.at(p2_00.X(), p2_00.Y());
						local_inds[5] =index_img2.at(p2_01.X(), p2_01.Y());
						local_inds[6] =index_img2.at(p2_10.X(), p2_10.Y());
						local_inds[7] =index_img2.at(p2_11.X(), p2_11.Y());
						
						for(int i=0;i<8;++i)
						{
#ifdef GPU_VERSION
							for(int j=0; j<8; ++j)
#else
							for(int j=0; j<=i; ++j)
#endif
							{
								if(local_inds[i]<0||local_inds[j]<0)
								{
									printf("%f,%f,%f,%f---%d,%d,%d,%d\n",pt1.X(),pt1.Y(),pt2.X(),pt2.Y(),var_index_images[0].GetWidth(),var_index_images[0].GetHeight(),
										var_index_images[1].GetWidth(),var_index_images[1].GetHeight());
									printf("%d,%d\n",local_inds[i],local_inds[j]);
								}
								int i_temp=local_inds[i], j_temp=local_inds[j];
#ifndef GPU_VERSION
								if(i_temp<j_temp) std::swap(i_temp, j_temp);
#endif
								mtx.coeffRef(i_temp, j_temp)+=ATA(i,j)*2;
							}
						}
						//for test
						//for(int i=0;i<8;++i)
						//{
						//	for(int j=0; j<8; ++j)
						//	{
						//		if(local_inds[i]<0||local_inds[j]<0)
						//		{
						//			printf("%f,%f,%f,%f---%d,%d,%d,%d\n",pt1.X(),pt1.Y(),pt2.X(),pt2.Y(),var_index_images[0].GetWidth(),var_index_images[0].GetHeight(),
						//				var_index_images[1].GetWidth(),var_index_images[1].GetHeight());
						//			printf("%d,%d\n",local_inds[i],local_inds[j]);
						//		}
						//		//if(i!=j)
						//		_mtx.Element_Add(local_inds[i],local_inds[j],ATA(i,j)*2);
						//	}
						//}
					}
				}
			}
		}
		
	}
	void PoissonFusion::AddCommonPointsPriors(const std::vector<std::pair<int, std::vector<int>>>& sub_link_map)
	{
		std::map<Wml::Vector2<int>, std::vector<Wml::Vector2<int>>>& tex_seams=objdata->tex_seams;
		std::map<Wml::Vector2<int>, std::vector<Wml::Vector2<int>>>::iterator r;
		//遍历所有存在的缝
		for (int i=0; i<sub_link_map.size(); ++i)
		{
			int cur_roiid=sub_link_map[i].first;
			const std::vector<int>& cur_neighbors=sub_link_map[i].second;
			for (int j=0; j<cur_neighbors.size(); ++j)
			{
				if(cur_neighbors[j]<i) continue;
				int cur_neighbor_roiid=sub_link_map[cur_neighbors[j]].first;
				int cur_roiid_temp=cur_roiid; int cur_index_temp=i;
				int cur_neighbor_roiid_temp=cur_neighbor_roiid; int neighbor_index_temp=cur_neighbors[j];
				if(cur_roiid_temp>cur_neighbor_roiid_temp) {std::swap(cur_roiid_temp, cur_neighbor_roiid_temp); std::swap(cur_index_temp, neighbor_index_temp);}
				AddCommonPointsPriorsForTwo(TexImgMptr->tex_components[cur_roiid_temp], TexImgMptr->tex_components[cur_neighbor_roiid_temp], tex_seams[Wml::Vector2<int>(cur_roiid_temp, cur_neighbor_roiid_temp)], var_index_images[cur_index_temp], var_index_images[neighbor_index_temp]);
			}
		}
	}
	void PoissonFusion::poisson_fusion(std::vector<std::pair<int, std::vector<int>>>& sub_link_map, int max_iters)
	{
		//data initialization
		printf("poisson_fusion start !!\n");
		WANG::ShowCurGPUMemory();
#ifdef GPU_VERSION
		//cuda context initialization
		cublasHandle_t cublashandle;
		cusparseHandle_t cusparsehandle;
		cublasCreate(&cublashandle);
		cusparseCreate(&cusparsehandle);
#endif

		const std::vector<Wml::Vector4<int>>& tex_rois=objdata->tex_rois;
		//printf("handled_tex_ids size is %d!!!!\n", handled_tex_ids.size());
		if(sub_link_map.size()<=1) 
		{
			printf("sub_link_map size is 1!!\n");
			return;
		}
		iVarCount=0;
		for(int i=0; i<var_index_images.size(); ++i) var_index_images[i].Clear();
		var_index_images.clear();
		var_index_images.resize(sub_link_map.size());

		//X.SetSize(0); B.SetSize(0);
		//_mtx.ClearData();
		printf("Create index imgs!!\n");
		for(int i=0; i<sub_link_map.size(); ++i)
		{
			int cur_roiid=sub_link_map[i].first;
			const Wml::Vector4<int>& cur_roi=tex_rois[cur_roiid];
			var_index_images[i].Create(cur_roi[2]-cur_roi[0], cur_roi[3]-cur_roi[1]);
			//add index
			for(int y=0; y<var_index_images[i].GetHeight(); ++y)
				for (int x=0; x<var_index_images[i].GetWidth(); ++x)
					var_index_images[i].at(x, y)=iVarCount++;
		}
		printf("create index img end\n");
		std::cout<<"Var count: "<<iVarCount<<std::endl;
		//X.SetSize(iVarCount); B.SetSize(iVarCount);
		X.resize(iVarCount); B.resize(iVarCount);
		mtx.resize(iVarCount, iVarCount);
		//_mtx.Create(iVarCount, iVarCount, 0);
		//
		//getchar();
		cout<<"Start ReserveRoomForSparseMtx!"<<endl;
		ReserveRoomForSparseMtx(sub_link_map);
		cout<<"finish ReserveRoomForSparseMtx!"<<endl;
		//getchar();
		////start building the linear equation
		int iters=max_iters;//0.0003*iVarCount;
		iters=MAX(500, iters);
		PRINT(iters);
#ifdef GPU_VERSION
		Eigen::ConjugateGradient<Eigen::SparseMatrix<float, Eigen::RowMajor>> cg;
		WANG::ConjugateGradientsPrecond GPUcg(cusparsehandle, cublashandle);
		GPUcg.SetMaxIters(iters);
#else
		Eigen::ConjugateGradient<Eigen::SparseMatrix<float>> cg;
#endif
		cg.setMaxIterations(iters);
		bool enough_memory;
		for(int c=0; c<3; ++c)
		{
			for(int i = 0; i < iVarCount; i++)
				B[i] = 0;
			AddSpatialPriors(sub_link_map, c);
			//WANG_DEBUG_OUT;
			if(c==0)
			{
				//cout<<"ohh"<<endl;
				AddCommonPointsPriors(sub_link_map);
				mtx.makeCompressed();
				cout<<"The nnz is "<<mtx.nonZeros()<<" The average number of neighbor are "<< (double)mtx.nonZeros()/mtx.rows()<<endl;
				cg.compute(mtx);
#ifdef GPU_VERSION
				cout<<"SetGPUdata"<<endl;
				//ShowCurGPUMemory();
				enough_memory=GPUcg.SetDate(mtx);
#endif
				//ShowCurGPUMemory();
				//getchar();
				//A=EigenSparse2CCS(mtx);
			}
#ifdef GPU_VERSION
			if(enough_memory)
			{
				WANG::ShowCurGPUMemory();
				Eigen::VectorXf solution;
				enough_memory=GPUcg.Solve(X, B, solution);
				//WANG::ShowCurGPUMemory();
				if(enough_memory)
					X=solution;
			}
			if(!enough_memory)
			{
				WANG::ShowCurGPUMemory();
				cout<<"Switch to CPU Solver!!"<<endl;
				
			}
#else
			X=cg.solveWithGuess(B, X);
#endif
			
			//WANG_DEBUG_OUT;
			/*if(ConjugateGradients(A, X, B, 500, 1e-15) != CCS_SUCCESS)
				printf("Linear Matrix No Solution!\n");*/
			//WANG_DEBUG_OUT;
			//write result
			for (int i=0; i<sub_link_map.size(); ++i)
			{
				int cur_roiid=sub_link_map[i].first;
				cv::Mat3b& cur_component=TexImgMptr->tex_components[cur_roiid];
				ZIntImage& cur_index_img=var_index_images[i];
				if(cur_component.empty()) {printf("write result error: cur_componts is empty!!\n"); continue;};
				for (int r=0; r<cur_component.rows; ++r)
				{
					cv::Vec3b* cur_component_ptr=cur_component[r];
					for(int col=0; col<cur_component.cols; ++col)
						cur_component_ptr[col][c]=MIN(255.0, MAX(X[cur_index_img.at(col, r)], 0.0));
				}
	    	}
			printf("Solve OK!\n");
		}
		//delete mtx
		mtx.resize(0, 0);
		printf("poisson_fusion end!!\n");
		//destory cuda context
#ifdef GPU_VERSION
		if(enough_memory)
		{
			cublasDestroy(cublashandle);
			cusparseDestroy(cusparsehandle);
		}
#endif
		//getchar();
	}
	/*CCSMatrix* EigenSparse2CCS(Eigen::SparseMatrix<float>& mtx)
	{
		CCSMatrix* ccs=new CCSMatrix;
		ccs->n=mtx.outerSize();
		ccs->m=mtx.innerSize();
		ccs->flags=CCS_SYMMETRIC|CCS_LOWER|CCS_DOUBLE;
		ccs->colptr=mtx.outerIndexPtr();
		ccs->rowind=mtx.innerIndexPtr();
		ccs->values=mtx.valuePtr();
		return ccs;	
	}*/
}
