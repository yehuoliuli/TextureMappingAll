#include "stdafx.h"
#include "TexturePackage.h"
#include "Wang_Helper.h"
namespace WANG
{
	std::string TexturePackage(ObjInterface& objdata, cv::Mat3b& new_textureimg, std::string new_teximgname)
	{
		FUNCTION_START;
		//cnmb
		objdata.placements.clear();
		objdata.placements.resize(objdata.tex_filenames.size());
		//acess necessary data
		std::vector<Wml::Vector3i>& faces=objdata.faces;
		std::vector<Wml::Vector2f>& tex_points=objdata.tex_points;
		std::vector<int>& tex_id=objdata.tex_id;
		//cnmb
		objdata.original_tex_id=tex_id;
		std::vector<std::string>& tex_filenames=objdata.tex_filenames;
		std::vector<Wml::Vector3i>& tex_faces=objdata.tex_faces;
		std::set<int>& invalid_texid=objdata.invalid_texid;
		objdata.build_teximg_faces();
		std::vector<std::vector<int>>& teximg_faces=objdata.teximg_faces;
		//newdata
		std::vector<Wml::Vector2f> new_tex_points;
		std::vector<int> new_tex_id(faces.size(), 1);
		for(int f=0; f<faces.size(); ++f)
		{
			if(invalid_texid.find(tex_id[f])!=invalid_texid.end())
				new_tex_id[f]=0;
		}
		std::vector<std::string> new_tex_filenames(2, "");
		new_tex_filenames[1]=new_teximgname;
		std::vector<Wml::Vector3i> new_tex_faces=objdata.tex_faces;
		//PRINT(objdata.tex_faces[129175]);
		std::set<int> new_invalid_texid; new_invalid_texid.insert(0);
		//
		std::vector<int> tex2patch(tex_filenames.size(), -1);
		std::vector<cv::Size> patches;
		std::vector<Placement> results;
		cv::Size outsize;
		int patchesnum=0;
		for (int t=0; t<tex_filenames.size(); ++t)
		{
			cv::Mat3b temp=objdata.LoadTex(t);
			//PRINT(objdata.loadfilepath+objdata.tex_filenames[t]);
			//getchar();
			if(!temp.empty())
			{
				patches.push_back(temp.size());
				tex2patch[t]=patchesnum;
				++patchesnum;
			}
		}
		//exit(0);
		PackPatches(patches, outsize, results, 1);
		//test
		{
			cv::Mat3b canvas;
			DrawLayout(patches, outsize, results, canvas);
			cv::imwrite("_layoutimg.png", canvas);
			cout<<"_layoutimg.png is saved successfully!"<<endl;
			//getchar();
		}
		int texpoints_num=0;
		for (int t=0; t<teximg_faces.size(); ++t)
		{
			std::vector<int>& cur_faces=teximg_faces[t];
			int cur_patch_ind=tex2patch[t];
			if(cur_patch_ind==-1) continue;
			cv::Size& cur_patch=patches[cur_patch_ind];
			Placement cur_placement=results[cur_patch_ind];
			objdata.placements[t]=cur_placement;
			/*if(cur_placement.rotate90)
				std::swap(cur_patch.height, cur_patch.width);
			float ax, bx;
			float ay, by;
			ax=((float)cur_patch.width)/outsize.width;
			bx=((float)cur_placement.tl.x)/outsize.width;

			ay=((float)cur_patch.height)/outsize.height;
			by=((float)cur_placement.tl.y)/outsize.height;*/
			for(int i=0; i<cur_faces.size(); ++i)
			{
				int cur_f=cur_faces[i];
				Wml::Vector3i& cur_texf=tex_faces[cur_f];
				for(int p=0; p<3; p++)
				{
					Wml::Vector2f cur_texp=tex_points[cur_texf[p]];
					cur_texp.X()=cur_texp.X()*cur_patch.width;
					cur_texp.Y()=(1-cur_texp.Y())*cur_patch.height;
					if(cur_placement.rotate90)
						std::swap(cur_texp.X(), cur_texp.Y());
					Wml::Vector2f new_texp;
					new_texp.X()=(cur_texp.X()+cur_placement.tl.x)/outsize.width;
					new_texp.Y()=1-(cur_texp.Y()+cur_placement.tl.y)/outsize.height;
					new_tex_points.push_back(new_texp);
					new_tex_faces[cur_f][p]=texpoints_num;
					++texpoints_num;
				}
			}
		}
		tex_id.swap(new_tex_id);				RELEASE_VECTOR(new_tex_id);
		tex_points.swap(new_tex_points);		RELEASE_VECTOR(new_tex_points);
		invalid_texid.swap(new_invalid_texid);  new_invalid_texid.clear();
		tex_faces.swap(new_tex_faces);			RELEASE_VECTOR(new_tex_faces);
		//PRINT(tex_faces[129175]);
		CompressTexPoints(objdata);		
		//create new texture
		new_textureimg.create(outsize);
		new_textureimg.setTo(cv::Vec3b(0, 0, 0));
		for (int t=0; t<tex_filenames.size(); ++t)
		{
			cv::Mat3b temp=objdata.LoadTex(t);
			if(!temp.empty())
			{
				int cur_patch_ind=tex2patch[t];
				Placement cur_placement=results[cur_patch_ind];
				//PRINT(cur_placement.rotate90);
				if(cur_placement.rotate90)
					cv::transpose(temp, temp);
				cv::Rect cur_roi(cur_placement.tl, temp.size());
				//PRINT(cur_roi);
				//PRINT(patches[cur_patch_ind]);
				//PRINT(outsize);
				temp.copyTo(cv::Mat3b(new_textureimg, cur_roi));
			}
		}
		tex_filenames.swap(new_tex_filenames);	RELEASE_VECTOR(new_tex_filenames);
		FUNCTION_END;
		return new_teximgname;
	}
}
