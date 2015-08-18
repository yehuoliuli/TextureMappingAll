#include "stdafx.h"
#include "wang_Create_newtexture.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <his\Foreach.hpp>
#include <algorithm>
namespace WANG
{
	void CreateNewTexture(std::vector<std::vector<int>>& TexFrmTriAglMap, std::vector<Wml::Vector2f>& tex_points, std::vector<Wml::Vector3i>& tex_faces, std::vector<Wml::Vector2f>& uv_points, std::vector<Wml::Vector3i>& faces_uv_indexs, std::string loadfilepath, std::string savefilepath, std::vector<std::string>& tex_filenames, int TextureSize)
	{
		//将uv points打印出来
		FILE* uvpointsdata=fopen("uvpointdata.txt", "w");
		printf("write uv points data\n");
		fprintf(uvpointsdata,"%d\n", uv_points.size());
		for(int i=0; i<uv_points.size(); ++i)
			fprintf(uvpointsdata, "%.4f %.4f\n", uv_points[i].X(), uv_points[i].Y());
		//			fprintf(uvpointsdata, "[ %.4f %.4f ]%c", uv_points[i].X(), uv_points[i].Y(), (i+1)%10?' ':'\n');
		fclose(uvpointsdata);
		printf("write uv points data end!!\n");
		printf("Parameterization start!!\n");
		//cv::Mat3f newtexture=cvCreateImage(cvSize(TEXTURESIZE, TEXTURESIZE), IPL_DEPTH_32F, 3);
		//cv::Mat1b newtexture_mask=cvCreateImage(cvSize(TEXTURESIZE, TEXTURESIZE), IPL_DEPTH_8U, 1);
		cv::Mat3f newtexture(TextureSize, TextureSize, cv::Vec3f(0, 0, 0));
		cv::Mat1b newtexture_mask(TextureSize, TextureSize, (uchar)0);
		cv::Mat1i label_msk(TextureSize, TextureSize, -1);
		//std::cout<<"caocao!!\n"<<std::endl;
		//cv::imshow("", src);
		//cv::waitKey(0);
		//这个数据结构包含了 一张纹理图所对应的三角面片
		int iTexNum=TexFrmTriAglMap.size();
		//debug
		cv::Mat3b last(TextureSize, TextureSize, cv::Vec3b(0, 0, 0));
		for(int t = 0; t < iTexNum; t++)
		{
			//载入原图
			if(tex_filenames[t].empty()) continue;
			std::cout<<(loadfilepath+tex_filenames[t])<<std::endl;
			cv::Mat3b src=cvLoadImage((loadfilepath+tex_filenames[t]).c_str());
			if(src.empty()) {printf("can not load image: %s!!\n", (loadfilepath+tex_filenames[t]).c_str()); continue;}
			std::cout<<src.rows<<" "<<src.cols<<std::endl;
			for(int f = 0; f < TexFrmTriAglMap[t].size(); f++)
			{
				int iTriAglIdx = TexFrmTriAglMap[t][f];
				std::vector<Wml::Vector2f> src_agl(3);
				std::vector<Wml::Vector2f> trg_agl(3);
				for(int j=0; j<3; ++j)
				{
					src_agl[j]=tex_points[tex_faces[iTriAglIdx][j]];
					trg_agl[j][0]=uv_points[faces_uv_indexs[iTriAglIdx][j]].X();
					trg_agl[j][1]=uv_points[faces_uv_indexs[iTriAglIdx][j]].Y();
				}
				//copy
				copy_triangle(src, newtexture, newtexture_mask, src_agl, trg_agl, label_msk, t);
			}
			
		}
		//对newtexture进行平均
		printf("average the image!!!\n");
		cv::Mat3b save_newtexture=cvCreateImage(cvSize(TextureSize, TextureSize), IPL_DEPTH_8U, 3);
		//printf("222!!!!!!!  ");
		std::cout<<newtexture.rows<<" "<<newtexture.cols<<std::endl;
		for (int r=0; r<newtexture.rows; ++r)
		{
			//printf("!!!!!!!  ");
			cv::Vec3f* fptr=newtexture[r];
			cv::Vec3b* save_newtexture_ptr=save_newtexture[r];
			uchar* newtexture_mask_ptr=newtexture_mask[r];
			for (int c=0; c<newtexture.cols; ++c)
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
		//ExtendBoundary
		Extendboundary(save_newtexture, newtexture_mask, 0.005*TextureSize);
		//保存图像
		cv::FileStorage labelf(savefilepath+"_texture1_label.yml", cv::FileStorage::WRITE);
		labelf<<"label_msk"<<label_msk;
		labelf.release();
		printf("save new_texture.jpg start!!!\n");
		//cv::imwrite("new_texture.jpg", newtexture);
		cvSaveImage((savefilepath+"_texture1.jpg").c_str(), &((IplImage)save_newtexture));
		//cvSaveImage((savefilepath+"_texture1mask.jpg").c_str(), &((IplImage)newtexture_mask));
		printf("save new_texture.jpg end!!!\n");
	}
	void Extendboundary(cv::Mat3b& src, cv::Mat1b& mask, int iter_nums/* =5 */)
	{
		//alorgrithm data
		printf("Extendboundary start!!\n");
		cv::Rect src_roi(0, 0, src.cols, src.rows);
		/*cv::Point offsets[8]={cv::Point(-1, -1), cv::Point(0, -1), cv::Point(1, -1),
					       cv::Point(-1, 0),                    cv::Point(1, 0),
						   cv::Point(-1, 1), cv::Point(0, 1), cv::Point(1, 1)};*/
		cv::Point offsets[8]={cv::Point(-1, 0), cv::Point(-1, -1), cv::Point(0, -1), cv::Point(1, -1), cv::Point(1, 0), cv::Point(1, 1), cv::Point(0, 1), cv::Point(-1, 1)};
		//initialization boundary 
		std::vector<cv::Point> boundary_pixels;
		his::for_each_withpos(mask, [&mask, &boundary_pixels, &offsets, &src_roi](uchar c, int i, int j)
									{
										cv::Point cur(j, i);
										if(c==0)
										{
											int flag=false;
											for(int i=0; i<8; ++i)
											{
												if(!src_roi.contains(cur+offsets[i])) continue;
												if(mask(cur+offsets[i]))
												{
													flag=true;
													//mask(cur)=255;
													break;
												}
											}
											if(flag) boundary_pixels.push_back(cur);
										}
									});
		for (int k=0; k<iter_nums; ++k)
		{
			
			std::vector<cv::Point> boundary_pixels_temp;
			//extend boundary pixels
			std::for_each(boundary_pixels.begin(), boundary_pixels.end(), [&mask, &offsets, &src_roi, &src](cv::Point cur)
																		  {
																			  cv::Point offsetsum(0, 0);
																			  int candidate;
																			  for (int m=0; m<8; ++m)
																			  {
																				  if(!src_roi.contains(cur+offsets[m])) continue;
																				  if(mask(cur+offsets[m]))
																				  {
																					  candidate=m;
																				      offsetsum=offsetsum+offsets[m];
																				  }
																			  }
																			  double theta=atan2((double)offsetsum.y, (double)offsetsum.x);
																			  int offsetid=(theta+M_PI)/(2*M_PI)*8;
																			  offsetid=offsetid%8;
																			  if(mask(cur+offsets[offsetid]))
																				  src(cur)=src(cur+offsets[offsetid]);
																			  else
																				  src(cur)=src(cur+offsets[candidate]);
																			});
			//updata mask
			std::for_each(boundary_pixels.begin(), boundary_pixels.end(), [&mask](cv::Point cur){mask(cur)=255;});
			//a iterative algorithm to find boundary pixels
			std::for_each(boundary_pixels.begin(), boundary_pixels.end(), [&mask, &offsets, &src_roi, &boundary_pixels_temp](cv::Point cur)
																		  {
																			  //label the new tarversed boundary pixels using 255
																			  for(int i=0; i<8; ++i)
																			  {
																				  if(!src_roi.contains(cur+offsets[i])) continue;
																				  if(!mask(cur+offsets[i]))
																				  {
																					  mask(cur+offsets[i])=255;
																					  boundary_pixels_temp.push_back(cur+offsets[i]);
																				  }
																			  }
																		  });
			boundary_pixels.swap(boundary_pixels_temp);
			//clear 255 flag
			std::for_each(boundary_pixels.begin(), boundary_pixels.end(), [&mask](cv::Point cur){mask(cur)=0;});
			
		}
		printf("Extendboundary end!!\n");
	}
}
#undef TEXTURESIZE