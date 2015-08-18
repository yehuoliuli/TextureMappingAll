#include "stdafx.h"
#include "wang_simplecompletion.h"
#define ISVALIDP(x, y, rows, cols) (((x)>=0)&&((x)<(cols))&&((y)>=0)&&((y)<(rows)))
static Wml::Vector2<float> neighbor_offset[5]={Wml::Vector2<float>(0, 1), Wml::Vector2<float>(1, 0), Wml::Vector2<float>(0, -1), Wml::Vector2<float>(-1, 0), Wml::Vector2<float>(0, 0)};
namespace WANG
{
	//将源图像的三角形copy到目标图像 并且在mask记录重叠区域的重叠数量 即边界区域的像素加上一个权重 所有在同一个边界上的像素进行平均
	void copy_triangle(const cv::Mat3b& src_img, cv::Mat3f& tar_img, cv::Mat1b& mask, std::vector<Wml::Vector2f>& src_triangle, std::vector<Wml::Vector2f>& tar_triangle, 
		cv::Mat1i& label_msk, int label)
	{
		//FILE* copy_trianle_errorlog=fopen("copy_trianle_errorlog.txt", "w");
		assert(tar_img.size()==mask.size());
		//std::cout<<tar_img.size()<<" "<<src_img.size()<<std::endl;
		if(tar_img.empty()||mask.empty()) fprintf(stderr, "tar_img is empty!!\n");
		if(src_triangle.size()<3||tar_triangle.size()<3) fprintf(stderr, "triangle size is not 3!!\n");
		//将0-1坐标转换到离散图像中去
		std::vector<Wml::Vector2f> src_img_triangle(3), tar_img_triangle(3);
		for (int i=0; i<3; ++i)
		{
			src_img_triangle[i].X()=src_triangle[i][0]*(src_img.cols-1);
			src_img_triangle[i].Y()=(1-src_triangle[i][1])*(src_img.rows-1);
			tar_img_triangle[i].X()=tar_triangle[i][0]*(tar_img.cols-1);
			tar_img_triangle[i].Y()=(1-tar_triangle[i][1])*(tar_img.rows-1);
		}
		/*printf("src: [%f %f], [%f %f], [%f %f] tar: [%f, %f] [%f, %f] [%f %f]\n", 
									   src_img_triangle[0].X(), src_img_triangle[0].Y(),
									   src_img_triangle[1].X(), src_img_triangle[1].Y(),
									   src_img_triangle[2].X(), src_img_triangle[2].Y(),
									   tar_img_triangle[0].X(), tar_img_triangle[0].Y(),
									   tar_img_triangle[1].X(), tar_img_triangle[1].Y(),
									   tar_img_triangle[2].X(), tar_img_triangle[2].Y());*/
		//计算仿射矩阵 
		Wml::Matrix3f src_m, tar_m;
		for(int i=0; i<3; ++i)
		{
			src_m.SetColumn(i, Wml::Vector3f(src_img_triangle[i].X(), src_img_triangle[i].Y(), 1));
			tar_m.SetColumn(i, Wml::Vector3f(tar_img_triangle[i].X(), tar_img_triangle[i].Y(), 1));
		}
		//计算从目标图像到原始图像的转移矩阵
		Wml::Matrix3f transform_tarTosrc=src_m*tar_m.Inverse();
		//统计bounding box
		float min_x, min_y;
		float max_x, max_y;
		//bug a 
	    min_x=min_y=tar_img.cols+10;
		max_x=max_y=-1;
		for(int i=0; i<3; ++i)
		{
			min_x=std::min(min_x, tar_img_triangle[i].X());
			min_y=std::min(min_y, tar_img_triangle[i].Y());
			max_x=std::max(max_x, tar_img_triangle[i].X());
			max_y=std::max(max_y, tar_img_triangle[i].Y());
		}
		/*int min_x_temp=min_x, min_y_temp=min_y;
		int max_x_temp=max_x+1, max_y_temp=max_y+1;
		*/
		Wml::Vector2<float> offset(min_x, min_y);
		//int roi_width=std::min(max_x_temp, src_img.cols)-min_x_temp;
		//int roi_height=std::min(max_y_temp, src_img.rows)-min_y_temp;
		cv::Rect roi((int)min_x, (int)min_y, (int)(max_x-min_x)+2, (int)(max_y-min_y)+2 );
		roi=roi&(cv::Rect(0, 0, tar_img.cols, tar_img.rows));
		if(roi.height==0&&roi.width==0) fprintf(stderr, "invalid roi!!\n");
		cv::Mat3f tar_img_roi(tar_img, roi);
		cv::Mat1b mask_roi(mask, roi);
		cv::Mat1i label_msk_roi;
		if(!label_msk.empty())
			label_msk_roi=label_msk(roi);
		for (int i=0; i<tar_img_roi.rows; ++i)
		{
			cv::Vec3f* row_ptr=tar_img_roi[i];
			uchar* mask_ptr=mask_roi[i];
			int* label_msk_ptr=NULL;
			if(!label_msk_roi.empty())
				label_msk_ptr=label_msk_roi[i];
			//当存在在三角形内部的点的时候就对这个像素赋值
			for (int j=0; j<tar_img_roi.cols; ++j)
			{
				//查找该点的四个邻域的点
				Wml::Vector2<float> cur_point(j, i);
				cur_point=cur_point+offset;
				float u, v;
				bool flag1=false;
				for(int k=0; k<5; ++k)
				{
					if(point_in_triangle<2>(cur_point+neighbor_offset[k], tar_img_triangle[0], tar_img_triangle[1], tar_img_triangle[2], u, v))
						flag1=true;
				}
				if(!flag1) continue;
				if(label_msk_ptr!=NULL)
				{
					if(label_msk_ptr[j]!=label)
						if(label_msk_ptr[j]==-1)
							label_msk_ptr[j]=label;
						else
							label_msk_ptr[j]+=label;
				}
				Wml::Vector3f tar_p(cur_point.X(), cur_point.Y(), 1);
				Wml::Vector3f src_p=transform_tarTosrc*tar_p;
				//双线性插值
				//if(src_p.Z()==0) printf("src_p.z()==0!!\n");
				float x=src_p.X()/src_p.Z(), y=src_p.Y()/src_p.Z();
				if(!ISVALIDP(x, y, src_img.rows, src_img.cols)) continue;
				//printf("Z: %f x: %f y: %f u+v: %f\n", src_p.Z(), x, y, u+v);
				//fprintf(copy_trianle_errorlog, "Z: %f x: %f y: %f u+v: %f\n", src_p.Z(), x, y, u+v);
				int lx=x, ly=y, rx=std::min(src_img.cols-1, lx+1), ry=std::min(src_img.rows-1, ly+1);
				float wx=x-lx;
				float wy=y-ly;

				cv::Vec3b c00=src_img(ly, lx);
				cv::Vec3b c01=src_img(ry, lx);
				cv::Vec3b c10=src_img(ly, rx);
				cv::Vec3b c11=src_img(ry, rx);
				cv::Vec3f meanc(0, 0, 0);
				for(int k=0; k<3; ++k)
					meanc[k]=(1-wx)*(1-wy)*c00[k]+(1-wx)*wy*c01[k]+wx*(1-wy)*c10[k]+wx*wy*c11[k];
				++mask_ptr[j];
				//Debug
				//meanc=cv::Vec3f(255, 255, 255);
				row_ptr[j]=meanc+row_ptr[j];
			}
		}
		//fclose(copy_trianle_errorlog);
	}
	void fill_triangle(const cv::Mat1b& mask, std::vector<Wml::Vector2f>& tar_triangle)
	{
		//将0-1坐标转换到离散图像中去
		std::vector<Wml::Vector2f> tar_img_triangle(3);
		for (int i=0; i<3; ++i)
		{
			tar_img_triangle[i].X()=tar_triangle[i][0]*(mask.cols-1);
			tar_img_triangle[i].Y()=(1-tar_triangle[i][1])*(mask.rows-1);
		}
		float min_x, min_y;
		float max_x, max_y;
	    min_x=min_y=mask.cols+10;
		max_x=max_y=-1;
		for(int i=0; i<3; ++i)
		{
			min_x=std::min(min_x, tar_img_triangle[i].X());
			min_y=std::min(min_y, tar_img_triangle[i].Y());
			max_x=std::max(max_x, tar_img_triangle[i].X());
			max_y=std::max(max_y, tar_img_triangle[i].Y());
		}
		/*int min_x_temp=min_x, min_y_temp=min_y;
		int max_x_temp=max_x+1, max_y_temp=max_y+1;
		*/
		Wml::Vector2<float> offset(min_x, min_y);
		//int roi_width=std::min(max_x_temp, src_img.cols)-min_x_temp;
		//int roi_height=std::min(max_y_temp, src_img.rows)-min_y_temp;
		cv::Rect roi((int)min_x, (int)min_y, (int)(max_x-min_x)+2, (int)(max_y-min_y)+2 );
		roi=roi&(cv::Rect(0, 0, mask.cols, mask.rows));
		if(roi.height==0&&roi.width==0) fprintf(stderr, "invalid roi!!\n");
		cv::Mat1b mask_roi(mask, roi);
		for (int i=0; i<mask_roi.rows; ++i)
		{
			uchar* mask_ptr=mask_roi[i];
			//当存在在三角形内部的点的时候就对这个像素赋值
			bool flag1=false;
			for (int j=0; j<mask_roi.cols; ++j)
			{
				//查找该点的四个邻域的点
				Wml::Vector2<float> cur_point(j, i);
				cur_point=cur_point+offset;
				float u, v;
				for(int k=0; k<5; ++k)
				{
					if(point_in_triangle<2>(cur_point+neighbor_offset[k], tar_img_triangle[0], tar_img_triangle[1], tar_img_triangle[2], u, v))
						flag1=true;
				}
				if(!flag1) continue;
				mask_ptr[j]=0;
			}
		}
	}
	void average(cv::Mat3b &A, cv::Mat1b mask, cv::Mat1b &mask1, int patch_w)
	{
		assert(A.size()==mask1.size()&&A.size()==mask.size());
		uchar* mask_ptr;
		int rows=A.rows,cols=A.cols;
		for (int i=0;i<rows;i++)
		{
			mask_ptr=mask[i];
			for (int j=0;j<cols;j++)
			{
				if(!mask_ptr[j])
					continue;
				else
				{
					int li=i-patch_w,lj=j-patch_w,ri=i+patch_w,rj=j+patch_w;
					li=MAX(0,li),lj=MAX(0,lj),ri=MIN(rows-1,ri),rj=MIN(cols-1,rj);
					int n=0; cv::Vec3i sum(0,0,0);
					for (int i1=li;i1<=ri;i1++)
					{
						for (int j1=lj;j1<=rj;j1++)
						{
							if(!mask1[i1][j1])
								continue;
							else
							{
								if(A[i1][j1]==cv::Vec3b(0, 0, 0)) continue;
								sum=sum+(cv::Vec3i)A[i1][j1],n++;
								//cout<<(Vec3i)A[i1][j1]<<endl;
								//cout<<sum<<endl;
							}
						}
					}
					A[i][j]=sum*(1.0/n);
				}
			}
		}
	}
	void simplecompletion(cv::Mat3b& img,const cv::Mat1b& _mask, int patch_size)
	{
		cv::Mat1b mask=_mask.clone();
		while(1)
		{
			int flag=false;
			bool has_white=false;
			for (int i=0; i<mask.rows; ++i)
			{
				uchar* ptr= mask[i];
				for(int j=0; j<mask.cols; ++j)
				{
					if(ptr[j]==0)
					{
						flag=true;
						//break;
					}
					else
						has_white=true;
					if(flag)
						break;
				}
			}
			if(!flag||!has_white) break;
			cv::Mat1b mask2;
			cv::dilate(mask, mask2, cv::Mat());
			//imshow("", img);
			average(img, mask2-mask, mask, patch_size);
			//cout<<i<<endl;
			mask=mask2;
			//cv::imshow("", img);
			//cv::waitKey(0);
		}
		//cv::medianBlur(img, img, 5);
	}
}