#include "stdafx.h"
#include "ImproveTM.h"
#include <deque>
namespace WANG
{
	float RangeNode::r=0.05;
	void AdaptiveSample(std::vector<std::pair<float, int>>& origindata, std::vector<std::pair<float, int>>& newdata)
	{
		std::set<RangeNode> sort_set;
		for(int i=0; i<origindata.size(); ++i)
			sort_set.insert(origindata[i]);
		//printf("Before sample: %d After sample: %d\n", origindata.size(), sort_set.size());
		newdata.clear();
		std::set<RangeNode>::iterator iter=sort_set.begin();
		for(; iter!=sort_set.end(); ++iter) newdata.push_back(iter->data);
		// 将数据写出
		/*FILE* origindata_file=fopen("origindata.txt", "w");
		FILE* newdata_file=fopen("newdata.txt", "w");
		for(int i=0; i<origindata.size(); ++i) fprintf(origindata_file, "%d %f\n", origindata[i].second, origindata[i].first);
		for(int i=0; i<newdata.size(); ++i) fprintf(newdata_file, "%d %f\n", newdata[i].second, newdata[i].first);
		fclose(origindata_file); fclose(newdata_file);*/
	}
}
void smoothNrmVec(std::vector<Wml::Vector3f> srcNrmVec, std::vector<Wml::Vector3f>& trgNrmVec, const std::vector<std::vector<int>> link_map, int _R)
{
	trgNrmVec.resize(srcNrmVec.size());
	std::vector<Wml::Vector3f> srcNrmVec_tmp=srcNrmVec;
	//辅助数据
	//std::vector<int> lastvisited;
#pragma omp parallel for num_threads(omp_get_num_procs()-1)//, firstprivate(link_map, srcNrmVec_tmp, _R)
	for (int i=0; i<srcNrmVec_tmp.size(); ++i)
	{
		int curlay_num;
		int sum_num;
		Wml::Vector3f meanNrm(0, 0, 0);
		//初始化
#define DEQUESIZE 100
		int q[DEQUESIZE];
		int q_head, q_tail;
		bool q_isfull=false;
		q_head=q_tail=0;
		std::set<int> is_visited;
		q[q_tail++]=i;
		//q.push_back(i);
		is_visited.insert(i);
		meanNrm=meanNrm+srcNrmVec_tmp[i];
		curlay_num=1;
		sum_num=1;
		for (int j=0; j<_R; ++j)
		{
			if(q_isfull)
				break;
			int temp_curlay_num=0;
			for (int k=0; k<curlay_num; ++k)
			{
				if(q_isfull) break;
				int h=q[q_head];
				q_head=(q_head+1)%DEQUESIZE;
				//int h=q.front();
				//q.pop_front();
				const std::vector<int>& cur_neighors=link_map[h+OBJOFFSET];
				const int* cur_neighors_ptr=&cur_neighors[0];
				int neigbors_num=cur_neighors.size();
				for(int n_iter=0; n_iter<neigbors_num; ++n_iter)
				{
					int t=cur_neighors_ptr[n_iter];
					if(is_visited.insert(t).second)
					{
						temp_curlay_num++;
						//sum_num++;
						meanNrm=meanNrm+srcNrmVec_tmp[t];
						q[q_tail]=t;
						q_tail=(q_tail+1)%DEQUESIZE;
						sum_num++;
						if(q_head==q_tail)
						{
							q_isfull=true;
							break;
						}
						//q.push_back(t);
					}
				}
			}
			curlay_num=temp_curlay_num;
		}
		meanNrm=meanNrm/sum_num;
		trgNrmVec[i]=meanNrm;
	}
}
