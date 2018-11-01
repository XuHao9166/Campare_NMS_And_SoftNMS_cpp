#include "Compare_nms.h"
#include<iostream>
#include<vector>
#include<algorithm>
using namespace std;


namespace XuHao
{
   
///////////传统的NMS
	///计算过程
	//1、根据候选框的类别分类概率做排序，假如有4个 BBox ，其置信度A>B>C>D。
	//2、先标记最大概率矩形框A是算法要保留的BBox；
	//3、从最大概率矩形框A开始，分别判断ABC与D的重叠度IOU（两框的交并比）是否大于某个设定的阈值(0.5)，假设D与A的重叠度超过阈值，那么就舍弃D；
	//4、从剩下的矩形框BC中，选择概率最大的B，标记为保留，然后判读C与B的重叠度，扔掉重叠度超过设定阈值的矩形框；
	//5、一直重复进行，标记完所有要保留下来的矩形框。

	//NMS缺点：
	//1、NMS算法中的最大问题就是它将相邻检测框的分数均强制归零(既将重叠部分大于重叠阈值Nt的检测框移除)。在这种情况下，如果一个真实物体在重叠区域出现，则将导致对该物体的检测失败并降低了算法的平均检测率（average precision, AP）。
	//2、NMS的阈值也不太容易确定，设置过小会出现误删，设置过高又容易增大误检。
	//3、NMS一般只能使用CPU计算，无法使用GPU计算。

	bool Traditinal_cmpScore(const BboxWithScore &lsh, const BboxWithScore &rsh) {
		if (lsh.score < rsh.score)
			return true;
		else
			return false;
	}

	void Traditinal_NMS(std::vector<BboxWithScore>& boundingBox_, const float overlap_threshold)
	{

		if (boundingBox_.empty()){
			return;
		}
		//对各个候选框根据score的大小进行升序排列
		sort(boundingBox_.begin(), boundingBox_.end(), Traditinal_cmpScore);
		float IOU = 0;
		float maxX = 0;
		float maxY = 0;
		float minX = 0;
		float minY = 0;
		vector<int> vPick;
		int nPick = 0;
		multimap<float, int> vScores;   //存放升序排列后的score和对应的序号
		const int num_boxes = boundingBox_.size();
		vPick.resize(num_boxes);
		for (int i = 0; i < num_boxes; ++i){
			vScores.insert(pair<float, int>(boundingBox_[i].score, i));
		}
		while (vScores.size() > 0){
			int last = vScores.rbegin()->second;  //反向迭代器，获得vScores序列的最后那个序列号
			vPick[nPick] = last;
			nPick += 1;
			for (multimap<float, int>::iterator it = vScores.begin(); it != vScores.end();){
				int it_idx = it->second;
				maxX = max(boundingBox_.at(it_idx).tx, boundingBox_.at(last).tx);
				maxY = max(boundingBox_.at(it_idx).ty, boundingBox_.at(last).ty);
				minX = min(boundingBox_.at(it_idx).bx, boundingBox_.at(last).bx);
				minY = min(boundingBox_.at(it_idx).by, boundingBox_.at(last).by);
				//转换成了两个边界框相交区域的边长
				maxX = ((minX - maxX + 1) > 0) ? (minX - maxX + 1) : 0;
				maxY = ((minY - maxY + 1) > 0) ? (minY - maxY + 1) : 0;
				//求交并比IOU

				IOU = (maxX * maxY) / (boundingBox_.at(it_idx).area + boundingBox_.at(last).area - IOU);
				if (IOU > overlap_threshold){
					it = vScores.erase(it);    //删除交并比大于阈值的候选框,erase返回删除元素的下一个元素
				}
				else{
					it++;
				}
			}
		}

		vPick.resize(nPick);
		vector<BboxWithScore> tmp_;
		tmp_.resize(nPick);
		for (int i = 0; i < nPick; i++){
			tmp_[i] = boundingBox_[vPick[i]];
		}
		boundingBox_ = tmp_;
	}

/////////////////////////////////////////////////////////////////////


///////////soft_NMS
	//softNMS优点：
	//1、SoftNMS可以很方便地引入到object detection算法中，不需要重新训练原有的模型、代码容易实现，
	//不增加计算量（计算量相比整个object detection算法可忽略）。并且很容易集成到目前所有使用NMS的目标检测算法。
	//2、soft - NMS在训练中采用传统的NMS方法，仅在推断代码中实现soft - NMS。作者应该做过对比试验，
	//在训练过程中采用soft - NMS没有显著提高。
	//3、NMS是Soft - NMS特殊形式，当得分重置函数采用二值化函数时，Soft - NMS和NMS是相同的。soft - NMS算法是一种更加通用的非最大抑制算法。

	//softNMS缺点：
	//softNMS也是一种贪心算法，并不能保证找到全局最优的检测框分数重置。除了以上这两种分数重置函数，我们也可以考虑开发其他包含更多参数的分数重置函数，比如Gompertz函数等。但是它们在完成分数重置的过程中增加了额外的参数。


	//def cpu_soft_nms(np.ndarray[float, ndim=2] boxes, float sigma=0.5, float Nt=0.3, float threshold=0.001, unsigned int method=0):
	//这是原作者给出的python版本的参数设置

	void softNms(std::vector<BboxWithScore>& bboxes,const int& method,
                               const float& sigma,const float& iou_thre,const float& threshold)
    {
        if (bboxes.empty())
        {
            return;
        }

        int N = bboxes.size();
        float max_score,max_pos,cur_pos,weight;
        BboxWithScore tmp_bbox,index_bbox;
        for (int i = 0; i < N; ++i)
        {
            max_score = bboxes[i].score;
            max_pos = i;
            tmp_bbox = bboxes[i];
            cur_pos = i + 1;

            //获得最大得分的 bbox
            while (cur_pos < N)
            {
                if (max_score < bboxes[cur_pos].score)
                {
                    max_score = bboxes[cur_pos].score;
                    max_pos = cur_pos;
                }
                cur_pos ++;
            }

            //对当前的最大Bbox 记下标记，即存入固定的内存内 
            bboxes[i] = bboxes[max_pos];

            //这是一个调换操作，将最大的Bbox 存入tmp_bbox 中，原来的tmp_bbox去填坑
            bboxes[max_pos] = tmp_bbox;
            tmp_bbox = bboxes[i];

            cur_pos = i + 1;

            while (cur_pos < N)
            {
                index_bbox = bboxes[cur_pos];

                float area = index_bbox.bx * index_bbox.by;
                float iou = calIOU_softNms(tmp_bbox,index_bbox); //分别轮流计算最大的Bbox和其余的Bbox的重叠率
                if (iou <= 0)
                {
                    cur_pos++;
                    continue;
                }
                iou /= area;
                if (method == 1) // 当选择的是线性加权法
                {
                    if (iou > iou_thre) //对于重叠率满足一定重叠率阈值的进行适当加权，而不像传统的NMS直接删除，softNMS只是进行降低分数
                    {
                        weight = 1 - iou;
                    } else
                    {
                        weight = 1;
                    }
                }else if (method == 2) //当选择的是高斯加权
                {
                    weight = exp(-(iou * iou) / sigma);
                }else // original NMS
                {
                    if (iou > iou_thre)
                    {
                        weight = 0;
                    }else
                    {
                        weight = 1;
                    }
                }
                bboxes[cur_pos].score *= weight;
                if (bboxes[cur_pos].score <= threshold)  //对最终的全部检测框的分数进行阈值筛选
                {
                    bboxes[cur_pos] = bboxes[N - 1];
                    N --;
                    cur_pos = cur_pos - 1;
                }
                cur_pos++;
            }
        }

        bboxes.resize(N);
    }


    float calIOU_softNms(const BboxWithScore& bbox1,const BboxWithScore& bbox2)
    {
        float iw = (std::min(bbox1.tx + bbox1.bx / 2.,bbox2.tx + bbox2.bx / 2.) -
                    std::max(bbox1.tx - bbox1.bx / 2.,bbox2.tx - bbox2.bx / 2.));
        if (iw < 0)
        {
            return 0.;
        }

        float ih = (std::min(bbox1.ty + bbox1.by / 2.,bbox2.ty + bbox2.by / 2.) -
                    std::max(bbox1.ty - bbox1.by / 2.,bbox2.ty - bbox2.by / 2.));

        if (ih < 0)
        {
            return 0.;
        }

        return iw * ih;
    }

/////////////////////////////////////////////////////////////////////////






}

