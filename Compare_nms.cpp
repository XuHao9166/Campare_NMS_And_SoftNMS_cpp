#include "Compare_nms.h"
#include<iostream>
#include<vector>
#include<algorithm>
using namespace std;


namespace XuHao
{
   
///////////��ͳ��NMS
	///�������
	//1�����ݺ�ѡ�����������������򣬼�����4�� BBox �������Ŷ�A>B>C>D��
	//2���ȱ�������ʾ��ο�A���㷨Ҫ������BBox��
	//3���������ʾ��ο�A��ʼ���ֱ��ж�ABC��D���ص���IOU������Ľ����ȣ��Ƿ����ĳ���趨����ֵ(0.5)������D��A���ص��ȳ�����ֵ����ô������D��
	//4����ʣ�µľ��ο�BC�У�ѡ���������B�����Ϊ������Ȼ���ж�C��B���ص��ȣ��ӵ��ص��ȳ����趨��ֵ�ľ��ο�
	//5��һֱ�ظ����У����������Ҫ���������ľ��ο�

	//NMSȱ�㣺
	//1��NMS�㷨�е������������������ڼ���ķ�����ǿ�ƹ���(�Ƚ��ص����ִ����ص���ֵNt�ļ����Ƴ�)������������£����һ����ʵ�������ص�������֣��򽫵��¶Ը�����ļ��ʧ�ܲ��������㷨��ƽ������ʣ�average precision, AP����
	//2��NMS����ֵҲ��̫����ȷ�������ù�С�������ɾ�����ù���������������졣
	//3��NMSһ��ֻ��ʹ��CPU���㣬�޷�ʹ��GPU���㡣

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
		//�Ը�����ѡ�����score�Ĵ�С������������
		sort(boundingBox_.begin(), boundingBox_.end(), Traditinal_cmpScore);
		float IOU = 0;
		float maxX = 0;
		float maxY = 0;
		float minX = 0;
		float minY = 0;
		vector<int> vPick;
		int nPick = 0;
		multimap<float, int> vScores;   //����������к��score�Ͷ�Ӧ�����
		const int num_boxes = boundingBox_.size();
		vPick.resize(num_boxes);
		for (int i = 0; i < num_boxes; ++i){
			vScores.insert(pair<float, int>(boundingBox_[i].score, i));
		}
		while (vScores.size() > 0){
			int last = vScores.rbegin()->second;  //��������������vScores���е�����Ǹ����к�
			vPick[nPick] = last;
			nPick += 1;
			for (multimap<float, int>::iterator it = vScores.begin(); it != vScores.end();){
				int it_idx = it->second;
				maxX = max(boundingBox_.at(it_idx).tx, boundingBox_.at(last).tx);
				maxY = max(boundingBox_.at(it_idx).ty, boundingBox_.at(last).ty);
				minX = min(boundingBox_.at(it_idx).bx, boundingBox_.at(last).bx);
				minY = min(boundingBox_.at(it_idx).by, boundingBox_.at(last).by);
				//ת�����������߽���ཻ����ı߳�
				maxX = ((minX - maxX + 1) > 0) ? (minX - maxX + 1) : 0;
				maxY = ((minY - maxY + 1) > 0) ? (minY - maxY + 1) : 0;
				//�󽻲���IOU

				IOU = (maxX * maxY) / (boundingBox_.at(it_idx).area + boundingBox_.at(last).area - IOU);
				if (IOU > overlap_threshold){
					it = vScores.erase(it);    //ɾ�������ȴ�����ֵ�ĺ�ѡ��,erase����ɾ��Ԫ�ص���һ��Ԫ��
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
	//softNMS�ŵ㣺
	//1��SoftNMS���Ժܷ�������뵽object detection�㷨�У�����Ҫ����ѵ��ԭ�е�ģ�͡���������ʵ�֣�
	//�����Ӽ��������������������object detection�㷨�ɺ��ԣ������Һ����׼��ɵ�Ŀǰ����ʹ��NMS��Ŀ�����㷨��
	//2��soft - NMS��ѵ���в��ô�ͳ��NMS�����������ƶϴ�����ʵ��soft - NMS������Ӧ�������Ա����飬
	//��ѵ�������в���soft - NMSû��������ߡ�
	//3��NMS��Soft - NMS������ʽ�����÷����ú������ö�ֵ������ʱ��Soft - NMS��NMS����ͬ�ġ�soft - NMS�㷨��һ�ָ���ͨ�õķ���������㷨��

	//softNMSȱ�㣺
	//softNMSҲ��һ��̰���㷨�������ܱ�֤�ҵ�ȫ�����ŵļ���������á��������������ַ������ú���������Ҳ���Կ��ǿ�������������������ķ������ú���������Gompertz�����ȡ�������������ɷ������õĹ����������˶���Ĳ�����


	//def cpu_soft_nms(np.ndarray[float, ndim=2] boxes, float sigma=0.5, float Nt=0.3, float threshold=0.001, unsigned int method=0):
	//����ԭ���߸�����python�汾�Ĳ�������

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

            //������÷ֵ� bbox
            while (cur_pos < N)
            {
                if (max_score < bboxes[cur_pos].score)
                {
                    max_score = bboxes[cur_pos].score;
                    max_pos = cur_pos;
                }
                cur_pos ++;
            }

            //�Ե�ǰ�����Bbox ���±�ǣ�������̶����ڴ��� 
            bboxes[i] = bboxes[max_pos];

            //����һ������������������Bbox ����tmp_bbox �У�ԭ����tmp_bboxȥ���
            bboxes[max_pos] = tmp_bbox;
            tmp_bbox = bboxes[i];

            cur_pos = i + 1;

            while (cur_pos < N)
            {
                index_bbox = bboxes[cur_pos];

                float area = index_bbox.bx * index_bbox.by;
                float iou = calIOU_softNms(tmp_bbox,index_bbox); //�ֱ�������������Bbox�������Bbox���ص���
                if (iou <= 0)
                {
                    cur_pos++;
                    continue;
                }
                iou /= area;
                if (method == 1) // ��ѡ��������Լ�Ȩ��
                {
                    if (iou > iou_thre) //�����ص�������һ���ص�����ֵ�Ľ����ʵ���Ȩ��������ͳ��NMSֱ��ɾ����softNMSֻ�ǽ��н��ͷ���
                    {
                        weight = 1 - iou;
                    } else
                    {
                        weight = 1;
                    }
                }else if (method == 2) //��ѡ����Ǹ�˹��Ȩ
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
                if (bboxes[cur_pos].score <= threshold)  //�����յ�ȫ������ķ���������ֵɸѡ
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

