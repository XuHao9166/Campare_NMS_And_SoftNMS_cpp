#ifndef PROJECT_SOFT_NMS_H
#define PROJECT_SOFT_NMS_H


#include <opencv2/opencv.hpp>



struct BboxWithScore
{
	float tx, ty, bx, by, area, score; //（tx,ty）检测框左上角顶点坐标 ，  （bx,by） 检测框右下角顶点坐标 ， area 检测框面积 ，score 得分 
    BboxWithScore()
    {
        tx = 0.;
        ty = 0.;
        bx = 0.;
        by = 0.;
		area = 0.;
        score = 0.;
    }
};





namespace XuHao
{
    //method 0 : origin nms, 1: liner, 2: gaussian
    void softNms(std::vector<BboxWithScore>& bboxes,const int& method = 0,
                               const float& sigma = 0.5,const float& iou_thre = 0.3,const float& threshold = 0.01);
    float calIOU_softNms(const BboxWithScore& bbox1,const BboxWithScore& bbox2);
    

	///传统NMS 
	
	bool Traditinal_cmpScore(const BboxWithScore &lsh, const BboxWithScore &rsh);
	void Traditinal_NMS(std::vector<BboxWithScore>& boundingBox_, const float overlap_threshold);

}

#endif //PROJECT_SOFT_NMS_H
