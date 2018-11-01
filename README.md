# Campare_NMS_And_SoftNMS_cpp

///////////传统的NMS
	计算过程
	1、根据候选框的类别分类概率做排序，假如有4个 BBox ，其置信度A>B>C>D。
	2、先标记最大概率矩形框A是算法要保留的BBox；
	3、从最大概率矩形框A开始，分别判断ABC与D的重叠度IOU（两框的交并比）是否大于某个设定的阈值(0.5)，假设D与A的重叠度超过阈值，那么就舍弃D；
	4、从剩下的矩形框BC中，选择概率最大的B，标记为保留，然后判读C与B的重叠度，扔掉重叠度超过设定阈值的矩形框；
	5、一直重复进行，标记完所有要保留下来的矩形框。

	NMS缺点：
	1、NMS算法中的最大问题就是它将相邻检测框的分数均强制归零(既将重叠部分大于重叠阈值Nt的检测框移除)。在这种情况下，如果一个真实物体在重叠区域出现，则将导致对该物体的检测失败并降低了算法的平均检测率（average precision, AP）。
	2、NMS的阈值也不太容易确定，设置过小会出现误删，设置过高又容易增大误检。
	3、NMS一般只能使用CPU计算，无法使用GPU计算。
  
  ///////////soft_NMS
	softNMS优点：
	1、SoftNMS可以很方便地引入到object detection算法中，不需要重新训练原有的模型、代码容易实现，
	不增加计算量（计算量相比整个object detection算法可忽略）。并且很容易集成到目前所有使用NMS的目标检测算法。
        2、softNMS在训练中采用传统的NMS方法，仅在推断代码中实现softNMS。作者应该做过对比试验，
         在训练过程中采用softNMS没有显著提高。
	3、NMS是Soft - NMS特殊形式，当得分重置函数采用二值化函数时，Soft - NMS和NMS是相同的。soft - NMS算法是一种更加通用的非最大抑制算法。

	softNMS缺点：
	softNMS也是一种贪心算法，并不能保证找到全局最优的检测框分数重置。除了以上这两种分数重置函数，我们也可以考虑开发其他包含更多参数的分数重置函数，比如Gompertz函数等。但是它们在完成分数重置的过程中增加了额外的参数。

 ![tu1](https://github.com/XuHao9166/Campare_NMS_And_SoftNMS_cpp/blob/master/1.jpg)


![tu2](https://github.com/XuHao9166/Campare_NMS_And_SoftNMS_cpp/blob/master/1.jpg)
