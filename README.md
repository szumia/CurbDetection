## 路沿检测
本仓库实现了基于传统视觉算法进行路沿检测算法，能对任务给出的不同难度等级的测试视频均有较好的效果。核心实现包括：  
* 预处理
* ROI提取
* Hough直线检测
* 帧差法进行跟踪和预测

### Result
以下动图给出对应于最低难度视频1.avi(静止，完整路沿）的检测效果：
![gif1](../CurbDetection/results/01avi检测结果.gif)  

以下动图给出对应于中等难度视频2.avi(运动，完整直线路沿）的检测效果：
![gif2](../CurbDetection/results/02avi检测结果.gif)  
  
以下动图给出对应于最高难度视频3.avi（路沿有残缺、拐弯、视频画质模糊）的检测效果：  
![gif3](../CurbDetection/results/03avi检测结果.gif)

绿色点为跟踪点，蓝线为检测出的路沿线，红线为跟踪预测出的路沿线，大部分情况下红蓝线是重合的。  
左上角显示“track跟踪状态”和“drop”跟踪丢失状态。

#### Contact
If you have any idea or questions, please contact 13046311074@163.com, thanks for your view and star.