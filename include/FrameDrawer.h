#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <unordered_set>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    /**
     * @brief 构造函数
     * 
     * @param[in] pMap  地图指针
     */
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    /**
     * @brief 将跟踪线程的数据拷贝到绘图线程（图像、特征点、地图、跟踪状态）
     * 
     * @param[in] pTracker 追踪线程
     */
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    //
    /**
     * @brief 绘制最近处理过的帧,这个将会在可视化查看器的窗口中被创建
     * 
     * @return cv::Mat 返回绘制完成的图像,可以直接进行显示
     */
    cv::Mat DrawFrame();

protected:

    /**
     * @brief 绘制底部的信息栏
     * 
     * @param[in]  im           原始图像
     * @param[in]  nState       当前系统的工作状态
     * @param[out] imText       叠加后的图像
     */
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    ///当前绘制的图像
    cv::Mat mIm;
    ///当前帧中特征点的数目
    int N;
    ///当前帧中的特征点
    vector<cv::KeyPoint> mvCurrentKeys;
    ///当前帧中的特征点是否在地图中的标记
    ///当前帧的特征点在地图中是否出现;后者是表示地图中没有出现,但是在当前帧中是第一次被观测得到的点
    vector<bool> mvbMap, mvbVO;
    ///当前帧中追踪到的特征点计数
    int mnTracked, mnTrackedVO;
    ///参考帧中的特征点
    vector<cv::KeyPoint> mvIniKeys;
    ///当前帧特征点和参考帧特征点的匹配关系
    vector<int> mvIniMatches;
    ///当前SLAM系统的工作状态
    int mState;

    ///地图指针
    Map* mpMap;

    //线程锁
    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
