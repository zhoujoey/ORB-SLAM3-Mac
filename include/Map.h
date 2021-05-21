#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#include <pangolin/pangolin.h>
#include <vector>
#include <list>
#include <mutex>

#include <boost/serialization/base_object.hpp>


namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;

/**
 * @brief 地图
 * 
 */
class Map
{
public:
    /** @brief 构造函数 */
    Map();
    Map(int initKFid);
    ~Map();

    /**
     * @brief 向地图中添加关键帧
     * 
     * @param[in] pKF 关键帧
     */
    void AddKeyFrame(KeyFrame* pKF);
    /**
     * @brief 向地图中添加地图点
     * 
     * @param[in] pMP 地图点
     */
    void AddMapPoint(MapPoint* pMP);
    /**
     * @brief 从地图中擦除地图点
     * 
     * @param[in] pMP 地图点
     */
    void EraseMapPoint(MapPoint* pMP);
    /**
     * @brief 从地图中删除关键帧
     * @detials 实际上这个函数中目前仅仅是删除了在std::set中保存的地图点的指针,并且删除后
     * 之前的地图点所占用的内存其实并没有得到释放
     * @param[in] pKF 关键帧
     */
    void EraseKeyFrame(KeyFrame* pKF);
    /**
     * @brief 设置参考地图点
     * @detials 一般是指,设置当前帧中的参考地图点; 这些点将用于DrawMapPoints函数画图
     * 
     * @param[in] vpMPs 地图点们
     */
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    /**
     * @brief 这个函数好像没有被用到过
     * 
     */
    //REVIEW
    void InformNewBigChange();
    /**
     * @brief 获取最大改变;但是这个函数最终好像并没有被使用到
     * 
     * @return int 
     */
    int GetLastBigChangeIdx();

    /**
     * @brief 获取地图中的所有关键帧
     * 
     * @return std::vector<KeyFrame*> 获得的关键帧序列
     */
    std::vector<KeyFrame*> GetAllKeyFrames();
    /**
     * @brief 获取地图中的所有地图点
     * 
     * @return std::vector<MapPoint*> 获得的地图点序列
     */
    std::vector<MapPoint*> GetAllMapPoints();
    /**
     * @brief 获取地图中的所有参考地图点
     * 
     * @return std::vector<MapPoint*> 获得的参考地图点序列
     */
    std::vector<MapPoint*> GetReferenceMapPoints();

    /**
     * @brief 获得当前地图中的地图点个数
     * 
     * @return long unsigned int 个数
     */
    long unsigned int MapPointsInMap();
    /**
     * @brief 获取当前地图中的关键帧个数
     * 
     * @return long unsigned 关键帧个数
     */
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    /**
     * @brief 获取关键帧的最大id
     * 
     * @return long unsigned int  id
     */
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void RotateMap(const cv::Mat &R);
    void ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel=false, const cv::Mat t=cv::Mat::zeros(cv::Size(1,3),CV_32F));

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();
    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    ///当更新地图时的互斥量.回环检测中和局部BA后更新全局地图的时候会用到这个
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    ///为了避免地图点id冲突设计的互斥量
    std::mutex mMutexPointCreation;

    static long unsigned int nNextId;

protected:

    long unsigned int mnId;

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    bool mIsInUse;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM2

#endif // MAP_H
