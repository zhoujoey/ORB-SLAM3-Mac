#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include <opencv2/core/core.hpp>
#include <mutex>


namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{
public:
    MapPoint();

    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);

    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    /**
     * @brief 获取观测到当前地图点的关键帧
     * @return std::map<KeyFrame*,size_t> 观测到当前地图点的关键帧序列； 
     *                                    size_t 这个对象对应为该地图点在该关键帧的特征点的访问id
     */
    std::map<KeyFrame*,size_t> GetObservations();
    
    // 获取当前地图点的被观测次数
    int Observations();

    /**
     * @brief 添加观测
     *
     * 记录哪些KeyFrame的那个特征点能观测到该MapPoint \n
     * 并增加观测的相机数目nObs，单目+1，双目或者grbd+2
     * 这个函数是建立关键帧共视关系的核心函数，能共同观测到某些MapPoints的关键帧是共视关键帧
     * @param[in] pKF KeyFrame,观测到当前地图点的关键帧
     * @param[in] idx MapPoint在KeyFrame中的索引
     */
    void AddObservation(KeyFrame* pKF,size_t idx);
    /**
     * @brief 取消某个关键帧对当前地图点的观测
     * @detials 如果某个关键帧要被删除，那么会发生这个操作
     * @param[in] pKF 
     */
    void EraseObservation(KeyFrame* pKF);

    /**
     * @brief 获取观测到当前地图点的关键帧,在观测数据中的索引
     * 
     * @param[in] pKF   关键帧
     * @return int      索引
     */
    int GetIndexInKeyFrame(KeyFrame* pKF);
    /**
     * @brief 查看某个关键帧是否看到了当前的地图点
     * 
     * @param[in] pKF   关键帧
     * @return true 
     * @return false 
     */
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();
    void SetNormalVector(cv::Mat& normal);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    Map* GetMap();
    void UpdateMap(Map* pMap);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    cv::Mat mPosMerge;
    cv::Mat mNormalVectorMerge;


    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;
    KeyFrame* mpHostKF;

    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
    // 观测到该MapPoint的KF和该MapPoint在KF中的索引
    std::map<KeyFrame*,size_t> mObservations; 
     // For save relation without pointer, this is necessary for save/load function
     std::map<long unsigned int, int> mBackupObservationsId1;
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;
     long unsigned int mBackupRefKFId;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     long long int mBackupReplacedId;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
