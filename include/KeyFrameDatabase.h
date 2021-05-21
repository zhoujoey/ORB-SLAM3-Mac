/**
 * @file KeyFrameDatabase.h
 * @author guoqing (1337841346@qq.com)
 * @brief 关键帧数据库,用于回环检测和重定位
 * @version 0.1
 * @date 2019-04-25
 * 
 * @copyright Copyright (c) 2019
 * 
 */



#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include <mutex>


namespace ORB_SLAM2
{

class KeyFrame;
class Frame;
class Map;

/** @brief 关键帧数据库 */
class KeyFrameDatabase
{
public:

  /**
   * @brief 构造函数
   * @param[in] voc 词袋模型的字典
   */
    KeyFrameDatabase(const ORBVocabulary &voc);

  /**
   * @brief 根据关键帧的词包，更新数据库的倒排索引
   * @param pKF 关键帧
   */
   void add(KeyFrame* pKF);

  /**
   * @brief 关键帧被删除后，更新数据库的倒排索引
   * @param pKF 关键帧
   */
   void erase(KeyFrame* pKF);

  /** @brief 清空关键帧数据库 */
   void clear();

   /**
   * @brief 在闭环检测中找到与该关键帧可能闭环的关键帧
   * @param pKF      需要闭环的关键帧
   * @param minScore 相似性分数最低要求
   * @return         可能闭环的关键帧
   * @see III-E Bags of Words Place Recognition
   */
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates);

   // Relocalization
   /**
   * @brief 在重定位中找到与该帧相似的关键帧
   * 1. 找出和当前帧具有公共单词的所有关键帧
   * 2. 只和具有共同单词较多的关键帧进行相似度计算
   * 3. 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
   * 4. 只返回累计得分较高的组中分数最高的关键帧
   * @param F 需要重定位的帧
   * @return  相似的关键帧
   * @see III-E Bags of Words Place Recognition
   */
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  // 预先训练好的词典
  const ORBVocabulary* mpVoc; 

  // Inverted file
  // 倒排索引，mvInvertedFile[i]表示包含了第i个word id的所有关键帧
  std::vector<list<KeyFrame*> > mvInvertedFile; 

  /// Mutex, 多用途的
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
