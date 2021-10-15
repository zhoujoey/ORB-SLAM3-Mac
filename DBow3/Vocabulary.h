
#ifndef __D_T__VOCABULARY__
#define __D_T__VOCABULARY__

#include <cassert>

#include <vector>
#include <numeric>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include "exports.h"
#include "FeatureVector.h"
#include "BowVector.h"
#include "ScoringObject.h"
#include <limits>
namespace DBoW3 {
///   Vocabulary
class DBOW_API Vocabulary
{		
friend class FastSearch;
public:
  

  Vocabulary(int k = 10, int L = 5,
    WeightingType weighting = TF_IDF, ScoringType scoring = L1_NORM);
  

  Vocabulary(const std::string &filename);
  

  Vocabulary(const char *filename);
  
  Vocabulary(std::istream &filename);
  
  Vocabulary(const Vocabulary &voc);
  
  virtual ~Vocabulary();
  
  Vocabulary& operator=(
    const Vocabulary &voc);

  virtual inline unsigned int size() const{  return (unsigned int)m_words.size();}


  virtual inline bool empty() const{ return m_words.empty();}

  void clear();

  virtual void transform(const std::vector<cv::Mat>& features, BowVector &v)
    const;

  virtual void transform(const  cv::Mat & features, BowVector &v)
    const;

  virtual void transform(const std::vector<cv::Mat>& features,
    BowVector &v, FeatureVector &fv, int levelsup) const;

  virtual WordId transform(const cv::Mat& feature) const;

  double score(const BowVector &a, const BowVector &b) const{    return m_scoring_object->score(a, b);}

  virtual NodeId getParentNode(WordId wid, int levelsup) const;
  
  void getWordsFromNode(NodeId nid, std::vector<WordId> &words) const;

  inline int getBranchingFactor() const { return m_k; }

  inline int getDepthLevels() const { return m_L; }
 
  float getEffectiveLevels() const;
  
  virtual inline cv::Mat getWord(WordId wid) const;
  
  virtual inline WordValue getWordWeight(WordId wid) const;
  
  inline WeightingType getWeightingType() const { return m_weighting; }
  
  inline ScoringType getScoringType() const { return m_scoring; }


  void load(const std::string &filename);

  bool load(std::istream &stream);

  virtual void load(const cv::FileStorage &fs, 
    const std::string &name = "vocabulary");

  virtual int stopWords(double minWeight);

  int getDescritorSize()const;

  int getDescritorType()const;

  void fromStream(  std::istream &str )   throw(std::exception);

 protected:

  typedef const cv::Mat pDescriptor;

  struct Node 
  {
    NodeId id;
    /// Weight if the node is a word
    WordValue weight;
    /// Children 
    std::vector<NodeId> children;
    /// Parent node (undefined in case of root)
    NodeId parent;
    /// Node descriptor
    cv::Mat descriptor;

    /// Word id if the node is a word
    WordId word_id;

    Node(): id(0), weight(0), parent(0), word_id(0){}
    
    Node(NodeId _id): id(_id), weight(0), parent(0), word_id(0){}

    inline bool isLeaf() const { return children.empty(); }
  };

protected:

  void createScoringObject();

  void getFeatures(const std::vector<std::vector<cv::Mat> > &training_features,
    std::vector<cv::Mat> &features) const;

  virtual void transform(const cv::Mat &feature,
    WordId &id, WordValue &weight, NodeId* nid  , int levelsup = 0) const;

  virtual void transform(const cv::Mat &feature,
    WordId &id, WordValue &weight ) const;

  virtual void transform(const cv::Mat &feature, WordId &id) const;


  virtual void initiateClusters(const std::vector<cv::Mat> &descriptors,
    std::vector<cv::Mat> &clusters) const;

  void initiateClustersKMpp(const std::vector<cv::Mat> &descriptors,
    std::vector<cv::Mat> &clusters) const;

  void createWords();

  
   DBOW_API friend std::ostream& operator<<(std::ostream &os,  const Vocabulary &voc);


protected:

  /// Branching factor
  int m_k;
  
  /// Depth levels 
  int m_L;
  
  /// Weighting method
  WeightingType m_weighting;
  
  /// Scoring method
  ScoringType m_scoring;
  
  /// Object for computing scores
  GeneralScoring* m_scoring_object;
  
  /// Tree nodes
  std::vector<Node> m_nodes;
  
  /// Words of the vocabulary (tree leaves)
  /// this condition holds: m_words[wid]->word_id == wid
  std::vector<Node*> m_words;
public:
  //for debug (REMOVE)
  inline Node* getNodeWord(uint32_t idx){return m_words[idx];}

};


} // namespace DBoW3

#endif
