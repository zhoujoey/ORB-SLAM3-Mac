#include "Vocabulary.h"
#include "DescManip.h"
#include "quicklz.h"
#include <sstream>
namespace DBoW3{
// --------------------------------------------------------------------------


Vocabulary::Vocabulary
  (int k, int L, WeightingType weighting, ScoringType scoring)
  : m_k(k), m_L(L), m_weighting(weighting), m_scoring(scoring),
  m_scoring_object(NULL)
{
  createScoringObject();
}

// --------------------------------------------------------------------------


Vocabulary::Vocabulary
  (const std::string &filename): m_scoring_object(NULL)
{
  load(filename);
}

// --------------------------------------------------------------------------


Vocabulary::Vocabulary
  (const char *filename): m_scoring_object(NULL)
{
  load(filename);
}

// --------------------------------------------------------------------------


Vocabulary::Vocabulary
  (std::istream& stream): m_scoring_object(NULL)
{
  load(stream);
}

// --------------------------------------------------------------------------


void Vocabulary::createScoringObject()
{
  delete m_scoring_object;
  m_scoring_object = NULL;

  switch(m_scoring)
  {
    case L1_NORM:
      m_scoring_object = new L1Scoring;
      break;

    case L2_NORM:
      m_scoring_object = new L2Scoring;
      break;

    case CHI_SQUARE:
      m_scoring_object = new ChiSquareScoring;
      break;

    case KL:
      m_scoring_object = new KLScoring;
      break;

    case BHATTACHARYYA:
      m_scoring_object = new BhattacharyyaScoring;
      break;

    case DOT_PRODUCT:
      m_scoring_object = new DotProductScoring;
      break;

  }
}

Vocabulary::Vocabulary(
  const Vocabulary &voc)
  : m_scoring_object(NULL)
{
  *this = voc;
}

// --------------------------------------------------------------------------


Vocabulary::~Vocabulary()
{
  delete m_scoring_object;
}

// --------------------------------------------------------------------------


Vocabulary&
Vocabulary::operator=
  (const Vocabulary &voc)
{
  this->m_k = voc.m_k;
  this->m_L = voc.m_L;
  this->m_scoring = voc.m_scoring;
  this->m_weighting = voc.m_weighting;

  this->createScoringObject();

  this->m_nodes.clear();
  this->m_words.clear();

  this->m_nodes = voc.m_nodes;
  this->createWords();

  return *this;
}



void Vocabulary::getFeatures(
  const std::vector<std::vector<cv::Mat> > &training_features,
  std::vector<cv::Mat> &features) const
{
  features.resize(0);
  for(size_t i=0;i<training_features.size();i++)
      for(size_t j=0;j<training_features[i].size();j++)
              features.push_back(training_features[i][j]);
}

// --------------------------------------------------------------------------

void Vocabulary::initiateClusters
  (const std::vector<cv::Mat> &descriptors,
   std::vector<cv::Mat> &clusters) const
{
  initiateClustersKMpp(descriptors, clusters);
}

// --------------------------------------------------------------------------


void Vocabulary::initiateClustersKMpp(
  const std::vector<cv::Mat> &pfeatures,
    std::vector<cv::Mat> &clusters) const
{

//  DUtils::Random::SeedRandOnce();

  clusters.resize(0);
  clusters.reserve(m_k);
  std::vector<double> min_dists(pfeatures.size(), std::numeric_limits<double>::max());

  // 1.

  int ifeature = rand()% pfeatures.size();//DUtils::Random::RandomInt(0, pfeatures.size()-1);

  // create first cluster
  clusters.push_back(pfeatures[ifeature]);

  // compute the initial distances
   std::vector<double>::iterator dit;
  dit = min_dists.begin();
  for(auto fit = pfeatures.begin(); fit != pfeatures.end(); ++fit, ++dit)
  {
    *dit = DescManip::distance((*fit), clusters.back());
  }

  while((int)clusters.size() < m_k)
  {
    // 2.
    dit = min_dists.begin();
    for(auto  fit = pfeatures.begin(); fit != pfeatures.end(); ++fit, ++dit)
    {
      if(*dit > 0)
      {
        double dist = DescManip::distance((*fit), clusters.back());
        if(dist < *dit) *dit = dist;
      }
    }

    // 3.
    double dist_sum = std::accumulate(min_dists.begin(), min_dists.end(), 0.0);

    if(dist_sum > 0)
    {
      double cut_d;
      do
      {

        cut_d = (double(rand())/ double(RAND_MAX))* dist_sum;
      } while(cut_d == 0.0);

      double d_up_now = 0;
      for(dit = min_dists.begin(); dit != min_dists.end(); ++dit)
      {
        d_up_now += *dit;
        if(d_up_now >= cut_d) break;
      }

      if(dit == min_dists.end())
        ifeature = pfeatures.size()-1;
      else
        ifeature = dit - min_dists.begin();


      clusters.push_back(pfeatures[ifeature]);
    } // if dist_sum > 0
    else
      break;

  } // while(used_clusters < m_k)

}

// --------------------------------------------------------------------------


void Vocabulary::createWords()
{
  m_words.resize(0);

  if(!m_nodes.empty())
  {
    m_words.reserve( (int)pow((double)m_k, (double)m_L) );


    auto  nit = m_nodes.begin(); // ignore root
    for(++nit; nit != m_nodes.end(); ++nit)
    {
      if(nit->isLeaf())
      {
        nit->word_id = m_words.size();
        m_words.push_back( &(*nit) );
      }
    }
  }
}


float Vocabulary::getEffectiveLevels() const
{
  long sum = 0;
   for(auto wit = m_words.begin(); wit != m_words.end(); ++wit)
  {
    const Node *p = *wit;

    for(; p->id != 0; sum++) p = &m_nodes[p->parent];
  }

  return (float)((double)sum / (double)m_words.size());
}

// --------------------------------------------------------------------------


cv::Mat Vocabulary::getWord(WordId wid) const
{
  return m_words[wid]->descriptor;
}

// --------------------------------------------------------------------------


WordValue Vocabulary::getWordWeight(WordId wid) const
{
  return m_words[wid]->weight;
}

// --------------------------------------------------------------------------


WordId Vocabulary::transform
  (const cv::Mat& feature) const
{
  if(empty())
  {
    return 0;
  }

  WordId wid;
  transform(feature, wid);
  return wid;
}

// --------------------------------------------------------------------------

void Vocabulary::transform(
        const cv::Mat& features, BowVector &v) const
{
    //    std::vector<cv::Mat> vf(features.rows);
    //    for(int r=0;r<features.rows;r++) vf[r]=features.rowRange(r,r+1);
    //    transform(vf,v);



    v.clear();

    if(empty())
    {
        return;
    }

    // normalize
    LNorm norm;
    bool must = m_scoring_object->mustNormalize(norm);


    if(m_weighting == TF || m_weighting == TF_IDF)
    {
        for(int r=0;r<features.rows;r++)
        {
            WordId id;
            WordValue w;
            // w is the idf value if TF_IDF, 1 if TF
            transform(features.row(r), id, w);
            // not stopped
            if(w > 0)  v.addWeight(id, w);
        }

        if(!v.empty() && !must)
        {
            // unnecessary when normalizing
            const double nd = v.size();
            for(BowVector::iterator vit = v.begin(); vit != v.end(); vit++)
                vit->second /= nd;
        }

    }
    else // IDF || BINARY
    {
        for(int r=0;r<features.rows;r++)
        {
            WordId id;
            WordValue w;
            // w is idf if IDF, or 1 if BINARY

            transform(features.row(r), id, w);

            // not stopped
            if(w > 0) v.addIfNotExist(id, w);

        } // if add_features
    } // if m_weighting == ...

    if(must) v.normalize(norm);

}



void Vocabulary::transform(
  const std::vector<cv::Mat>& features, BowVector &v) const
{
  v.clear();

  if(empty())
  {
    return;
  }

  // normalize
  LNorm norm;
  bool must = m_scoring_object->mustNormalize(norm);


  if(m_weighting == TF || m_weighting == TF_IDF)
  {
    for(auto fit = features.begin(); fit < features.end(); ++fit)
    {
      WordId id;
      WordValue w;
      // w is the idf value if TF_IDF, 1 if TF

      transform(*fit, id, w);

      // not stopped
      if(w > 0) v.addWeight(id, w);
    }

    if(!v.empty() && !must)
    {
      // unnecessary when normalizing
      const double nd = v.size();
      for(BowVector::iterator vit = v.begin(); vit != v.end(); vit++)
        vit->second /= nd;
    }

  }
  else // IDF || BINARY
  {
    for(auto fit = features.begin(); fit < features.end(); ++fit)
    {
      WordId id;
      WordValue w;
      // w is idf if IDF, or 1 if BINARY

      transform(*fit, id, w);

      // not stopped
      if(w > 0) v.addIfNotExist(id, w);

    } // if add_features
  } // if m_weighting == ...

  if(must) v.normalize(norm);
}

// --------------------------------------------------------------------------


void Vocabulary::transform(
  const std::vector<cv::Mat>& features,
  BowVector &v, FeatureVector &fv, int levelsup) const
{
  v.clear();
  fv.clear();

  if(empty()) // safe for subclasses
  {
    return;
  }

  // normalize
  LNorm norm;
  bool must = m_scoring_object->mustNormalize(norm);


  if(m_weighting == TF || m_weighting == TF_IDF)
  {
    unsigned int i_feature = 0;
    for(auto fit = features.begin(); fit < features.end(); ++fit, ++i_feature)
    {
      WordId id;
      NodeId nid;
      WordValue w;
      // w is the idf value if TF_IDF, 1 if TF

      transform(*fit, id, w, &nid, levelsup);

      if(w > 0) // not stopped
      {
        v.addWeight(id, w);
        fv.addFeature(nid, i_feature);
      }
    }

    if(!v.empty() && !must)
    {
      // unnecessary when normalizing
      const double nd = v.size();
      for(BowVector::iterator vit = v.begin(); vit != v.end(); vit++)
        vit->second /= nd;
    }

  }
  else // IDF || BINARY
  {
    unsigned int i_feature = 0;
    for(auto fit = features.begin(); fit < features.end(); ++fit, ++i_feature)
    {
      WordId id;
      NodeId nid;
      WordValue w;
      // w is idf if IDF, or 1 if BINARY

      transform(*fit, id, w, &nid, levelsup);

      if(w > 0) // not stopped
      {
        v.addIfNotExist(id, w);
        fv.addFeature(nid, i_feature);
      }
    }
  } // if m_weighting == ...

  if(must) v.normalize(norm);
}

// --------------------------------------------------------------------------


// --------------------------------------------------------------------------


void Vocabulary::transform
  (const cv::Mat &feature, WordId &id) const
{
  WordValue weight;
  transform(feature, id, weight);
}

// --------------------------------------------------------------------------


void Vocabulary::transform(const cv::Mat &feature,
  WordId &word_id, WordValue &weight, NodeId *nid, int levelsup) const
{
  // propagate the feature down the tree


  // level at which the node must be stored in nid, if given
  const int nid_level = m_L - levelsup;
  if(nid_level <= 0 && nid != NULL) *nid = 0; // root

  NodeId final_id = 0; // root
  int current_level = 0;

  do
  {
    ++current_level;
    auto const  &nodes = m_nodes[final_id].children;
    double best_d = std::numeric_limits<double>::max();
//    DescManip::distance(feature, m_nodes[final_id].descriptor);

    for(const auto  &id:nodes)
    {
      double d = DescManip::distance(feature, m_nodes[id].descriptor);
      if(d < best_d)
      {
        best_d = d;
        final_id = id;
      }
    }

    if(nid != NULL && current_level == nid_level)
      *nid = final_id;

  } while( !m_nodes[final_id].isLeaf() );

  // turn node id into word id
  word_id = m_nodes[final_id].word_id;
  weight = m_nodes[final_id].weight;
}



void Vocabulary::transform(const cv::Mat &feature,
  WordId &word_id, WordValue &weight ) const
{
  // propagate the feature down the tree


  // level at which the node must be stored in nid, if given

  NodeId final_id = 0; // root
//maximum speed by computing here distance and avoid calling to DescManip::distance

  //binary descriptor
  if (feature.type()==CV_8U){
      do
      {
          auto const  &nodes = m_nodes[final_id].children;
          uint64_t best_d = std::numeric_limits<uint64_t>::max();
          int idx=0,bestidx=0;
           for(const auto  &id:nodes)
          {
              //compute distance
             
              uint64_t dist= DescManip::distance_8uc1(feature, m_nodes[id].descriptor);
              if(dist < best_d)
              {
                  best_d = dist;
                  final_id = id;
                  bestidx=idx;
              }
              idx++;
          }
      } while( !m_nodes[final_id].isLeaf() );
   }
  else
  {
	  do
	  {
		  auto const  &nodes = m_nodes[final_id].children;
		  uint64_t best_d = std::numeric_limits<uint64_t>::max();
		  int idx = 0, bestidx = 0;
		  for (const auto &id : nodes)
		  {
			  //compute distance
			  uint64_t dist = DescManip::distance(feature, m_nodes[id].descriptor);
			  if (dist < best_d)
			  {
				  best_d = dist;
				  final_id = id;
				  bestidx = idx;
			  }
			  idx++;
		  }
	  } while (!m_nodes[final_id].isLeaf());
  }


  // turn node id into word id
  word_id = m_nodes[final_id].word_id;
  weight = m_nodes[final_id].weight;
}
// --------------------------------------------------------------------------

NodeId Vocabulary::getParentNode
  (WordId wid, int levelsup) const
{
  NodeId ret = m_words[wid]->id; // node id
  while(levelsup > 0 && ret != 0) // ret == 0 --> root
  {
    --levelsup;
    ret = m_nodes[ret].parent;
  }
  return ret;
}

// --------------------------------------------------------------------------


void Vocabulary::getWordsFromNode
  (NodeId nid, std::vector<WordId> &words) const
{
  words.clear();

  if(m_nodes[nid].isLeaf())
  {
    words.push_back(m_nodes[nid].word_id);
  }
  else
  {
    words.reserve(m_k); // ^1, ^2, ...

    std::vector<NodeId> parents;
    parents.push_back(nid);

    while(!parents.empty())
    {
      NodeId parentid = parents.back();
      parents.pop_back();

      const std::vector<NodeId> &child_ids = m_nodes[parentid].children;
      std::vector<NodeId>::const_iterator cit;

      for(cit = child_ids.begin(); cit != child_ids.end(); ++cit)
      {
        const Node &child_node = m_nodes[*cit];

        if(child_node.isLeaf())
          words.push_back(child_node.word_id);
        else
          parents.push_back(*cit);

      } // for each child
    } // while !parents.empty
  }
}

// --------------------------------------------------------------------------


int Vocabulary::stopWords(double minWeight)
{
  int c = 0;
   for(auto wit = m_words.begin(); wit != m_words.end(); ++wit)
  {
    if((*wit)->weight < minWeight)
    {
      ++c;
      (*wit)->weight = 0;
    }
  }
  return c;
}

// --------------------------------------------------------------------------


// --------------------------------------------------------------------------


void Vocabulary::load(const std::string &filename)
{
    //check first if it is a binary file
    std::ifstream ifile(filename,std::ios::binary);
    if (!ifile) throw std::runtime_error("Vocabulary::load Could not open file :"+filename+" for reading");
    if(!load(ifile)) {
	    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
	    if(!fs.isOpened()) throw std::string("Could not open file ") + filename;
	    load(fs);
    }
}


bool Vocabulary::load(std::istream &ifile)
{
    uint64_t sig;//magic number describing the file
    ifile.read((char*)&sig,sizeof(sig));
    if (sig != 88877711233) // Check if it is a binary file.
        return false;

    ifile.seekg(0,std::ios::beg);
    fromStream(ifile);
    return true;
}


void Vocabulary::fromStream(  std::istream &str )   throw(std::exception){


    m_words.clear();
    m_nodes.clear();
    uint64_t sig=0;//magic number describing the file
    str.read((char*)&sig,sizeof(sig));
    if (sig!=88877711233) throw std::runtime_error("Vocabulary::fromStream  is not of appropriate type");
    bool compressed;
    str.read((char*)&compressed,sizeof(compressed));
    uint32_t nnodes;
    str.read((char*)&nnodes,sizeof(nnodes));
    if(nnodes==0)return;
    std::stringstream decompressed_stream;
    std::istream *_used_str=0;
    if (compressed){
        qlz_state_decompress state_decompress;
        memset(&state_decompress, 0, sizeof(qlz_state_decompress));
        int chunkSize=10000;
        std::vector<char> decompressed(chunkSize);
        std::vector<char> input(chunkSize+400);
        //read how many chunks are there
        uint32_t nChunks;
        str.read((char*)&nChunks,sizeof(nChunks));
        for(int i=0;i<nChunks;i++){
            str.read(&input[0],9);
            int c=qlz_size_compressed(&input[0]);
            str.read(&input[9],c-9);
            size_t d=qlz_decompress(&input[0], &decompressed[0], &state_decompress);
            decompressed_stream.write(&decompressed[0],d);
        }
        _used_str=&decompressed_stream;
    }
    else{
        _used_str=&str;
    }

    _used_str->read((char*)&m_k,sizeof(m_k));
    _used_str->read((char*)&m_L,sizeof(m_L));
    _used_str->read((char*)&m_scoring,sizeof(m_scoring));
    _used_str->read((char*)&m_weighting,sizeof(m_weighting));

    createScoringObject();
    m_nodes.resize(nnodes );
    m_nodes[0].id = 0;



    for(size_t i = 1; i < m_nodes.size(); ++i)
    {
        NodeId nid;
        _used_str->read((char*)&nid,sizeof(NodeId));
        Node& child = m_nodes[nid];
        child.id=nid;
        _used_str->read((char*)&child.parent,sizeof(child.parent));
        _used_str->read((char*)&child.weight,sizeof(child.weight));
        DescManip::fromStream(child.descriptor,*_used_str);
        m_nodes[child.parent].children.push_back(child.id);
     }
     //    // words
    uint32_t m_words_size;
    _used_str->read((char*)&m_words_size,sizeof(m_words_size));
    m_words.resize(m_words_size);
    for(unsigned int i = 0; i < m_words.size(); ++i)
    {
        WordId wid;NodeId nid;
        _used_str->read((char*)&wid,sizeof(wid));
        _used_str->read((char*)&nid,sizeof(nid));
        m_nodes[nid].word_id = wid;
        m_words[wid] = &m_nodes[nid];
    }
}
// --------------------------------------------------------------------------



void Vocabulary::load(const cv::FileStorage &fs,
  const std::string &name)
{
  m_words.clear();
  m_nodes.clear();

  cv::FileNode fvoc = fs[name];

  m_k = (int)fvoc["k"];
  m_L = (int)fvoc["L"];
  m_scoring = (ScoringType)((int)fvoc["scoringType"]);
  m_weighting = (WeightingType)((int)fvoc["weightingType"]);

  createScoringObject();

  // nodes
  cv::FileNode fn = fvoc["nodes"];

  m_nodes.resize(fn.size() + 1); // +1 to include root
  m_nodes[0].id = 0;

  for(unsigned int i = 0; i < fn.size(); ++i)
  {
    NodeId nid = (int)fn[i]["nodeId"];
    NodeId pid = (int)fn[i]["parentId"];
    WordValue weight = (WordValue)fn[i]["weight"];
    std::string d = (std::string)fn[i]["descriptor"];

    m_nodes[nid].id = nid;
    m_nodes[nid].parent = pid;
    m_nodes[nid].weight = weight;
    m_nodes[pid].children.push_back(nid);

    DescManip::fromString(m_nodes[nid].descriptor, d);
  }

  // words
  fn = fvoc["words"];

  m_words.resize(fn.size());

  for(unsigned int i = 0; i < fn.size(); ++i)
  {
    NodeId wid = (int)fn[i]["wordId"];
    NodeId nid = (int)fn[i]["nodeId"];

    m_nodes[nid].word_id = wid;
    m_words[wid] = &m_nodes[nid];
  }
}



std::ostream& operator<<(std::ostream &os,
  const Vocabulary &voc)
{
  os << "Vocabulary: k = " << voc.getBranchingFactor()
    << ", L = " << voc.getDepthLevels()
    << ", Weighting = ";

  switch(voc.getWeightingType())
  {
    case TF_IDF: os << "tf-idf"; break;
    case TF: os << "tf"; break;
    case IDF: os << "idf"; break;
    case BINARY: os << "binary"; break;
  }

  os << ", Scoring = ";
  switch(voc.getScoringType())
  {
    case L1_NORM: os << "L1-norm"; break;
    case L2_NORM: os << "L2-norm"; break;
    case CHI_SQUARE: os << "Chi square distance"; break;
    case KL: os << "KL-divergence"; break;
    case BHATTACHARYYA: os << "Bhattacharyya coefficient"; break;
    case DOT_PRODUCT: os << "Dot product"; break;
  }

  os << ", Number of words = " << voc.size();

  return os;
}
/**
 * @brief Vocabulary::clear
 */
void Vocabulary::clear(){
    delete m_scoring_object;
    m_scoring_object=0;
    m_nodes.clear();
    m_words.clear();

}
int Vocabulary::getDescritorSize()const
{
    if (m_words.size()==0)return -1;
    else return m_words[0]->descriptor.cols;
}
int Vocabulary::getDescritorType()const{

    if (m_words.size()==0)return -1;
    else return m_words[0]->descriptor.type();
}


}
