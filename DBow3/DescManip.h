#ifndef __D_T_DESCMANIP__
#define __D_T_DESCMANIP__

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include "exports.h"

namespace DBoW3 {

/// Class to manipulate descriptors (calculating means, differences and IO routines)
class DBOW_API DescManip
{
public:

   static void meanValue(const std::vector<cv::Mat> &descriptors,
    cv::Mat &mean)  ;

   static double distance(const cv::Mat &a, const cv::Mat &b);
   static  inline uint32_t distance_8uc1(const cv::Mat &a, const cv::Mat &b);


  static void fromString(cv::Mat &a, const std::string &s);

  static void fromStream(cv::Mat &m,std::istream &str);
public:

  static size_t getDescSizeBytes(const cv::Mat & d){return d.cols* d.elemSize();}
};

uint32_t DescManip::distance_8uc1(const cv::Mat &a, const cv::Mat &b){
         const uint64_t *pa, *pb;
         pa = a.ptr<uint64_t>(); // a & b are actually CV_8U
         pb = b.ptr<uint64_t>();

         uint64_t v, ret = 0;
         int n=a.cols / sizeof(uint64_t);
         for(size_t i = 0; i < n; ++i, ++pa, ++pb)
         {
           v = *pa ^ *pb;
           v = v - ((v >> 1) & (uint64_t)~(uint64_t)0/3);
           v = (v & (uint64_t)~(uint64_t)0/15*3) + ((v >> 2) &
             (uint64_t)~(uint64_t)0/15*3);
           v = (v + (v >> 4)) & (uint64_t)~(uint64_t)0/255*15;
           ret += (uint64_t)(v * ((uint64_t)~(uint64_t)0/255)) >>
             (sizeof(uint64_t) - 1) * CHAR_BIT;
         }
         return ret;
}
} // namespace DBoW3

#endif
