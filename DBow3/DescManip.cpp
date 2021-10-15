#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <limits.h>

#include "DescManip.h"

using namespace std;

namespace DBoW3 {


// --------------------------------------------------------------------------
static  inline uint32_t distance_8uc1(const cv::Mat &a, const cv::Mat &b);

double DescManip::distance(const cv::Mat &a,  const cv::Mat &b)
{

    //binary descriptor
    if (a.type()==CV_8U){

        // Bit count function got from:
         // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
         // This implementation assumes that a.cols (CV_8U) % sizeof(uint64_t) == 0

         const uint64_t *pa, *pb;
         pa = a.ptr<uint64_t>(); // a & b are actually CV_8U
         pb = b.ptr<uint64_t>();

         uint64_t v, ret = 0;
         for(size_t i = 0; i < a.cols / sizeof(uint64_t); ++i, ++pa, ++pb)
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
    else{
        double sqd = 0.;
        assert(a.type()==CV_32F);
        assert(a.rows==1);
        const float *a_ptr=a.ptr<float>(0);
        const float *b_ptr=b.ptr<float>(0);
        for(int i = 0; i < a.cols; i ++)
            sqd += (a_ptr[i  ] - b_ptr[i  ])*(a_ptr[i  ] - b_ptr[i  ]);
        return sqd;
    }
}


void DescManip::fromString(cv::Mat &a, const std::string &s)
{

    //check if the dbow3 is present
    string ss_aux;ss_aux.reserve(10);
    for(size_t i=0;i<10 && i<s.size();i++)
        ss_aux.push_back(s[i]);
    if(ss_aux.find("dbw3")==std::string::npos){//is dbow2
        //READ UNTIL END
        stringstream ss(s);
        int val;
        vector<uchar> data;data.reserve(100);
        while( ss>>val) data.push_back(val);
        //copy to a
        a.create(1,data.size(),CV_8UC1);
        memcpy(a.ptr<char>(0),&data[0],data.size());
    }
    else {
        char szSign[10];
        int type,cols;
        stringstream ss(s);
        ss >> szSign >> type >> cols;
        a.create(1,  cols, type);
        if(type==CV_8UC1){
            unsigned char *p = a.ptr<unsigned char>();
            int n;
            for(int i = 0; i <  a.cols; ++i, ++p)
                if ( ss >> n) *p = (unsigned char)n;
        }
        else{
            float *p = a.ptr<float>();
            for(int i = 0; i <  a.cols; ++i, ++p)
                if ( !(ss >> *p))cerr<<"Error reading. Unexpected EOF. DescManip::fromString"<<endl;
        }

    }

}


void DescManip::fromStream(cv::Mat &m,std::istream &str){
    int type,cols,rows;
    str.read((char*)&cols,sizeof(cols));
    str.read((char*)&rows,sizeof(rows));
    str.read((char*)&type,sizeof(type));
    m.create(rows,cols,type);
    str.read((char*)m.ptr<char>(0),m.elemSize()*m.cols);
}


// --------------------------------------------------------------------------

} // namespace DBoW3

