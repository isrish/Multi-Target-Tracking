#ifndef DETECTIONREADER_H
#define DETECTIONREADER_H
#include "string.h"
#include "stdio.h"
#include "fstream"
#include <sstream>
#include <set>
#include <map>
#include <utility>
#include "opencv2/core/core.hpp"

typedef std::map<int,std::vector<cv::Rect>> detection_per_frame_set_;
// <frameID,<bbID,bb>>


class DetectionFileReader
{
public:
    DetectionFileReader(const std::string fn)
    {
        filename_ = fn;
        readDetection();
    }

    inline int getDetection(const int &id,std::vector<cv::Rect>&ret)
    {
        int iret = 0;
        ret.clear();
        detection_per_frame_set_::const_iterator pos = dets_.find(id    );
        if(pos != dets_.end())
            {
                ret.clear();
                ret = pos->second;
                iret = 1;
            }
        return iret;
    }

    inline int postprocessDetection(const std::string ptype, std::vector<cv::Rect>&ret,int w=0,int h=0)
    {
        int iret = 0;
        int numdet = ret.size();
        if(ptype=="scale")
            {
                for(int i=0;i<numdet;++i)
                    {
                        ret[i].height *=1.0;
                        ret[i].width *=0.95;
                    }
                iret = 1;
            }
        else if(ptype=="size")
            {
                std::vector<cv::Rect> newret;
                for(int i=0;i<numdet;++i)
                    {
                        if(ret[i].height <=80.0 && ret[i].width<=60.0)
                            newret.push_back(ret[i]);
                    }
                iret = 1;
                ret.clear();
                ret = newret;
            }
        else if(ptype=="bcheck")
            {
                if(w!=0 && h!=0)
                    {
                        std::vector<cv::Rect> newret;
                        for(int i=0;i<numdet;++i)
                            {
                                if((ret[i].x>=1.0) && (ret[i].y>=1.0) && (ret[i].height<=h) && (ret[i].width<=w))
                                    {
                                        newret.push_back(ret[i]);
                                    }
                            }
                        iret = 1;
                        ret.clear();
                        ret = newret;
                    }
            }
        else if(ptype=="nms")
            {
                std::vector<cv::Rect> newret;
                newret =  nmsRects(ret,0.5);
            }
        return iret;
    }

protected:
    inline void readDetection()
    {
        std::vector<std::vector<float>> alldet;
        int num_frame;
        readfile(num_frame,alldet);
        detection_per_frame_set_::const_iterator pos;

        for(int i=0; i< alldet.size();++i)
            {
                cv::Rect rect(alldet[i][1],alldet[i][2],alldet[i][3],alldet[i][4]);
                int fid = alldet[i][0];
                pos = dets_.find(fid);
                if(pos==dets_.end())
                    {
                        std::vector<cv::Rect> rt = {rect};
                        dets_.insert(std::make_pair(fid,rt));
                    }
                else
                    {
                        const_cast<std::vector<cv::Rect>&>(pos->second).push_back(rect);
                    }

            }
    }
    inline bool readfile(int &maxframe, std::vector<std::vector<float>>&ret)
    {
        maxframe = 0;
        std::ifstream infile(filename_);
        if(infile.is_open())
            {
                std::vector<float> rt;
                ret.clear();
                int frame,id;
                float x,y,z;
                float bb_left,bb_top,bb_width,bb_height,conf;
                char c1,c2,c3,c4,c5,c6,c7,c8,c9;
                while ((infile >>frame>>c1>>id>>c2>>bb_left>>c3>>bb_top>>c4>>bb_width>>c5>>bb_height>>c6>>conf>>c7>>x>>c8>>y>>c9>>z) &&
                       ((c1 == ',') && (c2 == ',') &&(c3 == ',') &&(c4 == ',') &&(c5 == ',') &&(c6 == ',') &&(c7 == ',') &&(c8 == ',')
                        &&(c9 == ',')))
                    {
                        rt.clear();
                        rt.push_back(frame);
                        rt.push_back(bb_left);
                        rt.push_back(bb_top);
                        rt.push_back(bb_width);
                        rt.push_back(bb_height);
                        ret.push_back(rt);
                        maxframe = maxframe<frame?frame:maxframe;
                    }
                infile.close();
            }
        else
            return false;
        return true;
    }

private:
    std::string filename_;
    std::string rootdir_;
    detection_per_frame_set_ dets_;
};

#endif // DETECTIONREADER_H

