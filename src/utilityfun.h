/**
* Software License Agreement (BSD License)
*
*           Copyright (C) 2015 by Israel D. Gebru,
*           Perception Team, INRIA-Grenoble, France
*                   All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the FreeBSD Project.
*/

#ifndef UTILITYFUN_H
#define UTILITYFUN_H

#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <string.h>
#include <opencv2/core/core.hpp>
#include "Rect.h"
#include "logging.h"

#define PI 3.14159265
typedef unsigned long long timestamp_t;

class runtime
{
public:
    runtime(){}
private:
    timestamp_t start_;
    double duration_;
protected:
    static timestamp_t get_timestamp ()
    {
        struct timeval now;
        gettimeofday (&now, NULL);
        return  now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
    }
public:
    inline void start()
    {
        start_ = get_timestamp();
    }
    inline void end(){duration_ = (get_timestamp()- start_)/1000000.0L;}
    inline double getDuration()const{return duration_;}
    inline std::string getsDuration()const{return std::to_string(duration_);}
    friend std::ostream& operator << (std::ostream& os, runtime &rt)
    {
        os<<std::to_string(rt.getDuration());
        return os;
    }
};

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

std::string getTimeStamp()
{
    char            fmt[64], buf[64];
    struct timeval  tv;
    struct tm       *tm;

    gettimeofday(&tv, NULL);
    if((tm = localtime(&tv.tv_sec)) != NULL)
        {
            strftime(fmt, sizeof fmt, "%H:%M:%S.%%06u", tm); //"%Y-%m-%d %H:%M:%S.%%06u %z"
            snprintf(buf, sizeof buf, fmt, tv.tv_usec);
        }
    return std::string(buf);
}


void setLabel(cv::Mat &im, const std::string &name, const cv::Point2f &og)
{
    int face[] = {cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX,
                  cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                  cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC};
    int fontface = face[0];
    double scale = 0.3;
    int thickness = 1;
    int baseline = 0.1;

    cv::Size text = cv::getTextSize(name, fontface, scale, thickness, &baseline);
    cv::rectangle(im, og + cv::Point2f(0, baseline), og + cv::Point2f(text.width, -text.height), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(im, name, og, fontface, scale, CV_RGB(255,255,0), thickness, CV_AA);

}

std::vector<cv::Rect> nmsRects(const std::vector<cv::Rect> &dets, double overlapThresh)
{
    std::vector<cv::Rect> ret;
    std::vector<int> picked;
    if(dets.size() ==0)
        return ret;
    int num_dets = dets.size();
    std::vector<double> x1, y1,x2,y2,area;
    //grab the coordinates of the bounding boxes
    for(int i= 0; i<num_dets;i++)
        {
            x1.push_back(dets[i].x);
            x2.push_back(dets[i].x + dets[i].width);
            y1.push_back(dets[i].y);
            y2.push_back(dets[i].y + dets[i].height);
            double tmp = dets[i].width * dets[i].height;
            area.push_back(tmp);
        }
    // sort the bounding boxes by the bottom-right y-coordinate of the bounding box
    std::vector<size_t> idx(num_dets);
    for (size_t i = 0; i != idx.size(); ++i)
        idx[i] = i;
    std::sort(idx.begin(), idx.end(),[y2](size_t i1, size_t i2) {return y2[i1] < y2[i2];});
    //    for(int i=0;i<num_dets;i++)
    //        std::cout<<y2[idx[i]]<<std::endl;

    std::vector<size_t> idx_new = idx;
    // keep looping while some indexes still remain in the indexes
    while(idx.size()>0)
        {
            //grab the last index in the indexes list and add the index value to the list of picked indexes
            int last = idx.size() - 1;
            int i = idx[last];
            picked.push_back(i);
            // find the largest (x, y) coordinates for the start of
            // the bounding box and the smallest (x, y) coordinates
            // for the end of the bounding box
            // vectorizing magic here!!
            double overlap;
            std::vector<double> to_remove;
            to_remove.push_back(last);
            double x1_sort,y1_sort,x2_sort,y2_sort,area_sort;
            for(int j=0;j<last;j++)
                {
                    x1_sort = x1[idx[j]];
                    x2_sort = x2[idx[j]];
                    y1_sort = y1[idx[j]];
                    y2_sort = y2[idx[j]];
                    area_sort = area[idx[j]];
                    double xx1 = std::max(x1[i], x1_sort);
                    double yy1 = std::max(y1[i], y1_sort);
                    double xx2 = std::min(x2[i], x2_sort);
                    double yy2 = std::min(y2[i], y2_sort);

                    // compute the width and height of the bounding box
                    double w = std::max(0.0, xx2 - xx1 + 1.0);
                    double h = std::max(0.0, yy2 - yy1 + 1.0);
                    //compute the ratio of overlap
                    overlap = (w * h) / area_sort;
                    // delete all indexes from the index list that have less overlap
                    if(overlap> overlapThresh)
                        to_remove.push_back(j);
                }
            idx_new.clear();
            for(int k=0; k<(int)idx.size();++k)
                {
                    if(find(to_remove.begin(),to_remove.end(),k) == to_remove.end())
                        idx_new.push_back(idx[k]);
                }
            idx.clear();
            idx = idx_new;
        }
    for(int i = 0 ;i<(int)picked.size();++i)
        ret.push_back(dets[picked[i]]);

    return ret;
}

float bb_overlap(const cv::Rect &rt1,const cv::Rect &rt2)
{
    float ret =0.0;
    float tmp1 = std::max(rt1.x,rt2.x);
    float tmp2 = std::max(rt1.y,rt2.y);
    float tmp3 = std::min(rt1.x+rt1.width, rt2.x+rt2.width);
    float tmp4 = std::min(rt1.y+rt1.height,rt2.y+rt2.height);

    float w = tmp3-tmp1+1.0;
    float h = tmp4-tmp2+1.0;
    float inter = w*h;
    if((w>0) && (h>0))
        ret = inter / (rt1.area() + rt2.area()-inter);
    return ret;
}

std::vector<float>  bb_dist(const cv::Rect &rt1,const cv::Rect &rt2)
{
    std::vector<float> ret;
    float tmp1 = std::abs(rt1.x - rt2.x);
    float tmp3 = rt1.width + rt2.width;
    ret.push_back((tmp1-tmp3));
    tmp1 = std::abs(rt1.y - rt2.y);
    tmp3 = rt1.height + rt2.height;
    ret.push_back((tmp1-tmp3));
    return ret;
}

template <typename T>
T normal_pdf(T x, T m, T s)
{
    static const T inv_sqrt_2pi = 0.3989422804014327;
    T a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-T(0.5) * a * a);
}

double gaussian_prob(double x, double m, double std)
{
    double var = std * std;
    if(std == 0) return 1.0;
    return 1 / sqrt(2 * M_PI * var) * exp(- pow(x - m, 2) / (2 * var));
}

double log_gaussian_prob(double x, double m, double std)
{
    if(std == 0) return 0.0; // no effect!
    return -log(sqrt(2 * M_PI) * std) - ( pow((x - m) / std, 2) / 2.0 );
}

double log_gaussian_prob(cv::Mat &x, cv::Mat& m, cv::Mat &icov, double det)
{
    assert(x.rows == m.rows);
    assert(x.rows == icov.cols);
    cv::Mat dx = x - m;
    cv::Mat temp = ( (dx.t() * icov) * dx  / 2.0 );
    assert(temp.rows == 1 && temp.cols == 1);
    double ret =  - x.rows * log(2 * M_PI) / 2 - log(det) / 2  - temp.at<double>(0, 0);
    return ret;
}

void logsumexp(std::vector<double> &log_prob)
{
    /** now log-exp-sum trick **/
    // maxof(zi)
    double m = *std::max_element(std::begin(log_prob), std::end(log_prob));
    // zi - maxof(zi)
    std::transform(log_prob.begin(), log_prob.end(), log_prob.begin(),bind2nd(std::minus<double>(), m));
    // exp(zi)
    std::transform(log_prob.begin(),log_prob.end(),log_prob.begin(),[](double d) -> double { return std::exp(d);});
    // sum of all (zi)
    double norm = 1.0/std::accumulate(log_prob.begin(),log_prob.end(),0); // norm const
    std::transform(log_prob.begin(),log_prob.end(),log_prob.begin(),std::bind1st(std::multiplies<double>(),norm)); // normalize
}

// this is to find the interaction potential between targets
typedef std::pair<int, float> MyPairType;
struct CompareSecond
{
    bool operator()(const MyPairType& left, const MyPairType& right) const
    {
        return left.second < right.second;
    }
};

FloatRect getFloatRect(const cv::Rect &rt)
{
    return FloatRect((float)rt.x,(float)rt.y,(float)rt.width,(float)rt.height);
}

cv::Rect getCVRect(const FloatRect &rt)
{
    cv::Rect ret(rt.XMin(),rt.YMin(),rt.Width(),rt.Height());
    return ret;
}

inline std::string BoolToString(bool b)
{
    return b ? "True" : "False";
}


void printvector(std::vector<double> vec,std::string st="")
{
    printf("%s",st.c_str());
    for(int i=0; i<(int)vec.size();++i)
        {
            printf(ANSI_COLOR_RED"[%d]:"ANSI_COLOR_RESET,i);
            printf(ANSI_COLOR_GREEN"%3.3f\n"ANSI_COLOR_RESET,vec[i]);
        }
}

#endif // UTILITYFUN_H

