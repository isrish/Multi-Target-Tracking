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

#include <boost/lexical_cast.hpp>
#include <iostream>
#include "upperbodynode.h"


UpperBodyNode::UpperBodyNode()
{
    group_threshold_ = 2;
    minSize_ = cv::Size(80,80);
    maxSize_ = cv::Size(200,280);
    scaleFactor_ = 1.2;
    minNeighbors_ = 3;
    ubody_cascade_name_ = "../model/haarcascade_mcs_upperbody.xml";
    det_std_x_ = 0.05;
    det_std_y_ = 0.1;
    det_std_h_ = 0.1;
    if( !impl_cpu_ubody_.load(ubody_cascade_name_ ) )
    {
        printf("--(!)Error loading\n");
        exit(0);
    }
}

void UpperBodyNode::setData(const void *data, const std::string &type)
{
    if(type == "image_mono")
        img_mono_ = *(cv::Mat*)data;
    else if(type=="image_rgb")
        img_rgb_ = *(cv::Mat*)data;

}

void UpperBodyNode::setParameter(const std::string &name, const std::string &value)
{

}

/**
 * @brief UpperBodyNode::preprocess
 */
void UpperBodyNode::preprocess()
{

    impl_cpu_ubody_.detectMultiScale(img_rgb_, found_, scaleFactor_,minNeighbors_,0,minSize_,maxSize_);
    org_found_ = found_;
    double cx,cy;
    std::vector<cv::Rect>::iterator it;
    for(it=found_.begin(); it < found_.end();it++)
    {
        cx = (*it).x + (double)(*it).width/2;
        cy = (*it).y + (double)(*it).height/2;
        (*it).width *= 0.33;
        (*it).height *= 0.5;
        (*it).x = cx  - (*it).width / 2;
        (*it).y = cy  - (*it).height / 2;
    }

    int nDetection = found_.size();
    if(nDetection>0)
    {
        for( int i = 0; i < nDetection; i++ )
        {
            found_.push_back(cv::Rect(found_[i]));
        }
        int groupThreshold = 1; // min number of rectangle minus one
        double eps_= 0.2; // relative difference between sides of the rectangles to merge them into a group.
        groupRectangles(found_, groupThreshold, eps_);
    }


}

/**
 * @brief UpperBodyNode::quaryData
 * @param name
 * @param data
 */
void UpperBodyNode::quaryData(const std::string &name, void *data)
{
    if(name == "up_detection")
        *((std::vector<cv::Rect>*)data) = org_found_;

}

/**
 * @brief UpperBodyNode::getDetections
 * @return
 */
std::vector<cv::Rect> UpperBodyNode::getDetections()
{
    return org_found_;

}

/**
 * @brief UpperBodyNode::clearDetections
 */
void UpperBodyNode::clearDetections()
{
    org_found_.clear();
    found_.clear();
}

std::vector<cv::Rect> UpperBodyNode::getfaceDetections()
{
    std::vector<cv::Rect> ret;
    if(org_found_.size()==0)
        return ret;

    int num_det = org_found_.size();
    for(int i=0;i<num_det;i++)
    {
        cv::Rect f;
        double w = org_found_[i].width * 0.30;
        double h = org_found_[i].height* 0.5;
        double x = org_found_[i].x + org_found_[i].width * 0.36 ;
        double y = org_found_[i].y + org_found_[i].height*0.16;
        // assuming face is found on the top 1/3 region
        f = cv::Rect(x,y,w,h);
        ret.push_back(f);
    }
    return ret;
}


