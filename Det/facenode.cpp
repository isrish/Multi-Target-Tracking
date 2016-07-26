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
#include "facenode.h"


FaceNode::FaceNode()
{
    group_threshold_ = 1;
    minSize_ = cv::Size(40,40);
    maxSize_ = cv::Size(200,200);
    scaleFactor_ = 1.5;
    minNeighbors_ = 3;
    det_std_x_ = 0.8; //  w*det_std_x
    det_std_y_ = 0.8;
    det_std_h_ = 0.4;
    face_cascade_name_ = "../model/haarcascade_frontalface_alt.xml";
    if( !impl_cpu_face_.load(face_cascade_name_ ) )
    {
        printf("--(!)Error loading\n");
        exit(0);
    }
}


void FaceNode::setParameter(const std::string &name, const std::string &value)
{
    if(name == "face_minNeighbors"){
        minNeighbors_ = boost::lexical_cast<int>(value);
    }
    else if(name =="face_scaleFactor"){
        scaleFactor_ = boost::lexical_cast<double>(value);
    }
    else if(name == "face_group_threshold"){
        group_threshold_ = boost::lexical_cast<int>(value);
    }

}

void FaceNode::setData(const void *data, const std::string &type)
{
    if(type == "image_mono")
        img_mono_ = *(cv::Mat*)data;
    else if(type=="image_rgb")
        img_rgb_ = *(cv::Mat*)data;
}


void FaceNode::preprocess()
{

    impl_cpu_face_.detectMultiScale(img_rgb_, found_, scaleFactor_,minNeighbors_,0,minSize_,maxSize_);
    org_found_ = found_;
    double cx,cy;
    std::vector<cv::Rect>::iterator it;
    for(it=found_.begin(); it < found_.end();it++)
    {
        cx = (*it).x + (double)(*it).width/2;
        cy = (*it).y + (double)(*it).height/2;
        (*it).width *= 0.85;
        (*it).height *= 1.05;
        (*it).x = cx - (*it).width / 2;
        (*it).y -= (*it).height / 10;
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


void FaceNode::quaryData(const std::string &name, void *data)
{
    if(name == "face_detection")
        *((std::vector<cv::Rect>*)data) = org_found_;
    if(name == "face_ub_detection")
        *((std::vector<cv::Rect>*)data) = found_;
}


std::vector<cv::Rect> FaceNode::getDetections()
{
    return found_;
}


double FaceNode::getConfidence(const cv::Rect &rt)
{
    double ret = 0.0, temp;

    std::vector<cv::Rect>::iterator it;
    for(it = found_.begin(); it < found_.end(); it++)
    {
        cv::Rect orect = rt & (*it); // (rectangle intersection)
        cv::Rect urect = rt | (*it); // minimum area rectangle containing rt and (*it))

        temp = (double)(orect.width * orect.height) / (urect.width * urect.height);
        if(temp > ret)
            ret = temp;
    }

    return ret;
}

