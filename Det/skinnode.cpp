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

#include "skinnode.h"

SkinNode::SkinNode()
{
    hval_lb_ = 2;
    hval_ub_ = 15;
    sval_lb_ = 60;
    sval_ub_ = 200;
    vval_lb_ = 40;
    vval_ub_ = 200;
    weight_ = 1.0;
    node_type_ = "skin_detector";
}

SkinNode::~SkinNode()
{
}

void SkinNode::setParameter(const std::string &name, const std::string &value)
{
    if(name == "skin_weight") weight_ = boost::lexical_cast<double>(value);
    if(name == "skin_hval_lb")	hval_lb_ = boost::lexical_cast<unsigned char>(value);
    if(name == "skin_hval_ub")	hval_ub_ = boost::lexical_cast<unsigned char>(value);
    if(name == "skin_sval_lb")	sval_lb_ = boost::lexical_cast<unsigned char>(value);
    if(name == "skin_sval_ub")	sval_ub_ = boost::lexical_cast<unsigned char>(value);
    if(name == "skin_vval_lb")	vval_lb_ = boost::lexical_cast<unsigned char>(value);
    if(name == "skin_vval_ub")	vval_ub_ = boost::lexical_cast<unsigned char>(value);
}

void SkinNode::setData(const void *data, const std::string& type)
{
    if(type == "image_hsv")
        img_hsv_ = *(cv::Mat*)data;
}

void SkinNode::preprocess()
{
    cv::Mat temp_mat(img_hsv_.rows, img_hsv_.cols, CV_8U);
    for(int i = 0; i < img_hsv_.rows; i++) {
        for(int j = 0; j < img_hsv_.cols; j++) {
            unsigned char hval = img_hsv_.at<char>(i, 3 * j);
            unsigned char sval = img_hsv_.at<char>(i, 3 * j + 1);
            unsigned char vval = img_hsv_.at<char>(i, 3 * j + 2);

            if((hval >= hval_lb_) && (hval <= hval_ub_) && (sval >= sval_lb_) && (sval <= sval_ub_) && (vval >= vval_lb_) && (vval <= vval_ub_))
                temp_mat.at<unsigned char>(i, j) = 1;
            else
                temp_mat.at<unsigned char>(i, j) = 0;
        }
    }
    cv::medianBlur(temp_mat, img_skin_, 3);
    cv::Mat temp = img_skin_.clone() * 100;
    //cv::imshow("skin pixels", temp);
    //cv::imwrite("fname.jpg",temp);
}

double SkinNode::getConfidence(const cv::Rect &rt)
{
    double ret = 0;
    cv::Rect roi = rt;
    float stepx = (float)(roi.width-1) / 10;
    float stepy = (float)(roi.height-1) / 10;

    for(float x = roi.x, i = 0; i < 10 ; x += stepx, i++)
    {
        for(float y = roi.y, j = 0; j < 10 ; y += stepy, j++)
        {
            int ix = floor(x), iy = floor(y);
            if((ix < 0) || (iy < 0))
                continue;
            ret += (float)img_skin_.at<unsigned char>(iy, ix) / 50;

        }
    }

    return ret * weight_;
}
