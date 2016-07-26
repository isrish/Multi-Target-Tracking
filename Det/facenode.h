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
#ifndef FACENODE_H
#define FACENODE_H
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>


class FaceNode
{
public:
    FaceNode();
    std::vector<cv::Rect> getDetections();
    void init();
    void preprocess();
    void setData(const void *data, const std::string &type);
    double getConfidence(const cv::Rect &rt);
    void setParameter(const std::string &name, const std::string &value);
    void quaryData(const std::string &name, void *data);
private:
    std::string face_cascade_name_;
    cv::Mat img_mono_;
    cv::Mat img_rgb_;
    std::vector<cv::Rect> found_;
    std::vector<cv::Rect> org_found_;
    cv::CascadeClassifier impl_cpu_face_;
    int group_threshold_;
    double scaleFactor_ ;  // Parameter specifying how much the image size is reduced at each image scale.
    int minNeighbors_ ;  // Parameter specifying how many neighbors each candidate rectangle should have to retain it.
    cv::Size minSize_;  // Minimum possible object size. Objects smaller than that are ignored.
    cv::Size maxSize_; // maxSize – Maximum possible object size. Objects bigger than that are ignored.
    double det_std_x_;
    double det_std_y_;
    double det_std_w_;
    double det_std_h_;
public:
    double getdet_std_x_(){return det_std_x_;}
    double getdet_std_y_(){return det_std_y_;}
    double getdet_std_w_(){return det_std_w_;}
    double getdet_std_h_(){return det_std_h_;}
    void setdet_std_x_(double val){det_std_x_ = val;}
    void setdet_std_y_(double val){det_std_y_ =val;}
    void setdet_std_w_(double val){det_std_w_ =val;}
    void setdet_std_h_(double val){det_std_h_ =val;}
};



#endif // FACENODE_H

