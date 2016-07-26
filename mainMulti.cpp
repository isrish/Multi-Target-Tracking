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


#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "facenode.h"
#include "upperbodynode.h"
#include "manager.h"
#include "utilityfun.h"
#include "Config.h"
#include "logging.h"
//#include "detectionreader.h"
#include "detectionreaderwithID.h"
#include "resultwriter.h"


void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour);
cv::Mat draw_detections(std::vector<cv::Rect> &dets,cv::Mat img_color,std::string st="D");
cv::Mat draw_detections(std::vector<cv::Rect> &dets,std::vector<int> ids,cv::Mat img_color,std::string st="D");
cv::Mat draw_avTrackingResult(std::vector<cv::Rect> &dets,std::vector<int> ids,int acvt_idx,cv::Mat img_color,std::string st="D");
cv::Mat draw_ssl(std::vector<double> ssl,cv::Mat img_color);
cv::Scalar get_target_color(int id);


int main(int argc, char* argv[])
{
    std::string configPath = "config.txt";
    if (argc > 1)
    {
        configPath = argv[1];
    }
    Config conf(configPath);
    std::cout << conf << std::endl;
    if (conf.features.size() == 0)
    {
        printerror("error: no features specified in config");
        return EXIT_FAILURE;
    }

    int frame_cnt = 0;
    double timesec = 0.0;
    double fps = 25.0;

    boxreadwriter  write_(conf.outputFilePath,conf.outputFileName,writer);
    runtime rt;
    DetectionFileReaderwithID detReader(conf.detectionFileName);
    Manager trkManager(conf);

    std::vector<cv::Rect> resultBBs;
    std::vector<int> resultIDs;

    std::vector<cv::Rect> det_rts_;
    std::vector<int> det_ids_;
    cv::Mat frame,frameOrig,frame_detection;
    cv::Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);

    //    cv::VideoCapture cap;
    //    if(!(cap.open(conf.sequenceFileName)))
    //    {
    //        printerror("Error:opening AVI file!");
    //        return EXIT_FAILURE;
    //    }


    if (conf.saveImage)
    {
        cv::namedWindow("result");
        cv::namedWindow("Detection");
        cv::moveWindow("result",10,10);
        cv::moveWindow("Detection",10,380);
    }



    for (frame_cnt=conf.frameStart;frame_cnt<=conf.frameEnd;++frame_cnt)
    {
        char buf[50];
        std::sprintf(buf, "Img-%06d.png", frame_cnt);
        std::string fn = conf.sequenceFileName + std::string(buf);
        frameOrig = cv::imread(fn.c_str());

        timesec = (double)frame_cnt/fps;
        /**  Image and Detections  **/
        {
            if (conf.frameWidth != frameOrig.cols || conf.frameHeight != frameOrig.rows)
            {
                std::cout<<fn<<":"<<conf.frameWidth<<"x"<<conf.frameHeight<<std::endl;
                cv::resize(frameOrig,frame,cv::Size(conf.frameWidth,conf.frameHeight));//resize image
            }
            else
                frameOrig.copyTo(frame);
            det_rts_.clear();
            detReader.getDetection(frame_cnt,det_rts_,det_ids_);
            detReader.postprocessDetection("nms",det_rts_);
            //detReader.postprocessDetection("size",det_rts_);
            //detReader.postprocessDetection("scale",det_rts_);
            //detReader.postprocessDetection("resize",det_rts_,2.666,2.666);
            detReader.postprocessDetection("bcheck",det_rts_,conf.frameHeight,conf.frameWidth);
        }

        frame.copyTo(result);
        rt.start();
        /** tracking part **/
        {
            trkManager.setData(&frame,"img_color");
            trkManager.setDetections(det_rts_);
            trkManager.preprocess(det_ids_);
            trkManager.MultiTargetTracking();
            trkManager.FilterTargetsTracking();
            trkManager.RefreshTargetTracking();

        }
        rt.end();
        // in quite mode, don't print anything
        if(!conf.quietMode)
        {
            trkManager.printManagerSummary();
            printotherBgred("Tracking part took: "+ rt.getsDuration() +" secs");
            printotherBggreen("\rFRAME-" + std::to_string(frame_cnt));
            printwarning("_________________________________________________");
        }

        /** Get tracking result bounding boxes **/
        trkManager.getTargetsCVRectBB_and_ID(resultBBs,resultIDs);

        /** write bounding box **/
        write_.writebb(frame_cnt,resultBBs,resultIDs);

        /** Draw **/
        if (conf.saveImage)
        {
            result =  draw_detections(resultBBs,resultIDs,result,"T");
            frame_detection =  draw_detections(det_rts_,frame,"F");
            cv::resize(result,result,cv::Size(640,360));
            cv::resize(frame_detection,frame_detection,cv::Size(640,360));
            cv::imshow("result",result);
            cv::imshow("Detection", frame_detection);
            if (cv::waitKey(1) == 27)
                break;

        }

    }

    return EXIT_SUCCESS;
}


cv::Mat draw_detections(std::vector<cv::Rect> &bbs,std::vector<int> ids,cv::Mat img_color,std::string st)
{
    cv::Mat img = img_color.clone();
    int thickline = floor(img.cols/250);
    double alpha = 0.5;
    for (int i = 0;i< (int)bbs.size();i++)
    {
        // keep bbs inside the image
        if(bbs[i].x +  bbs[i].width >= img.cols)
            bbs[i].width = img.cols- bbs[i].x- 15.0;
        if(bbs[i].x<=0)
            bbs[i].x = 15.0;
        if(bbs[i].y +  bbs[i].height >= img.rows)
            bbs[i].height = img.rows- bbs[i].y- 15.0;
        if(bbs[i].y<=0)
            bbs[i].y = 15.0;

        cv::Mat roi = img(bbs[i]);
        cv::Mat color(roi.size(), CV_8UC3, get_target_color(ids[i]+3));
        cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi);
        std::ostringstream text;
        text << "["<<st << ids[i]<< "]";
        cv::rectangle(img,bbs[i],get_target_color(ids[i]+23),thickline);
        setLabel(img,text.str(), bbs[i].tl());
    }
    return img;
}

cv::Mat draw_detections(std::vector<cv::Rect> &bbs,cv::Mat img_color,std::string st)
{
    cv::Mat img = img_color.clone();
    int thickline = floor(img.cols/250);
    double alpha = 0.5;
    for (int i = 0;i< (int)bbs.size();i++)
    {
        // keep bbs inside the image
        if(bbs[i].x +  bbs[i].width >= img.cols)
            bbs[i].width = img.cols- bbs[i].x- 5.0;
        if(bbs[i].x<=0)
            bbs[i].x = 5.0;
        if(bbs[i].y +  bbs[i].height >= img.rows)
            bbs[i].height = img.rows- bbs[i].y- 5.0;
        if(bbs[i].y<=0)
            bbs[i].y = 5.0;

        cv::Mat roi = img(bbs[i]);
        cv::Mat color(roi.size(), CV_8UC3, get_target_color((int)i+3));
        cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi);
        std::ostringstream text;
        text << "["<<st << i<< "]";
        cv::rectangle(img,bbs[i],get_target_color((int)i+23),thickline);
        setLabel(img,text.str(), bbs[i].tl());
    }
    return img;
}

cv::Mat draw_avTrackingResult(std::vector<cv::Rect> &bbs,std::vector<int> ids,int act_idx,cv::Mat img_color,std::string st)
{
    cv::Mat img = img_color.clone();
    int thickline = floor(img.cols/250);
    double alpha = 0.5;
    for (int i = 0;i<(int)bbs.size();i++)
    {
        // keep bbs inside the image
        if(bbs[i].x +  bbs[i].width >= img.cols)
            bbs[i].width = img.cols- bbs[i].x- 10.0;
        if(bbs[i].x<=0)
            bbs[i].x = 5.0;
        if(bbs[i].y +  bbs[i].height >= img.rows)
            bbs[i].height = img.rows- bbs[i].y- 10.0;
        if(bbs[i].y<=0)
            bbs[i].y = 5.0;

        cv::Mat roi = img(bbs[i]);
        cv::Mat color(roi.size(), CV_8UC3, get_target_color((int)i+3));
        cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi);
        if(i==act_idx)
        {
            int cx = bbs[i].x + 0.5*bbs[i].width;
            int cy = bbs[i].y + 0.5*bbs[i].height;
            int r =  std::max(bbs[i].width,bbs[i].height);
            cv::circle(img, cv::Point(cx, cy), r, cv::Scalar(200,0,0), -1, CV_AA);
        }
        else
        {
            cv::rectangle(img,bbs[i],get_target_color((int)i+23),thickline);
        }
        std::ostringstream text;
        text << "["<<st << ids[i]<< "]";
        setLabel(img,text.str(), bbs[i].tl());
    }
    return img;
}

void rectangle(cv::Mat& rMat, const FloatRect& rRect, const cv::Scalar& rColour)
{
    IntRect r(rRect);
    cv::rectangle(rMat, cv::Point(r.XMin(), r.YMin()), cv::Point(r.XMax(), r.YMax()), rColour);
}

cv::Scalar get_target_color(int id)
{
    cv::Scalar color = cv::Scalar(((id * 120) % 256), ((id * 60) % 256), ((id * 30) % 256));
    return color;
}

cv::Mat draw_ssl(std::vector<double> ssl,cv::Mat img_color)
{
    cv::Mat img = img_color.clone();
    cv::circle(img, cv::Point(ssl[0], ssl[1]), 10, cv::Scalar(200,0,0), 2,  CV_AA);
    cv::line(img, cv::Point(ssl[0],std::max(ssl[1]-100.0,10.0)), cv::Point(ssl[0], std::min(ssl[1]+100.0,200.0)), cv::Scalar(200,0,0), 4, CV_AA);
    return img;
}

