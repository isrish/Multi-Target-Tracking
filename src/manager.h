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

#ifndef MANAGER_H
#define MANAGER_H

#include <string.h>
#include <stdio.h>
#include <istream>
#include <iostream>
#include <set>
#include <map>
#include <utility>
#include <vector>
#include <iomanip>
#include <boost/make_shared.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "Rect.h"
#include "Config.h"
#include <vector>
#include <opencv/cv.h>

#include "Rect.h"
#include "Sampler.h"
#include "Config.h"
#include "ImageRep.h"
#include "RawFeatures.h"
#include "HaarFeatures.h"
#include "HaarFeature.h"
#include "HistogramFeatures.h"
#include "MultiFeatures.h"
#include "Kernels.h"
#include "LaRank.h"
#include "logging.h"
#include "utilityfun.h"

class Features;
class Kernel;
class LaRank;
class ImageRep;

class Target;
class Manager;

typedef boost::shared_ptr<Target> TargetPtr;
typedef boost::shared_ptr<Manager> ManagerPtr;

typedef std::map<int,TargetPtr> Tracklet;
typedef std::map<int,Tracklet> Tracklets;
template <typename T>
inline const typename T::key_type& last_key(const T& pMap)
{
  // in map (and sets) the first element is the smallest, and the last element is the largest.
  return pMap.rbegin()->first;
}
template <typename T>
inline const typename T::value_type& last_value(const T& pMap)
{
  // in map (and sets) the first element is the smallest, and the last element is the largest.
  return pMap.rbegin()->second;
}

enum TargetState
{
  TargetTracking=0,
  TargetTerminated,
  TargetPaused,
  TargetNew,
  TargetStateNum,
};

static const char * EnumStrings[] = { "TargetTracking", "TargetTerminated", "TargetPaused" ,"TargetNew"};
const char * getTextForEnum( const int enumVal )
{
  return EnumStrings[enumVal];
}


class Target
{
public:
  Target(int id,int initFrame):ID_(id),tracking_init_time_(initFrame), count_(0),locconfidence_( 0.0),hisconfidence_(0.0){}
  inline int getID(){return ID_;}

  inline TargetState getStatus()const{ return status_;}
  inline void setStatus(const TargetState &state){ status_=state;}

  inline void setLearning(const bool &l){learning_ = l;}
  inline bool getLearning()const{return learning_;}

  inline void setdetBB(const cv::Rect &bb){detBB_ = bb;}
  inline cv::Rect getdetBB()const{return detBB_;}

  inline int getLastDetTime()const{return last_det_ass_time_;}
  inline void setLastDetTime(const int &fr){last_det_ass_time_=fr;}

  inline bool getdetStatus()const{return is_det_available_;}
  inline void setdetStatus(const bool &det){is_det_available_=det;}

  inline void setLastTrackingTime(const int &l){last_tracking_time_= l;}
  inline int getLastTrackingTime()const{return last_tracking_time_;}

  inline void setBB(const FloatRect &bb){initBB_ = bb;}
  inline FloatRect getBB() const { return initBB_;}
  inline cv::Rect getBB_cv() const { return getCVRect(initBB_);}

  inline void  updateTrackletBB(){trkBB_.push_back(initBB_);status_=TargetTracking;}
  inline void setDetectionCounter(){count_++;}
  inline void sethisConfidence(const double &conf){ hisconfidence_ = conf;}
  inline double getlocConfidence()const{ return locconfidence_;}
  inline double gethisConfidence()const{ return hisconfidence_;}
  inline int getDetectionCounter(){return count_;}
  inline int getNumberofFrameTracked(){return trkBB_.size();}
  inline void setConf(ConfigPtr conf){m_config_ = *conf;}
  inline bool IsInitialised() const { return m_initialised_; }
  inline void setInitFlag(bool f){m_initialised_ = f;}

  inline void clearInteractionsMap(){interactions_.clear();}
  inline void addInteraction(const int id,const float &pot){interactions_.insert(std::make_pair(id,pot));}

  inline void addFeature(std::string fea_type,const void *data);
  inline void addKernel(std::string ker_type,const void *data);
  inline int  getFeatureCount(){return m_features.back()->GetCount();}
  inline std::vector<Features*> getFeatures(){return m_features;}
  inline std::vector<Kernel*> getKernels(){return m_kernels;}

  void setLearner();
  void UpdateLearner(const ImageRep& image);
  void Track();

  float getMinInteractionPotentail();
  void  Evaluate(const MultiSample sample, std::vector<double>&scores);
  void printTargetSummary();


private:
  bool learning_;
  bool is_det_available_;
  bool m_initialised_;
  int ID_;
  int count_;
  int last_det_ass_time_;
  int tracking_init_time_;
  int last_tracking_time_;
  double locconfidence_;
  double hisconfidence_;
  Config  m_config_;
  TargetState status_;
  cv::Rect detBB_;
  FloatRect initBB_;
  std::vector<FloatRect> trkBB_;
  std::map<int, float> interactions_;
  std::vector<Features*> m_features;
  std::vector<Kernel*> m_kernels;
  LaRank* m_pLearner;
public:
  // this is used to access other tragets in a multi-target tracking case
  // we used to do mining for negative samples
  // std::shared_ptr<Manager> parent_manger_;
  Manager *parent_manger_;

};

class Manager : public std::enable_shared_from_this<Manager>
{
public:
  static const int MAXTARGET = 1000;
  Manager():Framecount_(0),MAXID_(1000){}
  Manager(const Config& conf);
  ~Manager(){}
  std::shared_ptr<Manager> make_parent() // can only be created as shared_ptr
  {
    return shared_from_this();
  }


  TargetPtr setNewTarget(int id);
  void setData(const void *data, const std::string &type);
  void getData(void *data, const std::string &type);
  void setDetections(const std::vector<cv::Rect> dets);
  void preprocess(const std::vector<int>& ids = std::vector<int>());
  void setProposals();
  void setProposalsFromID(std::vector<int>ids);
  void MultiTargetTracking();
  void RefreshTargetTracking();
  void FilterTargetsTracking();
  void getTargetsCVRectBB_and_ID(std::vector<cv::Rect>&bb, std::vector<int>&id);
  void printManagerSummary();
  Tracklet getOthers(const int id, const int frame=-1);
  ImageRep image_rep_;
protected:
  void InteractionPotentail();
  void InitilizeTrack(FloatRect initbb,int tragetID=-1);
  std::vector<double> computeDistance(TargetPtr trackingTarget, std::vector<cv::Rect> det);
  std::vector<double> computeDistanceOV(TargetPtr trackingTarget, std::vector<cv::Rect> det);
  double minDistance(std::map<int,TargetPtr> trackingTargets,cv::Rect det);
protected:
  Tracklet tracklet_new_;
  Tracklet tracklet_deleted_;
  Tracklet tracklet_tracking_;
  Tracklets tracks_;

  std::vector<cv::Rect> detections_;
  std::vector<cv::Rect> new_track_propsals_;
  std::vector<int> new_track_propsals_Id_;

  ConfigPtr m_config_;
  cv::Mat image_color_;
  cv::Mat image_gray_;
  int numFeatures_;
  bool m_needsIntegralImage_;
  bool m_needsIntegralHist_;
  bool m_needsRaw_;
  int Framecount_;
  int numActiveTargets_;
  int MAXID_;

};

#endif // MANAGER_H
