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

#include "sort.h"
#include "manager.h"
#include "utilityfun.h"
#include "assignmentoptimal.h"
#include <eigen2/Eigen/Core>
#include <opencv/highgui.h>
#include <float.h>
#include <omp.h>
#include <pthread.h>
#include <stdexcept>

void Target::addFeature(std::string fea_type,const void *data)
{
    if (fea_type=="Haar") //const Config con
        m_features.push_back(new HaarFeatures(*((Config*)data)));
    else if (fea_type=="Raw")
        m_features.push_back(new RawFeatures(*((Config*)data)));
    else if (fea_type=="Histogram")
        m_features.push_back(new HistogramFeatures(*((Config*)data)));
    else if (fea_type=="Multi")
        m_features.push_back((MultiFeatures*)data);
}

void Target::addKernel(std::string ker_type,const void *data)
{

    if (ker_type=="Linear")
        m_kernels.push_back(new LinearKernel());
    else if(ker_type=="Gaussian")
        m_kernels.push_back(new GaussianKernel(*((double*)data)));
    else if (ker_type=="Intersection")
        m_kernels.push_back(new IntersectionKernel());
    else if (ker_type=="Chi2")
        m_kernels.push_back(new Chi2Kernel());
    else if (ker_type=="Multi")
        m_kernels.push_back((MultiKernel*)data);
}




void Target::UpdateLearner(const ImageRep& image)
{
    // note these return the centre sample at index 0
    std::vector<FloatRect> rects = Sampler::RadialSamples(initBB_, 2*m_config_.searchRadius, 5, 16);
    //vector<FloatRect> rects = Sampler::PixelSamples(m_bb, 2*m_config.searchRadius, true);

    std::vector<FloatRect> keptRects;
    keptRects.push_back(rects[0]); // the true sample
    // now add negative samples from the other tracklets if there is any
    Tracklet other_tracklet = parent_manger_->getOthers(ID_);
    for(auto it=other_tracklet.begin(); it!=other_tracklet.end(); ++it)
    {
        keptRects.push_back(it->second->getBB());
    }
    for (int i = 1; i < (int)rects.size(); ++i)
    {
        if (!rects[i].IsInside(image.GetRect()))
            continue;
        keptRects.push_back(rects[i]);
    }

    MultiSample sample(image, keptRects);
    m_pLearner->Update(sample, 0);
}

void Target::Track()
{
    // interactions_pot is 1 -  overlap ratio
    float interactions_pot = getMinInteractionPotentail();
    std::vector<FloatRect> rects_both;
    std::vector<FloatRect> keptRects;
    bool half_sample = true;
    int scale = 1.0;

    if(interactions_pot<0.5f)
        scale = 2.0;

    if(is_det_available_)
    {
        std::vector<FloatRect> centers;
        centers.push_back(initBB_);
        centers.push_back(getFloatRect(detBB_));
        std::vector<double> prob = {0.2,0.8};
        rects_both = Sampler::PixelSamples(centers, prob,scale*m_config_.searchRadius,half_sample);
        //printinfo("Sample for T[" +  std::to_string(getID()) +"]:" + std::to_string(rects_both.size()));
    }
    else
    {
        rects_both = Sampler::PixelSamples(initBB_, scale * m_config_.searchRadius,half_sample);
    }

    keptRects.reserve(rects_both.size());

    for (int i = 0; i < (int)rects_both.size(); ++i)
    {
        if (!rects_both[i].IsInside(parent_manger_->image_rep_.GetRect()))
            continue;
        keptRects.push_back(rects_both[i]);
    }

    MultiSample sample(parent_manger_->image_rep_, keptRects);

    std::vector<double> scores;
    m_pLearner->Eval(sample, scores);
    double bestScore = -DBL_MAX;
    int bestInd = -1;

    for(int i=1;i<(int)keptRects.size();++i)
    {
        if(scores[i]>bestScore)
        {
            bestScore = scores[i];
            bestInd = i;
        }
    }

    if(is_det_available_)
        locconfidence_ += bestScore;



    if (bestInd != -1)
    {
        initBB_ = keptRects[bestInd];
        if(is_det_available_ && learning_)
        {
            UpdateLearner(parent_manger_->image_rep_);
        }
    }
}

void Target::Evaluate(const MultiSample sample, std::vector<double> &scores)
{
    m_pLearner->Eval(sample, scores);
}

void Target::printTargetSummary()
{
    printinfo("\tStatus:" + std::string(getTextForEnum(getStatus())));
    printinfo("\tNumber of Frame Tracked: "+ std::to_string(trkBB_.size()));
    printinfo("\tDetection Counter: "+ std::to_string(count_));
    printinfo("\tDetection Avaliable: " + BoolToString(is_det_available_));
    printinfo("\tConfidence: "+ std::to_string(hisconfidence_));
    printinfo("\tInteractions:");
    for (auto it=interactions_.begin();it!=interactions_.end();++it)
    {
        printother("\t\t" + std::to_string(it->first) +" <----> " + std::to_string(it->second));
    }
    printother("\tMin Interaction: " + std::to_string(getMinInteractionPotentail()));
}

void Target::setLearner()
{
    m_pLearner = new LaRank(m_config_, *(m_features.back()), *(m_kernels.back()));
}

float Target::getMinInteractionPotentail()
{
    if(interactions_.empty())
        return 10000.0;
    else
    {
        std::pair<int, float> min = *min_element(interactions_.begin(), interactions_.end(), CompareSecond());
        return min.second;
    }
}


Manager::Manager(const Config &conf):Framecount_(0),MAXID_(1000)
{
    m_config_  =  boost::make_shared<Config>(conf);
    numFeatures_ = m_config_->features.size();
    m_needsIntegralImage_ = false;
    m_needsIntegralHist_ = false;
    m_needsRaw_ = false;
    for(int i=0;i<numFeatures_;++i)
    {
        switch (m_config_->features[i].feature)
        {
        case Config::kFeatureTypeHaar:
            m_needsIntegralImage_ = true;
            break;
        case Config::kFeatureTypeRaw:
            m_needsRaw_ = true;
            break;
        case Config::kFeatureTypeHistogram:
            m_needsIntegralHist_ = true;
            break;
        }

    }
}

TargetPtr Manager::setNewTarget(int id)
{
    TargetPtr newTrg = boost::make_shared<Target>(Target(id,Framecount_));
    newTrg->setConf(m_config_);
    std::vector<int> featureCounts;
    Config m_config = *m_config_;
    for (int i = 0; i < numFeatures_; ++i)
    {
        switch (m_config_->features[i].feature)
        {
        case Config::kFeatureTypeHaar:
            newTrg->addFeature("Haar",&m_config);
            break;
        case Config::kFeatureTypeRaw:
            newTrg->addFeature("Raw",&m_config);
            break;
        case Config::kFeatureTypeHistogram:
            newTrg->addFeature("Histogram",&m_config);
            break;
        }

        featureCounts.push_back(newTrg->getFeatureCount());

        switch (m_config_->features[i].kernel)
        {
        case Config::kKernelTypeLinear:
            newTrg->addKernel("Linear",NULL);
            break;
        case Config::kKernelTypeGaussian:
        {
            double data = m_config.features[i].params[0];
            newTrg->addKernel("Gaussian",&data);
        }
            break;
        case Config::kKernelTypeIntersection:
            newTrg->addKernel("Intersection",NULL);
            break;
        case Config::kKernelTypeChi2:
            newTrg->addKernel("Chi2",NULL);
            break;
        }
    }

    if (numFeatures_ > 1)
    {
        MultiFeatures* f = new MultiFeatures(newTrg->getFeatures());
        newTrg->addFeature("Multi",f);
        MultiKernel* k = new MultiKernel(newTrg->getKernels(),featureCounts);
        newTrg->addKernel("Multi",k);
    }
    newTrg->setLearner();
    newTrg->setInitFlag(true);
    newTrg->setStatus(TargetNew);
    newTrg->setLearning(true);
    newTrg->parent_manger_ = this; // std::shared_ptr<Manager>(this);
    return newTrg;
}


void Manager::setData(const void *data, const std::string &type)
{
    if(type=="img_mono")
        image_gray_ = *((cv::Mat*)data);
    else if(type=="img_color")
        image_color_ = *((cv::Mat*)data);

}

void Manager::getData(void *data, const std::string &type)
{
    if(type=="img_mono")
        data =  (void*)&image_gray_;
    else if(type=="img_color")
        data =  (void*)&image_color_;
    else if(type=="img_rep")
        data =(void*)&image_rep_;
}

void Manager::setDetections(const std::vector<cv::Rect> dets)
{
    detections_ = dets;
}

void Manager::preprocess(const std::vector<int> &ids)
{
    ImageRep image(image_color_, m_needsIntegralImage_, m_needsIntegralHist_,1);
    image_rep_ = image;
    Framecount_++;
    if (!ids.empty())
        setProposalsFromID(ids);
    else
        setProposals();
}

void Manager::setProposalsFromID(std::vector<int> ids)
{
    Tracklet trackingTargets;
    // get the active tracklets from the previous frame
    try
    {
        trackingTargets = tracks_.at(Framecount_-1);
    }
    catch(const std::out_of_range& oor)
    {
        printerror("Nothing to track from the previous frame: ");
    }
    assert(ids.size()==detections_.size());
    new_track_propsals_.clear();
    new_track_propsals_Id_.clear();
    Tracklet::const_iterator pos;
    for(int i=0; i<ids.size();++i)
    {
        pos = trackingTargets.find(ids[i]);
        if (pos==trackingTargets.end())
        {
            // new id to add
            new_track_propsals_.push_back(detections_[i]);
            new_track_propsals_Id_.push_back(ids[i]);
        }
        else
        {
            // existing set the update
            assert(pos->second->getStatus() == TargetTracking);
            pos->second->setDetectionCounter();
            pos->second->setdetBB(detections_[i]);
            pos->second->setdetStatus(true);
            pos->second->setLastDetTime(Framecount_);
        }
    }
    // mark tracklets with no detections
    for(std::map<int,TargetPtr>::iterator it = trackingTargets.begin(); it!=trackingTargets.end(); ++it)
    {
        if (it->second->getLastDetTime()!=Framecount_)
            it->second->setdetStatus(false);
    }

}

void Manager::setProposals()
{
    int i=0,j=0;
    std::vector<int> associations;
    std::vector<cv::Rect> proposals;
    associations.clear();
    proposals.clear();
    Tracklet trackingTargets;
    // get the active tracklets from the previous frame
    try
    {
        trackingTargets = tracks_.at(Framecount_-1);
    }
    catch(const std::out_of_range& oor)
    {
        printerror("Nothing to track from the previous frame: ");
    }
    // build assignment matrix
    if(!trackingTargets.empty())
    {
        size_t num_targets = trackingTargets.size();
        // printerror("HA cost Matrix:");
        // printinfo("--------------------------------");
        float cost;
        float *assign = new float [num_targets];
        float *distMat = new float[num_targets*detections_.size()];
        i=0; j=0;
        for(std::map<int,TargetPtr>::iterator it = trackingTargets.begin(); it!=trackingTargets.end(); ++it)
        {
            std::vector<double> tmp = computeDistanceOV(it->second,detections_);
            for(std::vector<cv::Rect>::iterator it_det=detections_.begin();it_det!=detections_.end(); ++it_det)
            {
                // printerror("("+std::to_string(i)+","+ std::to_string(j) +") "+std::to_string(tmp[j]));
                distMat[i + num_targets*j] = (float)tmp[j];
                j+=1;
            }
            i+=1;
            j=0;
        }
        // printinfo("--------------------------------");
        // solve data association problem
        assignmentoptimal(assign,&cost,distMat,trackingTargets.size(),detections_.size(),10e1);
        // assign matched detections to proposal, for each target being track get the assiciated detection
        for(i = 0; i< (int)trackingTargets.size();++i)
        {
            int detid = (int)assign[i];
            associations.push_back(detid);
        }
        delete assign;
        delete distMat;
    }

    // handle the remaining non-mathced detections
    for(i=0; i< (int)detections_.size();++i)
    {
        // if the detection i is not in association vector add it
        if(std::find(associations.begin(),associations.end(),i)==associations.end())
            associations.push_back((int)i);
    }

    //allocate proposals
    for(i = 0;i<(int)associations.size();++i)
    {
        cv::Rect rt(0,0,0,0);
        if(associations[i]>=0)
        {
            rt = detections_[associations[i]];
        }
        proposals.push_back(rt);
    }

    i=0;
    for(std::map<int,TargetPtr>::iterator it = trackingTargets.begin();it!=trackingTargets.end();++it)
    {
        cv::Rect rt = proposals[i];
        if(!((rt.height==0) | (rt.width ==0) | (rt.x==0) | (rt.y==0)))
        {
            assert(it->second->getStatus() == TargetTracking);
            it->second->setDetectionCounter();
            it->second->setdetBB(rt);
            it->second->setdetStatus(true);
            it->second->setLastDetTime(Framecount_);
        }
        else
            it->second->setdetStatus(false);
        tracklet_tracking_.insert(std::make_pair(it->second->getID(),it->second));
        i+=1;
    }
    // now check if the remaining detection match with any targets deleted from the previous frame
    std::vector<cv::Rect> rem_detections;
    rem_detections.insert(rem_detections.begin(),proposals.begin()+ trackingTargets.size(),proposals.end());
    Tracklet prev_deleted_targets = tracklet_deleted_;
    // build assignment matrix
    associations.clear();
    proposals.clear();
    if(!prev_deleted_targets.empty() && !rem_detections.empty())
    {
        size_t num_del_targets = prev_deleted_targets.size();
        // printerror("HA cost Matrix:");
        // printinfo("--------------------------------");
        float cost;
        float *assign = new float [num_del_targets];
        float *distMat = new float[num_del_targets*rem_detections.size()];
        i=0,j=0;
        for(std::map<int,TargetPtr>::iterator it = prev_deleted_targets.begin(); it!=prev_deleted_targets.end(); ++it)
        {
            std::vector<double> tmp = computeDistance(it->second, rem_detections);
            for(std::vector<cv::Rect>::iterator it_det=rem_detections.begin();it_det!=rem_detections.end(); ++it_det)
            {
                // printerror("("+std::to_string(i)+","+ std::to_string(j) +") "+std::to_string(tmp[j]));
                distMat[i + num_del_targets*j] = (float)tmp[j];
                j+=1;
            }
            i+=1;
            j=0;
        }
        // printinfo("--------------------------------");
        // solve data association problem
        assignmentoptimal(assign,&cost,distMat, prev_deleted_targets.size(),rem_detections.size(),10e1);
        // assign matched detections to proposal, for each target being track get the assiciated detection
        for(i = 0; i<(int)prev_deleted_targets.size();++i)
        {
            int detid = (int)assign[i];
            associations.push_back(detid);
        }
        delete assign;
        delete distMat;
    }
    // handle the remaining non-mathced detections
    for(i=0; i<(int)rem_detections.size();++i)
    {
        // if the detection i is not in association vector add it
        if(std::find(associations.begin(),associations.end(),i)==associations.end())
            associations.push_back((int)i);
    }
    //allocate proposals
    for(i=0;i<(int)associations.size();++i)
    {
        cv::Rect rt(0,0,0,0);
        if(associations[i]>=0)
        {
            rt = rem_detections[associations[i]];
        }
        proposals.push_back(rt);
    }
    if (!proposals.empty())
    {
        i=0;
        for(auto it = prev_deleted_targets.cbegin();it!= prev_deleted_targets.cend();)
        {
            cv::Rect rt = proposals[i];
            if(!((rt.height==0) | (rt.width ==0) | (rt.x==0) | (rt.y==0)))
            {
                assert(it->second->getStatus() == TargetTerminated);
                it->second->setDetectionCounter();
                it->second->setdetBB(rt);
                it->second->setdetStatus(true);
                it->second->setLastDetTime(Framecount_);
                it->second->setStatus(TargetTracking);
                // add this to tracking set
                tracklet_tracking_.insert(std::make_pair(it->second->getID(),it->second));
                prev_deleted_targets.erase(it++);
            }
            else
            {
                ++it;
            }
            i+=1;
        }
        // now the remaing detection are going to be used to initilize new tracks
        new_track_propsals_.clear();
        new_track_propsals_.insert(new_track_propsals_.begin(),proposals.begin()+ prev_deleted_targets.size(),proposals.end());
    }
}

void Manager::MultiTargetTracking()
{
    Tracklet targets_totrack;
    // get the traget from the previous frame
    try
    {
        targets_totrack = tracks_.at(Framecount_-1);
    }
    catch (const std::out_of_range& oor)
    {
        printerror("Nothing to track from the previous frame: ");
    }

    // Initilize new Targets
    int num_proposal = new_track_propsals_.size();
    for(int i=0;i<num_proposal;++i)
    {
        // check if it is a clean target
        double gap = minDistance(targets_totrack,new_track_propsals_[i]);
        if(gap>=m_config_->searchRadius)
        {
            printinfo("New Target:");
            printinfo("Gap:"+std::to_string(gap));
            std::cout<<new_track_propsals_[i]<<std::endl;
            FloatRect initBB = getFloatRect(new_track_propsals_[i]);
            InitilizeTrack(initBB, new_track_propsals_Id_[i]);
        }
    }

    // add newly initilize track to the tracking set
    for(std::map<int,TargetPtr>::iterator it = tracklet_new_.begin();it!=tracklet_new_.end(); ++it)
        tracklet_tracking_.insert(std::make_pair(it->second->getID(),it->second));

    // main part of tracking
    std::map<int,TargetPtr>::iterator it;
    for(it = tracklet_tracking_.begin(); it!=tracklet_tracking_.end(); ++it)
    {
        it->second->Track();
        it->second->updateTrackletBB();
        it->second->setLastTrackingTime(Framecount_);
    }
}

void Manager::RefreshTargetTracking()
{
    tracks_.insert(std::make_pair(Framecount_,tracklet_tracking_));
    tracklet_new_.clear();
    new_track_propsals_.clear();
    InteractionPotentail();
}

void Manager::FilterTargetsTracking()
{
    std::map<int,std::string> termination_reason;

    /** filter out targets outside of the image **/
    cv::Rect imrt(0, 0, m_config_->frameWidth, m_config_->frameHeight);
    for(auto it = tracklet_tracking_.cbegin(); it!=tracklet_tracking_.cend();)
    {
        cv::Rect trt = it->second->getBB_cv();
        trt.height /= 2;
        trt.width /=2;
        cv::Rect irt = trt & imrt; //rectangle intersection
        float visible = (float)(irt.width * irt.height) / (trt.width * trt.height);
        if(visible < 0.1)
        {
            termination_reason.insert(std::make_pair(it->second->getID(),"Out of image region, not visible anymore"));
            tracklet_deleted_.insert(std::make_pair(it->second->getID(),it->second));
            tracklet_tracking_.erase(it++);
        }
        else
        {
            ++it;
        }
    }

    /** Filter from low confidence targets  **/
    for(auto it = tracklet_tracking_.cbegin(); it!=tracklet_tracking_.cend();)
    {
        int num_frame_passed_wno_det = Framecount_ -  it->second->getLastDetTime();
        int num_frame_tracked = it->second->getNumberofFrameTracked();
        double conf_det = gaussian_prob(num_frame_passed_wno_det, 0.0, std::sqrt(m_config_->frameTolerance));
        conf_det /=(conf_det + 1.0e-6);
        it->second->sethisConfidence(conf_det);
        //printotherBggreen("conf:"+ std::to_string(conf_det));
        if(conf_det<0.100 && num_frame_tracked >m_config_->frameTolerance)
        {
            termination_reason.insert(std::make_pair(it->second->getID(),"Detection Confidence very Low"));
            tracklet_deleted_.insert(std::make_pair(it->second->getID(),it->second));
            tracklet_tracking_.erase(it++);
        }
        else
        {
            ++it;
        }
    }

    /** filter too small targets  **/
    for(auto it = tracklet_tracking_.cbegin(); it!=tracklet_tracking_.cend();)
    {
        FloatRect r1 = it->second->getBB();
        if(r1.Width()<12)
        {
            termination_reason.insert(std::make_pair(it->second->getID(),"To small size"));
            tracklet_deleted_.insert(std::make_pair(it->second->getID(),it->second));
            tracklet_tracking_.erase(it++);
        }
        else
        {
            ++it;
        }

    }

    for(auto it=termination_reason.begin();it!=termination_reason.end();++it)
    {
        printwarning("T[" + std::to_string(it->first) + "] removed because of:" + it->second);
    }

    numActiveTargets_ = tracklet_tracking_.size();

}

void Manager::InitilizeTrack(FloatRect initBB,int targetID)
{
    int newtrgID;
    if (targetID ==-1)
    {
        newtrgID = MAXTARGET - MAXID_;
        MAXID_-=1;
    }
    else
        newtrgID =  targetID;
    TargetPtr newTrg = setNewTarget(newtrgID);
    newTrg->setBB(initBB);
    newTrg->UpdateLearner(image_rep_);
    newTrg->setdetBB(getCVRect(initBB));
    newTrg->setdetStatus(true);
    tracklet_new_.insert(std::make_pair(newtrgID,newTrg));
}

void Manager::getTargetsCVRectBB_and_ID(std::vector<cv::Rect> &bb, std::vector<int> &id)
{
    bb.clear();
    id.clear();
    for(auto it=tracklet_tracking_.begin(); it!=tracklet_tracking_.end(); ++it)
    {
        id.push_back(it->first);
        bb.push_back(it->second->getBB_cv());
    }
}

Tracklet Manager::getOthers(int id,int frame)
{
    Tracklet ret,tmp;
    if (frame==-1)
        frame= Framecount_-1;
    try
    {
        tmp = tracks_.at(frame);
        ret.insert(tmp.begin(), tmp.end());
    }
    catch(const std::out_of_range &oor)
    {
        //nothing to do here
    }
    // remove the id
    ret.erase(id);
    return ret;
}

std::vector<double> Manager::computeDistance(TargetPtr trackingTarget,std::vector<cv::Rect> dets)
{
    std::vector<FloatRect> keptRects;
    std::vector<double> scores;
    for(int i=0; i<(int)dets.size();++i)
    {
        FloatRect rt(dets[i].x,dets[i].y,dets[i].width,dets[i].height);
        keptRects.push_back(rt);
    }

    MultiSample sample(image_rep_, keptRects);
    trackingTarget->Evaluate(sample, scores);
    for(int i=0; i<(int)dets.size();++i)
    {
        if(scores[i]<=0.2)
            scores[i] = 10e5;
        else
            scores[i] = -std::log(scores[i]);

    }
    return scores;
}

std::vector<double> Manager::computeDistanceOV(TargetPtr trackingTarget,std::vector<cv::Rect> dets)
{
    FloatRect rect = trackingTarget->getBB();
    std::vector<FloatRect> keptRects;
    std::vector<double> scoresAPP;
    std::vector<double> scoresLOC;
    std::vector<double> scores;
    for(int i=0; i<(int)dets.size();++i)
    {
        FloatRect rt = getFloatRect(dets[i]);
        scoresLOC.push_back(rect.Overlap(rt));
        keptRects.push_back(rt);
    }

    MultiSample sample(image_rep_, keptRects);
    trackingTarget->Evaluate(sample, scoresAPP);
    for(int i=0; i<(int)dets.size();++i)
    {
        double tmp = 1.0e2;
        if(scoresLOC[i]>=0.2) // if overlapped
        {
            if(scoresAPP[i]>=0.2) // if they are also similary
            {
                tmp = 1e-6;
            }
        }
        scores.push_back(tmp);
    }
    return scores;
}

void Manager::InteractionPotentail()
{
    for(auto it=tracklet_tracking_.begin();it!=tracklet_tracking_.end();++it)
        it->second->clearInteractionsMap();
    for(auto it=tracklet_tracking_.begin(); it!=tracklet_tracking_.end(); ++it)
    {
        FloatRect rt1 = it->second->getBB();
        for(auto it2=std::next(it,1); it2!=tracklet_tracking_.end(); ++it2)
        {
            FloatRect rt2 = it2->second->getBB();
            std::vector<float> tmp = {1.0f-(float)rt1.Overlap(rt2)};//bb_dist(rt1,rt2);
            it->second->addInteraction(it2->second->getID(),tmp[0]);
            it2->second->addInteraction(it->second->getID(),tmp[0]);
        }
    }
}

double Manager::minDistance(std::map<int,TargetPtr> trackingTargets,cv::Rect det)
{
    double ret = 10.0e6;
    double cx = det.x - 0.5*det.width;
    double gap;
    double zdist;

    for(std::map<int,TargetPtr>::iterator it = trackingTargets.begin(); it!=trackingTargets.end(); ++it)
    {
        zdist = 0.5*det.width;
        cv::Rect tmp = getCVRect(it->second->getBB());
        double ctx = tmp.x - 0.5*tmp.width;
        double dist = std::abs(cx-ctx);
        zdist  = 0.5*det.width + 0.5*tmp.width;
        gap = dist - zdist; // gap in x axis
        if(gap<ret)
            ret = gap;
    }
    return ret;
}

void Manager::printManagerSummary()
{
    printother("------------------Manager-------------------");
    printinfo(getTimeStamp());
    printinfo("Frame: "+ std::to_string(Framecount_));
    printinfo("Number of Target: " + std::to_string(tracklet_tracking_.size()));
    printinfo("Number of Tracking Target: " + std::to_string(numActiveTargets_));
    printinfo("Each Target Info: ");
    for(auto it = tracklet_tracking_.begin();it!=tracklet_tracking_.end(); ++it)
    {
        printwarning("Target["+std::to_string(it->first)+"]");
        it->second->printTargetSummary();
    }
    printinfo("Number of Deleted Targets:" + std::to_string(tracklet_deleted_.size()));
#ifdef DEBUG
    printinfo("New Target Proposals Info: ");
    for(int i=0; i<(int)proposals_.size();++i)
    {
        printwarning("\tproposal[");
        std::cout<<proposals_[i]<<std::endl;
    }
#endif
    printother("------------------*******-------------------");
}
