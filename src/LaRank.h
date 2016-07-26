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


#ifndef LARANK_H
#define LARANK_H

#include "Rect.h"
#include "Sample.h"

#include <vector>
#include <eigen2/Eigen/Core>

#include <opencv/cv.h>

class Config;
class Features;
class Kernel;

class LaRank
{
public:
  LaRank(const Config& conf, const Features& features, const Kernel& kernel);
  ~LaRank();

  virtual void Eval(const MultiSample& x, std::vector<double>& results);
  virtual void Update(const MultiSample& x, int y);

  virtual void Debug();

private:

  struct SupportPattern
  {
    std::vector<Eigen::VectorXd> x;
    std::vector<FloatRect> yv;
    std::vector<cv::Mat> images;
    int y;
    int refCount;
  };

  struct SupportVector
  {
    SupportPattern* x;
    int y;
    double b;
    double g;
    cv::Mat image;
  };

  const Config& m_config;
  const Features& m_features;
  const Kernel& m_kernel;

  std::vector<SupportPattern*> m_sps;
  std::vector<SupportVector*> m_svs;

  cv::Mat m_debugImage;

  double m_C;
  Eigen::MatrixXd m_K;

  inline double Loss(const FloatRect& y1, const FloatRect& y2) const
  {
    // overlap loss
    return 1.0-y1.Overlap(y2);
    // squared distance loss
    //double dx = y1.XMin()-y2.XMin();
    //double dy = y1.YMin()-y2.YMin();
    //return dx*dx+dy*dy;
  }

  double ComputeDual() const;

  void SMOStep(int ipos, int ineg);
  std::pair<int, double> MinGradient(int ind);
  void ProcessNew(int ind);
  void Reprocess();
  void ProcessOld();
  void Optimize();

  int AddSupportVector(SupportPattern* x, int y, double g);
  void RemoveSupportVector(int ind);
  void RemoveSupportVectors(int ind1, int ind2);
  void SwapSupportVectors(int ind1, int ind2);

  void BudgetMaintenance();
  void BudgetMaintenanceRemove();

  double Evaluate(const Eigen::VectorXd& x, const FloatRect& y) const;
  void UpdateDebugImage();
};

#endif
