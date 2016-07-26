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
#ifndef CONFIG_H
#define CONFIG_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <vector>
#include <string>
#include <ostream>

#define VERBOSE (0)

class Config
{
public:
  Config() { SetDefaults(); }
  Config(const std::string& path);

  enum FeatureType
  {
    kFeatureTypeHaar,
    kFeatureTypeRaw,
    kFeatureTypeHistogram
  };

  enum KernelType
  {
    kKernelTypeLinear,
    kKernelTypeGaussian,
    kKernelTypeIntersection,
    kKernelTypeChi2
  };

  struct FeatureKernelPair
  {
    FeatureType feature;
    KernelType kernel;
    std::vector<double> params;
  };

  bool	quietMode;
  bool debugMode;
  bool saveImage;

  std::string sequenceName;
  std::string sequenceFileName;
  std::string detectionFileName;
  std::string outputFileName;
  std::string outputFilePath;

  int frameStart;
  int frameEnd;
  int frameWidth;
  int frameHeight;

  int seed;
  int searchRadius;
  double svmC;
  int svmBudgetSize;
  std::vector<FeatureKernelPair> features;
  int frameTolerance;

  friend std::ostream& operator<< (std::ostream& out, const Config& conf);

private:
  void SetDefaults();
  static std::string FeatureName(FeatureType f);
  static std::string KernelName(KernelType k);
};

typedef boost::shared_ptr<Config> ConfigPtr;

#endif
