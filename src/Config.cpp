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
#include "Config.h"

#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

Config::Config(const std::string& path)
{
  SetDefaults();

  ifstream f(path.c_str());
  if (!f)
    {
      cout << "error: could not load config file: " << path << endl;
      return;
    }

  string line, name, tmp;
  while (getline(f, line))
    {
      istringstream iss(line);
      iss >> name >> tmp;

      // skip invalid lines and comments
      if (iss.fail() || tmp != "=" || name[0] == '#') continue;

      if      (name == "seed") iss >> seed;
      else if (name == "quietMode") iss >> quietMode;
      else if (name == "saveImage") iss >> saveImage;
      else if (name == "debugMode") iss >> debugMode;
      else if (name == "sequenceName") iss >> sequenceName;
      else if (name == "sequenceFileName") iss >> sequenceFileName;
      else if (name == "detectionFileName") iss >> detectionFileName;
      else if (name == "outputFileName") iss >> outputFileName;
      else if (name == "outputFilePath") iss >> outputFilePath;
      else if (name == "frameStart") iss >> frameStart;
      else if (name == "frameEnd") iss >> frameEnd;
      else if (name == "frameWidth") iss >> frameWidth;
      else if (name == "frameHeight") iss >> frameHeight;
      else if (name == "seed") iss >> seed;
      else if (name == "searchRadius") iss >> searchRadius;
      else if (name == "svmC") iss >> svmC;
      else if (name == "svmBudgetSize") iss >> svmBudgetSize;
      else if (name == "frameTolerance") iss>> frameTolerance;
      else if (name == "feature")
        {
          string featureName, kernelName;
          double param;
          iss >> featureName >> kernelName >> param;

          FeatureKernelPair fkp;

          if      (featureName == FeatureName(kFeatureTypeHaar)) fkp.feature = kFeatureTypeHaar;
          else if (featureName == FeatureName(kFeatureTypeRaw)) fkp.feature = kFeatureTypeRaw;
          else if (featureName == FeatureName(kFeatureTypeHistogram)) fkp.feature = kFeatureTypeHistogram;
          else
            {
              cout << "error: unrecognised feature: " << featureName << endl;
              continue;
            }

          if      (kernelName == KernelName(kKernelTypeLinear)) fkp.kernel = kKernelTypeLinear;
          else if (kernelName == KernelName(kKernelTypeIntersection)) fkp.kernel = kKernelTypeIntersection;
          else if (kernelName == KernelName(kKernelTypeChi2)) fkp.kernel = kKernelTypeChi2;
          else if (kernelName == KernelName(kKernelTypeGaussian))
            {
              if (iss.fail())
                {
                  cout << "error: gaussian kernel requires a parameter (sigma)" << endl;
                  continue;
                }
              fkp.kernel = kKernelTypeGaussian;
              fkp.params.push_back(param);
            }
          else
            {
              cout << "error: unrecognised kernel: " << kernelName << endl;
              continue;
            }

          features.push_back(fkp);
        }
    }
}

void Config::SetDefaults()
{

  quietMode = false;
  debugMode = false;
  saveImage = false;

  sequenceName = "";
  sequenceFileName = "";
  detectionFileName = "";
  outputFilePath = "";
  outputFileName = "";

  frameWidth = 640;
  frameHeight = 480;
  frameStart = 0;
  frameEnd = 0;

  seed = 0;
  searchRadius = 50;
  svmC = 1.0;
  svmBudgetSize = 100;
  frameTolerance = 60;
  features.clear();
}

std::string Config::FeatureName(FeatureType f)
{
  switch (f)
    {
    case kFeatureTypeRaw:
      return "raw";
    case kFeatureTypeHaar:
      return "haar";
    case kFeatureTypeHistogram:
      return "histogram";
    default:
      return "";
    }
}

std::string Config::KernelName(KernelType k)
{
  switch (k)
    {
    case kKernelTypeLinear:
      return "linear";
    case kKernelTypeGaussian:
      return "gaussian";
    case kKernelTypeIntersection:
      return "intersection";
    case kKernelTypeChi2:
      return "chi2";
    default:
      return "";
    }
}

ostream& operator<< (ostream& out, const Config& conf)
{
  out << "config:" << endl;
  out << "  quietMode          = " << conf.quietMode << endl;
  out << "  debugMode          = " << conf.debugMode << endl;
  out << "  sequenceName       = " << conf.sequenceName << endl;
  out << "  sequenceFileName   = " << conf.sequenceFileName << endl;
  out << "  detectionFileName  = " << conf.detectionFileName << endl;
  out << "  resultsPath        = " << conf.outputFileName << endl;
  out << "  frameWidth         = " << conf.frameWidth << endl;
  out << "  frameHeight        = " << conf.frameHeight << endl;
  out << "  frameStart         = " << conf.frameStart << endl;
  out << "  frameEnd           = " << conf.frameEnd << endl;
  out << "  seed               = " << conf.seed << endl;
  out << "  searchRadius       = " << conf.searchRadius << endl;
  out << "  svmC               = " << conf.svmC << endl;
  out << "  svmBudgetSize      = " << conf.svmBudgetSize << endl;
  out << " frameTolerance      = " << conf.frameTolerance<< endl;

  for (int i = 0; i < (int)conf.features.size(); ++i)
    {
      out << "  feature " << i << endl;
      out << "    feature: " << Config::FeatureName(conf.features[i].feature) << endl;
      out << "    kernel:  " << Config::KernelName(conf.features[i].kernel) <<endl;
      if (conf.features[i].params.size() > 0)
        {
          out << "    params: ";
          for (int j = 0; j < (int)conf.features[i].params.size(); ++j)
            {
              out << " " << conf.features[i].params[j];
            }
          out << endl;
        }
    }

  return out;
}
