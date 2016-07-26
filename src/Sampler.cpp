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

#include "Sampler.h"
#include "Config.h"

#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;

vector<FloatRect> Sampler::RadialSamples(FloatRect centre, int radius, int nr, int nt)
{
  vector<FloatRect> samples;

  FloatRect s(centre);
  float rstep = (float)radius/nr;
  float tstep = 2*(float)M_PI/8.0;
  samples.push_back(centre);

  for (int ir = 1; ir <= nr; ++ir)
    {
      float phase = (ir % 2)*tstep/2;
      for (int it = 0; it < nt; ++it)
        {
          float dx = ir*rstep*cosf(it*tstep+phase);
          float dy = ir*rstep*sinf(it*tstep+phase);
          s.SetXMin(centre.XMin()+dx);
          s.SetYMin(centre.YMin()+dy);
          samples.push_back(s);
        }
    }

  return samples;
}

vector<FloatRect> Sampler::PixelSamples(FloatRect centre, int radius, bool halfSample)
{
  vector<FloatRect> samples;

  IntRect s(centre);
  samples.push_back(s);

  int r2 = radius*radius;
  for (int iy = -radius; iy <= radius; ++iy)
    {
      for (int ix = -radius; ix <= radius; ++ix)
        {
          if (ix*ix+iy*iy > r2) continue;
          if (iy == 0 && ix == 0) continue; // already put this one at the start

          int x = (int)centre.XMin() + ix;
          int y = (int)centre.YMin() + iy;
          if (halfSample && (ix % 2 != 0 || iy % 2 != 0)) continue;

          s.SetXMin(x);
          s.SetYMin(y);
          samples.push_back(s);
        }
    }

  return samples;
}

static std::vector<FloatRect> Sampler::PixelSamples(std::vector<FloatRect> centers, std::vector<double> prob,int radius,bool halfSample)
{
  assert(centers.size()==prob.size());
  vector<FloatRect> samples;

  // make sure prob is normalized
  double sum_of_elems = 0.0;
  for(int it=0; it <prob.size(); ++it)
    sum_of_elems += prob[it];
  for(int it=0; it <prob.size(); ++it)
    {
      prob[it]/=sum_of_elems ; //normalize
      IntRect s(centers[it]);
      samples.push_back(s); // push them rect first
    }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis((double)0.0,(double)1.0);

  int idx = 0;
  int r2 = radius*radius;
  for (int iy = -radius; iy <= radius; ++iy)
    {
      for (int ix = -radius; ix <= radius; ++ix)
        {
          if (ix*ix+iy*iy > r2) continue;
          if (iy == 0 && ix == 0) continue; // already put this one at the start

          double rval = dis(gen);
          if(rval -= prob[0]<0)
            idx = 0;
          else
            idx = 1;
          IntRect s(centers[idx]);
          int x = (int)centers[idx].XMin() + ix;
          int y = (int)centers[idx].YMin() + iy;
          if (halfSample && (ix % 2 != 0 || iy % 2 != 0)) continue;

          s.SetXMin(x);
          s.SetYMin(y);
          samples.push_back(s);
        }
    }

  return samples;

}
