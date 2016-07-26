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

#include "HistogramFeatures.h"
#include "Config.h"
#include "Sample.h"
#include "Rect.h"

#include <iostream>

using namespace Eigen;
using namespace cv;
using namespace std;

static const int kNumBins = 16;
static const int kNumLevels = 4;
static const int kNumCellsX = 3;
static const int kNumCellsY = 3;

HistogramFeatures::HistogramFeatures(const Config& conf)
{
	int nc = 0;
	for (int i = 0; i < kNumLevels; ++i)
	{
		//nc += 1 << 2*i;
		nc += (i+1)*(i+1);
	}
	SetCount(kNumBins*nc);
    //cout << "histogram bins: " << GetCount() << endl;
}

void HistogramFeatures::UpdateFeatureVector(const Sample& s)
{
	IntRect rect = s.GetROI(); // note this truncates to integers
	//cv::Rect roi(rect.XMin(), rect.YMin(), rect.Width(), rect.Height());
	//cv::resize(s.GetImage().GetImage(0)(roi), m_patchImage, m_patchImage.size());
	
	m_featVec.setZero();
	VectorXd hist(kNumBins);
	
	int histind = 0;
	for (int il = 0; il < kNumLevels; ++il)
	{
		int nc = il+1;
		float w = s.GetROI().Width()/nc;
		float h = s.GetROI().Height()/nc;
		FloatRect cell(0.f, 0.f, w, h);
		for (int iy = 0; iy < nc; ++iy)
		{
			cell.SetYMin(s.GetROI().YMin()+iy*h);
			for (int ix = 0; ix < nc; ++ix)
			{
				cell.SetXMin(s.GetROI().XMin()+ix*w);
				s.GetImage().Hist(cell, hist);
				m_featVec.segment(histind*kNumBins, kNumBins) = hist;
				++histind;
			}
		}
	}
	m_featVec /= histind;
}
