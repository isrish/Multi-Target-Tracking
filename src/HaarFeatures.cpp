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

#include "HaarFeatures.h"
#include "Config.h"

static const int kSystematicFeatureCount = 192;

HaarFeatures::HaarFeatures(const Config& conf)
{
	SetCount(kSystematicFeatureCount);
	GenerateSystematic();
}

void HaarFeatures::GenerateSystematic()
{
	float x[] = {0.2f, 0.4f, 0.6f, 0.8f};
	float y[] = {0.2f, 0.4f, 0.6f, 0.8f};
	float s[] = {0.2f, 0.4f};
	for (int iy = 0; iy < 4; ++iy)
	{
		for (int ix = 0; ix < 4; ++ix)
		{
			for (int is = 0; is < 2; ++is)
			{
				FloatRect r(x[ix]-s[is]/2, y[iy]-s[is]/2, s[is], s[is]);
				for (int it = 0; it < 6; ++it)
				{
					m_features.push_back(HaarFeature(r, it));
				}
			}
		}
	}
}

void HaarFeatures::UpdateFeatureVector(const Sample& s)
{
	for (int i = 0; i < m_featureCount; ++i)
	{
		m_featVec[i] = m_features[i].Eval(s);
	}
}
