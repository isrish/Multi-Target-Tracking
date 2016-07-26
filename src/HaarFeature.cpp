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

#include "HaarFeature.h"
#include "Sample.h"

#include <cassert>
#include <iostream>

using namespace std;

HaarFeature::HaarFeature(const FloatRect& bb, int type) :
	m_bb(bb)
{
	assert(type < 6);
	
	switch (type)
	{
	case 0:
		{
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin(), bb.Width(), bb.Height()/2));
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin()+bb.Height()/2, bb.Width(), bb.Height()/2));
			m_weights.push_back(1.f);
			m_weights.push_back(-1.f);
			m_factor = 255*1.f/2;
			break;
		}
	case 1:
		{
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin(), bb.Width()/2, bb.Height()));
			m_rects.push_back(FloatRect(bb.XMin()+bb.Width()/2, bb.YMin(), bb.Width()/2, bb.Height()));
			m_weights.push_back(1.f);
			m_weights.push_back(-1.f);
			m_factor = 255*1.f/2;
			break;
		}
	case 2:
		{
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin(), bb.Width()/3, bb.Height()));
			m_rects.push_back(FloatRect(bb.XMin()+bb.Width()/3, bb.YMin(), bb.Width()/3, bb.Height()));
			m_rects.push_back(FloatRect(bb.XMin()+2*bb.Width()/3, bb.YMin(), bb.Width()/3, bb.Height()));
			m_weights.push_back(1.f);
			m_weights.push_back(-2.f);
			m_weights.push_back(1.f);
			m_factor = 255*2.f/3;
			break;
		}
	case 3:
		{
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin(), bb.Width(), bb.Height()/3));
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin()+bb.Height()/3, bb.Width(), bb.Height()/3));
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin()+2*bb.Height()/3, bb.Width(), bb.Height()/3));
			m_weights.push_back(1.f);
			m_weights.push_back(-2.f);
			m_weights.push_back(1.f);
			m_factor = 255*2.f/3;
			break;
		}
	case 4:
		{
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin(), bb.Width()/2, bb.Height()/2));
			m_rects.push_back(FloatRect(bb.XMin()+bb.Width()/2, bb.YMin()+bb.Height()/2, bb.Width()/2, bb.Height()/2));
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin()+bb.Height()/2, bb.Width()/2, bb.Height()/2));
			m_rects.push_back(FloatRect(bb.XMin()+bb.Width()/2, bb.YMin(), bb.Width()/2, bb.Height()/2));
			m_weights.push_back(1.f);
			m_weights.push_back(1.f);
			m_weights.push_back(-1.f);
			m_weights.push_back(-1.f);
			m_factor = 255*1.f/2;
			break;
		}
	case 5:
		{
			m_rects.push_back(FloatRect(bb.XMin(), bb.YMin(), bb.Width(), bb.Height()));
			m_rects.push_back(FloatRect(bb.XMin()+bb.Width()/4, bb.YMin()+bb.Height()/4, bb.Width()/2, bb.Height()/2));
			m_weights.push_back(1.f);
			m_weights.push_back(-4.f);
			m_factor = 255*3.f/4;
			break;
		}				
	}
}

HaarFeature::~HaarFeature()
{
}

float HaarFeature::Eval(const Sample& s) const
{
	const ImageRep& image = s.GetImage();
	const FloatRect& roi = s.GetROI();
	float value = 0.f;
	for (int i = 0; i < (int)m_rects.size(); ++i)
	{
		const FloatRect& r = m_rects[i];
		IntRect sampleRect((int)(roi.XMin()+r.XMin()*roi.Width()+0.5f), (int)(roi.YMin()+r.YMin()*roi.Height()+0.5f),
			(int)(r.Width()*roi.Width()), (int)(r.Height()*roi.Height()));
		value += m_weights[i]*image.Sum(sampleRect);
	}
	return value / (m_factor*roi.Area()*m_bb.Area());
}
