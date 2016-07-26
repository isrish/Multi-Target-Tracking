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

#include "ImageRep.h"

#include <cassert>

#include <opencv/highgui.h>

using namespace std;
using namespace cv;

static const int kNumBins = 16;

ImageRep::ImageRep(const Mat& image, bool computeIntegral, bool computeIntegralHist, bool colour) :
	m_channels(colour ? 3 : 1),
	m_rect(0, 0, image.cols, image.rows)
{	
	for (int i = 0; i < m_channels; ++i)
	{
		m_images.push_back(Mat(image.rows, image.cols, CV_8UC1));
		if (computeIntegral) m_integralImages.push_back(Mat(image.rows+1, image.cols+1, CV_32SC1));
		if (computeIntegralHist)
		{
			for (int j = 0; j < kNumBins; ++j)
			{
				m_integralHistImages.push_back(Mat(image.rows+1, image.cols+1, CV_32SC1));
			}
		}
	}
		
	if (colour)
	{
		assert(image.channels() == 3);
		split(image, m_images);
	}
	else
	{
		assert(image.channels() == 1 || image.channels() == 3);
		if (image.channels() == 3)
		{
			cvtColor(image, m_images[0], CV_RGB2GRAY);
		}
		else if (image.channels() == 1)
		{
			image.copyTo(m_images[0]);
		}
	}
	
	if (computeIntegral)
	{
		for (int i = 0; i < m_channels; ++i)
		{
			//equalizeHist(m_images[i], m_images[i]);
			integral(m_images[i], m_integralImages[i]);
		}
	}
	
	if (computeIntegralHist)
	{
		Mat tmp(image.rows, image.cols, CV_8UC1);
		tmp.setTo(0);
		for (int j = 0; j < kNumBins; ++j)
		{
			for (int y = 0; y < image.rows; ++y)
			{
				const uchar* src = m_images[0].ptr(y);
				uchar* dst = tmp.ptr(y);
				for (int x = 0; x < image.cols; ++x)
				{
					int bin = (int)(((float)*src/256)*kNumBins);
					*dst = (bin == j) ? 1 : 0;
					++src;
					++dst;
				}
			}
			
			integral(tmp, m_integralHistImages[j]);			
		}
	}
}

int ImageRep::Sum(const IntRect& rRect, int channel) const
{
	assert(rRect.XMin() >= 0 && rRect.YMin() >= 0 && rRect.XMax() <= m_images[0].cols && rRect.YMax() <= m_images[0].rows);
	return m_integralImages[channel].at<int>(rRect.YMin(), rRect.XMin()) +
			m_integralImages[channel].at<int>(rRect.YMax(), rRect.XMax()) -
			m_integralImages[channel].at<int>(rRect.YMax(), rRect.XMin()) -
			m_integralImages[channel].at<int>(rRect.YMin(), rRect.XMax());
}

void ImageRep::Hist(const IntRect& rRect, Eigen::VectorXd& h) const
{
	assert(rRect.XMin() >= 0 && rRect.YMin() >= 0 && rRect.XMax() <= m_images[0].cols && rRect.YMax() <= m_images[0].rows);
	int norm = rRect.Area();
	for (int i = 0; i < kNumBins; ++i)
	{
		int sum = m_integralHistImages[i].at<int>(rRect.YMin(), rRect.XMin()) +
			m_integralHistImages[i].at<int>(rRect.YMax(), rRect.XMax()) -
			m_integralHistImages[i].at<int>(rRect.YMax(), rRect.XMin()) -
			m_integralHistImages[i].at<int>(rRect.YMin(), rRect.XMax());
		h[i] = (float)sum/norm;
	}
}
