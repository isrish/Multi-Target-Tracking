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

#ifndef RECT_H
#define RECT_H

#include <iostream>
#include <algorithm>

template <typename T>
class STRect
{
public:
	STRect() :
		m_xMin(0),
		m_yMin(0),
		m_width(0),
		m_height(0)
	{
	}
	
	STRect(T xMin, T yMin, T width, T height) :
		m_xMin(xMin),
		m_yMin(yMin),
		m_width(width),
		m_height(height)
	{
	}
	
	template <typename T2>
	STRect(const STRect<T2>& rOther) :
		m_xMin((T)rOther.XMin()),
		m_yMin((T)rOther.YMin()),
		m_width((T)rOther.Width()),
		m_height((T)rOther.Height())
	{
	}

	inline void Set(T xMin, T yMin, T width, T height)
	{
		m_xMin = xMin;
		m_yMin = yMin;
		m_width = width;
		m_height = height;
	}
	
	inline T XMin() const { return m_xMin; }
	inline void SetXMin(T val) { m_xMin = val; }
	inline T YMin() const { return m_yMin; }
	inline void SetYMin(T val) { m_yMin = val; }
	inline T Width() const { return m_width; }
	inline void SetWidth(T val) { m_width = val; }
	inline T Height() const { return m_height; }
	inline void SetHeight(T val) { m_height = val; }
	
	inline void Translate(T x, T y) { m_xMin += x; m_yMin += y; }

	inline T XMax() const { return m_xMin + m_width; }
	inline T YMax() const { return m_yMin + m_height; }
	inline float XCentre() const { return (float)m_xMin + (float)m_width / 2; }
	inline float YCentre() const { return (float)m_yMin + (float)m_height / 2; }
	inline T Area() const { return m_width * m_height; }
	
	template <typename T2>
	friend std::ostream& operator <<(std::ostream &rOS, const STRect<T2>& rRect);
	
	template <typename T2>
	float Overlap(const STRect<T2>& rOther) const;
	
	template <typename T2>
	bool IsInside(const STRect<T2>& rOther) const;

private:
	T m_xMin;
	T m_yMin;
	T m_width;
	T m_height;
};

typedef STRect<int> IntRect;
typedef STRect<float> FloatRect;

template <typename T>
std::ostream& operator <<(std::ostream &rOS, const STRect<T>& rRect)
{
	rOS << "[origin: (" << rRect.m_xMin << ", " << rRect.m_yMin << ") size: (" << rRect.m_width << ", " << rRect.m_height << ")]";
	return rOS;
}

template <typename T>
template <typename T2>
float STRect<T>::Overlap(const STRect<T2>& rOther) const
{
	float x0 = std::max((float)XMin(), (float)rOther.XMin());
	float x1 = std::min((float)XMax(), (float)rOther.XMax());
	float y0 = std::max((float)YMin(), (float)rOther.YMin());
	float y1 = std::min((float)YMax(), (float)rOther.YMax());
	
	if (x0 >= x1 || y0 >= y1) return 0.f;
	
	float areaInt = (x1-x0)*(y1-y0);
	return areaInt/((float)Area()+(float)rOther.Area()-areaInt);
}

template <typename T>
template <typename T2>
bool STRect<T>::IsInside(const STRect<T2>& rOther) const
{
	return (XMin()>=rOther.XMin()) && (YMin()>=rOther.YMin()) && (XMax()<=rOther.XMax()) && (YMax()<=rOther.YMax());
}

#endif
